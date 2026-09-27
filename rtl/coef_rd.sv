// Coefficient reader: spec 5.11.39 coeffs( ) for one transform block, minus what the block level owns.
//
// Phase A (start_a): decodes all_zero with the caller-supplied context (all_zero_ctx needs the above/left
//   level contexts and the residual block size, which live at block level).
// Phase B (start_b, after the caller has read/derived PlaneTxType): eob, levels (with the coefficient
//   context engine), signs, Golomb escapes. Produces Quant (readable through q_addr/q_data until the next
//   start), eob, culLevel and dcCategory for the caller's context-array update.
//
// Storage: lev[1024] 4-bit saturated levels for the context neighbourhood reads (flops: 5 random reads
// per coefficient), quant_mem[1024] 21-bit signed final coefficients (write-once, RAM) with an nz bitmask
// so untouched positions read 0 without a clear pass.
module coef_rd
  import cdf_map_pkg::*;
  import tx_tables_pkg::*;
(
    input  logic              clk,
    input  logic              rst,
    // block configuration (stable from start_a to the end of phase B)
    input  logic [4:0]        cfg_tx,        // txSz
    input  logic              cfg_ptype,     // plane > 0
    // phase A
    input  logic              start_a,
    input  logic [3:0]        az_ctx,        // all_zero context 0..12
    output logic              done_a,
    output logic              all_zero,
    // phase B
    input  logic              start_b,
    input  logic [3:0]        tx_type,       // PlaneTxType
    input  logic [1:0]        dcs_ctx,       // dc_sign context 0..2
    output logic              done_b,
    output logic [10:0]       eob_o,
    output logic [5:0]        cul_level,
    output logic [1:0]        dc_category,
    output logic              nonconformant, // Golomb length overflow (sticky until next start)
    // coefficient read port (registered, 1 cycle)
    input  logic [9:0]        q_addr,
    output logic signed [20:0] q_data,
    // symbol sequencer master port
    output logic              sq_go,
    output logic [CDF_AW-1:0] sq_addr,
    output logic [3:0]        sq_n,
    output logic [1:0]        sq_kind,
    input  logic              sq_done,
    input  logic [3:0]        sq_sym
);
    localparam logic [1:0] CLS_2D = 2'd0, CLS_H = 2'd1, CLS_V = 2'd2;

    // ---------------------------------------------------------------- derived block parameters
    logic [4:0] adj;
    logic [2:0] bwl, hlog, tx_sz_ctx, eob_ms;
    logic [1:0] cls;
    logic [10:0] area;                       // height << bwl (adjusted)
    always_comb begin
        adj  = tx_adj(cfg_tx);
        bwl  = tx_w_log2(adj);
        hlog = tx_h_log2(adj);
        tx_sz_ctx = 3'((4'(tx_sqr(cfg_tx)) + 4'(tx_sqr_up(cfg_tx)) + 4'd1) >> 1);
        eob_ms = 3'(4'(tx_w_log2(cfg_tx) > 3'd5 ? 3'd5 : tx_w_log2(cfg_tx)) +
                    4'(tx_h_log2(cfg_tx) > 3'd5 ? 3'd5 : tx_h_log2(cfg_tx)) - 4'd4);
        cls = (tx_type == 4'd10 || tx_type == 4'd12 || tx_type == 4'd14) ? CLS_V :
              (tx_type == 4'd11 || tx_type == 4'd13 || tx_type == 4'd15) ? CLS_H : CLS_2D;
        area = 11'd1 << (4'(bwl) + 4'(hlog));
    end

    // ---------------------------------------------------------------- state
    typedef enum logic [4:0] {
        S_IDLE, A_W, B_EOBPT, B_EOBPT_W, B_EXTRA_W, B_XBIT, B_XBIT_W,
        L_SCAN, L_POS, L_POS2, L_CTX, L_BASE_W, L_BR, L_BR_W, L_STORE,
        P_SCAN, P_POS, P_POS2, P_CHK, P_SIGN_W, P_GLEN, P_GLEN_W, P_GDATA, P_GDATA_W, P_STORE, S_DONE
    } st_t;
    st_t st;

    logic [10:0] eob, c;                     // eob 0..1024; c: coefficient index
    logic [3:0]  eob_pt;
    logic [3:0]  xbit_i;                     // eob_extra_bit loop index
    logic [9:0]  pos;
    logic [4:0]  row, col;                   // pos >> bwl, pos & (width-1)  (adjusted, < 32)
    logic [3:0]  level;
    logic [5:0]  ctx_base;                   // 0..41
    logic [4:0]  ctx_br;                     // 0..20
    logic [1:0]  br_i;
    logic        sign;
    logic [5:0]  glen;
    logic [31:0] gx;
    logic [5:0]  gi;
    logic [5:0]  cul;
    logic [1:0]  dccat;

    // ---------------------------------------------------------------- level scratch and outputs
    logic [3:0]  lev [0:1023];
    logic [1023:0] nz;
    logic signed [20:0] quant_mem [0:1023];

    // scan ROM (default zig-zag scans); Mrow/Mcol are arithmetic
    logic [SCAN_ROM_AW-1:0] scan_addr;
    logic [9:0] scan_data;
    assign scan_addr = scan_base(cfg_tx) + SCAN_ROM_AW'(c);
    scan_rom u_scan (.clk(clk), .addr(scan_addr), .data(scan_data));
    logic [9:0] pos_now;
    always_comb begin
        case (cls)
            CLS_V:   pos_now = c[9:0];                                                  // Mrow: row-major
            CLS_H:   pos_now = ((c[9:0] & ((10'd1 << hlog) - 10'd1)) << bwl) | (c[9:0] >> hlog);   // Mcol
            default: pos_now = scan_data;
        endcase
        if (tx_type == 4'd9) pos_now = scan_data;                                       // IDTX: default scan
    end

    // ---------------------------------------------------------------- context engine (combinational on lev)
    // Sig_Ref_Diff_Offset[cls][0..4] and Mag_Ref_Offset_With_Tx_Class[cls][0..2] (the first 3 sig offsets
    // are the mag offsets for every class).
    logic [2:0] d_r [5];
    logic [2:0] d_c [5];
    always_comb begin
        d_r[0] = 3'd0; d_c[0] = 3'd1;
        d_r[1] = 3'd1; d_c[1] = 3'd0;
        case (cls)
            CLS_H:   begin d_r[2] = 3'd0; d_c[2] = 3'd2; d_r[3] = 3'd0; d_c[3] = 3'd3; d_r[4] = 3'd0; d_c[4] = 3'd4; end
            CLS_V:   begin d_r[2] = 3'd2; d_c[2] = 3'd0; d_r[3] = 3'd3; d_c[3] = 3'd0; d_r[4] = 3'd4; d_c[4] = 3'd0; end
            default: begin d_r[2] = 3'd1; d_c[2] = 3'd1; d_r[3] = 3'd0; d_c[3] = 3'd2; d_r[4] = 3'd2; d_c[4] = 3'd0; end
        endcase
    end
    logic [5:0] nb_r [5];
    logic [5:0] nb_c [5];
    logic       nb_ok [5];
    logic [3:0] nb_v [5];
    logic [4:0] mag3;                        // sum of min(lev,3) over 5 (<= 15)
    logic [6:0] mag15;                       // sum of min(lev,15) over 3 (<= 45)
    logic [2:0] ctx_m;
    logic [2:0] ctx_b;
    logic [5:0] ctx_base_now;
    logic [4:0] ctx_br_now;
    logic [2:0] pos_idx;
    always_comb begin
        mag3 = '0; mag15 = '0;
        for (int i = 0; i < 5; i++) begin
            nb_r[i]  = 6'(row) + 6'(d_r[i]);
            nb_c[i]  = 6'(col) + 6'(d_c[i]);
            nb_ok[i] = (nb_r[i] < (6'd1 << hlog)) && (nb_c[i] < (6'd1 << bwl));
            nb_v[i]  = nb_ok[i] ? lev[(10'(nb_r[i][4:0]) << bwl) | 10'(nb_c[i][4:0])] : 4'd0;
            mag3 += 5'(nb_v[i] > 4'd3 ? 4'd3 : nb_v[i]);
            if (i < 3) mag15 += 7'(nb_v[i]);
        end
        ctx_m = 3'((5'(mag3 + 5'd1) >> 1) > 5'd4 ? 5'd4 : (5'(mag3 + 5'd1) >> 1));
        pos_idx = 3'd0;
        if (cls == CLS_2D) begin
            if (row == 5'd0 && col == 5'd0) ctx_base_now = 6'd0;
            else ctx_base_now = 6'(ctx_m) + coeff_base_ctx_offset(cfg_tx, row > 5'd4 ? 3'd4 : row[2:0], col > 5'd4 ? 3'd4 : col[2:0]);
        end else begin
            pos_idx = (cls == CLS_V) ? (row > 5'd2 ? 3'd2 : row[2:0]) : (col > 5'd2 ? 3'd2 : col[2:0]);
            ctx_base_now = 6'(ctx_m) + (pos_idx == 3'd0 ? 6'd26 : pos_idx == 3'd1 ? 6'd31 : 6'd36);
        end
    end
    always_comb begin
        ctx_b = 3'((7'(mag15 + 7'd1) >> 1) > 7'd6 ? 7'd6 : (7'(mag15 + 7'd1) >> 1));
        if (pos == 10'd0) ctx_br_now = 5'(ctx_b);
        else if (cls == CLS_2D) ctx_br_now = 5'(ctx_b) + ((row < 5'd2 && col < 5'd2) ? 5'd7 : 5'd14);
        else if (cls == CLS_H) ctx_br_now = 5'(ctx_b) + ((col == 5'd0) ? 5'd7 : 5'd14);
        else ctx_br_now = 5'(ctx_b) + ((row == 5'd0) ? 5'd7 : 5'd14);
    end
    // eob-position context for the first (last-in-scan) coefficient: Coeff_Base_Eob ctx 0..3
    logic [1:0] ctx_eob;
    always_comb begin
        if (c == 11'd0) ctx_eob = 2'd0;
        else if (c <= (area >> 3)) ctx_eob = 2'd1;
        else if (c <= (area >> 2)) ctx_eob = 2'd2;
        else ctx_eob = 2'd3;
    end

    // ---------------------------------------------------------------- CDF addresses
    logic [CDF_AW-1:0] a_txb_skip, a_eob_pt, a_eob_extra, a_base_eob, a_base, a_br, a_dc_sign;
    logic [3:0] n_eob_pt;
    logic [2:0] br_ctx_sz;
    always_comb begin
        a_txb_skip = CDF_AW'(CDF_TXB_SKIP + int'(tx_sz_ctx) * CDF_TXB_SKIP_S0 + int'(az_ctx));
        case (eob_ms)
            3'd0: begin a_eob_pt = CDF_AW'(CDF_EOB_PT_16  + int'(cfg_ptype) * 2 + int'(cls != CLS_2D)); n_eob_pt = 4'(CDF_EOB_PT_16_N - 1); end
            3'd1: begin a_eob_pt = CDF_AW'(CDF_EOB_PT_32  + int'(cfg_ptype) * 2 + int'(cls != CLS_2D)); n_eob_pt = 4'(CDF_EOB_PT_32_N - 1); end
            3'd2: begin a_eob_pt = CDF_AW'(CDF_EOB_PT_64  + int'(cfg_ptype) * 2 + int'(cls != CLS_2D)); n_eob_pt = 4'(CDF_EOB_PT_64_N - 1); end
            3'd3: begin a_eob_pt = CDF_AW'(CDF_EOB_PT_128 + int'(cfg_ptype) * 2 + int'(cls != CLS_2D)); n_eob_pt = 4'(CDF_EOB_PT_128_N - 1); end
            3'd4: begin a_eob_pt = CDF_AW'(CDF_EOB_PT_256 + int'(cfg_ptype) * 2 + int'(cls != CLS_2D)); n_eob_pt = 4'(CDF_EOB_PT_256_N - 1); end
            3'd5: begin a_eob_pt = CDF_AW'(CDF_EOB_PT_512 + int'(cfg_ptype));                          n_eob_pt = 4'(CDF_EOB_PT_512_N - 1); end
            default: begin a_eob_pt = CDF_AW'(CDF_EOB_PT_1024 + int'(cfg_ptype));                     n_eob_pt = 4'(CDF_EOB_PT_1024_N - 1); end
        endcase
        a_eob_extra = CDF_AW'(CDF_EOB_EXTRA + int'(tx_sz_ctx) * CDF_EOB_EXTRA_S0 + int'(cfg_ptype) * CDF_EOB_EXTRA_S1 + int'(eob_pt) - 3);
        a_base_eob  = CDF_AW'(CDF_COEFF_BASE_EOB + int'(tx_sz_ctx) * CDF_COEFF_BASE_EOB_S0 + int'(cfg_ptype) * CDF_COEFF_BASE_EOB_S1 + int'(ctx_eob));
        a_base      = CDF_AW'(CDF_COEFF_BASE + int'(tx_sz_ctx) * CDF_COEFF_BASE_S0 + int'(cfg_ptype) * CDF_COEFF_BASE_S1 + int'(ctx_base_now));
        br_ctx_sz   = tx_sz_ctx > 3'd3 ? 3'd3 : tx_sz_ctx;
        a_br        = CDF_AW'(CDF_COEFF_BR + int'(br_ctx_sz) * CDF_COEFF_BR_S0 + int'(cfg_ptype) * CDF_COEFF_BR_S1 + int'(ctx_br));
        a_dc_sign   = CDF_AW'(CDF_DC_SIGN + int'(cfg_ptype) * CDF_DC_SIGN_S0 + int'(dcs_ctx));
    end

    // ---------------------------------------------------------------- sequencer requests (combinational)
    always_comb begin
        sq_go = 1'b0; sq_addr = '0; sq_n = 4'd1; sq_kind = 2'd0;
        case (st)
            S_IDLE:  if (start_a) begin sq_go = 1'b1; sq_addr = a_txb_skip; sq_n = 4'd1; end
            B_EOBPT: begin sq_go = 1'b1; sq_addr = a_eob_pt; sq_n = n_eob_pt; end
            B_XBIT:  begin sq_go = 1'b1; sq_kind = 2'd2; end
            L_CTX:   begin sq_go = 1'b1; sq_addr = (c == eob - 11'd1) ? a_base_eob : a_base; sq_n = (c == eob - 11'd1) ? 4'd2 : 4'd3; end
            L_BR:    begin sq_go = 1'b1; sq_addr = a_br; sq_n = 4'd3; end
            P_CHK:   if (lev[pos] != 4'd0) begin sq_go = 1'b1; if (c == 11'd0) begin sq_addr = a_dc_sign; sq_n = 4'd1; end else sq_kind = 2'd2; end
            P_GLEN:  begin sq_go = 1'b1; sq_kind = 2'd2; end
            P_GDATA: begin sq_go = 1'b1; sq_kind = 2'd2; end
            default: ;
        endcase
        // eob_extra: issued from B_EOBPT_W on the response (see FSM); modelled as its own request cycle below
        if (st == B_EOBPT_W && sq_done && (4'(sq_sym) + 4'd1) >= 4'd3) begin
            sq_go = 1'b1; sq_kind = 2'd0; sq_n = 4'd1;
            sq_addr = CDF_AW'(CDF_EOB_EXTRA + int'(tx_sz_ctx) * CDF_EOB_EXTRA_S0 + int'(cfg_ptype) * CDF_EOB_EXTRA_S1 + int'(sq_sym) + 1 - 3);
        end
    end

    // ---------------------------------------------------------------- main FSM
    logic [10:0] eob_from_pt;
    always_comb begin
        // eobPt = sym + 1; eob = eobPt < 2 ? eobPt : (1 << (eobPt - 2)) + 1
        eob_from_pt = (4'(sq_sym) + 4'd1) < 4'd2 ? 11'(4'(sq_sym) + 4'd1) : (11'd1 << (4'(sq_sym) + 4'd1 - 4'd2)) + 11'd1;
    end
    logic [19:0] val20;
    always_comb val20 = (lev[pos] == 4'd15) ? 20'(gx + 32'd14) : 20'(lev[pos]);
    logic [6:0] cul_sum;
    always_comb cul_sum = 7'(cul) + 7'(val20 > 20'd63 ? 20'd63 : val20);

    always_ff @(posedge clk) begin
        done_a <= 1'b0; done_b <= 1'b0;
        if (rst) begin
            st <= S_IDLE; nz <= '0; nonconformant <= 1'b0;
        end else begin
            case (st)
                S_IDLE: begin
                    if (start_a) begin
                        st <= A_W;
                        nonconformant <= 1'b0;
                    end else if (start_b) begin
                        st <= B_EOBPT;
                        for (int i = 0; i < 1024; i++) lev[i] <= 4'd0;
                        nz <= '0; cul <= 6'd0; dccat <= 2'd0; eob <= 11'd0;
                    end
                end
                A_W: if (sq_done) begin
                    all_zero <= sq_sym[0]; done_a <= 1'b1; st <= S_IDLE;
                end
                // ---- eob
                B_EOBPT: st <= B_EOBPT_W;
                B_EOBPT_W: if (sq_done) begin
                    eob_pt <= 4'(sq_sym) + 4'd1;
                    eob <= eob_from_pt;
                    if ((4'(sq_sym) + 4'd1) >= 4'd3) st <= B_EXTRA_W;       // eob_extra requested this cycle
                    else st <= L_SCAN;
                end
                B_EXTRA_W: if (sq_done) begin
                    if (sq_sym[0]) eob <= eob + (11'd1 << (eob_pt - 4'd3));
                    xbit_i <= 4'd1;
                    st <= (eob_pt - 4'd2 > 4'd1) ? B_XBIT : L_SCAN;         // bits for i in 1 .. eobPt-3
                end
                B_XBIT: st <= B_XBIT_W;
                B_XBIT_W: if (sq_done) begin
                    // eobShift = (eobPt - 2) - 1 - i
                    if (sq_sym[0]) eob <= eob + (11'd1 << (eob_pt - 4'd3 - xbit_i));
                    xbit_i <= xbit_i + 4'd1;
                    st <= (xbit_i + 4'd1 < eob_pt - 4'd2) ? B_XBIT : L_SCAN;
                end
                // ---- level pass, c = eob-1 .. 0  (eob >= 1 here since all_zero == 0)
                L_SCAN: begin c <= eob - 11'd1; st <= L_POS; end
                L_POS:  st <= L_POS2;                              // scan ROM addressed with c; data valid in L_POS2
                L_POS2: st <= L_CTX;                               // pos/row/col captured at this edge
                L_CTX: begin                                       // neighbourhood contexts on registered pos/row/col
                    ctx_br <= ctx_br_now; st <= L_BASE_W;
                end
                L_BASE_W: if (sq_done) begin
                    level <= (c == eob - 11'd1) ? 4'(sq_sym) + 4'd1 : 4'(sq_sym);
                    br_i <= 2'd0;
                    st <= (((c == eob - 11'd1) ? 4'(sq_sym) + 4'd1 : 4'(sq_sym)) > 4'd2) ? L_BR : L_STORE;
                end
                L_BR: st <= L_BR_W;
                L_BR_W: if (sq_done) begin
                    level <= level + 4'(sq_sym);
                    br_i <= br_i + 2'd1;
                    st <= (sq_sym < 4'd3 || br_i == 2'd3) ? L_STORE : L_BR;
                end
                L_STORE: begin
                    lev[pos] <= level;
                    if (c == 11'd0) begin c <= 11'd0; st <= P_SCAN; end
                    else begin c <= c - 11'd1; st <= L_POS; end
                end
                // ---- sign / escape pass, c = 0 .. eob-1
                P_SCAN: begin c <= 11'd0; st <= P_POS; end
                P_POS:  st <= P_POS2;
                P_POS2: st <= P_CHK;
                P_CHK: begin
                    sign <= 1'b0; gx <= 32'd1; glen <= 6'd0;
                    if (lev[pos] != 4'd0) st <= P_SIGN_W;
                    else st <= P_STORE;
                end
                P_SIGN_W: if (sq_done) begin
                    sign <= sq_sym[0];
                    st <= (lev[pos] == 4'd15) ? P_GLEN : P_STORE;
                end
                P_GLEN: begin glen <= glen + 6'd1; st <= P_GLEN_W; end
                P_GLEN_W: if (sq_done) begin
                    if (sq_sym[0]) begin
                        // length-1 data bits follow (i = length-2 .. 0)
                        if (glen >= 6'd2) begin gi <= glen - 6'd2; st <= P_GDATA; end
                        else st <= P_STORE;
                    end else if (glen >= 6'd32) begin
                        nonconformant <= 1'b1; st <= P_STORE;
                    end else st <= P_GLEN;
                end
                P_GDATA: st <= P_GDATA_W;
                P_GDATA_W: if (sq_done) begin
                    gx <= (gx << 1) | 32'(sq_sym[0]);
                    if (gi == 6'd0) st <= P_STORE;
                    else begin gi <= gi - 6'd1; st <= P_GDATA; end
                end
                P_STORE: begin
                    if (lev[pos] != 4'd0) begin
                        quant_mem[pos] <= sign ? -21'(val20) : 21'(val20);
                        nz[pos] <= 1'b1;
                        cul <= cul_sum > 7'd63 ? 6'd63 : cul_sum[5:0];
                        if (pos == 10'd0 && val20 != 20'd0) dccat <= sign ? 2'd1 : 2'd2;
                    end
                    if (c + 11'd1 == eob) st <= S_DONE;
                    else begin c <= c + 11'd1; st <= P_POS; end
                end
                S_DONE: begin done_b <= 1'b1; st <= S_IDLE; end
                default: st <= S_IDLE;
            endcase
        end
    end

    // pos/row/col: capture the scan result when leaving L_POS2 / P_POS2 (scan_data valid then)
    always_ff @(posedge clk)
        if (st == L_POS2 || st == P_POS2) begin
            pos <= pos_now;
            row <= 5'(pos_now >> bwl);
            col <= 5'(pos_now & ((10'd1 << bwl) - 10'd1));
        end

    assign eob_o = eob;
    assign cul_level = cul;
    assign dc_category = dccat;

    always_ff @(posedge clk)
        q_data <= nz[q_addr] ? quant_mem[q_addr] : 21'sd0;
endmodule
