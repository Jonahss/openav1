// openav1 — 2D inverse transform block (AV1 spec 7.13.3) around the itx1d engine.
//
// Load the dequantized coefficients through the write port (only the top-left 32x32 can be
// non-zero, per spec), pulse start with the transform size / type / bit depth / lossless flag,
// wait for done, then read the h x w Residual through the read port.
//
// Sequence, exactly as the spec orders it:
//   rows i = 0..h-1 : T[j] = Dequant[i][j] (0 for i or j >= 32)
//                     if |log2W - log2H| == 1: T[j] = Round2(T[j] * 2896, 12)
//                     1D row transform (WHT shift 2 if lossless), r = BitDepth + 8
//                     Residual[i][j] = Round2(T[j], rowShift)   then clamp to colClampRange bits
//   cols j = 0..w-1 : T[i] = Residual[i][j]; 1D column transform (WHT shift 0), r = colClampRange
//                     Residual[i][j] = Round2(T[i], colShift)   (colShift = 4, or 0 if lossless)
// Rows i >= 32 are all-zero and every 1D transform maps zero to zero, so they are skipped.
//
// Dequant and Residual share one transpose memory (below): 64 x 64 x TW in 64 banks of distributed RAM, skewed
// so that any whole row or whole column is one access. Flips (FLIPADST) are applied by the reconstruction
// stage, not here.
//
// v2 (throughput): NLANES 1D engines run NLANES rows (then NLANES columns) at once. A group of rows is
// loaded into the lanes one row per cycle, transformed (DCT4 5 cycles .. DCT64 65), and stored one row per
// cycle; the next group follows. Row groups at i >= 32 are all-zero and written without a transform.
// Cost per block ~ ceil(h/NLANES) x (rowprog + 2 NLANES + 1) + ceil(w/NLANES) x (colprog + 2 NLANES + 1) + 2.
//
// v3 (mappable): the transpose memory replaces the two register arrays (coef[32][32], res[64][64]) whose
// random multi-row reads were a 64:1 mux per element and synthesised to several hundred thousand LUTs.
// Element (i, j) lives in bank (i + j) mod 64 at address i, so row i is "all banks, address i" and column j
// is "bank b, address (b - j) mod 64", each followed by a rotate by i or j. Coefficients are written in
// place (one element per cycle while idle) and the row pass overwrites each row with its residual, so
// no second array is needed; a 32x32 valid bitmap stands in for clearing the coefficient array.
module itx2d #(
    parameter int TW = 20,
    parameter int NLANES = 4              // power of two, 1..32 (row groups must not straddle row 32)
) (
    input  logic          clk,
    input  logic          rst,

    // coefficient write port (valid while idle): addr = i*32 + j, i,j < 32; coef_clr zeroes the whole array
    input  logic          coef_clr,
    input  logic          coef_we,
    input  logic [9:0]    coef_addr,
    input  logic [TW-1:0] coef_data,

    // control
    input  logic          start,
    input  logic [4:0]    tx_sz,          // TX_4X4 .. TX_64X16 (spec order)
    input  logic [3:0]    tx_type,        // DCT_DCT .. H_FLIPADST (spec order)
    input  logic [3:0]    bit_depth,      // 8, 10 or 12
    input  logic          lossless,
    output logic          busy,
    output logic          done,

    // residual read port (valid after done): addr = i*64 + j
    input  logic [11:0]   res_addr,
    output logic [TW-1:0] res_data,
    output logic [4*TW-1:0] res_data4       // the aligned group of 4 columns containing res_addr
);
    // ---------------------------------------------------------------- tables (spec)
    logic [2:0] log2w, log2h;
    logic [1:0] row_shift_tab;
    always_comb begin
        case (tx_sz)
            5'd0:  begin log2w = 2; log2h = 2; row_shift_tab = 0; end   // 4x4
            5'd1:  begin log2w = 3; log2h = 3; row_shift_tab = 1; end   // 8x8
            5'd2:  begin log2w = 4; log2h = 4; row_shift_tab = 2; end   // 16x16
            5'd3:  begin log2w = 5; log2h = 5; row_shift_tab = 2; end   // 32x32
            5'd4:  begin log2w = 6; log2h = 6; row_shift_tab = 2; end   // 64x64
            5'd5:  begin log2w = 2; log2h = 3; row_shift_tab = 0; end   // 4x8
            5'd6:  begin log2w = 3; log2h = 2; row_shift_tab = 0; end   // 8x4
            5'd7:  begin log2w = 3; log2h = 4; row_shift_tab = 1; end   // 8x16
            5'd8:  begin log2w = 4; log2h = 3; row_shift_tab = 1; end   // 16x8
            5'd9:  begin log2w = 4; log2h = 5; row_shift_tab = 1; end   // 16x32
            5'd10: begin log2w = 5; log2h = 4; row_shift_tab = 1; end   // 32x16
            5'd11: begin log2w = 5; log2h = 6; row_shift_tab = 1; end   // 32x64
            5'd12: begin log2w = 6; log2h = 5; row_shift_tab = 1; end   // 64x32
            5'd13: begin log2w = 2; log2h = 4; row_shift_tab = 1; end   // 4x16
            5'd14: begin log2w = 4; log2h = 2; row_shift_tab = 1; end   // 16x4
            5'd15: begin log2w = 3; log2h = 5; row_shift_tab = 2; end   // 8x32
            5'd16: begin log2w = 5; log2h = 3; row_shift_tab = 2; end   // 32x8
            5'd17: begin log2w = 4; log2h = 6; row_shift_tab = 2; end   // 16x64
            default: begin log2w = 6; log2h = 4; row_shift_tab = 2; end // 64x16
        endcase
    end

    // 1D class per direction: 0 DCT, 1 ADST, 2 identity  (spec 7.13.3 lists)
    logic [1:0] row_cls, col_cls;
    always_comb begin
        case (tx_type)
            4'd0, 4'd1, 4'd4, 4'd11:                              row_cls = 2'd0; // DCT_DCT ADST_DCT FLIPADST_DCT H_DCT
            4'd2, 4'd3, 4'd5, 4'd6, 4'd7, 4'd8, 4'd13, 4'd15:     row_cls = 2'd1; // *ADST rows
            default:                                              row_cls = 2'd2; // IDTX V_DCT V_ADST V_FLIPADST
        endcase
        case (tx_type)
            4'd0, 4'd2, 4'd5, 4'd10:                              col_cls = 2'd0; // DCT_DCT DCT_ADST DCT_FLIPADST V_DCT
            4'd1, 4'd3, 4'd4, 4'd6, 4'd7, 4'd8, 4'd12, 4'd14:     col_cls = 2'd1; // *ADST cols
            default:                                              col_cls = 2'd2; // IDTX H_DCT H_ADST H_FLIPADST
        endcase
    end

    function automatic logic [3:0] prog_id(input logic [1:0] cls, input logic [2:0] nn, input logic ll);
        if (ll) return 4'd12;
        case (cls)
            2'd0:    return 4'(nn - 3'd2);
            2'd1:    return 4'd5 + 4'(nn - 3'd2);
            default: return 4'd8 + 4'(nn - 3'd2);
        endcase
    endfunction

    // ---------------------------------------------------------------- transpose memory
    // 64 banks x 64 entries x TW. Row/column access: element k of the vector is bank (k + s) mod 64, where s is
    // the row index (row mode: every bank at address s) or the column index (column mode: bank b at address
    // (b - s) mod 64). Distributed RAM: synchronous write, asynchronous read, so the residual read port below
    // stays combinational as before.
    typedef logic [TW-1:0] vec_t [64];
    logic          tm_we [64];
    logic [5:0]    tm_waddr [64], tm_raddr [64];
    logic [TW-1:0] tm_wdata [64], tm_rdata [64];
    genvar gb;
    generate
        for (gb = 0; gb < 64; gb++) begin : g_bank
            (* ram_style = "distributed" *) logic [TW-1:0] mem [0:63];
            always_ff @(posedge clk) if (tm_we[gb]) mem[tm_waddr[gb]] <= tm_wdata[gb];
            assign tm_rdata[gb] = mem[tm_raddr[gb]];
        end
    endgenerate
    // read side: mode/index chosen by the sequencer (loading a group) or the residual port (otherwise)
    logic       rd_col;                 // 0: row s, 1: column s
    logic [5:0] rd_s;
    vec_t       rd_vec;                 // element k (column j = k in row mode, row i = k in column mode)
    // The rotations by s are explicit 6-stage log rotators (2:1 muxes, 2^st words per stage): written as
    // indexed lookups, Vivado built 64 independent 64:1 muxes per bit (~10k LUTs each way, 2026-10-13).
    function automatic vec_t rot_left(input vec_t v, input logic [5:0] s);   // out[k] = in[(k + s) mod 64]
        vec_t t;
        t = v;
        for (int st = 0; st < 6; st++) if (s[st]) begin
            vec_t u;
            for (int k = 0; k < 64; k++) u[k] = t[(k + (1 << st)) % 64];
            t = u;
        end
        return t;
    endfunction
    function automatic vec_t rot_right(input vec_t v, input logic [5:0] s);  // out[b] = in[(b - s) mod 64]
        vec_t t;
        t = v;
        for (int st = 0; st < 6; st++) if (s[st]) begin
            vec_t u;
            for (int k = 0; k < 64; k++) u[k] = t[(k + 64 - (1 << st)) % 64];
            t = u;
        end
        return t;
    endfunction
    always_comb begin
        vec_t rdat;
        for (int b = 0; b < 64; b++) tm_raddr[b] = rd_col ? 6'(b - 32'(rd_s)) : rd_s;
        for (int b = 0; b < 64; b++) rdat[b] = tm_rdata[b];
        rd_vec = rot_left(rdat, rd_s);                                      // rotate left by s
    end
    // write side: a vector (row or column s) with per-element enables, from the store states or the coefficient port
    logic        wr_col;
    logic [5:0]  wr_s;
    logic [63:0] wr_en;
    vec_t        wr_vec;
    always_comb begin
        vec_t wdat;
        wdat = rot_right(wr_vec, wr_s);                                     // element b - s lands in bank b
        for (int b = 0; b < 64; b++) begin
            logic [5:0] k;
            k = 6'(b - 32'(wr_s));
            tm_we[b]    = wr_en[k];
            tm_waddr[b] = wr_col ? k : wr_s;
            tm_wdata[b] = wdat[b];
        end
    end

    // coefficient valid bitmap (the array is never cleared; a row read masks with its valid bits)
    logic [31:0] cv [0:31];
    always_ff @(posedge clk) begin
        if (coef_clr && !busy) for (int i = 0; i < 32; i++) cv[i] <= '0;
        else if (coef_we && !busy) cv[coef_addr[9:5]][coef_addr[4:0]] <= 1'b1;
    end

    // ---------------------------------------------------------------- latched parameters
    logic [3:0] l_bd;
    logic       l_lossless;
    logic [2:0] l_log2w, l_log2h;
    logic [6:0] w, h;
    logic       rect;                  // |log2W - log2H| == 1
    logic [1:0] l_rowshift;
    logic [4:0] row_clamp, col_clamp;
    assign w = 7'd1 << l_log2w;
    assign h = 7'd1 << l_log2h;
    assign row_clamp = 5'(l_bd) + 5'd8;
    assign col_clamp = (5'(l_bd) + 5'd6 > 5'd16) ? 5'(l_bd) + 5'd6 : 5'd16;

    // ---------------------------------------------------------------- engines (one per lane)
    localparam int NL = NLANES;
    logic             go;                  // registered start pulse for the lanes
    logic [3:0]       e_prog;
    logic [2:0]       e_n;
    logic [4:0]       e_r;
    logic [1:0]       e_wht;
    logic [64*TW-1:0] e_in  [NL];          // registered: loaded one lane per cycle from the transpose memory
    logic [64*TW-1:0] e_out [NL];
    logic             e_done [NL];
    logic             e_busy [NL];
    genvar gl;
    generate
        for (gl = 0; gl < NL; gl++) begin : g_lane
            itx1d #(.TW(TW)) u_1d (
                .clk(clk), .rst(rst), .start(go), .prog(e_prog), .n(e_n), .r(e_r), .wht_shift(e_wht),
                .in_vec(e_in[gl]), .busy(e_busy[gl]), .done(e_done[gl]), .out_vec(e_out[gl])
            );
        end
    endgenerate

    typedef enum logic [2:0] { S_IDLE, S_RLD, S_RRUN, S_RST, S_CLD, S_CRUN, S_CST, S_DONE } state_t;
    state_t state;
    logic [6:0] idx;                    // first row i (row pass) or first column j (column pass) of the current group
    logic [$clog2(NL > 1 ? NL : 2)-1:0] lane;   // lane being loaded / stored
    logic       zero_grp;               // row group at i >= 32: all zero, no transform
    logic [1:0] row_cls_l, col_cls_l;
    logic [6:0] cur;                    // row / column handled this cycle
    assign cur = idx + 7'(lane);

    // Round2 helper with a variable shift (0..4)
    function automatic logic signed [TW-1:0] round2v(input logic signed [TW-1:0] x, input logic [2:0] sh);
        logic signed [TW:0] t;
        if (sh == 3'd0) return x;
        t = (TW+1)'(x) + (TW+1)'(1 <<< (sh - 3'd1));
        return TW'(t >>> sh);
    endfunction

    logic signed [TW-1:0] clip_lo, clip_hi;
    assign clip_lo = -(TW'(1) <<< (col_clamp - 5'd1));
    assign clip_hi =  (TW'(1) <<< (col_clamp - 5'd1)) - TW'(1);

    // loaded vector for the current lane: row cur of the coefficients (valid bits, size, rectangular pre-scale)
    // or column cur of the residuals (rows below h)
    vec_t ld_vec;
    always_comb begin
        for (int k = 0; k < 64; k++) begin
            logic signed [TW-1:0]  c;
            logic signed [TW+12:0] p;
            c = '0; p = '0;
            if (state == S_RLD) begin
                c = (cur < 7'd32 && cur < h && k < 32 && k < 32'(w) && cv[cur[4:0]][k[4:0]]) ? signed'(rd_vec[k]) : '0;
                p = c * 13'sd2896;
                ld_vec[k] = rect ? TW'((p + (TW+13)'(2048)) >>> 12) : TW'(c);
            end else begin
                ld_vec[k] = (k < 32'(h) && cur < w) ? rd_vec[k] : '0;
            end
        end
    end
    // read port select: the sequencer while loading, the residual port otherwise
    always_comb begin
        if (state == S_RLD) begin rd_col = 1'b0; rd_s = cur[5:0]; end
        else if (state == S_CLD) begin rd_col = 1'b1; rd_s = cur[5:0]; end
        else begin rd_col = 1'b0; rd_s = res_addr[11:6]; end
    end
    assign res_data = rd_vec[res_addr[5:0]];
    always_comb for (int l = 0; l < 4; l++) res_data4[l*TW +: TW] = rd_vec[{res_addr[5:2], 2'(l)}];

    // write port: stored lane vector (row or column cur) or one coefficient while idle
    always_comb begin
        wr_col = 1'b0; wr_s = cur[5:0]; wr_en = '0;
        for (int k = 0; k < 64; k++) wr_vec[k] = '0;
        case (state)
            S_RST: begin
                wr_en = (cur < h) ? '1 : '0;
                if (!zero_grp)
                    for (int j = 0; j < 64; j++) begin
                        logic signed [TW-1:0] v;
                        v = round2v(e_out[lane][j*TW +: TW], 3'(l_rowshift));
                        v = (v < clip_lo) ? clip_lo : (v > clip_hi) ? clip_hi : v;
                        wr_vec[j] = v;
                    end
            end
            S_CST: begin
                wr_col = 1'b1;
                wr_en = (cur < w) ? '1 : '0;
                for (int i = 0; i < 64; i++) wr_vec[i] = round2v(e_out[lane][i*TW +: TW], l_lossless ? 3'd0 : 3'd4);
            end
            S_IDLE: if (coef_we && !busy) begin
                wr_s = {1'b0, coef_addr[9:5]};
                wr_en[6'(coef_addr[4:0])] = 1'b1;
                wr_vec[6'(coef_addr[4:0])] = coef_data;
            end
            default: ;
        endcase
    end

    always_comb begin
        e_prog = 4'd0; e_n = 3'd2; e_r = 5'd16; e_wht = 2'd0;
        if (state == S_RLD || state == S_RRUN) begin
            e_prog = prog_id(row_cls_l, l_log2w, l_lossless);
            e_n    = l_log2w;
            e_r    = row_clamp;
            e_wht  = 2'd2;
        end else begin
            e_prog = prog_id(col_cls_l, l_log2h, l_lossless);
            e_n    = l_log2h;
            e_r    = col_clamp;
            e_wht  = 2'd0;
        end
    end

    always_ff @(posedge clk) begin
        done <= 1'b0;
        go   <= 1'b0;
        if (rst) begin
            state <= S_IDLE;
            busy  <= 1'b0;
            idx   <= '0;
            lane  <= '0;
        end else begin
            case (state)
                S_IDLE: if (start) begin
                    l_bd       <= bit_depth;
                    l_lossless <= lossless;
                    l_log2w    <= log2w;
                    l_log2h    <= log2h;
                    rect       <= (log2w > log2h) ? (log2w - log2h == 3'd1) : (log2h - log2w == 3'd1);
                    l_rowshift <= lossless ? 2'd0 : row_shift_tab;
                    row_cls_l  <= row_cls;
                    col_cls_l  <= col_cls;
                    idx        <= '0;
                    lane       <= '0;
                    zero_grp   <= 1'b0;
                    busy       <= 1'b1;
                    state      <= S_RLD;
                end
                // ---- rows: load NL rows (one per cycle), transform, store NL rows; groups at i >= 32 are zero
                S_RLD: begin
                    for (int l = 0; l < NL; l++)
                        if (l == 32'(lane)) for (int k = 0; k < 64; k++) e_in[l][k*TW +: TW] <= ld_vec[k];
                    if (32'(lane) == NL - 1) begin
                        lane <= '0;
                        if (idx < 7'd32) begin go <= 1'b1; state <= S_RRUN; end
                        else begin zero_grp <= 1'b1; state <= S_RST; end
                    end else lane <= lane + 1'b1;
                end
                S_RRUN: if (e_done[0]) state <= S_RST;
                S_RST: begin
                    if (32'(lane) == NL - 1) begin
                        lane <= '0; zero_grp <= 1'b0;
                        if (idx + 7'(NL) >= h) begin idx <= '0; state <= S_CLD; end
                        else begin idx <= idx + 7'(NL); state <= S_RLD; end
                    end else lane <= lane + 1'b1;
                end
                // ---- columns: load NL columns, transform, store NL columns
                S_CLD: begin
                    for (int l = 0; l < NL; l++)
                        if (l == 32'(lane)) for (int k = 0; k < 64; k++) e_in[l][k*TW +: TW] <= ld_vec[k];
                    if (32'(lane) == NL - 1) begin lane <= '0; go <= 1'b1; state <= S_CRUN; end
                    else lane <= lane + 1'b1;
                end
                S_CRUN: if (e_done[0]) state <= S_CST;
                S_CST: begin
                    if (32'(lane) == NL - 1) begin
                        lane <= '0;
                        if (idx + 7'(NL) >= w) state <= S_DONE;
                        else begin idx <= idx + 7'(NL); state <= S_CLD; end
                    end else lane <= lane + 1'b1;
                end
                S_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
