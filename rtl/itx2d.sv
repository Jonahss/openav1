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
// v4 (level engine, 2026-10-13): the NLANES microcoded itx1d engines are replaced by one itx_lvl engine that
// executes a spec level per clock on 32 fixed pair units and packs 64/N rows of an N-point transform into its
// 64 lanes. Rows of a group shift into pk_in one per clock (row g ends in slot g from the top), the engine
// runs (DCT4 2 .. DCT64 10 cycles for the whole group), the results shift out of pk_out one row per clock
// (top slot first, so rows store in reverse order). ADST4, identity and WHT rows stream through itx_row
// instead (load one cycle, store the next). NLANES is kept for interface compatibility and ignored.
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

    // ---------------------------------------------------------------- level engine + row unit
    // The pass being executed: N = row length (w for rows, h for columns), G = rows per engine vector.
    logic       col;                    // 0: row pass, 1: column pass
    logic [2:0] n_l;                    // log2 N for this pass
    logic [1:0] cls_l;                  // 0 DCT, 1 ADST, 2 identity
    logic       stream;                 // rows go through itx_row (ADST4 / identity / WHT), not the engine
    logic [6:0] gmax;                   // 64 >> n_l
    logic [6:0] count;                  // vectors in this pass: h (rows) or w (columns)
    logic [6:0] real_cnt;               // rows that get a transform: rows >= 32 are all-zero (columns: all)
    assign n_l      = col ? l_log2h : l_log2w;
    assign cls_l    = col ? col_cls_l : row_cls_l;
    assign stream   = l_lossless || cls_l == 2'd2 || (cls_l == 2'd1 && n_l == 3'd2);
    assign gmax     = stream ? 7'd1 : (7'd64 >> n_l);
    assign count    = col ? w : h;
    assign real_cnt = col ? w : ((h > 7'd32) ? 7'd32 : h);

    logic [1:0] row_cls_l, col_cls_l;   // latched 1D classes
    typedef enum logic [2:0] { S_IDLE, S_GSZ, S_LD, S_GO, S_RUN, S_ST, S_DONE } state_t;
    state_t     state;
    logic [6:0] idx;                    // first row / column of the current group
    logic [4:0] g;                      // rows loaded / stored so far in this group
    logic [4:0] gsz;                    // rows in this group: min(gmax, real_cnt - idx), or the zero rows
    logic       zero_grp;               // row group at idx >= 32: stored as zeros, no transform
    logic [6:0] cur;                    // row / column handled this cycle
    always_comb begin
        if (state == S_ST && !stream && !zero_grp) cur = idx + 7'(gsz) - 7'd1 - 7'(g);   // top slot first
        else cur = idx + 7'(g);
    end

    logic [64*TW-1:0] pk_in, pk_out;    // packed engine input (shift-in from the top) / output (shift-out)
    logic             e_start, e_busy, e_done;
    logic [2:0]       e_prog;
    logic [4:0]       e_r;
    logic [64*TW-1:0] e_out;
    always_comb begin
        e_prog = (cls_l == 2'd0) ? 3'(n_l - 3'd2) : 3'(n_l + 3'd2);     // DCT n -> 0..4, ADST8 -> 5, ADST16 -> 6
        e_r    = col ? col_clamp : row_clamp;
    end
    itx_lvl #(.TW(TW)) u_lvl (.clk, .rst, .start(e_start), .prog(e_prog), .r(e_r), .in_vec(pk_in),
                              .busy(e_busy), .done(e_done), .out_vec(e_out));
    logic [64*TW-1:0] ld_vec_f;         // the loaded row as a flat vector
    logic [1:0]       rk;               // row unit kind: 0 ADST4, 1 identity, 2 WHT
    logic [64*TW-1:0] rw_out;
    logic [64*TW-1:0] st_row;           // streamed row, registered at load, stored the next cycle
    assign rk = l_lossless ? 2'd2 : (cls_l == 2'd2) ? 2'd1 : 2'd0;
    itx_row #(.TW(TW)) u_row (.kind(rk), .n(n_l), .wht_shift(col ? 2'd0 : 2'd2), .in_vec(ld_vec_f), .out_vec(rw_out));

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

    // loaded vector for the current row / column: row cur of the coefficients (valid bits, size, rectangular
    // pre-scale) or column cur of the residuals (rows below h)
    vec_t ld_vec;
    always_comb begin
        for (int k = 0; k < 64; k++) begin
            logic signed [TW-1:0]  c;
            logic signed [TW+12:0] p;
            c = '0; p = '0;
            if (!col) begin
                c = (cur < 7'd32 && cur < h && k < 32 && k < 32'(w) && cv[cur[4:0]][k[4:0]]) ? signed'(rd_vec[k]) : '0;
                p = c * 13'sd2896;
                ld_vec[k] = rect ? TW'((p + (TW+13)'(2048)) >>> 12) : TW'(c);
            end else begin
                ld_vec[k] = (k < 32'(h) && cur < w) ? rd_vec[k] : '0;
            end
            ld_vec_f[k*TW +: TW] = ld_vec[k];
        end
    end
    // read port select: the sequencer while loading, the residual port otherwise
    always_comb begin
        if (state == S_LD) begin rd_col = col; rd_s = cur[5:0]; end
        else begin rd_col = 1'b0; rd_s = res_addr[11:6]; end
    end
    assign res_data = rd_vec[res_addr[5:0]];
    always_comb for (int l = 0; l < 4; l++) res_data4[l*TW +: TW] = rd_vec[{res_addr[5:2], 2'(l)}];

    // the row being stored: the top slot of pk_out (lanes 64-N .. 63), the streamed row, or zeros
    logic [TW-1:0] st_vec [64];
    always_comb begin
        for (int j = 0; j < 64; j++) st_vec[j] = '0;
        if (zero_grp) begin
            ;                                               // rows >= 32: zeros
        end else if (stream) begin
            for (int j = 0; j < 64; j++) st_vec[j] = st_row[j*TW +: TW];
        end else begin
            case (n_l)
                3'd2:    for (int j = 0; j < 4;  j++) st_vec[j] = pk_out[(60 + j)*TW +: TW];
                3'd3:    for (int j = 0; j < 8;  j++) st_vec[j] = pk_out[(56 + j)*TW +: TW];
                3'd4:    for (int j = 0; j < 16; j++) st_vec[j] = pk_out[(48 + j)*TW +: TW];
                3'd5:    for (int j = 0; j < 32; j++) st_vec[j] = pk_out[(32 + j)*TW +: TW];
                default: for (int j = 0; j < 64; j++) st_vec[j] = pk_out[j*TW +: TW];
            endcase
        end
    end

    // write port: the stored row / column (rounded, and clamped after the row pass) or one coefficient while idle.
    // One shared round2v/clamp datapath for both passes: the row store rounds by l_rowshift and clamps to the
    // column range, the column store rounds by 4 (0 when lossless) without clamp.
    always_comb begin
        logic [2:0] st_sh;
        wr_col = col; wr_s = cur[5:0]; wr_en = '0;
        st_sh  = col ? (l_lossless ? 3'd0 : 3'd4) : 3'(l_rowshift);
        for (int k = 0; k < 64; k++) begin
            logic signed [TW-1:0] v;
            v = round2v(signed'(st_vec[k]), st_sh);
            if (!col) v = (v < clip_lo) ? clip_lo : (v > clip_hi) ? clip_hi : v;
            wr_vec[k] = (state == S_ST) ? v : '0;
        end
        case (state)
            S_ST: wr_en = (cur < count) ? '1 : '0;
            S_IDLE: if (coef_we && !busy) begin
                wr_col = 1'b0;
                wr_s = {1'b0, coef_addr[9:5]};
                wr_en[6'(coef_addr[4:0])] = 1'b1;
                wr_vec[6'(coef_addr[4:0])] = coef_data;
            end
            default: ;
        endcase
    end

    // ---------------------------------------------------------------- sequencer
    logic       first;                  // next group is the first of its pass
    logic [6:0] idx_next;
    logic       zero_grp_next;
    logic [4:0] gsz_next;
    assign idx_next      = first ? 7'd0 : idx + 7'(gsz);
    assign zero_grp_next = !col && (idx_next >= 7'd32);
    always_comb begin
        if (zero_grp_next) gsz_next = 5'(((count - idx_next) < gmax) ? (count - idx_next) : gmax);
        else               gsz_next = 5'(((real_cnt - idx_next) < gmax) ? (real_cnt - idx_next) : gmax);
    end

    always_ff @(posedge clk) begin
        done    <= 1'b0;
        e_start <= 1'b0;
        if (rst) begin
            state <= S_IDLE;
            busy  <= 1'b0;
            idx   <= '0;
            g     <= '0;
            gsz   <= 5'd1;
            first <= 1'b0;
            col   <= 1'b0;
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
                    col        <= 1'b0;
                    idx        <= '0;
                    g          <= '0;
                    zero_grp   <= 1'b0;
                    first      <= 1'b1;
                    busy       <= 1'b1;
                    state      <= S_GSZ;
                end
                S_GSZ: begin                       // group size from the latched parameters (one cycle)
                    gsz      <= gsz_next;
                    zero_grp <= zero_grp_next;
                    idx      <= idx_next;
                    first    <= 1'b0;
                    g        <= '0;
                    state    <= zero_grp_next ? S_ST : S_LD;
                end
                // ---- load the rows of the group: shift into pk_in from the top (engine) or through itx_row (stream)
                S_LD: begin
                    st_row <= rw_out;
                    case (n_l)
                        3'd2:    pk_in <= {ld_vec_f[4*TW-1:0],  pk_in[64*TW-1:4*TW]};
                        3'd3:    pk_in <= {ld_vec_f[8*TW-1:0],  pk_in[64*TW-1:8*TW]};
                        3'd4:    pk_in <= {ld_vec_f[16*TW-1:0], pk_in[64*TW-1:16*TW]};
                        3'd5:    pk_in <= {ld_vec_f[32*TW-1:0], pk_in[64*TW-1:32*TW]};
                        default: pk_in <= ld_vec_f;
                    endcase
                    if (g + 5'd1 == gsz) begin
                        g     <= '0;
                        state <= stream ? S_ST : S_GO;
                    end else g <= g + 5'd1;
                end
                S_GO: begin e_start <= 1'b1; state <= S_RUN; end
                S_RUN: if (e_done) begin pk_out <= e_out; state <= S_ST; end
                // ---- store the rows of the group (top slot first; the streamed row; or zeros)
                S_ST: begin
                    case (n_l)
                        3'd2:    pk_out <= {pk_out[60*TW-1:0], {(4*TW){1'b0}}};
                        3'd3:    pk_out <= {pk_out[56*TW-1:0], {(8*TW){1'b0}}};
                        3'd4:    pk_out <= {pk_out[48*TW-1:0], {(16*TW){1'b0}}};
                        3'd5:    pk_out <= {pk_out[32*TW-1:0], {(32*TW){1'b0}}};
                        default: pk_out <= '0;
                    endcase
                    if (g + 5'd1 == gsz) begin
                        g <= '0;
                        if (idx_next >= count) begin
                            if (col) state <= S_DONE;
                            else begin col <= 1'b1; first <= 1'b1; state <= S_GSZ; end
                        end else state <= S_GSZ;
                    end else g <= g + 5'd1;
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
