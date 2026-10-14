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
// v5 (tm4, 2026-10-13): the transpose memory is 16 banks of 4 lanes instead of 64 banks of one element, so
// the two rotations are 16-word rotations instead of 64-element ones. Rows are still one access each (word k
// of row i is bank (k + i) mod 16 at address i); columns are accessed as 4-column x 16-row tiles (word ro of
// tile (jg, t) is bank (jg + ro) mod 16 at address 16t + ro), so the column pass loads and stores up to four
// columns per clock. A tile's columns scatter into the column-packed engine vector and gather back out of it
// with per-size constant wiring (an engine vector holds at least four columns for N <= 16, so only the 32-
// and 64-point columns select columns within a tile). Rows of a group are placed directly into their slot and
// stored from the bottom slot while the vector shifts down, so rows store in order. A first version with
// 4 banks of 16 lanes spent more on the column windows than the rotators had cost.
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
    // 16 banks x 4 lanes x 64 entries x TW (distributed RAM, synchronous write, asynchronous read). Element
    // (i, j) lives in bank (j/4 + i) mod 16, lane j mod 4, address i. Two access shapes, both one clock:
    //   row i:         word k (= j/4) from bank (k + i) mod 16 at address i          -> rd_vec[k*4 + l] = (i, 4k + l)
    //   tile (jg, t):  word ro from bank (jg + ro) mod 16 at address 16t + ro         -> rd_vec[ro*4 + l] = (16t + ro, 4jg + l)
    // so with s = i mod 16 (rows) or jg (tiles), word k is bank (k + s) mod 16: one 16:1 word rotation each way.
    typedef logic [TW-1:0] vec_t [64];
    logic          tm_we [16][4];
    logic [5:0]    tm_waddr [16], tm_raddr [16];
    logic [TW-1:0] tm_wdata [16][4], tm_rdata [16][4];
    genvar gb, gl;
    generate
        for (gb = 0; gb < 16; gb++) begin : g_bank
            for (gl = 0; gl < 4; gl++) begin : g_lane
                (* ram_style = "distributed" *) logic [TW-1:0] mem [0:63];
                always_ff @(posedge clk) if (tm_we[gb][gl]) mem[tm_waddr[gb]] <= tm_wdata[gb][gl];
                assign tm_rdata[gb][gl] = mem[tm_raddr[gb]];
            end
        end
    endgenerate
    // read side: mode/index chosen by the sequencer (loading) or the residual port (otherwise)
    logic       rd_tile;                // 0: row rd_i, 1: tile (rd_jg, rd_t)
    logic [5:0] rd_i;
    logic [3:0] rd_jg;
    logic [1:0] rd_t;
    vec_t       rd_vec;
    always_comb begin
        logic [3:0] s;
        s = rd_tile ? rd_jg : rd_i[3:0];
        for (int b = 0; b < 16; b++) tm_raddr[b] = rd_tile ? {rd_t, 4'(b - 32'(rd_jg))} : rd_i;
        for (int k = 0; k < 16; k++)
            for (int l = 0; l < 4; l++) rd_vec[k*4 + l] = tm_rdata[4'(k + 32'(s))][l];
    end
    // write side: a row or a tile with per-element enables
    logic        wr_tile;
    logic [5:0]  wr_i;
    logic [3:0]  wr_jg;
    logic [1:0]  wr_t;
    logic [63:0] wr_en;
    vec_t        wr_vec;
    always_comb begin
        logic [3:0] s;
        s = wr_tile ? wr_jg : wr_i[3:0];
        for (int b = 0; b < 16; b++) begin
            logic [3:0] k;
            k = 4'(b - 32'(s));
            tm_waddr[b] = wr_tile ? {wr_t, k} : wr_i;
            for (int l = 0; l < 4; l++) begin
                tm_wdata[b][l] = wr_vec[32'(k)*4 + l];
                tm_we[b][l]    = wr_en[32'(k)*4 + l];
            end
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
    logic [1:0] row_cls_l, col_cls_l;   // latched 1D classes
    logic [4:0] row_clamp, col_clamp;
    assign w = 7'd1 << l_log2w;
    assign h = 7'd1 << l_log2h;
    assign row_clamp = 5'(l_bd) + 5'd8;
    assign col_clamp = (5'(l_bd) + 5'd6 > 5'd16) ? 5'(l_bd) + 5'd6 : 5'd16;

    // ---------------------------------------------------------------- level engine + row unit
    // The pass being executed: N = row length (w for rows, h for columns), G = rows / columns per engine vector.
    // Column pass tiles: ntc column tiles per group (G/4, at least 1) x ntr row tiles per column (N/16, at least 1).
    logic       col;                    // 0: row pass, 1: column pass
    logic [2:0] n_l;                    // log2 N for this pass
    logic [1:0] cls_l;                  // 0 DCT, 1 ADST, 2 identity
    logic       stream;                 // rows go through itx_row (ADST4 / identity / WHT), not the engine
    logic [6:0] gmax;                   // 64 >> n_l
    logic [6:0] count;                  // vectors in this pass: h (rows) or w (columns)
    logic [6:0] real_cnt;               // rows that get a transform: rows >= 32 are all-zero (columns: all)
    logic [2:0] ntr;                    // row tiles per column: max(1, N/16)
    logic [2:0] ntc;                    // column tiles per group: max(1, G/4)
    assign n_l      = col ? l_log2h : l_log2w;
    assign cls_l    = col ? col_cls_l : row_cls_l;
    assign stream   = l_lossless || cls_l == 2'd2 || (cls_l == 2'd1 && n_l == 3'd2);
    assign gmax     = stream ? 7'd1 : (7'd64 >> n_l);
    assign count    = col ? w : h;
    assign real_cnt = col ? w : ((h > 7'd32) ? 7'd32 : h);
    assign ntr      = (n_l > 3'd4) ? 3'(3'd1 << (n_l - 3'd4)) : 3'd1;
    assign ntc      = (n_l < 3'd4) ? 3'(3'd1 << (3'd4 - n_l)) : 3'd1;

    typedef enum logic [2:0] { S_IDLE, S_GSZ, S_LD, S_GO, S_RUN, S_ST, S_STRM, S_DONE } state_t;
    state_t     state;
    logic [6:0] idx;                    // first row / column of the current group
    logic [4:0] g;                      // rows loaded / stored so far in this group (row pass)
    logic [2:0] gc, gt;                 // column tile / row tile being loaded or stored (column pass)
    logic [4:0] gsz;                    // rows / columns in this group: min(gmax, real_cnt - idx), or the zero rows
    logic       zero_grp;               // row group at idx >= 32: stored as zeros, no transform
    logic [6:0] cur;                    // row handled this cycle (row pass)
    assign cur = idx + 7'(g);
    logic [1:0] c0;                     // first column of the group within its 4-column tile (32- and 64-point columns)
    assign c0 = idx[1:0];

    logic [64*TW-1:0] pk_in, pk_out;    // packed engine input / output
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
    // streamed rows / columns (ADST4 / identity / WHT): a 3-stage pipeline, ntr clocks per vector —
    //   1 read row ri (or the ntr row tiles of column ri) -> ld_row   2 ld_row -> itx_row -> st_row   3 st_row -> write
    logic [1:0]       rk;               // row unit kind: 0 ADST4, 1 identity, 2 WHT
    logic [32*TW-1:0] rw_out;
    logic [32*TW-1:0] ld_row, st_row;
    logic             ld_v, st_v, ld_zero;
    logic [6:0]       ri, ld_i, st_i;
    logic [1:0]       tc, tw;           // row-tile counters: reads of column ri, writes of column st_i
    assign rk = l_lossless ? 2'd2 : (cls_l == 2'd2) ? 2'd1 : 2'd0;
    itx_row #(.TW(TW)) u_row (.kind(rk), .n(n_l), .wht_shift(col ? 2'd0 : 2'd2), .in_vec(ld_row), .out_vec(rw_out));

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

    // loaded row (row pass): row ld_idx of the coefficients (valid bits, size, rectangular pre-scale)
    vec_t ld_vec;
    logic [6:0] ld_idx;                 // the row being read: the stream index or the group's cur
    assign ld_idx = (state == S_STRM) ? ri : cur;
    always_comb begin
        for (int k = 0; k < 64; k++) begin
            logic signed [TW-1:0]  c;
            logic signed [TW+12:0] p;
            c = (ld_idx < 7'd32 && ld_idx < h && k < 32 && k < 32'(w) && cv[ld_idx[4:0]][k[4:0]]) ? signed'(rd_vec[k]) : '0;
            p = c * 13'sd2896;
            ld_vec[k] = rect ? TW'((p + (TW+13)'(2048)) >>> 12) : TW'(c);
            ld_vec_f[k*TW +: TW] = ld_vec[k];
        end
    end
    // loaded tile (column pass) as columns: wnd[c*16 + ro] = element (16t + ro, 4jg + c0 + c). For N <= 16 the
    // group's columns start at the tile's first column (c0 = 0); 32-point columns pick a pair, 64-point one.
    logic [TW-1:0] wnd [64];
    always_comb begin
        for (int k = 0; k < 64; k++) wnd[k] = '0;
        for (int ro = 0; ro < 16; ro++)
            case (n_l)
                3'd5:    for (int c = 0; c < 2; c++) wnd[c*16 + ro] = rd_vec[ro*4 + 32'({c0[1], 1'(c)})];
                3'd6:                                wnd[ro]        = rd_vec[ro*4 + 32'(c0)];
                default: for (int c = 0; c < 4; c++) wnd[c*16 + ro] = rd_vec[ro*4 + c];
            endcase
    end
    // streamed column: the 16 rows of column ri in the tile just read
    logic [TW-1:0] cpart [16];
    always_comb for (int ro = 0; ro < 16; ro++) cpart[ro] = rd_vec[ro*4 + 32'(ri[1:0])];
    // read port select: the sequencer while loading, the residual port otherwise
    always_comb begin
        rd_tile = 1'b0; rd_i = res_addr[11:6]; rd_jg = 4'd0; rd_t = 2'd0;
        if (state == S_LD) begin
            if (col) begin rd_tile = 1'b1; rd_jg = idx[5:2] + 4'(gc); rd_t = 2'(gt); end
            else rd_i = cur[5:0];
        end else if (state == S_STRM) begin
            if (col) begin rd_tile = 1'b1; rd_jg = ri[5:2]; rd_t = tc; end
            else rd_i = ri[5:0];
        end
    end
    assign res_data = rd_vec[res_addr[5:0]];
    always_comb for (int l = 0; l < 4; l++) res_data4[l*TW +: TW] = rd_vec[{res_addr[5:2], 2'(l)}];

    // the vector being stored: rows from the bottom slot of pk_out; tiles gathered from pk_out (after the
    // shifts, column tile gc's columns sit in slots 0..3 and row tile gt's rows at offsets 0..15:
    // U[c*16 + ro] = pk_out[c*N + ro]) and placed at tile lanes ro*4 + c0 + c; the streamed chunk; or zeros
    logic [TW-1:0] st_vec [64];
    logic [TW-1:0] ugat [64];
    always_comb begin
        for (int k = 0; k < 64; k++) ugat[k] = '0;
        for (int ro = 0; ro < 16; ro++)
            case (n_l)
                3'd2:    for (int c = 0; c < 4; c++) if (ro < 4)  ugat[c*16 + ro] = pk_out[(c*4 + ro)*TW +: TW];
                3'd3:    for (int c = 0; c < 4; c++) if (ro < 8)  ugat[c*16 + ro] = pk_out[(c*8 + ro)*TW +: TW];
                3'd4:    for (int c = 0; c < 4; c++)              ugat[c*16 + ro] = pk_out[(c*16 + ro)*TW +: TW];
                3'd5:    for (int c = 0; c < 2; c++)              ugat[c*16 + ro] = pk_out[(c*32 + ro)*TW +: TW];
                default:                                          ugat[ro]        = pk_out[ro*TW +: TW];
            endcase
        for (int j = 0; j < 64; j++) st_vec[j] = '0;
        if (state == S_STRM) begin
            if (col) begin
                for (int ro = 0; ro < 16; ro++)
                    for (int l = 0; l < 4; l++) st_vec[ro*4 + l] = st_row[(32'(tw)*16 + ro)*TW +: TW];   // lane st_i[1:0] enabled
            end else begin
                for (int j = 0; j < 32; j++) st_vec[j] = st_row[j*TW +: TW];
            end
        end else if (zero_grp) begin
            ;                                               // rows >= 32: zeros
        end else if (col) begin
            for (int ro = 0; ro < 16; ro++)
                case (n_l)
                    3'd5:    for (int c = 0; c < 2; c++) st_vec[ro*4 + 32'({c0[1], 1'(c)})] = ugat[c*16 + ro];
                    3'd6:                                st_vec[ro*4 + 32'(c0)]             = ugat[ro];
                    default: for (int c = 0; c < 4; c++) st_vec[ro*4 + c]                  = ugat[c*16 + ro];
                endcase
        end else begin
            case (n_l)
                3'd2:    for (int j = 0; j < 4;  j++) st_vec[j] = pk_out[j*TW +: TW];
                3'd3:    for (int j = 0; j < 8;  j++) st_vec[j] = pk_out[j*TW +: TW];
                3'd4:    for (int j = 0; j < 16; j++) st_vec[j] = pk_out[j*TW +: TW];
                3'd5:    for (int j = 0; j < 32; j++) st_vec[j] = pk_out[j*TW +: TW];
                default: for (int j = 0; j < 64; j++) st_vec[j] = pk_out[j*TW +: TW];
            endcase
        end
    end

    // write port: the stored row / tile (rounded, and clamped after the row pass) or one coefficient while idle.
    // One shared round2v/clamp datapath for both passes: the row store rounds by l_rowshift and clamps to the
    // column range, the column store rounds by 4 (0 when lossless) without clamp.
    always_comb begin
        logic [2:0] st_sh;
        logic [6:0] ncols;                                  // columns of this tile that belong to the group
        wr_tile = 1'b0; wr_i = cur[5:0]; wr_jg = idx[5:2] + 4'(gc); wr_t = 2'(gt); wr_en = '0;
        st_sh  = col ? (l_lossless ? 3'd0 : 3'd4) : 3'(l_rowshift);
        ncols  = 7'(gsz) - 7'(gc) * 7'd4;
        for (int k = 0; k < 64; k++) begin
            logic signed [TW-1:0] v;
            v = round2v(signed'(st_vec[k]), st_sh);
            if (!col) v = (v < clip_lo) ? clip_lo : (v > clip_hi) ? clip_hi : v;
            wr_vec[k] = (state == S_ST || state == S_STRM) ? v : '0;
        end
        case (state)
            S_ST: begin
                if (col) begin                                      // tile (gc, gt): the group's columns in it, rows < N
                    wr_tile = 1'b1;
                    for (int ro = 0; ro < 16; ro++)
                        for (int l = 0; l < 4; l++)
                            wr_en[ro*4 + l] = (l >= 32'(c0)) && (32'(l) < 32'(c0) + 32'(ncols)) && (ro < (32'd1 << n_l));
                end else wr_en = (cur < count) ? '1 : '0;
            end
            S_STRM: begin
                if (col) begin
                    wr_tile = 1'b1; wr_jg = st_i[5:2]; wr_t = tw;
                    for (int ro = 0; ro < 16; ro++) wr_en[ro*4 + 32'(st_i[1:0])] = st_v && (ro < (32'd1 << n_l));
                end else begin
                    wr_i = st_i[5:0];
                    wr_en = st_v ? '1 : '0;
                end
            end
            S_IDLE: if (coef_we && !busy) begin
                wr_i = {1'b0, coef_addr[9:5]};
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
    logic last_tile;                    // the column pass's tile loop (gc outer, gt inner) is on its last tile
    assign last_tile = (gt + 3'd1 >= ntr) && (gc + 3'd1 >= ntc || 7'(gc) * 7'd4 + 7'd4 >= 7'(gsz));

    always_ff @(posedge clk) begin
        done    <= 1'b0;
        e_start <= 1'b0;
        if (rst) begin
            state <= S_IDLE;
            busy  <= 1'b0;
            idx   <= '0;
            g     <= '0;
            gc    <= '0;
            gt    <= '0;
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
                    g        <= '0; gc <= '0; gt <= '0;
                    state    <= stream ? S_STRM : zero_grp_next ? S_ST : S_LD;
                    ri <= '0; ld_v <= 1'b0; st_v <= 1'b0; tc <= '0; tw <= '0;
                end
                // ---- streamed rows / columns: the whole pass through the 3-stage pipeline, ntr clocks per vector
                S_STRM: begin
                    // stage 1: a row, or row tile tc (rows 16tc .. 16tc+15) of column ri
                    if (col) begin
                        for (int k = 0; k < 32; k++) if ((k >> 4) == 32'(tc)) ld_row[k*TW +: TW] <= cpart[k & 15];
                    end else ld_row <= ld_vec_f[32*TW-1:0];
                    ld_v <= 1'b0;
                    if (tc + 2'd1 >= 2'(ntr)) begin
                        tc      <= '0;
                        ld_v    <= (ri < count);
                        ld_i    <= ri;
                        ld_zero <= !col && (ri >= 7'd32);
                        ri      <= ri + 7'd1;
                    end else tc <= tc + 2'd1;
                    // stage 2: the vector is complete the clock after its last chunk
                    if (ld_v) begin st_row <= ld_zero ? '0 : rw_out; st_i <= ld_i; st_v <= 1'b1; tw <= '0; end
                    // stage 3: write chunk tw of column st_i (one clock for a row)
                    else if (st_v) begin
                        if (tw + 2'd1 >= 2'(ntr)) begin st_v <= 1'b0; tw <= '0; end
                        else tw <= tw + 2'd1;
                    end
                    if (ri >= count && !ld_v && !st_v && tc == 2'd0) begin   // all vectors read, pipeline drained
                        if (col) state <= S_DONE;
                        else begin col <= 1'b1; first <= 1'b1; state <= S_GSZ; end
                    end
                end
                // ---- load the group: row g into slot g, or tile (gc, gt) scattered into the column slots
                S_LD: begin
                    // constant indices per size (a runtime shift here made Vivado build 64:1 muxes per lane)
                    case (n_l)
                        3'd2: for (int L = 0; L < 64; L++) begin           // N = 4: slot L >> 2, offset L & 3
                            if (col) begin
                                if (((L >> 2) >> 2) == 32'(gc) && (((L & 3) >> 4) == 32'(gt)))
                                    pk_in[L*TW +: TW] <= wnd[((L >> 2) & 3)*16 + (L & 3)];
                            end else begin
                                if ((L >> 2) == 32'(g)) pk_in[L*TW +: TW] <= ld_vec_f[(L & 3)*TW +: TW];
                            end
                        end
                        3'd3: for (int L = 0; L < 64; L++) begin           // N = 8: slot L >> 3, offset L & 7
                            if (col) begin
                                if (((L >> 3) >> 2) == 32'(gc) && (((L & 7) >> 4) == 32'(gt)))
                                    pk_in[L*TW +: TW] <= wnd[((L >> 3) & 3)*16 + (L & 7)];
                            end else begin
                                if ((L >> 3) == 32'(g)) pk_in[L*TW +: TW] <= ld_vec_f[(L & 7)*TW +: TW];
                            end
                        end
                        3'd4: for (int L = 0; L < 64; L++) begin           // N = 16: slot L >> 4, offset L & 15
                            if (col) begin
                                if (((L >> 4) >> 2) == 32'(gc) && (((L & 15) >> 4) == 32'(gt)))
                                    pk_in[L*TW +: TW] <= wnd[((L >> 4) & 3)*16 + (L & 15)];
                            end else begin
                                if ((L >> 4) == 32'(g)) pk_in[L*TW +: TW] <= ld_vec_f[(L & 15)*TW +: TW];
                            end
                        end
                        3'd5: for (int L = 0; L < 64; L++) begin           // N = 32: slot L >> 5, offset L & 31
                            if (col) begin
                                if (((L >> 5) >> 2) == 32'(gc) && (((L & 31) >> 4) == 32'(gt)))
                                    pk_in[L*TW +: TW] <= wnd[((L >> 5) & 3)*16 + (L & 15)];
                            end else begin
                                if ((L >> 5) == 32'(g)) pk_in[L*TW +: TW] <= ld_vec_f[(L & 31)*TW +: TW];
                            end
                        end
                        3'd6: for (int L = 0; L < 64; L++) begin           // N = 64: slot L >> 6, offset L & 63
                            if (col) begin
                                if (((L >> 6) >> 2) == 32'(gc) && (((L & 63) >> 4) == 32'(gt)))
                                    pk_in[L*TW +: TW] <= wnd[((L >> 6) & 3)*16 + (L & 15)];
                            end else begin
                                if ((L >> 6) == 32'(g)) pk_in[L*TW +: TW] <= ld_vec_f[(L & 63)*TW +: TW];
                            end
                        end
                        default: ;
                    endcase
                    if (col) begin
                        if (last_tile) begin gc <= '0; gt <= '0; state <= S_GO; end
                        else if (gt + 3'd1 >= ntr) begin gt <= '0; gc <= gc + 3'd1; end
                        else gt <= gt + 3'd1;
                    end else begin
                        if (g + 5'd1 >= gsz) begin g <= '0; state <= S_GO; end
                        else g <= g + 5'd1;
                    end
                end
                S_GO: begin e_start <= 1'b1; state <= S_RUN; end
                S_RUN: if (e_done) begin pk_out <= e_out; state <= S_ST; end
                // ---- store the group: rows from the bottom slot (shift down by N); tiles (shift down by 16 rows
                //      per row tile, by four slots per column tile)
                S_ST: begin
                    if (col) begin
                        if (gt + 3'd1 >= ntr) begin                 // next column tile: four slots down
                            case (n_l)
                                3'd2:    pk_out <= pk_out >> (16*TW);
                                3'd3:    pk_out <= pk_out >> (32*TW);
                                default: pk_out <= '0;
                            endcase
                        end else pk_out <= pk_out >> (16*TW);        // next row tile
                    end else case (n_l)
                        3'd2:    pk_out <= pk_out >> (4*TW);
                        3'd3:    pk_out <= pk_out >> (8*TW);
                        3'd4:    pk_out <= pk_out >> (16*TW);
                        3'd5:    pk_out <= pk_out >> (32*TW);
                        default: pk_out <= '0;
                    endcase
                    if (col ? last_tile : (g + 5'd1 >= gsz)) begin
                        g <= '0; gc <= '0; gt <= '0;
                        if (idx_next >= count) begin
                            if (col) state <= S_DONE;
                            else begin col <= 1'b1; first <= 1'b1; state <= S_GSZ; end
                        end else state <= S_GSZ;
                    end else if (col) begin
                        if (gt + 3'd1 >= ntr) begin gt <= '0; gc <= gc + 3'd1; end
                        else gt <= gt + 3'd1;
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
