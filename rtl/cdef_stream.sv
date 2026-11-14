// CDEF stage of the streaming filter chain (docs/filter-chain.md) for one plane, one superblock row (band) at a time.
// The description below is for luma (a CDEF block = 2 x 2 blocks of 4x4); a subsampled chroma plane's CDEF block is
// one 4x4 block wide and / or high, takes the luma direction from the yd port and steps by its own size.
// First cut: correct and simple, not yet at the planned rate (see "Rate" below).
//
// In: the deblock stage's output as lf_stream sends it: 4x4 blocks of the band column by column, rows -2 .. n_blk - 3
// counted from the band's first block row (the band comes shifted up by 8 lines; the picture's first band starts at
// row 0, its last band runs through row n_blk - 1). The shift is one 8x8 row, so the blocks are aligned to CDEF's grid.
// Per band the stage filters the 8x8 block rows that cover 4x4 rows -4 .. n_blk - 5 (from 0 in the first band, through
// n_blk - 1 in the last): a block row reads two lines below itself, which arrive with the next band. Of the band above
// it keeps, per column, the three blocks of rows -5, -4, -3 (the last 8x8 row and the two lines above it).
// The 8x8 block column C is the 4x4 columns 2C and 2C + 1 and reads columns 2C - 1 .. 2C + 2; it is filtered top to
// bottom when column 2C + 2 has arrived (or the band's columns have ended). Six column buffers rotate: the four of
// the window and two that fill meanwhile.
// Per 8x8 block: the skip flags of its four 4x4 units and its cdef_idx from mi_store decide whether it is filtered;
// cdef_dir gives the direction and variance, cdef_str the parameters, and the block's eight lines go through
// cdef_row (8 lanes), one a clock. A block that is not filtered goes through cdef_row with both strengths 0, which
// returns the samples unchanged.
// Out: line segments of 8 samples: 8x8 block column, line counted from the band's first line (negative above it).
// The output cannot be held line by line; col_hold keeps the next block column from starting instead (a column sends
// at most 4 (n_blk + 4) lines, and up to four lines of the column before may still be on their way out).
// Rate: eight clocks an 8x8 block inside a column (the next block's rows, direction search, skip flags and parameters
// are prepared during the current block's lines); the first block of a column and blocks one 4x4 row high (subsampled
// chroma) take the steps in sequence, about 18 and 14 clocks.
module cdef_stream
  import syn_pkg::*;
#(
    parameter int NBMAX = 32,                // most 4x4 block rows in a band
    parameter int CW = 10                    // log2 of the most 4x4 block columns
) (
    input  logic            clk,
    input  logic            rst,
    input  logic [3:0]      bit_depth,
    input  cdef_hdr_t       ch,
    input  logic            luma,            // the plane is luma: the direction is searched here; chroma takes it from yd
    input  logic            ssx, ssy,        // chroma subsampling (0 for luma): a CDEF block is (2 >> ssx) x (2 >> ssy) 4x4 blocks
    input  logic [MIW-1:0]  mi_rows, mi_cols,   // the picture in 4x4 luma units (skip flags are read there)
    input  logic            band_start,      // pulse while idle
    input  logic [MIW-1:0]  row0,            // the band's first 4x4 row in the plane
    input  logic [MIW-1:0]  pic_rows,        // the plane's 4x4 rows
    input  logic [5:0]      n_blk,           // the plane's 4x4 block rows in the band
    input  logic [CW:0]     n_col,           // the plane's 4x4 block columns
    input  logic            first_band,
    input  logic            last_band,
    output logic            busy,
    output logic            band_done,
    input  logic            in_valid,
    output logic            in_ready,
    input  logic [191:0]    in_blk,
    // mode info: skip flag of a 4x4 unit and cdef_idx of its 64x64 unit, both one clock after the address
    output logic [MIW-1:0]  rd_row, rd_col,
    input  logic            rd_skip,
    output logic [SBW-1:0]  cdr_row64, cdr_col64,
    input  logic [3:0]      cdr_val,         // {valid, idx}
    // chroma: the luma direction of the 8x8 block at 4x4 luma unit (yd_row, yd_col), one clock after the address
    output logic [MIW-1:0]  yd_row, yd_col,
    input  logic [2:0]      yd,
    // luma: the direction found for the block at (dw_row, dw_col), for the chroma stages
    output logic            dw_we,
    output logic [MIW-1:0]  dw_row, dw_col,
    output logic [2:0]      dw_dir,
    input  logic            col_hold,        // do not start a block column (the taker has no room for a column's lines)
    output logic [CW:0]     col_next,        // first 4x4 column of the block column filtered next: every column before it is done
    output logic            out_valid,
    output logic [95:0]     out_line,        // out_n4 groups of four samples from bit 0
    output logic [1:0]      out_n4,          // 1 or 2
    output logic [CW-1:0]   out_col,         // the segment's first 4x4 column
    output logic signed [8:0] out_y
);
    localparam int AW = $clog2(NBMAX + 4);
    logic [3:0] cs;
    assign cs = bit_depth - 4'd8;
    logic [1:0] bw4, bh4;                    // the plane's CDEF block in 4x4 blocks
    assign bw4 = ssx ? 2'd1 : 2'd2;
    assign bh4 = ssy ? 2'd1 : 2'd2;
    // ---------------------------------------------------------------- column buffers (entry = block row + 2) and carried rows
    logic [5:0]      cb_we;
    logic [AW-1:0]   cb_wa, cb_ra;
    logic [191:0]    cb_rd [6];
    generate for (genvar k = 0; k < 6; k++) begin : g_cb
        lf_cbuf #(.DW(192), .AW(AW)) u_c (.clk, .we(cb_we[k]), .wa(cb_wa), .wd(in_blk), .ra(cb_ra), .rd(cb_rd[k]));
    end endgenerate
    logic [575:0]    ctx_mem [1 << CW];      // per 4x4 column: rows -5, -4, -3 of the band above ({row -3, row -4, row -5})
    logic [575:0]    ctx_q;
    logic            ctx_we;
    logic [CW-1:0]   ctx_a;
    logic [575:0]    ctx_wd;
    always_ff @(posedge clk) begin
        if (ctx_we) ctx_mem[ctx_a] <= ctx_wd;
        ctx_q <= ctx_mem[ctx_a];
    end

    // ---------------------------------------------------------------- arrival
    logic            a_act;
    logic [CW:0]     a_col;                  // 4x4 column arriving (n_col: all in)
    logic signed [6:0] a_row;                // its next block row
    logic [2:0]      a_slot;                 // buffer of the arriving column (a_col mod 6)
    logic [CW:0]     p_c2;                   // first 4x4 column of the block column filtered next
    logic signed [6:0] a_first, a_last;      // first and last block row that arrives
    assign a_first = first_band ? 7'sd0 : -7'sd2;
    assign a_last  = last_band ? 7'(n_blk) - 7'sd1 : 7'(n_blk) - 7'sd3;
    // a column may arrive while its buffer is not one of the window's: columns up to 2C + 4
    assign in_ready = a_act && (a_col < n_col) && (a_col <= p_c2 + (CW+1)'(4));
    logic a_step;
    assign a_step = in_valid && in_ready;
    always_comb begin
        cb_we = '0;
        cb_wa = AW'(a_row + 7'sd2);
        if (a_step) cb_we[a_slot] = 1'b1;
    end

    // ---------------------------------------------------------------- filtering one 8x8 block column
    typedef enum logic [3:0] {P_IDLE, P_CTX, P_CTXW, P_ROW, P_SKIP, P_DIRW, P_STR, P_PIX, P_NEXT, P_SAVE_RD, P_SAVE_WR, P_DONE} pst_t;
    pst_t            pst;
    logic [2:0]      p_base;                 // buffer of column 2C - 1; the window's column k is in buffer (p_base + k) mod 6
    logic [1:0]      k_cnt;                  // window column being loaded / saved
    logic [575:0]    ctxw [4];               // the window columns' carried rows
    logic signed [6:0] p_r;                  // first 4x4 row of the 8x8 block (even)
    logic signed [6:0] p_first, p_last;      // first and last 8x8 row (as its first 4x4 row)
    // a block row needs two lines of the row below it: a band's own block rows end three 4x4 rows before its last
    // arriving row (rounded down to the block height), and the band starts where the band above stopped
    assign p_first = first_band ? 7'sd0 : -(7'sd2 + 7'(bh4));
    assign p_last  = last_band ? 7'(n_blk) - 7'(bh4) : (ssy ? 7'(n_blk) - 7'sd4 : 7'(n_blk) - 7'sd6);
    logic signed [6:0] ld_r;                 // the block row being loaded into the window
    logic [2:0]      ld_n;                   // rows still to load before the block can be filtered
    logic [191:0]    win [4][4];             // [row r - 1 .. r + 2][column 2C - 1 .. 2C + 2]
    // Look-ahead (blocks two 4x4 rows high): while a block's eight lines go out, the two rows the next block adds are
    // read (nx3, nx4), its direction is searched, its skip flags and cdef_idx are read and its parameters formed, so
    // that its lines follow without a gap: eight clocks a block inside a column.
    logic [191:0]    nx3 [4], nx4 [4];
    logic            la;                     // the block's lines are running with a next block being prepared
    logic            la_blk;                 // addresses and the direction search are the next block's
    logic            n_sk_all;
    logic [2:0]      n_cidx;
    logic            n_cidx_v;
    function automatic logic [2:0] slot_k(input logic [2:0] base, input logic [1:0] k);
        logic [3:0] t;
        t = 4'(base) + 4'(k);
        slot_k = (t >= 4'd6) ? 3'(t - 4'd6) : 3'(t);
    endfunction
    // a column of the window exists in the picture
    function automatic logic col_ok(input logic [1:0] k);
        logic signed [CW+2:0] c;
        c = $signed({2'b0, p_c2}) + $signed({{(CW+1){1'b0}}, k}) - (CW+3)'(1);
        col_ok = (c >= 0) && (c < $signed({2'b0, n_col}));
    endfunction
    // block (row ld_r, window column k): from the carried rows, the column's buffer, or nothing
    logic [191:0] ld_blk [4];
    assign cb_ra = AW'(ld_r + 7'sd2);
    always_comb begin
        for (int k = 0; k < 4; k++) begin
            ld_blk[k] = '0;
            if (col_ok(2'(k))) begin
                if (ld_r <= -7'sd3) begin
                    if (ld_r >= -7'sd5) ld_blk[k] = ctxw[k][192 * 3'(ld_r + 7'sd5) +: 192];
                end else if (ld_r <= a_last) ld_blk[k] = cb_rd[slot_k(p_base, 2'(k))];
            end
        end
    end
    // the 8x8 block's samples for the direction search: window rows 1, 2 x columns 1, 2
    logic [767:0] dir_px;
    always_comb begin
        for (int i = 0; i < 8; i++)
            for (int j = 0; j < 8; j++)
                dir_px[(8 * i + j) * 12 +: 12] = la_blk ? ((i < 4) ? win[3][1 + j / 4][((i % 4) * 4 + (j % 4)) * 12 +: 12]
                                                                   : nx3[1 + j / 4][((i % 4) * 4 + (j % 4)) * 12 +: 12])
                                                        : win[1 + i / 4][1 + j / 4][((i % 4) * 4 + (j % 4)) * 12 +: 12];
    end
    logic        dir_go, dir_v;
    logic [2:0]  dir_d;
    logic [31:0] dir_var;
    cdef_dir u_dir (.clk, .rst, .v_in(dir_go), .px(dir_px), .cs, .v_out(dir_v), .dir(dir_d), .var_o(dir_var));
    // skip flags and cdef_idx
    logic [1:0]  sk_cnt;
    logic        sk_all, filt;               // all four units skipped so far; the block is filtered
    logic [2:0]  cidx;
    logic        cidx_v;
    logic [MIW-1:0] b_mr, b_mc, b_mr1, b_mc1;    // the block's first 4x4 unit and the next (clamped to the picture)
    always_comb begin
        b_mr = MIW'($signed({1'b0, row0}) + (MIW+1)'(p_r) + (la_blk ? (MIW+1)'(2) : (MIW+1)'(0))) << ssy;
        b_mc = MIW'(p_c2) << ssx;
        b_mr1 = (b_mr + MIW'(1) < mi_rows) ? b_mr + MIW'(1) : mi_rows - MIW'(1);
        b_mc1 = (b_mc + MIW'(1) < mi_cols) ? b_mc + MIW'(1) : mi_cols - MIW'(1);
        yd_row = b_mr; yd_col = b_mc;
        rd_row = sk_cnt[0] ? b_mr1 : b_mr;
        rd_col = sk_cnt[1] ? b_mc1 : b_mc;
        cdr_row64 = SBW'(b_mr >> 4); cdr_col64 = SBW'(b_mc >> 4);
    end
    logic [2:0]  s_dr; logic [11:0] s_pri, s_sec; logic [4:0] s_adjp, s_adjs; logic s_odd;
    logic [2:0]  dr_r; logic [11:0] pri_r, sec_r; logic [4:0] adjp_r, adjs_r; logic odd_r;
    logic [2:0]  yd_q;                       // chroma: the luma direction as read
    cdef_str u_str (.ch, .cs, .ssx, .ssy, .idx(la_blk ? n_cidx : cidx), .luma, .ydir(luma ? dir_d : yd_q), .var_i(dir_var),
                    .dr(s_dr), .pri(s_pri), .sec(s_sec), .adjp(s_adjp), .adjs(s_adjs), .priodd(s_odd));
    // the line's neighbourhood: 5 lines x 12 columns out of the window (16 lines x 16 columns; the block starts at 4, 4)
    logic [2:0]  p_l;                        // line of the block
    logic [5*12*12-1:0] nb;
    logic [5*12-1:0]    nbok;
    logic signed [MIW+3:0] y_top;            // the block's first line in the picture
    assign y_top = ((MIW+4)'($signed({2'b0, row0})) + (MIW+4)'(p_r)) <<< 2;
    always_comb begin
        for (int rr = 0; rr < 5; rr++)
            for (int cc = 0; cc < 12; cc++) begin
                int wl, wc;
                logic signed [MIW+3:0] ya;
                logic signed [CW+4:0]  xa;
                wl = int'(p_l) + rr + 2;                 // window line (0 .. 15)
                wc = cc + 2;                             // window sample column (0 .. 15)
                nb[(rr * 12 + cc) * 12 +: 12] = win[wl / 4][wc / 4][((wl % 4) * 4 + (wc % 4)) * 12 +: 12];
                ya = y_top + (MIW+4)'(p_l) + (MIW+4)'(rr) - (MIW+4)'(2);
                xa = ($signed({4'b0, p_c2}) <<< 2) + (CW+5)'(cc) - (CW+5)'(2);
                nbok[rr * 12 + cc] = (ya >= 0) && (ya < ($signed({4'b0, pic_rows}) <<< 2)) && (xa >= 0) && (xa < ($signed({4'b0, n_col}) <<< 2));
            end
    end
    logic        row_v, row_vo;
    logic [95:0] row_out;
    cdef_row #(.N(8)) u_row (.clk, .rst, .v_in(row_v), .nb, .nbok, .dr(dr_r), .pri(pri_r), .sec(sec_r), .adjp(adjp_r), .adjs(adjs_r),
                             .priodd(odd_r), .v_out(row_vo), .out(row_out));
    assign row_v = (pst == P_PIX);
    // the line's position travels beside cdef_row (four clocks)
    logic [CW-1:0]      tg_c [4];
    logic signed [8:0]  tg_y [4];
    always_ff @(posedge clk) begin
        tg_c[0] <= CW'(p_c2); tg_y[0] <= 9'((p_r <<< 2)) + 9'(p_l);
        for (int i = 1; i < 4; i++) begin tg_c[i] <= tg_c[i - 1]; tg_y[i] <= tg_y[i - 1]; end
    end
    assign out_valid = row_vo; assign out_line = row_out; assign out_col = tg_c[3]; assign out_y = tg_y[3];
    assign out_n4 = bw4;
    assign col_next = p_c2;
    // luma: the block's direction for the chroma stages, as cdef_dir delivers it
    assign dw_we = luma && ((pst == P_STR) || (pst == P_PIX && la && p_l == 3'd7)); assign dw_row = b_mr; assign dw_col = b_mc; assign dw_dir = dir_d;

    // ---------------------------------------------------------------- sequencing
    logic done_pend;
    logic [1:0] tail;                        // lines still inside cdef_row after the last block
    logic [2:0] wait_n;
    logic       p_can;                       // the block column's window is complete
    assign p_can = a_act && (p_c2 < n_col) && ((a_col > p_c2 + (CW+1)'(bw4)) || (a_col == n_col));   // (with !col_hold)
    assign busy = a_act || (pst != P_IDLE) || done_pend;
    always_ff @(posedge clk) begin
        band_done <= 1'b0; dir_go <= 1'b0; ctx_we <= 1'b0;
        if (rst) begin
            a_act <= 1'b0; a_col <= '0; a_row <= '0; a_slot <= '0; p_c2 <= '0; p_base <= 3'd5; pst <= P_IDLE; done_pend <= 1'b0;
            k_cnt <= '0; p_r <= '0; ld_r <= '0; ld_n <= '0; sk_cnt <= '0; sk_all <= 1'b0; filt <= 1'b0; cidx <= '0; cidx_v <= 1'b0;
            p_l <= '0; wait_n <= '0; tail <= '0; ctx_a <= '0; ctx_wd <= '0; la <= 1'b0; la_blk <= 1'b0; n_sk_all <= 1'b0; n_cidx <= '0; n_cidx_v <= 1'b0;
            dr_r <= '0; pri_r <= '0; sec_r <= '0; adjp_r <= '0; adjs_r <= '0; odd_r <= 1'b0;
        end else begin
            if (band_start && !busy) begin
                a_act <= 1'b1; a_col <= '0; a_row <= a_first; a_slot <= 3'd0; p_c2 <= '0; p_base <= 3'd5; done_pend <= 1'b1;
            end
            // ---- arrival
            if (a_step) begin
                if (a_row == a_last) begin
                    a_row <= a_first; a_col <= a_col + 1'b1; a_slot <= (a_slot == 3'd5) ? 3'd0 : a_slot + 3'd1;
                end else a_row <= a_row + 7'sd1;
            end
            // ---- one 8x8 block column
            case (pst)
                P_IDLE: if (p_can && !col_hold) begin k_cnt <= 2'd0; ctx_a <= CW'(p_c2 - (CW+1)'(1)); pst <= P_CTX; end
                // the four columns' carried rows: address out, a clock for the memory, then the data
                P_CTX: pst <= P_CTXW;
                P_CTXW: begin
                    ctxw[k_cnt] <= ctx_q;
                    k_cnt <= k_cnt + 2'd1;
                    ctx_a <= CW'(p_c2 + (CW+1)'(k_cnt));        // column 2C - 1 + (k_cnt + 1)
                    if (k_cnt == 2'd3) begin
                        p_r <= p_first; ld_n <= 3'd4; k_cnt <= 2'd0;
                        if (p_first > p_last) begin ld_r <= 7'(n_blk) - 7'sd5; pst <= P_SAVE_RD; end   // a band too short to hold an 8x8 row of its own
                        else begin ld_r <= p_first - 7'sd1; pst <= P_ROW; end
                    end else pst <= P_CTX;
                end
                // rows into the window: four before the column's first block, two before every other
                P_ROW: begin
                    for (int k = 0; k < 4; k++) begin
                        win[0][k] <= win[1][k]; win[1][k] <= win[2][k]; win[2][k] <= win[3][k]; win[3][k] <= ld_blk[k];
                    end
                    ld_r <= ld_r + 7'sd1; ld_n <= ld_n - 3'd1;
                    if (ld_n == 3'd1) begin sk_cnt <= 2'd0; sk_all <= 1'b1; pst <= P_SKIP; dir_go <= 1'b1; end
                end
                // the four skip flags (address now, flag next clock) and cdef_idx; the direction search runs meanwhile
                P_SKIP: begin
                    if (sk_cnt != 2'd0) sk_all <= sk_all && rd_skip;
                    if (sk_cnt == 2'd1) begin cidx_v <= cdr_val[3]; cidx <= cdr_val[2:0]; end
                    sk_cnt <= sk_cnt + 2'd1;
                    if (sk_cnt == 2'd3) begin wait_n <= 3'd0; pst <= P_DIRW; end
                end
                P_DIRW: begin
                    if (wait_n == 3'd0) sk_all <= sk_all && rd_skip;      // the fourth flag
                    wait_n <= wait_n + 3'd1;
                    yd_q <= yd;
                    if (luma ? dir_v : (wait_n != 3'd0)) pst <= P_STR;
                end
                P_STR: begin
                    filt <= cidx_v && !sk_all;
                    dr_r <= s_dr; adjp_r <= s_adjp; adjs_r <= s_adjs; odd_r <= s_odd;
                    pri_r <= (cidx_v && !sk_all) ? s_pri : 12'd0;
                    sec_r <= (cidx_v && !sk_all) ? s_sec : 12'd0;
                    p_l <= 3'd0; pst <= P_PIX;
                    la <= (bh4 == 2'd2) && (p_r < p_last);
                end
                P_PIX: begin
                    p_l <= p_l + 3'd1;
                    if (la) begin
                        // the next block alongside this block's lines
                        case (p_l)
                            3'd0: begin for (int k = 0; k < 4; k++) nx3[k] <= ld_blk[k]; ld_r <= ld_r + 7'sd1; end
                            3'd1: begin
                                for (int k = 0; k < 4; k++) nx4[k] <= ld_blk[k];
                                ld_r <= ld_r + 7'sd1; la_blk <= 1'b1; dir_go <= 1'b1; sk_cnt <= 2'd0; n_sk_all <= 1'b1;
                            end
                            3'd2: sk_cnt <= 2'd1;                                   // first skip address out
                            3'd3: begin n_sk_all <= n_sk_all && rd_skip; n_cidx_v <= cdr_val[3]; n_cidx <= cdr_val[2:0]; sk_cnt <= 2'd2; yd_q <= yd; end
                            3'd4: begin n_sk_all <= n_sk_all && rd_skip; sk_cnt <= 2'd3; end
                            3'd5: n_sk_all <= n_sk_all && rd_skip;
                            3'd6: n_sk_all <= n_sk_all && rd_skip;                  // the fourth flag; the direction arrives
                            default: begin                                          // line 7: the next block takes over
                                for (int k = 0; k < 4; k++) begin
                                    win[0][k] <= win[2][k]; win[1][k] <= win[3][k]; win[2][k] <= nx3[k]; win[3][k] <= nx4[k];
                                end
                                filt <= n_cidx_v && !n_sk_all; cidx <= n_cidx; cidx_v <= n_cidx_v; sk_all <= n_sk_all;
                                dr_r <= s_dr; adjp_r <= s_adjp; adjs_r <= s_adjs; odd_r <= s_odd;
                                pri_r <= (n_cidx_v && !n_sk_all) ? s_pri : 12'd0;
                                sec_r <= (n_cidx_v && !n_sk_all) ? s_sec : 12'd0;
                                p_r <= p_r + 7'sd2; p_l <= 3'd0; la_blk <= 1'b0;
                                la <= (p_r + 7'sd2 < p_last);                       // and a block after that one?
                            end
                        endcase
                    end else if (p_l == ((bh4 == 2'd2) ? 3'd7 : 3'd3)) pst <= P_NEXT;
                end
                P_NEXT: begin
                    if (p_r >= p_last) begin k_cnt <= 2'd0; ld_r <= 7'(n_blk) - 7'sd5; pst <= P_SAVE_RD; end
                    else begin p_r <= p_r + 7'(bh4); ld_n <= 3'(bh4); pst <= P_ROW; end
                end
                // the next band's carried rows (n_blk - 5 .. n_blk - 3) of the two columns leaving the window (all four
                // after the band's last block column); nothing to keep after the picture's last band
                P_SAVE_RD: begin
                    if (last_band) pst <= P_DONE;
                    else begin
                        ctx_wd[192 * 3'(ld_r - (7'(n_blk) - 7'sd5)) +: 192] <= ld_blk[k_cnt];
                        if (ld_r == 7'(n_blk) - 7'sd3) pst <= P_SAVE_WR;
                        else ld_r <= ld_r + 7'sd1;
                    end
                end
                P_SAVE_WR: begin
                    if (col_ok(k_cnt)) begin ctx_we <= 1'b1; ctx_a <= CW'(p_c2 + (CW+1)'(k_cnt) - (CW+1)'(1)); end
                    k_cnt <= k_cnt + 2'd1; ld_r <= 7'(n_blk) - 7'sd5;
                    if ((k_cnt == bw4 - 2'd1 && (p_c2 + (CW+1)'(bw4) < n_col)) || k_cnt == 2'd3) pst <= P_DONE;
                    else pst <= P_SAVE_RD;
                end
                P_DONE: begin
                    p_c2 <= p_c2 + (CW+1)'(bw4);
                    p_base <= slot_k(p_base, bw4);
                    tail <= 2'd3;
                    pst <= P_IDLE;
                end
                default: pst <= P_IDLE;
            endcase
            if (a_act && a_col == n_col && p_c2 >= n_col && pst == P_IDLE) a_act <= 1'b0;
            if (tail != 2'd0 && pst == P_IDLE) tail <= tail - 2'd1;
            if (done_pend && !a_act && pst == P_IDLE && tail == 2'd0 && !row_vo && !(band_start && !busy)) begin
                done_pend <= 1'b0; band_done <= 1'b1;
            end
        end
    end
endmodule
