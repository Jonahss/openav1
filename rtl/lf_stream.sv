// Deblocking stage of the streaming filter chain (docs/filter-chain.md), one plane, one superblock row (band) at a
// time, one 4x4 block a clock. Model: tb/lf_stage_model.py DeblockColStage (four block columns in the working window,
// eight context lines carried from band to band).
//
// Blocks of the band enter column by column (left to right; inside a column top to bottom), each with the strength of
// the edge on its left side (vpar) and of the edge on its top side (hpar): {apply, size[4:0], limit[7:0], blimit[7:0],
// thresh[7:0]}, apply = the edge is filtered (7.14.2, level > 0), the rest as lf_filt takes them.
//   pass A, as block (i, r) enters: the vertical edge on the left of column i - 1, row r. Its 14 samples a line lie in
//           columns i - 3 .. i; the four lines of the edge unit go through four lf_filt side by side and the four
//           blocks are stored back. After the band's last column one more (empty) column runs for the last edge.
//           Three clocks: the blocks are read into registers, lf_filt decides, lf_filt's outputs are written back.
//   pass B, for column i - 3 when column i is complete: its horizontal edges top to bottom, through a sliding window
//           of four blocks and four more lf_filt (one per sample column). An edge is decided on the window in one
//           clock and applied in the next, when the window has moved on by a block: the changed samples (lf_filt's
//           mod bits) go into the blocks where they now are. Two applied edges in a row never read each other's
//           changed samples (the filter size is bounded by the transform sizes on both sides), so the decision may
//           run a clock ahead of the apply. The block that has left the window at the top is final and goes out.
//           The window starts with the column's two context blocks (the last eight lines of the band above) and
//           ends by saving the band's last two blocks as the next band's context; in the picture's last band they
//           go out instead.
// The five column buffers (four in pass A, one in pass B) are distributed RAMs that change roles as columns advance.
// Out: finished blocks with their column and row; row counts from the band's first block row, the two context rows
// are rows -2 and -1 (not sent in the picture's first band). No back-pressure on the output yet.
// A block is 16 samples of 12 bits, sample (line l, column c) at bits (4 l + c) * 12.
module lf_stream #(
    parameter int LUMA = 1,                  // 0: a chroma-only stage (lf_filt without the 8- and 14-tap filters)
    parameter int NBMAX = 32,                // most block rows in a band (128-line superblock)
    parameter int CW = 10                    // log2 of the most block columns (4096 / 4)
) (
    input  logic            clk,
    input  logic            rst,
    input  logic [3:0]      bit_depth,
    input  logic            chroma,
    input  logic            band_start,      // pulse while idle
    input  logic [5:0]      n_blk,           // block rows in the band, 1 .. NBMAX
    input  logic [CW:0]     n_col,           // block columns, >= 1
    input  logic            first_band,
    input  logic            last_band,
    output logic            busy,
    output logic            band_done,       // pulse: every block of the band that is final has gone out
    input  logic            in_valid,
    output logic            in_ready,
    input  logic [191:0]    in_blk,
    input  logic [29:0]     in_vpar,
    input  logic [29:0]     in_hpar,
    output logic            out_valid,
    output logic [191:0]    out_blk,
    output logic [CW-1:0]   out_col,
    output logic signed [6:0] out_row
);
    localparam int RW = $clog2(NBMAX);
    // ---------------------------------------------------------------- column and strength buffers
    logic [4:0]      cb_we;
    logic [RW-1:0]   cb_wa [5], cb_ra [5];
    logic [191:0]    cb_wd [5], cb_rd [5];
    logic [4:0]      hb_we;
    logic [RW-1:0]   hb_ra;
    logic [29:0]     hb_q [5];
    logic [RW-1:0]   a_row;
    logic [29:0]     vp_rd;
    logic            vb_we;
    generate for (genvar k = 0; k < 5; k++) begin : g_cb
        lf_cbuf #(.DW(192), .AW(RW)) u_c (.clk, .we(cb_we[k]), .wa(cb_wa[k]), .wd(cb_wd[k]), .ra(cb_ra[k]), .rd(cb_rd[k]));
        lf_cbuf #(.DW(30), .AW(RW)) u_h (.clk, .we(hb_we[k]), .wa(a_row), .wd(in_hpar), .ra(hb_ra), .rd(hb_q[k]));   // top-edge strengths, with the column
    end endgenerate
    lf_cbuf #(.DW(30), .AW(RW)) u_v (.clk, .we(vb_we), .wa(a_row), .wd(in_vpar), .ra(a_row), .rd(vp_rd));   // left-edge strengths of the column before
    logic [383:0]    ctx_mem [1 << CW];      // per block column: the last two blocks of the band above
    logic [383:0]    ctx_q;
    logic            ctx_we;
    logic [CW-1:0]   ctx_wa;
    logic [383:0]    ctx_wd;
    always_ff @(posedge clk) if (ctx_we) ctx_mem[ctx_wa] <= ctx_wd;

    // ---------------------------------------------------------------- pass A: issue
    logic            a_act;
    logic [CW+2:0]   a_col;                  // column entering (n_col .. n_col + 2: empty columns that flush)
    logic [2:0]      s0, s1, s2, s3, sb;     // buffers of columns a_col, a_col - 1, a_col - 2, a_col - 3; the fifth
    logic            a_real, a_step, a_last_row, a_short, slow;
    logic            b_idle, b_pend;
    logic            v1, v2;                 // pass A's pipeline stages hold an edge unit
    assign a_real = (a_col < (CW+3)'(n_col));
    assign a_short = (a_col > (CW+3)'(n_col));               // nothing to filter: one step
    assign a_last_row = a_short || (6'(a_row) == n_blk - 6'd1);
    assign slow = (n_blk < 6'd4);                            // a band shorter than the pipeline: one unit at a time
    logic a_go;
    assign a_go = a_act && !(a_last_row && (!b_idle || b_pend)) && !(slow && (v1 || v2));
    assign in_ready = a_go && a_real;
    assign a_step = a_go && (a_real ? in_valid : 1'b1);
    logic ex3, ex2, ex1;
    assign ex3 = (a_col >= (CW+3)'(3)); assign ex2 = (a_col >= (CW+3)'(2));
    assign ex1 = (a_col >= (CW+3)'(1)) && (a_col <= (CW+3)'(n_col));
    // ---------------------------------------------------------------- pass A: stage 1 (blocks read), stage 2 (decided)
    logic [RW-1:0]   row1, row2;
    logic [2:0]      t3_1, t2_1, t1_1, t0_1, t3_2, t2_2, t1_2, t0_2;      // the buffers the unit's four blocks go back to
    logic [3:0]      w_1, w_2;                                            // which of the four are written back
    logic            ap_1, ap_2;                                          // the edge is filtered
    logic [191:0]    b3_1, b2_1, b1_1, b0_1, b3_2, b2_2, b1_2, b0_2, n3, n2, n1, n0;
    logic [29:0]     vp_1;
    logic [14*12-1:0] a_pix [4];
    logic [12*12-1:0] a_out [4];
    logic [11:0]      a_mod [4];
    always_comb begin                        // the edge unit's four lines
        for (int l = 0; l < 4; l++)
            a_pix[l] = {b0_1[(4*l+2)*12 +: 12], b0_1[(4*l+1)*12 +: 12], b0_1[(4*l+0)*12 +: 12],
                        b1_1[(4*l+3)*12 +: 12], b1_1[(4*l+2)*12 +: 12], b1_1[(4*l+1)*12 +: 12], b1_1[(4*l+0)*12 +: 12],
                        b2_1[(4*l+3)*12 +: 12], b2_1[(4*l+2)*12 +: 12], b2_1[(4*l+1)*12 +: 12], b2_1[(4*l+0)*12 +: 12],
                        b3_1[(4*l+3)*12 +: 12], b3_1[(4*l+2)*12 +: 12], b3_1[(4*l+1)*12 +: 12]};
    end
    generate for (genvar gl = 0; gl < 4; gl++) begin : g_av
        lf_filt #(.PIPE(1), .LUMA(LUMA)) u_f (.clk, .ce(1'b1), .bit_depth, .chroma, .filter_size(vp_1[28:24]), .limit(vp_1[23:16]), .blimit(vp_1[15:8]),
                                 .thresh(vp_1[7:0]), .pix(a_pix[gl]), .out(a_out[gl]), .mod(a_mod[gl]));
    end endgenerate
    always_comb begin                        // stage 2: the changed samples into the four blocks
        n3 = b3_2; n2 = b2_2; n1 = b1_2; n0 = b0_2;
        for (int l = 0; l < 4; l++) begin
            if (ap_2) begin
                if (a_mod[l][0]) n3[(4*l+2)*12 +: 12] = a_out[l][0*12 +: 12];
                if (a_mod[l][1]) n3[(4*l+3)*12 +: 12] = a_out[l][1*12 +: 12];
                for (int c = 0; c < 4; c++) begin
                    if (a_mod[l][2+c]) n2[(4*l+c)*12 +: 12] = a_out[l][(2+c)*12 +: 12];
                    if (a_mod[l][6+c]) n1[(4*l+c)*12 +: 12] = a_out[l][(6+c)*12 +: 12];
                end
                if (a_mod[l][10]) n0[(4*l+0)*12 +: 12] = a_out[l][10*12 +: 12];
                if (a_mod[l][11]) n0[(4*l+1)*12 +: 12] = a_out[l][11*12 +: 12];
            end
        end
    end

    // ---------------------------------------------------------------- pass B
    typedef enum logic [2:0] {B_IDLE, B_P0, B_P1, B_EDGE, B_LAST, B_T0, B_T1, B_CTXW} bst_t;
    bst_t            bst;
    logic [CW-1:0]   b_col;
    logic [2:0]      b_slot;
    logic [5:0]      b_r;                    // the edge being decided
    logic [RW-1:0]   b_ra;                   // the block row read from the column this clock
    logic [191:0]    wx, w0, w1, w2, w3;     // wx: the block that left the window at the top a clock ago
    logic            d_v, d_app;             // an edge was decided last clock (its outputs are valid now); it is filtered
    logic [5:0]      d_r;
    logic [29:0]     hp;
    assign hb_ra = b_r[RW-1:0];
    assign hp = hb_q[b_slot];                // strength of edge b_r
    logic [14*12-1:0] b_pix [4];
    logic [12*12-1:0] b_out [4];
    logic [11:0]      b_mod [4];
    always_comb begin                        // edge b_r on the window: rows b_r - 2 .. b_r + 1 in w0 .. w3
        for (int c = 0; c < 4; c++)
            b_pix[c] = {w3[(4*2+c)*12 +: 12], w3[(4*1+c)*12 +: 12], w3[(4*0+c)*12 +: 12],
                        w2[(4*3+c)*12 +: 12], w2[(4*2+c)*12 +: 12], w2[(4*1+c)*12 +: 12], w2[(4*0+c)*12 +: 12],
                        w1[(4*3+c)*12 +: 12], w1[(4*2+c)*12 +: 12], w1[(4*1+c)*12 +: 12], w1[(4*0+c)*12 +: 12],
                        w0[(4*3+c)*12 +: 12], w0[(4*2+c)*12 +: 12], w0[(4*1+c)*12 +: 12]};
    end
    generate for (genvar gc = 0; gc < 4; gc++) begin : g_bh
        lf_filt #(.PIPE(1), .LUMA(LUMA)) u_f (.clk, .ce(1'b1), .bit_depth, .chroma, .filter_size(hp[28:24]), .limit(hp[23:16]), .blimit(hp[15:8]),
                                 .thresh(hp[7:0]), .pix(b_pix[gc]), .out(b_out[gc]), .mod(b_mod[gc]));
    end endgenerate
    // the edge decided last clock (d_r), applied now: its rows d_r - 2 .. d_r + 1 are in wx, w0, w1, w2
    logic [191:0] wxa, w0a, w1a, w2a;
    always_comb begin
        wxa = wx; w0a = w0; w1a = w1; w2a = w2;
        for (int c = 0; c < 4; c++) begin
            if (d_v && d_app) begin
                if (b_mod[c][0]) wxa[(4*2+c)*12 +: 12] = b_out[c][0*12 +: 12];
                if (b_mod[c][1]) wxa[(4*3+c)*12 +: 12] = b_out[c][1*12 +: 12];
                for (int l = 0; l < 4; l++) begin
                    if (b_mod[c][2+l]) w0a[(4*l+c)*12 +: 12] = b_out[c][(2+l)*12 +: 12];
                    if (b_mod[c][6+l]) w1a[(4*l+c)*12 +: 12] = b_out[c][(6+l)*12 +: 12];
                end
                if (b_mod[c][10]) w2a[(4*0+c)*12 +: 12] = b_out[c][10*12 +: 12];
                if (b_mod[c][11]) w2a[(4*1+c)*12 +: 12] = b_out[c][11*12 +: 12];
            end
        end
    end
    // the fifth buffer is read by pass B (block row b_r + 2 while edge b_r is decided); the others by pass A at its row
    always_comb begin
        case (bst)
            B_P1: b_ra = RW'(1);
            B_EDGE: b_ra = RW'(b_r + 6'd2);
            default: b_ra = '0;
        endcase
        for (int k = 0; k < 5; k++) cb_ra[k] = (!b_idle && b_slot == 3'(k)) ? b_ra : a_row;
    end

    // ---------------------------------------------------------------- writes
    always_comb begin
        for (int k = 0; k < 5; k++) begin
            cb_we[k] = 1'b0; cb_wa[k] = row2; cb_wd[k] = n0; hb_we[k] = 1'b0;
            if (v2) begin
                if (w_2[3] && t3_2 == 3'(k)) begin cb_we[k] = 1'b1; cb_wd[k] = n3; end
                if (w_2[2] && t2_2 == 3'(k)) begin cb_we[k] = 1'b1; cb_wd[k] = n2; end
                if (w_2[1] && t1_2 == 3'(k)) begin cb_we[k] = 1'b1; cb_wd[k] = n1; end
                if (w_2[0] && t0_2 == 3'(k)) begin cb_we[k] = 1'b1; cb_wd[k] = n0; end
            end
            hb_we[k] = a_step && a_real && !a_short && (s0 == 3'(k));
        end
        vb_we = a_step && a_real && !a_short;
    end

    // ---------------------------------------------------------------- sequencing
    logic done_pend;
    logic [CW+2:0] b_pend_col;
    logic [2:0]    b_pend_slot;
    assign b_idle = (bst == B_IDLE);
    assign busy = a_act || !b_idle || b_pend || v1 || v2 || done_pend;
    always_ff @(posedge clk) begin
        out_valid <= 1'b0; band_done <= 1'b0; ctx_we <= 1'b0;
        if (rst) begin
            a_act <= 1'b0; a_col <= '0; a_row <= '0; bst <= B_IDLE; done_pend <= 1'b0; b_pend <= 1'b0; v1 <= 1'b0; v2 <= 1'b0;
            s0 <= 3'd0; s1 <= 3'd1; s2 <= 3'd2; s3 <= 3'd3; sb <= 3'd4;
            b_col <= '0; b_slot <= '0; b_r <= '0; wx <= '0; w0 <= '0; w1 <= '0; w2 <= '0; w3 <= '0; ctx_q <= '0;
            d_v <= 1'b0; d_app <= 1'b0; d_r <= '0;
            out_blk <= '0; out_col <= '0; out_row <= '0; ctx_wa <= '0; ctx_wd <= '0;
            b_pend_col <= '0; b_pend_slot <= '0;
            row1 <= '0; row2 <= '0; w_1 <= '0; w_2 <= '0; ap_1 <= 1'b0; ap_2 <= 1'b0; vp_1 <= '0;
            t3_1 <= '0; t2_1 <= '0; t1_1 <= '0; t0_1 <= '0; t3_2 <= '0; t2_2 <= '0; t1_2 <= '0; t0_2 <= '0;
        end else begin
            if (band_start && !busy) begin a_act <= 1'b1; a_col <= '0; a_row <= '0; done_pend <= 1'b1; end
            // ---- pass A, stage 1: the unit's blocks and strength into registers
            v1 <= a_step && !a_short;
            if (a_step && !a_short) begin
                row1 <= a_row; t3_1 <= s3; t2_1 <= s2; t1_1 <= s1; t0_1 <= s0;
                w_1 <= {ex3, ex2, ex1, a_real};
                b3_1 <= ex3 ? cb_rd[s3] : '0; b2_1 <= ex2 ? cb_rd[s2] : '0; b1_1 <= ex1 ? cb_rd[s1] : '0;
                b0_1 <= a_real ? in_blk : '0;
                vp_1 <= vp_rd; ap_1 <= vp_rd[29] && ex1 && ex2;
            end
            // ---- stage 2: lf_filt has decided (its own register); the blocks follow
            v2 <= v1;
            row2 <= row1; t3_2 <= t3_1; t2_2 <= t2_1; t1_2 <= t1_1; t0_2 <= t0_1; w_2 <= w_1; ap_2 <= ap_1;
            b3_2 <= b3_1; b2_2 <= b2_1; b1_2 <= b1_1; b0_2 <= b0_1;
            // ---- column advance
            if (a_step) begin
                if (a_last_row) begin
                    a_row <= '0; a_col <= a_col + (CW+3)'(1);
                    s0 <= sb; s1 <= s0; s2 <= s1; s3 <= s2; sb <= s3;
                    if (a_col == (CW+3)'(n_col) + (CW+3)'(2)) a_act <= 1'b0;
                    if (a_col >= (CW+3)'(3) && (a_col - (CW+3)'(3)) < (CW+3)'(n_col)) begin
                        b_pend <= 1'b1; b_pend_col <= a_col - (CW+3)'(3); b_pend_slot <= s3;
                    end
                end else a_row <= a_row + 1'b1;
            end
            // ---- pass B
            d_v <= 1'b0;
            case (bst)
                B_IDLE: if (b_pend && !(slow && (v1 || v2))) begin
                    b_pend <= 1'b0;
                    b_col <= CW'(b_pend_col); b_slot <= b_pend_slot; b_r <= 6'd0;
                    ctx_q <= ctx_mem[CW'(b_pend_col)];
                    bst <= B_P0;
                end
                B_P0: begin
                    w0 <= ctx_q[191:0]; w1 <= ctx_q[383:192]; w2 <= cb_rd[b_slot]; bst <= B_P1;
                end
                B_P1: begin w3 <= (n_blk >= 6'd2) ? cb_rd[b_slot] : '0; bst <= B_EDGE; end
                B_EDGE, B_LAST: begin
                    // the edge decided a clock ago is applied where its rows now are; the block that left goes out
                    if (d_v && (d_r >= 6'd2 || !first_band)) begin
                        out_valid <= 1'b1; out_blk <= wxa; out_col <= b_col; out_row <= 7'(d_r) - 7'sd2;
                    end
                    if (bst == B_EDGE) begin
                        // edge b_r is decided this clock (lf_filt's register); the window moves on a block
                        d_v <= 1'b1; d_app <= hp[29]; d_r <= b_r;
                        wx <= w0a; w0 <= w1a; w1 <= w2a; w2 <= w3;
                        w3 <= (b_r + 6'd2 < n_blk) ? cb_rd[b_slot] : '0;
                        b_r <= b_r + 6'd1;
                        if (b_r == n_blk - 6'd1) bst <= B_LAST;
                    end else begin
                        // the last edge's apply: the two blocks still held are final
                        wx <= w0a; w0 <= w1a;
                        bst <= last_band ? B_T0 : B_CTXW;
                    end
                end
                B_T0: begin                                   // the picture's last band: the last two blocks go out too
                    if (n_blk >= 6'd2 || !first_band) begin
                        out_valid <= 1'b1; out_blk <= wx; out_col <= b_col; out_row <= 7'(n_blk) - 7'sd2;
                    end
                    bst <= B_T1;
                end
                B_T1: begin
                    out_valid <= 1'b1; out_blk <= w0; out_col <= b_col; out_row <= 7'(n_blk) - 7'sd1;
                    bst <= B_IDLE;
                end
                B_CTXW: begin ctx_we <= 1'b1; ctx_wa <= b_col; ctx_wd <= {w0, wx}; bst <= B_IDLE; end
                default: bst <= B_IDLE;
            endcase
            if (done_pend && !a_act && b_idle && !b_pend && !v1 && !v2 && !ctx_we && !(band_start && !busy)) begin done_pend <= 1'b0; band_done <= 1'b1; end
        end
    end
endmodule
