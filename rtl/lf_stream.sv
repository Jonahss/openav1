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
//   pass B, for column i - 3 when column i is complete: its horizontal edges top to bottom, through a sliding window
//           of four blocks (rows r - 2 .. r + 1) and four more lf_filt (one per sample column). The block that
//           leaves the window at the top is final and goes out. The window starts with the column's two context
//           blocks (the last eight lines of the band above) and ends by saving the band's last two blocks as the
//           next band's context; in the picture's last band they go out instead.
// Out: finished blocks with their column and row; row counts from the band's first block row, the two context rows
// are rows -2 and -1 (not sent in the picture's first band). No back-pressure on the output yet.
// A block is 16 samples of 12 bits, sample (line l, column c) at bits (4 l + c) * 12.
module lf_stream #(
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
    // ---------------------------------------------------------------- storage
    logic [191:0] colbuf [5][NBMAX];         // the four columns of pass A and the one of pass B
    logic [29:0]  hbuf [5][NBMAX];           // top-edge strengths, kept with the column until pass B
    logic [29:0]  vbuf [NBMAX];              // left-edge strengths of the column that entered last
    logic [383:0] ctx_mem [1 << CW];         // per block column: the last two blocks of the band above
    logic [383:0] ctx_q;

    function automatic logic [2:0] slot(input logic [CW+2:0] c);     // column -> buffer
        slot = 3'(c % 5);
    endfunction

    // ---------------------------------------------------------------- pass A
    logic            a_act;
    logic [CW+2:0]   a_col;                  // column entering (n_col .. n_col + 2: empty columns that flush)
    logic [RW-1:0]   a_row;
    logic            a_real, a_step, a_last_row, a_short;
    logic            b_idle;
    assign a_real = (a_col < (CW+3)'(n_col));
    assign a_short = (a_col > (CW+3)'(n_col));               // nothing to filter: one step
    assign a_last_row = a_short || (6'(a_row) == n_blk - 6'd1);
    assign in_ready = a_act && a_real && !(a_last_row && !b_idle);
    assign a_step = a_act && (a_real ? (in_valid && in_ready) : (!a_last_row || b_idle));
    logic [2:0] s3, s2, s1, s0;
    assign s3 = slot(a_col + (CW+3)'(2)); assign s2 = slot(a_col + (CW+3)'(3));   // (a_col - 3) and (a_col - 2) mod 5
    assign s1 = slot(a_col + (CW+3)'(4)); assign s0 = slot(a_col);
    logic ex3, ex2, ex1;
    assign ex3 = (a_col >= (CW+3)'(3)); assign ex2 = (a_col >= (CW+3)'(2));
    assign ex1 = (a_col >= (CW+3)'(1)) && (a_col <= (CW+3)'(n_col));
    logic [191:0] a3, a2, a1, a0, n3, n2, n1, n0;
    assign a3 = ex3 ? colbuf[s3][a_row] : '0; assign a2 = ex2 ? colbuf[s2][a_row] : '0;
    assign a1 = ex1 ? colbuf[s1][a_row] : '0; assign a0 = a_real ? in_blk : '0;
    logic [29:0] vp;
    assign vp = vbuf[a_row];
    logic v_apply;
    assign v_apply = vp[29] && ex1 && ex2 && !a_short;
    logic [14*12-1:0] a_pix [4];
    logic [12*12-1:0] a_out [4];
    logic [11:0]      a_mod [4];
    always_comb begin                        // the edge unit's four lines (a block of its own: the results below read a_out)
        for (int l = 0; l < 4; l++)
            a_pix[l] = {a0[(4*l+2)*12 +: 12], a0[(4*l+1)*12 +: 12], a0[(4*l+0)*12 +: 12],
                        a1[(4*l+3)*12 +: 12], a1[(4*l+2)*12 +: 12], a1[(4*l+1)*12 +: 12], a1[(4*l+0)*12 +: 12],
                        a2[(4*l+3)*12 +: 12], a2[(4*l+2)*12 +: 12], a2[(4*l+1)*12 +: 12], a2[(4*l+0)*12 +: 12],
                        a3[(4*l+3)*12 +: 12], a3[(4*l+2)*12 +: 12], a3[(4*l+1)*12 +: 12]};
    end
    always_comb begin
        n3 = a3; n2 = a2; n1 = a1; n0 = a0;
        for (int l = 0; l < 4; l++) begin
            if (v_apply) begin
                n3[(4*l+2)*12 +: 12] = a_out[l][0*12 +: 12]; n3[(4*l+3)*12 +: 12] = a_out[l][1*12 +: 12];
                for (int c = 0; c < 4; c++) begin
                    n2[(4*l+c)*12 +: 12] = a_out[l][(2+c)*12 +: 12];
                    n1[(4*l+c)*12 +: 12] = a_out[l][(6+c)*12 +: 12];
                end
                n0[(4*l+0)*12 +: 12] = a_out[l][10*12 +: 12]; n0[(4*l+1)*12 +: 12] = a_out[l][11*12 +: 12];
            end
        end
    end
    generate for (genvar gl = 0; gl < 4; gl++) begin : g_av
        lf_filt u_f (.bit_depth, .chroma, .filter_size(vp[28:24]), .limit(vp[23:16]), .blimit(vp[15:8]), .thresh(vp[7:0]),
                     .pix(a_pix[gl]), .out(a_out[gl]), .mod(a_mod[gl]));
    end endgenerate

    // ---------------------------------------------------------------- pass B
    typedef enum logic [2:0] {B_IDLE, B_P0, B_P1, B_EDGE, B_T0, B_T1, B_CTXW} bst_t;
    bst_t            bst;
    logic [CW-1:0]   b_col;
    logic [2:0]      b_slot;
    logic [5:0]      b_r;
    logic [191:0]    w0, w1, w2, w3, w0n, w1n, w2n, w3n;
    logic            b_trig;
    logic [CW+2:0]   b_trig_col;
    assign b_idle = (bst == B_IDLE);
    assign b_trig_col = a_col - (CW+3)'(3);
    assign b_trig = a_step && a_last_row && (a_col >= (CW+3)'(3)) && (b_trig_col < (CW+3)'(n_col));
    logic [29:0] hp;
    assign hp = hbuf[b_slot][b_r[RW-1:0]];
    logic [14*12-1:0] b_pix [4];
    logic [12*12-1:0] b_out [4];
    logic [11:0]      b_mod [4];
    always_comb begin
        for (int c = 0; c < 4; c++)
            b_pix[c] = {w3[(4*2+c)*12 +: 12], w3[(4*1+c)*12 +: 12], w3[(4*0+c)*12 +: 12],
                        w2[(4*3+c)*12 +: 12], w2[(4*2+c)*12 +: 12], w2[(4*1+c)*12 +: 12], w2[(4*0+c)*12 +: 12],
                        w1[(4*3+c)*12 +: 12], w1[(4*2+c)*12 +: 12], w1[(4*1+c)*12 +: 12], w1[(4*0+c)*12 +: 12],
                        w0[(4*3+c)*12 +: 12], w0[(4*2+c)*12 +: 12], w0[(4*1+c)*12 +: 12]};
    end
    always_comb begin
        w0n = w0; w1n = w1; w2n = w2; w3n = w3;
        for (int c = 0; c < 4; c++) begin
            if (hp[29]) begin
                w0n[(4*2+c)*12 +: 12] = b_out[c][0*12 +: 12]; w0n[(4*3+c)*12 +: 12] = b_out[c][1*12 +: 12];
                for (int l = 0; l < 4; l++) begin
                    w1n[(4*l+c)*12 +: 12] = b_out[c][(2+l)*12 +: 12];
                    w2n[(4*l+c)*12 +: 12] = b_out[c][(6+l)*12 +: 12];
                end
                w3n[(4*0+c)*12 +: 12] = b_out[c][10*12 +: 12]; w3n[(4*1+c)*12 +: 12] = b_out[c][11*12 +: 12];
            end
        end
    end
    generate for (genvar gc = 0; gc < 4; gc++) begin : g_bh
        lf_filt u_f (.bit_depth, .chroma, .filter_size(hp[28:24]), .limit(hp[23:16]), .blimit(hp[15:8]), .thresh(hp[7:0]),
                     .pix(b_pix[gc]), .out(b_out[gc]), .mod(b_mod[gc]));
    end endgenerate

    // ---------------------------------------------------------------- sequencing
    logic done_pend;
    assign busy = a_act || !b_idle || done_pend;
    always_ff @(posedge clk) begin
        out_valid <= 1'b0; band_done <= 1'b0;
        if (rst) begin
            a_act <= 1'b0; a_col <= '0; a_row <= '0; bst <= B_IDLE; done_pend <= 1'b0;
            b_col <= '0; b_slot <= '0; b_r <= '0; w0 <= '0; w1 <= '0; w2 <= '0; w3 <= '0; ctx_q <= '0;
            out_blk <= '0; out_col <= '0; out_row <= '0;
        end else begin
            if (band_start && !busy) begin a_act <= 1'b1; a_col <= '0; a_row <= '0; done_pend <= 1'b1; end
            // ---- pass A
            if (a_step) begin
                if (!a_short) begin
                    if (ex3) colbuf[s3][a_row] <= n3;
                    if (ex2) colbuf[s2][a_row] <= n2;
                    if (ex1) colbuf[s1][a_row] <= n1;
                    if (a_real) begin colbuf[s0][a_row] <= n0; hbuf[s0][a_row] <= in_hpar; vbuf[a_row] <= in_vpar; end
                end
                if (a_last_row) begin
                    a_row <= '0; a_col <= a_col + (CW+3)'(1);
                    if (a_col == (CW+3)'(n_col) + (CW+3)'(2)) a_act <= 1'b0;
                end else a_row <= a_row + 1'b1;
            end
            // ---- pass B
            case (bst)
                B_IDLE: if (b_trig) begin
                    b_col <= CW'(b_trig_col); b_slot <= slot(b_trig_col); b_r <= 6'd0;
                    ctx_q <= ctx_mem[CW'(b_trig_col)];
                    bst <= B_P0;
                end
                B_P0: begin
                    w0 <= ctx_q[191:0]; w1 <= ctx_q[383:192]; w2 <= colbuf[b_slot][0]; bst <= B_P1;
                end
                B_P1: begin w3 <= colbuf[b_slot][1]; bst <= B_EDGE; end
                B_EDGE: begin
                    if (b_r >= 6'd2 || !first_band) begin
                        out_valid <= 1'b1; out_blk <= w0n; out_col <= b_col; out_row <= 7'(b_r) - 7'sd2;
                    end
                    w0 <= w1n; w1 <= w2n; w2 <= w3n;
                    w3 <= (b_r + 6'd2 < n_blk) ? colbuf[b_slot][RW'(b_r + 6'd2)] : '0;
                    b_r <= b_r + 6'd1;
                    if (b_r == n_blk - 6'd1) bst <= last_band ? B_T0 : B_CTXW;
                end
                B_T0: begin                                   // the picture's last band: the last two blocks go out too
                    if (n_blk >= 6'd2 || !first_band) begin
                        out_valid <= 1'b1; out_blk <= w0; out_col <= b_col; out_row <= 7'(n_blk) - 7'sd2;
                    end
                    bst <= B_T1;
                end
                B_T1: begin
                    out_valid <= 1'b1; out_blk <= w1; out_col <= b_col; out_row <= 7'(n_blk) - 7'sd1;
                    bst <= B_IDLE;
                end
                B_CTXW: begin ctx_mem[b_col] <= {w1, w0}; bst <= B_IDLE; end
                default: bst <= B_IDLE;
            endcase
            if (done_pend && !a_act && b_idle && !b_trig && !(band_start && !busy)) begin done_pend <= 1'b0; band_done <= 1'b1; end
        end
    end
endmodule
