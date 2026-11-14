// Boundary lines of the streaming filter chain's restoration stage (docs/filter-chain.md), one plane: picks the lines
// around the stripe boundaries out of the deblock stage's block stream and sends them on as line segments, the form
// the upscaler (sr_stream) and the boundary line store (lr_bstore) take.
// Restoration reads across a stripe boundary in the deblocked picture (7.17.6): the two lines above a stripe's first
// line B and the two from B on. With B4 = B / 4 (B = 64 k - 8 in luma, 32 k - 4 in a vertically subsampled plane):
// block row B4 - 1 holds lines B - 2 and B - 1 (its lines 2 and 3), block row B4 holds B and B + 1.
//   In: lf_stream's finished 4x4 blocks with their column and row (counted from the band's first block row), column
//   by column. in_ready falls for the two clocks a boundary block's lines take to leave.
//   Out: for every segment column (two 4x4 columns, or one in a horizontally subsampled plane) the boundary lines of
//   its blocks in arrival order, 8 (4) samples a line, out_last on the column's last one,
//   out_tag = {boundary k modulo 4, line 0 .. 3 (B - 2, B - 1, B, B + 1)}.
// Every column of a band sends the same lines; a band without a boundary row sends nothing.
module lr_bconv
  import syn_pkg::*;
#(
    parameter int CW = 10                    // log2 of the most 4x4 columns
) (
    input  logic            clk,
    input  logic            rst,
    input  logic            ssx, ssy,
    input  logic            band_start,
    input  logic [MIW-1:0]  row0,            // the band's first 4x4 row in the plane
    input  logic [5:0]      n_blk,           // the plane's 4x4 rows in the band
    input  logic            last_band,
    input  logic            in_valid,        // a block of the deblock stage is being taken by the stage behind it ...
    output logic            in_ready,        // ... which may only take it while this is high
    input  logic [191:0]    in_blk,
    input  logic [CW-1:0]   in_col,
    input  logic signed [6:0] in_row,
    output logic            out_valid,
    input  logic            out_ready,
    output logic [95:0]     out_line,
    output logic [8:0]      out_tag,
    output logic            out_last
);
    logic [MIW:0]   ra, m_bot, m_top, r_end;
    logic           is_bot, is_top, last_b;
    logic [1:0]     slot;
    logic [4:0]     per;
    always_comb begin
        ra = (MIW+1)'(signed'({1'b0, row0}) + (MIW+1)'(in_row));
        m_bot = ra + (ssy ? (MIW+1)'(1) : (MIW+1)'(2));          // 4 (ra + 2) = 64 k - 8 + 16: block row B4 of boundary k
        m_top = m_bot + (MIW+1)'(1);
        is_bot = ssy ? (m_bot[2:0] == 3'd0) : (m_bot[3:0] == 4'd0);
        is_top = ssy ? (m_top[2:0] == 3'd0) : (m_top[3:0] == 4'd0);
        slot = is_bot ? (ssy ? m_bot[4:3] : m_bot[5:4]) : (ssy ? m_top[4:3] : m_top[5:4]);
        per = ssy ? 5'd8 : 5'd16;
        // the column's last block row; the boundary row after this one is the next row (after a top) or per - 1 on
        r_end = (MIW+1)'(signed'({1'b0, row0})) + (last_band ? (MIW+1)'(n_blk) - (MIW+1)'(1) : (MIW+1)'(n_blk) - (MIW+1)'(3));
        last_b = is_top ? (ra + (MIW+1)'(1) > r_end) : (ra + (MIW+1)'(per) - (MIW+1)'(1) > r_end);
    end
    // the first half of a segment (an even 4x4 column of a plane that is not subsampled): its boundary blocks' lines, kept
    logic [95:0] ev [8];                     // a column of 34 block rows (a 128-line band with its context rows) has at most six
    logic [2:0]  e_n;                        // boundary blocks seen in the column
    logic        pair;                       // the column is a segment's second half
    assign pair = !ssx && in_col[0];
    // two lines on their way out
    logic [1:0]  s_n;
    logic [95:0] s_l [2];
    logic [8:0]  s_t [2];
    logic        s_last;
    logic        take;
    assign in_ready = (s_n == 2'd0);
    assign take = in_valid && in_ready && (is_bot || is_top);
    assign out_valid = (s_n != 2'd0);
    assign out_line = s_l[0]; assign out_tag = s_t[0]; assign out_last = s_last && (s_n == 2'd1);
    always_ff @(posedge clk) begin
        if (rst || band_start) begin e_n <= '0; s_n <= 2'd0; end
        else begin
            if (in_valid && in_ready && in_row == (last_band ? 7'(n_blk) - 7'sd1 : 7'(n_blk) - 7'sd3)) e_n <= '0;   // the column ends
            else if (take) e_n <= e_n + 3'd1;
            if (take) begin
                if (!ssx && !in_col[0]) ev[e_n] <= is_top ? in_blk[191:96] : in_blk[95:0];
                else begin
                    logic [95:0] a, b;                           // this block's two lines, the kept half's two lines
                    a = is_top ? in_blk[191:96] : in_blk[95:0];
                    b = ev[e_n];
                    s_l[0] <= pair ? {a[47:0], b[47:0]} : {48'd0, a[47:0]};
                    s_l[1] <= pair ? {a[95:48], b[95:48]} : {48'd0, a[95:48]};
                    s_t[0] <= {5'd0, slot, is_top ? 2'd0 : 2'd2};
                    s_t[1] <= {5'd0, slot, is_top ? 2'd1 : 2'd3};
                    s_last <= last_b;
                    s_n <= 2'd2;
                end
            end
            if (out_valid && out_ready) begin
                s_n <= s_n - 2'd1; s_l[0] <= s_l[1]; s_t[0] <= s_t[1];
            end
        end
    end
endmodule
