// Boundary line store of the streaming filter chain's restoration stage (docs/filter-chain.md), one plane: the four
// deblocked lines around each stripe boundary (two above the stripe's first line B, two from it on), which
// restoration reads instead of CDEF's output across a stripe boundary (7.17.6).
//   In: the boundary lines as lr_bconv picks them out of the deblock stage's blocks (upscaled by sr_stream when the
//   frame uses super-resolution): line segments column by column, wr_last on a column's last line, the tag
//   {boundary k modulo 4, line 0 .. 3}. The store counts the segment columns from band_start.
//   Read: boundary bl_k at 4x4 column bl_col, the first column of a segment of 8 samples (4 in a horizontally
//   subsampled plane); one clock later the lines B - 2, B - 1, B, B + 1 of the segment, line i at bits i * 96.
// Four boundaries are kept (bl_k modulo 4). In a band of 128 lines restoration reads boundaries k and k + 1 while the
// deblock stage of the same band writes into k, k + 1 and k + 2 (the two lines above k + 2 are final by then), so
// three are alive at once; the deblock stage must not be a band ahead of restoration's reads.
// Lines that were never written (outside the picture) read as whatever the memory holds: lr_stream does not use them.
// Memory: four memories (one a line) of 96 bits by {boundary, segment column}.
module lr_bstore
  import syn_pkg::*;
#(
    parameter int CW = 10                    // log2 of the most segment columns / 4x4 columns
) (
    input  logic            clk,
    input  logic            rst,
    input  logic            ssx,             // segments of one 4x4 column (else two)
    input  logic            band_start,
    input  logic            wr_valid,
    input  logic [95:0]     wr_line,
    input  logic [8:0]      wr_tag,
    input  logic            wr_last,
    input  logic [SBW:0]    bl_k,
    input  logic [CW-1:0]   bl_col,
    output logic [383:0]    bl_rd
);
    logic [CW-1:0] w_seg, r_seg;
    assign r_seg = ssx ? bl_col : {1'b0, bl_col[CW-1:1]};
    always_ff @(posedge clk) begin
        if (rst || band_start) w_seg <= '0;
        else if (wr_valid && wr_last) w_seg <= w_seg + CW'(1);
    end
    generate for (genvar l = 0; l < 4; l++) begin : g_l
        (* ram_style = "block" *) logic [95:0] mem [1 << (CW + 2)];
        logic [95:0] q;
        always_ff @(posedge clk) begin
            if (wr_valid && wr_tag[1:0] == 2'(l)) mem[{wr_tag[3:2], w_seg}] <= wr_line;
            q <= mem[{bl_k[1:0], r_seg}];
        end
        assign bl_rd[96 * l +: 96] = q;
    end endgenerate
endmodule
