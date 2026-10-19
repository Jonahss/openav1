// Segmentation map band (spec PrevSegmentIds): the previous frame's segment ids, 3 bits per 4x4 unit, 16 units per
// 48-bit word. Rows are indexed modulo 2^SPL2R (a superblock row's band on the board; the whole frame in simulation).
// Written by the host / the reference fetch engine one word at a time, read one word per cycle (1-cycle latency) by
// blk_syntax's get_segment_id (the minimum over the block's area).
module seg_band
  import syn_pkg::*;
#(
    parameter int SPL2R = 5,
    parameter int SPL2W = 6                  // log2 words per row (16 columns each)
) (
    input  logic        clk,
    input  logic        we,
    input  logic [MIW-1:0] w_row,
    input  logic [MIW-5:0] w_wc,             // word column (col >> 4)
    input  logic [47:0] w_data,
    input  logic [MIW-1:0] rd_row,
    input  logic [MIW-5:0] rd_wc,
    output logic [47:0] rd_data
);
    localparam int AW = SPL2R + SPL2W;
    logic [47:0] mem [0:(1 << AW) - 1];
    logic [AW-1:0] wa, ra;
    assign wa = {w_row[SPL2R-1:0], w_wc[SPL2W-1:0]};
    assign ra = {rd_row[SPL2R-1:0], rd_wc[SPL2W-1:0]};
    always_ff @(posedge clk) begin
        if (we) mem[wa] <= w_data;
        rd_data <= mem[ra];
    end
endmodule
