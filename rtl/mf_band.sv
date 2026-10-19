// Motion field band (spec 7.9 MotionFieldMvs[ref][y8][x8]): the projected motion vectors the temporal candidates of
// the MV stack read, one entry per 8x8 luma unit and reference frame (LAST_FRAME .. ALTREF_FRAME = 0 .. 6 here).
// Rows are indexed modulo 2^MFL2R: a superblock row's band (8 rows per 64 pixels; 16 for 128x128 superblocks),
// since the projection never leaves its 64-pixel row band and the temporal scan never leaves the superblock.
// Written by the host / the projection engine (we/w_*), read one position per cycle (registered, 1-cycle latency).
// An entry is {valid, row, col}: valid = 0 is the spec's MV_INVALID hole.
module mf_band
  import syn_pkg::*;
#(
    parameter int MFL2R = 4,
    parameter int MFL2C = 9
) (
    input  logic        clk,
    input  logic        we,
    input  logic [2:0]  w_ref,
    input  logic [MIW-2:0] w_y8, w_x8,
    input  logic        w_valid,
    input  logic signed [15:0] w_row, w_col,
    input  logic [2:0]  rd_ref,
    input  logic [MIW-2:0] rd_y8, rd_x8,
    output logic        rd_valid,
    output logic signed [15:0] rd_row, rd_col
);
    localparam int AW = 3 + MFL2R + MFL2C;
    logic [32:0] mem [0:(1 << AW) - 1];
    logic [AW-1:0] wa, ra;
    logic [32:0] rdata;
    assign wa = {w_ref, w_y8[MFL2R-1:0], w_x8[MFL2C-1:0]};
    assign ra = {rd_ref, rd_y8[MFL2R-1:0], rd_x8[MFL2C-1:0]};
    always_ff @(posedge clk) begin
        if (we) mem[wa] <= {w_valid, w_row, w_col};
        rdata <= mem[ra];
    end
    assign rd_valid = rdata[32];
    assign rd_row = rdata[31:16];
    assign rd_col = rdata[15:0];
endmodule
