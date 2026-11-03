// Motion field band (spec 7.9 MotionFieldMvs[ref][y8][x8]): the projected motion vectors the temporal candidates of
// the MV stack read, one entry per 8x8 luma unit and reference frame (LAST_FRAME .. ALTREF_FRAME = 0 .. 6 here).
// Rows are indexed modulo 2^MFL2R: a superblock row's band (8 rows per 64 pixels; 16 for 128x128 superblocks),
// since the projection never leaves its 64-pixel row band and the temporal scan never leaves the superblock.
// One memory per reference, so a write can touch every reference of a position at once: the projection engine
// clears the field at one position a cycle and stores a cell's seven destinations in one cycle (we[r], w_row / w_col
// at [16r +: 16]); the host's register path writes one reference (we = 1 << ref). Read one position per cycle
// (registered, 1-cycle latency). An entry is {valid, row, col}: valid = 0 is the spec's MV_INVALID hole.
module mf_band
  import syn_pkg::*;
#(
    parameter int MFL2R = 4,
    parameter int MFL2C = 9
) (
    input  logic        clk,
    input  logic [6:0]  we,
    input  logic [MIW-2:0] w_y8, w_x8,
    input  logic        w_valid,
    input  logic [111:0] w_row, w_col,
    input  logic [2:0]  rd_ref,
    input  logic [MIW-2:0] rd_y8, rd_x8,
    output logic        rd_valid,
    output logic signed [15:0] rd_row, rd_col
);
    localparam int AW = MFL2R + MFL2C;
    logic [AW-1:0] wa, ra;
    logic [32:0]   rdata [7];
    logic [2:0]    rd_ref_q;
    assign wa = {w_y8[MFL2R-1:0], w_x8[MFL2C-1:0]};
    assign ra = {rd_y8[MFL2R-1:0], rd_x8[MFL2C-1:0]};
    for (genvar r = 0; r < 7; r++) begin : g_ref
        logic [32:0] mem [0:(1 << AW) - 1];
        always_ff @(posedge clk) begin
            if (we[r]) mem[wa] <= {w_valid, w_row[16 * r +: 16], w_col[16 * r +: 16]};
            rdata[r] <= mem[ra];
        end
    end
    always_ff @(posedge clk) rd_ref_q <= rd_ref;
    logic [32:0] rsel;
    assign rsel = (rd_ref_q == 3'd7) ? 33'd0 : rdata[rd_ref_q];
    assign rd_valid = rsel[32];
    assign rd_row = rsel[31:16];
    assign rd_col = rsel[15:0];
endmodule
