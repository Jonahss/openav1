// Motion field band (spec 7.9 MotionFieldMvs[ref][y8][x8]): the projected motion vectors the temporal candidates of
// the MV stack read, one entry per 8x8 luma unit and reference frame (LAST_FRAME .. ALTREF_FRAME = 0 .. 6 here).
// Rows are indexed modulo 2^MFL2R: a superblock row's band (8 rows per 64 pixels; 16 for 128x128 superblocks),
// since the projection never leaves its 64-pixel row band and the temporal scan never leaves the superblock.
// A write can touch every reference of a position at once: the projection engine clears the field at one position a
// cycle and stores a cell's seven destinations in one cycle (we[r], w_row / w_col at [16r +: 16]). Read one position
// per cycle (registered, 1-cycle latency, on rd_en: the output holds between reads). An entry is {valid, row, col}:
// valid = 0 is the spec's MV_INVALID hole.
// Storage (2026-11-20, docs/4k60-budget.md Count 1): UltraRAM. Four 72-bit words per position, two references a
// word, each entry {row, col} in four bytes with its own four byte-write enables, so the seven per-reference write
// enables stay (the ninth byte and the eighth half are unused). The hole is a row of 16'h8000: the projection clips
// its vectors to +-16383, so no entry has it. 16 UltraRAMs an engine at the 4K size, where seven block-RAM memories
// of 16384 x 33 took 105 RAMB36. Vivado infers UltraRAM only for whole-word writes (frame_mem_uram.sv found the same),
// so synthesis instantiates URAM288 (port A writes, port B reads; a word deeper than 4096 is NU of them by address
// decode with a registered output select) and simulation uses the behavioural model below it.
// The write port goes through a register (a write lands one clock later): its sources are the host's register file
// and the projection engine, its sinks seven memories in each tile engine (2026-11-17: 96-99 % route on these paths
// in both routed images). The field is written before the tiles of a frame read it; a simulation check holds that.
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
    input  logic        rd_en,
    input  logic [2:0]  rd_ref,
    input  logic [MIW-2:0] rd_y8, rd_x8,
    output logic        rd_valid,
    output logic signed [15:0] rd_row, rd_col
);
    localparam int AW = MFL2R + MFL2C;
    logic [AW-1:0] wa, ra;
    localparam logic [15:0] HOLE = 16'h8000;
    logic [63:0]   rword [4];
    logic [31:0]   rdata [7];
    logic [2:0]    rd_ref_q;
    logic [6:0]    we_r;
    logic          w_valid_r;
    logic [111:0]  w_row_r, w_col_r;
    always_ff @(posedge clk) begin
        we_r <= we; w_valid_r <= w_valid; w_row_r <= w_row; w_col_r <= w_col;
        wa <= {w_y8[MFL2R-1:0], w_x8[MFL2C-1:0]};
    end
    assign ra = {rd_y8[MFL2R-1:0], rd_x8[MFL2C-1:0]};
    for (genvar g = 0; g < 4; g++) begin : g_w
        logic [63:0] wword;
        logic [7:0]  bwe;
        always_comb begin
            wword = '0; bwe = '0;
            for (int h = 0; h < 2; h++) if (2 * g + h < 7) begin
                wword[32 * h +: 32] = {w_valid_r ? w_row_r[16 * (2 * g + h) +: 16] : HOLE, w_col_r[16 * (2 * g + h) +: 16]};
                bwe[4 * h +: 4]     = {4{we_r[2 * g + h]}};
            end
        end
`ifdef SYNTHESIS
        localparam int NU = (AW > 12) ? (1 << (AW - 12)) : 1;    // UltraRAMs a word (4096 x 72 each)
        localparam int SW = (NU > 1) ? AW - 12 : 1;
        logic [71:0]   dout [NU];
        logic [SW-1:0] rsel_q;
        always_ff @(posedge clk) if (rd_en) rsel_q <= (NU > 1) ? SW'(ra >> 12) : '0;
        assign rword[g] = dout[rsel_q][63:0];
        for (genvar k = 0; k < NU; k++) begin : g_uram
            logic w_here, r_here;
            assign w_here = (|bwe) && (((NU > 1) ? SW'(wa >> 12) : '0) == SW'(k));
            assign r_here = rd_en && (((NU > 1) ? SW'(ra >> 12) : '0) == SW'(k));
            URAM288 #(
                .CASCADE_ORDER_A("NONE"), .CASCADE_ORDER_B("NONE"), .OREG_A("FALSE"), .OREG_B("FALSE"), .REG_CAS_A("FALSE"), .REG_CAS_B("FALSE"),
                .BWE_MODE_A("PARITY_INDEPENDENT"), .BWE_MODE_B("PARITY_INDEPENDENT"), .EN_AUTO_SLEEP_MODE("FALSE"),
                .IREG_PRE_A("FALSE"), .IREG_PRE_B("FALSE"), .RST_MODE_A("SYNC"), .RST_MODE_B("SYNC"), .USE_EXT_CE_A("FALSE"), .USE_EXT_CE_B("FALSE")
            ) u (
                .CLK(clk), .SLEEP(1'b0), .RST_A(1'b0), .RST_B(1'b0),
                .EN_A(w_here), .RDB_WR_A(1'b1), .ADDR_A(23'(wa[((AW > 12) ? 11 : AW - 1):0])), .BWE_A({1'b0, bwe}), .DIN_A({8'd0, wword}), .DOUT_A(),
                .EN_B(r_here), .RDB_WR_B(1'b0), .ADDR_B(23'(ra[((AW > 12) ? 11 : AW - 1):0])), .BWE_B(9'd0), .DIN_B(72'd0), .DOUT_B(dout[k]),
                .OREG_CE_A(1'b0), .OREG_CE_B(1'b0), .OREG_ECC_CE_A(1'b0), .OREG_ECC_CE_B(1'b0),
                .INJECT_SBITERR_A(1'b0), .INJECT_DBITERR_A(1'b0), .INJECT_SBITERR_B(1'b0), .INJECT_DBITERR_B(1'b0),
                .CAS_IN_ADDR_A(23'd0), .CAS_IN_ADDR_B(23'd0), .CAS_IN_BWE_A(9'd0), .CAS_IN_BWE_B(9'd0), .CAS_IN_DBITERR_A(1'b0), .CAS_IN_DBITERR_B(1'b0),
                .CAS_IN_DIN_A(72'd0), .CAS_IN_DIN_B(72'd0), .CAS_IN_DOUT_A(72'd0), .CAS_IN_DOUT_B(72'd0), .CAS_IN_EN_A(1'b0), .CAS_IN_EN_B(1'b0),
                .CAS_IN_RDACCESS_A(1'b0), .CAS_IN_RDACCESS_B(1'b0), .CAS_IN_RDB_WR_A(1'b0), .CAS_IN_RDB_WR_B(1'b0), .CAS_IN_SBITERR_A(1'b0), .CAS_IN_SBITERR_B(1'b0),
                .CAS_OUT_ADDR_A(), .CAS_OUT_ADDR_B(), .CAS_OUT_BWE_A(), .CAS_OUT_BWE_B(), .CAS_OUT_DBITERR_A(), .CAS_OUT_DBITERR_B(),
                .CAS_OUT_DIN_A(), .CAS_OUT_DIN_B(), .CAS_OUT_DOUT_A(), .CAS_OUT_DOUT_B(), .CAS_OUT_EN_A(), .CAS_OUT_EN_B(),
                .CAS_OUT_RDACCESS_A(), .CAS_OUT_RDACCESS_B(), .CAS_OUT_RDB_WR_A(), .CAS_OUT_RDB_WR_B(), .CAS_OUT_SBITERR_A(), .CAS_OUT_SBITERR_B(),
                .DBITERR_A(), .DBITERR_B(), .SBITERR_A(), .SBITERR_B(), .RDACCESS_A(), .RDACCESS_B()
            );
        end
`else
        logic [63:0] mem [0:(1 << AW) - 1];
        always_ff @(posedge clk) begin
            for (int b = 0; b < 8; b++) if (bwe[b]) mem[wa][8 * b +: 8] <= wword[8 * b +: 8];
            if (rd_en) rword[g] <= mem[ra];
        end
`endif
    end
    for (genvar r = 0; r < 7; r++) begin : g_ref
        assign rdata[r] = rword[r / 2][32 * (r % 2) +: 32];
`ifndef SYNTHESIS
        always_ff @(posedge clk) begin
            if (rd_en && we_r[r] && rd_ref == 3'(r) && ra == wa) $fatal(1, "mf_band: reference %0d read of an entry still in the write register", r);
            if (we_r[r] && w_valid_r && w_row_r[16 * r +: 16] == HOLE) $fatal(1, "mf_band: reference %0d written with the row value that marks a hole", r);
        end
`endif
    end
    always_ff @(posedge clk) if (rd_en) rd_ref_q <= rd_ref;
    logic [31:0] rsel;
    assign rsel = (rd_ref_q == 3'd7) ? {HOLE, 16'd0} : rdata[rd_ref_q];
    assign rd_valid = (rsel[31:16] != HOLE);
    assign rd_row = rsel[31:16];
    assign rd_col = rsel[15:0];
endmodule
