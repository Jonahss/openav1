// Test wrapper: lr_stream with its boundary line store and the converter in front of it. The test sends the deblock
// stage's blocks (wr_*) instead of answering the boundary port.
module lr_stage_tb_top
  import syn_pkg::*;
  import lr_pkg::*;
#(
    parameter int CW = 10
) (
    input  logic            clk,
    input  logic            rst,
    input  logic [3:0]      bit_depth,
    input  logic            en,
    input  logic [1:0]      plane,
    input  logic            ssx, ssy,
    input  logic [MIW+1:0]  pl_w, pl_h,
    input  logic [3:0]      u_log2,
    input  logic [SBW-1:0]  u_rows, u_cols,
    input  logic            band_start,
    input  logic [MIW-1:0]  row0,
    input  logic [MIW-1:0]  pic_rows,
    input  logic [5:0]      n_blk,
    input  logic [CW:0]     n_col,
    input  logic            first_band,
    input  logic            last_band,
    output logic            busy,
    output logic            band_done,
    input  logic            in_valid,
    output logic            in_ready,
    input  logic [95:0]     in_line,
    input  logic signed [8:0] in_y,
    output logic [1:0]      lrr_plane,
    output logic [SBW-1:0]  lrr_row, lrr_col,
    input  lr_rec_t         lrr_rec,
    // the deblock stage's blocks, through lr_bconv: its own band (the test also runs bands of noise through it)
    input  logic            wr_start,
    input  logic [MIW-1:0]  wr_row0,
    input  logic [5:0]      wr_n_blk,
    input  logic            wr_last_band,
    input  logic            wr_valid,
    output logic            wr_ready,
    input  logic [191:0]    wr_blk,
    input  logic [CW-1:0]   wr_col,
    input  logic signed [6:0] wr_row,
    input  logic            out_hold,
    output logic            out_valid,
    output logic [95:0]     out_line,
    output logic [1:0]      out_n4,
    output logic [CW-1:0]   out_col,
    output logic signed [8:0] out_y
);
    logic [SBW:0]   bl_k;
    logic [CW-1:0]  bl_col;
    logic [383:0]   bl_rd;
    logic           bc_v, bc_last;
    logic [95:0]    bc_line;
    logic [8:0]     bc_tag;
    lr_stream #(.CW(CW)) u_lr (.*);
    lr_bconv #(.CW(CW)) u_bc (.clk, .rst, .ssx, .ssy, .band_start(wr_start), .row0(wr_row0), .n_blk(wr_n_blk), .last_band(wr_last_band),
                              .in_valid(wr_valid), .in_ready(wr_ready), .in_blk(wr_blk), .in_col(wr_col), .in_row(wr_row),
                              .out_valid(bc_v), .out_ready(1'b1), .out_line(bc_line), .out_tag(bc_tag), .out_last(bc_last));
    lr_bstore #(.CW(CW)) u_bs (.clk, .rst, .ssx, .band_start(wr_start), .wr_valid(bc_v), .wr_line(bc_line), .wr_tag(bc_tag), .wr_last(bc_last),
                               .bl_k, .bl_col, .bl_rd);
endmodule
