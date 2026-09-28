// Test wrapper for CDEF: cdef_top + mi_store + two frame buffers (src = deblocked, dst = CdefFrame), with host
// ports to preload the deblocked picture and the per-4x4 / per-64x64 state and to read the result back.
module cdef_tb_top
  import syn_pkg::*;
#(
    parameter int FBX = 10,
    parameter int FBY = 9
) (
    input  logic        clk,
    input  logic        rst,
    input  hdr_t        hdr,
    input  cdef_hdr_t   ch,
    input  logic        start,
    output logic        busy,
    output logic        done,
    // preload: src frame write (while idle), skips (block write), cdef_idx per unit; result read from dst
    input  logic        h_we,
    input  logic [1:0]  h_plane,
    input  logic [FBX-1:0] h_x,
    input  logic [FBY-1:0] h_y,
    input  logic [11:0] h_wdata,
    output logic [11:0] h_rdata,
    input  logic        blk_we,
    input  logic [10:0] blk_r, blk_c,
    input  logic [5:0]  blk_bw4, blk_bh4,
    input  mi_lf_t      blk_data,
    output logic        blk_busy,
    input  logic        cd_clr,
    input  logic        cd_we,
    input  logic [6:0]  cd_row64, cd_col64,
    input  logic        cd_sb128,
    input  logic [3:0]  cd_mask,
    input  logic [2:0]  cd_idx
);
    logic [10:0] rd_row, rd_col; mi_lf_t rd_info; logic [6:0] cdr_row64, cdr_col64; logic [3:0] cdr_val;
    logic [4:0] txr_sz;
    mi_store u_mi (.clk, .rst, .blk_we, .blk_r, .blk_c, .blk_bw4, .blk_bh4, .blk_data, .blk_busy,
                   .tx_we(1'b0), .tx_plane(2'd0), .tx_row(11'd0), .tx_col(11'd0), .tx_w4(5'd0), .tx_h4(5'd0), .tx_sz(5'd0), .tx_busy(),
                   .rd_row, .rd_col, .rd_info, .txr_plane(2'd0), .txr_row(11'd0), .txr_col(11'd0), .txr_sz,
                   .cd_clr, .cd_we, .cd_row64, .cd_col64, .cd_sb128, .cd_mask, .cd_idx, .cdr_row64, .cdr_col64, .cdr_val,
                   .lr_we(1'b0), .lr_rec('0), .lrr_plane(2'd0), .lrr_row(6'd0), .lrr_col(6'd0), .lrr_rec());
    logic src_re, dst_we; logic [1:0] src_plane, dst_plane; logic [FBX-1:0] src_x, dst_x; logic [FBY-1:0] src_y, dst_y; logic [11:0] src_rdata, dst_wdata;
    cdef_top #(.FBX(FBX), .FBY(FBY)) u_cdef (.clk, .rst, .hdr, .ch, .start, .busy, .done,
                                             .rd_row, .rd_col, .rd_info, .cdr_row64, .cdr_col64, .cdr_val,
                                             .src_re, .src_plane, .src_x, .src_y, .src_rdata, .dst_we, .dst_plane, .dst_x, .dst_y, .dst_wdata);
    logic [11:0] unused_h0;
    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12)) u_src (.clk, .re(busy ? src_re : 1'b0), .we(busy ? 1'b0 : h_we), .plane(busy ? src_plane : h_plane),
                                                      .x(busy ? src_x : h_x), .y(busy ? src_y : h_y), .wdata(h_wdata), .rdata(src_rdata),
                                                      .h_plane(2'd0), .h_x('0), .h_y('0), .h_rdata(unused_h0));
    logic [11:0] unused_r1;
    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12)) u_dst (.clk, .re(1'b0), .we(dst_we), .plane(dst_plane), .x(dst_x), .y(dst_y), .wdata(dst_wdata), .rdata(unused_r1),
                                                      .h_plane, .h_x, .h_y, .h_rdata);
endmodule
