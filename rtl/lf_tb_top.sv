// Test wrapper for the deblocking filter: lf_top + mi_store + frame_mem, with host ports to preload the
// pre-filter picture and the per-4x4 state and to read the result back.
module lf_tb_top
  import syn_pkg::*;
#(
    parameter int FBX = 10,
    parameter int FBY = 9
) (
    input  logic        clk,
    input  logic        rst,
    input  hdr_t        hdr,
    input  lf_hdr_t     lh,
    input  logic        start,
    output logic        busy,
    output logic        done,
    // preload: frame buffer write (only while idle) and per-4x4 state
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
    input  logic        tx_we,
    input  logic [1:0]  tx_plane,
    input  logic [10:0] tx_row, tx_col,
    input  logic [4:0]  tx_w4, tx_h4,
    input  logic [4:0]  tx_sz,
    output logic        tx_busy
);
    logic [10:0] rd_row, rd_col, txr_row, txr_col; mi_lf_t rd_info; logic [1:0] txr_plane; logic [4:0] txr_sz;
    mi_store u_mi (.clk, .rst, .blk_we, .blk_r, .blk_c, .blk_bw4, .blk_bh4, .blk_data, .blk_busy,
                   .tx_we, .tx_plane, .tx_row, .tx_col, .tx_w4, .tx_h4, .tx_sz, .tx_busy,
                   .rd_row, .rd_col, .rd_info, .txr_plane, .txr_row, .txr_col, .txr_sz,
                   .cd_clr(1'b0), .cd_we(1'b0), .cd_row64(7'd0), .cd_col64(7'd0), .cd_sb128(1'b0), .cd_mask(4'd0), .cd_idx(3'd0),
                   .cdr_row64(7'd0), .cdr_col64(7'd0), .cdr_val());
    logic fb_re, fb_we; logic [1:0] fb_plane; logic [FBX-1:0] fb_x; logic [FBY-1:0] fb_y; logic [11:0] fb_wdata, fb_rdata;
    lf_top #(.FBX(FBX), .FBY(FBY)) u_lf (.clk, .rst, .hdr, .lh, .start, .busy, .done,
                                         .rd_row, .rd_col, .rd_info, .txr_plane, .txr_row, .txr_col, .txr_sz,
                                         .fb_re, .fb_we, .fb_plane, .fb_x, .fb_y, .fb_wdata, .fb_rdata);
    // frame buffer: the filter owns the port while busy, the host otherwise
    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12)) u_fb (.clk, .re(fb_re), .we(busy ? fb_we : h_we), .plane(busy ? fb_plane : h_plane),
                                                     .x(busy ? fb_x : h_x), .y(busy ? fb_y : h_y), .wdata(busy ? fb_wdata : h_wdata), .rdata(fb_rdata),
                                                     .h_plane, .h_x, .h_y, .h_rdata);
endmodule
