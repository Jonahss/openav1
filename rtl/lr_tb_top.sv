// Test wrapper for loop restoration: lr_top + mi_store + three frame buffers (0 = deblocked, 1 = CdefFrame,
// 2 = LrFrame), with host ports to preload the two input pictures and the unit records, and to read the result.
module lr_tb_top
  import syn_pkg::*;
#(
    parameter int FBX = 10,
    parameter int FBY = 9
) (
    input  logic        clk,
    input  logic        rst,
    input  hdr_t        hdr,
    input  logic        start,
    output logic        busy,
    output logic        done,
    input  logic        h_we,                 // preload (while idle): h_buf 0 = deblocked, 1 = CdefFrame
    input  logic        h_buf,
    input  logic [1:0]  h_plane,
    input  logic [FBX-1:0] h_x,
    input  logic [FBY-1:0] h_y,
    input  logic [11:0] h_wdata,
    output logic [11:0] h_rdata,              // LrFrame read
    input  logic        lr_we,
    input  lr_rec_t     lr_rec
);
    logic [1:0] lrr_plane; logic [5:0] lrr_row, lrr_col; lr_rec_t lrr_rec;
    mi_lf_t rd_info_u; logic [4:0] txr_sz_u; logic [3:0] cdr_val_u;
    mi_store u_mi (.clk, .rst, .blk_we(1'b0), .blk_r(11'd0), .blk_c(11'd0), .blk_bw4(6'd0), .blk_bh4(6'd0), .blk_data('0), .blk_busy(),
                   .tx_we(1'b0), .tx_plane(2'd0), .tx_row(11'd0), .tx_col(11'd0), .tx_w4(5'd0), .tx_h4(5'd0), .tx_sz(5'd0), .tx_busy(),
                   .rd_row(11'd0), .rd_col(11'd0), .rd_info(rd_info_u), .txr_plane(2'd0), .txr_row(11'd0), .txr_col(11'd0), .txr_sz(txr_sz_u),
                   .cd_clr(1'b0), .cd_we(1'b0), .cd_row64(7'd0), .cd_col64(7'd0), .cd_sb128(1'b0), .cd_mask(4'd0), .cd_idx(3'd0),
                   .cdr_row64(7'd0), .cdr_col64(7'd0), .cdr_val(cdr_val_u),
                   .lr_we, .lr_rec, .lrr_plane, .lrr_row, .lrr_col, .lrr_rec);
    logic s0_re, s1_re, d_we; logic [1:0] s_plane, d_plane; logic [FBX-1:0] s_x, d_x; logic [FBY-1:0] s_y, d_y; logic [11:0] s0_rdata, s1_rdata, d_wdata;
    lr_top #(.FBX(FBX), .FBY(FBY)) u_lr (.clk, .rst, .hdr, .start, .busy, .done, .lrr_plane, .lrr_row, .lrr_col, .lrr_rec,
                                         .s0_re, .s1_re, .s_plane, .s_x, .s_y, .s0_rdata, .s1_rdata, .d_we, .d_plane, .d_x, .d_y, .d_wdata);
    logic [11:0] u0, u1, u2;
    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12)) u_fb0 (.clk, .w2_we(1'b0), .w2_plane(2'd0), .w2_x('0), .w2_y('0), .w2_wdata('0), .re(busy ? s0_re : 1'b0), .we(busy ? 1'b0 : (h_we && !h_buf)), .plane(busy ? s_plane : h_plane),
                                                      .x(busy ? s_x : h_x), .y(busy ? s_y : h_y), .wdata(h_wdata), .rdata(s0_rdata),
                                                      .h_plane(2'd0), .h_x('0), .h_y('0), .h_rdata(u0));
    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12)) u_fb1 (.clk, .w2_we(1'b0), .w2_plane(2'd0), .w2_x('0), .w2_y('0), .w2_wdata('0), .re(busy ? s1_re : 1'b0), .we(busy ? 1'b0 : (h_we && h_buf)), .plane(busy ? s_plane : h_plane),
                                                      .x(busy ? s_x : h_x), .y(busy ? s_y : h_y), .wdata(h_wdata), .rdata(s1_rdata),
                                                      .h_plane(2'd0), .h_x('0), .h_y('0), .h_rdata(u1));
    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12)) u_fb2 (.clk, .w2_we(1'b0), .w2_plane(2'd0), .w2_x('0), .w2_y('0), .w2_wdata('0), .re(1'b0), .we(d_we), .plane(d_plane), .x(d_x), .y(d_y), .wdata(d_wdata), .rdata(u2),
                                                      .h_plane, .h_x, .h_y, .h_rdata);
endmodule
