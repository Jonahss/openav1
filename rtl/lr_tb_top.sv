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
    mi_store u_mi (.clk, .rst, .rd2_row(11'd0), .rd2_col(11'd0), .rd2_info(), .blk_we(1'b0), .blk_r(11'd0), .blk_c(11'd0), .blk_bw4(6'd0), .blk_bh4(6'd0), .blk_data('0), .blk_busy(),
                   .tx_we(1'b0), .tx_plane(2'd0), .tx_row(11'd0), .tx_col(11'd0), .tx_w4(5'd0), .tx_h4(5'd0), .tx_sz(5'd0), .tx_busy(),
                   .rd_row(11'd0), .rd_col(11'd0), .rd_info(rd_info_u), .txr_plane(2'd0), .txr_row(11'd0), .txr_col(11'd0), .txr_sz(txr_sz_u),
                   .cd_clr(1'b0), .cd_we(1'b0), .cd_row64(7'd0), .cd_col64(7'd0), .cd_sb128(1'b0), .cd_mask(4'd0), .cd_idx(3'd0),
                   .cdr_row64(7'd0), .cdr_col64(7'd0), .cdr_val(cdr_val_u),
                   .lr_we, .lr_rec, .lrr_plane, .lrr_row, .lrr_col, .lrr_rec);
    logic s0_re, s1_re, d_we; logic [1:0] s_plane, d_plane; logic [FBX-1:0] s_x, d_x; logic [FBY-1:0] s_y, d_y; logic [11:0] s0_rdata, s1_rdata, d_wdata;
    lr_top #(.FBX(FBX), .FBY(FBY)) u_lr (.clk, .rst, .hdr, .start, .ly_first(13'd0), .ly_last(hdr.frame_height - 13'd1), .busy, .done, .lrr_plane, .lrr_row, .lrr_col, .lrr_rec,
                                         .s0_re, .s1_re, .s_plane, .s_x, .s_y, .s0_rdata, .s1_rdata, .s0_rdata4(u0_4), .s1_rdata4(u1_4), .d4_we, .d4_be, .d4_wdata, .d_we, .d_plane, .d_x, .d_y, .d_wdata);
    logic [11:0] u0, u1, u2;
    logic w_h0_we; logic [3:0] w_h0_be; logic [1:0] w_h0_pl; logic [FBX-1:0] w_h0_x; logic [FBY-1:0] w_h0_y; logic [47:0] w_h0_d;
    frame_mem_w #(.FBX(FBX), .FBY(FBY)) u_w_h0 (.we1(!busy && h_we && !h_buf), .plane1(h_plane), .x1(h_x), .y1(h_y), .d1(h_wdata),
                                                .we4(1'b0), .plane4(2'd0), .x4('0), .y4('0), .d4('0),
                                                .we(w_h0_we), .be(w_h0_be), .plane(w_h0_pl), .x(w_h0_x), .y(w_h0_y), .data(w_h0_d));
    logic [47:0] u0_4, u1_4, u2_4, d4_wdata; logic d4_we; logic [3:0] d4_be;
    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12), .NRD(1), .NWR(1)) u_fb0 (.clk, .re(busy && s0_re), .r_plane(s_plane), .r_x(s_x), .r_y(s_y), .rdata(s0_rdata), .rdata4(u0_4),
                                                                        .we(w_h0_we), .w_be(w_h0_be), .w_plane(w_h0_pl), .w_x(w_h0_x), .w_y(w_h0_y), .w_data(w_h0_d),
                                                                        .h_plane(2'd0), .h_x('0), .h_y('0), .h_rdata(u0));
    logic w_h1_we; logic [3:0] w_h1_be; logic [1:0] w_h1_pl; logic [FBX-1:0] w_h1_x; logic [FBY-1:0] w_h1_y; logic [47:0] w_h1_d;
    frame_mem_w #(.FBX(FBX), .FBY(FBY)) u_w_h1 (.we1(!busy && h_we && h_buf), .plane1(h_plane), .x1(h_x), .y1(h_y), .d1(h_wdata),
                                                .we4(1'b0), .plane4(2'd0), .x4('0), .y4('0), .d4('0),
                                                .we(w_h1_we), .be(w_h1_be), .plane(w_h1_pl), .x(w_h1_x), .y(w_h1_y), .data(w_h1_d));
    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12), .NRD(1), .NWR(1)) u_fb1 (.clk, .re(busy && s1_re), .r_plane(s_plane), .r_x(s_x), .r_y(s_y), .rdata(s1_rdata), .rdata4(u1_4),
                                                                        .we(w_h1_we), .w_be(w_h1_be), .w_plane(w_h1_pl), .w_x(w_h1_x), .w_y(w_h1_y), .w_data(w_h1_d),
                                                                        .h_plane(2'd0), .h_x('0), .h_y('0), .h_rdata(u1));
    logic w_d_we; logic [3:0] w_d_be; logic [1:0] w_d_pl; logic [FBX-1:0] w_d_x; logic [FBY-1:0] w_d_y; logic [47:0] w_d_d;
    assign w_d_we = d4_we; assign w_d_be = d4_be; assign w_d_pl = d_plane; assign w_d_x = d_x; assign w_d_y = d_y; assign w_d_d = d4_wdata;
    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12), .NRD(1), .NWR(1)) u_fb2 (.clk, .re(1'b0), .r_plane(2'd0), .r_x('0), .r_y('0), .rdata(u2), .rdata4(u2_4),
                                                                        .we(w_d_we), .w_be(w_d_be), .w_plane(w_d_pl), .w_x(w_d_x), .w_y(w_d_y), .w_data(w_d_d),
                                                                        .h_plane, .h_x, .h_y, .h_rdata);
endmodule
