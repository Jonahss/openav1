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
    mi_store u_mi (.clk, .rst, .rd2_row(11'd0), .rd2_col(11'd0), .rd2_info(), .blk_we, .blk_r, .blk_c, .blk_bw4, .blk_bh4, .blk_data, .blk_busy,
                   .tx_we, .tx_plane, .tx_row, .tx_col, .tx_w4, .tx_h4, .tx_sz, .tx_busy,
                   .rd_row, .rd_col, .rd_info, .txr_plane, .txr_row, .txr_col, .txr_sz,
                   .cd_clr(1'b0), .cd_we(1'b0), .cd_row64(7'd0), .cd_col64(7'd0), .cd_sb128(1'b0), .cd_mask(4'd0), .cd_idx(3'd0),
                   .cdr_row64(7'd0), .cdr_col64(7'd0), .cdr_val(),
                   .lr_we(1'b0), .lr_rec('0), .lrr_plane(2'd0), .lrr_row(6'd0), .lrr_col(6'd0), .lrr_rec());
    logic fb_re, fb_we; logic [1:0] fb_plane; logic [FBX-1:0] fb_x; logic [FBY-1:0] fb_y; logic [11:0] fb_wdata, fb_rdata; logic [47:0] fb_rdata4, fb4_wdata; logic fb4_we; logic [FBX-1:0] fb4_x; logic [FBY-1:0] fb4_y;
    lf_top #(.FBX(FBX), .FBY(FBY)) u_lf (.clk, .rst, .hdr, .lh, .start, .row_first(11'd0), .row_last(hdr.mi_rows - 11'd1), .busy, .done,
                                         .rd_row, .rd_col, .rd_info, .txr_plane, .txr_row, .txr_col, .txr_sz,
                                         .fb_re, .fb_we, .fb_plane, .fb_x, .fb_y, .fb_wdata, .fb_rdata,
                                         .fb_rdata4, .fb4_we, .fb4_x, .fb4_y, .fb4_wdata);
    // frame buffer: the filter owns the port while busy, the host otherwise
    logic w_f_we, w_h_we; logic [3:0] w_f_be, w_h_be; logic [1:0] w_f_pl, w_h_pl; logic [FBX-1:0] w_f_x, w_h_x; logic [FBY-1:0] w_f_y, w_h_y; logic [47:0] w_f_d, w_h_d;
    frame_mem_w #(.FBX(FBX), .FBY(FBY)) u_w_f (.we1(busy && fb_we), .plane1(fb_plane), .x1(fb_x), .y1(fb_y), .d1(fb_wdata),
                                               .we4(busy && fb4_we), .plane4(fb_plane), .x4(fb4_x), .y4(fb4_y), .d4(fb4_wdata),
                                               .we(w_f_we), .be(w_f_be), .plane(w_f_pl), .x(w_f_x), .y(w_f_y), .data(w_f_d));
    frame_mem_w #(.FBX(FBX), .FBY(FBY)) u_w_h (.we1(!busy && h_we), .plane1(h_plane), .x1(h_x), .y1(h_y), .d1(h_wdata),
                                               .we4(1'b0), .plane4(2'd0), .x4('0), .y4('0), .d4('0),
                                               .we(w_h_we), .be(w_h_be), .plane(w_h_pl), .x(w_h_x), .y(w_h_y), .data(w_h_d));
    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12), .NRD(1), .NWR(2)) u_fb (.clk, .re(fb_re), .r_plane(fb_plane), .r_x(fb_x), .r_y(fb_y), .rdata(fb_rdata), .rdata4(fb_rdata4),
                                                                       .we({w_h_we, w_f_we}), .w_be({w_h_be, w_f_be}), .w_plane({w_h_pl, w_f_pl}), .w_x({w_h_x, w_f_x}), .w_y({w_h_y, w_f_y}), .w_data({w_h_d, w_f_d}),
                                                                       .h_plane, .h_x, .h_y, .h_rdata);
endmodule
