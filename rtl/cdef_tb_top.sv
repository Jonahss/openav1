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
    mi_store u_mi (.clk, .rst, .rd2_row(11'd0), .rd2_col(11'd0), .rd2_info(), .blk_we, .blk_r, .blk_c, .blk_bw4, .blk_bh4, .blk_data, .blk_busy,
                   .tx_we(1'b0), .tx_plane(2'd0), .tx_row(11'd0), .tx_col(11'd0), .tx_w4(5'd0), .tx_h4(5'd0), .tx_sz(5'd0), .tx_busy(),
                   .rd_row, .rd_col, .rd_info, .txr_plane(2'd0), .txr_row(11'd0), .txr_col(11'd0), .txr_sz,
                   .cd_clr, .cd_we, .cd_row64, .cd_col64, .cd_sb128, .cd_mask, .cd_idx, .cdr_row64, .cdr_col64, .cdr_val,
                   .lr_we(1'b0), .lr_rec('0), .lrr_plane(2'd0), .lrr_row(6'd0), .lrr_col(6'd0), .lrr_rec());
    logic src_re, dst_we; logic [1:0] src_plane, dst_plane; logic [FBX-1:0] src_x, dst_x; logic [FBY-1:0] src_y, dst_y; logic [11:0] src_rdata, dst_wdata; logic [47:0] src_rdata4, dst4_wdata; logic dst4_we; logic [1:0] dst4_plane; logic [FBX-1:0] dst4_x; logic [FBY-1:0] dst4_y;
    cdef_top #(.FBX(FBX), .FBY(FBY)) u_cdef (.clk, .rst, .hdr, .ch, .start, .row_first(11'd0), .row_last(hdr.mi_rows - 11'd1), .busy, .done,
                                             .rd_row, .rd_col, .rd_info, .cdr_row64, .cdr_col64, .cdr_val,
                                             .src_re, .src_plane, .src_x, .src_y, .src_rdata, .src_rdata4, .dst_we, .dst_plane, .dst_x, .dst_y, .dst_wdata,
                                             .dst4_we, .dst4_plane, .dst4_x, .dst4_y, .dst4_wdata);
    logic [11:0] unused_h0;
    logic w_h_we; logic [3:0] w_h_be; logic [1:0] w_h_pl; logic [FBX-1:0] w_h_x; logic [FBY-1:0] w_h_y; logic [47:0] w_h_d;
    frame_mem_w #(.FBX(FBX), .FBY(FBY)) u_w_h (.we1(!busy && h_we), .plane1(h_plane), .x1(h_x), .y1(h_y), .d1(h_wdata),
                                               .we4(1'b0), .plane4(2'd0), .x4('0), .y4('0), .d4('0),
                                               .we(w_h_we), .be(w_h_be), .plane(w_h_pl), .x(w_h_x), .y(w_h_y), .data(w_h_d));
    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12), .NRD(1), .NWR(1)) u_src (.clk, .re(busy && src_re), .r_plane(src_plane), .r_x(src_x), .r_y(src_y), .rdata(src_rdata), .rdata4(src_rdata4),
                                                                        .we(w_h_we), .w_be(w_h_be), .w_plane(w_h_pl), .w_x(w_h_x), .w_y(w_h_y), .w_data(w_h_d),
                                                                        .h_plane(2'd0), .h_x('0), .h_y('0), .h_rdata(unused_h0));
    logic [11:0] unused_r1;
    logic w_d_we; logic [3:0] w_d_be; logic [1:0] w_d_pl; logic [FBX-1:0] w_d_x; logic [FBY-1:0] w_d_y; logic [47:0] w_d_d; logic [47:0] unused_r4;
    frame_mem_w #(.FBX(FBX), .FBY(FBY)) u_w_d (.we1(dst_we), .plane1(dst_plane), .x1(dst_x), .y1(dst_y), .d1(dst_wdata),
                                               .we4(dst4_we), .plane4(dst4_plane), .x4(dst4_x), .y4(dst4_y), .d4(dst4_wdata),
                                               .we(w_d_we), .be(w_d_be), .plane(w_d_pl), .x(w_d_x), .y(w_d_y), .data(w_d_d));
    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12), .NRD(1), .NWR(1)) u_dst (.clk, .re(1'b0), .r_plane(2'd0), .r_x('0), .r_y('0), .rdata(unused_r1), .rdata4(unused_r4),
                                                                        .we(w_d_we), .w_be(w_d_be), .w_plane(w_d_pl), .w_x(w_d_x), .w_y(w_d_y), .w_data(w_d_d),
                                                                        .h_plane, .h_x, .h_y, .h_rdata);
endmodule
