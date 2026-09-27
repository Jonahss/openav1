// Intra tile decoder top: tile_syntax (entropy decoding + syntax) -> recon_top (prediction, transforms,
// reconstruction) -> frame_mem, plus mi_store + lf_top (deblocking, run over the finished frame on lf_start). Software supplies the parsed headers (hdr_t / rec_hdr_t), the CDF defaults
// and the tile bytes; the picture (pre loop filter) is read back through the frame buffer's host port.
module dec_top
  import cdf_map_pkg::*;
  import syn_pkg::*;
#(
    parameter int FBX = 10,                  // 1024 x 512 frame buffer: covers the test corpus (<= 640x360 + overhang)
    parameter int FBY = 9
) (
    input  logic              clk,
    input  logic              rst,
    input  hdr_t              hdr,
    input  rec_hdr_t          rh,
    input  lf_hdr_t           lh,
    // tile bytes
    input  logic [7:0]        in_data,
    input  logic              in_valid,
    output logic              in_ready,
    input  logic              in_eos,
    // cdf defaults
    input  logic              def_we,
    input  logic [CDF_AW-1:0] def_addr,
    input  logic [245:0]      def_data,
    // control
    input  logic              tile_start,
    output logic              tile_busy,
    output logic              tile_done,           // syntax finished and every block reconstructed
    output logic              unsupported,
    // in-loop filters, run by software after all tiles of the frame: deblocking (7.14)
    input  logic              lf_start,
    output logic              lf_busy,
    output logic              lf_done,
    // observability (records as they stream between the stages)
    output logic              blk_done,
    output blk_rec_t          blk_rec,
    output logic              tx_done,
    output tx_rec_t           tx_rec,
    output logic              lr_done,
    output lr_rec_t           lr_rec,
    // frame buffer host read port (registered)
    input  logic [1:0]        h_plane,
    input  logic [FBX-1:0]    h_x,
    input  logic [FBY-1:0]    h_y,
    output logic [11:0]       h_rdata
);
    logic ts_done, tx_ack, sb_start, blk_info, pal_hold, pm_plane; logic [10:0] sb_r, sb_c;
    logic [9:0] q_addr; logic signed [20:0] q_data; logic [5:0] pm_x, pm_y; logic [2:0] pm_idx;
    tile_syntax u_ts (.clk, .rst, .hdr, .in_data, .in_valid, .in_ready, .in_eos, .def_we, .def_addr, .def_data,
                      .tile_start, .tile_busy, .tile_done(ts_done), .unsupported,
                      .sb_start_o(sb_start), .sb_r_o(sb_r), .sb_c_o(sb_c), .blk_info, .blk_done, .blk_rec,
                      .tx_done, .tx_rec, .tx_ack, .lr_done, .lr_rec, .q_addr, .q_data,
                      .pal_hold, .blk_ack(pal_hold), .pm_plane, .pm_x, .pm_y, .pm_idx);

    logic fb_re, fb_we, rc_busy; logic [1:0] fb_plane; logic [FBX-1:0] fb_x; logic [FBY-1:0] fb_y; logic [11:0] fb_wdata, fb_rdata;
    logic mi_blk_we, mi_blk_busy, mi_tx_we, mi_tx_busy; logic [10:0] mi_blk_r, mi_blk_c, mi_tx_row, mi_tx_col; logic [5:0] mi_blk_bw4, mi_blk_bh4;
    mi_lf_t mi_blk_data; logic [1:0] mi_tx_plane; logic [4:0] mi_tx_w4, mi_tx_h4, mi_tx_sz;
    recon_top #(.FBX(FBX), .FBY(FBY)) u_rc (.clk, .rst, .hdr, .rh, .sb_start, .sb_r, .sb_c, .blk_info, .blk_rec,
                                            .tx_done, .tx_rec, .tx_ack, .q_addr, .q_data, .pm_plane, .pm_x, .pm_y, .pm_idx, .busy(rc_busy),
                                            .mi_blk_we, .mi_blk_r, .mi_blk_c, .mi_blk_bw4, .mi_blk_bh4, .mi_blk_data, .mi_blk_busy,
                                            .mi_tx_we, .mi_tx_plane, .mi_tx_row, .mi_tx_col, .mi_tx_w4, .mi_tx_h4, .mi_tx_sz, .mi_tx_busy,
                                            .fb_re, .fb_we, .fb_plane, .fb_x, .fb_y, .fb_wdata, .fb_rdata);

    // per-4x4 state + deblocking filter; the filter owns the frame buffer port while it runs
    logic [10:0] rd_row, rd_col, txr_row, txr_col; mi_lf_t rd_info; logic [1:0] txr_plane; logic [4:0] txr_sz;
    mi_store u_mi (.clk, .rst, .blk_we(mi_blk_we), .blk_r(mi_blk_r), .blk_c(mi_blk_c), .blk_bw4(mi_blk_bw4), .blk_bh4(mi_blk_bh4), .blk_data(mi_blk_data), .blk_busy(mi_blk_busy),
                   .tx_we(mi_tx_we), .tx_plane(mi_tx_plane), .tx_row(mi_tx_row), .tx_col(mi_tx_col), .tx_w4(mi_tx_w4), .tx_h4(mi_tx_h4), .tx_sz(mi_tx_sz), .tx_busy(mi_tx_busy),
                   .rd_row, .rd_col, .rd_info, .txr_plane, .txr_row, .txr_col, .txr_sz,
                   .cd_clr(1'b0), .cd_we(1'b0), .cd_row64(7'd0), .cd_col64(7'd0), .cd_sb128(1'b0), .cd_mask(4'd0), .cd_idx(3'd0),
                   .cdr_row64(7'd0), .cdr_col64(7'd0), .cdr_val());
    logic lf_re, lf_we; logic [1:0] lf_plane; logic [FBX-1:0] lf_x; logic [FBY-1:0] lf_y; logic [11:0] lf_wdata;
    lf_top #(.FBX(FBX), .FBY(FBY)) u_lf (.clk, .rst, .hdr, .lh, .start(lf_start), .busy(lf_busy), .done(lf_done),
                                         .rd_row, .rd_col, .rd_info, .txr_plane, .txr_row, .txr_col, .txr_sz,
                                         .fb_re(lf_re), .fb_we(lf_we), .fb_plane(lf_plane), .fb_x(lf_x), .fb_y(lf_y), .fb_wdata(lf_wdata), .fb_rdata);

    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12)) u_fb (.clk, .re(lf_busy ? lf_re : fb_re), .we(lf_busy ? lf_we : fb_we), .plane(lf_busy ? lf_plane : fb_plane),
                                                     .x(lf_busy ? lf_x : fb_x), .y(lf_busy ? lf_y : fb_y), .wdata(lf_busy ? lf_wdata : fb_wdata), .rdata(fb_rdata),
                                                     .h_plane, .h_x, .h_y, .h_rdata);

    // tile_done once the syntax is finished and the reconstruction stage has drained
    logic done_pend;
    always_ff @(posedge clk) begin
        tile_done <= 1'b0;
        if (rst) done_pend <= 1'b0;
        else begin
            if (ts_done) done_pend <= 1'b1;
            if ((done_pend || ts_done) && !rc_busy) begin done_pend <= 1'b0; tile_done <= 1'b1; end
        end
    end
endmodule
