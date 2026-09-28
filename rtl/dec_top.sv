// Intra tile decoder top: tile_syntax (entropy decoding + syntax) -> recon_top (prediction, transforms,
// reconstruction) -> frame_mem, plus mi_store + lf_top (deblocking, on lf_start) + cdef_top (on cdef_start, into a
// second frame buffer) + lr_top (on lr_start, into a third). h_buf selects which picture the host reads. Software supplies the parsed headers (hdr_t / rec_hdr_t), the CDF defaults
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
    input  cdef_hdr_t         ch,
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
    output logic              tile_done,           // syntax finished and every block reconstructed (1-cycle pulse)
    output logic              tile_done_lvl,       // same, held until the next tile_start (for slow pollers)
    output logic              unsupported,
    output logic [31:0]       perf_syn_stall,      // tile cycles the syntax decoder held a transform block waiting for reconstruction
    output logic [31:0]       perf_rec_idle,       // tile cycles reconstruction had nothing to do (syntax-bound)
    // in-loop filters, run by software after all tiles of the frame: deblocking (7.14)
    input  logic              lf_start,
    output logic              lf_busy,
    output logic              lf_done,
    input  logic              cdef_start,          // after lf_done (when enable_cdef && !CodedLossless && !allow_intrabc)
    output logic              cdef_busy,
    output logic              cdef_done,
    input  logic              sr_start,            // super-resolution (7.16) of buffer sr_buf in place, before lr_start
    input  logic              sr_buf,              // 0: fb0 (deblocked frame), 1: fb1 (CDEF frame)
    output logic              sr_busy,
    output logic              sr_done,
    input  logic              lr_start,            // after cdef_done (or lf_done when CDEF is off); reads fb0 + fb1, writes fb2
    input  logic              lr_from_deblocked,   // 1 when CDEF did not run this frame: CdefFrame == deblocked frame, read fb0 for both
    output logic              lr_busy,
    output logic              lr_done_o,
    // observability (records as they stream between the stages)
    output logic              blk_done,
    output blk_rec_t          blk_rec,
    output logic              tx_done,
    output tx_rec_t           tx_rec,
    output logic              lr_done,
    output lr_rec_t           lr_rec,
    // frame buffer host read port (registered): h_buf 0 = deblocked frame, 1 = CdefFrame, 2 = LrFrame
    input  logic [1:0]        h_buf,
    input  logic [1:0]        h_plane,
    input  logic [FBX-1:0]    h_x,
    input  logic [FBY-1:0]    h_y,
    output logic [11:0]       h_rdata
);
    logic ts_done, tx_ack, sb_start, blk_info, pal_hold, pm_plane, blk_ack; logic [10:0] sb_r, sb_c;
    logic [9:0] q_addr; logic [2:0] q_slot_w, q_slot_r; logic signed [20:0] q_data; logic [5:0] pm_x, pm_y; logic [2:0] pm_idx;
    tile_syntax #(.ML2R(FBY - 2), .ML2C(FBX - 2)) u_ts (.clk, .rst, .hdr, .in_data, .in_valid, .in_ready, .in_eos, .def_we, .def_addr, .def_data,
                      .tile_start, .tile_busy, .tile_done(ts_done), .unsupported,
                      .sb_start_o(sb_start), .sb_r_o(sb_r), .sb_c_o(sb_c), .blk_info, .blk_done, .blk_rec,
                      .tx_done, .tx_rec, .tx_ack, .lr_done, .lr_rec, .q_slot_w, .q_slot_r, .q_addr, .q_data,
                      .pal_hold, .blk_ack, .pm_plane, .pm_x, .pm_y, .pm_idx);
    // event queue: the syntax decoder runs ahead of reconstruction; a palette block is released (blk_ack) only
    // once reconstruction has drained the queue, because its colour map lives in the syntax stage
    logic ev_valid, ev_pop; logic [1:0] ev_kind; logic [10:0] ev_sb_r, ev_sb_c; blk_rec_t ev_blk; tx_rec_t ev_tx; logic [3:0] tx_inflight;
    rec_fifo u_q (.clk, .rst, .flush(tile_start), .sb_start, .sb_r, .sb_c, .blk_info, .blk_rec, .tx_done, .tx_rec, .tx_ack, .w_slot(q_slot_w),
                  .ev_valid, .ev_kind, .ev_sb_r, .ev_sb_c, .ev_blk, .ev_tx, .ev_pop, .tx_inflight);
    assign blk_ack = pal_hold && !ev_valid && !rc_busy;

    logic fb_re, fb_we, rc_busy; logic [1:0] fb_plane; logic [FBX-1:0] fb_x; logic [FBY-1:0] fb_y; logic [11:0] fb_wdata, fb_rdata;
    logic fb2_we; logic [1:0] fb2_plane; logic [FBX-1:0] fb2_x; logic [FBY-1:0] fb2_y; logic [11:0] fb2_wdata;
    logic fb4_we; logic [1:0] fb4_plane; logic [FBX-1:0] fb4_x; logic [FBY-1:0] fb4_y; logic [47:0] fb4_wdata, fb_rdata4;
    logic mi_blk_we, mi_blk_busy, mi_tx_we, mi_tx_busy; logic [10:0] mi_blk_r, mi_blk_c, mi_tx_row, mi_tx_col; logic [5:0] mi_blk_bw4, mi_blk_bh4;
    mi_lf_t mi_blk_data; logic [1:0] mi_tx_plane; logic [4:0] mi_tx_w4, mi_tx_h4, mi_tx_sz;
    logic mi_cd_clr, mi_cd_we; logic [6:0] mi_cd_row64, mi_cd_col64; logic [3:0] mi_cd_mask; logic [2:0] mi_cd_idx;
    recon_top #(.FBX(FBX), .FBY(FBY), .ML2R(FBY - 2), .ML2C(FBX - 2)) u_rc (.clk, .rst, .hdr, .rh,
                                            .ev_valid, .ev_kind, .ev_sb_r, .ev_sb_c, .ev_blk, .ev_tx, .ev_pop,
                                            .q_slot(q_slot_r), .q_addr, .q_data, .pm_plane, .pm_x, .pm_y, .pm_idx, .busy(rc_busy),
                                            .mi_blk_we, .mi_blk_r, .mi_blk_c, .mi_blk_bw4, .mi_blk_bh4, .mi_blk_data, .mi_blk_busy,
                                            .mi_tx_we, .mi_tx_plane, .mi_tx_row, .mi_tx_col, .mi_tx_w4, .mi_tx_h4, .mi_tx_sz, .mi_tx_busy,
                                            .mi_cd_clr, .mi_cd_we, .mi_cd_row64, .mi_cd_col64, .mi_cd_mask, .mi_cd_idx,
                                            .fb_re, .fb_we, .fb_plane, .fb_x, .fb_y, .fb_wdata, .fb_rdata, .fb_rdata4,
                                            .fb2_we, .fb2_plane, .fb2_x, .fb2_y, .fb2_wdata,
                                            .fb4_we, .fb4_plane, .fb4_x, .fb4_y, .fb4_wdata);

    // per-4x4 state + deblocking filter; the filter owns the frame buffer port while it runs
    logic [10:0] rd_row, rd_col, txr_row, txr_col; mi_lf_t rd_info; logic [1:0] txr_plane; logic [4:0] txr_sz;
    mi_store #(.ML2R(FBY - 2), .ML2C(FBX - 2)) u_mi (.clk, .rst, .blk_we(mi_blk_we), .blk_r(mi_blk_r), .blk_c(mi_blk_c), .blk_bw4(mi_blk_bw4), .blk_bh4(mi_blk_bh4), .blk_data(mi_blk_data), .blk_busy(mi_blk_busy),
                   .tx_we(mi_tx_we), .tx_plane(mi_tx_plane), .tx_row(mi_tx_row), .tx_col(mi_tx_col), .tx_w4(mi_tx_w4), .tx_h4(mi_tx_h4), .tx_sz(mi_tx_sz), .tx_busy(mi_tx_busy),
                   .rd_row(mi_rd_row), .rd_col(mi_rd_col), .rd_info, .txr_plane, .txr_row, .txr_col, .txr_sz,
                   .cd_clr(mi_cd_clr), .cd_we(mi_cd_we), .cd_row64(mi_cd_row64), .cd_col64(mi_cd_col64), .cd_sb128(hdr.sb128), .cd_mask(mi_cd_mask), .cd_idx(mi_cd_idx),
                   .cdr_row64, .cdr_col64, .cdr_val,
                   .lr_we(lr_done), .lr_rec, .lrr_plane, .lrr_row, .lrr_col, .lrr_rec);
    logic lf_re, lf_we; logic [1:0] lf_plane; logic [FBX-1:0] lf_x; logic [FBY-1:0] lf_y; logic [11:0] lf_wdata;
    logic lf4_we; logic [FBX-1:0] lf4_x; logic [FBY-1:0] lf4_y; logic [47:0] lf4_wdata;
    lf_top #(.FBX(FBX), .FBY(FBY)) u_lf (.clk, .rst, .hdr, .lh, .start(lf_start), .busy(lf_busy), .done(lf_done),
                                         .rd_row, .rd_col, .rd_info, .txr_plane, .txr_row, .txr_col, .txr_sz,
                                         .fb_re(lf_re), .fb_we(lf_we), .fb_plane(lf_plane), .fb_x(lf_x), .fb_y(lf_y), .fb_wdata(lf_wdata), .fb_rdata,
                                         .fb_rdata4, .fb4_we(lf4_we), .fb4_x(lf4_x), .fb4_y(lf4_y), .fb4_wdata(lf4_wdata));
    // CDEF: reads the deblocked frame (fb0, shared port while cdef_busy), writes CdefFrame (fb1)
    logic [10:0] cd_rd_row, cd_rd_col; logic [6:0] cdr_row64, cdr_col64; logic [3:0] cdr_val;
    logic cd_re, cd_we; logic [1:0] cd_plane, cd_dplane; logic [FBX-1:0] cd_x, cd_dx; logic [FBY-1:0] cd_y, cd_dy; logic [11:0] cd_wdata;
    logic cd4_we; logic [1:0] cd4_plane; logic [FBX-1:0] cd4_x; logic [FBY-1:0] cd4_y; logic [47:0] cd4_wdata;
    cdef_top #(.FBX(FBX), .FBY(FBY)) u_cdef (.clk, .rst, .hdr, .ch, .start(cdef_start), .busy(cdef_busy), .done(cdef_done),
                                             .rd_row(cd_rd_row), .rd_col(cd_rd_col), .rd_info, .cdr_row64, .cdr_col64, .cdr_val,
                                             .src_re(cd_re), .src_plane(cd_plane), .src_x(cd_x), .src_y(cd_y), .src_rdata(fb_rdata), .src_rdata4(fb_rdata4),
                                             .dst4_we(cd4_we), .dst4_plane(cd4_plane), .dst4_x(cd4_x), .dst4_y(cd4_y), .dst4_wdata(cd4_wdata),
                                             .dst_we(cd_we), .dst_plane(cd_dplane), .dst_x(cd_dx), .dst_y(cd_dy), .dst_wdata(cd_wdata));
    logic [10:0] mi_rd_row, mi_rd_col;
    assign mi_rd_row = cdef_busy ? cd_rd_row : rd_row;
    assign mi_rd_col = cdef_busy ? cd_rd_col : rd_col;

    // loop restoration: reads fb0 (deblocked) and fb1 (CdefFrame) while lr_busy, writes fb2 (LrFrame)
    logic [1:0] lrr_plane; logic [5:0] lrr_row, lrr_col; lr_rec_t lrr_rec;
    logic lr_s0_re, lr_s1_re, lr_we2; logic [1:0] lr_splane, lr_dplane; logic [FBX-1:0] lr_sx, lr_dx; logic [FBY-1:0] lr_sy, lr_dy; logic [11:0] lr_wdata, fb1_rdata;
    lr_top #(.FBX(FBX), .FBY(FBY)) u_lr (.clk, .rst, .hdr, .start(lr_start), .busy(lr_busy), .done(lr_done_o),
                                         .lrr_plane, .lrr_row, .lrr_col, .lrr_rec,
                                         .s0_re(lr_s0_re), .s1_re(lr_s1_re), .s_plane(lr_splane), .s_x(lr_sx), .s_y(lr_sy), .s0_rdata(fb_rdata),
                                         .s1_rdata(lr_from_deblocked ? fb_rdata : fb1_rdata),
                                         .d_we(lr_we2), .d_plane(lr_dplane), .d_x(lr_dx), .d_y(lr_dy), .d_wdata(lr_wdata));

    logic sr_re, sr_we; logic [1:0] sr_plane; logic [FBX-1:0] sr_sx, sr_dx; logic [FBY-1:0] sr_sy, sr_dy; logic [11:0] sr_wdata;
    logic sr0, sr1;
    assign sr0 = sr_busy && !sr_buf;
    assign sr1 = sr_busy && sr_buf;
    sr_top #(.FBX(FBX), .FBY(FBY)) u_sr (.clk, .rst, .hdr, .lh, .start(sr_start), .busy(sr_busy), .done(sr_done),
                                         .s_re(sr_re), .plane(sr_plane), .s_x(sr_sx), .s_y(sr_sy), .s_rdata(sr_buf ? fb1_rdata : fb_rdata),
                                         .d_we(sr_we), .d_x(sr_dx), .d_y(sr_dy), .d_wdata(sr_wdata));
    logic [11:0] h_rdata0, h_rdata1, h_rdata2;
    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12)) u_fb (.clk, .re(lf_busy ? lf_re : cdef_busy ? cd_re : sr0 ? sr_re : lr_busy ? (lr_s0_re || (lr_from_deblocked && lr_s1_re)) : fb_re),
                                                     .we(lf_busy ? lf_we : sr0 ? sr_we : (cdef_busy || lr_busy || sr1) ? 1'b0 : fb_we),
                                                     .plane(lf_busy ? lf_plane : cdef_busy ? cd_plane : sr0 ? sr_plane : lr_busy ? lr_splane : fb_plane),
                                                     .x(lf_busy ? lf_x : cdef_busy ? cd_x : sr0 ? (sr_we ? sr_dx : sr_sx) : lr_busy ? lr_sx : fb_x),
                                                     .y(lf_busy ? lf_y : cdef_busy ? cd_y : sr0 ? (sr_we ? sr_dy : sr_sy) : lr_busy ? lr_sy : fb_y),
                                                     .wdata(lf_busy ? lf_wdata : sr0 ? sr_wdata : fb_wdata), .rdata(fb_rdata),
                                                     .w2_we(fb2_we && !lf_busy && !cdef_busy && !sr_busy && !lr_busy), .w2_plane(fb2_plane), .w2_x(fb2_x), .w2_y(fb2_y), .w2_wdata(fb2_wdata),
                                                     .w4_we(lf_busy ? lf4_we : (fb4_we && !cdef_busy && !sr_busy && !lr_busy)), .w4_plane(lf_busy ? lf_plane : fb4_plane),
                                                     .w4_x(lf_busy ? lf4_x : fb4_x), .w4_y(lf_busy ? lf4_y : fb4_y), .w4_wdata(lf_busy ? lf4_wdata : fb4_wdata), .rdata4(fb_rdata4),
                                                     .h_plane, .h_x, .h_y, .h_rdata(h_rdata0));
    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12)) u_fb1 (.clk, .w2_we(1'b0), .w2_plane(2'd0), .w2_x('0), .w2_y('0), .w2_wdata('0), .w4_we(cdef_busy && cd4_we), .w4_plane(cd4_plane), .w4_x(cd4_x), .w4_y(cd4_y), .w4_wdata(cd4_wdata), .rdata4(), .re(sr1 ? sr_re : lr_busy ? (lr_s1_re && !lr_from_deblocked) : 1'b0), .we(cdef_busy ? cd_we : sr1 ? sr_we : 1'b0),
                                                      .plane(sr1 ? sr_plane : lr_busy ? lr_splane : cd_dplane),
                                                      .x(sr1 ? (sr_we ? sr_dx : sr_sx) : lr_busy ? lr_sx : cd_dx), .y(sr1 ? (sr_we ? sr_dy : sr_sy) : lr_busy ? lr_sy : cd_dy),
                                                      .wdata(sr1 ? sr_wdata : cd_wdata), .rdata(fb1_rdata),
                                                      .h_plane, .h_x, .h_y, .h_rdata(h_rdata1));
    logic [11:0] unused_r2;
    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12)) u_fb2 (.clk, .w2_we(1'b0), .w2_plane(2'd0), .w2_x('0), .w2_y('0), .w2_wdata('0), .w4_we(1'b0), .w4_plane(2'd0), .w4_x('0), .w4_y('0), .w4_wdata('0), .rdata4(), .re(1'b0), .we(lr_we2), .plane(lr_dplane), .x(lr_dx), .y(lr_dy), .wdata(lr_wdata), .rdata(unused_r2),
                                                      .h_plane, .h_x, .h_y, .h_rdata(h_rdata2));
    assign h_rdata = (h_buf == 2'd2) ? h_rdata2 : (h_buf == 2'd1) ? h_rdata1 : h_rdata0;

    // tile_done once the syntax is finished and the reconstruction stage has drained
    logic done_pend;
    always_ff @(posedge clk) begin
        tile_done <= 1'b0;
        if (rst) begin done_pend <= 1'b0; tile_done_lvl <= 1'b0; perf_syn_stall <= 32'd0; perf_rec_idle <= 32'd0; end
        else begin
            if (tile_start) begin tile_done_lvl <= 1'b0; perf_syn_stall <= 32'd0; perf_rec_idle <= 32'd0; end
            else begin
                if (tx_done && !tx_ack) perf_syn_stall <= perf_syn_stall + 32'd1;          // queue full
                if (tile_busy && !ev_valid && !rc_busy) perf_rec_idle <= perf_rec_idle + 32'd1;   // nothing queued
            end
            if (ts_done) done_pend <= 1'b1;
            if ((done_pend || ts_done) && !rc_busy && !ev_valid) begin done_pend <= 1'b0; tile_done <= 1'b1; tile_done_lvl <= 1'b1; end
        end
    end
endmodule
