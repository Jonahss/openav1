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
    // frame sequencer (pipe_en = 1): the in-loop filters trail the tile decoder by superblock rows; software
    // pulses frame_start after the headers, feeds the tiles as usual and waits for frame_done
    input  logic              pipe_en,
    input  logic              frame_start,
    input  logic [5:0]        n_tile_cols,         // tile columns: a superblock row is decoded once every column passed it
    output logic              frame_busy,
    output logic              frame_done,          // 1-cycle pulse
    output logic              frame_done_lvl,      // held until the next frame_start
    // in-loop filters, run by software after all tiles of the frame (pipe_en = 0): deblocking (7.14)
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
                   .rd_row(rd_row), .rd_col(rd_col), .rd_info, .rd2_row(cd_rd_row), .rd2_col(cd_rd_col), .rd2_info(cd_rd_info), .txr_plane, .txr_row, .txr_col, .txr_sz,
                   .cd_clr(mi_cd_clr), .cd_we(mi_cd_we), .cd_row64(mi_cd_row64), .cd_col64(mi_cd_col64), .cd_sb128(hdr.sb128), .cd_mask(mi_cd_mask), .cd_idx(mi_cd_idx),
                   .cdr_row64, .cdr_col64, .cdr_val,
                   .lr_we(lr_done), .lr_rec, .lrr_plane, .lrr_row, .lrr_col, .lrr_rec);
    // stage row ranges (whole frame in host-driven mode; the frame sequencer bounds them per superblock row)
    logic [10:0] lf_row_first, lf_row_last, cd_row_first, cd_row_last; logic [12:0] lr_ly_first, lr_ly_last, sr_ly_first, sr_ly_last;
    logic lf_go, cd_go, sr_go, lr_go, lf_start_i, cdef_start_i, sr_start_i, lr_start_i, sr_buf_i, lr_from_deblocked_i;
    logic [6:0] lf_next, cd_next, sr_next, lr_next;   // next superblock row (stripe for LR) each stage will process
    logic [12:0] lr_ly0, lr_ly1;
    assign lf_start_i = pipe_en ? lf_go : lf_start; assign cdef_start_i = pipe_en ? cd_go : cdef_start;
    assign sr_start_i = pipe_en ? sr_go : sr_start; assign lr_start_i = pipe_en ? lr_go : lr_start;
    logic lf_re, lf_we; logic [1:0] lf_plane; logic [FBX-1:0] lf_x; logic [FBY-1:0] lf_y; logic [11:0] lf_wdata;
    logic lf4_we; logic [FBX-1:0] lf4_x; logic [FBY-1:0] lf4_y; logic [47:0] lf4_wdata;
    lf_top #(.FBX(FBX), .FBY(FBY)) u_lf (.clk, .rst, .hdr, .lh, .start(lf_start_i), .row_first(lf_row_first), .row_last(lf_row_last), .busy(lf_busy), .done(lf_done),
                                         .rd_row, .rd_col, .rd_info, .txr_plane, .txr_row, .txr_col, .txr_sz,
                                         .fb_re(lf_re), .fb_we(lf_we), .fb_plane(lf_plane), .fb_x(lf_x), .fb_y(lf_y), .fb_wdata(lf_wdata), .fb_rdata(lf_rdata),
                                         .fb_rdata4(lf_rdata4), .fb4_we(lf4_we), .fb4_x(lf4_x), .fb4_y(lf4_y), .fb4_wdata(lf4_wdata));
    // CDEF: reads the deblocked frame (fb0, shared port while cdef_busy), writes CdefFrame (fb1)
    logic [10:0] cd_rd_row, cd_rd_col; logic [6:0] cdr_row64, cdr_col64; logic [3:0] cdr_val; mi_lf_t cd_rd_info;
    logic cd_re, cd_we; logic [1:0] cd_plane, cd_dplane; logic [FBX-1:0] cd_x, cd_dx; logic [FBY-1:0] cd_y, cd_dy; logic [11:0] cd_wdata;
    logic cd4_we; logic [1:0] cd4_plane; logic [FBX-1:0] cd4_x; logic [FBY-1:0] cd4_y; logic [47:0] cd4_wdata;
    cdef_top #(.FBX(FBX), .FBY(FBY)) u_cdef (.clk, .rst, .hdr, .ch, .start(cdef_start_i), .row_first(cd_row_first), .row_last(cd_row_last), .busy(cdef_busy), .done(cdef_done),
                                             .rd_row(cd_rd_row), .rd_col(cd_rd_col), .rd_info(cd_rd_info), .cdr_row64, .cdr_col64, .cdr_val,
                                             .src_re(cd_re), .src_plane(cd_plane), .src_x(cd_x), .src_y(cd_y), .src_rdata(cd_rdata), .src_rdata4(cd_rdata4),
                                             .dst4_we(cd4_we), .dst4_plane(cd4_plane), .dst4_x(cd4_x), .dst4_y(cd4_y), .dst4_wdata(cd4_wdata),
                                             .dst_we(cd_we), .dst_plane(cd_dplane), .dst_x(cd_dx), .dst_y(cd_dy), .dst_wdata(cd_wdata));

    // loop restoration: reads fb0 (deblocked) and fb1 (CdefFrame) while lr_busy, writes fb2 (LrFrame)
    logic [1:0] lrr_plane; logic [5:0] lrr_row, lrr_col; lr_rec_t lrr_rec;
    logic lr_s0_re, lr_s1_re, lr_we2; logic [1:0] lr_splane, lr_dplane; logic [FBX-1:0] lr_sx, lr_dx; logic [FBY-1:0] lr_sy, lr_dy; logic [11:0] lr_wdata; logic lr_we4; logic [3:0] lr_be4; logic [47:0] lr_wdata4, lr0_rdata4, lr1_rdata4;
    lr_top #(.FBX(FBX), .FBY(FBY)) u_lr (.clk, .rst, .hdr, .start(lr_start_i), .ly_first(lr_ly_first), .ly_last(lr_ly_last), .busy(lr_busy), .done(lr_done_o),
                                         .lrr_plane, .lrr_row, .lrr_col, .lrr_rec,
                                         .s0_re(lr_s0_re), .s1_re(lr_s1_re), .s_plane(lr_splane), .s_x(lr_sx), .s_y(lr_sy), .s0_rdata(lr0_rdata), .s0_rdata4(lr0_rdata4),
                                         .s1_rdata(lr_from_deblocked_i ? lr0_rdata : lr1_rdata), .s1_rdata4(lr_from_deblocked_i ? lr0_rdata4 : lr1_rdata4),
                                         .d4_we(lr_we4), .d4_be(lr_be4), .d4_wdata(lr_wdata4),
                                         .d_we(lr_we2), .d_plane(lr_dplane), .d_x(lr_dx), .d_y(lr_dy), .d_wdata(lr_wdata));

    logic sr_re, sr_we; logic [1:0] sr_plane; logic [FBX-1:0] sr_sx, sr_dx; logic [FBY-1:0] sr_sy, sr_dy; logic [11:0] sr_wdata;
    logic sr0, sr1;
    assign sr_buf_i = pipe_en ? sr_sub : sr_buf;
    assign lr_from_deblocked_i = pipe_en ? !cdef_en : lr_from_deblocked;
    assign sr0 = sr_busy && !sr_buf_i;
    assign sr1 = sr_busy && sr_buf_i;
    sr_top #(.FBX(FBX), .FBY(FBY)) u_sr (.clk, .rst, .hdr, .lh, .start(sr_start_i), .ly_first(sr_ly_first), .ly_last(sr_ly_last), .busy(sr_busy), .done(sr_done),
                                         .s_re(sr_re), .plane(sr_plane), .s_x(sr_sx), .s_y(sr_sy), .s_rdata(sr_buf_i ? sr1_rdata : sr0_rdata),
                                         .d_we(sr_we), .d_x(sr_dx), .d_y(sr_dy), .d_wdata(sr_wdata));
    logic [11:0] h_rdata0, h_rdata1, h_rdata2;
    // ---- frame buffers, one port per stage (no sharing: the stages may run concurrently)
    // fb0 = pre-filter / deblocked frame. reads: recon, LF, CDEF, LR source 0 (and source 1 when CDEF did not run), SR; writes: recon, LF, SR
    logic [11:0] lf_rdata, cd_rdata, lr0_rdata, sr0_rdata, lr1_rdata, sr1_rdata; logic [47:0] lf_rdata4, cd_rdata4;
    logic [4:0] fb0_re; logic [9:0] fb0_rplane; logic [5*FBX-1:0] fb0_rx; logic [5*FBY-1:0] fb0_ry; logic [59:0] fb0_rdata; logic [239:0] fb0_rdata4;
    logic [2:0] fb0_we; logic [11:0] fb0_be; logic [5:0] fb0_wplane; logic [3*FBX-1:0] fb0_wx; logic [3*FBY-1:0] fb0_wy; logic [143:0] fb0_wdata;
    logic [47:0] unused_rd4;
    assign fb0_re = {sr0 && sr_re, lr_s0_re || (lr_from_deblocked_i && lr_s1_re), cd_re, lf_re, fb_re};
    assign fb0_rplane = {sr_plane, lr_splane, cd_plane, lf_plane, fb_plane};
    assign fb0_rx = {sr_sx, lr_sx, cd_x, lf_x, fb_x};
    assign fb0_ry = {sr_sy, lr_sy, cd_y, lf_y, fb_y};
    assign {sr0_rdata, lr0_rdata, cd_rdata, lf_rdata, fb_rdata} = fb0_rdata;
    assign {unused_rd4, lr0_rdata4, cd_rdata4, lf_rdata4, fb_rdata4} = fb0_rdata4;
    logic w_rc_we, w_lf_we, w_sr0_we; logic [3:0] w_rc_be, w_lf_be, w_sr0_be; logic [1:0] w_rc_pl, w_lf_pl, w_sr0_pl;
    logic [FBX-1:0] w_rc_x, w_lf_x, w_sr0_x; logic [FBY-1:0] w_rc_y, w_lf_y, w_sr0_y; logic [47:0] w_rc_d, w_lf_d, w_sr0_d;
    frame_mem_w #(.FBX(FBX), .FBY(FBY)) u_w_rc (.we1(fb_we || fb2_we), .plane1(fb2_we ? fb2_plane : fb_plane), .x1(fb2_we ? fb2_x : fb_x), .y1(fb2_we ? fb2_y : fb_y), .d1(fb2_we ? fb2_wdata : fb_wdata),
                                                .we4(fb4_we), .plane4(fb4_plane), .x4(fb4_x), .y4(fb4_y), .d4(fb4_wdata),
                                                .we(w_rc_we), .be(w_rc_be), .plane(w_rc_pl), .x(w_rc_x), .y(w_rc_y), .data(w_rc_d));
    frame_mem_w #(.FBX(FBX), .FBY(FBY)) u_w_lf (.we1(lf_we), .plane1(lf_plane), .x1(lf_x), .y1(lf_y), .d1(lf_wdata),
                                                .we4(lf4_we), .plane4(lf_plane), .x4(lf4_x), .y4(lf4_y), .d4(lf4_wdata),
                                                .we(w_lf_we), .be(w_lf_be), .plane(w_lf_pl), .x(w_lf_x), .y(w_lf_y), .data(w_lf_d));
    frame_mem_w #(.FBX(FBX), .FBY(FBY)) u_w_sr0 (.we1(sr0 && sr_we), .plane1(sr_plane), .x1(sr_dx), .y1(sr_dy), .d1(sr_wdata),
                                                 .we4(1'b0), .plane4(2'd0), .x4('0), .y4('0), .d4('0),
                                                 .we(w_sr0_we), .be(w_sr0_be), .plane(w_sr0_pl), .x(w_sr0_x), .y(w_sr0_y), .data(w_sr0_d));
    assign fb0_we = {w_sr0_we, w_lf_we, w_rc_we}; assign fb0_be = {w_sr0_be, w_lf_be, w_rc_be}; assign fb0_wplane = {w_sr0_pl, w_lf_pl, w_rc_pl};
    assign fb0_wx = {w_sr0_x, w_lf_x, w_rc_x}; assign fb0_wy = {w_sr0_y, w_lf_y, w_rc_y}; assign fb0_wdata = {w_sr0_d, w_lf_d, w_rc_d};
    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12), .NRD(5), .NWR(3)) u_fb (.clk, .re(fb0_re), .r_plane(fb0_rplane), .r_x(fb0_rx), .r_y(fb0_ry), .rdata(fb0_rdata), .rdata4(fb0_rdata4),
                                                                       .we(fb0_we), .w_be(fb0_be), .w_plane(fb0_wplane), .w_x(fb0_wx), .w_y(fb0_wy), .w_data(fb0_wdata),
                                                                       .h_plane, .h_x, .h_y, .h_rdata(h_rdata0));
    // fb1 = CdefFrame. reads: LR source 1, SR; writes: CDEF, SR
    logic [1:0] fb1_re; logic [3:0] fb1_rplane; logic [2*FBX-1:0] fb1_rx; logic [2*FBY-1:0] fb1_ry; logic [23:0] fb1_rdata; logic [95:0] fb1_rdata4;
    logic [1:0] fb1_we; logic [7:0] fb1_be; logic [3:0] fb1_wplane; logic [2*FBX-1:0] fb1_wx; logic [2*FBY-1:0] fb1_wy; logic [95:0] fb1_wdata;
    assign fb1_re = {sr1 && sr_re, lr_s1_re && !lr_from_deblocked_i};
    assign fb1_rplane = {sr_plane, lr_splane}; assign fb1_rx = {sr_sx, lr_sx}; assign fb1_ry = {sr_sy, lr_sy};
    assign {sr1_rdata, lr1_rdata} = fb1_rdata;
    assign lr1_rdata4 = fb1_rdata4[47:0];
    logic w_cd_we, w_sr1_we; logic [3:0] w_cd_be, w_sr1_be; logic [1:0] w_cd_pl, w_sr1_pl; logic [FBX-1:0] w_cd_x, w_sr1_x; logic [FBY-1:0] w_cd_y, w_sr1_y; logic [47:0] w_cd_d, w_sr1_d;
    frame_mem_w #(.FBX(FBX), .FBY(FBY)) u_w_cd (.we1(cd_we), .plane1(cd_dplane), .x1(cd_dx), .y1(cd_dy), .d1(cd_wdata),
                                                .we4(cd4_we), .plane4(cd4_plane), .x4(cd4_x), .y4(cd4_y), .d4(cd4_wdata),
                                                .we(w_cd_we), .be(w_cd_be), .plane(w_cd_pl), .x(w_cd_x), .y(w_cd_y), .data(w_cd_d));
    frame_mem_w #(.FBX(FBX), .FBY(FBY)) u_w_sr1 (.we1(sr1 && sr_we), .plane1(sr_plane), .x1(sr_dx), .y1(sr_dy), .d1(sr_wdata),
                                                 .we4(1'b0), .plane4(2'd0), .x4('0), .y4('0), .d4('0),
                                                 .we(w_sr1_we), .be(w_sr1_be), .plane(w_sr1_pl), .x(w_sr1_x), .y(w_sr1_y), .data(w_sr1_d));
    assign fb1_we = {w_sr1_we, w_cd_we}; assign fb1_be = {w_sr1_be, w_cd_be}; assign fb1_wplane = {w_sr1_pl, w_cd_pl};
    assign fb1_wx = {w_sr1_x, w_cd_x}; assign fb1_wy = {w_sr1_y, w_cd_y}; assign fb1_wdata = {w_sr1_d, w_cd_d};
    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12), .NRD(2), .NWR(2)) u_fb1 (.clk, .re(fb1_re), .r_plane(fb1_rplane), .r_x(fb1_rx), .r_y(fb1_ry), .rdata(fb1_rdata), .rdata4(fb1_rdata4),
                                                                        .we(fb1_we), .w_be(fb1_be), .w_plane(fb1_wplane), .w_x(fb1_wx), .w_y(fb1_wy), .w_data(fb1_wdata),
                                                                        .h_plane, .h_x, .h_y, .h_rdata(h_rdata1));
    // fb2 = LrFrame. writes: LR
    logic [11:0] fb2_rdata_u; logic [47:0] fb2_rdata4_u;
    logic w_lr_we; logic [3:0] w_lr_be; logic [1:0] w_lr_pl; logic [FBX-1:0] w_lr_x; logic [FBY-1:0] w_lr_y; logic [47:0] w_lr_d;
    assign w_lr_we = lr_we4; assign w_lr_be = lr_be4; assign w_lr_pl = lr_dplane; assign w_lr_x = lr_dx; assign w_lr_y = lr_dy; assign w_lr_d = lr_wdata4;
    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12), .NRD(1), .NWR(1)) u_fb2 (.clk, .re(1'b0), .r_plane(2'd0), .r_x('0), .r_y('0), .rdata(fb2_rdata_u), .rdata4(fb2_rdata4_u),
                                                                        .we(w_lr_we), .w_be(w_lr_be), .w_plane(w_lr_pl), .w_x(w_lr_x), .w_y(w_lr_y), .w_data(w_lr_d),
                                                                        .h_plane, .h_x, .h_y, .h_rdata(h_rdata2));
    assign h_rdata = (h_buf == 2'd2) ? h_rdata2 : (h_buf == 2'd1) ? h_rdata1 : h_rdata0;

    // ================================================================ frame sequencer (pipe_en): stages per superblock row
    // Row r of the frame is decoded once every tile column has reconstructed past it (row_cnt[r] == n_tile_cols); a
    // tile passes a row when its reconstruction pops the first superblock of the next row, or at tile_done.
    // Rules (see notes/projects/open-av1-decoder-throughput.md, step 7): LF(r) after row r+1 is decoded [vertical
    // edges of row r change its last line, which row r+1 predicts from]; CDEF(r) after LF(r+1) [window reads 2 lines
    // into row r+1, and LF(r+1)'s boundary edge changes row r's last lines]; SR(r) after CDEF(r+1) [CDEF reads row r's
    // last 2 lines at frame width] (or after LF(r+1) without CDEF); LR(stripe s) after CDEF and SR of the rows the
    // stripe touches. The last row takes "r+1" as already done.
    localparam int NSBR = 1 << (FBY - 6);                 // superblock rows (64-pixel; a 128 superblock counts as two entries' worth of mi rows)
    logic [4:0]  sb_shift;                                // log2(mi rows per superblock row)
    logic [6:0]  n_rows, n_stripes;
    logic [7:0]  row_cnt [0:NSBR-1];
    logic        cdef_en, sr_en, lr_en, sr_sub, seq_active, rows_seen;
    // frame geometry and stage enables follow the live header (the host loads it with the first tile, after frame_start)
    assign n_rows = 7'((hdr.mi_rows + (11'd1 << sb_shift) - 11'd1) >> sb_shift);
    assign n_stripes = 7'(((hdr.frame_height + 13'd7) >> 6) + 13'd1);
    assign cdef_en = hdr.enable_cdef && !hdr.coded_lossless && !hdr.allow_intrabc;
    assign sr_en = hdr.use_superres;
    assign lr_en = (hdr.lr_type[1:0] != 2'd0) || (!hdr.mono && ((hdr.lr_type[3:2] != 2'd0) || (hdr.lr_type[5:4] != 2'd0)));
    logic [6:0]  tile_row_prev; logic tile_row_valid;
    logic        sb_pop; logic [6:0] sb_pop_row;
    assign sb_shift = hdr.sb128 ? 5'd5 : 5'd4;
    assign sb_pop = ev_valid && ev_pop && (ev_kind == 2'd2);
    assign sb_pop_row = 7'(ev_sb_r >> sb_shift);
    function automatic logic row_dec(input logic [6:0] r);   // row r decoded by every tile column
        row_dec = (r >= n_rows) || (row_cnt[r[FBY-7:0]] >= 8'(n_tile_cols));
    endfunction
    // stage ranges
    logic [12:0] sb_px;                                   // luma rows per superblock row
    assign sb_px = hdr.sb128 ? 13'd128 : 13'd64;
    always_comb begin
        if (pipe_en) begin
            lf_row_first = 11'(lf_next) << sb_shift; lf_row_last = (11'(lf_next + 7'd1) << sb_shift) - 11'd1;
            if (lf_row_last >= hdr.mi_rows) lf_row_last = hdr.mi_rows - 11'd1;
            cd_row_first = 11'(cd_next) << sb_shift; cd_row_last = (11'(cd_next + 7'd1) << sb_shift) - 11'd1;
            if (cd_row_last >= hdr.mi_rows) cd_row_last = hdr.mi_rows - 11'd1;
            sr_ly_first = 13'(sr_next) * sb_px; sr_ly_last = 13'(sr_next + 7'd1) * sb_px - 13'd1;
            if (sr_ly_last >= hdr.frame_height) sr_ly_last = hdr.frame_height - 13'd1;
            lr_ly0 = (lr_next == 7'd0) ? 13'd0 : 13'(lr_next) * 13'd64 - 13'd8;
            lr_ly1 = 13'(lr_next) * 13'd64 + 13'd55;
            if (lr_ly1 >= hdr.frame_height) lr_ly1 = hdr.frame_height - 13'd1;
            lr_ly_first = lr_ly0; lr_ly_last = lr_ly1;
        end else begin
            lf_row_first = 11'd0; lf_row_last = hdr.mi_rows - 11'd1; cd_row_first = 11'd0; cd_row_last = hdr.mi_rows - 11'd1;
            sr_ly_first = 13'd0; sr_ly_last = hdr.frame_height - 13'd1; lr_ly_first = 13'd0; lr_ly_last = hdr.frame_height - 13'd1;
            lr_ly0 = 13'd0; lr_ly1 = 13'd0;
        end
    end
    // readiness of the row a stage wants next (rows finish in order, so "stage done up to r" is next > r)
    logic lf_ok, cd_ok, sr_ok, lr_ok;
    logic [6:0] lr_row;                                   // superblock row holding the stripe's last line
    always_comb begin
        logic [6:0] r;
        r = lf_next; lf_ok = (r < n_rows) && row_dec(r) && row_dec(r + 7'd1) && !mi_blk_busy && !mi_tx_busy;
        r = cd_next; cd_ok = cdef_en && (r < n_rows) && (lf_next > r) && ((r + 7'd1 >= n_rows) || (lf_next > r + 7'd1));
        r = sr_next; sr_ok = sr_en && (r < n_rows) && (cdef_en ? ((cd_next > r) && ((r + 7'd1 >= n_rows) || (cd_next > r + 7'd1)))
                                                            : ((lf_next > r) && ((r + 7'd1 >= n_rows) || (lf_next > r + 7'd1))));
        lr_row = 7'(lr_ly1 >> (hdr.sb128 ? 7 : 6));
        lr_ok = lr_en && (lr_next < n_stripes) && (cdef_en ? (cd_next > lr_row) : ((lf_next > lr_row) && ((lr_row + 7'd1 >= n_rows) || (lf_next > lr_row + 7'd1))))
                      && (!sr_en || (sr_next > lr_row));
    end
    logic lf_pend, cd_pend, sr_pend, lr_pend;             // start issued, done not yet seen
    always_ff @(posedge clk) begin
        lf_go <= 1'b0; cd_go <= 1'b0; sr_go <= 1'b0; lr_go <= 1'b0; frame_done <= 1'b0;
        if (rst) begin
            seq_active <= 1'b0; frame_busy <= 1'b0; frame_done_lvl <= 1'b0; tile_row_valid <= 1'b0; rows_seen <= 1'b0;
            lf_pend <= 1'b0; cd_pend <= 1'b0; sr_pend <= 1'b0; lr_pend <= 1'b0;
            lf_next <= 7'd0; cd_next <= 7'd0; sr_next <= 7'd0; lr_next <= 7'd0; sr_sub <= 1'b0;
            for (int i = 0; i < NSBR; i++) row_cnt[i] <= 8'd0;
        end else if (frame_start) begin
            seq_active <= 1'b1; frame_busy <= 1'b1; frame_done_lvl <= 1'b0; tile_row_valid <= 1'b0;
            lf_pend <= 1'b0; cd_pend <= 1'b0; sr_pend <= 1'b0; lr_pend <= 1'b0;
            lf_next <= 7'd0; cd_next <= 7'd0; sr_next <= 7'd0; lr_next <= 7'd0; sr_sub <= 1'b0;
            for (int i = 0; i < NSBR; i++) row_cnt[i] <= 8'd0;
            rows_seen <= 1'b0;
        end else if (seq_active) begin
            if (sb_pop) rows_seen <= 1'b1;
            // row completion per tile
            if (sb_pop) begin
                if (tile_row_valid && sb_pop_row != tile_row_prev) row_cnt[tile_row_prev[FBY-7:0]] <= row_cnt[tile_row_prev[FBY-7:0]] + 8'd1;
                tile_row_prev <= sb_pop_row; tile_row_valid <= 1'b1;
            end
            if (tile_done && tile_row_valid) begin row_cnt[tile_row_prev[FBY-7:0]] <= row_cnt[tile_row_prev[FBY-7:0]] + 8'd1; tile_row_valid <= 1'b0; end
            // stage starts
            if (!lf_pend && !lf_busy && lf_ok) begin lf_go <= 1'b1; lf_pend <= 1'b1; end
            if (lf_pend && lf_done) begin lf_pend <= 1'b0; lf_next <= lf_next + 7'd1; end
            if (!cd_pend && !cdef_busy && cd_ok) begin cd_go <= 1'b1; cd_pend <= 1'b1; end
            if (cd_pend && cdef_done) begin cd_pend <= 1'b0; cd_next <= cd_next + 7'd1; end
            if (!sr_pend && !sr_busy && sr_ok) begin sr_go <= 1'b1; sr_pend <= 1'b1; end
            if (sr_pend && sr_done) begin
                sr_pend <= 1'b0;
                if (cdef_en && !sr_sub) sr_sub <= 1'b1;                    // the CDEF frame's rows next
                else begin sr_sub <= 1'b0; sr_next <= sr_next + 7'd1; end
            end
            if (!lr_pend && !lr_busy && lr_ok) begin lr_go <= 1'b1; lr_pend <= 1'b1; end
            if (lr_pend && lr_done_o) begin lr_pend <= 1'b0; lr_next <= lr_next + 7'd1; end
            // frame done: every enabled stage past its last unit
            if (rows_seen && !tile_busy && (lf_next >= n_rows) && (!cdef_en || cd_next >= n_rows) && (!sr_en || sr_next >= n_rows) && (!lr_en || lr_next >= n_stripes)
                && !lf_pend && !cd_pend && !sr_pend && !lr_pend) begin
                seq_active <= 1'b0; frame_busy <= 1'b0; frame_done <= 1'b1; frame_done_lvl <= 1'b1;
            end
        end
    end

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
    // ---- decode-progress trace for tools/vis_render.py: +vis=<file> (docs/vis-trace.md). Simulation only. ----
`ifdef VERILATOR
    int          vis_fd = 0;
    string       vis_path;
    logic [63:0] vis_cyc = 64'd0;
    initial begin
        if ($value$plusargs("vis=%s", vis_path)) vis_fd = $fopen(vis_path, "w");
    end
    always_ff @(posedge clk) begin
        if (rst || frame_start) vis_cyc <= 64'd0; else vis_cyc <= vis_cyc + 64'd1;
        if (vis_fd != 0 && !rst) begin
            if (tx_ack)
                $fwrite(vis_fd, "T %0d %0d %0d %0d %0d %0d\n", vis_cyc, tx_rec.plane, tx_rec.x, tx_rec.y,
                        blk_tables_pkg::tx_width(tx_rec.txsz), blk_tables_pkg::tx_height(tx_rec.txsz));
            if (lf_pend && lf_done)
                $fwrite(vis_fd, "F %0d L %0d %0d\n", vis_cyc, 32'(lf_row_first) * 4, 32'(lf_row_last) * 4 + 3);
            if (cd_pend && cdef_done)
                $fwrite(vis_fd, "F %0d C %0d %0d\n", vis_cyc, 32'(cd_row_first) * 4, 32'(cd_row_last) * 4 + 3);
            if (sr_pend && sr_done)
                $fwrite(vis_fd, "F %0d S %0d %0d\n", vis_cyc, 32'(sr_ly_first), 32'(sr_ly_last));
            if (lr_pend && lr_done_o)
                $fwrite(vis_fd, "F %0d R %0d %0d\n", vis_cyc, 32'(lr_ly_first), 32'(lr_ly_last));
            if (frame_done)
                $fwrite(vis_fd, "E %0d\n", vis_cyc);
        end
    end
`endif
endmodule
