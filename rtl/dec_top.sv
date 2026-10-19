// Intra tile decoder top: tile_syntax (entropy decoding + syntax) -> recon_top (prediction, transforms,
// reconstruction) -> frame_mem, plus mi_store + lf_top (deblocking, on lf_start) + cdef_top (on cdef_start, into a
// second frame buffer) + lr_top (on lr_start, into a third). h_buf selects which picture the host reads. Software supplies the parsed headers (hdr_t / rec_hdr_t), the CDF defaults
// and the tile bytes; the picture (pre loop filter) is read back through the frame buffer's host port.
module dec_top
  import cdf_map_pkg::*;
  import syn_pkg::*;
#(
    parameter int FBX = 10,                  // 1024 x 512 frame buffer: covers the test corpus (<= 640x360 + overhang)
    parameter int FBY = 9,
    parameter int ITX_LANES = 4,             // transform engines: 4 (throughput) or 2 (K26 area, +2.6% cycles)
    parameter int FRAME_URAM = 0,             // 1: fb0 is the on-chip double-pumped UltraRAM (rtl/frame_mem_uram.sv, needs clk2x /
    parameter int FRAME_WIN = 0              // >0 with FRAME_URAM: the frame memory holds this many superblock rows (window, docs/frame-memory.md)
                                             //    phase); fb1 / fb2 are left out, so only the core build (no CDEF / LR / SR) can use it
) (
    input  logic              clk,
    input  logic              clk2x,             // 2x clk, phase-aligned (FRAME_URAM = 1 only; tie low otherwise)
    input  logic              phase,             // 1 at the mid-cycle clk2x edge, 0 at the clk-aligned one
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
    input  logic              rb_en,           // cdf read-back (tile_syntax)
    input  logic [CDF_AW-1:0] rb_addr,
    output logic [245:0]      rb_data,
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
    // window frame memory (FRAME_WIN > 0): superblock rows released for the flush to DDR, and the flushes done
    output logic              row_final,           // 1-cycle pulse: superblock row row_final_idx is final
    output logic [SBW-1:0]    row_final_idx,
    input  logic              flush_done,          // 1-cycle pulse: the oldest released row is in DDR
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
    output logic [11:0]       h_rdata,
    output logic [47:0]       h_rdata4,            // the four samples of h_x's aligned group, lane 0 lowest
    output logic              h_ok                 // the host read sampled at the last edge was served (the UltraRAM shares a slot)
);
    logic ts_done, tx_ack, sb_start, blk_info, pal_hold, pm_plane, blk_ack; logic [MIW-1:0] sb_r, sb_c;
    logic [9:0] q_addr; logic [2:0] q_slot_w, q_slot_r; logic signed [20:0] q_data; logic [5:0] pm_x, pm_y; logic [2:0] pm_idx;
    logic ts_unsup;                                 // tile_syntax's; the window build adds its own limits below
    // Window builds narrow the per-4x4 stores to the rows in play: motion info 32 rows (the MV stack looks one
    // superblock row up), the filters' info and reconstruction's mode map 64 rows (the four superblock rows live
    // between reconstruction and the flush); their row index is simply truncated, i.e. a ring.
    tile_syntax #(.ML2R(FRAME_WIN > 0 ? 5 : FBY - 2), .ML2C(FBX - 2)) u_ts (.clk, .rst, .hdr, .in_data, .in_valid, .in_ready, .in_eos, .def_we, .def_addr, .def_data, .rb_en, .rb_addr, .rb_data,
                      .tile_start, .tile_busy, .tile_done(ts_done), .unsupported(ts_unsup),
                      .sb_start_o(sb_start), .sb_r_o(sb_r), .sb_c_o(sb_c), .blk_info, .blk_done, .blk_rec,
                      .tx_done, .tx_rec, .tx_ack, .lr_done, .lr_rec, .q_slot_w, .q_slot_r, .q_addr, .q_data,
                      .pal_hold, .blk_ack, .pm_plane, .pm_x, .pm_y, .pm_idx);
    // event queue: the syntax decoder runs ahead of reconstruction; a palette block is released (blk_ack) only
    // once reconstruction has drained the queue, because its colour map lives in the syntax stage
    logic ev_valid_rc;                              // ev_valid as the reconstruction stage sees it (held by the window)
    logic ev_valid, ev_pop; logic [1:0] ev_kind; logic [MIW-1:0] ev_sb_r, ev_sb_c; blk_rec_t ev_blk; tx_rec_t ev_tx; logic [3:0] tx_inflight;
    // declared ahead of their first use: Vivado makes an implicit 1-bit net of an identifier used before its declaration
    logic rc_busy, cdef_en, sr_sub;
    logic [MIW-1:0] cd_rd_row, cd_rd_col; logic [SBW-1:0] cdr_row64, cdr_col64; logic [3:0] cdr_val; mi_lf_t cd_rd_info;
    logic [11:0] lf_rdata, cd_rdata, lr0_rdata, sr0_rdata, lr1_rdata, sr1_rdata; logic [47:0] lf_rdata4, cd_rdata4;
    rec_fifo u_q (.clk, .rst, .flush(tile_start), .sb_start, .sb_r, .sb_c, .blk_info, .blk_rec, .tx_done, .tx_rec, .tx_ack, .w_slot(q_slot_w),
                  .ev_valid, .ev_kind, .ev_sb_r, .ev_sb_c, .ev_blk, .ev_tx, .ev_pop, .tx_inflight);
    assign blk_ack = pal_hold && !ev_valid && !rc_busy;

    logic fb_re, fb_we; logic [1:0] fb_plane; logic [FBX-1:0] fb_x; logic [FBY-1:0] fb_y; logic [11:0] fb_wdata, fb_rdata;
    logic fb2_we; logic [1:0] fb2_plane; logic [FBX-1:0] fb2_x; logic [FBY-1:0] fb2_y; logic [11:0] fb2_wdata;
    logic fb4_we; logic [1:0] fb4_plane; logic [FBX-1:0] fb4_x; logic [FBY-1:0] fb4_y; logic [47:0] fb4_wdata, fb_rdata4;
    logic mi_blk_we, mi_blk_busy, mi_tx_we, mi_tx_busy; logic [MIW-1:0] mi_blk_r, mi_blk_c, mi_tx_row, mi_tx_col; logic [5:0] mi_blk_bw4, mi_blk_bh4;
    mi_lf_t mi_blk_data; logic [1:0] mi_tx_plane; logic [4:0] mi_tx_w4, mi_tx_h4, mi_tx_sz;
    logic mi_cd_clr, mi_cd_we; logic [SBW-1:0] mi_cd_row64, mi_cd_col64; logic [3:0] mi_cd_mask; logic [2:0] mi_cd_idx;
    recon_top #(.FBX(FBX), .FBY(FBY), .ML2R(FRAME_WIN > 0 ? 6 : FBY - 2), .ML2C(FBX - 2), .ITX_LANES(ITX_LANES)) u_rc (.clk, .rst, .hdr, .rh,
                                            .ev_valid(ev_valid_rc), .ev_kind, .ev_sb_r, .ev_sb_c, .ev_blk, .ev_tx, .ev_pop,
                                            .q_slot(q_slot_r), .q_addr, .q_data, .pm_plane, .pm_x, .pm_y, .pm_idx, .busy(rc_busy),
                                            .mi_blk_we, .mi_blk_r, .mi_blk_c, .mi_blk_bw4, .mi_blk_bh4, .mi_blk_data, .mi_blk_busy,
                                            .mi_tx_we, .mi_tx_plane, .mi_tx_row, .mi_tx_col, .mi_tx_w4, .mi_tx_h4, .mi_tx_sz, .mi_tx_busy,
                                            .mi_cd_clr, .mi_cd_we, .mi_cd_row64, .mi_cd_col64, .mi_cd_mask, .mi_cd_idx,
                                            .fb_re, .fb_we, .fb_plane, .fb_x, .fb_y, .fb_wdata, .fb_rdata, .fb_rdata4,
                                            .fb2_we, .fb2_plane, .fb2_x, .fb2_y, .fb2_wdata,
                                            .fb4_we, .fb4_plane, .fb4_x, .fb4_y, .fb4_wdata);

    // per-4x4 state + deblocking filter; the filter owns the frame buffer port while it runs
    logic [MIW-1:0] rd_row, rd_col, txr_row, txr_col; mi_lf_t rd_info; logic [1:0] txr_plane; logic [4:0] txr_sz;
    mi_store #(.ML2R(FRAME_WIN > 0 ? 6 : FBY - 2), .ML2C(FBX - 2)) u_mi (.clk, .rst, .blk_we(mi_blk_we), .blk_r(mi_blk_r), .blk_c(mi_blk_c), .blk_bw4(mi_blk_bw4), .blk_bh4(mi_blk_bh4), .blk_data(mi_blk_data), .blk_busy(mi_blk_busy),
                   .tx_we(mi_tx_we), .tx_plane(mi_tx_plane), .tx_row(mi_tx_row), .tx_col(mi_tx_col), .tx_w4(mi_tx_w4), .tx_h4(mi_tx_h4), .tx_sz(mi_tx_sz), .tx_busy(mi_tx_busy),
                   .rd_row(rd_row), .rd_col(rd_col), .rd_info, .rd2_row(cd_rd_row), .rd2_col(cd_rd_col), .rd2_info(cd_rd_info), .txr_plane, .txr_row, .txr_col, .txr_sz,
                   .cd_clr(mi_cd_clr), .cd_we(mi_cd_we), .cd_row64(mi_cd_row64), .cd_col64(mi_cd_col64), .cd_sb128(hdr.sb128), .cd_mask(mi_cd_mask), .cd_idx(mi_cd_idx),
                   .cdr_row64, .cdr_col64, .cdr_val,
                   .lr_we(lr_done), .lr_rec, .lrr_plane, .lrr_row, .lrr_col, .lrr_rec);
    // stage row ranges (whole frame in host-driven mode; the frame sequencer bounds them per superblock row)
    logic [MIW-1:0] lf_row_first, lf_row_last, cd_row_first, cd_row_last; logic [PXW-1:0] lr_ly_first, lr_ly_last, sr_ly_first, sr_ly_last;
    logic lf_go, cd_go, sr_go, lr_go, lf_start_i, cdef_start_i, sr_start_i, lr_start_i, sr_buf_i, lr_from_deblocked_i;
    logic [SBW-1:0] lf_next, cd_next, sr_next, lr_next;   // next superblock row (stripe for LR) each stage will process
    logic [PXW-1:0] lr_ly0, lr_ly1;
    assign lf_start_i = pipe_en ? lf_go : lf_start; assign cdef_start_i = pipe_en ? cd_go : cdef_start;
    assign sr_start_i = pipe_en ? sr_go : sr_start; assign lr_start_i = pipe_en ? lr_go : lr_start;
    logic lf_re, lf_we; logic [1:0] lf_plane; logic [FBX-1:0] lf_x; logic [FBY-1:0] lf_y; logic [11:0] lf_wdata;
    logic lf4_we; logic [FBX-1:0] lf4_x; logic [FBY-1:0] lf4_y; logic [47:0] lf4_wdata;
    lf_top #(.FBX(FBX), .FBY(FBY)) u_lf (.clk, .rst, .hdr, .lh, .start(lf_start_i), .row_first(lf_row_first), .row_last(lf_row_last), .busy(lf_busy), .done(lf_done),
                                         .rd_row, .rd_col, .rd_info, .txr_plane, .txr_row, .txr_col, .txr_sz,
                                         .fb_re(lf_re), .fb_we(lf_we), .fb_plane(lf_plane), .fb_x(lf_x), .fb_y(lf_y), .fb_wdata(lf_wdata), .fb_rdata(lf_rdata),
                                         .fb_rdata4(lf_rdata4), .fb4_we(lf4_we), .fb4_x(lf4_x), .fb4_y(lf4_y), .fb4_wdata(lf4_wdata));
    // CDEF: reads the deblocked frame (fb0, shared port while cdef_busy), writes CdefFrame (fb1)
    logic cd_re, cd_we; logic [1:0] cd_plane, cd_dplane; logic [FBX-1:0] cd_x, cd_dx; logic [FBY-1:0] cd_y, cd_dy; logic [11:0] cd_wdata;
    logic cd4_we; logic [1:0] cd4_plane; logic [FBX-1:0] cd4_x; logic [FBY-1:0] cd4_y; logic [47:0] cd4_wdata;
    cdef_top #(.FBX(FBX), .FBY(FBY)) u_cdef (.clk, .rst, .hdr, .ch, .start(cdef_start_i), .row_first(cd_row_first), .row_last(cd_row_last), .busy(cdef_busy), .done(cdef_done),
                                             .rd_row(cd_rd_row), .rd_col(cd_rd_col), .rd_info(cd_rd_info), .cdr_row64, .cdr_col64, .cdr_val,
                                             .src_re(cd_re), .src_plane(cd_plane), .src_x(cd_x), .src_y(cd_y), .src_rdata(cd_rdata), .src_rdata4(cd_rdata4),
                                             .dst4_we(cd4_we), .dst4_plane(cd4_plane), .dst4_x(cd4_x), .dst4_y(cd4_y), .dst4_wdata(cd4_wdata),
                                             .dst_we(cd_we), .dst_plane(cd_dplane), .dst_x(cd_dx), .dst_y(cd_dy), .dst_wdata(cd_wdata));

    // loop restoration: reads fb0 (deblocked) and fb1 (CdefFrame) while lr_busy, writes fb2 (LrFrame)
    logic [1:0] lrr_plane; logic [SBW-1:0] lrr_row, lrr_col; lr_rec_t lrr_rec;
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
    logic [47:0] h_rdata40, h_rdata41, h_rdata42;
    logic        h_ok0, h_ok1, h_ok2;
    // ---- frame buffers, one port per stage (no sharing: the stages may run concurrently)
    // fb0 = pre-filter / deblocked frame. reads: recon, LF, CDEF, LR source 0 (and source 1 when CDEF did not run), SR; writes: recon, LF, SR
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
    generate if (FRAME_URAM != 0) begin : g_fb0_uram
        // the core build: ports 0 (recon) and 1 (LF) of fb0 on the on-chip memory; the CDEF / LR / SR ports are idle
        logic [47:0] unused_rd4u;
        frame_mem_uram #(.FBX(FBX), .FBY(FBY), .PW(12), .FRAME_WIN(FRAME_WIN)) u_fb (.clk, .clk2x, .phase,
            .re(fb0_re[1:0]), .r_plane(fb0_rplane[3:0]), .r_x(fb0_rx[2*FBX-1:0]), .r_y(fb0_ry[2*FBY-1:0]),
            .rdata(fb0_rdata[23:0]), .rdata4(fb0_rdata4[95:0]),
            .we(fb0_we[1:0]), .w_be(fb0_be[7:0]), .w_plane(fb0_wplane[3:0]), .w_x(fb0_wx[2*FBX-1:0]), .w_y(fb0_wy[2*FBY-1:0]), .w_data(fb0_wdata[95:0]),
            .h_plane, .h_x, .h_y, .h_rdata(h_rdata0), .h_rdata4(h_rdata40), .h_ok(h_ok0));
        assign fb0_rdata[59:24] = '0; assign fb0_rdata4[239:96] = '0; assign unused_rd4u = '0;
    end else begin : g_fb0_model
        frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12), .NRD(5), .NWR(3)) u_fb (.clk, .re(fb0_re), .r_plane(fb0_rplane), .r_x(fb0_rx), .r_y(fb0_ry), .rdata(fb0_rdata), .rdata4(fb0_rdata4),
                                                                           .we(fb0_we), .w_be(fb0_be), .w_plane(fb0_wplane), .w_x(fb0_wx), .w_y(fb0_wy), .w_data(fb0_wdata),
                                                                           .h_plane, .h_x, .h_y, .h_rdata(h_rdata0), .h_rdata4(h_rdata40), .h_ok(h_ok0));
    end endgenerate
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
    generate if (FRAME_URAM != 0) begin : g_fb1_none
        assign fb1_rdata = '0; assign fb1_rdata4 = '0; assign h_rdata1 = '0; assign h_rdata41 = '0; assign h_ok1 = 1'b1;
    end else begin : g_fb1_model
        frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12), .NRD(2), .NWR(2)) u_fb1 (.clk, .re(fb1_re), .r_plane(fb1_rplane), .r_x(fb1_rx), .r_y(fb1_ry), .rdata(fb1_rdata), .rdata4(fb1_rdata4),
                                                                            .we(fb1_we), .w_be(fb1_be), .w_plane(fb1_wplane), .w_x(fb1_wx), .w_y(fb1_wy), .w_data(fb1_wdata),
                                                                            .h_plane, .h_x, .h_y, .h_rdata(h_rdata1), .h_rdata4(h_rdata41), .h_ok(h_ok1));
    end endgenerate
    // fb2 = LrFrame. writes: LR
    logic [11:0] fb2_rdata_u; logic [47:0] fb2_rdata4_u;
    logic w_lr_we; logic [3:0] w_lr_be; logic [1:0] w_lr_pl; logic [FBX-1:0] w_lr_x; logic [FBY-1:0] w_lr_y; logic [47:0] w_lr_d;
    assign w_lr_we = lr_we4; assign w_lr_be = lr_be4; assign w_lr_pl = lr_dplane; assign w_lr_x = lr_dx; assign w_lr_y = lr_dy; assign w_lr_d = lr_wdata4;
    generate if (FRAME_URAM != 0) begin : g_fb2_none
        assign fb2_rdata_u = '0; assign fb2_rdata4_u = '0; assign h_rdata2 = '0; assign h_rdata42 = '0; assign h_ok2 = 1'b1;
    end else begin : g_fb2_model
        frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12), .NRD(1), .NWR(1)) u_fb2 (.clk, .re(1'b0), .r_plane(2'd0), .r_x('0), .r_y('0), .rdata(fb2_rdata_u), .rdata4(fb2_rdata4_u),
                                                                            .we(w_lr_we), .w_be(w_lr_be), .w_plane(w_lr_pl), .w_x(w_lr_x), .w_y(w_lr_y), .w_data(w_lr_d),
                                                                            .h_plane, .h_x, .h_y, .h_rdata(h_rdata2), .h_rdata4(h_rdata42), .h_ok(h_ok2));
    end endgenerate
    assign h_ok     = (h_buf == 2'd2) ? h_ok2     : (h_buf == 2'd1) ? h_ok1     : h_ok0;
    assign h_rdata  = (h_buf == 2'd2) ? h_rdata2  : (h_buf == 2'd1) ? h_rdata1  : h_rdata0;
    assign h_rdata4 = (h_buf == 2'd2) ? h_rdata42 : (h_buf == 2'd1) ? h_rdata41 : h_rdata40;

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
    logic [SBW-1:0]  n_rows, n_stripes;
    logic [7:0]  row_cnt [0:NSBR-1];
    logic        sr_en, lr_en, seq_active, rows_seen;
    // frame geometry and stage enables follow the live header (the host loads it with the first tile, after frame_start)
    assign n_rows = SBW'((hdr.mi_rows + (MIW'(1) << sb_shift) - MIW'(1)) >> sb_shift);
    assign n_stripes = SBW'(((hdr.frame_height + PXW'(7)) >> 6) + PXW'(1));
    assign cdef_en = hdr.enable_cdef && !hdr.coded_lossless && !hdr.allow_intrabc;
    assign sr_en = hdr.use_superres;
    assign lr_en = (hdr.lr_type[1:0] != 2'd0) || (!hdr.mono && ((hdr.lr_type[3:2] != 2'd0) || (hdr.lr_type[5:4] != 2'd0)));
    logic [SBW-1:0]  tile_row_prev; logic tile_row_valid;
    logic        sb_pop; logic [SBW-1:0] sb_pop_row;
    assign sb_shift = hdr.sb128 ? 5'd5 : 5'd4;
    // A superblock event held back from the reconstruction stage by the window (hold_rc, below) still marks the end
    // of the previous row once reconstruction has finished its last block: the hold waits for a flush that waits
    // for the filter that waits for this row count (2026-10-14: a deadlock at the first held row otherwise). The
    // real pop later finds tile_row_prev already on this row and counts nothing twice.
    assign sb_pop = ev_valid && (ev_kind == 2'd2) && (ev_pop || (hold_rc && !rc_busy));
    assign sb_pop_row = SBW'(ev_sb_r >> sb_shift);
    // ---- window frame memory: rows released as they become final, the hold on slot reuse
    // LF(r) done makes row r-1 final (it wrote into that row's last lines); the last pass releases the last row too.
    // Reconstruction may not start superblock row R before row R - FRAME_WIN, the previous occupant of its slot, is
    // flushed (flushed_cnt counts flush_done pulses this frame: rows 0 .. flushed_cnt-1 are in DDR).
    logic [SBW-1:0] rel_upto, rel_next, flushed_cnt;
    logic           hold_rc;
    assign hold_rc = (FRAME_WIN > 0) && ev_valid && (ev_kind == 2'd2) && (32'(sb_pop_row) >= FRAME_WIN)
                     && (32'(sb_pop_row) - FRAME_WIN >= 32'(flushed_cnt));
    assign ev_valid_rc = ev_valid && !hold_rc;
    // the window build's limits (docs/frame-memory.md): 64-pixel superblocks, 4:2:0 or mono, pipelined frames, one tile
    // column (a tile column spans every row, so a window of the full width could never release one), no intra block
    // copy. Sticky until the next frame, so software sees it after the tile (which may well have deadlocked on the hold).
    logic win_unsup;
    always_ff @(posedge clk) begin
        if (rst || frame_start) win_unsup <= 1'b0;
        else if ((FRAME_WIN > 0) && tile_busy && (hdr.sb128 || (!hdr.mono && !(hdr.ssx && hdr.ssy)) || hdr.allow_intrabc || !pipe_en
                                                  || n_tile_cols != 6'd1)) win_unsup <= 1'b1;
    end
    assign unsupported = ts_unsup || win_unsup;
`ifndef SYNTHESIS
    always_ff @(posedge clk) if (FRAME_WIN > 0 && !rst) begin                      // window trace (simulation only)
        if (lf_pend && lf_done) $display("[win] t=%0t LF done row %0d (n_rows %0d) rel_upto->%0d", $time, lf_next, n_rows, (lf_next + SBW'(1) >= n_rows) ? n_rows : lf_next);
        if (row_final) $display("[win] t=%0t row_final %0d", $time, row_final_idx);
        if (flush_done) $display("[win] t=%0t flush_done (flushed_cnt -> %0d)", $time, flushed_cnt + 1);
        if (frame_done) $display("[win] t=%0t frame_done rel_next %0d rel_upto %0d", $time, rel_next, rel_upto);
        if (frame_start) $display("[win] t=%0t frame_start", $time);
        if (hold_rc && !rc_busy && ev_pop) $display("[win] t=%0t hold row %0d (flushed %0d)", $time, sb_pop_row, flushed_cnt);
    end
`endif
    always_ff @(posedge clk) begin
        row_final <= 1'b0;
        if (rst || frame_start) begin
            rel_upto <= '0; rel_next <= '0; flushed_cnt <= '0;
        end else begin
            if (lf_pend && lf_done) rel_upto <= (lf_next + SBW'(1) >= n_rows) ? n_rows : lf_next;
            if (rel_next != rel_upto && rel_next < rel_upto) begin
                row_final <= 1'b1; row_final_idx <= rel_next; rel_next <= rel_next + SBW'(1);
            end
            if (flush_done) flushed_cnt <= flushed_cnt + SBW'(1);
        end
    end
    function automatic logic row_dec(input logic [SBW-1:0] r);   // row r decoded by every tile column
        row_dec = (r >= n_rows) || (row_cnt[(FBY - 6)'(r)] >= 8'(n_tile_cols));
    endfunction
    // stage ranges
    logic [PXW-1:0] sb_px;                                   // luma rows per superblock row
    assign sb_px = hdr.sb128 ? PXW'(128) : PXW'(64);
    always_comb begin
        if (pipe_en) begin
            lf_row_first = MIW'(lf_next) << sb_shift; lf_row_last = (MIW'(lf_next + SBW'(1)) << sb_shift) - MIW'(1);
            if (lf_row_last >= hdr.mi_rows) lf_row_last = hdr.mi_rows - MIW'(1);
            cd_row_first = MIW'(cd_next) << sb_shift; cd_row_last = (MIW'(cd_next + SBW'(1)) << sb_shift) - MIW'(1);
            if (cd_row_last >= hdr.mi_rows) cd_row_last = hdr.mi_rows - MIW'(1);
            sr_ly_first = PXW'(sr_next) * sb_px; sr_ly_last = PXW'(sr_next + SBW'(1)) * sb_px - PXW'(1);
            if (sr_ly_last >= hdr.frame_height) sr_ly_last = hdr.frame_height - PXW'(1);
            lr_ly0 = (lr_next == SBW'(0)) ? PXW'(0) : PXW'(lr_next) * PXW'(64) - PXW'(8);
            lr_ly1 = PXW'(lr_next) * PXW'(64) + PXW'(55);
            if (lr_ly1 >= hdr.frame_height) lr_ly1 = hdr.frame_height - PXW'(1);
            lr_ly_first = lr_ly0; lr_ly_last = lr_ly1;
        end else begin
            lf_row_first = MIW'(0); lf_row_last = hdr.mi_rows - MIW'(1); cd_row_first = MIW'(0); cd_row_last = hdr.mi_rows - MIW'(1);
            sr_ly_first = PXW'(0); sr_ly_last = hdr.frame_height - PXW'(1); lr_ly_first = PXW'(0); lr_ly_last = hdr.frame_height - PXW'(1);
            lr_ly0 = PXW'(0); lr_ly1 = PXW'(0);
        end
    end
    // readiness of the row a stage wants next (rows finish in order, so "stage done up to r" is next > r)
    logic lf_ok, cd_ok, sr_ok, lr_ok;
    logic [SBW-1:0] lr_row;                                   // superblock row holding the stripe's last line
    always_comb begin
        logic [SBW-1:0] r;
        r = lf_next; lf_ok = (r < n_rows) && row_dec(r) && row_dec(r + SBW'(1)) && !mi_blk_busy && !mi_tx_busy;
        r = cd_next; cd_ok = cdef_en && (r < n_rows) && (lf_next > r) && ((r + SBW'(1) >= n_rows) || (lf_next > r + SBW'(1)));
        r = sr_next; sr_ok = sr_en && (r < n_rows) && (cdef_en ? ((cd_next > r) && ((r + SBW'(1) >= n_rows) || (cd_next > r + SBW'(1))))
                                                            : ((lf_next > r) && ((r + SBW'(1) >= n_rows) || (lf_next > r + SBW'(1)))));
        lr_row = SBW'(lr_ly1 >> (hdr.sb128 ? 7 : 6));
        lr_ok = lr_en && (lr_next < n_stripes) && (cdef_en ? (cd_next > lr_row) : ((lf_next > lr_row) && ((lr_row + SBW'(1) >= n_rows) || (lf_next > lr_row + SBW'(1)))))
                      && (!sr_en || (sr_next > lr_row));
    end
    logic lf_pend, cd_pend, sr_pend, lr_pend;             // start issued, done not yet seen
    always_ff @(posedge clk) begin
        lf_go <= 1'b0; cd_go <= 1'b0; sr_go <= 1'b0; lr_go <= 1'b0; frame_done <= 1'b0;
        if (rst) begin
            seq_active <= 1'b0; frame_busy <= 1'b0; frame_done_lvl <= 1'b0; tile_row_valid <= 1'b0; rows_seen <= 1'b0;
            lf_pend <= 1'b0; cd_pend <= 1'b0; sr_pend <= 1'b0; lr_pend <= 1'b0;
            lf_next <= SBW'(0); cd_next <= SBW'(0); sr_next <= SBW'(0); lr_next <= SBW'(0); sr_sub <= 1'b0;
            for (int i = 0; i < NSBR; i++) row_cnt[i] <= 8'd0;
        end else if (frame_start) begin
            seq_active <= 1'b1; frame_busy <= 1'b1; frame_done_lvl <= 1'b0; tile_row_valid <= 1'b0;
            lf_pend <= 1'b0; cd_pend <= 1'b0; sr_pend <= 1'b0; lr_pend <= 1'b0;
            lf_next <= SBW'(0); cd_next <= SBW'(0); sr_next <= SBW'(0); lr_next <= SBW'(0); sr_sub <= 1'b0;
            for (int i = 0; i < NSBR; i++) row_cnt[i] <= 8'd0;
            rows_seen <= 1'b0;
        end else if (seq_active) begin
            if (sb_pop) rows_seen <= 1'b1;
            // row completion per tile
            if (sb_pop) begin
                if (tile_row_valid && sb_pop_row != tile_row_prev) row_cnt[(FBY - 6)'(tile_row_prev)] <= row_cnt[(FBY - 6)'(tile_row_prev)] + 8'd1;
                tile_row_prev <= sb_pop_row; tile_row_valid <= 1'b1;
            end
            if (tile_done && tile_row_valid) begin row_cnt[(FBY - 6)'(tile_row_prev)] <= row_cnt[(FBY - 6)'(tile_row_prev)] + 8'd1; tile_row_valid <= 1'b0; end
            // stage starts
            if (!lf_pend && !lf_busy && lf_ok) begin lf_go <= 1'b1; lf_pend <= 1'b1; end
            if (lf_pend && lf_done) begin lf_pend <= 1'b0; lf_next <= lf_next + SBW'(1); end
            if (!cd_pend && !cdef_busy && cd_ok) begin cd_go <= 1'b1; cd_pend <= 1'b1; end
            if (cd_pend && cdef_done) begin cd_pend <= 1'b0; cd_next <= cd_next + SBW'(1); end
            if (!sr_pend && !sr_busy && sr_ok) begin sr_go <= 1'b1; sr_pend <= 1'b1; end
            if (sr_pend && sr_done) begin
                sr_pend <= 1'b0;
                if (cdef_en && !sr_sub) sr_sub <= 1'b1;                    // the CDEF frame's rows next
                else begin sr_sub <= 1'b0; sr_next <= sr_next + SBW'(1); end
            end
            if (!lr_pend && !lr_busy && lr_ok) begin lr_go <= 1'b1; lr_pend <= 1'b1; end
            if (lr_pend && lr_done_o) begin lr_pend <= 1'b0; lr_next <= lr_next + SBW'(1); end
            // frame done: every enabled stage past its last unit
            if (rows_seen && !tile_busy && (lf_next >= n_rows) && (!cdef_en || cd_next >= n_rows) && (!sr_en || sr_next >= n_rows) && (!lr_en || lr_next >= n_stripes)
                && !lf_pend && !cd_pend && !sr_pend && !lr_pend && (FRAME_WIN == 0 || rel_next == rel_upto)) begin
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
    logic [63:0] vis_cyc;
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
