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
    parameter int FRAME_WIN = 0,             // >0 with FRAME_URAM: the frame memory holds this many superblock rows (window, docs/frame-memory.md)
    parameter int FRAME_WIN1 = -1,           // with FRAME_URAM: fb1 (CdefFrame) as an UltraRAM ring of this many superblock rows (0: the whole
    parameter int FRAME_WIN2 = -1,           //   picture; -1: no fb1 / fb2 at all, the core build); fb2 (LrFrame) likewise (docs/picture-memory.md)
    parameter int INTER = 1,                 // 0: the intra-only build (no ref_fetch / mc_filt / warp_setup, no inter syntax): the K26 board
    parameter int SB128 = 0,                 // 1: the window build takes 128x128 superblocks too (128-line window rows, deeper mode-info rings)
    parameter int TE = 1                     // tile engines: the tile columns of a tile row decoded at once (rtl/tile_engines.sv)
                                             //    phase); fb1 / fb2 are left out, so only the core build (no CDEF / LR / SR) can use it
) (
    input  logic              clk,
    input  logic              clk2x,             // 2x clk, phase-aligned (FRAME_URAM = 1 only; tie low otherwise)
    input  logic              phase,             // 1 at the mid-cycle clk2x edge, 0 at the clk-aligned one
    input  logic              rst,
    input  hdr_t              hdr,
    input  ihdr_t             ihdr,
    input  rec_hdr_t          rh,
    input  lf_hdr_t           lh,
    input  cdef_hdr_t         ch,
    // tile bytes, one feed per tile engine
    input  logic [TE*8-1:0]   in_data,
    input  logic [TE-1:0]     in_valid,
    output logic [TE-1:0]     in_ready,
    input  logic [TE-1:0]     in_eos,
    // cdf defaults
    input  logic              def_we,
    input  logic [CDF_AW-1:0] def_addr,
    input  logic [245:0]      def_data,
    input  logic              rb_en,           // cdf read-back (tile_syntax)
    input  logic [CDF_AW-1:0] rb_addr,
    input  logic [$clog2(TE > 1 ? TE : 2)-1:0] rb_sel,   // the engine read back
    output logic [245:0]      rb_data,
    // inter frames: the motion field band and the previous segmentation map band (host-written in stage 1)
    input  logic              mf_we,
    input  logic [2:0]        mf_wref,
    input  logic [MIW-2:0]    mf_wy8, mf_wx8,
    input  logic              mf_wvalid,
    input  logic signed [15:0] mf_wrow, mf_wcol,
    input  logic              sp_we,
    input  logic [MIW-1:0]    sp_wrow,
    input  logic [MIW-5:0]    sp_wwc,
    input  logic [47:0]       sp_wdata,
    // control
    // ---- inter prediction (stage 2a): the reference slot table (dec_host registers) and ref_fetch's AXI read master
    input  logic [20:0]       ref_slot,            // slot of LAST_FRAME + i at [3i +: 3]
    input  logic [1727:0]     ref_planes,          // slot s, plane p: {stride[31:0], base[39:0]} at [72 (3s + p) +: 72]
    input  logic [255:0]      ref_dims,            // slot s: {frame height[15:0], upscaled width[15:0]} at [32s +: 32]
    input  logic [255:0]      ref_scales,          // slot s: {yScale[15:0], xScale[15:0]} (7.11.3.3, 16384 = unscaled) at [32s +: 32]
    input  logic              ref_fmt16,
    input  logic [215:0]      cur_planes,          // the current picture's flush layout (dec_host DMA plane registers): intra block copy
    input  logic              cur_fmt16,
    output logic [39:0]       m_axi_araddr,
    output logic [7:0]        m_axi_arlen,
    output logic [2:0]        m_axi_arsize,
    output logic [1:0]        m_axi_arburst,
    output logic [3:0]        m_axi_arcache,
    output logic [2:0]        m_axi_arprot,
    output logic              m_axi_arvalid,
    input  logic              m_axi_arready,
    input  logic [63:0]       m_axi_rdata,
    input  logic [1:0]        m_axi_rresp,
    input  logic              m_axi_rlast,
    input  logic              m_axi_rvalid,
    output logic              m_axi_rready,
    input  logic              tile_start,
    input  logic [TE-1:0]     eng_mask,            // engines of this tile row (0: engine 0, the header's columns)
    input  logic [TE*MIW-1:0] col_start,           // MiColStart / MiColEnd per engine
    input  logic [TE*MIW-1:0] col_end,
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
    // frame-state sideband (rtl/frame_state_w.sv, INTER builds): finished rows for dec_host's sideband DMA + its read port
    // frame-state loader (rtl/fs_load.sv, stage B): the segment map and the CDF rows from the reference slot's areas
    input  logic              ld_start,
    input  logic [1:0]        ld_cfg,
    input  logic [39:0]       ld_seg_base,
    input  logic [15:0]       ld_seg_rowb,
    input  logic [39:0]       ld_cdf_base,
    output logic              ld_busy,
    output logic              ld_done,
    output logic              ld_err,
    output logic              fs_row_ready,
    output logic [SBW-1:0]    fs_row_idx,
    input  logic              fs_done,
    input  logic              fs_rd_sel,
    input  logic [4:0]        fs_rd_line,
    input  logic [8:0]        fs_rd_beat,
    input  logic              fs_rd_buf,
    output logic [63:0]       fs_rd_data,
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
    output logic              h_ok,                // the host read sampled at the last edge was served (the UltraRAM shares a slot)
    output logic [1:0]        final_buf            // the buffer holding the final picture of this frame (2 with LR, 1 with CDEF, else 0)
);
    logic ts_done, tx_ack, pm_plane;
    logic [9:0] q_addr; logic [2:0] q_slot_w, q_slot_r; logic signed [20:0] q_data; logic [5:0] pm_x, pm_y; logic [2:0] pm_idx;
    logic ts_unsup;                                 // tile_syntax's; the window build adds its own limits below
    // Window builds narrow the per-4x4 stores to the rows in play: motion info 32 rows (the MV stack looks one
    // superblock row up), the filters' info and reconstruction's mode map 64 rows (the four superblock rows live
    // between reconstruction and the flush); their row index is simply truncated, i.e. a ring.
    // the motion-field and segmentation-map bands hold the PREVIOUS frame's whole picture (the host writes them before the
    // frame), so they are full-height whenever inter is built; the window build trims them only at INTER = 0 (unused there)
    // ---- the frame-state loader (rtl/fs_load.sv): the read channel belongs to it while it runs (before the first tile),
    // to ref_fetch otherwise; its band and CDF writes merge with the host's register writes
    logic           def_we_i, sp_we_i;
    logic [CDF_AW-1:0] def_addr_i; logic [245:0] def_data_i;
    logic [MIW-1:0] sp_wrow_i; logic [MIW-5:0] sp_wwc_i; logic [47:0] sp_wdata_i;
    logic [39:0]    rf_araddr, ld_araddr; logic [7:0] rf_arlen, ld_arlen; logic [2:0] rf_arsize, ld_arsize; logic [1:0] rf_arburst, ld_arburst;
    logic [3:0]     rf_arcache, ld_arcache; logic [2:0] rf_arprot, ld_arprot; logic rf_arvalid, ld_arvalid, rf_arready, ld_arready, rf_rvalid, ld_rvalid, rf_rready, ld_rready;
    logic           ld_sp_we, ld_def_we; logic [MIW-1:0] ld_sp_wrow; logic [MIW-5:0] ld_sp_wwc; logic [47:0] ld_sp_wdata; logic [CDF_AW-1:0] ld_def_addr; logic [245:0] ld_def_data;
    if (INTER != 0 && FRAME_WIN > 0) begin : g_fsl
        fs_load u_fsl (.clk, .rst, .hdr, .ld_start, .ld_cfg, .ld_seg_base, .ld_seg_rowb, .ld_cdf_base, .ld_busy, .ld_done, .ld_err,
            .m_axi_araddr(ld_araddr), .m_axi_arlen(ld_arlen), .m_axi_arsize(ld_arsize), .m_axi_arburst(ld_arburst), .m_axi_arcache(ld_arcache), .m_axi_arprot(ld_arprot),
            .m_axi_arvalid(ld_arvalid), .m_axi_arready(ld_arready), .m_axi_rdata, .m_axi_rresp, .m_axi_rlast, .m_axi_rvalid(ld_rvalid), .m_axi_rready(ld_rready),
            .sp_we(ld_sp_we), .sp_wrow(ld_sp_wrow), .sp_wwc(ld_sp_wwc), .sp_wdata(ld_sp_wdata), .def_we(ld_def_we), .def_addr(ld_def_addr), .def_data(ld_def_data));
    end else begin : g_nofsl
        assign ld_busy = 1'b0; assign ld_done = 1'b0; assign ld_err = 1'b0;
        assign ld_araddr = '0; assign ld_arlen = '0; assign ld_arsize = '0; assign ld_arburst = '0; assign ld_arcache = '0; assign ld_arprot = '0;
        assign ld_arvalid = 1'b0; assign ld_rready = 1'b0;
        assign ld_sp_we = 1'b0; assign ld_sp_wrow = '0; assign ld_sp_wwc = '0; assign ld_sp_wdata = '0; assign ld_def_we = 1'b0; assign ld_def_addr = '0; assign ld_def_data = '0;
    end
    assign m_axi_araddr  = ld_busy ? ld_araddr  : rf_araddr;
    assign m_axi_arlen   = ld_busy ? ld_arlen   : rf_arlen;
    assign m_axi_arsize  = ld_busy ? ld_arsize  : rf_arsize;
    assign m_axi_arburst = ld_busy ? ld_arburst : rf_arburst;
    assign m_axi_arcache = ld_busy ? ld_arcache : rf_arcache;
    assign m_axi_arprot  = ld_busy ? ld_arprot  : rf_arprot;
    assign m_axi_arvalid = ld_busy ? ld_arvalid : rf_arvalid;
    assign m_axi_rready  = ld_busy ? ld_rready  : rf_rready;
    assign ld_arready = ld_busy && m_axi_arready;  assign rf_arready = !ld_busy && m_axi_arready;
    assign ld_rvalid  = ld_busy && m_axi_rvalid;   assign rf_rvalid  = !ld_busy && m_axi_rvalid;
    assign def_we_i   = def_we | ld_def_we;
    assign def_addr_i = ld_def_we ? ld_def_addr : def_addr;
    assign def_data_i = ld_def_we ? ld_def_data : def_data;
    assign sp_we_i    = sp_we | ld_sp_we;
    assign sp_wrow_i  = ld_sp_we ? ld_sp_wrow : sp_wrow;
    assign sp_wwc_i   = ld_sp_we ? ld_sp_wwc : sp_wwc;
    assign sp_wdata_i = ld_sp_we ? ld_sp_wdata : sp_wdata;
    tile_engines #(.TE(TE), .INTER(INTER), .ML2R(FRAME_WIN > 0 ? 5 + SB128 : FBY - 2), .ML2C(FBX - 2), .MFL2R((FRAME_WIN > 0 && INTER == 0) ? 4 + SB128 : FBY - 3), .MFL2C(FBX - 3),
                   .SPL2R((FRAME_WIN > 0 && INTER == 0) ? 5 + SB128 : FBY - 2), .SPL2W(FBX - 6)) u_ts (
                   .clk, .rst, .hdr, .ihdr, .eng_mask, .col_start, .col_end,
                   .in_data, .in_valid, .in_ready, .in_eos, .def_we(def_we_i), .def_addr(def_addr_i), .def_data(def_data_i), .rb_en, .rb_addr, .rb_sel, .rb_data,
                   .mf_we, .mf_wref, .mf_wy8, .mf_wx8, .mf_wvalid, .mf_wrow, .mf_wcol, .sp_we(sp_we_i), .sp_wrow(sp_wrow_i), .sp_wwc(sp_wwc_i), .sp_wdata(sp_wdata_i),
                   .tile_start, .tile_busy, .tile_done(ts_done), .unsupported(ts_unsup),
                   .ev_valid, .ev_kind, .ev_sb_r, .ev_sb_c, .ev_blk, .ev_tx, .ev_pop, .tx_inflight, .rc_busy, .rc_col_start, .rc_col_end,
                   .q_slot_r, .q_addr, .q_data, .pm_plane, .pm_x, .pm_y, .pm_idx,
                   .lr_done, .lr_rec, .blk_done, .blk_rec, .tx_done, .tx_rec, .tx_ack);
    // event queue (inside tile_engines, one per engine): the syntax decoder runs ahead of reconstruction; a palette
    // block is released (blk_ack, per engine inside tile_engines) only once its engine's queue has drained and
    // reconstruction is idle, because its colour map lives in the syntax stage
    logic ev_valid_rc;                              // ev_valid as the reconstruction stage sees it (held by the window)
    logic ev_valid, ev_pop; logic [1:0] ev_kind; logic [MIW-1:0] ev_sb_r, ev_sb_c; blk_rec_t ev_blk; tx_rec_t ev_tx; logic [3:0] tx_inflight;
    // declared ahead of their first use: Vivado makes an implicit 1-bit net of an identifier used before its declaration
    logic rc_busy, cdef_en, sr_sub;
    logic [MIW-1:0] cd_rd_row, cd_rd_col; logic [SBW-1:0] cdr_row64, cdr_col64; logic [3:0] cdr_val; mi_lf_t cd_rd_info;
    logic [11:0] lf_rdata, cd_rdata, lr0_rdata, sr0_rdata, lr1_rdata, sr1_rdata; logic [47:0] lf_rdata4, cd_rdata4;

    logic fb_re, fb_we; logic [1:0] fb_plane; logic [FBX-1:0] fb_x; logic [FBY-1:0] fb_y; logic [11:0] fb_wdata, fb_rdata;
    logic fb2_we; logic [1:0] fb2_plane; logic [FBX-1:0] fb2_x; logic [FBY-1:0] fb2_y; logic [11:0] fb2_wdata;
    logic fb4_we; logic [1:0] fb4_plane; logic [FBX-1:0] fb4_x; logic [FBY-1:0] fb4_y; logic [47:0] fb4_wdata, fb_rdata4;
    logic [3:0] fb4_be; logic mc_unsup;
    logic mi_blk_we, mi_blk_busy, mi_tx_we, mi_tx_busy; logic [MIW-1:0] mi_blk_r, mi_blk_c, mi_tx_row, mi_tx_col; logic [5:0] mi_blk_bw4, mi_blk_bh4;
    mi_lf_t mi_blk_data; logic [1:0] mi_tx_plane; logic [4:0] mi_tx_w4, mi_tx_h4, mi_tx_sz;
    logic mi_cd_clr, mi_cd_we; logic [SBW-1:0] mi_cd_row64, mi_cd_col64; logic [3:0] mi_cd_mask; logic [2:0] mi_cd_idx;
    // reconstruction's header: the frame's, with the tile column bounds of the engine whose block it is working on
    // (is_inside for neighbour availability, the superblock clip at the tile's right edge); the arbiter switches
    // engines only while reconstruction is idle
    logic [MIW-1:0] rc_col_start, rc_col_end;
    hdr_t hdr_rc;
    always_comb begin hdr_rc = hdr; hdr_rc.mi_col_start = rc_col_start; hdr_rc.mi_col_end = rc_col_end; end
    recon_top #(.FBX(FBX), .FBY(FBY), .ML2R(FRAME_WIN > 0 ? 6 + SB128 : FBY - 2), .ML2C(FBX - 2), .ITX_LANES(ITX_LANES), .INTER(INTER), .FRAME_WIN(FRAME_WIN)) u_rc (.clk, .rst, .hdr(hdr_rc), .rh,
                                            .ev_valid(ev_valid_rc), .ev_kind, .ev_sb_r, .ev_sb_c, .ev_blk, .ev_tx, .ev_pop,
                                            .q_slot(q_slot_r), .q_addr, .q_data, .pm_plane, .pm_x, .pm_y, .pm_idx, .busy(rc_busy),
                                            .mi_blk_we, .mi_blk_r, .mi_blk_c, .mi_blk_bw4, .mi_blk_bh4, .mi_blk_data, .mi_blk_busy,
                                            .mi_tx_we, .mi_tx_plane, .mi_tx_row, .mi_tx_col, .mi_tx_w4, .mi_tx_h4, .mi_tx_sz, .mi_tx_busy,
                                            .mi_cd_clr, .mi_cd_we, .mi_cd_row64, .mi_cd_col64, .mi_cd_mask, .mi_cd_idx,
                                            .fb_re, .fb_we, .fb_plane, .fb_x, .fb_y, .fb_wdata, .fb_rdata, .fb_rdata4,
                                            .fb2_we, .fb2_plane, .fb2_x, .fb2_y, .fb2_wdata,
                                            .fb4_we, .fb4_plane, .fb4_x, .fb4_y, .fb4_wdata, .fb4_be,
                                            .ihdr, .ref_slot, .ref_planes, .ref_dims, .ref_scales, .ref_fmt16, .cur_planes, .cur_fmt16, .mc_unsup,
                                            .m_axi_araddr(rf_araddr), .m_axi_arlen(rf_arlen), .m_axi_arsize(rf_arsize), .m_axi_arburst(rf_arburst), .m_axi_arcache(rf_arcache), .m_axi_arprot(rf_arprot),
                                            .m_axi_arvalid(rf_arvalid), .m_axi_arready(rf_arready), .m_axi_rdata, .m_axi_rresp, .m_axi_rlast, .m_axi_rvalid(rf_rvalid), .m_axi_rready(rf_rready));

    // per-4x4 state + deblocking filter; the filter owns the frame buffer port while it runs
    logic [MIW-1:0] rd_row, rd_col, txr_row, txr_col; mi_lf_t rd_info; logic [1:0] txr_plane; logic [4:0] txr_sz;
    // the per-4x4 filter state (and CDEF indices, LR units) is a ring of 2^ML2R MI rows: every row the fb0 ring can hold
    // live (FRAME_WIN superblock rows of 16: recon(r) down to LR(r-3)'s stripe) must fit, or recon overwrites the row LR
    // is reading (2026-10-24: 64 MI rows with a 5-row ring lost row 0's LR units to row 4)
    mi_store #(.ML2R(FRAME_WIN > 0 ? $clog2(FRAME_WIN * (SB128 != 0 ? 32 : 16)) : FBY - 2), .ML2C(FBX - 2)) u_mi (.clk, .rst, .blk_we(mi_blk_we), .blk_r(mi_blk_r), .blk_c(mi_blk_c), .blk_bw4(mi_blk_bw4), .blk_bh4(mi_blk_bh4), .blk_data(mi_blk_data), .blk_busy(mi_blk_busy),
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

    logic sr_re, sr_we; logic [1:0] sr_plane, sr_dplane; logic [FBX-1:0] sr_sx, sr_dx; logic [FBY-1:0] sr_sy, sr_dy; logic [11:0] sr_wdata;
    logic sr0, sr1;
    assign sr_buf_i = pipe_en ? sr_sub : sr_buf;
    assign lr_from_deblocked_i = pipe_en ? !cdef_en : lr_from_deblocked;
    assign sr0 = sr_busy && !sr_buf_i;
    assign sr1 = sr_busy && sr_buf_i;
    sr_top #(.FBX(FBX), .FBY(FBY)) u_sr (.clk, .rst, .hdr, .lh, .start(sr_start_i), .ly_first(sr_ly_first), .ly_last(sr_ly_last), .busy(sr_busy), .done(sr_done),
                                         .s_re(sr_re), .plane(sr_plane), .s_x(sr_sx), .s_y(sr_sy), .s_rdata(sr_buf_i ? sr1_rdata : sr0_rdata),
                                         .d_we(sr_we), .d_plane(sr_dplane), .d_x(sr_dx), .d_y(sr_dy), .d_wdata(sr_wdata));
    logic        sr_en;                                      // super-resolution this frame (assigned with the sequencer below)
    logic [11:0] h_rdata0, h_rdata1, h_rdata2, h_rdata3;
    logic [47:0] h_rdata40, h_rdata41, h_rdata42, h_rdata43;
    logic        h_ok0, h_ok1, h_ok2, h_ok3;
    // RING: the FPGA build with every frame buffer on UltraRAM rings (docs/picture-memory.md, step A). FRAME_WIN1 < 0 is
    // the core build (CDEF / LR / SR nulled, fb0 alone).
    localparam bit RING = (FRAME_URAM != 0) && (FRAME_WIN1 >= 0);
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
                                                .we4(fb4_we), .plane4(fb4_plane), .x4(fb4_x), .y4(fb4_y), .d4(fb4_wdata), .be4(fb4_be),
                                                .we(w_rc_we), .be(w_rc_be), .plane(w_rc_pl), .x(w_rc_x), .y(w_rc_y), .data(w_rc_d));
    frame_mem_w #(.FBX(FBX), .FBY(FBY)) u_w_lf (.we1(lf_we), .plane1(lf_plane), .x1(lf_x), .y1(lf_y), .d1(lf_wdata),
                                                .we4(lf4_we), .plane4(lf_plane), .x4(lf4_x), .y4(lf4_y), .d4(lf4_wdata),
                                                .we(w_lf_we), .be(w_lf_be), .plane(w_lf_pl), .x(w_lf_x), .y(w_lf_y), .data(w_lf_d));
    frame_mem_w #(.FBX(FBX), .FBY(FBY)) u_w_sr0 (.we1(sr0 && sr_we), .plane1(sr_dplane), .x1(sr_dx), .y1(sr_dy), .d1(sr_wdata),
                                                 .we4(1'b0), .plane4(2'd0), .x4('0), .y4('0), .d4('0),
                                                 .we(w_sr0_we), .be(w_sr0_be), .plane(w_sr0_pl), .x(w_sr0_x), .y(w_sr0_y), .data(w_sr0_d));
    assign fb0_we = {w_sr0_we, w_lf_we, w_rc_we}; assign fb0_be = {w_sr0_be, w_lf_be, w_rc_be}; assign fb0_wplane = {w_sr0_pl, w_lf_pl, w_rc_pl};
    assign fb0_wx = {w_sr0_x, w_lf_x, w_rc_x}; assign fb0_wy = {w_sr0_y, w_lf_y, w_rc_y}; assign fb0_wdata = {w_sr0_d, w_lf_d, w_rc_d};
    generate if (FRAME_URAM != 0) begin : g_fb0_uram
        // fb0 on the on-chip memory: two read and two write ports (double-pumped), recon (port 0) and LF (port 1).
        // The filter stages' three read ports and SR's write port need more: in the RING build they are served by
        //   fb0m, a mirror of fb0 (written by recon and LF alongside fb0, same ring geometry) read by CDEF and by
        //        SR (the deblocked rows it upscales) or, without super-res, by LR source 0 (and source 1 when CDEF
        //        is off: lr_top asks for both sources at one position, one read serves them);
        //   fb0s, the upscaled deblocked rows SR writes (in the model build SR upscales fb0 in place), read by LR
        //        source 0 under super-res and the host when nothing follows (final_buf 3; FRAME_WIN1 rows).
        // The core build (FRAME_WIN1 < 0) leaves both out: CDEF / LR / SR are nulled there and their ports read 0.
        logic [47:0] unused_rd4u;
        frame_mem_uram #(.FBX(FBX), .FBY(FBY), .PW(12), .FRAME_WIN(FRAME_WIN), .SB128(SB128)) u_fb (.clk, .clk2x, .phase, .sb128(hdr.sb128),
            .re(fb0_re[1:0]), .r_plane(fb0_rplane[3:0]), .r_x(fb0_rx[2*FBX-1:0]), .r_y(fb0_ry[2*FBY-1:0]),
            .rdata(fb0_rdata[23:0]), .rdata4(fb0_rdata4[95:0]),
            .we(fb0_we[1:0]), .w_be(fb0_be[7:0]), .w_plane(fb0_wplane[3:0]), .w_x(fb0_wx[2*FBX-1:0]), .w_y(fb0_wy[2*FBY-1:0]), .w_data(fb0_wdata[95:0]),
            .h_plane, .h_x, .h_y, .h_rdata(h_rdata0), .h_rdata4(h_rdata40), .h_ok(h_ok0));
        if (RING) begin : g_mirror
            logic [23:0] m_rdata, s_rdata; logic [95:0] m_rdata4, s_rdata4;
            logic [11:0] unused_hm; logic [47:0] unused_hm4; logic unused_hokm;
            frame_mem_uram #(.FBX(FBX), .FBY(FBY), .PW(12), .FRAME_WIN(FRAME_WIN), .SB128(SB128)) u_fbm (.clk, .clk2x, .phase, .sb128(hdr.sb128),
                .re({sr_en ? fb0_re[4] : fb0_re[3], fb0_re[2]}),
                .r_plane({sr_en ? fb0_rplane[9:8] : fb0_rplane[7:6], fb0_rplane[5:4]}),
                .r_x({sr_en ? fb0_rx[5*FBX-1 -: FBX] : fb0_rx[4*FBX-1 -: FBX], fb0_rx[3*FBX-1 -: FBX]}),
                .r_y({sr_en ? fb0_ry[5*FBY-1 -: FBY] : fb0_ry[4*FBY-1 -: FBY], fb0_ry[3*FBY-1 -: FBY]}),
                .rdata(m_rdata), .rdata4(m_rdata4),
                .we(fb0_we[1:0]), .w_be(fb0_be[7:0]), .w_plane(fb0_wplane[3:0]), .w_x(fb0_wx[2*FBX-1:0]), .w_y(fb0_wy[2*FBY-1:0]), .w_data(fb0_wdata[95:0]),
                .h_plane(2'd0), .h_x('0), .h_y('0), .h_rdata(unused_hm), .h_rdata4(unused_hm4), .h_ok(unused_hokm));
            frame_mem_uram #(.FBX(FBX), .FBY(FBY), .PW(12), .FRAME_WIN(FRAME_WIN1), .SB128(SB128)) u_fbs (.clk, .clk2x, .phase, .sb128(hdr.sb128),
                .re({1'b0, sr_en && fb0_re[3]}), .r_plane({2'd0, fb0_rplane[7:6]}),
                .r_x({FBX'(0), fb0_rx[4*FBX-1 -: FBX]}), .r_y({FBY'(0), fb0_ry[4*FBY-1 -: FBY]}),
                .rdata(s_rdata), .rdata4(s_rdata4),
                .we({1'b0, fb0_we[2]}), .w_be({4'd0, fb0_be[11:8]}), .w_plane({2'd0, fb0_wplane[5:4]}),
                .w_x({FBX'(0), fb0_wx[3*FBX-1 -: FBX]}), .w_y({FBY'(0), fb0_wy[3*FBY-1 -: FBY]}), .w_data({48'd0, fb0_wdata[143:96]}),
                .h_plane, .h_x, .h_y, .h_rdata(h_rdata3), .h_rdata4(h_rdata43), .h_ok(h_ok3));
            assign fb0_rdata[35:24]   = m_rdata[11:0];                                   // CDEF
            assign fb0_rdata[47:36]   = sr_en ? s_rdata[11:0] : m_rdata[23:12];           // LR source 0
            assign fb0_rdata[59:48]   = m_rdata[23:12];                                  // SR
            assign fb0_rdata4[143:96]  = m_rdata4[47:0];
            assign fb0_rdata4[191:144] = sr_en ? s_rdata4[47:0] : m_rdata4[95:48];
            assign fb0_rdata4[239:192] = m_rdata4[95:48];
            assign unused_rd4u = s_rdata4[95:48];
        end else begin : g_core
            assign fb0_rdata[59:24] = '0; assign fb0_rdata4[239:96] = '0; assign unused_rd4u = '0;
            assign h_rdata3 = '0; assign h_rdata43 = '0; assign h_ok3 = 1'b1;
        end
    end else begin : g_fb0_model
        assign h_rdata3 = '0; assign h_rdata43 = '0; assign h_ok3 = 1'b1;
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
    frame_mem_w #(.FBX(FBX), .FBY(FBY)) u_w_sr1 (.we1(sr1 && sr_we), .plane1(sr_dplane), .x1(sr_dx), .y1(sr_dy), .d1(sr_wdata),
                                                 .we4(1'b0), .plane4(2'd0), .x4('0), .y4('0), .d4('0),
                                                 .we(w_sr1_we), .be(w_sr1_be), .plane(w_sr1_pl), .x(w_sr1_x), .y(w_sr1_y), .data(w_sr1_d));
    assign fb1_we = {w_sr1_we, w_cd_we}; assign fb1_be = {w_sr1_be, w_cd_be}; assign fb1_wplane = {w_sr1_pl, w_cd_pl};
    assign fb1_wx = {w_sr1_x, w_cd_x}; assign fb1_wy = {w_sr1_y, w_cd_y}; assign fb1_wdata = {w_sr1_d, w_cd_d};
    generate if (FRAME_URAM != 0 && FRAME_WIN1 < 0) begin : g_fb1_none
        assign fb1_rdata = '0; assign fb1_rdata4 = '0; assign h_rdata1 = '0; assign h_rdata41 = '0; assign h_ok1 = 1'b1;
    end else if (FRAME_URAM != 0) begin : g_fb1_uram
        // the CDEF frame as an UltraRAM ring (docs/picture-memory.md, step A): readers LR source 1 and SR, writers CDEF and SR
        frame_mem_uram #(.FBX(FBX), .FBY(FBY), .PW(12), .FRAME_WIN(FRAME_WIN1), .SB128(SB128)) u_fb1 (.clk, .clk2x, .phase, .sb128(hdr.sb128),
            .re(fb1_re), .r_plane(fb1_rplane), .r_x(fb1_rx), .r_y(fb1_ry), .rdata(fb1_rdata), .rdata4(fb1_rdata4),
            .we(fb1_we), .w_be(fb1_be), .w_plane(fb1_wplane), .w_x(fb1_wx), .w_y(fb1_wy), .w_data(fb1_wdata),
            .h_plane, .h_x, .h_y, .h_rdata(h_rdata1), .h_rdata4(h_rdata41), .h_ok(h_ok1));
    end else begin : g_fb1_model
        frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12), .NRD(2), .NWR(2)) u_fb1 (.clk, .re(fb1_re), .r_plane(fb1_rplane), .r_x(fb1_rx), .r_y(fb1_ry), .rdata(fb1_rdata), .rdata4(fb1_rdata4),
                                                                            .we(fb1_we), .w_be(fb1_be), .w_plane(fb1_wplane), .w_x(fb1_wx), .w_y(fb1_wy), .w_data(fb1_wdata),
                                                                            .h_plane, .h_x, .h_y, .h_rdata(h_rdata1), .h_rdata4(h_rdata41), .h_ok(h_ok1));
    end endgenerate
    // fb2 = LrFrame. writes: LR
    logic [11:0] fb2_rdata_u; logic [47:0] fb2_rdata4_u;
    logic w_lr_we; logic [3:0] w_lr_be; logic [1:0] w_lr_pl; logic [FBX-1:0] w_lr_x; logic [FBY-1:0] w_lr_y; logic [47:0] w_lr_d;
    assign w_lr_we = lr_we4; assign w_lr_be = lr_be4; assign w_lr_pl = lr_dplane; assign w_lr_x = lr_dx; assign w_lr_y = lr_dy; assign w_lr_d = lr_wdata4;
    generate if (FRAME_URAM != 0 && FRAME_WIN2 < 0) begin : g_fb2_none
        assign fb2_rdata_u = '0; assign fb2_rdata4_u = '0; assign h_rdata2 = '0; assign h_rdata42 = '0; assign h_ok2 = 1'b1;
    end else if (FRAME_URAM != 0) begin : g_fb2_uram
        // the LR frame as an UltraRAM ring: one writer (LR), no decoder-side reader, so the host port is always served
        frame_mem_uram #(.FBX(FBX), .FBY(FBY), .PW(12), .FRAME_WIN(FRAME_WIN2), .SB128(SB128)) u_fb2 (.clk, .clk2x, .phase, .sb128(hdr.sb128),
            .re(2'b00), .r_plane(4'd0), .r_x('0), .r_y('0), .rdata({fb2_rdata4_u[11:0], fb2_rdata_u}), .rdata4(),
            .we({1'b0, w_lr_we}), .w_be({4'd0, w_lr_be}), .w_plane({2'd0, w_lr_pl}), .w_x({FBX'(0), w_lr_x}), .w_y({FBY'(0), w_lr_y}), .w_data({48'd0, w_lr_d}),
            .h_plane, .h_x, .h_y, .h_rdata(h_rdata2), .h_rdata4(h_rdata42), .h_ok(h_ok2));
        assign fb2_rdata4_u[47:12] = '0;
    end else begin : g_fb2_model
        frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12), .NRD(1), .NWR(1)) u_fb2 (.clk, .re(1'b0), .r_plane(2'd0), .r_x('0), .r_y('0), .rdata(fb2_rdata_u), .rdata4(fb2_rdata4_u),
                                                                            .we(w_lr_we), .w_be(w_lr_be), .w_plane(w_lr_pl), .w_x(w_lr_x), .w_y(w_lr_y), .w_data(w_lr_d),
                                                                            .h_plane, .h_x, .h_y, .h_rdata(h_rdata2), .h_rdata4(h_rdata42), .h_ok(h_ok2));
    end endgenerate
    assign h_ok     = (h_buf == 2'd3) ? h_ok3     : (h_buf == 2'd2) ? h_ok2     : (h_buf == 2'd1) ? h_ok1     : h_ok0;
    assign h_rdata  = (h_buf == 2'd3) ? h_rdata3  : (h_buf == 2'd2) ? h_rdata2  : (h_buf == 2'd1) ? h_rdata1  : h_rdata0;
    assign h_rdata4 = (h_buf == 2'd3) ? h_rdata43 : (h_buf == 2'd2) ? h_rdata42 : (h_buf == 2'd1) ? h_rdata41 : h_rdata40;

    // ================================================================ frame sequencer (pipe_en): stages per superblock row
    // Row r of the frame is decoded once every tile column has reconstructed past it (row_cnt[r] >= n_tile_cols): a pass of
    // the reconstruction stream (the first superblock of the next row popped, or tile_done) counts for the engines of
    // the tile row (n_pass): the tile engines merge the columns they run at once into one raster-ordered stream, so a
    // row of a two-engine tile row passes once and counts twice, while tiles fed one at a time (run_tile) pass it once
    // per column (2026-10-28: counting one pass per row started LF on a row whose other column was still undecoded,
    // seed 9 of the dec_host pipe suite: the tile edge deblocked against stale samples).
    // Rules (see notes/projects/open-av1-decoder-throughput.md, step 7): LF(r) after row r+1 is decoded [vertical
    // edges of row r change its last line, which row r+1 predicts from]; CDEF(r) after LF(r+1) [window reads 2 lines
    // into row r+1, and LF(r+1)'s boundary edge changes row r's last lines]; SR(r) after CDEF(r+1) [CDEF reads row r's
    // last 2 lines at frame width] (or after LF(r+1) without CDEF); LR(stripe s) after CDEF and SR of the rows the
    // stripe touches. The last row takes "r+1" as already done.
    localparam int NSBR = 1 << (FBY - 6);                 // superblock rows (64-pixel; a 128 superblock counts as two entries' worth of mi rows)
    logic [4:0]  sb_shift;                                // log2(mi rows per superblock row)
    logic [SBW-1:0]  n_rows, n_stripes;
    logic [7:0]  row_cnt [0:NSBR-1];
    logic [7:0]  n_pass;                                    // tile columns a pass of the reconstruction stream stands for: the
    assign n_pass = (eng_mask == '0) ? 8'd1 : 8'($countones(eng_mask));   // engines of the tile row (one, when tiles come one at a time)
    logic        lr_en, seq_active, rows_seen;
    // frame geometry and stage enables follow the live header (the host loads it with the first tile, after frame_start)
    assign n_rows = SBW'((hdr.mi_rows + (MIW'(1) << sb_shift) - MIW'(1)) >> sb_shift);
    assign n_stripes = SBW'(((hdr.frame_height + PXW'(7)) >> 6) + PXW'(1));
    assign cdef_en = hdr.enable_cdef && !hdr.coded_lossless && !hdr.allow_intrabc;
    assign sr_en = hdr.use_superres;
    assign lr_en = (hdr.lr_type[1:0] != 2'd0) || (!hdr.mono && ((hdr.lr_type[3:2] != 2'd0) || (hdr.lr_type[5:4] != 2'd0)));
    logic [SBW-1:0]  tile_row_prev; logic tile_row_valid;
    logic        sb_pop; logic [SBW-1:0] sb_pop_row;
    assign sb_shift = hdr.sb128 ? 5'd5 : 5'd4;
    // LR works in 64-line stripes (stripe s = lines 64s-8 .. 64s+55); the window counts superblock rows (64 or 128 lines):
    // the rows wholly final after lr_next stripes, and the last row stripe lr_next touches (sb64: lr_next - 1 and lr_next)
    logic [SBW-1:0] lr_rows_done, lr_row_cur;
    always_comb begin
        lr_rows_done = (lr_next == SBW'(0)) ? SBW'(0) : SBW'((32'(lr_next) * 64 - 8) >> (hdr.sb128 ? 7 : 6));
        lr_row_cur   = SBW'((32'(lr_next) * 64 + 55) >> (hdr.sb128 ? 7 : 6));
    end
    logic           hold_rc, hold_cd, hold_lr, hold_sr;      // slot-reuse holds (assigned below)
    // A superblock event held back from the reconstruction stage by the window (hold_rc, below) still marks the end
    // of the previous row once reconstruction has finished its last block: the hold waits for a flush that waits
    // for the filter that waits for this row count (2026-10-14: a deadlock at the first held row otherwise). The
    // real pop later finds tile_row_prev already on this row and counts nothing twice.
    assign sb_pop = ev_valid && (ev_kind == 2'd2) && (ev_pop || (hold_rc && !rc_busy));
    assign sb_pop_row = SBW'(ev_sb_r >> sb_shift);
    // ---- window frame memories (docs/picture-memory.md, step A): rows released as they become final in the last
    // enabled stage's buffer, and a hold on slot reuse per ring. Row q of a buffer is free when every consumer of
    // that buffer has passed it (and, for the flushed buffer, when its flush is done; flushed_cnt counts flush_done
    // pulses this frame: rows 0 .. flushed_cnt-1 are in DDR):
    //   fb0: LF past q+1 (it wrote into q's last lines), CDEF past q+1 (reads q's last lines), SR past q, LR past
    //        stripe q+1 (stripe q+1 starts at line 64q+56-3), flushed when fb0 is final;
    //   fb1: SR past q, LR past stripe q+1, flushed when fb1 is final;      fb2: flushed.
    // Reconstruction may not start superblock row R before row R - FRAME_WIN (the previous occupant of its slot) is
    // free; CDEF holds row r until fb1's row r - FRAME_WIN1 is free; LR holds stripe s (rows s-1 and s) until fb2's
    // row s - FRAME_WIN2 is free.
    logic [SBW-1:0] rel_upto, rel_next, flushed_cnt;
    logic [SBW-1:0] fb0_free, fb1_free, fb2_free, fbs_free;                 // registered copies of the *_c below
    logic [SBW-1:0] fb0_free_c, fb1_free_c, fb2_free_c, fbs_free_c, rel_upto_c;
    // the final picture: LrFrame, else CdefFrame, else (RING build, super-res) SR's upscaled rows in fb0s, else fb0
    assign final_buf = lr_en ? 2'd2 : cdef_en ? 2'd1 : (RING && sr_en) ? 2'd3 : 2'd0;
    function automatic logic [SBW-1:0] minr(input logic [SBW-1:0] a, input logic [SBW-1:0] b); return (a < b) ? a : b; endfunction
    function automatic logic [SBW-1:0] dec1(input logic [SBW-1:0] a); return (a == SBW'(0)) ? SBW'(0) : a - SBW'(1); endfunction
    always_comb begin
        // "stage past q+1" with next = q+2 is a count of q+1 free rows: next-1; a finished stage frees every row
        fb0_free_c = (lf_next >= n_rows) ? n_rows : dec1(lf_next);
        if (cdef_en) fb0_free_c = minr(fb0_free_c, (cd_next >= n_rows) ? n_rows : dec1(cd_next));
        if (sr_en)   fb0_free_c = minr(fb0_free_c, sr_next);
        if (lr_en)   fb0_free_c = minr(fb0_free_c, (lr_next >= n_stripes) ? n_rows : lr_rows_done);
        if (final_buf == 2'd0) fb0_free_c = minr(fb0_free_c, flushed_cnt);
        fb1_free_c = n_rows;
        if (sr_en)   fb1_free_c = minr(fb1_free_c, sr_next);
        if (lr_en)   fb1_free_c = minr(fb1_free_c, (lr_next >= n_stripes) ? n_rows : lr_rows_done);
        if (final_buf == 2'd1) fb1_free_c = minr(fb1_free_c, flushed_cnt);
        fb2_free_c = flushed_cnt;
        fbs_free_c = n_rows;                                     // fb0s: SR's rows, read by LR source 0, flushed when final
        if (lr_en)   fbs_free_c = minr(fbs_free_c, (lr_next >= n_stripes) ? n_rows : lr_rows_done);
        if (final_buf == 2'd3) fbs_free_c = minr(fbs_free_c, flushed_cnt);
        // the final rows of the last enabled stage's buffer (rows 0 .. rel_upto-1)
        if (lr_en)        rel_upto_c = (lr_next >= n_stripes) ? n_rows : lr_rows_done;   // stripe s done writes rows s-1 and s: s-1 final
        else if (sr_en)   rel_upto_c = sr_next;                                             // SR(r) in place, after CDEF / LF of r+1
        else if (cdef_en) rel_upto_c = cd_next;                                             // CDEF(r) writes fb1 row r whole
        else              rel_upto_c = (lf_next >= n_rows) ? n_rows : dec1(lf_next);        // LF(r) done makes row r-1 final
    end
    // The counts are used a cycle late. Every input only grows within a frame (the stage cursors, the flushed count), so
    // a stale count is at most one row short: a hold lasts a cycle longer, a release comes a cycle later, nothing is
    // released or started early. At 100 MHz routed the header's mi_rows through n_rows, the four-way minimum, the
    // hold compare and the event pop into row_cnt was 29 logic levels (-0.44 ns).
    // Cleared with the frame (a stale rel_upto from the previous frame released rows at the next frame's start), and the
    // frame-done test below uses the live rel_upto_c (with the registered one, rel_next matched it a cycle before the last
    // rows were released: the 2026-10-26 nightly flushed 49,152 of 61,440 bytes).
    always_ff @(posedge clk) begin
        if (rst || frame_start) begin fb0_free <= '0; fb1_free <= '0; fb2_free <= '0; fbs_free <= '0; rel_upto <= '0; end
        else begin fb0_free <= fb0_free_c; fb1_free <= fb1_free_c; fb2_free <= fb2_free_c; fbs_free <= fbs_free_c; rel_upto <= rel_upto_c; end
    end
    assign hold_rc = (FRAME_WIN > 0) && ev_valid && (ev_kind == 2'd2) && (32'(sb_pop_row) >= FRAME_WIN)
                     && (32'(sb_pop_row) - FRAME_WIN >= 32'(fb0_free));
    assign hold_cd = (FRAME_URAM != 0) && (FRAME_WIN1 > 0) && (32'(cd_next) >= FRAME_WIN1) && (32'(cd_next) - FRAME_WIN1 >= 32'(fb1_free));
    assign hold_lr = (FRAME_URAM != 0) && (FRAME_WIN2 > 0) && (32'(lr_row_cur) >= FRAME_WIN2) && (32'(lr_row_cur) - FRAME_WIN2 >= 32'(fb2_free));
    assign hold_sr = RING && (FRAME_WIN1 > 0) && (32'(sr_next) >= FRAME_WIN1) && (32'(sr_next) - FRAME_WIN1 >= 32'(fbs_free));
    logic fs_hold;
    assign ev_valid_rc = ev_valid && !hold_rc && !fs_hold;
    // the frame-state sideband writer taps the popped block events (docs/frame-state.md)
    if (INTER != 0 && FRAME_WIN > 0) begin : g_fsw
        frame_state_w #(.FBX(FBX), .SB128(SB128)) u_fsw (.clk, .rst, .hdr, .ihdr, .frame_start, .tile_done,
            .ev_pop(ev_valid_rc && ev_pop), .ev_kind, .ev_blk, .fs_hold,
            .fs_row_ready, .fs_row_idx, .fs_done, .fs_rd_sel, .fs_rd_line, .fs_rd_beat, .fs_rd_buf, .fs_rd_data);
    end else begin : g_nofsw
        assign fs_hold = 1'b0; assign fs_row_ready = 1'b0; assign fs_row_idx = '0; assign fs_rd_data = '0;
    end
    // the window build's limits (docs/frame-memory.md): 64-pixel superblocks (128 with SB128), 4:2:0 or mono, pipelined
    // frames, at most TE tile columns (a tile column spans every row, so a window of the full width could never release
    // one unless every column of the row is decoded at once: rtl/tile_engines.sv), no intra block copy. Sticky until the next frame, so software sees it after the tile (which may well have deadlocked on the hold).
    logic win_unsup;
    always_ff @(posedge clk) begin
        if (rst || frame_start) win_unsup <= 1'b0;
        else if ((FRAME_WIN > 0) && tile_busy && ((hdr.sb128 && SB128 == 0) || (!hdr.mono && !(hdr.ssx && hdr.ssy)) || !pipe_en
                                                  || 32'(n_tile_cols) > TE)) win_unsup <= 1'b1;   // more tile columns than engines
        // (a core build asked for CDEF / LR is the host software's business: HostDecoder.check_core_build; an RTL flag here
        // broke the core build's lf-stage suites on 2026-10-24, whose frames carry CDEF headers the host never starts)
        else if (RING && (FRAME_WIN > 0) && tile_busy && pipe_en && (32'(FRAME_WIN) < 3 + 32'(cdef_en) + 32'(lr_en)))
            win_unsup <= 1'b1;                                   // the fb0 ring is too shallow for the stages in flight: recon(r), LF(r-1),
                                                                 // CDEF(r-2), LR(r-3) and the hold slot would deadlock (2026-10-24, WIN 3 + CDEF + LR)
    end
    assign unsupported = ts_unsup || win_unsup || mc_unsup;        // mc_unsup: an inter feature beyond stage 2a
    always_ff @(posedge clk) begin
        row_final <= 1'b0;
        if (rst || frame_start) begin
            rel_next <= '0; flushed_cnt <= '0;
        end else begin
            if (seq_active && rel_next < rel_upto) begin
                row_final <= 1'b1; row_final_idx <= rel_next; rel_next <= rel_next + SBW'(1);
            end
            if (flush_done) flushed_cnt <= flushed_cnt + SBW'(1);
        end
    end
    function automatic logic row_dec(input logic [SBW-1:0] r);   // row r reconstructed (every tile column: one merged stream)
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
        r = cd_next; cd_ok = cdef_en && (r < n_rows) && (lf_next > r) && ((r + SBW'(1) >= n_rows) || (lf_next > r + SBW'(1))) && !hold_cd;
        r = sr_next; sr_ok = sr_en && (r < n_rows) && (cdef_en ? ((cd_next > r) && ((r + SBW'(1) >= n_rows) || (cd_next > r + SBW'(1))))
                                                            : ((lf_next > r) && ((r + SBW'(1) >= n_rows) || (lf_next > r + SBW'(1))))) && !hold_sr;
        lr_row = SBW'(lr_ly1 >> (hdr.sb128 ? 7 : 6));
        lr_ok = lr_en && (lr_next < n_stripes) && (cdef_en ? (cd_next > lr_row) : ((lf_next > lr_row) && ((lr_row + SBW'(1) >= n_rows) || (lf_next > lr_row + SBW'(1)))))
                      && (!sr_en || (sr_next > lr_row)) && !hold_lr;
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
                if (tile_row_valid && sb_pop_row != tile_row_prev) row_cnt[(FBY - 6)'(tile_row_prev)] <= row_cnt[(FBY - 6)'(tile_row_prev)] + n_pass;
                tile_row_prev <= sb_pop_row; tile_row_valid <= 1'b1;
            end
            if (tile_done && tile_row_valid) begin row_cnt[(FBY - 6)'(tile_row_prev)] <= row_cnt[(FBY - 6)'(tile_row_prev)] + n_pass; tile_row_valid <= 1'b0; end
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
                && !lf_pend && !cd_pend && !sr_pend && !lr_pend && rel_next == rel_upto_c && rel_upto == rel_upto_c) begin   // every row released (the flush queue sees them all)
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
`ifndef SYNTHESIS
    time trace_last_rc_w;                                                           // window trace (simulation only)
    int  trace_lf_w;
    always_ff @(posedge clk) if (fb0_we[0]) trace_last_rc_w <= $time;
    always_ff @(posedge clk) if (lf_go) trace_lf_w <= 0; else if (fb0_we[1]) trace_lf_w <= trace_lf_w + 1;
    always_ff @(posedge clk) if (FRAME_WIN > 0 && !rst) begin                      // window trace (simulation only)
        if (lf_pend && lf_done) $display("[win] t=%0t LF done row %0d (n_rows %0d) rel_upto->%0d", $time, lf_next, n_rows, (lf_next + SBW'(1) >= n_rows) ? n_rows : lf_next);
        if (row_final) $display("[win] t=%0t row_final %0d", $time, row_final_idx);
        if (flush_done) $display("[win] t=%0t flush_done (flushed_cnt -> %0d)", $time, flushed_cnt + 1);
        if (frame_done) $display("[win] t=%0t frame_done rel_next %0d rel_upto %0d", $time, rel_next, rel_upto);
        if (frame_start) $display("[win] t=%0t frame_start cdef_en %0d sr_en %0d lr_en %0d final_buf %0d n_rows %0d n_stripes %0d", $time, cdef_en, sr_en, lr_en, final_buf, n_rows, n_stripes);
        if (cd_pend && cdef_done) $display("[win] t=%0t CDEF done row %0d", $time, cd_next);
        if (lf_go) $display("[win] t=%0t LF start row %0d MI rows %0d..%0d (hdr.mi_rows %0d, tile_busy %0d rc_busy %0d row_cnt %0d/%0d/%0d, last recon fb0 write t=%0t)", $time, lf_next,
                            lf_row_first, lf_row_last, hdr.mi_rows, tile_busy, rc_busy, row_cnt[0], row_cnt[1], row_cnt[2], trace_last_rc_w);
        if (lf_pend && lf_done) $display("[win] t=%0t LF wrote %0d words this row", $time, trace_lf_w);
        if (tile_done) $display("[win] t=%0t tile_done (rc_busy %0d, last recon fb0 write t=%0t)", $time, rc_busy, trace_last_rc_w);
        if (sb_pop) $display("[win] t=%0t sb_pop row %0d (prev %0d valid %0d)", $time, sb_pop_row, tile_row_prev, tile_row_valid);
        if (sr_pend && sr_done) $display("[win] t=%0t SR done row %0d sub %0d", $time, sr_next, sr_sub);
        if (lr_pend && lr_done_o) $display("[win] t=%0t LR done stripe %0d (n_stripes %0d)", $time, lr_next, n_stripes);
        if (hold_rc && !rc_busy && ev_pop) $display("[win] t=%0t hold row %0d (fb0 free %0d, flushed %0d)", $time, sb_pop_row, fb0_free, flushed_cnt);
        if (hold_cd && cdef_en && !cd_pend && !cdef_busy) $display("[win] t=%0t hold CDEF row %0d (fb1 free %0d)", $time, cd_next, fb1_free);
        if (hold_lr && lr_en && !lr_pend && !lr_busy) $display("[win] t=%0t hold LR stripe %0d (fb2 free %0d)", $time, lr_next, fb2_free);
        if (hold_sr && sr_en && !sr_pend && !sr_busy) $display("[win] t=%0t hold SR row %0d (fb0s free %0d)", $time, sr_next, fbs_free);
    end
    // heartbeat (simulation only): the sequencer's state every 2^18 cycles while a frame is active, so a stalled twin
    // shows which stage is waiting for what (and whether simulated time still advances at all)
    int trace_hb;
    always_ff @(posedge clk) if (FRAME_WIN > 0 && !rst) begin
        trace_hb <= frame_start ? 0 : trace_hb + 1;
        if (seq_active && trace_hb[17:0] == 18'd0)
            $display("[win] t=%0t hb: tile_busy %0d rc_busy %0d ev valid/kind/pop %0d/%0d/%0d sb_r %0d hold rc/cd/lr/sr %0d%0d%0d%0d | lf %0d pend %0d | cd %0d pend %0d busy %0d | sr %0d pend %0d | lr %0d pend %0d busy %0d | rel %0d/%0d flushed %0d free fb0/1/2/s %0d/%0d/%0d/%0d | row_cnt[n-2..n-1] %0d %0d",
                     $time, tile_busy, rc_busy, ev_valid, ev_kind, ev_pop, sb_pop_row, hold_rc, hold_cd, hold_lr, hold_sr,
                     lf_next, lf_pend, cd_next, cd_pend, cdef_busy, sr_next, sr_pend, lr_next, lr_pend, lr_busy,
                     rel_next, rel_upto, flushed_cnt, fb0_free, fb1_free, fb2_free, fbs_free,
                     row_cnt[(FBY - 6)'(n_rows - SBW'(2))], row_cnt[(FBY - 6)'(n_rows - SBW'(1))]);
    end
`endif
endmodule
