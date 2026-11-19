// Decoder top: tile_syntax (entropy decoding + syntax) -> recon_top (prediction, transforms, reconstruction) -> fb0,
// and behind it, superblock row by superblock row under the frame sequencer, the streaming filter chain (fc_top:
// deblocking, CDEF, super-resolution, loop restoration; docs/filter-chain.md), which writes the final picture into
// fb2. Software supplies the parsed headers (hdr_t / rec_hdr_t / lf_hdr_t / cdef_hdr_t), the CDF defaults and the tile
// bytes, pulses frame_start before the first tile and waits for frame_done; h_buf selects the picture the host reads.
module dec_top
  import cdf_map_pkg::*;
  import syn_pkg::*;
#(
    parameter int FBX = 10,                  // 1024 x 512 frame buffer: covers the test corpus (<= 640x360 + overhang)
    parameter int FBY = 9,
    parameter int ITX_LANES = 4,             // transform engines: 4 (throughput) or 2 (K26 area, +2.6% cycles)
    parameter int FRAME_URAM = 0,             // 1: the frame buffers are on-chip double-pumped UltraRAM (rtl/frame_mem_uram.sv, needs clk2x / phase)
    parameter int FRAME_WIN = 0,             // >0 with FRAME_URAM: fb0 holds this many superblock rows (window, docs/frame-memory.md)
    parameter int FRAME_WIN2 = 0,            // with FRAME_URAM: fb2 (the final picture) as a ring of this many superblock rows, each flushed
                                             //    to DDR before its slot is reused (docs/picture-memory.md); 0: the whole picture
    parameter int INTER = 1,                 // 0: the intra-only build (no ref_fetch / mc_filt / warp_setup, no inter syntax): the K26 board
    parameter int SB128 = 0,                 // 1: the window build takes 128x128 superblocks too (128-line window rows, deeper mode-info rings)
    parameter int TE = 1,                     // tile engines: the tile columns of a tile row decoded at once (rtl/tile_engines.sv)
    parameter int REF_RING = 16              // ref_fetch line buffers (recon_top REF_RING; the U200 build's U200_REF_RING)
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
    input  logic [6:0]        mf_we,               // one per reference: the host writes one, the projection engine all seven
    input  logic [MIW-2:0]    mf_wy8, mf_wx8,
    input  logic              mf_wvalid,
    input  logic [111:0]      mf_wrow, mf_wcol,    // per reference at [16r +: 16]
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
    output logic              mem_err,             // a reference read failed on the AXI bus (recon_top): the frame is not trustworthy
    output logic [31:0]       perf_syn_stall,      // tile cycles the syntax decoder held a transform block waiting for reconstruction
    output logic [31:0]       perf_rec_idle,       // tile cycles reconstruction had nothing to do (syntax-bound)
    // frame sequencer: the filter chain trails the tile decoder by superblock rows; software pulses frame_start
    // after the headers, feeds the tiles as usual and waits for frame_done
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
    input  logic [3:0]        ld_cfg,             // bit 3: the motion-field projection (rtl/mf_project.sv) after the loads
    input  logic [2:0]        mf_nsrc,
    input  logic [199:0]      mf_src_base,
    input  logic [79:0]       mf_src_rowb,
    input  logic [4:0]        mf_src_sign,
    input  logic [39:0]       mf_src_tocur,
    input  logic [279:0]      mf_src_off,
    input  logic [55:0]       mf_dst_dist,
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
    // observability (records as they stream between the stages)
    output logic              blk_done,
    output blk_rec_t          blk_rec,
    output logic              tx_done,
    output tx_rec_t           tx_rec,
    output logic              lr_done,
    output lr_rec_t           lr_rec,
    // frame buffer host read port (registered): h_buf 0 = fb0 (the reconstruction), 1 = fb2 (the final picture)
    input  logic              h_buf,
    input  logic [1:0]        h_plane,
    input  logic [FBX-1:0]    h_x,
    input  logic [FBY-1:0]    h_y,
    output logic [11:0]       h_rdata,
    output logic [47:0]       h_rdata4,            // the four samples of h_x's aligned group, lane 0 lowest
    output logic              h_ok                 // the host read sampled at the last edge was served (the UltraRAM shares a slot)
);
    logic ts_done, tx_ack, pm_plane;
    logic [9:0] q_addr; logic [2:0] q_slot_r; logic signed [20:0] q_data; logic [5:0] pm_x, pm_y; logic [2:0] pm_idx;
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
    // the loader group: fs_load (segment map, CDF rows) and, with ld_cfg[3], mf_project, started together: the engine clears the
    // band while fs_load reads, then takes the read channel once fs_load is idle (src_go); one busy / done to the host
    logic           fsl_busy, fsl_done, fsl_err, pj_busy, pj_done, pj_err, pj_start;
    logic [39:0]    pj_araddr; logic [7:0] pj_arlen;
    logic           pj_arvalid, pj_arready, pj_rvalid, pj_rready;
    logic [6:0]     pj_we; logic [MIW-2:0] pj_wy8, pj_wx8; logic pj_wvalid; logic [111:0] pj_wrow, pj_wcol;
    if (INTER != 0 && FRAME_WIN > 0) begin : g_fsl
        fs_load u_fsl (.clk, .rst, .hdr, .ld_start, .ld_cfg(ld_cfg[2:0]), .ld_seg_base, .ld_seg_rowb, .ld_cdf_base, .ld_busy(fsl_busy), .ld_done(fsl_done), .ld_err(fsl_err),
            .m_axi_araddr(ld_araddr), .m_axi_arlen(ld_arlen), .m_axi_arsize(ld_arsize), .m_axi_arburst(ld_arburst), .m_axi_arcache(ld_arcache), .m_axi_arprot(ld_arprot),
            .m_axi_arvalid(ld_arvalid), .m_axi_arready(ld_arready), .m_axi_rdata, .m_axi_rresp, .m_axi_rlast, .m_axi_rvalid(ld_rvalid), .m_axi_rready(ld_rready),
            .sp_we(ld_sp_we), .sp_wrow(ld_sp_wrow), .sp_wwc(ld_sp_wwc), .sp_wdata(ld_sp_wdata), .def_we(ld_def_we), .def_addr(ld_def_addr), .def_data(ld_def_data));
        mf_project u_pj (.clk, .rst, .hdr, .mf_start(pj_start), .src_go(!fsl_busy), .band_go(1'b1), .bands_done(), .mf_nsrc, .src_base(mf_src_base), .src_rowb(mf_src_rowb), .src_sign(mf_src_sign),
            .src_tocur(mf_src_tocur), .src_off(mf_src_off), .dst_dist(mf_dst_dist), .mf_busy(pj_busy), .mf_done(pj_done), .mf_err(pj_err),
            .m_axi_araddr(pj_araddr), .m_axi_arlen(pj_arlen), .m_axi_arsize(), .m_axi_arburst(), .m_axi_arcache(), .m_axi_arprot(),   // (the attributes every reader uses: ref_fetch drives them)
            .m_axi_arvalid(pj_arvalid), .m_axi_arready(pj_arready), .m_axi_rdata, .m_axi_rresp, .m_axi_rlast, .m_axi_rvalid(pj_rvalid), .m_axi_rready(pj_rready),
            .mf_we(pj_we), .mf_wy8(pj_wy8), .mf_wx8(pj_wx8), .mf_wvalid(pj_wvalid), .mf_wrow(pj_wrow), .mf_wcol(pj_wcol));
        assign pj_start = ld_start && ld_cfg[3];
        assign ld_busy = fsl_busy || pj_busy;
        assign ld_done = pj_done || (fsl_done && !pj_busy);   // the engine's reads wait for fs_load, so its done is the later one (its done pulse lands the cycle its busy drops)
        assign ld_err  = fsl_err || pj_err;
    end else begin : g_nofsl
        assign ld_busy = 1'b0; assign ld_done = 1'b0; assign ld_err = 1'b0; assign fsl_busy = 1'b0; assign fsl_done = 1'b0; assign fsl_err = 1'b0;
        assign pj_busy = 1'b0; assign pj_done = 1'b0; assign pj_err = 1'b0; assign pj_start = 1'b0;
        assign ld_araddr = '0; assign ld_arlen = '0; assign ld_arsize = '0; assign ld_arburst = '0; assign ld_arcache = '0; assign ld_arprot = '0;
        assign ld_arvalid = 1'b0; assign ld_rready = 1'b0;
        assign pj_araddr = '0; assign pj_arlen = '0;
        assign pj_arvalid = 1'b0; assign pj_rready = 1'b0;
        assign ld_sp_we = 1'b0; assign ld_sp_wrow = '0; assign ld_sp_wwc = '0; assign ld_sp_wdata = '0; assign ld_def_we = 1'b0; assign ld_def_addr = '0; assign ld_def_data = '0;
        assign pj_we = '0; assign pj_wy8 = '0; assign pj_wx8 = '0; assign pj_wvalid = 1'b0; assign pj_wrow = '0; assign pj_wcol = '0;
    end
    // the read channel: fs_load has it while it runs (before the tiles: nothing else is in flight then); after that
    // reference fetch and the projection engine share it burst by burst (rtl/axi_rd_share.sv: the projection reads its
    // sources band by band during the decode, docs/motion-field-band.md). Every reader uses the same burst attributes.
    logic [39:0]    sh_araddr; logic [7:0] sh_arlen; logic sh_arvalid, sh_rready;
    axi_rd_share #(.QL2(5)) u_rsh (.clk, .rst,
        .s_araddr({pj_araddr, rf_araddr}), .s_arlen({pj_arlen, rf_arlen}), .s_arvalid({pj_arvalid, rf_arvalid}),
        .s_arready({pj_arready, rf_arready}), .s_rvalid({pj_rvalid, rf_rvalid}), .s_rready({pj_rready, rf_rready}),
        .m_araddr(sh_araddr), .m_arlen(sh_arlen), .m_arvalid(sh_arvalid), .m_arready(!fsl_busy && m_axi_arready),
        .m_rvalid(!fsl_busy && m_axi_rvalid), .m_rlast(m_axi_rlast), .m_rready(sh_rready));
    assign m_axi_araddr  = fsl_busy ? ld_araddr  : sh_araddr;
    assign m_axi_arlen   = fsl_busy ? ld_arlen   : sh_arlen;
    assign m_axi_arsize  = fsl_busy ? ld_arsize  : rf_arsize;
    assign m_axi_arburst = fsl_busy ? ld_arburst : rf_arburst;
    assign m_axi_arcache = fsl_busy ? ld_arcache : rf_arcache;
    assign m_axi_arprot  = fsl_busy ? ld_arprot  : rf_arprot;
    assign m_axi_arvalid = fsl_busy ? ld_arvalid : sh_arvalid;
    assign m_axi_rready  = fsl_busy ? ld_rready  : sh_rready;
    assign ld_arready = fsl_busy && m_axi_arready;
    assign ld_rvalid  = fsl_busy && m_axi_rvalid;
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
                   .mf_we(mf_we | pj_we), .mf_wy8(pj_we != 7'd0 ? pj_wy8 : mf_wy8), .mf_wx8(pj_we != 7'd0 ? pj_wx8 : mf_wx8),
                   .mf_wvalid(pj_we != 7'd0 ? pj_wvalid : mf_wvalid), .mf_wrow(pj_we != 7'd0 ? pj_wrow : mf_wrow), .mf_wcol(pj_we != 7'd0 ? pj_wcol : mf_wcol),
                   .sp_we(sp_we_i), .sp_wrow(sp_wrow_i), .sp_wwc(sp_wwc_i), .sp_wdata(sp_wdata_i),
                   .tile_start, .tile_busy, .tile_done(ts_done), .unsupported(ts_unsup),
                   .ev_valid, .ev_kind, .ev_sb_r, .ev_sb_c, .ev_blk, .ev_tx, .ev_pop, .ev_par, .ev_taken, .nb_valid, .nb_blk, .nb_par, .nb_at_head, .nb_pop,
                   .tx_rel, .tx_inflight, .rc_busy, .rc_col_start, .rc_col_end,
                   .q_slot_r, .q_addr, .q_data, .pm_plane, .pm_x, .pm_y, .pm_idx,
                   .lr_done, .lr_rec, .blk_done, .blk_rec, .tx_done, .tx_rec, .tx_ack);
    // event queue (inside tile_engines, one per engine): the syntax decoder runs ahead of reconstruction; a palette
    // block is released (blk_ack, per engine inside tile_engines) only once its engine's queue has drained and
    // reconstruction is idle, because its colour map lives in the syntax stage
    logic ev_valid_rc;                              // ev_valid as the reconstruction stage sees it (held by the window)
    logic ev_valid, ev_pop, tx_rel; logic [1:0] ev_kind; logic [MIW-1:0] ev_sb_r, ev_sb_c; blk_rec_t ev_blk; tx_rec_t ev_tx; logic [3:0] tx_inflight;
    logic ev_par, ev_taken, nb_valid, nb_par, nb_at_head, nb_pop; blk_rec_t nb_blk;   // early block take (rec_fifo)
    // declared ahead of their first use: Vivado makes an implicit 1-bit net of an identifier used before its declaration
    logic rc_busy;
    logic [3:0] cdr_val; mi_lf_t cd_rd_info;

    logic fb_re, fb_we; logic [1:0] fb_plane; logic [FBX-1:0] fb_x; logic [FBY-1:0] fb_y; logic [11:0] fb_wdata, fb_rdata;
    logic fb2_we; logic [1:0] fb2_plane; logic [FBX-1:0] fb2_x; logic [FBY-1:0] fb2_y; logic [11:0] fb2_wdata;
    logic fb4_we; logic [1:0] fb4_plane; logic [FBX-1:0] fb4_x; logic [FBY-1:0] fb4_y; logic [47:0] fb4_wdata, fb_rdata4;
    logic [3:0] fb4_be; logic mc_unsup;
    logic fb_edge, el_sel; logic [11:0] el_rdata; logic [47:0] el_rdata4;   // intra edge reads from the unfiltered last-line copy (edge_line)
    logic mi_blk_we, mi_blk_busy, mi_tx_we, mi_tx_busy; logic [MIW-1:0] mi_blk_r, mi_blk_c, mi_tx_row, mi_tx_col; logic [5:0] mi_blk_bw4, mi_blk_bh4;
    mi_lf_t mi_blk_data; logic [1:0] mi_tx_plane; logic [4:0] mi_tx_w4, mi_tx_h4, mi_tx_sz;
    logic mi_cd_clr, mi_cd_we; logic [SBW-1:0] mi_cd_row64, mi_cd_col64; logic [3:0] mi_cd_mask; logic [2:0] mi_cd_idx;
    // reconstruction's header: the frame's, with the tile column bounds of the engine whose block it is working on
    // (is_inside for neighbour availability, the superblock clip at the tile's right edge); the arbiter switches
    // engines only while reconstruction is idle
    logic [MIW-1:0] rc_col_start, rc_col_end;
    hdr_t hdr_rc;
    always_comb begin hdr_rc = hdr; hdr_rc.mi_col_start = rc_col_start; hdr_rc.mi_col_end = rc_col_end; end
    recon_top #(.FBX(FBX), .FBY(FBY), .ML2R(FRAME_WIN > 0 ? 6 + SB128 : FBY - 2), .ML2C(FBX - 2), .ITX_LANES(ITX_LANES), .INTER(INTER), .FRAME_WIN(FRAME_WIN), .REF_RING(REF_RING)) u_rc (.clk, .rst, .hdr(hdr_rc), .rh,
                                            .ev_valid(ev_valid_rc), .ev_kind, .ev_sb_r, .ev_sb_c, .ev_blk, .ev_tx, .ev_pop, .tx_rel,
                                            .ev_par, .ev_taken, .nb_valid, .nb_blk, .nb_par, .nb_at_head, .nb_pop,
                                            .q_slot(q_slot_r), .q_addr, .q_data, .pm_plane, .pm_x, .pm_y, .pm_idx, .busy(rc_busy),
                                            .mi_blk_we, .mi_blk_r, .mi_blk_c, .mi_blk_bw4, .mi_blk_bh4, .mi_blk_data, .mi_blk_busy,
                                            .mi_tx_we, .mi_tx_plane, .mi_tx_row, .mi_tx_col, .mi_tx_w4, .mi_tx_h4, .mi_tx_sz, .mi_tx_busy,
                                            .mi_cd_clr, .mi_cd_we, .mi_cd_row64, .mi_cd_col64, .mi_cd_mask, .mi_cd_idx,
                                            .fb_re, .fb_we, .fb_plane, .fb_x, .fb_y, .fb_wdata, .fb_rdata, .fb_rdata4,
                                            .fb_edge, .el_sel, .el_rdata, .el_rdata4,
                                            .fb2_we, .fb2_plane, .fb2_x, .fb2_y, .fb2_wdata,
                                            .fb4_we, .fb4_plane, .fb4_x, .fb4_y, .fb4_wdata, .fb4_be,
                                            .ihdr, .ref_slot, .ref_planes, .ref_dims, .ref_scales, .ref_fmt16, .cur_planes, .cur_fmt16, .mc_unsup, .mem_err,
                                            .m_axi_araddr(rf_araddr), .m_axi_arlen(rf_arlen), .m_axi_arsize(rf_arsize), .m_axi_arburst(rf_arburst), .m_axi_arcache(rf_arcache), .m_axi_arprot(rf_arprot),
                                            .m_axi_arvalid(rf_arvalid), .m_axi_arready(rf_arready), .m_axi_rdata, .m_axi_rresp, .m_axi_rlast, .m_axi_rvalid(rf_rvalid), .m_axi_rready(rf_rready));

    // ---- the streaming filter chain's side of the shared stores (the chain itself is instantiated after the frame sequencer)
    localparam int FCW = FBX - 2;
    logic [2:0][MIW-1:0] fc_rd_row, fc_rd_col, fc_rd2_row, fc_rd2_col, fc_txr_row, fc_txr_col;
    logic [2:0][1:0]     fc_txr_plane, fc_lrr_plane;
    logic [2:0][SBW-1:0] fc_cdr_row64, fc_cdr_col64, fc_lrr_row, fc_lrr_col;
    logic                fc_busy, fc_done;             // the chain as the sequencer's per-row stage
    logic [1:0]          fo_we; logic [1:0][1:0] fo_plane; logic [1:0][FBX-1:0] fo_x; logic [1:0][FBY-1:0] fo_y; logic [1:0][47:0] fo_d;   // its two writes into fb2
    // per-4x4 state, read by the chain: its luma plane reads this store, its chroma planes two more (written alike, below)
    mi_lf_t rd_info; logic [4:0] txr_sz; lr_rec_t lrr_rec;
    // the per-4x4 filter state (and CDEF indices, LR units) is a ring of 2^ML2R MI rows: every row the fb0 ring can hold
    // live (FRAME_WIN superblock rows of 16: recon(r) down to the band the chain is filtering) must fit, or recon overwrites the row LR
    // is reading (2026-10-24: 64 MI rows with a 5-row ring lost row 0's LR units to row 4)
    mi_store #(.ML2R(FRAME_WIN > 0 ? $clog2(FRAME_WIN * (SB128 != 0 ? 32 : 16)) : FBY - 2), .ML2C(FBX - 2)) u_mi (.clk, .rst, .blk_we(mi_blk_we), .blk_r(mi_blk_r), .blk_c(mi_blk_c), .blk_bw4(mi_blk_bw4), .blk_bh4(mi_blk_bh4), .blk_data(mi_blk_data), .blk_busy(mi_blk_busy),
                   .tx_we(mi_tx_we), .tx_plane(mi_tx_plane), .tx_row(mi_tx_row), .tx_col(mi_tx_col), .tx_w4(mi_tx_w4), .tx_h4(mi_tx_h4), .tx_sz(mi_tx_sz), .tx_busy(mi_tx_busy),
                   .rd_row(fc_rd_row[0]), .rd_col(fc_rd_col[0]), .rd_info, .rd2_row(fc_rd2_row[0]), .rd2_col(fc_rd2_col[0]), .rd2_info(cd_rd_info),
                   .txr_plane(fc_txr_plane[0]), .txr_row(fc_txr_row[0]), .txr_col(fc_txr_col[0]), .txr_sz,
                   .cd_clr(mi_cd_clr), .cd_we(mi_cd_we), .cd_row64(mi_cd_row64), .cd_col64(mi_cd_col64), .cd_sb128(hdr.sb128), .cd_mask(mi_cd_mask), .cd_idx(mi_cd_idx),
                   .cdr_row64(fc_cdr_row64[0]), .cdr_col64(fc_cdr_col64[0]), .cdr_val,
                   .lr_we(lr_done), .lr_rec, .lrr_plane(fc_lrr_plane[0]), .lrr_row(fc_lrr_row[0]), .lrr_col(fc_lrr_col[0]), .lrr_rec);
    // the chain's band (superblock row) under the frame sequencer
    logic [MIW-1:0] fc_row_first, fc_row_last;
    logic           fc_go;
    logic [SBW-1:0] fc_next;                                // the next superblock row the chain will filter
    logic [11:0] h_rdata0, h_rdata2; logic [47:0] h_rdata40, h_rdata42; logic h_ok0, h_ok2;
    // ---- frame buffers
    // fb0 = the reconstruction, unfiltered. One reader and one writer, reconstruction: the chain reads its own row store
    // (fc_rstore, a mirror of these writes), and the references of later frames are the flushed final pictures
    logic w_rc_we; logic [3:0] w_rc_be; logic [1:0] w_rc_pl; logic [FBX-1:0] w_rc_x; logic [FBY-1:0] w_rc_y; logic [47:0] w_rc_d;
    frame_mem_w #(.FBX(FBX), .FBY(FBY)) u_w_rc (.we1(fb_we || fb2_we), .plane1(fb2_we ? fb2_plane : fb_plane), .x1(fb2_we ? fb2_x : fb_x), .y1(fb2_we ? fb2_y : fb_y), .d1(fb2_we ? fb2_wdata : fb_wdata),
                                                .we4(fb4_we), .plane4(fb4_plane), .x4(fb4_x), .y4(fb4_y), .d4(fb4_wdata), .be4(fb4_be),
                                                .we(w_rc_we), .be(w_rc_be), .plane(w_rc_pl), .x(w_rc_x), .y(w_rc_y), .data(w_rc_d));
    // the unfiltered copy of every superblock row's last line, for the intra edge reads of the row below (so that
    // filtering a row need not wait for the next row's reconstruction: fc_ok)
    edge_line #(.FBX(FBX), .FBY(FBY)) u_el (.clk, .sb128(hdr.sb128), .ssy(hdr.ssy),
                                            .we(w_rc_we), .be(w_rc_be), .wplane(w_rc_pl), .wx(w_rc_x), .wy(w_rc_y), .wdata(w_rc_d),
                                            .re(fb_re && fb_edge), .rplane(fb_plane), .rx(fb_x), .ry(fb_y),
                                            .sel(el_sel), .rdata(el_rdata), .rdata4(el_rdata4));
`ifndef SYNTHESIS
    // sim only: how many intra edge reads the copy answered in the frame, and how many of those differ from what the
    // frame memory returned for the same request (= reads of a sample reconstruction has since overwritten)
    int el_n, el_diff;
    always_ff @(posedge clk) begin
        if (rst || frame_start) begin el_n <= 0; el_diff <= 0; end
        else if (el_sel) begin
            el_n <= el_n + 1;
            if (el_rdata !== fb_rdata) begin
                el_diff <= el_diff + 1;
                if (el_diff < 3) $display("[el] t=%0t edge read: copy %h, frame memory %h", $time, el_rdata, fb_rdata);
            end
        end
        if (frame_done && el_n != 0) $display("[el] t=%0t frame: %0d last-line edge reads, %0d differ from the frame memory", $time, el_n, el_diff);
    end
`endif
    generate if (FRAME_URAM != 0) begin : g_fb0_uram
        // the on-chip memory has two read and two write ports (double-pumped); reconstruction uses the first of each
        logic [11:0] unused_rd; logic [47:0] unused_rd4;
        frame_mem_uram #(.FBX(FBX), .FBY(FBY), .PW(12), .FRAME_WIN(FRAME_WIN), .SB128(SB128)) u_fb (.clk, .clk2x, .phase, .sb128(hdr.sb128),
            .re({1'b0, fb_re}), .r_plane({2'd0, fb_plane}), .r_x({FBX'(0), fb_x}), .r_y({FBY'(0), fb_y}),
            .rdata({unused_rd, fb_rdata}), .rdata4({unused_rd4, fb_rdata4}),
            .we({1'b0, w_rc_we}), .w_be({4'd0, w_rc_be}), .w_plane({2'd0, w_rc_pl}), .w_x({FBX'(0), w_rc_x}), .w_y({FBY'(0), w_rc_y}), .w_data({48'd0, w_rc_d}),
            .h_plane, .h_x, .h_y, .h_rdata(h_rdata0), .h_rdata4(h_rdata40), .h_ok(h_ok0));
    end else begin : g_fb0_model
        frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12), .NRD(1), .NWR(1)) u_fb (.clk, .re(fb_re), .r_plane(fb_plane), .r_x(fb_x), .r_y(fb_y), .rdata(fb_rdata), .rdata4(fb_rdata4),
                                                                           .we(w_rc_we), .w_be(w_rc_be), .w_plane(w_rc_pl), .w_x(w_rc_x), .w_y(w_rc_y), .w_data(w_rc_d),
                                                                           .h_plane, .h_x, .h_y, .h_rdata(h_rdata0), .h_rdata4(h_rdata40), .h_ok(h_ok0));
    end endgenerate
    // fb2 = the final picture: the chain's two write ports (fc_out), no decoder-side reader, so the host port (and the row
    // flush through it) is always served
    generate if (FRAME_URAM != 0) begin : g_fb2_uram
        frame_mem_uram #(.FBX(FBX), .FBY(FBY), .PW(12), .FRAME_WIN(FRAME_WIN2), .SB128(SB128)) u_fb2 (.clk, .clk2x, .phase, .sb128(hdr.sb128),
            .re(2'b00), .r_plane(4'd0), .r_x('0), .r_y('0), .rdata(), .rdata4(),
            .we(fo_we), .w_be({{4{fo_we[1]}}, {4{fo_we[0]}}}), .w_plane({fo_plane[1], fo_plane[0]}), .w_x({fo_x[1], fo_x[0]}), .w_y({fo_y[1], fo_y[0]}), .w_data({fo_d[1], fo_d[0]}),
            .h_plane, .h_x, .h_y, .h_rdata(h_rdata2), .h_rdata4(h_rdata42), .h_ok(h_ok2));
    end else begin : g_fb2_model
        frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12), .NRD(1), .NWR(2)) u_fb2 (.clk, .re(1'b0), .r_plane(2'd0), .r_x('0), .r_y('0), .rdata(), .rdata4(),
                                                                            .we(fo_we), .w_be({{4{fo_we[1]}}, {4{fo_we[0]}}}), .w_plane({fo_plane[1], fo_plane[0]}), .w_x({fo_x[1], fo_x[0]}),
                                                                            .w_y({fo_y[1], fo_y[0]}), .w_data({fo_d[1], fo_d[0]}),
                                                                            .h_plane, .h_x, .h_y, .h_rdata(h_rdata2), .h_rdata4(h_rdata42), .h_ok(h_ok2));
    end endgenerate
    assign h_ok     = h_buf ? h_ok2     : h_ok0;
    assign h_rdata  = h_buf ? h_rdata2  : h_rdata0;
    assign h_rdata4 = h_buf ? h_rdata42 : h_rdata40;

    // ================================================================ frame sequencer: the chain's bands per superblock row
    // Row r of the frame is decoded once every tile column has reconstructed past it (row_cnt[r] >= n_tile_cols): a pass of
    // the reconstruction stream (the first superblock of the next row popped, or tile_done) counts for the engines of
    // the tile row (n_pass): the tile engines merge the columns they run at once into one raster-ordered stream, so a
    // row of a two-engine tile row passes once and counts twice, while tiles fed one at a time (run_tile) pass it once
    // per column (2026-10-28: counting one pass per row started LF on a row whose other column was still undecoded,
    // seed 9 of the dec_host pipe suite: the tile edge deblocked against stale samples).
    // The chain filters band r (= superblock row r) as soon as row r is decoded: the intra edge reads of row r + 1 come
    // from the unfiltered copy in edge_line (since 2026-11-13; before that deblocking of row r waited for row r + 1).
    localparam int NSBR = 1 << (FBY - 6);                 // superblock rows (64-pixel; a 128 superblock counts as two entries' worth of mi rows)
    logic [4:0]  sb_shift;                                // log2(mi rows per superblock row)
    logic [SBW-1:0]  n_rows;
    logic [7:0]  row_cnt [0:NSBR-1];
    logic [7:0]  n_pass;                                    // tile columns a pass of the reconstruction stream stands for: the
    assign n_pass = (eng_mask == '0) ? 8'd1 : 8'($countones(eng_mask));   // engines of the tile row (one, when tiles come one at a time)
    logic        seq_active, rows_seen;
    // frame geometry and stage enables follow the live header (the host loads it with the first tile, after frame_start)
    assign n_rows = SBW'((hdr.mi_rows + (MIW'(1) << sb_shift) - MIW'(1)) >> sb_shift);
    logic        fc_cdef_on;
    assign fc_cdef_on = hdr.enable_cdef && !hdr.coded_lossless && !hdr.allow_intrabc;
    logic [SBW-1:0]  tile_row_prev; logic tile_row_valid;
    logic        sb_pop; logic [SBW-1:0] sb_pop_row;
    assign sb_shift = hdr.sb128 ? 5'd5 : 5'd4;
    logic           hold_rc, hold_fc, hold_rs;               // slot-reuse holds (assigned below)
    // A superblock event held back from the reconstruction stage by the window (hold_rc, below) still marks the end
    // of the previous row once reconstruction has finished its last block: the hold waits for a flush that waits
    // for the filter that waits for this row count (2026-10-14: a deadlock at the first held row otherwise). The
    // real pop later finds tile_row_prev already on this row and counts nothing twice.
    assign sb_pop = ev_valid && (ev_kind == 2'd2) && (ev_pop || (hold_rc && !rc_busy));
    assign sb_pop_row = SBW'(ev_sb_r >> sb_shift);
    // ---- window frame memories (docs/picture-memory.md): rows of fb2 released for the flush as they become final, and
    // a hold on slot reuse per ring (flushed_cnt counts flush_done pulses this frame: rows 0 .. flushed_cnt-1 are in DDR).
    // Band r done makes row r - 1 final in fb2 (the band's output ends 19 lines above the band's end). Reconstruction
    // may not start superblock row R before row R - FRAME_WIN of fb0 (the previous occupant of its slot) is free; the
    // chain may not start band r before fb2's row r - FRAME_WIN2 is flushed.
    logic [SBW-1:0] rel_upto, rel_next, flushed_cnt;
    logic [SBW-1:0] fb0_free, fb2_free;                                     // registered (used a cycle late, below)
    logic [SBW-1:0] fb0_free_c, rel_upto_c;
    function automatic logic [SBW-1:0] minr(input logic [SBW-1:0] a, input logic [SBW-1:0] b); return (a < b) ? a : b; endfunction
    function automatic logic [SBW-1:0] dec1(input logic [SBW-1:0] a); return (a == SBW'(0)) ? SBW'(0) : a - SBW'(1); endfunction
    always_comb begin
        // "the chain past q+1" with next = q+2 is a count of q+1 final rows: next-1; a finished frame frees every row
        rel_upto_c = (fc_next >= n_rows) ? n_rows : dec1(fc_next);
        fb0_free_c = rel_upto_c;
        // intra block copy reads the rows that left fb0 from DDR, so they must be flushed first
        if (hdr.allow_intrabc) fb0_free_c = minr(fb0_free_c, flushed_cnt);
    end
    // The counts are used a cycle late. Every input only grows within a frame (the stage cursors, the flushed count), so
    // a stale count is at most one row short: a hold lasts a cycle longer, a release comes a cycle later, nothing is
    // released or started early. At 100 MHz routed the header's mi_rows through n_rows, the minimum, the
    // hold compare and the event pop into row_cnt was 29 logic levels (-0.44 ns).
    // Cleared with the frame (a stale rel_upto from the previous frame released rows at the next frame's start), and the
    // frame-done test below uses the live rel_upto_c (with the registered one, rel_next matched it a cycle before the last
    // rows were released: the 2026-10-26 nightly flushed 49,152 of 61,440 bytes).
    always_ff @(posedge clk) begin
        if (rst || frame_start) begin fb0_free <= '0; fb2_free <= '0; rel_upto <= '0; end
        else begin fb0_free <= fb0_free_c; fb2_free <= flushed_cnt; rel_upto <= rel_upto_c; end
    end
    // the chain's row store keeps 64 luma 4x4 rows (4 superblock rows of 64 lines, 2 of 128): reconstruction may not
    // start row R before the chain has read row R - 4 (R - 2), i.e. finished that band; and the chain may not start band
    // r (which writes rows r - 1 and r of fb2) before fb2's row r - FRAME_WIN2 is flushed
    logic [31:0] rs_rows;
    assign rs_rows = hdr.sb128 ? 32'd2 : 32'd4;
    assign hold_rs = ev_valid && (ev_kind == 2'd2) && (32'(sb_pop_row) >= rs_rows) && (32'(sb_pop_row) - rs_rows >= 32'(fc_next));
    assign hold_fc = (FRAME_URAM != 0) && (FRAME_WIN2 > 0) && (32'(fc_next) >= FRAME_WIN2) && (32'(fc_next) - FRAME_WIN2 >= 32'(fb2_free));
    assign hold_rc = ((FRAME_WIN > 0) && ev_valid && (ev_kind == 2'd2) && (32'(sb_pop_row) >= FRAME_WIN)
                      && (32'(sb_pop_row) - FRAME_WIN >= 32'(fb0_free))) || hold_rs;
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
    // the window build's limits (docs/frame-memory.md): 64-pixel superblocks (128 with SB128), 4:2:0 or mono, at most TE
    // tile columns (a tile column spans every row, so a window of the full width could never release one unless every
    // column of the row is decoded at once: rtl/tile_engines.sv). And in every build a tile needs its frame: the chain
    // runs under the sequencer, so a tile started without frame_start would never be filtered. Sticky until the next
    // frame, so software sees it after the tile (which may well have deadlocked on a hold).
    logic win_unsup;
    always_ff @(posedge clk) begin
        if (rst || frame_start) win_unsup <= 1'b0;
        else if ((FRAME_WIN > 0) && tile_busy && ((hdr.sb128 && SB128 == 0) || (!hdr.mono && !(hdr.ssx && hdr.ssy))
                                                  || 32'(n_tile_cols) > TE)) win_unsup <= 1'b1;   // more tile columns than engines
        else if (tile_busy && !seq_active) win_unsup <= 1'b1;
    end
    // the fb0 ring holds the row in reconstruction, the band in the chain and the hold slot: a shallower one would deadlock
    if (FRAME_URAM != 0 && FRAME_WIN > 0 && FRAME_WIN < 3) begin : g_win_depth
        $error("dec_top: FRAME_WIN must be 0 (the whole picture) or at least 3 superblock rows");
    end
    assign unsupported = ts_unsup || win_unsup || mc_unsup || mem_err;   // mc_unsup: an inter feature beyond stage 2a; mem_err: a failed reference read
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
    // the band's mode-info rows, and whether it can start: its row decoded, the stores' writes settled, a free slot in fb2
    logic fc_ok, fc_pend;                                    // (fc_pend: start issued, done not yet seen)
    always_comb begin
        fc_row_first = MIW'(fc_next) << sb_shift; fc_row_last = (MIW'(fc_next + SBW'(1)) << sb_shift) - MIW'(1);
        if (fc_row_last >= hdr.mi_rows) fc_row_last = hdr.mi_rows - MIW'(1);
        fc_ok = (fc_next < n_rows) && row_dec(fc_next) && !mi_blk_busy && !mi_tx_busy && !hold_fc;
    end
    always_ff @(posedge clk) begin
        fc_go <= 1'b0; frame_done <= 1'b0;
        if (rst) begin
            seq_active <= 1'b0; frame_busy <= 1'b0; frame_done_lvl <= 1'b0; tile_row_valid <= 1'b0; rows_seen <= 1'b0;
            fc_pend <= 1'b0; fc_next <= SBW'(0);
            for (int i = 0; i < NSBR; i++) row_cnt[i] <= 8'd0;
        end else if (frame_start) begin
            seq_active <= 1'b1; frame_busy <= 1'b1; frame_done_lvl <= 1'b0; tile_row_valid <= 1'b0;
            fc_pend <= 1'b0; fc_next <= SBW'(0);
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
            // the chain's bands
            if (!fc_pend && !fc_busy && fc_ok) begin fc_go <= 1'b1; fc_pend <= 1'b1; end
            if (fc_pend && fc_done) begin fc_pend <= 1'b0; fc_next <= fc_next + SBW'(1); end
            // frame done: every band filtered, every row released (the flush queue sees them all)
            if (rows_seen && !tile_busy && (fc_next >= n_rows) && !fc_pend && rel_next == rel_upto_c && rel_upto == rel_upto_c) begin
                seq_active <= 1'b0; frame_busy <= 1'b0; frame_done <= 1'b1; frame_done_lvl <= 1'b1;
            end
        end
    end

    // ================================================================ the streaming filter chain
    // Band r = superblock row r, started by the sequencer's fc_go when row r is reconstructed. The chain reads the row's
    // 4x4 blocks from its row store (a mirror of reconstruction's writes), the mode info from three stores written
    // alike (its planes read at the same time), and writes the final line segments into fb2 through fc_out. The band's
    // output ends 19 lines above the band's end, so row r - 1 is final when band r is done (rel_upto_c).
    logic [2:0]            fc_lf_on;
    logic                  fcb_busy, fcb_done, fc_ovf, fo_busy, fo_ovf, fc_dn_l;
    logic [2:0]            bk_re, rd2_skip, fc_out_valid, fc_out_hold;
    logic [2:0][5:0]       bk_row;
    logic [2:0][FCW-1:0]   bk_col, fc_out_col;
    logic [2:0][191:0]     bk_data;
    mi_lf_t [2:0]          fc_rd_info;
    logic [2:0][4:0]       fc_txr_sz;
    logic [2:0][3:0]       fc_cdr_val;
    lr_rec_t [2:0]         fc_lrr_rec;
    logic [2:0][95:0]      fc_out_line;
    logic [2:0][1:0]       fc_out_n4;
    logic signed [2:0][8:0] fc_out_y;
    logic [5:0]            fc_n_blk;
    logic [2:0][FBY-1:0]   fc_y0;
    logic                  lf_frame_on;
    always_comb begin
        lf_frame_on = (lh.level[5:0] != 6'd0) || (lh.level[11:6] != 6'd0);                // 7.14.1
        fc_lf_on = {lf_frame_on && (lh.level[23:18] != 6'd0), lf_frame_on && (lh.level[17:12] != 6'd0), lf_frame_on};
        fc_n_blk = 6'(fc_row_last - fc_row_first + MIW'(1));
        fc_y0[0] = FBY'({fc_row_first, 2'b00});
        fc_y0[1] = FBY'({fc_row_first >> hdr.ssy, 2'b00});
        fc_y0[2] = fc_y0[1];
    end
    // plane 0 reads the decoder's own store (u_mi); planes 1 and 2 two more, written alike
    assign fc_rd_info[0] = rd_info; assign fc_txr_sz[0] = txr_sz; assign rd2_skip[0] = cd_rd_info.skip;
    assign fc_cdr_val[0] = cdr_val; assign fc_lrr_rec[0] = lrr_rec;
    for (genvar p = 1; p < 3; p++) begin : g_mi
        // (the busy flags, the second read and the CDEF index of these two stores are plane 0's: left open)
        mi_store #(.ML2R(FRAME_WIN > 0 ? $clog2(FRAME_WIN * (SB128 != 0 ? 32 : 16)) : FBY - 2), .ML2C(FBX - 2)) u_mi (.clk, .rst,
            .blk_we(mi_blk_we), .blk_r(mi_blk_r), .blk_c(mi_blk_c), .blk_bw4(mi_blk_bw4), .blk_bh4(mi_blk_bh4), .blk_data(mi_blk_data), .blk_busy(),
            .tx_we(mi_tx_we), .tx_plane(mi_tx_plane), .tx_row(mi_tx_row), .tx_col(mi_tx_col), .tx_w4(mi_tx_w4), .tx_h4(mi_tx_h4), .tx_sz(mi_tx_sz), .tx_busy(),
            .rd_row(fc_rd_row[p]), .rd_col(fc_rd_col[p]), .rd_info(fc_rd_info[p]), .rd2_row(MIW'(0)), .rd2_col(MIW'(0)), .rd2_info(),
            .txr_plane(fc_txr_plane[p]), .txr_row(fc_txr_row[p]), .txr_col(fc_txr_col[p]), .txr_sz(fc_txr_sz[p]),
            .cd_clr(1'b0), .cd_we(1'b0), .cd_row64(SBW'(0)), .cd_col64(SBW'(0)), .cd_sb128(1'b0), .cd_mask(4'd0), .cd_idx(3'd0),
            .cdr_row64(SBW'(0)), .cdr_col64(SBW'(0)), .cdr_val(),
            .lr_we(lr_done), .lr_rec, .lrr_plane(fc_lrr_plane[p]), .lrr_row(fc_lrr_row[p]), .lrr_col(fc_lrr_col[p]), .lrr_rec(fc_lrr_rec[p]));
        assign rd2_skip[p] = 1'b0; assign fc_cdr_val[p] = 4'd0;           // a chroma plane's CDEF takes luma's record, not the store
    end
    // (the window build is 4:2:0 only: its chroma planes need half the rows and half the columns)
    // (its band row through a register here: it holds still for the whole band, and the read address is formed from it)
    logic [MIW-1:0] rs_row0; logic rs_ssy;
    always_ff @(posedge clk) begin rs_row0 <= fc_row_first; rs_ssy <= hdr.ssy; end
    fc_rstore #(.FBX(FBX), .FBY(FBY), .CW(FCW), .CWC(FRAME_WIN > 0 ? FCW - 1 : FCW), .RBY(6), .RBC(FRAME_WIN > 0 ? 5 : 6)) u_rs (.clk, .ssy(rs_ssy),
        .we(w_rc_we), .be(w_rc_be), .wplane(w_rc_pl), .wx(w_rc_x), .wy(w_rc_y), .wdata(w_rc_d),
        .row0(rs_row0), .bk_re, .bk_row, .bk_col, .bk_data);
    fc_top #(.NBMAX(32), .CW(FCW)) u_fc (.clk, .rst, .hdr, .lh, .ch, .lf_on(fc_lf_on), .cdef_on(fc_cdef_on),
        .band_start(fc_go), .row0(fc_row_first), .n_blk(fc_n_blk), .first_band(fc_next == SBW'(0)), .last_band(fc_next + SBW'(1) >= n_rows),
        .busy(fcb_busy), .band_done(fcb_done), .err_ovf(fc_ovf),
        .bk_re, .bk_row, .bk_col, .bk_data,
        .rd_row(fc_rd_row), .rd_col(fc_rd_col), .rd_info(fc_rd_info), .txr_plane(fc_txr_plane), .txr_row(fc_txr_row), .txr_col(fc_txr_col), .txr_sz(fc_txr_sz),
        .rd2_row(fc_rd2_row), .rd2_col(fc_rd2_col), .rd2_skip, .cdr_row64(fc_cdr_row64), .cdr_col64(fc_cdr_col64), .cdr_val(fc_cdr_val),
        .lrr_plane(fc_lrr_plane), .lrr_row(fc_lrr_row), .lrr_col(fc_lrr_col), .lrr_rec(fc_lrr_rec),
        .out_hold(fc_out_hold), .out_valid(fc_out_valid), .out_line(fc_out_line), .out_n4(fc_out_n4), .out_col(fc_out_col), .out_y(fc_out_y));
    fc_out #(.FBX(FBX), .FBY(FBY), .CW(FCW)) u_fo (.clk, .rst, .y0(fc_y0), .in_valid(fc_out_valid), .in_line(fc_out_line), .in_n4(fc_out_n4),
        .in_col(fc_out_col), .in_y(fc_out_y), .out_hold(fc_out_hold), .busy(fo_busy), .err_ovf(fo_ovf),
        .we(fo_we), .wplane(fo_plane), .wx(fo_x), .wy(fo_y), .wdata(fo_d));
    // the stage is done for the sequencer when the band is and its last segments are in fb2
    always_ff @(posedge clk) begin
        fc_done <= 1'b0;
        if (rst || frame_start) fc_dn_l <= 1'b0;
        else begin
            if (fcb_done) fc_dn_l <= 1'b1;
            if (fc_dn_l && !fo_busy) begin fc_dn_l <= 1'b0; fc_done <= 1'b1; end
        end
    end
    assign fc_busy = fcb_busy || fc_dn_l || fo_busy;
`ifndef SYNTHESIS
    always_ff @(posedge clk) if (!rst && (fc_ovf || fo_ovf)) $error("[chain] a line was dropped at a FIFO (fc_ovf %0d fo_ovf %0d)", fc_ovf, fo_ovf);
`endif
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
            if (fc_pend && fc_done)
                $fwrite(vis_fd, "F %0d L %0d %0d\n", vis_cyc, 32'(fc_row_first) * 4, 32'(fc_row_last) * 4 + 3);
            if (frame_done)
                $fwrite(vis_fd, "E %0d\n", vis_cyc);
        end
    end
`endif
`ifndef SYNTHESIS
    time trace_last_rc_w;                                                           // window trace (simulation only)
    always_ff @(posedge clk) if (w_rc_we) trace_last_rc_w <= $time;
    always_ff @(posedge clk) if (FRAME_WIN > 0 && !rst) begin                      // window trace (simulation only)
        if (fc_pend && fc_done) $display("[win] t=%0t band done row %0d (n_rows %0d) rel_upto->%0d", $time, fc_next, n_rows, (fc_next + SBW'(1) >= n_rows) ? n_rows : fc_next);
        if (row_final) $display("[win] t=%0t row_final %0d", $time, row_final_idx);
        if (flush_done) $display("[win] t=%0t flush_done (flushed_cnt -> %0d)", $time, flushed_cnt + 1);
        if (frame_done) $display("[win] t=%0t frame_done rel_next %0d rel_upto %0d", $time, rel_next, rel_upto);
        if (frame_start) $display("[win] t=%0t frame_start n_rows %0d", $time, n_rows);
        if (fc_go) $display("[win] t=%0t band start row %0d MI rows %0d..%0d (hdr.mi_rows %0d, tile_busy %0d rc_busy %0d row_cnt %0d/%0d/%0d, last recon fb0 write t=%0t)", $time, fc_next,
                            fc_row_first, fc_row_last, hdr.mi_rows, tile_busy, rc_busy, row_cnt[0], row_cnt[1], row_cnt[2], trace_last_rc_w);
        if (tile_done) $display("[win] t=%0t tile_done (rc_busy %0d, last recon fb0 write t=%0t)", $time, rc_busy, trace_last_rc_w);
        if (sb_pop) $display("[win] t=%0t sb_pop row %0d (prev %0d valid %0d)", $time, sb_pop_row, tile_row_prev, tile_row_valid);
        if (hold_rc && !rc_busy && ev_pop) $display("[win] t=%0t hold row %0d (fb0 free %0d, flushed %0d)", $time, sb_pop_row, fb0_free, flushed_cnt);
        if (hold_fc && !fc_pend && !fc_busy) $display("[win] t=%0t hold band %0d (fb2 free %0d)", $time, fc_next, fb2_free);
    end
    // heartbeat (simulation only): the sequencer's state every 2^18 cycles while a frame is active, so a stalled twin
    // shows which stage is waiting for what (and whether simulated time still advances at all)
    int trace_hb;
    always_ff @(posedge clk) if (FRAME_WIN > 0 && !rst) begin
        trace_hb <= frame_start ? 0 : trace_hb + 1;
        if (seq_active && trace_hb[17:0] == 18'd0)
            $display("[win] t=%0t hb: tile_busy %0d rc_busy %0d ev valid/kind/pop %0d/%0d/%0d sb_r %0d hold rc/fc/rs %0d%0d%0d | band %0d pend %0d busy %0d | rel %0d/%0d flushed %0d free fb0/2 %0d/%0d | row_cnt[n-2] %0d [n-1] %0d",
                     $time, tile_busy, rc_busy, ev_valid, ev_kind, ev_pop, sb_pop_row, hold_rc, hold_fc, hold_rs,
                     fc_next, fc_pend, fc_busy, rel_next, rel_upto, flushed_cnt, fb0_free, fb2_free,
                     row_cnt[(FBY - 6)'(n_rows - SBW'(2))], row_cnt[(FBY - 6)'(n_rows - SBW'(1))]);
    end
`endif
endmodule
