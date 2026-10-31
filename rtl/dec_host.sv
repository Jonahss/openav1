// Host interface for the FPGA builds: an AXI4-Lite register file around dec_top, so that software on the PS
// (or a JTAG master) drives the decoder the way tb/test_dec_top.py does: headers and CDF defaults written
// into registers, tile bytes pushed into a FIFO, stage starts pulsed, done bits polled, the picture read back
// one sample at a time through the frame memory's host port. Everything runs on clk (the AXI clock is the
// decoder clock; the PS side's own clock domain crossing is in its interconnect). Register map (byte offsets,
// 32-bit words; the words of a wide field are little-endian: word 0 = bits 31:0):
//   0x000 ID        R   32'hA71D_0001
//   0x004 CFG       R   {4'd0, TE[3:0], FBX[7:0], FBY[7:0], ITX_LANES[3:0], 1'd0, EOS_PER_ENGINE, SB128, FRAME_URAM}
//                       (bit 2: a CTRL write with bit 8 ends the stream of the engine CTRL[27:24] selects, as soon as that
//                       engine's bytes are in; images without it end every engine's stream with one write at the end)
//   0x008 CTRL      RW  bit 0 rst (decoder + FIFO reset while set), 1 pipe_en, 2 lr_from_deblocked, 3 sr_buf,
//                       [5:4] h_buf (the host port's frame buffer: 0 fb0, 1 CdefFrame, 2 LrFrame, 3 SR's upscaled rows
//                       in the RING build; a row flush reads dec_top's final_buf instead), 8 eos (no more bytes for
//                       this tile: in_eos once the FIFO drains; cleared by a tile start), [21:16] n_tile_cols
//   0x00C START     W   one-cycle pulses: bit 0 tile_start (also empties the tile FIFO: push the bytes after it), 1 frame_start, 2 lf_start, 3 cdef_start, 4 sr_start,
//                       5 lr_start (a start clears its stage's sticky done bit)
//   0x010 STATUS    R   bit 0 tile_busy, 1 tile_done_lvl, 2 unsupported, 3 frame_busy, 4 frame_done_lvl,
//                       5 lf_busy, 6 lf_done (sticky), 7 cdef_busy, 8 cdef_done (sticky), 9 sr_busy,
//                       10 sr_done (sticky), 11 lr_busy, 12 lr_done (sticky), 13 in_ready, 14 fifo overflow or a
//                       push with a partial write strobe (sticky until rst), 15 fifo empty (and no byte pending),
//                       [31:16] free FIFO words
//   0x014 PERF_SYN  R   perf_syn_stall       0x018 PERF_REC  R   perf_rec_idle
//   0x01C DATA1     W   push one tile byte [7:0]        0x020 DATA4  W  push four bytes, [7:0] first (into the FIFO of
//                       engine CTRL[25:24]; STATUS bits 13 / 14 and [31:16] describe that engine's FIFO; CDF read-back too)
//   0x038 TILE_ENG  RW  engines of the tile row [TE-1:0] (0: engine 0 with the header's columns); START.tile starts them all
//   0x060 + 4e      RW  engine e MiColStart [12:0] / MiColEnd [28:16] (TE > 1 and TILE_ENG != 0)
//   0x070 + 4e      R   engine e FIFO: free words [31:16], in_ready [1], empty [0]
//                       (a push into a full FIFO, or one whose write strobe is not the whole word, is dropped
//                       and sets STATUS bit 14: poll the free count; make each push one 32-bit bus transfer)
//   0x024 HADDR     RW  host read address {2'd0, plane[1:0], y[13:0], x[13:0]}: a write fetches the four samples at
//                       x .. x+3 of that row into a buffer (16 cycles); a data read before they are in waits
//   0x028 HDATA     R   sample x [11:0] (the address stays)
//   0x030 HDATA2    R   {4'd0, sample x+1, 4'd0, sample x}: then x += 2 and the next four are fetched
//   0x034 HDATA4    R   {x+3, x+2, x+1, x}[7:0] (8-bit pictures): then x += 4 and the next four are fetched
//   0x02C DEF_ADDR  W   CDF default row address; the write strobes def_we with DEF_DATA. Bit 31 set: read back
//                       that row of the tile's working store into DEF_DATA[0..7] instead (the adapted CDFs after a tile;
//                       only between tiles: a decoding tile owns the store). Reads wait until the row is in.
//   0x040-0x05C DEF_DATA[0..7]  RW  246-bit default row
//   0x080-0x0BC HDR[0..15]  RW  syn_pkg::hdr_t     0x0C0-0x0FC RH[0..15]  RW  rec_hdr_t
//   0x100-0x13C LH[0..15]   RW  lf_hdr_t           0x140-0x17C CH[0..15]  RW  cdef_hdr_t
//   0x200-0x2FC IH[0..63]   RW  syn_pkg::ihdr_t (the inter frame header; zero for intra frames)
//   Inter frame bands (stage 1: written by the host before the frame; a projection engine takes over in stage 2):
//   0x300 MF_ADDR   RW  motion field entry {valid[31], ref[26:24] (LAST_FRAME = 0), y8[23:12], x8[11:0]}
//   0x304 MF_DATA   W   {row[31:16], col[15:0]} (1/8 samples, signed): the write stores the entry at MF_ADDR
//   0x308 SP_ADDR   RW  previous segmentation map word {row[27:16], word column[11:0]} (16 ids of 3 bits per word)
//   0x30C SP_DATA0  RW  ids 0 .. 9 and the low bits of id 10 (bits 31:0 of the 48-bit word)
//   0x310 SP_DATA1  W   bits 47:32: the write stores the word at SP_ADDR
//   Reference slot table (stage 2a inter prediction; ref_fetch reads the slots through m_axi's read channels):
//   0x400 + 0x40 s + 0x10 p   slot s (0..7), plane p (0..2): +0 BASE_LO, +4 BASE_HI (bits 39:32), +8 STRIDE (bytes);
//   0x400 + 0x40 s + 0x30     slot s SIZE {frame height[31:16], upscaled width[15:0]} in luma samples
//   0x400 + 0x40 s + 0x34     slot s SCALE {yScale[31:16], xScale[15:0]} of 7.11.3.3 for the current frame
//                             (((RefUpscaledWidth << 14) + FrameWidth / 2) / FrameWidth; 16384 = unscaled)
//   0x600 REF_SLOT  RW  the slot of LAST_FRAME + i at [3i +: 3] (ref_frame_idx)      0x604 REF_CFG  RW  bit 0 fmt16
//   Picture DMA (the decoded frame into DDR through the AXI4 write master m_axi, 64-bit data, 40-bit address):
//   0x180 DMA_CFG   RW  bit 0 fmt16 (16-bit little-endian samples; else one byte per sample, 8-bit pictures),
//                       [5:4] number of planes - 1, bit 8 auto-flush (window builds, FRAME_WIN > 0: every superblock
//                       row the decoder releases is written to the plane bases / strides as it becomes final; program
//                       the planes before START.frame and wait for DMA_STATUS bit 3 to clear after frame_done)
//   0x184 DMA_START W   any write starts a run (the decoder must be idle: the engine owns the frame memory's host
//                       port while it runs, and data reads through HDATA* wait)
//   0x188 DMA_STATUS R  bit 0 busy, 1 done (sticky, cleared by a start), 2 error (a BRESP other than OKAY, sticky until the
//                       next frame start or software DMA start: an auto flush does not clear it),
//                       3 flush busy (rows queued or being written), [23:16] rows flushed since the last frame start
//   0x18C DMA_BEATS  R  64-bit beats written by the last run
//   Frame-state sideband (rtl/frame_state_w.sv, docs/frame-state.md; INTER window builds): per finished superblock row
//   the card writes its MV words (8x8 units, 2 per beat) and segment ids (4x4 units, one byte each) to DDR:
//   0x1C0 FS_CFG     RW  bit 0 enable (rows are dropped when clear)
//   0x1C4 FS_MV_LO / 0x1C8 FS_MV_HI    RW  MV area base (40-bit); 0x1CC FS_MV_ROWB  RW  bytes per 8x8 line (multiple of 8)
//   0x1D0 FS_SEG_LO / 0x1D4 FS_SEG_HI  RW  segment-id area base;  0x1D8 FS_SEG_ROWB RW  bytes per 4x4 line (multiple of 8)
//   0x1DC FS_STATUS  R   bit 0 sideband busy, 1 sideband error (BRESP), 2 load busy, 3 load done (sticky), 4 load error,
//                       5 save busy, 6 save done (sticky), 7 save error, 8 a reference read failed on the AXI bus (sticky, also
//                       STATUS bit 2 unsupported), [23:16] sideband rows written since the frame start
//   Frame-state loader / CDF save (rtl/fs_load.sv, stage B): START bit 6 loads the segment map and / or the CDF rows from
//   the reference slot's areas (before the frame's first tile); START bit 7 saves engine SV_ENG's CDF rows to SV_CDF
//   (after the context tile's row, before the next tile row starts):
//   0x700 LD_CFG     RW  bit 0 load the segment map, bit 1 load the CDF rows, bit 2 zero the segment map (PrevSegmentIds = 0),
//                       bit 3 project the motion field after the loads (rtl/mf_project.sv, spec 7.9) from the table below:
//   0x724 MF_NSRC    RW  sources to project, 0..5, in the spec's order;  0x728 / 0x72C MF_DST  RW  refToDst for destinations
//                       1..4 / 5..7 (signed bytes, destination d at byte (d - 1) % 4)
//   0x740 + 0x20 s   source s (0..4): +0 BASE_LO, +4 BASE_HI (its MV sideband area), +8 ROWB (bytes per 8x8 line),
//                       +C {sign[8] (1: dstSign = +1), refToCur[7:0] signed}, +10 OFF0 (refOffset of saved references
//                       1..4, signed bytes), +14 OFF1 (references 5..7)
//   0x704 LD_SEG_LO / 0x708 LD_SEG_HI  RW  segment-id area; 0x70C LD_SEG_ROWB RW bytes per 4x4 line
//   0x710 LD_CDF_LO / 0x714 LD_CDF_HI  RW  CDF area to load (CDF_ROWS x 32 bytes)
//   0x718 SV_CDF_LO / 0x71C SV_CDF_HI  RW  CDF area to save into;  0x720 SV_ENG RW the engine whose rows are saved
//   0x190 + 0x10 p  plane p (0..2): +0 BASE_LO (bits 31:0), +4 BASE_HI (bits 39:32), +8 STRIDE (bytes),
//                       +C SIZE {height[31:16], width[15:0]} in samples. Base and stride are multiples of 8 (the
//                       driver rounds the stride up); a row goes out as 64-bit beats, the last with the byte strobes
//                       of the samples that exist, in INCR bursts of up to 16 beats that never cross 4 KB.
// Reads of other words return 0; writes are ignored. (tb/test_dec_host.py drives this map and records the
// register transcript tools/kv260 replays on the board.)
module dec_host
    import syn_pkg::*;
    import cdf_map_pkg::*;
#(
    parameter int FBX = 9,
    parameter int FBY = 9,
    parameter int ITX_LANES = 2,
    parameter int FRAME_URAM = 1,
    parameter int FRAME_WIN = 0,             // the frame memory as a window of this many superblock rows (0: whole picture)
    parameter int FRAME_WIN1 = -1,           // fb1 / fb2 rings with FRAME_URAM (dec_top; -1: left out, the core build)
    parameter int FRAME_WIN2 = -1,
    parameter int FIFO_LW = 10,              // tile-byte FIFO: 2^FIFO_LW words of up to 4 bytes (one BRAM36 at 10)
    parameter int INTER = 1,                 // 0: the intra-only decoder (dec_top INTER)
    parameter int SB128 = 0,                 // 1: the window build takes 128x128 superblocks (dec_top SB128; CFG bit 1)
    parameter int TE = 1                     // tile engines (dec_top TE; CFG[27:24]): the tile columns of a tile row decoded at once
) (
    input  logic        clk,
    input  logic        clk2x,
    input  logic        phase,
    input  logic        aresetn,
    // AXI4-Lite slave (the block design wraps this in fpga/kv260/dec_host_v.v, which carries the interface attributes)
    input  logic [11:0] s_axi_awaddr,
    input  logic        s_axi_awvalid,
    output logic        s_axi_awready,
    input  logic [31:0] s_axi_wdata,
    input  logic [3:0]  s_axi_wstrb,
    input  logic        s_axi_wvalid,
    output logic        s_axi_wready,
    output logic [1:0]  s_axi_bresp,
    output logic        s_axi_bvalid,
    input  logic        s_axi_bready,
    input  logic [11:0] s_axi_araddr,
    input  logic        s_axi_arvalid,
    output logic        s_axi_arready,
    output logic [31:0] s_axi_rdata,
    output logic [1:0]  s_axi_rresp,
    output logic        s_axi_rvalid,
    input  logic        s_axi_rready,
    // AXI4 write master: the decoded picture into DDR (the read channels are tied off)
    output logic [39:0] m_axi_awaddr,
    output logic [7:0]  m_axi_awlen,
    output logic [2:0]  m_axi_awsize,
    output logic [1:0]  m_axi_awburst,
    output logic [3:0]  m_axi_awcache,
    output logic [2:0]  m_axi_awprot,
    output logic        m_axi_awvalid,
    input  logic        m_axi_awready,
    output logic [63:0] m_axi_wdata,
    output logic [7:0]  m_axi_wstrb,
    output logic        m_axi_wlast,
    output logic        m_axi_wvalid,
    input  logic        m_axi_wready,
    input  logic [1:0]  m_axi_bresp,
    input  logic        m_axi_bvalid,
    output logic        m_axi_bready,
    output logic [39:0] m_axi_araddr,
    output logic [7:0]  m_axi_arlen,
    output logic [2:0]  m_axi_arsize,
    output logic [1:0]  m_axi_arburst,
    output logic [3:0]  m_axi_arcache,
    output logic [2:0]  m_axi_arprot,
    output logic        m_axi_arvalid,
    input  logic        m_axi_arready,
    input  logic [63:0] m_axi_rdata,
    input  logic [1:0]  m_axi_rresp,
    input  logic        m_axi_rlast,
    input  logic        m_axi_rvalid,
    output logic        m_axi_rready
);
    localparam int HDR_W = $bits(hdr_t), RH_W = $bits(rec_hdr_t), LH_W = $bits(lf_hdr_t), CH_W = $bits(cdef_hdr_t), IH_W = $bits(ihdr_t);
    initial begin
        if (HDR_W > 512 || RH_W > 512 || LH_W > 512 || CH_W > 512) $error("dec_host: a header struct exceeds its 16-word window");
        if (IH_W > 2048) $error("dec_host: ihdr_t exceeds its 64-word window");
        if (FBX > 14 || FBY > 14) $error("dec_host: HADDR holds 14-bit coordinates");
    end

    // ---------------------------------------------------------------- registers
    logic [31:0] ctrl;
    logic [31:0] haddr;
    logic [31:0] def_w [8];
    logic [31:0] hdr_w [16], rh_w [16], lh_w [16], ch_w [16];
    logic [31:0] ih_w [64];
    logic        rst;                                        // decoder + FIFO reset
    assign rst = !aresetn || ctrl[0];

    // ---------------------------------------------------------------- the decoder
    hdr_t      hdr;
    rec_hdr_t  rh;
    lf_hdr_t   lh;
    cdef_hdr_t ch;
    ihdr_t     ihdr;
    logic [511:0] hdr_f, rh_f, lh_f, ch_f;
    logic [2047:0] ih_f;
    always_comb begin
        for (int i = 0; i < 16; i++) begin
            hdr_f[32*i +: 32] = hdr_w[i]; rh_f[32*i +: 32] = rh_w[i]; lh_f[32*i +: 32] = lh_w[i]; ch_f[32*i +: 32] = ch_w[i];
        end
        for (int i = 0; i < 64; i++) ih_f[32*i +: 32] = ih_w[i];
        hdr = hdr_t'(hdr_f[HDR_W-1:0]); rh = rec_hdr_t'(rh_f[RH_W-1:0]); lh = lf_hdr_t'(lh_f[LH_W-1:0]); ch = cdef_hdr_t'(ch_f[CH_W-1:0]);
        ihdr = ihdr_t'(ih_f[IH_W-1:0]);
    end
    logic [TE*8-1:0]   in_data;
    logic [TE-1:0]     in_valid, in_ready, in_eos;
    localparam int EW = $clog2(TE > 1 ? TE : 2);
    logic [EW-1:0]     eng_sel;                                // CTRL[25:24]: the engine DATA1 / DATA4 push to, the CDF read back, STATUS's FIFO
    logic [TE-1:0]     eng_mask_r;
    logic [TE*MIW-1:0] col_start_r, col_end_r;
    logic              def_we;
    logic [CDF_AW-1:0] def_addr;
    logic              rb_en, rb_l1;                 // cdf read-back: request pulse, row-in cycle
    logic [245:0]      rb_data;
    logic [31:0]       mf_addr_r, sp_addr_r, sp_d0;    // inter frame band writes
    logic              mf_we, sp_we;
    logic [31:0]       mf_wd;
    logic [15:0]       sp_d1;
    logic [245:0]      def_data;
    logic [7:0]        start;                                // tile, frame, lf, cdef, sr, lr, state load, cdf save
    logic              tile_busy, tile_done, tile_done_lvl, unsupported;
    logic [31:0]       perf_syn_stall, perf_rec_idle;
    logic              frame_busy, frame_done, frame_done_lvl;
    logic              lf_busy, lf_done, cdef_busy, cdef_done, sr_busy, sr_done, lr_busy, lr_done_o;
    logic              blk_done, tx_done, lr_done;
    blk_rec_t          blk_rec;
    tx_rec_t           tx_rec;
    lr_rec_t           lr_rec;
    logic [11:0]       h_rdata;
    logic [47:0]       h_rdata4;
    logic              h_ok;
    logic [1:0]        h_plane_m;                            // the host port: the DMA engine's while it runs, else the register prefetch's
    logic [FBX-1:0]    h_x_m;
    logic [FBY-1:0]    h_y_m;
    logic           row_final, flush_done;
    logic [SBW-1:0] row_final_idx;
    logic [1:0]     final_buf;
    logic           flush_rd;                                // the host port belongs to a running flush (reads the final buffer)
    logic           dma_busy, dma_done, dma_err;
    logic           fl_run;                                  // the run in progress is a flush (else a software picture DMA)
    logic [39:0]    rs_base [8][3];                          // reference slot table (declared ahead of the dec_top instance below:
    logic [31:0]    rs_stride [8][3];                        // Vivado makes an implicit 1-bit wire of a name used before its declaration)
    logic [15:0]    rs_w [8], rs_h [8];
    logic [15:0]    rs_xs [8], rs_ys [8];                    // 7.11.3.3 scale factors per slot (for the current frame)
    logic [20:0]    ref_slot_r;
    logic [31:0]    ref_cfg_r;
    // frame-state sideband DMA (declared ahead of the dec_top instance)
    logic           fs_row_ready, fs_done, fs_rd_sel, fs_rd_buf;
    logic [SBW-1:0] fs_row_idx;
    logic [4:0]     fs_rd_line;
    logic [8:0]     fs_rd_beat;
    logic [63:0]    fs_rd_data;
    logic [31:0]    fs_cfg;
    logic [39:0]    fs_mv_base, fs_seg_base;
    logic [15:0]    fs_mv_rowb, fs_seg_rowb;
    logic           fs_active, fs_err;
    logic [7:0]     fs_rows;
    logic [39:0]    fs_addr;
    logic [4:0]     fs_len, fs_cnt;
    logic           fs_awvalid, fs_wvalid, fs_bready, fs_wlast;
    logic [31:0]    ld_cfg_r;
    logic [39:0]    ld_seg_base, ld_cdf_base, sv_cdf_base;
    logic [15:0]    ld_seg_rowb;
    logic           ld_busy, ld_done, ld_err, ld_done_s;
    logic [EW-1:0]  sv_eng;
    logic           sv_busy, sv_done_s, sv_err, sv_rb_en, sv_rb_l1;
    logic [CDF_AW-1:0] sv_row;
    logic [255:0]   sv_data;
    logic           mem_err;                                 // recon_top: a reference read failed (also raises STATUS unsupported)
    // the projection engine's table (rtl/mf_project.sv): 5 sources x (base, rowb, sign + refToCur, 7 refOffsets), 7 refToDst
    logic [31:0]    mf_nsrc_r;
    logic [199:0]   mf_src_base;
    logic [79:0]    mf_src_rowb;
    logic [4:0]     mf_src_sign;
    logic [39:0]    mf_src_tocur;
    logic [279:0]   mf_src_off;
    logic [55:0]    mf_dst_dist;
    logic [215:0]   cur_planes;                              // the current picture's flush layout for intra block copy
    logic           fmt16;                                  // DMA_CFG bit 0, declared ahead of the core instance that takes it (cur_fmt16)
    dec_top #(.FBX(FBX), .FBY(FBY), .ITX_LANES(ITX_LANES), .FRAME_URAM(FRAME_URAM), .FRAME_WIN(FRAME_WIN), .FRAME_WIN1(FRAME_WIN1), .FRAME_WIN2(FRAME_WIN2), .INTER(INTER), .SB128(SB128), .TE(TE)) u_dec (
        .clk, .clk2x, .phase, .rst, .hdr, .ihdr, .rh, .lh, .ch,
        .in_data, .in_valid, .in_ready, .in_eos,
        .def_we, .def_addr, .def_data, .rb_en(rb_en | sv_rb_en), .rb_addr(sv_busy ? sv_row : def_addr), .rb_sel(sv_busy ? sv_eng : eng_sel), .rb_data,
        .ld_start(start[6]), .ld_cfg(ld_cfg_r[3:0]), .ld_seg_base, .ld_seg_rowb, .ld_cdf_base, .ld_busy, .ld_done, .ld_err,
        .mf_nsrc(mf_nsrc_r[2:0]), .mf_src_base, .mf_src_rowb, .mf_src_sign, .mf_src_tocur, .mf_src_off, .mf_dst_dist,
        .mf_we, .mf_wref(mf_addr_r[26:24]), .mf_wy8((MIW-1)'(mf_addr_r[23:12])), .mf_wx8((MIW-1)'(mf_addr_r[11:0])), .mf_wvalid(mf_addr_r[31]),
        .mf_wrow(signed'(mf_wd[31:16])), .mf_wcol(signed'(mf_wd[15:0])),
        .sp_we, .sp_wrow(MIW'(sp_addr_r[27:16])), .sp_wwc((MIW-4)'(sp_addr_r[11:0])), .sp_wdata({sp_d1, sp_d0}),
        .ref_slot(ref_slot_r), .ref_planes, .ref_dims, .ref_scales, .ref_fmt16(ref_cfg_r[0]), .cur_planes, .cur_fmt16(fmt16),
        .m_axi_araddr, .m_axi_arlen, .m_axi_arsize, .m_axi_arburst, .m_axi_arcache, .m_axi_arprot, .m_axi_arvalid, .m_axi_arready,
        .m_axi_rdata, .m_axi_rresp, .m_axi_rlast, .m_axi_rvalid, .m_axi_rready,
        .tile_start(start[0]), .eng_mask(eng_mask_r), .col_start(col_start_r), .col_end(col_end_r), .tile_busy, .tile_done, .tile_done_lvl, .unsupported, .mem_err, .perf_syn_stall, .perf_rec_idle,
        .pipe_en(ctrl[1]), .frame_start(start[1]), .n_tile_cols(ctrl[21:16]), .frame_busy, .frame_done, .frame_done_lvl,
        .row_final, .row_final_idx, .flush_done,
        .fs_row_ready, .fs_row_idx, .fs_done, .fs_rd_sel, .fs_rd_line, .fs_rd_beat, .fs_rd_buf, .fs_rd_data,
        .lf_start(start[2]), .lf_busy, .lf_done,
        .cdef_start(start[3]), .cdef_busy, .cdef_done,
        .sr_start(start[4]), .sr_buf(ctrl[3]), .sr_busy, .sr_done,
        .lr_start(start[5]), .lr_from_deblocked(ctrl[2]), .lr_busy, .lr_done_o,
        .blk_done, .blk_rec, .tx_done, .tx_rec, .lr_done, .lr_rec,
        .h_buf(flush_rd ? final_buf : ctrl[5:4]), .h_plane(h_plane_m), .h_x(h_x_m), .h_y(h_y_m), .h_rdata, .h_rdata4, .h_ok, .final_buf
    );
    logic [CDF_AW-1:0] def_addr_r;
    always_comb begin
        def_addr = def_addr_r;
        for (int i = 0; i < 7; i++) def_data[32*i +: 32] = def_w[i];
        def_data[245:224] = def_w[7][21:0];
    end

    // sticky stage-done bits (the stage outputs are one-cycle pulses)
    logic lf_done_s, cdef_done_s, sr_done_s, lr_done_s;
    always_ff @(posedge clk) begin
        if (rst) begin lf_done_s <= 1'b0; cdef_done_s <= 1'b0; sr_done_s <= 1'b0; lr_done_s <= 1'b0; end
        else begin
            lf_done_s   <= start[2] ? 1'b0 : (lf_done_s   | lf_done);
            cdef_done_s <= start[3] ? 1'b0 : (cdef_done_s | cdef_done);
            sr_done_s   <= start[4] ? 1'b0 : (sr_done_s   | sr_done);
            lr_done_s   <= start[5] ? 1'b0 : (lr_done_s   | lr_done_o);
        end
    end

    // ---------------------------------------------------------------- tile-byte FIFOs (words of 1..4 bytes), one per engine
    localparam int FD = 1 << FIFO_LW;
    logic               push;
    logic [33:0]        push_d;
    logic               f_ovf, f_badw;                       // overflow (any engine) / partial-strobe push: both STATUS bit 14
    logic [TE-1:0]      f_ovf_e, f_empty_e;
    logic [TE-1:0]      eos_e;                               // end of stream per engine: set by a CTRL write with bit 8 for the
                                                             // engine it selects, cleared by a tile start (2026-10-28: one global
                                                             // bit set after every engine's bytes deadlocked a two-engine row whose
                                                             // first engine needed it to finish while the second held a palette block)
    logic [TE-1:0][FIFO_LW:0] f_free_e;                   // packed: the register read mux indexes it by engine (an unpacked
                                                             // array driven per element from the generate loop came out of Vivado
                                                             // returning engine 0's count for every engine; gate te2a/te2b 2026-10-29)
    assign eng_sel = (TE > 1) ? ctrl[24 +: EW] : '0;
    // A tile start empties the FIFOs: the arithmetic decoder leaves a tile's trailing bytes unread, and the next tile of
    // the same frame must not see them (2026-10-24: test47's third tile decoded from the second tile's tail; every
    // earlier stream through dec_host had one tile per frame). Software pushes a tile's bytes after its START.
    for (genvar e = 0; e < TE; e++) begin : g_fifo
        logic [33:0]        fmem [0:FD-1];                   // {count-1, bytes}
        logic [FIFO_LW:0]   wptr, rptr;
        logic               f_empty, f_full;
        logic               ow_valid;                        // the word being handed out byte by byte
        logic [31:0]        ow;
        logic [1:0]         ow_n, ow_i;
        logic               take, ow_done, fetch, push_e;
        assign push_e  = push && (eng_sel == EW'(e));
        assign f_empty = (wptr == rptr);
        assign f_full  = (wptr[FIFO_LW-1:0] == rptr[FIFO_LW-1:0]) && (wptr[FIFO_LW] != rptr[FIFO_LW]);
        assign f_free_e[e] = (FIFO_LW + 1)'(FD) - (wptr - rptr);
        assign f_empty_e[e] = !ow_valid && f_empty;
        assign take    = ow_valid && in_ready[e];
        assign ow_done = take && (ow_i == ow_n);
        assign fetch   = !f_empty && (!ow_valid || ow_done);
        always_ff @(posedge clk) begin
            if (rst || start[0]) begin wptr <= '0; rptr <= '0; ow_valid <= 1'b0; f_ovf_e[e] <= 1'b0; ow_i <= 2'd0; end
            else begin
                if (push_e) begin
                    if (f_full) f_ovf_e[e] <= 1'b1;
                    else begin fmem[wptr[FIFO_LW-1:0]] <= push_d; wptr <= wptr + 1'b1; end
                end
                if (fetch) begin
                    {ow_n, ow} <= fmem[rptr[FIFO_LW-1:0]]; rptr <= rptr + 1'b1; ow_valid <= 1'b1; ow_i <= 2'd0;
                end else if (ow_done) ow_valid <= 1'b0;
                else if (take) ow_i <= ow_i + 1'b1;
            end
        end
        assign in_valid[e] = ow_valid;
        assign in_data[e * 8 +: 8] = ow[8 * ow_i +: 8];
        assign in_eos[e]   = eos_e[e] && !ow_valid && f_empty;
    end
    assign f_ovf = |f_ovf_e;
    logic [FIFO_LW:0]   f_free;                              // the selected engine's, for STATUS
    logic               f_empty_sel, in_ready_sel;
    assign f_free = f_free_e[eng_sel];
    assign f_empty_sel = f_empty_e[eng_sel];
    assign in_ready_sel = in_ready[eng_sel];

    // ---------------------------------------------------------------- picture read-back prefetch
    // After an HADDR write, and after a packed read advanced the address, the four samples at x .. x+3 of the
    // addressed row are read into hbuf one after another, each address held for HL cycles (both frame memories
    // answer the cycle after the address; the hold covers that with margin, 16 cycles per fetch against a
    // microsecond per AXI transaction from the PS). A data read while the fetch is in progress waits on the
    // AXI read channel (arready stays low), so software never sees stale samples.
    localparam int HL = 3;
    logic [FBX-1:0] hx_q;                                    // the address presented to the decoder's host port
    logic [11:0]    hbuf [4];
    logic           hbuf_ok, h_busy, h_fetch;                // h_fetch: (re)start at haddr (pulse from the AXI side)
    logic [1:0]     h_idx, h_cnt;
    always_ff @(posedge clk) begin
        if (!aresetn) begin hbuf_ok <= 1'b0; h_busy <= 1'b0; h_idx <= 2'd0; h_cnt <= 2'd0; hx_q <= '0; end
        else if (h_fetch) begin
            hbuf_ok <= 1'b0; h_busy <= 1'b1; h_idx <= 2'd0; h_cnt <= 2'd0; hx_q <= haddr[FBX-1:0];
        end else if (h_busy) begin
            if (h_cnt == 2'(HL)) begin
                hbuf[h_idx] <= h_rdata;
                h_cnt <= 2'd0;
                hx_q  <= hx_q + 1'b1;
                h_idx <= h_idx + 1'b1;
                if (h_idx == 2'd3) begin h_busy <= 1'b0; hbuf_ok <= 1'b1; end
            end else h_cnt <= h_cnt + 1'b1;
        end
    end
    logic h_data_read;                                       // a read of HDATA / HDATA2 / HDATA4

    // ---------------------------------------------------------------- picture DMA engine (AXI4 write master)
    logic [31:0]    dma_cfg;
    logic           dma_start;                               // pulse from the AXI-Lite side
    // auto-flush (window builds): released rows queue here; a run then covers that row's lines of every plane,
    // from per-plane row cursors that advance as rows are flushed (no multiply: rows flush in order)
    localparam int FQ = 8;
    logic [SBW-1:0] fq [FQ];
    logic [3:0]     fq_w, fq_r;
    logic           fq_empty, fq_full;
    assign fq_empty = (fq_w == fq_r);
    assign fq_full  = (fq_w[2:0] == fq_r[2:0]) && (fq_w[3] != fq_r[3]);
    logic [39:0]    fl_addr [3];                             // next row's first address per plane
    logic [15:0]    fl_y [3];                                // next row's first line per plane
    logic [7:0]     fl_rows;                                 // rows flushed since the last frame start
    logic           fl_auto;
    assign fl_auto = dma_cfg[8];
    assign flush_rd = dma_busy && fl_run;
    logic [1727:0]  ref_planes;
    logic [255:0]   ref_dims, ref_scales;
    always_comb begin
        for (int sl = 0; sl < 8; sl++) begin
            for (int pl = 0; pl < 3; pl++) ref_planes[72 * (3 * sl + pl) +: 72] = {rs_stride[sl][pl], rs_base[sl][pl]};
            ref_dims[32 * sl +: 32] = {rs_h[sl], rs_w[sl]};
            ref_scales[32 * sl +: 32] = {rs_ys[sl], rs_xs[sl]};
        end
    end
    logic [39:0]    dma_base [3];
    logic [31:0]    dma_stride [3];
    always_comb for (int p = 0; p < 3; p++) cur_planes[72 * p +: 72] = {dma_stride[p], dma_base[p]};   // the picture being decoded: intra block copy
    logic [15:0]    dma_w [3], dma_h [3];
    logic [31:0]    dma_beats;
    typedef enum logic [3:0] {D_IDLE, D_PLANE, D_ROW, D_BURST, D_AW, D_F0, D_F1, D_F2, D_W, D_B, D_NEXT} dstate_t;
    dstate_t        ds;
    logic [1:0]     d_plane;
    logic [15:0]    row_lines;                               // lines of plane d_plane in one released superblock row
    assign row_lines = (d_plane == 2'd0) ? ((SB128 != 0 && hdr.sb128) ? 16'd128 : 16'd64) : ((SB128 != 0 && hdr.sb128) ? 16'd64 : 16'd32);
    logic [FBY-1:0] d_y;
    logic [FBX-1:0] d_x;                                     // first sample of the beat being built
    logic [39:0]    d_row_addr, d_addr;                      // this row's first byte; the next beat's address
    logic [15:0]    d_row_beats;                             // beats still to write in this row
    logic [4:0]     d_len, d_cnt;                            // burst beats, beats done in it
    logic [47:0]    d_g0, d_g1;                              // the two sample groups of the beat
    logic [1:0]     n_planes;
    assign fmt16 = dma_cfg[0];
    assign n_planes = dma_cfg[5:4];
    logic [15:0]    d_y1;                                    // the run's last line + 1 in this plane
    logic [15:0]    row_bytes;                               // W * bytes per sample
    assign row_bytes = fmt16 ? {dma_w[d_plane][14:0], 1'b0} : dma_w[d_plane];
    logic [15:0]    rem_bytes;                               // bytes of the row not yet written
    assign rem_bytes = row_bytes - 16'(d_x) * (fmt16 ? 16'd2 : 16'd1);
    logic [9:0]     to4k;                                    // beats before the next 4 KB boundary (>= 1: 8-aligned)
    assign to4k = 10'((13'd4096 - 13'(d_addr[11:0])) >> 3);
    logic [4:0]     burst_len;
    always_comb begin
        burst_len = 5'd16;
        if (d_row_beats < 16'(burst_len)) burst_len = 5'(d_row_beats);
        if (to4k < 10'(burst_len)) burst_len = 5'(to4k);
    end
    logic [7:0]     beat_strb;
    always_comb beat_strb = (rem_bytes >= 16'd8) ? 8'hFF : 8'((9'd1 << rem_bytes[3:0]) - 9'd1);
    assign h_plane_m = dma_busy ? d_plane : haddr[29:28];
    assign h_y_m     = dma_busy ? d_y : haddr[14 +: FBY];
    assign h_x_m     = dma_busy ? ((ds == D_F1) ? d_x + FBX'(4) : d_x) : hx_q;
    assign m_axi_awaddr  = fs_active ? fs_addr : d_addr;
    assign m_axi_awlen   = fs_active ? 8'(fs_len - 5'd1) : 8'(d_len - 5'd1);
    assign m_axi_awsize  = 3'b011;                           // 8 bytes
    assign m_axi_awburst = 2'b01;                            // INCR
    assign m_axi_awcache = 4'b0011;                          // normal, non-cacheable, bufferable
    assign m_axi_awprot  = 3'b000;
    assign m_axi_awvalid = fs_active ? fs_awvalid : (ds == D_AW);
    assign m_axi_wdata   = fs_active ? (sv_busy ? sv_data[64 * fs_cnt[1:0] +: 64] : fs_rd_data)
                         : fmt16 ? {4'd0, d_g0[47:36], 4'd0, d_g0[35:24], 4'd0, d_g0[23:12], 4'd0, d_g0[11:0]}
                                 : {d_g1[43:36], d_g1[31:24], d_g1[19:12], d_g1[7:0], d_g0[43:36], d_g0[31:24], d_g0[19:12], d_g0[7:0]};
    assign m_axi_wstrb   = fs_active ? 8'hFF : beat_strb;
    assign m_axi_wlast   = fs_active ? fs_wlast : (d_cnt == d_len - 5'd1);
    assign m_axi_wvalid  = fs_active ? fs_wvalid : (ds == D_W);
    assign m_axi_bready  = fs_active ? fs_bready : (ds == D_B);
    // the read channels belong to ref_fetch (dec_top)

    always_ff @(posedge clk) begin
        flush_done <= 1'b0;
        if (!aresetn) begin
            ds <= D_IDLE; dma_busy <= 1'b0; dma_done <= 1'b0; dma_err <= 1'b0; dma_beats <= '0;
            d_plane <= '0; d_y <= '0; d_x <= '0; d_row_addr <= '0; d_addr <= '0; d_row_beats <= '0; d_len <= '0; d_cnt <= '0;
            d_g0 <= '0; d_g1 <= '0; d_y1 <= '0;
            fq_w <= '0; fq_r <= '0; fl_rows <= '0; fl_run <= 1'b0;
            for (int p = 0; p < 3; p++) begin fl_addr[p] <= '0; fl_y[p] <= '0; end
        end else begin
            // released rows into the queue; a frame start rewinds the cursors to the plane bases
            if (row_final && !fq_full) begin fq[fq_w[2:0]] <= row_final_idx; fq_w <= fq_w + 1'b1; end
            if (start[1]) begin
                for (int p = 0; p < 3; p++) begin fl_addr[p] <= dma_base[p]; fl_y[p] <= '0; end
                fl_rows <= '0; dma_err <= 1'b0;              // the error bit covers the whole frame's flushes (2026-10-30 audit)
            end
            case (ds)
                D_IDLE: if (fs_active || fs_go) ;              // the sideband engine owns the write channel
                else if (dma_start) begin
                    dma_busy <= 1'b1; dma_done <= 1'b0; dma_err <= 1'b0; dma_beats <= '0; d_plane <= 2'd0; fl_run <= 1'b0; ds <= D_PLANE;
                end else if (fl_auto && !fq_empty) begin     // a released superblock row: its lines of every plane
                    dma_busy <= 1'b1; d_plane <= 2'd0; fl_run <= 1'b1; fq_r <= fq_r + 1'b1; ds <= D_PLANE;   // dma_err stays: sticky over the frame's flushes
                end
                D_PLANE: begin                               // first line of plane d_plane in this run
                    d_row_addr <= fl_run ? fl_addr[d_plane] : dma_base[d_plane];
                    d_y <= fl_run ? FBY'(fl_y[d_plane]) : '0;
                    // a released superblock row is 64 luma / 32 chroma lines, 128 / 64 for an sb128 stream (SB128 builds; the
                    // 64-line flush of a 128-line row left half of every row unflushed and the sequencer waiting: 2026-10-27)
                    d_y1 <= fl_run ? ((fl_y[d_plane] + row_lines < dma_h[d_plane]) ? fl_y[d_plane] + row_lines : dma_h[d_plane])
                                   : dma_h[d_plane];
                    ds <= (fl_run && fl_y[d_plane] >= dma_h[d_plane]) ? D_NEXT : D_ROW;   // nothing left in this plane
                end
                D_ROW: begin                                 // a row: its beats and first address
                    d_addr <= d_row_addr; d_x <= '0; d_row_beats <= (row_bytes + 16'd7) >> 3; ds <= D_BURST;
                end
                D_BURST: begin
                    if (d_row_beats == 16'd0) ds <= D_NEXT;
                    else begin d_len <= burst_len; d_cnt <= 5'd0; ds <= D_AW; end
                end
                D_AW: if (m_axi_awready) ds <= D_F0;
                D_F0: ds <= fmt16 ? D_F2 : D_F1;             // h_x = d_x this cycle; its group is readable next cycle
                // the window memory serves the host port only when the filter's read port is idle (a flush runs
                // alongside decoding): a group the memory did not serve (h_ok low) is fetched again from D_F0
                D_F1: begin d_g0 <= h_rdata4; ds <= h_ok ? D_F2 : D_F0; end   // h_x = d_x + 4 this cycle (8-bit: two groups a beat)
                D_F2: begin
                    if (fmt16) d_g0 <= h_rdata4; else d_g1 <= h_rdata4;
                    ds <= h_ok ? D_W : D_F0;
                end
                D_W: if (m_axi_wready) begin
                    dma_beats <= dma_beats + 1'b1;
                    d_addr <= d_addr + 40'd8;
                    d_x <= d_x + (fmt16 ? FBX'(4) : FBX'(8));
                    d_row_beats <= d_row_beats - 1'b1;
                    d_cnt <= d_cnt + 1'b1;
                    ds <= m_axi_wlast ? D_B : D_F0;
                end
                D_B: if (m_axi_bvalid) begin
                    if (m_axi_bresp[1]) dma_err <= 1'b1;
                    ds <= D_BURST;
                end
                D_NEXT: begin                                // line done
                    if (16'(d_y) + 16'd1 >= d_y1 || (fl_run && fl_y[d_plane] >= dma_h[d_plane])) begin
                        if (fl_run) begin                    // the plane's cursor past this row
                            fl_addr[d_plane] <= d_row_addr + 40'(dma_stride[d_plane]);
                            fl_y[d_plane] <= d_y1;
                        end
                        if (d_plane == n_planes) begin
                            dma_busy <= 1'b0; ds <= D_IDLE;
                            if (fl_run) begin flush_done <= 1'b1; fl_rows <= fl_rows + 8'd1; end
                            else dma_done <= 1'b1;
                        end else begin d_plane <= d_plane + 1'b1; ds <= D_PLANE; end
                    end else begin
                        d_y <= d_y + 1'b1; d_row_addr <= d_row_addr + 40'(dma_stride[d_plane]); ds <= D_ROW;
                    end
                end
                default: ds <= D_IDLE;
            endcase
        end
    end

    // ---------------------------------------------------------------- frame-state sideband DMA
    // A finished superblock row (fs_row_ready from frame_state_w) is written as its 8x8 lines (MV words) then its 4x4
    // lines (segment bytes), each line at the area's running cursor (rows arrive in order; a frame start rewinds the
    // cursors to the bases). Lines of a partial last row are clipped to the frame. The engine takes the write channel
    // between picture runs (fs_active) and answers fs_done when the row's buffer may be refilled.
    typedef enum logic [3:0] {FS_IDLE, FS_SEL, FS_LINE, FS_LEN, FS_AW, FS_RD, FS_W, FS_B, FS_DONE,
                              SV_RB, SV_RB1, SV_AW, SV_W, SV_B, SV_DONE} fsstate_t;
    fsstate_t       fss;
    logic           fs_go, fs_pend, fs_sel;
    logic [SBW-1:0] fs_row;
    logic [39:0]    fs_mv_cur, fs_seg_cur;                   // the next line's address per area
    logic [15:0]    fs_lines, fs_line;                       // lines of this row in the current area, the current line
    logic [15:0]    fs_line_beats;                           // beats per line (rowb / 8)
    logic [15:0]    fs_beats;                                // beats left in this line
    logic [12:0]    fs_beat;                                 // beat index within the line (the writer's read port)
    logic [15:0]    fs_rows_total;                           // 8x8 or 4x4 lines of the frame in this area
    logic [15:0]    fs_row_lines;                            // lines per superblock row in this area
    logic [9:0]     fs_to4k;
    assign fs_to4k = 10'((13'd4096 - 13'(fs_addr[11:0])) >> 3);
    logic           sv_pend;
    assign fs_go = (fss == FS_IDLE) && (fs_pend || sv_pend) && (ds == D_IDLE) && (fs_cfg[0] || sv_pend);
    assign fs_rows_total = fs_sel ? 16'(hdr.mi_rows) : 16'(hdr.mi_rows >> 1);
    assign fs_row_lines  = fs_sel ? ((SB128 != 0 && hdr.sb128) ? 16'd32 : 16'd16) : ((SB128 != 0 && hdr.sb128) ? 16'd16 : 16'd8);
    assign fs_rd_sel = fs_sel;
    assign fs_rd_buf = fs_row[0];
    assign fs_rd_line = fs_line[4:0];
    assign fs_rd_beat = fs_beat[8:0];
    assign fs_awvalid = (fss == FS_AW) || (fss == SV_AW);
    assign fs_wvalid  = (fss == FS_W) || (fss == SV_W);
    assign fs_bready  = (fss == FS_B) || (fss == SV_B);
    assign fs_wlast   = (fs_cnt == fs_len - 5'd1);
    always_ff @(posedge clk) begin
        fs_done <= 1'b0;
        if (!aresetn) begin
            fss <= FS_IDLE; fs_pend <= 1'b0; fs_active <= 1'b0; fs_sel <= 1'b0; fs_row <= '0; fs_err <= 1'b0; fs_rows <= '0;
            fs_mv_cur <= '0; fs_seg_cur <= '0; fs_lines <= '0; fs_line <= '0; fs_line_beats <= '0; fs_beats <= '0; fs_beat <= '0;
            fs_addr <= '0; fs_len <= '0; fs_cnt <= '0;
            sv_pend <= 1'b0; sv_busy <= 1'b0; sv_done_s <= 1'b0; sv_err <= 1'b0; sv_rb_en <= 1'b0; sv_rb_l1 <= 1'b0; sv_row <= '0; sv_data <= '0; ld_done_s <= 1'b0;
        end else begin
            sv_rb_en <= 1'b0; sv_rb_l1 <= sv_rb_en;
            if (sv_rb_l1) sv_data <= {10'd0, rb_data};
            if (start[7]) begin sv_pend <= 1'b1; sv_done_s <= 1'b0; sv_err <= 1'b0; end
            if (start[6]) ld_done_s <= 1'b0; else if (ld_done) ld_done_s <= 1'b1;
            if (fs_row_ready) begin fs_pend <= 1'b1; fs_row <= fs_row_idx; end
            if (start[1]) begin fs_mv_cur <= fs_mv_base; fs_seg_cur <= fs_seg_base; fs_rows <= '0; fs_err <= 1'b0; end
            case (fss)
                FS_IDLE: if (fs_pend && !fs_cfg[0]) begin fs_pend <= 1'b0; fs_done <= 1'b1; end   // disabled: drop the row
                         else if (fs_go && sv_pend) begin       // the CDF save first: the host waits on it between tile rows
                             fs_active <= 1'b1; sv_busy <= 1'b1; sv_pend <= 1'b0; sv_row <= '0; fs_addr <= sv_cdf_base; fss <= SV_RB;
                         end
                         else if (fs_go) begin fs_active <= 1'b1; fs_pend <= 1'b0; fs_sel <= 1'b0; fss <= FS_SEL; end
                FS_SEL: begin                                // the lines of this row in area fs_sel (clipped to the frame)
                    fs_line <= '0;
                    fs_lines <= (16'(fs_row) * fs_row_lines + fs_row_lines <= fs_rows_total) ? fs_row_lines
                              : (16'(fs_row) * fs_row_lines < fs_rows_total) ? fs_rows_total - 16'(fs_row) * fs_row_lines : 16'd0;
                    fs_line_beats <= (fs_sel ? fs_seg_rowb : fs_mv_rowb) >> 3;
                    fss <= FS_LINE;
                end
                FS_LINE: begin
                    if (fs_line >= fs_lines || fs_line_beats == 16'd0) begin
                        if (fs_sel) fss <= FS_DONE; else begin fs_sel <= 1'b1; fss <= FS_SEL; end
                    end else begin
                        fs_addr <= fs_sel ? fs_seg_cur : fs_mv_cur; fs_beats <= fs_line_beats; fs_beat <= '0; fss <= FS_LEN;
                        if (fs_sel) fs_seg_cur <= fs_seg_cur + 40'(fs_seg_rowb); else fs_mv_cur <= fs_mv_cur + 40'(fs_mv_rowb);
                    end
                end
                FS_LEN: begin                                // the burst length before awvalid (awlen must be stable with it):
                    fs_len <= (fs_beats < 16'd16) ? ((fs_to4k < 10'(fs_beats)) ? 5'(fs_to4k) : 5'(fs_beats))   // up to 16 beats, not across 4 KB
                                                  : ((fs_to4k < 10'd16) ? 5'(fs_to4k) : 5'd16);
                    fs_cnt <= '0;
                    fss <= FS_AW;
                end
                FS_AW: if (m_axi_awready) fss <= FS_RD;
                FS_RD: fss <= FS_W;                          // the read port answers the beat's address a cycle later
                FS_W: if (m_axi_wready) begin
                    fs_addr <= fs_addr + 40'd8; fs_beat <= fs_beat + 13'd1; fs_beats <= fs_beats - 16'd1; fs_cnt <= fs_cnt + 5'd1;
                    fss <= fs_wlast ? FS_B : FS_RD;
                end
                FS_B: if (m_axi_bvalid) begin
                    if (m_axi_bresp[1]) fs_err <= 1'b1;
                    if (fs_beats == 16'd0) begin fs_line <= fs_line + 16'd1; fss <= FS_LINE; end
                    else fss <= FS_LEN;
                end
                FS_DONE: begin fs_active <= 1'b0; fs_done <= 1'b1; fs_rows <= fs_rows + 8'd1; fss <= FS_IDLE; end
                // the CDF save: one row (246 bits in 4 beats) at a time from engine sv_eng's store through the read-back port
                SV_RB:  begin sv_rb_en <= 1'b1; fss <= SV_RB1; end
                SV_RB1: if (sv_rb_l1) begin fs_len <= 5'd4; fs_cnt <= '0; fss <= SV_AW; end   // sv_data lands this cycle
                SV_AW:  if (m_axi_awready) fss <= SV_W;
                SV_W:   if (m_axi_wready) begin fs_addr <= fs_addr + 40'd8; fs_cnt <= fs_cnt + 5'd1; if (fs_wlast) fss <= SV_B; end
                SV_B:   if (m_axi_bvalid) begin
                    if (m_axi_bresp[1]) sv_err <= 1'b1;
                    if (sv_row == CDF_AW'(CDF_ROWS - 1)) fss <= SV_DONE; else begin sv_row <= sv_row + CDF_AW'(1); fss <= SV_RB; end
                end
                SV_DONE: begin fs_active <= 1'b0; sv_busy <= 1'b0; sv_done_s <= 1'b1; fss <= FS_IDLE; end
                default: fss <= FS_IDLE;
            endcase
        end
    end

    // ---------------------------------------------------------------- AXI4-Lite slave
    logic        aw_v, w_v, ar_v;
    logic [11:0] aw_a, ar_a;
    logic [31:0] w_d;
    logic [3:0]  w_s;
    logic        do_write, do_read;
    assign s_axi_awready = !aw_v;
    assign s_axi_wready  = !w_v;
    assign s_axi_arready = !ar_v && !s_axi_rvalid;
    assign s_axi_bresp   = 2'b00;
    assign s_axi_rresp   = 2'b00;
    assign do_write = aw_v && w_v && !s_axi_bvalid;
    logic [9:0] wi, ri;                                      // word index
    logic [2:0] mfs;                                         // the projection source a 0x740 + 0x20 s write addresses
    assign mfs = wi[5:3] - 3'd2;
    assign h_data_read = (ri == 10'h00A) || (ri == 10'h00C) || (ri == 10'h00D);
    assign do_read  = ar_v && !(h_data_read && !(hbuf_ok && !dma_busy)) && !(rb_en || rb_l1);
    assign wi = aw_a[11:2];
    assign ri = ar_a[11:2];
    function automatic logic [31:0] strb(input logic [31:0] old, input logic [31:0] nw, input logic [3:0] s);
        logic [31:0] r;
        for (int b = 0; b < 4; b++) r[8*b +: 8] = s[b] ? nw[8*b +: 8] : old[8*b +: 8];
        return r;
    endfunction
    logic [31:0] ctrl_nx;                                   // CTRL as this write leaves it (Vivado: no select on a function call)
    assign ctrl_nx = strb(ctrl, w_d, w_s);
    always_ff @(posedge clk) begin
        if (!aresetn) begin
            aw_v <= 1'b0; w_v <= 1'b0; ar_v <= 1'b0; s_axi_bvalid <= 1'b0; s_axi_rvalid <= 1'b0;
            ctrl <= 32'd0; eos_e <= '0; haddr <= 32'd0; def_addr_r <= '0; eng_mask_r <= '0; col_start_r <= '0; col_end_r <= '0;
            for (int i = 0; i < 8; i++) def_w[i] <= '0;
            for (int i = 0; i < 16; i++) begin hdr_w[i] <= '0; rh_w[i] <= '0; lh_w[i] <= '0; ch_w[i] <= '0; end
            for (int i = 0; i < 64; i++) ih_w[i] <= '0;
            start <= '0; push <= 1'b0; def_we <= 1'b0; f_badw <= 1'b0; h_fetch <= 1'b0; rb_en <= 1'b0; rb_l1 <= 1'b0; mf_we <= 1'b0; sp_we <= 1'b0;
            mf_addr_r <= '0; sp_addr_r <= '0; sp_d0 <= '0; sp_d1 <= '0; mf_wd <= '0;
            dma_cfg <= '0; dma_start <= 1'b0;
            fs_cfg <= '0; fs_mv_base <= '0; fs_seg_base <= '0; fs_mv_rowb <= '0; fs_seg_rowb <= '0;
            ld_cfg_r <= '0; ld_seg_base <= '0; ld_seg_rowb <= '0; ld_cdf_base <= '0; sv_cdf_base <= '0; sv_eng <= '0;
            mf_nsrc_r <= '0; mf_src_base <= '0; mf_src_rowb <= '0; mf_src_sign <= '0; mf_src_tocur <= '0; mf_src_off <= '0; mf_dst_dist <= '0;
            for (int p = 0; p < 3; p++) begin dma_base[p] <= '0; dma_stride[p] <= '0; dma_w[p] <= '0; dma_h[p] <= '0; end
            for (int sl = 0; sl < 8; sl++) begin rs_w[sl] <= '0; rs_h[sl] <= '0; rs_xs[sl] <= 16'd16384; rs_ys[sl] <= 16'd16384; for (int pl = 0; pl < 3; pl++) begin rs_base[sl][pl] <= '0; rs_stride[sl][pl] <= '0; end end
            ref_slot_r <= '0; ref_cfg_r <= '0;
        end else begin
            start <= '0; push <= 1'b0; def_we <= 1'b0; h_fetch <= 1'b0; dma_start <= 1'b0;
            rb_en <= 1'b0; rb_l1 <= rb_en; mf_we <= 1'b0; sp_we <= 1'b0;
            if (rb_l1) begin for (int i = 0; i < 7; i++) def_w[i] <= rb_data[32*i +: 32]; def_w[7] <= 32'(rb_data[245:224]); end
            if (ctrl[0]) f_badw <= 1'b0;
            if (s_axi_awvalid && s_axi_awready) begin aw_a <= s_axi_awaddr; aw_v <= 1'b1; end
            if (s_axi_wvalid && s_axi_wready) begin w_d <= s_axi_wdata; w_s <= s_axi_wstrb; w_v <= 1'b1; end
            if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
            if (do_write) begin
                aw_v <= 1'b0; w_v <= 1'b0; s_axi_bvalid <= 1'b1;
                case (wi)
                    10'h002: begin
                        ctrl <= strb(ctrl, w_d, w_s);
                        if (w_s[1] && w_d[8]) eos_e[(TE > 1) ? ctrl_nx[24 +: EW] : '0] <= 1'b1;
                    end
                    10'h003: start <= w_d[7:0];
                    // a push must be one whole transfer: a narrow or split write (a byte-wise memcpy on the PS,
                    // first board run) is dropped and flagged like an overflow
                    10'h007: begin if (w_s[0]) begin push <= 1'b1; push_d <= {2'd0, 24'd0, w_d[7:0]}; end else f_badw <= 1'b1; end
                    10'h008: begin if (w_s == 4'hF) begin push <= 1'b1; push_d <= {2'd3, w_d}; end else f_badw <= 1'b1; end
                    10'h009: begin haddr <= strb(haddr, w_d, w_s); h_fetch <= 1'b1; end
                    10'h00B: begin def_addr_r <= w_d[CDF_AW-1:0]; if (w_d[31]) rb_en <= 1'b1; else def_we <= 1'b1; end
                    10'h00E: eng_mask_r <= w_d[TE-1:0];
                    default: ;
                endcase
                if (TE > 1 && int'(wi) >= 24 && int'(wi) < 24 + TE) begin                   // 0x060 + 4e: engine e's columns
                    col_start_r[(int'(wi) - 24) * MIW +: MIW] <= w_d[MIW-1:0];
                    col_end_r[(int'(wi) - 24) * MIW +: MIW] <= w_d[16 +: MIW];
                end
                case (wi)
                    10'h060: dma_cfg <= strb(dma_cfg, w_d, w_s);                                              // 0x180
                    10'h070: fs_cfg <= strb(fs_cfg, w_d, w_s);                                                // 0x1C0
                    10'h071: fs_mv_base[31:0] <= strb(fs_mv_base[31:0], w_d, w_s);                            // 0x1C4
                    10'h072: fs_mv_base[39:32] <= w_d[7:0];                                                   // 0x1C8
                    10'h073: fs_mv_rowb <= w_d[15:0];                                                         // 0x1CC
                    10'h074: fs_seg_base[31:0] <= strb(fs_seg_base[31:0], w_d, w_s);                          // 0x1D0
                    10'h075: fs_seg_base[39:32] <= w_d[7:0];                                                  // 0x1D4
                    10'h076: fs_seg_rowb <= w_d[15:0];                                                        // 0x1D8
                    10'h1C0: ld_cfg_r <= strb(ld_cfg_r, w_d, w_s);                                            // 0x700
                    10'h1C1: ld_seg_base[31:0] <= strb(ld_seg_base[31:0], w_d, w_s);                          // 0x704
                    10'h1C2: ld_seg_base[39:32] <= w_d[7:0];                                                  // 0x708
                    10'h1C3: ld_seg_rowb <= w_d[15:0];                                                        // 0x70C
                    10'h1C4: ld_cdf_base[31:0] <= strb(ld_cdf_base[31:0], w_d, w_s);                          // 0x710
                    10'h1C5: ld_cdf_base[39:32] <= w_d[7:0];                                                  // 0x714
                    10'h1C6: sv_cdf_base[31:0] <= strb(sv_cdf_base[31:0], w_d, w_s);                          // 0x718
                    10'h1C7: sv_cdf_base[39:32] <= w_d[7:0];                                                  // 0x71C
                    10'h1C8: sv_eng <= EW'(w_d);                                                              // 0x720
                    10'h1C9: mf_nsrc_r <= strb(mf_nsrc_r, w_d, w_s);                                          // 0x724
                    10'h1CA: mf_dst_dist[31:0] <= strb(mf_dst_dist[31:0], w_d, w_s);                          // 0x728
                    10'h1CB: mf_dst_dist[55:32] <= w_d[23:0];                                                 // 0x72C
                    10'h061: dma_start <= !dma_busy;                                                           // 0x184
                    // 0x300 .. 0x310: the inter frame bands (motion field, previous segment ids); no-ops at INTER = 0
                    10'h0C0: if (INTER != 0) mf_addr_r <= strb(mf_addr_r, w_d, w_s);                          // 0x300
                    10'h0C1: if (INTER != 0) begin mf_wd <= w_d; mf_we <= 1'b1; end                           // 0x304
                    10'h0C2: if (INTER != 0) sp_addr_r <= strb(sp_addr_r, w_d, w_s);                          // 0x308
                    10'h0C3: if (INTER != 0) sp_d0 <= strb(sp_d0, w_d, w_s);                                  // 0x30C
                    10'h0C4: if (INTER != 0) begin sp_d1 <= w_d[15:0]; sp_we <= 1'b1; end                     // 0x310
                    10'h180: if (INTER != 0) ref_slot_r <= 21'(strb({11'd0, ref_slot_r}, w_d, w_s));             // 0x600 (no select on a function call: Vivado Synth 8-12513)
                    10'h181: if (INTER != 0) ref_cfg_r <= strb(ref_cfg_r, w_d, w_s);                          // 0x604
                    default: begin
                        if (wi[9:3] == 7'h02)        def_w[wi[2:0]] <= strb(def_w[wi[2:0]], w_d, w_s);       // 0x040
                        else if (wi[9:4] == 6'h02)   hdr_w[wi[3:0]] <= strb(hdr_w[wi[3:0]], w_d, w_s);       // 0x080
                        else if (wi[9:4] == 6'h03)   rh_w[wi[3:0]]  <= strb(rh_w[wi[3:0]], w_d, w_s);        // 0x0C0
                        else if (wi[9:4] == 6'h04)   lh_w[wi[3:0]]  <= strb(lh_w[wi[3:0]], w_d, w_s);        // 0x100
                        else if (wi[9:4] == 6'h05)   ch_w[wi[3:0]]  <= strb(ch_w[wi[3:0]], w_d, w_s);        // 0x140
                        else if (wi[9:6] == 4'h2) begin if (INTER != 0) ih_w[wi[5:0]] <= strb(ih_w[wi[5:0]], w_d, w_s); end   // 0x200: inter header window (no-op at INTER = 0)
                        else if (INTER != 0 && wi[9:7] == 3'b010) begin                                         // 0x400..0x5FC: the reference slot table
                            if (wi[3:2] != 2'd3) begin
                                case (wi[1:0])
                                    2'd0: rs_base[wi[6:4]][wi[3:2]][31:0]  <= strb(rs_base[wi[6:4]][wi[3:2]][31:0], w_d, w_s);
                                    2'd1: rs_base[wi[6:4]][wi[3:2]][39:32] <= w_d[7:0];
                                    2'd2: rs_stride[wi[6:4]][wi[3:2]]      <= strb(rs_stride[wi[6:4]][wi[3:2]], w_d, w_s);
                                    default: ;
                                endcase
                            end else if (wi[1:0] == 2'd0) begin rs_w[wi[6:4]] <= w_d[15:0]; rs_h[wi[6:4]] <= w_d[31:16]; end
                            else if (wi[1:0] == 2'd1) begin rs_xs[wi[6:4]] <= w_d[15:0]; rs_ys[wi[6:4]] <= w_d[31:16]; end
                        end
                        else if (wi[9:3] >= 7'h3A && wi[9:3] <= 7'h3E) begin                                   // 0x740..0x7DC: the projection sources
                            case (wi[2:0])
                                3'd0: mf_src_base[40 * mfs +: 32]       <= strb(mf_src_base[40 * mfs +: 32], w_d, w_s);
                                3'd1: mf_src_base[40 * mfs + 32 +: 8]   <= w_d[7:0];
                                3'd2: mf_src_rowb[16 * mfs +: 16]       <= w_d[15:0];
                                3'd3: begin mf_src_sign[mfs] <= w_d[8]; mf_src_tocur[8 * mfs +: 8] <= w_d[7:0]; end
                                3'd4: mf_src_off[56 * mfs +: 32]        <= strb(mf_src_off[56 * mfs +: 32], w_d, w_s);
                                3'd5: mf_src_off[56 * mfs + 32 +: 24]   <= w_d[23:0];
                                default: ;
                            endcase
                        end
                        else if (wi[9:4] == 6'h06 && wi[3:2] != 2'd0) begin                                     // 0x190..0x1BC: planes 0..2
                            case (wi[1:0])
                                2'd0: dma_base[wi[3:2] - 2'd1][31:0]  <= strb(dma_base[wi[3:2] - 2'd1][31:0], w_d, w_s);
                                2'd1: dma_base[wi[3:2] - 2'd1][39:32] <= w_d[7:0];
                                2'd2: dma_stride[wi[3:2] - 2'd1]      <= strb(dma_stride[wi[3:2] - 2'd1], w_d, w_s);
                                default: begin dma_w[wi[3:2] - 2'd1] <= w_d[15:0]; dma_h[wi[3:2] - 2'd1] <= w_d[31:16]; end
                            endcase
                        end
                    end
                endcase
            end
            // a tile start ends the previous tile's end-of-stream
            if (start[0]) begin ctrl[8] <= 1'b0; eos_e <= '0; end
            if (s_axi_arvalid && s_axi_arready) begin ar_a <= s_axi_araddr; ar_v <= 1'b1; end
            if (s_axi_rvalid && s_axi_rready) s_axi_rvalid <= 1'b0;
            if (do_read) begin
                ar_v <= 1'b0; s_axi_rvalid <= 1'b1;
                if (TE > 1 && int'(ri) >= 24 && int'(ri) < 24 + TE)                          // 0x060 + 4e: engine e's columns
                    s_axi_rdata <= {3'd0, col_end_r[(int'(ri) - 24) * MIW +: MIW], 3'd0, col_start_r[(int'(ri) - 24) * MIW +: MIW]};
                else if (int'(ri) >= 28 && int'(ri) < 28 + TE) begin                           // 0x070 + 4e: engine e's FIFO
                    for (int e = 0; e < TE; e++)                                                   // constant indices: Vivado's netlist
                        if (int'(ri) == 28 + e)                                                    // fed engine 0's count to every
                            s_axi_rdata <= {16'(f_free_e[e]), 14'd0, in_ready[e], f_empty_e[e]};     // engine's read through a computed
                end                                                                                // index (gate te2a..te2c 2026-10-29)
                else
                case (ri)
                    10'h000: s_axi_rdata <= 32'hA71D_0001;
                    10'h00E: s_axi_rdata <= 32'(eng_mask_r);
                    10'h001: s_axi_rdata <= {4'd0, 4'(TE), 8'(FBX), 8'(FBY), 4'(ITX_LANES), 1'd0, 1'b1, 1'(SB128), 1'(FRAME_URAM)};   // bit 2: EOS per engine
                    10'h002: s_axi_rdata <= ctrl;
                    10'h004: s_axi_rdata <= {16'(f_free), f_empty_sel, f_ovf || f_badw, in_ready_sel, lr_done_s, lr_busy, sr_done_s, sr_busy,
                                             cdef_done_s, cdef_busy, lf_done_s, lf_busy, frame_done_lvl, frame_busy, unsupported, tile_done_lvl, tile_busy};
                    10'h005: s_axi_rdata <= perf_syn_stall;
                    10'h006: s_axi_rdata <= perf_rec_idle;
                    10'h009: s_axi_rdata <= haddr;
                    10'h060: s_axi_rdata <= dma_cfg;
                    10'h062: s_axi_rdata <= {8'd0, fl_rows, 12'd0, (dma_busy && fl_run) || !fq_empty, dma_err, dma_done, dma_busy};
                    10'h063: s_axi_rdata <= dma_beats;
                    10'h070: s_axi_rdata <= fs_cfg;
                    10'h071: s_axi_rdata <= fs_mv_base[31:0];
                    10'h072: s_axi_rdata <= {24'd0, fs_mv_base[39:32]};
                    10'h073: s_axi_rdata <= {16'd0, fs_mv_rowb};
                    10'h074: s_axi_rdata <= fs_seg_base[31:0];
                    10'h075: s_axi_rdata <= {24'd0, fs_seg_base[39:32]};
                    10'h076: s_axi_rdata <= {16'd0, fs_seg_rowb};
                    10'h077: s_axi_rdata <= {8'd0, fs_rows, 7'd0, mem_err, sv_err, sv_done_s, sv_busy, ld_err, ld_done_s, ld_busy, fs_err, fs_active || fs_pend};
                    10'h1C0: s_axi_rdata <= ld_cfg_r;
                    10'h1C1: s_axi_rdata <= ld_seg_base[31:0];
                    10'h1C2: s_axi_rdata <= {24'd0, ld_seg_base[39:32]};
                    10'h1C3: s_axi_rdata <= {16'd0, ld_seg_rowb};
                    10'h1C4: s_axi_rdata <= ld_cdf_base[31:0];
                    10'h1C5: s_axi_rdata <= {24'd0, ld_cdf_base[39:32]};
                    10'h1C6: s_axi_rdata <= sv_cdf_base[31:0];
                    10'h1C7: s_axi_rdata <= {24'd0, sv_cdf_base[39:32]};
                    10'h1C8: s_axi_rdata <= 32'(sv_eng);
                    10'h1C9: s_axi_rdata <= mf_nsrc_r;
                    10'h1CA: s_axi_rdata <= mf_dst_dist[31:0];
                    10'h1CB: s_axi_rdata <= {8'd0, mf_dst_dist[55:32]};
                    10'h0C0: s_axi_rdata <= mf_addr_r;
                    10'h180: s_axi_rdata <= {11'd0, ref_slot_r};
                    10'h181: s_axi_rdata <= ref_cfg_r;
                    10'h0C2: s_axi_rdata <= sp_addr_r;
                    10'h0C3: s_axi_rdata <= sp_d0;
                    10'h00A: s_axi_rdata <= {20'd0, hbuf[0]};
                    10'h00C: begin s_axi_rdata <= {4'd0, hbuf[1], 4'd0, hbuf[0]}; haddr[FBX-1:0] <= haddr[FBX-1:0] + FBX'(2); h_fetch <= 1'b1; end
                    10'h00D: begin s_axi_rdata <= {hbuf[3][7:0], hbuf[2][7:0], hbuf[1][7:0], hbuf[0][7:0]}; haddr[FBX-1:0] <= haddr[FBX-1:0] + FBX'(4); h_fetch <= 1'b1; end
                    default: begin
                        if (ri[9:3] == 7'h02)      s_axi_rdata <= def_w[ri[2:0]];
                        else if (ri[9:4] == 6'h02) s_axi_rdata <= hdr_w[ri[3:0]];
                        else if (ri[9:4] == 6'h03) s_axi_rdata <= rh_w[ri[3:0]];
                        else if (ri[9:4] == 6'h04) s_axi_rdata <= lh_w[ri[3:0]];
                        else if (ri[9:4] == 6'h05) s_axi_rdata <= ch_w[ri[3:0]];
                        else if (ri[9:6] == 4'h2)  s_axi_rdata <= ih_w[ri[5:0]];
                        else if (ri[9:4] == 6'h06 && ri[3:2] != 2'd0)
                            case (ri[1:0])
                                2'd0: s_axi_rdata <= dma_base[ri[3:2] - 2'd1][31:0];
                                2'd1: s_axi_rdata <= {24'd0, dma_base[ri[3:2] - 2'd1][39:32]};
                                2'd2: s_axi_rdata <= dma_stride[ri[3:2] - 2'd1];
                                default: s_axi_rdata <= {dma_h[ri[3:2] - 2'd1], dma_w[ri[3:2] - 2'd1]};
                            endcase
                        else                       s_axi_rdata <= 32'd0;
                    end
                endcase
            end
        end
    end
endmodule
