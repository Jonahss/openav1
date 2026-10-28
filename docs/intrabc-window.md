# Intra block copy on the window build

_Design note, 2026-10-28. Status: planned; the next coverage item after the two tile engines landed in main (fc12bee)._

## What it is

Intra block copy (IntraBC, spec 7.11.3 with `refIdx = -1`) predicts a block by copying an already decoded area of
the **same picture**: the block carries a block vector, the predictor is the current frame's pre-filter samples at
that offset. Luma vectors are whole samples; chroma of a subsampled plane may land on a half sample and takes the
BILINEAR filter (taps 64/64) through the single-prediction rounding chain (InterRound0 3 / InterRound1 11, 5 / 9 at
12-bit). The frame has `allow_intrabc`, which also switches every in-loop filter off (deblocking, CDEF, loop
restoration, super-res cannot combine with it), so the pre-filter picture **is** the final picture.

The spec's validity rules (the encoder's `is_dv_valid`; the decoder trusts a conformant stream) are what make it
pipeline-friendly: the source lies inside the current tile, is fully decoded, and is at least `INTRABC_DELAY_PIXELS`
(256) behind the current superblock in raster order, with a wavefront allowance of one superblock further right per
superblock row above (`gradient = 1 + 4 + sb128`). So in the current superblock row the source is 4 superblocks or
more to the left; in the row above it may reach one superblock to the right of the current column; rows further up
are open all the way across the tile.

Where it is used: screen content (text, UI, code, spreadsheets, remote desktop, screen sharing over WebRTC, game
HUDs, presentation video). Encoders enable it with the screen-content tools; Argon's stress streams use it freely
(test47 frames 3 and 8 are intrabc frames; the corpus census is `~/measure1080/intrabc_census.txt` on atium).

## What the decoder does today

- The syntax stage decodes intrabc blocks completely (`blk_syntax`: `use_ibc`, the MV stack and `read_mv` with
  `MV_INTRABC_CONTEXT`, the block record carries `intrabc` and the vector).
- The whole-picture build reconstructs them in `recon_top`'s block FSM (`B_IBC_*`): per output sample it reads the
  2x2 neighbourhood from fb0 one sample at a time (`B_IBC_R0..R4`, up to four `fb_re` reads), applies the bilinear
  pair and writes through the one-sample port (`fb2_we`). About 5-6 cycles a sample.
- The window build refuses the frame (`dec_top` `win_unsup` on `hdr.allow_intrabc`; `HostDecoder.window_ok` keeps it
  model-only): fb0 is a ring of `FRAME_WIN` superblock rows, a read at an arbitrary `y` wraps into the wrong row.

## The design

Route intrabc blocks through the **inter prediction path** with the current picture as the reference, and give that
path two sources for the patch: the ring for rows still in it, DDR for rows already flushed.

1. **Prediction = single-list translational MC, filter BILINEAR, no scaling, whole plane block as one unit.**
   `B_IBC_INIT` hands the block to the `B_MC_*` flow with an `ibc` flag: list 0 only, `interp = BILINEAR`,
   `mv = bv`, `motion_mode = SIMPLE`, no OBMC / masks / warps / compound (none of them can apply: the block's
   reference is `INTRA_FRAME`). Luma: integer vector, the filter is a copy. Chroma: the half-sample case is exactly
   mc_filt's bilinear row. The rounding is the single-prediction chain mc_filt already implements. Output through
   the 4-wide `fb4` port as every inter block, then the transform flow adds residuals (unchanged). The sample-at-a-
   time `B_IBC_R*` states go away; the whole-picture build takes the same path (one implementation to verify).
2. **Where a source row lives.** The window never reuses a ring slot before its row is flushed (`hold_rc`), so at
   any time a superblock row is in the ring, or in DDR, or both; `flushed_cnt` says which rows are complete in
   DDR, `rel_upto` / the ring occupancy says which are still in fb0. The wavefront rule means the referenced
   samples are already reconstructed (reconstruction is one raster pipeline over all engines: the row above is
   complete before this row starts, and the same row is read at least 256 px behind). No stall is ever needed for
   the ring source. A DDR row is readable once `flush_done` has counted it.
3. **Two loaders into one patch.** `mc_filt` consumes a patch row by row (`patch_we` / `patch_row` / `patch_w4` /
   `patch_d4`), so two loaders can fill one patch:
   - `ref_fetch` (unchanged) for the rows below `flushed_cnt`: request base / stride / fmt16 / lastx / lasty of the
     **current picture's output buffer**, i.e. the DMA plane registers dec_host already holds for the flush
     (`dma_base[p]`, `dma_stride[p]`, `dma_cfg.fmt16`); these go to `recon_top` as a ninth "slot".
   - a new `fb_fetch` (the ring loader) for the rows at or above `flushed_cnt`: reads fb0 through the 4-wide read
     port (`fb_rdata4`), one word a cycle, clamps like ref_fetch, writes the same patch interface. A 64x64 patch
     (72 rows x 18 words) takes ~1.3k cycles: 0.3 cycles a sample against 5-6 today.
   A strip whose rows straddle the boundary is split at the boundary: rows `[0, k)` to ref_fetch, `[k, h)` to
   fb_fetch; both run, `done` when both are done. Strips are at most 64 x 64 (sb128 frames: the spec caps intrabc
   blocks at 64x64... to be checked against `is_dv_valid`; if 128 is allowed, two strips as for inter).
4. **dec_top / dec_host.** `win_unsup` drops the `allow_intrabc` term. The current picture's plane bases / strides /
   format are new ports from dec_host into dec_top and recon_top (they exist as registers). `rel_upto` and the
   row-in-ring test are already in the window's bookkeeping; the ring loader needs the ring's row mapping, which
   is `frame_mem_uram`'s address function (row index truncated: the same function the flush uses).
5. **No new flush granularity.** The picture-memory note (step C) suggested per-superblock flushes for intrabc;
   with the two-source patch that is unnecessary: the row above is read from the ring, never from DDR, and the
   rows far above are flushed long before (the ring is 5 rows deep, the flush lags by about two). A per-superblock
   flush remains a throughput item, not a correctness one.

## Verification

- Twin: `test_dec_host_inter` (window config) lifts the intrabc term from its limits; the model already decodes
  intrabc. Streams: the census (`~/measure1080/intrabc_census.txt`), starting with test47 frames 3 / 8 (16-52 px
  wide, one column) and the widest intrabc frames with two tile columns on TE=2.
- Whole-picture build: the existing intrabc seeds / Argon frames must stay identical through the new path (the old
  `B_IBC_R*` path is the control until the switch-over commit).
- Gate replay: one intrabc frame recorded at FB9 WIN 5/3/3, replayed against the OOC post-synth netlist (the
  routed-cell replay of `u200_gate.sh` never elaborates: `opt_design` redraws the cell boundary).
- Card: an intrabc Argon batch on the next U200 image; the sweep's `model_only` count drops by the intrabc frames.

## Cost

RTL: `fb_fetch` (a ref_fetch sibling without AXI, ~150 lines), the `ibc` flag through the MC flow, the split, the
three new ports, the `win_unsup` change; `recon_top` loses the `B_IBC_R*` states. Twin and driver limits. One to two
days including the batch; the card image after the two-engine gate verdict.
