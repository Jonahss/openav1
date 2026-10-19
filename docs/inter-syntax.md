# Inter frame syntax in RTL (stage 1 of inter prediction)

Status 2026-10-19: `tile_syntax` decodes the block syntax of inter frames (spec 5.11.7 inter_frame_mode_info and
5.11.23 .. 5.11.33) and emits block records with the inter fields; Argon profile-0 inter streams decode record- and
symbol-exactly against `tb/tile_model.py` (`tb/test_tile_syntax.py` with the frame sequence). Reconstruction of inter
blocks (motion compensation, references in DDR) is stage 2; `recon_top` keys its block copy on `intrabc` and
still predicts other inter blocks as intra, so pictures of inter frames are not yet meaningful.

## What a tile of an inter frame needs from the frames before it

| state | spec | where it lives | who fills it (stage 1) |
|---|---|---|---|
| starting CDFs: the primary reference's saved CDFs with zeroed counters, or the defaults | 7.20 load_cdfs | `cdf_store` def rows (DEF_ADDR / DEF_DATA, 1501 rows) | the host, from the model's sequence state (`seq_model`); the saved CDFs of a frame are read back after tile `context_update_tile_id` with DEF_ADDR bit 31 |
| PrevSegmentIds (temporal prediction of segment ids; the verbatim copy when the map is not updated) | 5.11.9, 7.4 | `seg_band`: 3 bits per 4x4, 16 per 48-bit word (SP_ADDR / SP_DATA0 / SP_DATA1 at 0x308) | the host, per frame |
| MotionFieldMvs[ref][y8][x8] (temporal MV candidates) | 7.9 | `mf_band`: one 33-bit entry {valid, row, col} per 8x8 and reference (MF_ADDR / MF_DATA at 0x300) | the host, from the model's 7.9 projection; a projection engine over the saved fields in DDR in stage 2 |
| the current frame's per-4x4 motion info (spatial candidates, warp samples, neighbour contexts) | 7.10.2 | `mv_mem` (`mv_ent_t`, 85 bits: references, two MVs, mode family bits, size, skip mode, compound indices, filters, segment prediction); a 32-row ring in window builds | every block writes its area when its record is complete |
| the inter frame header: skip mode frames, MV precision, filters, order hints, sign bias, scaled references, global motion, segment reference features, segmentation map flags | 5.9 | `ihdr_t` (64-word window at 0x200; `host_regs.ihdr_vals`) | the host, from the parsed header |

Rows of the two bands are indexed modulo their depth: the projection never leaves its 64-pixel row band and the temporal
scan never leaves the superblock, so a superblock row's band suffices on the board (`dec_top` sizes them); in
simulation `tile_syntax`'s defaults (and `FBX`/`FBY` through the runner) cover whole frames.

## The block chain (blk_syntax)

Intra frames decode as before. In an inter frame a block starts by reading the motion entries of its above and left
neighbours (the 8.3.2 contexts use only those two: references, intra-ness, single/compound, skip mode, compound
indices, filters, segment prediction), after the previous block's entry write has landed. Then, in symbol order:
inter_segment_id (the predicted id is the minimum of the previous map over the block, one 16-column word per cycle;
`seg_id_predicted` with the neighbours' contexts; otherwise read_segment_id), skip_mode, skip, the post-skip
segment id, cdef, delta q / lf, is_inter (or the segment's reference / globalmv feature), then either the intra chain
with `Y_Mode[Size_Group]` or: read_ref_frames as a 16-step decision tree (comp_mode, comp_ref_type, uni_comp_ref*,
comp_ref*, comp_bwdref*, single_ref_p1..p6 with the count contexts), find_mv_stack (`mv_stack`), the mode (skip mode
and segment features force it; compound_mode with Compound_Mode_Ctx_Map; new_mv / zero_mv / ref_mv), drl_mode (NEWMV
and NEW_NEWMV: indices 0..1; the NEAR modes: 1..2; only while NumMvFound > idx + 1), assign_mv per list (the predicted
MV from the stack, the global MV, or a read difference: mv_joint, sign, class, class0 bit / bits, `fr` unless
force_integer_mv, `hp` when allow_high_precision_mv), inter-intra (mode, wedge, index), motion mode
(has_overlappable_candidates scans the above row and left column; find_warp_samples scans the above row, left column,
top-left and top-right with the neighbours' sizes; NumSamples == 0 iff no scanned neighbour has RefFrame[0] alone; then
use_obmc or motion_mode), compound type (comp_group_idx, compound_idx with the order-hint distances, compound_type,
wedge index + sign, mask type), interp_filter (one or two directions, the neighbours' filters as context), then the
inter transform-size tree, inter transform types and the residual that intra block copy already used.

## The MV stack (mv_stack)

7.10.2 whole: the global motion vectors of the block's references (translation or the affine centre projection, 3
cycles per list), the spatial scans (row -1, col -1, top-right; the nearest bookkeeping; top-left; rows -3 / -5, cols
-3 / -5) with the spec's candidate rules (single: either of the neighbour's references matches, GLOBALMV neighbours of
8x8 and up substituted by the global vector; compound: both in order), temporal candidates from the band (positions
stepping 2 or 4 over the block and the three extension positions inside the 64x64), the two sorts, the extra search
(sign-bias-flipped neighbours; the compound combinations of same- and different-reference neighbours), the contexts
(NewMv, RefMv, ZeroMv, DrlCtxStack) and the clamping. `tb/test_mv_stack.py` fuzzes it against the model on synthetic
frames decoded in order (random partitions, random motion fields with holes, 128x128 superblocks, sub-tiles).

## Verification

- `tb/test_tile_syntax.py` with `TS_IVF=<Argon stream>`: `seq_model.SequenceState` runs the frame sequence (load_cdfs,
  the segmentation map, motion field estimation, the 7.19 / 7.20 storage) and loads the frame-side state into the
  DUT before each frame; every block record (all inter fields) and every symbol are compared with the model.
- Nightly: `tile_syntax_inter` over three Argon inter streams; `tools/nightly.sh`.
- Gate level: `tile_syntax` recordings of Argon inter runs replayed against the Vivado netlist (atium `~/inter_gate.sh`).

## Not in stage 1

Motion compensation, reference frames in DDR, the saved motion field of the current frame (7.19) going out with the
picture, the per-superblock-row projection engine (7.9) and the band refills, CDF save / load through DDR, large-scale
tile, film grain (not planned). See the design note (projects/open-av1-decoder-inter.md on the Pi) for the stages.
