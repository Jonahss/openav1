# Frame state on the card (the frame-to-frame state machine)

2026-10-30. Jonah, on seeing that an inter stream on the U200 runs at the Python model's speed: "an inter frame decodes
on the card but python on the cpu is passing the outputs of one frame to the next and holding state. let's clean that up."
This note is the census of that state, the design that moves it onto the card, and the staged plan.

## 1. What the host hands the card per frame today (tb/host_regs.py decode_frames_inter)

| State (spec) | Produced by | Handed over as | Size at 1080p |
|---|---|---|---|
| CDF tables the frame starts with: load_cdfs(ref_frame_idx[primary_ref_frame]) with the counts zeroed (8.2) | the model's SequenceState: the CDFs saved by the frame that refreshed that slot (the tile context_update_tile_id's final CDFs, or the frame's initial ones if disable_frame_end_update_cdf) | DEF_ADDR/DEF_DATA, 1,269 rows x 246 bits, once per tile row | 40 KB |
| PrevSegmentIds: load_previous_segment_ids (7.20) | the model: the slot's saved map (per 4x4; verbatim copy of the previous map when segmentation_update_map = 0) | SP_ADDR/SP_DATA0/1 band writes, 16 ids per 48-bit word | 480 x 270 x 3 bits = 47 KB |
| MotionFieldMvs[ref][y8][x8]: motion_field_estimation (7.9) | the model: projection of up to 5 source slots' saved MVs (7.19: one {ref, mv} per 8x8 from the odd 4x4, first list with a negative relative distance and |mv| <= 4095) | MF_ADDR/MF_DATA entry writes | 7 refs x 240 x 135 entries |
| Reference pictures and their sizes / scales | the card's own flushed pictures in DDR (the model only for frames the image cannot take) | slot table 0x400.., REF_SLOT | addresses |
| OrderHints, refresh flags, segmentation flags | the frame headers (obu_parser) | header registers (ihdr_t) | - |

Only the first three need the model, and only because the card keeps no record of a finished frame beyond its pixels:
mv_mem and mi_store are superblock-row bands that the next rows overwrite, and the adapted CDFs stay inside the engine
that produced them. The pixels are the card's; the memory of the frame is the host's.

## 2. Design: the frame's state lives in DDR next to its picture, written and read by the card

A reference slot in DDR becomes {picture planes, MV sideband, segment-id sideband, CDF rows}. The host keeps only the
slot table (addresses, sizes, order hints), which it derives from the frame headers; nothing on the host decodes.

**Writers (frame end, per superblock row, as the syntax stage finishes it):**
- *MV sideband*: one 32-bit word per 8x8 {ref[2:0] (0 = none), mv_row[12:0], mv_col[12:0]} = the spec's
  MfRefFrames/MfMvs of 7.19, decided at write time from the block's (ref0, mv0) / (ref1, mv1): the first list whose
  reference has a negative relative distance (a 7-bit per-frame mask from the host, `ref_dist_neg`, in ihdr_t) and
  |mv| <= REFMVS_LIMIT. Captured from the mv_mem area fill at the odd 4x4 (2*y8+1, 2*x8+1) into a double-buffered
  superblock-row buffer (1080p: 8 x 240 words = 7.5 KB per row), DMA'd to the slot's MV area when the row is done.
- *Segment-id sideband*: 16 ids per 48-bit word (the seg_band word), from the block area fill; when
  segmentation_enabled && !segmentation_update_map the source is the previous map (seg_band) instead (test10147's rule).
- *CDF rows*: at tile_done of the frame, engine context_update_tile_id's cdf_store rows (1,269 x 32 B) to the slot's CDF
  area; with disable_frame_end_update_cdf the engine's def_mem (the frame's starting rows) instead.
All three go through the existing AXI write master in dec_host (the flush engine gains two requesters besides released
picture rows: "sideband row ready" and "CDF save").

**Loaders (frame start, driven by the sequencer like ref_fetch):**
- *CDF loader*: the primary slot's CDF area -> def_we rows into every engine, the count field of each row zeroed.
- *Segment-map loader*: the primary slot's seg area -> seg_band, one superblock row ahead of the syntax stage.
- *Motion-field projection engine* (7.9): for each source slot (LAST, BWDREF, ALTREF2, ALTREF, LAST2 by the spec's
  order and refStamp rule), read its MV sideband for the current 64-pixel row band (projection never leaves the band:
  MAX_OFFSET_HEIGHT = 0; columns move at most 8 x 8 px), project each entry with Div_Mult[refOffset] (a 32-entry ROM),
  clip, and write mf_band[dst][y8][x8] for the 7 destinations. 1080p: 1,920 entries x 5 sources x 7 destinations
  = 67 k operations per superblock row, one per cycle, against a ~370 k-cycle row decode: it runs a row ahead.
All reads go through the AXI read channels ref_fetch owns today (a second requester on its arbiter).

**Host after this:** parse headers, keep the slot table (a pool allocation with the refresh/show_existing rules of 7.20
and 7.21 as address bookkeeping), write ihdr/hdr registers, push bytes, collect pictures. tb/seq_model.py stays as the
model of what the card now does, and tb/test_dec_host_inter.py compares the card's sideband and CDF areas with it.

## 3. Stages and estimates (from the ring memory + flush and ref_fetch, each ~1 day of my own work)

- **Stage A (~2 days): the card writes, the host moves.** Sideband writers + CDF save/load through DDR; the host reads
  the sideband back over PCIe, runs the projection (seq_model.motion_field_estimation on the card's saved MVs, no tile
  decoding) and writes mf_band/seg_band as today. The model leaves the loop; inter streams run at card speed minus one
  PCIe round trip per frame. Verified by the twin (sideband == the model's saved arrays, CDF area == the model's saved
  CDFs), then gate replay, then the card image.
- **Stage B (~2.5 days): the card loads.** Segment-map loader, CDF loader, the projection engine; the host writes no
  bands. Same ladder.
- Stage A is realistic before the symposium (2026-11-04); Stage B is more likely the week after. The deck says which.
