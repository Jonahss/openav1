# The segment map as a band (design, 2026-11-19)

Status: **RTL on branch seg-band, in test (2026-11-20).** Tickets under K60-39. It follows `docs/motion-field-band.md`: the segment map is the
last whole-frame table in the tile engines.

## The count

Out-of-context synthesis of the card's decoder at the 4K frame-buffer size (FBX = FBY = 12), mf-band `df28c6a`,
read from `util.rpt` and `tools/mem_by_module.py`:

| | two engines | four engines | the card |
|---|---|---|---|
| block RAM tiles | 1,907.5 | 2,177.5 | 2,160 |
| of which `u_ts` (RAMB36) | 470 | 876 | |
| UltraRAM | 709 | 773 | 960 |
| LUTs | 583,242 | 768,412 | 1,182,240 |

Four engines do not fit. `seg_band` is `2^SPL2R x 2^SPL2W` words of 48 bits; inter window builds size it for the
whole frame (`SPL2R = FBY - 2`, `SPL2W = FBX - 6`): at the 4K size 1024 x 64 x 48 bits = 3.1 Mbit = 86 RAMB36 an
engine (the count of 2026-11-18 read 172 for two).

As a band of 64 rows of 4x4 units (four 64-pixel bands, as the motion field keeps): 64 x 64 x 48 bits = 197 kbit =
6 RAMB36 an engine. Four engines by arithmetic: 2,177.5 - 4 x 80 = about 1,858. A sum, to be replaced by a synthesis
reading (step 3).

## Why a band is allowed

- **It is read-only during the decode.** `seg_band` holds PrevSegmentIds, the previous frame's map. `fs_load` writes it
  before the first tile (from the primary slot's segment-id area in DDR, or zeros); the current frame's ids never go
  into it: `frame_state_w` writes them to this frame's own area in DDR.
- **Reads stay inside the block.** `blk_syntax` (get_segment_id, states `S_SP_*`) reads the words covering the
  block's own rows and columns and takes the minimum. A block lies inside its superblock, so a superblock row reads
  only its own band (128-pixel superblocks: its two bands).

So band k of the map (rows 16k .. 16k+15) is needed only while an engine is in superblock row k, exactly the
motion field's situation, and the same hold serves both.

## The design

Load band by band, just ahead of the tile engines, in step with the motion-field projection.

- `seg_band`: `SPL2R = 6` in inter window builds, rows addressed modulo 64 as the module already does.
- `fs_load`: the CDF rows first and alone, as today (the host waits for them: `ld_done`). The segment map (or the zero
  map) then goes band by band: `band_go` in, `bands_done` and `bands_all` out, the pattern of `mf_project`. Each band is
  16 lines of the area.
- Order inside a band: the segment map's lines, then the projection's (the projection already waits for the loader
  with `src_go`). A band counts as done for the engines when every table that is switched on for the frame has it.
  The engines' hold (`tile_engines`, at `T_SB`) and the "less than four bands above the lowest busy engine" rule do
  not change; they take the smaller of the two counts and the AND of the two `bands_all`.
- Read channel: today `fs_load` has the channel to itself before the tiles and the projection shares it with
  reference fetch through `axi_rd_share` (until 2026-11-22: reference fetch then got a 512-bit read channel of its
  own and the share module went; loader and projection alone are on the 64-bit channel). The loader's band reads now fall in the decode too, so loader and projection
  (which never read at the same time) go into the arbiter's one port together.
- Frames with neither table (no segmentation map to load and no projection): nothing is switched on, nothing waits.

## What can go wrong, and the tests that would see it

- A band loaded late or overwritten early gives a wrong segment id, hence wrong quantiser / loop-filter level: a
  picture difference in any stream with temporal segment-id prediction. The band clips of 2026-11-19 (`mf_tall*`)
  have no segmentation; a tall clip WITH segmentation and `segmentation_temporal_update` is needed (aomenc
  `--aq-mode`, checked in the headers before it is trusted), plus the Argon streams that use it (census first).
- The hold is only tested when the loads fall behind the engines: `TS_MF_SLOW` already slows the reads of the
  segment-id area as well. The mutant runs repeat: no hold must fail, hold must pass.
- BUG-11's lesson: the number of bands is counted in one place per table (`bands_all`), never recomputed in
  `tile_engines`; the twin starts the load through `HR.load_seq`.
- The zero map (`ld_cfg[2]`) takes the same band path, with no reads.

## Steps

1. `fs_load` band by band, every band let go at once: pictures unchanged, cycle counts read.
2. The read channel: loader and projection through one arbiter port.
3. The hold on both counts, `SPL2R = 6`; the segmentation tall clip and its mutants; synthesis at the 1080p size and
   at the 4K size with four engines (the reading that replaces "about 1,858").

## The reading that replaces "about 1,858" (measured 2026-11-20)

Out-of-context synthesis of `dec_host` with the band (`seg-band` 83c84a9, U200 part), `util.rpt`, `util_hier.rpt` and
`tools/ram_by_table.py`:

| | 1080p size, two engines (`k4count-seg11`) | 4K size, four engines (`k4count-seg12te4`) | before, 4K four engines |
|---|---|---|---|
| block RAM tiles | 1,004.5 (46.5 %) | **1,916 (88.7 %)** | 2,177.5 (100.8 %) |
| of which the tile engines | | 532 | 876 |
| UltraRAM | 356 | 777 (80.9 %) | 773 |
| LUTs | | 737,912 (62.4 %) | 768,412 |

- One engine at the 4K size: 133 block RAMs (the motion-field bands 105, the CDF stores 28) and 33 UltraRAMs. Vivado
  put the band (4096 x 48) into one UltraRAM, so the segment map went from 86 block RAMs to none.
- The engines dropped by 344 block RAMs, as counted (4 x 86). The total dropped by 261.5, not 320: the filter chain
  `u_fc` reads 397.5 tiles here against 321 in the earlier run and 28 k fewer LUTs. Vivado moved some of its
  memories from LUT RAM into block RAM; the same module read 423.5 (386 + 75 half tiles) in the two-engine run of the same earlier commit.
  The total is what the card has to hold.
- With the shell (about 104 block RAMs, the 2026-11-19 figure) the four-engine decoder stands near 2,020 of 2,160:
  **it fits, at about 93.5 %.** Moving the motion-field bands to UltraRAM (105 an engine) is what would make it
  comfortable.
- The same six `Synth 8-4767` notes as before the change (`mi_store`, `rec_fifo`, `fb_fetch`: small register arrays);
  none new.
