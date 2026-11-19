# The motion field as a band (design, 2026-11-18)

Status: **design, no RTL yet.** Ticket K60-54. It follows from the memory count of 2026-11-18 (K60-50 / K60-40).

## The count

Out-of-context synthesis of the card's decoder for the U200 (two tile engines, sb128, fb0 window 5 rows, fb2 3 rows),
at the 1080p frame-buffer size (FBX = FBY = 11) and at a 4K one (FBX = FBY = 12), every memory read from
`report_ram_utilization` (`tools/mem_by_module.py` for the per-module table):

| | 1080p size | 4K size | the card |
|---|---|---|---|
| LUTs | 541,019 | 667,499 | 1,182,240 |
| block RAM (RAMB36) | 1,713 | 4,889 | 2,160 |
| UltraRAM | 356 | 709 | 960 |

| memory | 1080p size | 4K size |
|---|---|---|
| `mf_band` `g_ref[].mem` (tile engines, block RAM) | 840 | 3,360 |
| `fc_rstore` (the chain's row store, block RAM) | 384 | 768 |
| `seg_band` (tile engines, block RAM) | 42 | 172 |
| `fc_plane` `u_bs` (block RAM) | 60 | 132 |
| `lr_stream` `cy_mem` (block RAM) | 54 | 108 |
| fb0 window (UltraRAM) | 120 | 240 |
| fb2 window (UltraRAM) | 72 | 144 |
| `mi_store` x 3 (UltraRAM) | 128 | 259 |

FBY = 12 is 4096 lines; a 2160-line picture needs about half of the tables sized by the frame height.

One table is 49 % of the block RAM the image uses today, and it is what stops both a 4K frame and four tile engines.

## Why it is that big

`mf_band` holds MotionFieldMvs[ref][y8][x8] (spec 7.9): 33 bits (valid, row, col) for each of 7 references at every
8x8 luma position. `dec_top` sizes it `MFL2R = FBY - 3`, `MFL2C = FBX - 3` in inter builds: the whole frame,
2^8 x 2^8 positions x 231 bits = 15.1 Mbit = 420 RAMB36, and `tile_engines` builds one per engine (840 with two).

The module's header describes a band of rows, and the spec allows one:

- **Writes.** `block_position` (7.9.3) accepts a projected position only if its row stays inside the 8-row band of
  the source row: MAX_OFFSET_HEIGHT is 0 (`tb/seq_model.py` `block_position`: `v8 < base8 - maxOff8 or v8 >= base8 + 8
  + maxOff8` with `maxOff8 = 0`). So band k of the field (rows 8k .. 8k+7) is a function of rows 8k .. 8k+7 of each
  source's saved motion vectors and of nothing else.
- **Reads.** The temporal candidates of the MV stack (7.10.2.5) read positions inside the current superblock.

It is the whole frame today because `mf_project` fills the entire field at frame start, before the first tile.

## The design

Project band by band, just ahead of the tile engines.

- `mf_band`: `MFL2R = 5` (32 rows = four 64-pixel bands = two 128-pixel superblock rows), rows addressed modulo 32 as
  now. Per engine at the 1080p size 2^5 x 2^8 x 231 bits = 1.9 Mbit = 53 RAMB36 (from 420); at the 4K size 105.
- `mf_project`: after `mf_start` it works one band at a time, in ascending order: clear the band's rows, then for each
  source read that band's 8 lines (the saved MV words are stored line by line at `src_rowb` bytes a line, so each
  source keeps a running line address) and project them as today. `band_done` after each; `mf_done` after the last.
- A band may be projected when its slot is free: band k overwrites band k - 4, so every tile engine must have left
  the superblock row that reads band k - 4.
- A tile engine may start a superblock row when the band (sb128: the two bands) of that row is projected. This is a
  hold on the syntax stage at the superblock boundary, per engine.
- No deadlock: an engine waits only for bands at or below the most advanced engine's, and those wait only for the
  slowest engine to leave a row four bands behind, which needs nothing from the projection.
- The host no longer waits for the projection before the tiles (the loader's done covers the segment map and the CDF
  rows only); the hold does the waiting, band by band.

The projection also leaves the front of the frame: today a 4K frame would spend 130k positions a source there.

## Order of work

1. `mf_project` band mode with its unit test against `tb/seq_model.py` (bands in order give the same field as the
   whole-frame pass; each band touches only its rows).
2. The two holds in `dec_top` / `tile_engines` / `tile_syntax`, the sequencing of bands, `MFL2R = 5`.
3. The inter twins and the nightly; out-of-context synthesis read (the table gone from the top of the list).
4. After it: `fc_rstore` into UltraRAM (384 RAMB36 today, about 48 UltraRAM), then `seg_band` the same way as the
   motion field if the count still asks for it.

## What it costs (found reading the code, 2026-11-18 evening)

Two things the band needs that the whole-frame field does not:

1. **The DDR read channel during the decode.** `dec_top` gives the one AXI read master to the loader group
   (`fs_load`, then `mf_project`) before the tiles and to `ref_fetch` after (`rf_arready = !ld_busy && ...`). Band by
   band, the projection reads while reference fetch is reading. Needed: a read arbiter at burst granularity with an
   owner queue (responses come back in order on one ID), about the size of the descriptor queue `mf_project` already
   has. The projection reads little: 8 lines of each source a band (1080p: 7.7 kB a source).
2. **The host-written field has to go.** Stage A of `docs/frame-state.md` still exists: the host (or the test) writes
   every entry of the field before the tiles, through `R_MF_ADDR` / `R_MF_DATA` in `dec_host` and the `mf_we` ports in
   the `dec_top` tests. A 32-row band cannot take a whole field. Using it today: `test_dec_inter` (all `dec_inter*`
   suites), `test_dec_host_inter` without `TS_FSTATE_B` (`dec_host_inter`, `dec_host_inter_argon`, gate shards b, c,
   d), `decode.py --inter` without `--card-load`. Using stage B (the card loads and projects): `dec_host_cardload`,
   `dec_host_census15`, gate shards a and e, and the card itself (`dec.sh`, the scoreboard).
   So: the twins and `test_dec_inter` move to stage B (the saved MV words of the model go into the AXI memory, the
   RTL projects), the known limit of the stage-B twin is lifted (a frame inheriting CDFs after a frame that disabled
   the frame-end update), and the host-written path is deleted.

The smaller alternative, for the record: one whole-frame copy shared by the engines, in UltraRAM. 1080p: 64 UltraRAM
instead of 840 block RAMs, no protocol change. It grows with the frame (4K: 136 to 256 UltraRAM, which leaves the
card at about 940 of 960 with the row store moved too) and needs a second copy from the third engine on. It would be
replaced by the band later.

Order, revised: (1) every test on stage B and the host-written field deleted: needed for the band, and a retirement
that stands on its own; (2) the read arbiter; (3) band projection, the holds, `MFL2R = 5`.

## Notes from the first night (2026-11-18)

- Census: `dec_host_inter` and `dec_host_inter_argon` pass unchanged with the card-side load; they and gate shards
  b, c, d now run that way. What still needed the host-written state was frames only the model decodes (a single
  frame picked with `TS_FRAMES`, frames outside the window limits): their state now goes into the AXI memory in the
  area formats the card writes (`host_regs.fs_pack_mv` / `fs_pack_seg` / `fs_pack_cdf`), so a later frame on the RTL
  loads it like any other.
- The tests that drive `dec_top` directly have no loader in their build (it exists in window builds with INTER); they
  keep the whole-frame field and write it through the `mf_we` ports. The band is the window build: `MFL2R = 5` there.
- `mf_project` works band by band since 440790e, with every band let go at once for now: the card-load twin (9 frames
  with a field, 14 projected sources) gives identical pictures.
- The hold point of a tile engine is the superblock start state of `tile_syntax` (`T_SB`); the band sequencing
  (which band may be projected, which engine waits) belongs in `tile_engines`, which sees every engine row.

