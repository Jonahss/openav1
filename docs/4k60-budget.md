# 4K60 budget: where each stage stands (2026-11-13)

The goal is 3840x2160 at 60 fps. This note measures the distance stage by stage, so that work goes to what 4K60
needs and not to the 1080p scoreboard for its own sake. Every figure is marked **measured** (read from a log named
here), **arithmetic** (follows from measured figures and the format), or **not measured**.

## The requirement (arithmetic)

- 4K60 4:2:0 is 3840 x 2160 x 1.5 x 60 = **746.5 M samples a second**.
- The card today (checkpoint C14, 1080p clip, 21.3 fps unchecked) decodes 1920 x 1080 x 1.5 x 21.3 = 66.3 M samples a
  second: **11.3 times less**.
- In samples per clock cycle: 0.66 today; 4K60 needs **7.5 at 100 MHz**, 6.0 at 125 MHz, 3.7 at 200 MHz.
- A clock step gives at most its own ratio. The rest is samples per cycle: wider datapaths and stages side by side.

## What real 4K streams look like (measured, small sample)

Headers of the first ~250 frames of nine YouTube 2160p AV1 videos, seven of them 60 fps
(`~/projects/openav1-4k60/` on the Pi: `yt_sample.sh`, `hdr_census.py`, the samples):

| tile columns x rows | superblock | videos |
|---|---|---|
| 4 x 1 | 64 | 5 |
| 8 x 1 | 64 | 1 |
| 8 x 4 | 128 | 3 |

- None is a single tile. The fewest tile columns is 4.
- The loop filter is on in nearly every frame for seven of the nine (0 of 255 and 66 of 255 frames in the other two).
- None of the nine uses loop restoration or super-res; six use CDEF in every frame (see the next section).
- Our 1080p scoreboard clip (`in_to_tree-aom.ivf`) has the loop filter **off in 428 of 548 frames** and is a single
  tile: it under-weights the filters and cannot show a tile-engine gain.
- Limits of this sample: one service, nine videos. libaom's default is one tile, so a single-tile 4K stream must
  decode correctly; it need not reach 60 fps.

## Which filters and how many tiles: more streams and the encoders themselves (measured 2026-11-13 evening)

Frame headers read with `~/projects/openav1-4k60/hdr_census2.py` (on the Pi; samples in `other/`, encodes in `enc2/`).

Real streams:

| source | samples | tiles | loop restoration | CDEF | super-res |
|---|---|---|---|---|---|
| YouTube 4K / 8K / 1080p, family A (sb64) | 9 | 2x1 (1080p), 4x1, 8x1 | never | allowed in all, non-zero in most frames of 8 | never |
| YouTube 4K / 8K, family B (sb128) | 8 | 8x4 | never | switched off for the whole stream | never |
| Bilibili (only 852x480 AV1 without a login) | 5 | 1x1 | in 12, 47, 63, 185 and 198 of 204 frames, Wiener and self-guided | non-zero in most frames | never |

What the software encoders write by default and at their speed settings (Debian 13 packages through ffmpeg: libaom
3.12.1, SVT-AV1 2.3.0, rav1e 0.7.1; 24 frames of an 8K sample scaled down to 4K and to 1080p, constant quality):

| encoder, setting | tiles at 4K | loop restoration at 4K (frames) | at 1080p | CDEF non-zero at 4K |
|---|---|---|---|---|
| libaom good, cpu-used 1 (ffmpeg default; 7 frames) | 1x1 | 1 of 7, Wiener | 1 of 7 | 6 of 7 |
| libaom good, cpu-used 4 | 1x1 | 6 of 26, Wiener and self-guided | 8 of 26 | 15 of 26 |
| libaom good, cpu-used 6 and 8 | 1x1 | none | none | 18 of 25 |
| libaom realtime, cpu-used 8 | 1x1 | none | none | 24 of 24 |
| SVT-AV1 presets 4, 6, 8 | 1x1 | 7, 10, 11 of 24, Wiener only | 10 of 24 | 17 to 20 of 24 |
| SVT-AV1 presets 10, 13 and ffmpeg's default | 1x1 | none | none | 0 of 24 |
| rav1e default, speed 4, speed 6 | 1x1 | 24 of 24 (switchable) | 24 of 24 | 24 of 24 |
| rav1e speed 10 | 1x1 | none | none | 24 of 24 |

Documented: SVT-AV1's parameter table lists restoration, CDEF and the deblocking filter as enabled by default and
the tile defaults as 0 ("default changes per resolution"); libaom's speed features switch both restoration filters
off in realtime mode and the self-guided one from speed 5 in good mode; the aomenc manual says restoration is on by
default except in realtime mode.

Readings:

- **Restoration is not used by YouTube at any resolution sampled, and it is used elsewhere**: by another service at
  low resolution, and at 4K by the slower settings of all three software encoders (a third of the frames with libaom
  and SVT-AV1, every frame with rav1e). SVT-AV1, the most used of them, wrote Wiener only.
- **Every software encoder writes a single tile at 4K unless told otherwise.** Tile engines multiply YouTube's
  streams; a 4K file someone encoded themselves is one tile and needs the whole rate from one engine.
- Super-res did not appear anywhere.
- Limits: 22 real streams from two services, one 24-frame clip per encoder setting, one version of each encoder.
  No data on Netflix, Meta or hardware encoders: a web search found no primary source that lists the tools they use.

## Two kinds of stage

1. **Per tile**: syntax, coefficients, transforms, intra and inter prediction. These multiply with tile engines.
   With N engines each needs 7.5 / N samples a cycle at 100 MHz: **1.9 with four engines**, 0.9 with eight.
2. **Frame-wide**: loop filter, CDEF, loop restoration, the flush to DDR, the frame memory itself. Tiles do not
   split these. Each must carry the whole stream: **7.5 samples a cycle at 100 MHz**.

## Two targets side by side (arithmetic from the table below)

Since every software encoder writes one tile at 4K by default, the budget carries two figures for the per-tile
stages, at 100 MHz:

| target | what one tile engine must do | today (light / heavy 1080p frame) | short by |
|---|---|---|---|
| 4K60 in four or more tile columns (YouTube's streams) | 1.9 samples a clock | 1.16 / 0.41 to 0.51 | 1.6x / about 4x |
| 4K60 in one tile (a self-encoded file) | 7.5 samples a clock | 1.16 / 0.41 to 0.51 | 6.4x / 15 to 18x |
| 4K24 in one tile (a film encoded at home) | 3.0 samples a clock | 1.16 / 0.41 to 0.51 | 2.6x / 6 to 7x |

Inside one tile only the entropy decoding is strictly serial; prediction, transforms and the filters can be widened
and overlapped. The single-tile figures are what the per-engine work (the MC path, the transform and coefficient
path) is measured against; tile engines only close the first row.

## Per stage, today (1080p; a superblock row is 184,320 samples, a frame 3.11 M)

| stage | kind | today, samples a cycle | needed at 100 MHz | short by | source |
|---|---|---|---|---|---|
| tile engine, light inter frame (frame 2) | per tile | 1.16 | 1.9 (4 engines) | 1.6x | measured: tile done in 2,692,755 cycles |
| tile engine, heavy inter frame (frame 14) | per tile | 0.51 | 1.9 | 3.7x | measured: 6,047,547 cycles |
| same frame with the filters running (dec_host) | per tile | 0.41 | 1.9 | 4.6x | measured: tile done at 75.2 ms |
| loop filter, level 4 (frame 14) | frame-wide | 0.9 (0.87 to 1.79) | 7.5 | 8x | measured: 103k to 211k cycles a row, median 203k |
| CDEF, free-running | frame-wide | 1.2 | 7.5 | 6x | measured 2026-11-11: 155k cycles a row |
| loop restoration | frame-wide | 1.8 | 7.5 | 4x | measured 2026-11-11: about 100k cycles a stripe |
| flush to DDR (streamed since today) | frame-wide | not measured per row | 7.5 | ? | frame 2 ends 1.79 ms earlier; no per-row figure yet |
| whole card, clip mean | | 0.66 | 7.5 | 11x | measured: C14, 21.3 fps |

Sources: `~/logs/perf1080-f{2,14}-mctddccafe-r16.log`, `~/logs/tail_twin_lfe_f14.log`, journal 2026-11-11 for CDEF
and loop restoration (frame 2, before today's changes; not re-measured).

Readings that follow from the table:

- **The frame-wide filters are the wall that tiles do not help**: each of the three is 4 to 8 times short, they run
  in a chain, and on real 4K streams the loop filter is nearly always on.
- **The filters already slow the tile engine today.** Frame 14's tile takes 6.05 M cycles alone and 7.52 M with the
  filters behind it: 24 % more, from the window holding reconstruction back while the filters catch up.
- **Per engine the gap is 2 to 5 times with four engines**, less with eight (three of the nine samples have 8 tile
  columns). Which part of the engine limits depends on the frame: frame 2 is prediction-bound (transform side idle
  92 % of the tile), frame 14 spends 35 % of the tile in the transform path and its syntax stage is the busy one.

## Inside the tile engine: motion compensation (measured, the two strip traces)

`~/projects/openav1-4k60/traces/mct_f{2,14}.txt`, totals from `mct_stats.py`:

| | frame 2 | frame 14 |
|---|---|---|
| strips (fetch + filter runs) | 17,708 | 42,203 |
| cycles a strip (tile span / strips) | 152 | 143 |
| reference samples written into patches, per output sample | 2.04 | 3.36 |
| mean fetch time a strip (accepted to patch complete) | 170 | 118 |
| mean horizontal + vertical filter time a strip | 112 | 62 |
| most common strip | luma warp 8x8 (5,362) | luma warp 8x8 (19,585) |

- The fetch takes about as long as the strip period on both frames: about one strip is in the fetch at any time.
  That is a hypothesis for the coupling seen on 2026-11-12 (three what-ifs, each about 3 %), not yet tested; the
  cycle model is being built to test it.
- Every output sample costs 2 to 3.4 reference samples moved, because each strip fetches its own (w + 7) x (h + 7)
  patch and warp blocks are cut into 8x8 strips.

## Memory and the DDR port (arithmetic unless marked)

- **Frame memory on the chip**: the window holds 5 / 3 / 3 superblock rows of the three picture buffers at 1920
  wide and uses 556 of the U200's 960 UltraRAMs (measured, C14: 57.9 %). At 3840 wide the same window is about
  twice that: **it does not fit as it is.** Options: a shorter window, a different split between on-chip window and
  DDR, or a larger device.
- **The frame memory runs at twice the clock** (double-pumped). It is the tightest timing family on the card
  (measured: 497 paths within 0.5 ns, worst 0.007 ns, on the 200 MHz clock) and it is what a clock step runs into
  first.
- **DDR port**: reference fetch at 4K60 is 746.5 M x (2 to 3.4) = 1.5 to 2.5 G samples a second read, plus 746.5 M
  written by the flush. The decoder's memory port is 64 bits at 100 MHz: 0.8 GB a second at best. **Too narrow by 3
  to 4 times** even before counting the write side. The U200's memory controller offers a 512-bit port.
- **Logic** (measured, C14): the decoder is 381,579 of 1,182,240 LUTs with two tile engines (32 %; the whole image
  463,230 = 39 %), 1,153 of 2,160 block RAMs, 1,001 of 6,840 DSPs. The per-engine share is not measured, so whether
  four or eight engines fit is open.

## Ranking (my reading of the above)

1. **The three in-loop filters as streaming filters** at 8 or more samples a cycle each, fed from line buffers. They
   are frame-wide, 4 to 8 times short, always on in real streams, and already hold the tile engine back.
2. **Frame memory for 3840-wide pictures without the double clock**, together with a wider DDR port (512 bits) and
   the flush on it. Nothing at 4K runs without this, and it frees the clock.
3. **Four tile engines** (eight later): the cheapest multiplier for the per-tile stages on streams as they are.
   Needs the per-engine area measured first.
4. **Inside each engine**: the wide motion-compensation path (fewer reference samples per output sample, a wider
   fetch, luma and chroma side by side), then the transform and coefficient path for heavy frames. The cycle model
   prices these.
5. **Clock**: margin and a step to 110 to 125 MHz once the frame memory no longer runs at twice the clock. 200 MHz is
   not assumed.

## Not measured yet

- Loop filter at higher levels and on intra frames (level 4 only so far); CDEF and loop restoration on today's main.
- The flush per row after today's change.
- Area of one tile engine.
- Syntax throughput in symbols a cycle, and what a 4K60 bitrate (the samples run 10 to 28 Mbit/s) asks of it.
- Whether a 4K scoreboard clip decodes at all on today's RTL in simulation (FBX 12): no 4K run has been made.
- Tile layouts outside YouTube (other services, local encodes).
