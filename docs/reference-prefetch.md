# Reference prefetch: taking DDR latency off the inter critical path (design note, 2026-11-02)

Why now: the U200 decodes 1080p inter content at 3.5 fps end to end (in_to_tree-aom.ivf, 548 coded frames, image
`u200-fm2s100p-0fd3b46`, host 771f8f6: 158.3 s, of which the tile stage is 148.2 s = 94 %). The intra frame of the
same clip takes 0.13 s (13 M cycles at 100 MHz, 6.3 cycles per luma pixel, in line with the 2026-10-27 intra census);
the inter frames take 0.18-0.49 s (median 0.24 s). The difference is the reference fetch, and the reason is structural,
not bandwidth: `ref_fetch` reads a prediction unit's patch one row at a time with one AXI read in flight, and
`recon_top` waits for the whole patch before the filter starts. Every patch row pays a DDR round trip through the
card's AXI interconnect and the DDR4 controller, serially. This note measures that and lays out the fix in steps
that keep the decoder bit-exact at every commit. The TE=2 census of the same day (301 of 764 Argon profile-0 streams
fully on the card, 0 differ) does not change any of it: the fetch path is shared by both engines. Companion notes: `docs/picture-memory.md` (the memory plan;
its §4 is the bandwidth half of this), `docs/throughput-budget.md`, the inter note §7.

## 1. What the fetch engine does today (rtl/ref_fetch.sv, recon_top B_MC_FETCH)

Per prediction unit and list, the patch is rows `[y0 - 3, y0 + h + 4)` and columns `[x0 - 3, x0 - 3 + 4 (ngrp + 2))`,
`ngrp = ceil(w / 4)`. The engine walks the rows: F_ROW (clamp) → F_ADDR → F_AR (one AR, ≤ 16 beats of 8 bytes, never
across 4 KB) → F_R (beats into the line buffer) → F_WR (patch words to `mc_filt`) → F_NEXT, next row. Nothing is in
flight while a row's data is on its way back, and the next row's address is not issued until this row is written.
Rows that clamp to the same picture row are read again. `recon_top` issues the luma patch, then each chroma patch,
and starts `mc_filt` only when `rf_done` says the last patch is complete; the filter (four output columns per
clock) then runs with the fetch engine idle. Chroma for sub-8 blocks and OBMC neighbours add further patches of the
same shape. So per block the fetch costs `rows × (L + beats)` cycles, with `L` the read latency the card's AXI
path shows the decoder and `beats` the row's burst length, and the filter cost is added, not overlapped.

The simulation twins hide this: `tb/axi_mem.py` answers a read within a few cycles (random stalls at 30 %), so the
cycle counts the nightly reports for inter streams are a lower bound for the card. The card is the only place the
latency term shows, which is why the inter frames' cost was a surprise on 2026-11-01.

## 2. Measured: what the frames ask of DDR

Syntax-model census of in_to_tree-aom.ivf (1920x1080 4:2:0 8-bit, libaom, cq30-class), frames 1-13, counting what
`ref_fetch` as built issues (one AXI read per patch row, 8-byte beats; chroma blocks below 8 samples count as 8;
OBMC neighbours as half a block extra): `~/measure1080/rb/fetch_census.txt`, script `tools/fetch_census.py`.

| frame | inter blocks | lists | typical sizes | patch rows (= AXI reads) | 8-byte beats | DDR 64 B words: linear / tiled / 128-tile cache | card tile time |
|---|---|---|---|---|---|---|---|
| 1 | 7,718 | 7,718 | 16x16, 8x8, 16x8 | 404,354 | 1,456,036 | 531k / 48k / 113k | 0.34 s |
| 2 | 2,361 | 3,645 | 32x32, 16x16, 64x64 | 268,193 | 1,333,200 | 407k / 85k / 175k | 0.24 s |
| 3 | 1,556 | 2,292 | 32x32, 16x16, 64x64 | 197,580 | 1,137,311 | 346k / 85k / 156k | 0.20 s |
| 4 | 945 | 1,023 | 64x64, 32x32 | 112,123 | 751,408 | 228k / 60k / 102k | 0.21 s |
| 8 | 1,468 | 2,569 | 32x32, 64x64, 16x16 | 222,037 | 1,278,945 | 387k / 94k / 181k | 0.19 s |
| 13 | 510 | 510 | 64x64 only | 75,990 | 600,780 | 181k / 49k / 83k | 0.36 s |
| 5-12 (range) | 612-867 | 1,043-1,413 | 64x64, 32x32 | 132k-156k | 0.95-1.06 M | 287k-324k / 84k-96k / 141k-160k | 0.18-0.23 s |

Three readings. Bandwidth is not the wall at 1080p: the linear rows touch 180k-531k 64-byte DDR words per frame
(12-34 MB; 0.7-2 GB/s at 60 fps against a 19.2 GB/s bank, 14 usable), and a tiled layout would touch 48k-96k distinct
64-byte tiles (3-6 MB, 4-6x less: the compulsory traffic), a 128-tile direct-mapped cache per list about 1.7x that
(these are the §4 numbers `docs/picture-memory.md` asked for before building either). The count of reads is the
problem: 76k-404k serial round trips per frame. And frame 13 (the fewest rows, 64x64 blocks everywhere)
is the slowest inter frame: its cost is syntax and residual, not fetch, so the per-row latency has to be
separated from the rest before the fix is sized. §3 does that with the simulation twin on the same frames.

## 3. Measured: the latency term

The same frames through the simulation twin (`tb/test_dec_host_inter.py`, FBX = FBY = 11, WIN 5/3/3, stage B with the
10-buffer pool, i.e. the card's configuration) against the card (`~/measure1080/rb/raw.log`, image
`u200-fm2s100p-0fd3b46`). Simulated cycles are the sequencer's own count (the Python model's time does not advance the
clock) from a frame's `frame started` line to its `identical` line; the card's are the tile-stage wall time at 100 MHz
(two digits: ±0.5 M cycles, i.e. ±1-3 cycles per read). The twin's own frame-state load between frames is left out
on both sides: it is 0.24 M cycles with stage B (frame 1) and 1.4 M through the register path (frames 2-3, which ran
with stage B off after the first run stopped on the loader wait bound), and it is host time, not tile time.

| frame | kind | reads | simulation | card | card − sim | per read |
|---|---|---|---|---|---|---|
| 0 | intra | 0 | 12.79 M cycles | 13 M (0.13 s) | ~0 | — |
| 1 | inter, 7,718 blocks (16x16 / 8x8), no compound | 404,354 | 14.8 M | 34 M (0.34 s) | 19.2 M | **48 cycles** |
| 2 | inter, 2,361 blocks (32x32 / 16x16 / 64x64), 1,284 compound | 268,193 | 11.2 M | 24 M (0.24 s) | 12.8 M | **48 cycles** |
| 3 | inter, 1,556 blocks (32x32 / 16x16 / 64x64), 736 compound | 197,580 | 10.1 M | 20 M (0.20 s) | 9.9 M | **50 cycles** |

The intra frame agrees to 2 %: everything the testbench models (syntax, reconstruction, the filters under the
sequencer, the register traffic) costs on the card what it costs in simulation. The inter frame does not, and the
whole of the difference sits in the one thing the testbench answers in a few cycles and the card does not: 404k
reference reads at about 48 cycles each, 480 ns per round trip through the XDMA interconnect and the DDR4 controller.
Frames 2 and 3 say the same: 48 and 50 cycles per read with half the reads, larger blocks (fewer, longer rows) and
a third to a half of the blocks compound (two references per block, so two patches). The term is a per-row round trip
that does not depend on block size or on the compound share, which is what a serial fetch predicts, and the design
below is sized against ~48 cycles. The Argon inter streams (small pictures) remain to be measured. A side finding from the same run: at 1080p the stage B
frame-state loader (zero segment map, CDF rows, motion-field projection) ran past 460k cycles on frame 2, where the twin's
wait bound stopped it (raised); on the card that is ~5 ms a frame, hidden inside the host's timeout today but 15 % of a
33 ms frame at 30 fps, so the projection engine joins the throughput list. The number is also what makes
frame 13 the slowest inter frame despite the fewest reads: with 64x64 blocks its cost is syntax and residual, which
the simulation reproduces, so a fix that removes the latency term alone leaves the 64x64 frames where they are.

## 4. The design, in steps

**A. Pipelined rows in `ref_fetch` (the fix that matters).** Split the engine into an address side and a data
side joined by a row descriptor FIFO: the address side walks the rows as now (clamp, address, burst length) and
issues an AR per row as fast as the interconnect accepts them, up to `N_OUT` outstanding (8-16: the row FIFO depth);
the data side consumes R beats in order (one AXI ID, in-order responses, which the XDMA/MIG path and the KV260's
HP ports both give) into a line buffer ring (two or three rows of 80 samples, as today's one) and writes patch words
as each row completes. The patch then costs `L + rows × (beats + 1)` instead of `rows × (L + beats)`: at `L` of
several tens of cycles and 17 rows per patch, 5-10x fewer fetch cycles per block. Nothing outside the module
changes: same request, same `done`, same patch write port, same clamping; the twin suites (`dec_host_inter`,
`dec_host_inter_argon`, `dec_host_cardload`, `dec_inter*`) and a netlist replay of a recording prove it, and the
U200 1080p clip measures it. Risk: AXI responses interleaving (none on a single ID), 4 KB boundary bursts (the
address side splits them as today). Cost: one small FIFO and a second state machine, ~500 LUT.

**B. Block-ahead fetch.** `recon_top` issues block n+1's patches while `mc_filt` filters block n: a second patch
buffer in `mc_filt` (ping-pong; the patch is 72 rows × 20 words × 4 × 12 bits ≈ 8.6 KB, so two fit in BRAM as
today's one does), `B_MC_FETCH` split into "issue" and "wait", the block record pipeline one deeper. The remaining
`L` per block hides behind the previous block's filter whenever the filter takes longer than one round trip
(it does for 16x16 and up). Touches the recon sequencer; verified by the same suites. Do after A has been measured
on the card; A may already leave the filter as the limit.

**C. Bandwidth: tiled layout and a tile cache** (`docs/picture-memory.md` §4). Not needed at 1080p (§2: 0.7-2 GB/s
linear); at 4K60 the linear figure is 3-8 GB/s on this kind of content and more on 8x8-heavy frames, so the tiled
layout (4-6x less, measured above) and a cache (a 128-tile direct-mapped one already halves the tiled traffic's
misses on frame 1's 8x8/16x16 content: 113k misses for 48k tiles) go in with the 4K memory. Sequence it with the 4K memory
(step C of picture-memory.md), not before.

**D. Reference rows on chip (the FPGA-only option).** A ring of reference rows per slot around the current
superblock row (±64 lines + taps), filled ahead by a prefetcher from the slot table, with DDR as the fallback for
motion vectors reaching outside. At 1080p: 7 slots × 3 planes × ~140 lines × 1920 × 2 B ≈ 11 MB, ~310 of the U200's
960 UltraRAMs; at 4K four times that, which does not fit. It is the one step that removes DDR from the inter path
entirely, and the one that does not carry to silicon unchanged (a tile cache does). Only if A-C leave the fetch as
the wall at 4K, and only on the measurement.

## 5. Order and what to expect

A first (this week, after the symposium), measured on the 1080p clip and on the Argon inter streams; B if the card
still shows fetch idle time in the filter; C with the 4K memory. The expectation from §2-3: the inter frames drop
toward the intra frame's cost plus the filter time, which for this clip is 2-3x on the tile stage, i.e. 1080p
inter from 3.5 fps to 7-9 fps at 100 MHz with nothing else changed. The further steps on the roadmap (second tile
engine on real content, four pixels per clock in the tile stage, 200 MHz) multiply from there.

## 6. Measured (2026-11-03)

**Step A on the card.** Image `u200-fm2pfs100p-91c481f` (fm2 + the pipelined `ref_fetch` + per-UltraRAM phase copies,
TE=1, sb128, 100 MHz, lr_pblock, ExtraTimingOpt; WNS +0.013 ns, 0 failing): `in_to_tree-aom.ivf`, 548 coded frames, 500
pictures, all identical to dav1d:

| image | clock | fetch | 548 frames | fps | frame 1 tiles | frame 2 | frame 3 | frame 13 |
|---|---|---|---|---|---|---|---|---|
| fm2s100p-0fd3b46 | 100 MHz | serial | 158.3 s | 3.5 | 0.34 s | 0.24 | 0.20 | 0.36 |
| fm2s80p-0fd3b46 | 80 MHz | serial | 181.3 s | 3.0 | 0.39 | 0.27 | 0.23 | 0.41 |
| fm2pfs100p-91c481f | 100 MHz | rows in flight (A) | 78.9 s | **6.9** | 0.19 | 0.12 | 0.09 | 0.15 |

The 80 MHz serial row says the same thing as §3 from the other side: 15 % slower at 80 % of the clock, because the DDR
round trip is fixed in nanoseconds. With A the inter frames sit at 0.09-0.19 s = 9-19 M cycles against the 12.8 M of the
intra frame and the 14.8 M the testbench predicts for frame 1 at zero latency: a few million cycles of latency are left
per frame, which is step B's target (block-ahead fetch, bit-exact in the twin on the fm2 line; netlist rung and image
pending). The §5 expectation for A alone was 7-9 fps.

**The first prefetch image missed timing** (-0.981 ns, 3,104 failing endpoints) on `phase -> LUT3 -> URAM288 ADDR/EN`:
one phase register plus one request mux drove 120 UltraRAMs across the die; earlier images had closed the same path at
+0.001. The fix (`frame_mem_uram.sv`, synthesis branch): two clk2x stages of `phase` per UltraRAM (the copy equals
`phase` exactly since it alternates every cycle) and a request mux per UltraRAM; the netlist replay checks the
behavioural branch still matches. An incremental implementation from the previous routed checkpoint failed outright
(Place 46-13: the changed fetch engine moved too much), and a provisional run of the timing-failing image corrupted
every frame including the intra one: the +0.001 images really were at the edge.

**The loader group** (§3's side finding): the host's waits measured on the card with the TE=2 serial image, same clip:
loads 5.69 s = **10.4 ms a frame** (zero segment map + CDF rows + motion-field projection before every frame), CDF saves
0.32 s = 0.6 ms a frame. Rewritten the same day (`docs/frame-state.md` §6): the band one memory per reference, the clear
one position a cycle and overlapped with the CDF load, cells one a cycle, both loaders and the save with bursts in
flight; the stage B twin at 48-cycle latency goes from 267,909 to 71,972 cycles over 20 small frames (3.7x), the 1080p
twin and the card image are pending.
