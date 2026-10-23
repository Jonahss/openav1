# Picture memory for the full decoder (design, 2026-10-23)

Why now: the inter decoder is verified (gate rounds 7-10) but has no frame memory it can run on hardware with
(`FRAME_URAM = 0` is the simulation model, a black box in synthesis), the Alveo U200 arrives 2026-10-28, and
Jonah set the finish line at **4K60, in silicon if not on the FPGA** (2026-10-23). The memory system is the part
of a decoder that is hardest to widen later, so it is designed against that target from the start, and built in
steps that each keep the decoder bit-exact. Companion notes: `docs/frame-memory.md` (the on-chip and window
versions, what exists), `docs/throughput-budget.md` (where the cycles go), the inter note §7 (reference fetch).

## 1. The budget

4K60 4:2:0 10-bit: 3840 x 2160 x 60 = **498 M luma pixels/s**, 747 M samples/s with chroma. Samples are stored as
16 bits in DDR (the flush layout the inter decoder already uses; 8-bit pictures as bytes).

| | per second | per luma pixel |
|---|---|---|
| picture out (final frame to DDR, also the next frames' reference) | 1.5 GB/s | 3 B |
| reference reads, 8x8 blocks, two lists, 15 x 15 patch per 8 x 8, linear layout, 64 B DDR words | up to 40 GB/s | up to 80 B |
| reference reads, same, tiled layout + tile cache (§4) | 3-6 GB/s | 6-12 B |
| motion field + segmentation map + CDF save/load | < 0.1 GB/s | |
| one U200 DDR4 bank (72-bit DDR4-2400) | 19.2 GB/s peak, ~14 usable | |

Cycle budget per stage at 200 MHz: 0.40 cycles per luma pixel (`docs/throughput-budget.md`), i.e. every stage
has to move **2.5 luma pixels (3.75 samples) per cycle on average**, so the datapaths and the memory ports are
designed for **8 samples per cycle** (a 128-bit word of eight 16-bit lanes), twice the average, with the slack
absorbing the stress streams. Today's word is 4 samples (72-bit URAM word, 4 x 18); today's stages move 1 to 4.

## 2. What the stages ask of the memory (from the RTL as built)

Every stage is **row-local with a bounded vertical reach**; that is what makes line buffers sufficient:

| stage | reads | writes | vertical reach beyond the current superblock |
|---|---|---|---|
| reconstruction (intra edges, residual add) | fb0: the line above, the column to the left | fb0: the block | 1 line above |
| reconstruction, inter | references in DDR (`ref_fetch`), fb0 for OBMC read-modify-write of the block itself | fb0 | none in fb0; ±4 (+3 taps) around the block in the reference |
| intra block copy (`allow_intrabc` frames, filters off) | fb0 anywhere already decoded in the tile, at least 256 px behind | fb0 | whole rows above |
| deblocking (`lf_top`) | fb0 around every 4x4 edge | fb0 in place | 7 lines above (14-tap horizontal edges at the superblock top) |
| CDEF (`cdef_top`) | fb0, 8x8 block + 2-3 sample border | fb1 | 3 lines above **and below** |
| super-resolution (`sr_top`) | fb0 / fb1 row | same row in place | none |
| loop restoration (`lr_top`) | fb0 and fb1, a 4-line band + 3 lines above and below, stripes of 64 offset by 8 | fb2 | 3 lines above and below, stripe rule |
| host / display / next frame | fb2 (or fb1 / fb0 when LR / CDEF are off) | | |

CDEF and LR reading **below** the current superblock is the one structural fact: the bottom lines of a
superblock row cannot be CDEF-filtered until the row below is deblocked. Either the stage lags by a whole
superblock row (how `dec_top` sequences today: tiles of row r, deblocking of row r-1), or it processes the row
except its last lines and comes back for those when the next row is there (how production decoders do it, with
a line buffer holding the deferred lines).

## 3. Architecture

Three layers, each already present in some form:

1. **DDR holds pictures**: the frame being decoded is written out superblock row by superblock row as it becomes
   final (the window build's flush, `dec_host`'s DMA engine on `m_axi`); references are read back by `ref_fetch`
   (stage 2a) from the slot table; the host reads the final picture from DDR. Eight slots x three planes, in the
   layout software programs (§4 adds the tiled option).
2. **On-chip rings hold the rows in flight**: fb0, fb1 and fb2 each become a ring of superblock rows (today only
   fb0 is, `FRAME_WIN`), addressed by `row mod N` with the same ports and one-cycle latency the stages have now.
   A stage works on its row while the earlier stages work on the rows after it and the later ones on the rows
   before it; the ring depth is the number of rows in flight plus one for the hold.
3. **Line buffers replace the rings where memory is scarce** (silicon): when the stages are pipelined per
   superblock instead of per row (§6, step C), the rows in flight shrink to the lines each stage reaches across
   the superblock boundary (1, 7, 3+8, 3+3) plus the current superblock, and the ring of N x 64 lines becomes a
   ring of a dozen lines per stage. Same ports, same addressing function (`wline` already maps picture lines to
   ring lines), smaller N and finer-grained release.

The stage schedule per superblock row r (row-granular pipeline, step A):

```
tiles / recon   r
deblocking      r-1      (writes the last 7 lines of r-2 too: those are final after this)
CDEF            r-2      (needs r-1 deblocked for its bottom lines)
super-res       r-2      (after CDEF of the row, fb0 and fb1 rows in place)
LR              r-3      (stripe offset 8: needs r-2's first lines in fb0/fb1; its output is the final picture)
flush -> DDR    r-3 (fb2), or r-2 (fb1) when LR is off, or r-2 (fb0) when CDEF is off too
```

Ring depths: fb0 holds rows r .. r-3 plus the hold slot = **5**; fb1 rows r-2 .. r-3 + hold = **3**; fb2 the row
LR writes + the one being flushed = **2**. Each superblock row is `64 x UpscaledWidth x 1.5` samples (4:2:0; sb128
doubles it or takes two slots). On the U200 in 4-sample URAM words:

| picture | words per row | URAM per row | fb0 (5) + fb1 (3) + fb2 (2) |
|---|---|---|---|
| 1080p (1920) | 46k | 12 | 120 URAM |
| 4K (3840, super-res up to 4096) | 98k | 24 | 240 URAM |

of 960, with the K26 window build's 35-URAM 1080p fb0 as the reference point. Fine for the FPGA. In silicon the
row-granular rings at 4K are ~70 Mbit: too much, which is what step C is for (line-granular: ~4 Mbit).

**Intra block copy** reads the current frame anywhere behind the decoding position: within the ring rows it is a
plain fb0 read (as today without the window); further up the rows are in DDR and go through the reference fetch
engine, which already knows how to read a clamped patch from a slot (the current frame's own slot, filters are
off in intrabc frames so fb0 is the final picture). The spec's 256-pixel delay constraint is what makes the DDR
copy of the row above complete before it is referenced; the flush must therefore run per superblock, not per
row, in intrabc frames (or the ring keeps one more row).

**Reset of the one-tile-column limit**: the window build refuses several tile columns because a full-width row
cannot be released before the next column starts. With CDEF / LR in the pipeline the same holds; the fix is the
same as for intrabc: release and flush per superblock (64 columns of the row) rather than per row, with the
hold on slot reuse at superblock granularity. Step A keeps per-row release for the first inter build and adds
per-superblock release as the first throughput change, since every later step needs it.

## 4. References in DDR: tiled layout and tile cache

The reference fetch as built reads linear rows: `(h + 7)` bursts of `w + 7` samples per block per list, each
burst a 64-byte-aligned DDR access or two. For 8x8 blocks on 16-bit samples that is 15 rows x 64 B = 960 B per
list for 450 useful, and 4K60 at 8x8 everywhere would ask ~40 GB/s: more than a DDR4 bank. Two measures, both
standard in video decoders:

- **Tiled frame layout**: store each plane as 64-byte tiles of 8 columns x 4 rows (16-bit) or 16 x 4 (8-bit).
  A `(w + 7) x (h + 7)` patch touches `ceil` tiles: 3 x 5 = 15 tiles = 960 B for 8x8 (no better) but the fetch
  issues 15 aligned 64 B reads instead of 15 partially used bursts, and for 16x16 and up the waste drops to
  ~1.5x. Addresses: `base + (y / 4) * tiles_per_row * 64 + (x / 8) * 64 + (y % 4) * 16 + (x % 8) * 2`.
- **Tile cache in the fetch engine**: neighbouring blocks share most of their reference tiles (the next 8x8 to the
  right needs 3 x 5 tiles of which 2 x 5 were just fetched). A small direct-mapped cache per list (64-128 tiles,
  4-8 KB BRAM, indexed by tile address, one frame's slot id as part of the tag) cuts reference traffic by 2-3x on
  dense frames and costs nothing on sparse ones. Hit rate is measurable in simulation before building it.

Together: 3-6 GB/s at 4K60 worst case, inside one bank. The flush writes tiled when the slot says so (a per-slot
format bit next to `fmt16`); the host software detiles (numpy) or asks for a linear copy. Linear stays available
for the KV260 intra path and the display. Decided per measurement in step B, not before: the simulation's AXI
memory model (`tb/axi_mem.py`) can count DDR words per frame today.

## 5. Interfaces that do not change

- The stage-side frame memory ports (`frame_mem.sv` contract: re / we with lanes, one-cycle latency, no
  back-pressure): the rings honour it as the window does. Widening to 8 lanes is a parameter of the same ports.
- `dec_host`'s register map, slot table, DMA engine and `m_axi`: the flush and the fetch are already there; the
  rings add a second and third flush source and the tiled format bit.
- The verification flow: the simulation model remains the oracle, every ring configuration replays the whole-
  decoder recordings against the netlist (`tb/gate`), the frame memory itself is fuzzed against the model
  (`tb/test_frame_mem_uram.py`).

## 6. Steps

**A. Rings for fb0 / fb1 / fb2 with stage lags; flush of the final buffer; inter from DDR.** `frame_mem_uram`
gains an instance per buffer (fb1 and fb2 are the two-port cases the double-pump already serves), `dec_top`'s
sequencer gains the CDEF / SR / LR lags and `row_final` moves to the last stage in use, `dec_host` flushes the
right buffer, `lf_top` / `cdef_top` / `lr_top` run on row ranges (LR already takes a stripe; CDEF and the frame-
level LF loops get a row window). References linear (as verified). Result: the complete inter decoder with
CDEF and LR on the U200 at 1080p and 4K, all four Argon profiles' geometry, verified by the existing suites at
window geometry and the netlist replays. This is the memory the first inter decode on the card uses; nothing in
it is discarded later.

**B. Reference bandwidth**: count DDR words per frame in simulation on the dense Argon streams and natural 4K
clips; add the tiled layout and the tile cache if the numbers say so (expected yes for 4K60, no for 1080p).

**C. Superblock-granular pipeline and line buffers**: per-superblock release and flush (removes the one-column
limit, enables intrabc through the fetch), stages that process a superblock and defer the bottom lines, rings
measured in lines; the filter datapaths at 4-8 samples per cycle out of line buffers (the `lf_top` header has
said so since 2026-09-28). This is the throughput phase's memory half and the silicon-sized configuration; the
decoder-side datapath half (8-lane reconstruction, 4 transform lanes, parallel tile columns) is sequenced by
the census in `docs/throughput-budget.md`.

**D. 8-sample words** when the datapaths widen: 128-bit lanes in the rings (two URAMs side by side on the
FPGA), 128-bit `m_axi` bursts, the fetch engine's patch buffer two words wide.

Step A starts 2026-10-24.
