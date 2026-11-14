# Streaming filter chain: a proposal (2026-11-13)

Status: **sketch for discussion, no RTL yet.** It follows from `docs/4k60-budget.md`: the three in-loop filters are
frame-wide, 4 to 8 times short of 4K60, and built on a frame-memory port that cannot pass 4 samples a clock.

## What changes

Today:

```
tile engines -> fb0 (window, 5 superblock rows) <-> deblock (in place)
                fb0 -> CDEF -> fb1 (3 rows) -> restoration -> fb2 (3 rows) -> flush -> DDR
```

Every stage reads and writes whole picture buffers through 4-sample ports of double-pumped UltraRAM. The buffers
hold 5 + 3 + 3 superblock rows of the full width (556 UltraRAMs at 1920 wide).

Proposed:

```
tile engines -> row store (pre-filter, 2 superblock rows)
                   |  one superblock row at a time, left to right, 16 samples a cycle
                   v
                deblock -> CDEF -> super-res -> restoration -> writer -> DDR (512-bit port)
                (each stage keeps only its own context lines)
```

- The filters stop using picture buffers. Each is a pipeline stage that takes a column of blocks in, keeps the few
  lines and columns of context it needs in its own small memories, and passes finished samples on.
- The only picture-sized memory left on the chip is the **row store**: the pre-filter samples of the superblock row
  being reconstructed and the one before it.
- `edge_line` (today's unfiltered last-line copy for intra prediction) stays as it is.

## Why a row store and not a pure stream

- Tile engines produce superblocks tile by tile, several tile columns at once. The filters cross tile edges and must
  see a superblock row across the whole frame width. So reconstruction of row r is collected first, then the chain
  runs over it left to right. That is the rule the sequencer already uses (`row_dec`).
- Reconstruction itself needs random access to the block it is building (prediction written, residual added, OBMC
  and inter-intra blends read back). The row store is that working memory; each tile engine works in its own columns
  of it.
- Two rows deep so that the engines can go on with row r + 1 while the chain drains row r.

## What each stage holds (arithmetic, from the AV1 filter definitions; to be confirmed against the models)

| stage | input | context it must keep | why |
|---|---|---|---|
| deblock | pre-filter row | 8 lines above per plane, 8 columns to the left | a horizontal edge at the row top reads 8 and changes up to 6 lines above; vertical edges come first |
| CDEF | deblocked | 2 lines above, 2 columns left of deblocked samples, plus the 8x8 direction search | taps reach 2 samples out; it must read deblocked, not CDEF-filtered, neighbours |
| super-res | CDEF output | a few columns | horizontal upscale, row by row |
| restoration | CDEF / super-res output and deblocked stripe boundary lines | 3 lines above and below, 2 + 2 saved boundary lines per 64-line stripe | 7-tap Wiener / self-guided windows, the stripe rule |
| writer | final samples | one burst | 512-bit DDR port |

Confirmed so far with stage models that take the picture band by band and raise on any read outside what they hold
(`tb/lf_stage_model.py`, `tb/cdef_stage_model.py`, checks `tools/check_lf_stream.py`, `tools/check_filter_chain.py`;
generated frames in 4:2:0 / 4:2:2 / 4:4:4 / mono at 8, 10 and 12 bits and the first frame of five real streams, both
superblock sizes, identical to the frame-at-once models):

- deblock: horizontal edges two columns behind the vertical edges, row by row, is bit-exact with the specification's
  order; 8 lines of context per plane are enough and 6 are not (7 is the minimum);
- CDEF behind it: carries at most 18 luma and 6 chroma deblocked lines (4:2:0) from one superblock row to the next;
  a reach of 1 instead of 2 fails;
- restoration behind that (`tb/lr_stage_model.py`): carries at most 11 luma and 7 chroma CDEF lines, and of the
  deblocked picture only the four lines around each stripe boundary (6 luma, 4 chroma held at once); keeping three of
  the four, or one CDEF line less above a block row, fails.

All three stages chained are identical to the frame-at-once models on frame 0 of the natural, screen and synthetic
streams (restoration on in three of them).

Inter frames and super-res (`tb/filter_chain_model.py`, run on every frame a decoder test bench filters when
`TS_CHAIN_CHECK=1`): the `dec_inter_lr` suite's 24 frames (13 inter; CDEF on in 22, restoration compared in 9, 5 with
super-res, where the deblocked and CDEF lines are upscaled one by one on the way into the restoration stage) and 13
frames of Argon test6641 / test8854 (CDEF and restoration in 10) are identical.

The column context of the deblock stage (`DeblockColStage` in `tb/lf_stage_model.py`): block columns enter one at a
time; with column i in, the vertical edge on the left of column i - 1 is filtered, then the horizontal edges of
column i - 3, which leaves. A window of four block columns is identical on 58 generated frames x superblock sizes;
three columns fail on 34 of them. The CDEF stage's column window (`CdefColStage`): while an 8x8 block is filtered
only the deblocked samples from two columns left of it to two columns right may be read (12 luma columns); identical
on the real frames, and a reach of one column fails. Restoration still sees whole lines in the model.

Because a stage's output for the bottom lines of row r depends on the top of row r + 1, each stage finishes a row
shifted up by its context (deblock by 8 lines, CDEF by 2 more, restoration by 3 more). The context memories carry the
unfinished lines to the next row. In total about 20 lines of the frame width per plane: under 2 Mbit at 4K, against
the 11 superblock rows of picture buffer today.

## Rates

- Target for every stage: **16 samples a clock** (one 4x4 block), twice the 7.5 that 4K60 needs at 100 MHz, so that
  stalls and row boundaries have room. Deblock: four filter rows side by side on one 4-sample edge unit a cycle.
- The row store's read side for the chain is sequential, so it can be a plain wide memory at the system clock: the
  double-pumped UltraRAM and its timing family go away.
- Reconstruction keeps its 4-sample ports per engine: 1.9 samples a cycle per engine is under that ceiling.

## Memory at 4K (arithmetic)

- Row store: 2 rows x 3840 x 128 lines x 1.5 = 1.47 M samples; at 12 bits 17.7 Mbit, about 62 UltraRAMs (today's
  window: 556 at half the width).
- Context lines: under 2 Mbit.
- So 4K width fits on the U200 with room. This is the answer to "do we need a bigger FPGA" for memory; logic for four
  to eight tile engines is still unmeasured.

## What must be checked before RTL

1. **Who else reads the picture buffers today.** Intra block copy reads the current picture well above the current
   row; those frames have the filters off, so the samples it needs are already what goes to DDR, but the fetch path
   (`fb_fetch`) has to move to DDR for rows that left the row store. The host read-back port and the frame-state
   paths need the same look.
2. **128-pixel superblocks and frames narrower than a superblock**, 4:4:4 and 4:2:2, 10-bit: the context sizes above
   are for 4:2:0.
3. **Loop restoration units up to 256 and super-res together**: restoration works in the upscaled frame.
4. **Order of work inside a row for deblock**: vertical edges of a block column, then horizontal edges of the column
   to its left, so that every sample a horizontal edge reads has had its vertical edges.

## How it lands without losing a working decoder

- Each stage is built as its own module with a unit test against the existing Python model (`lf_model`, `cdef_model`,
  `lr_model`), and every test is shown to fail on a deliberately broken design before it counts.
- The models gain a banded entry point so a stage can be compared row by row, not only at the end of the frame.
- The chain goes in beside the current path behind a build parameter. The old path stays the default until the chain
  passes the full nightly, the gate rung and the card census with identical pictures.
- Order: deblock, CDEF, restoration with super-res, the writer on the wide DDR port, then the row store replaces the
  three windows. The 1080p card stays the scoreboard throughout; main is tagged before the switch.

## What this does not do

- It does not speed up the tile engines. Four engines and the MC path are the next items in the budget.
- It does not need a clock change, but it removes the 200 MHz memory clock that stands in the way of one.

## Deblock stage: first cut of the hardware (2026-11-13, not built)

Unit of work: one 4x4 block of a plane per clock, so 16 samples a clock.

- **Input**: the row store is read a column of 4x4 blocks at a time (4 samples wide, the band's height: 16 blocks of a
  64-line row), left to right. One block a clock.
- **Working window**: the last four block columns of the band plus the 8 context lines above them, in registers or
  small distributed RAM (4 x 4 samples wide, 72 lines high at 64-line superblocks: under 14 kbit a plane).
- **Vertical edges**: the edge on the left side of block column c is filtered when column c + 1 has arrived (the
  widest filter changes six samples on each side and reads seven). Four filter datapaths side by side, one per sample
  row of the 4x4 edge unit: one edge unit a clock.
- **Horizontal edges**: block column c - 2, one edge unit a clock on four more datapaths (one per sample column),
  reading and writing the window and, for the band's top edge, the 8 context lines.
- **Context lines**: one small memory of 8 lines of the frame width per plane. A column's context is read when the
  column enters the window; the column's last 8 lines are written back when it leaves.
- **Output**: a block column leaves the window three columns after it entered, shifted up by 8 lines, 16 samples a
  clock, to the CDEF stage.
- **Edge parameters** (apply, filter size, limit, blimit, thresh): one mode-info read per 4x4 block, a clock or more
  ahead of the filters. The left neighbour's values are the previous column's, kept in a 16-entry register file; the
  above neighbour's are the previous entry of the column, or for the top block a line of saved values from the band
  above. Both passes use the same values, so they are computed once and carried with the block.

Arithmetic for 4K: a 64-line superblock row is 960 block columns x 16 blocks = 15,360 clocks of luma; 34 rows a frame
and 60 frames a second is 31 M clocks a second, about a third of a 100 MHz clock. The chroma planes run beside luma
on their own, smaller datapaths.

Built so far (branch `filter-chain`, simulation only, not in the decoder):
- `rtl/lf_filt.sv`: the sample filter for one line across an edge, combinational, fuzzed against `lf_model` (30,000
  lines, every one of the six outcomes at least 200 times; three deliberate faults each fail).
- `rtl/lf_stream.sv`: the stage as described above for one plane, a band at a time, with eight `lf_filt`. Its unit test
  (`tb/test_lf_stream.py`, suite `lf_stream_unit`) feeds whole frames band by band with the model's edge strengths,
  for superblock rows of 64 and 128 lines, with gaps in the input, and requires every sample of the plane to come out
  once and equal to `lf_model`: 8 generated frames (48 plane runs, 57,600 blocks) and frame 0 of four real streams
  are identical; four deliberate faults each fail. One of them (the last column's vertical edge left out) passed the
  first seed list and the real frames, where no sample depends on that edge: seeds where samples do were added.
- `rtl/lf_edge.sv`: the edge decision (filtered or not, size, strength) for one position and pass, combinational,
  lf_top's logic lifted out. Checked against `lf_model` on every position of generated frames, of three Argon frames
  where the level falls back to the neighbour's, and of six inter frames dumped from the `dec_inter_lr` suite
  (`tb/lf_edge_vec.py`, `tb/vectors/lf_edge_inter`); five deliberate faults each fail in one of the three suites.
- `rtl/lf_feed.sv`: walks a band's blocks, reads mode info and transform sizes from `mi_store` and the block from
  the row store one clock ahead, keeps the left and top neighbours, and hands `lf_stream` the block with both
  strengths. With it the stage needs nothing but what the decoder has. `rtl/lf_stage_tb_top.sv` + `tb/test_lf_stage.py`
  (suite `lf_stage_unit`): 6 generated frames, 36 plane runs, identical; three deliberate faults in the neighbour
  and address logic each fail. 1.7 clocks a block on these small frames, start and end of each band included.
- Pipelined since the first synthesis (out of context, 10 ns clock, K26 part, no placement): the single-step version
  had a 14.3 ns path of 47 logic levels and its column buffers came out as 30,720 flip-flops. Now the column buffers
  are five distributed RAMs with rotating roles (`rtl/lf_cbuf.sv`), `lf_filt` has a two-stage form (`PIPE`), pass A
  takes three clocks and pass B decides an edge a clock before applying it. Same suites identical, plus
  `lf_stream_short` / `lf_stage_short` (a last band shorter than the pipeline). Synthesis: **4.3 ns of slack at 10 ns,
  21 levels** on the worst path (filter outputs into a column buffer), 4,763 registers, **33,375 LUTs for one plane**.
  That is a synthesis estimate, before placement and routing. 1.5 clocks a block on the small test frames.
- `lf_feed` hands the block over through a register with its strengths; the whole stage synthesised together
  (`lf_stage_tb_top`) has 2.2 ns of slack at 10 ns, its worst path now the edge decision in `lf_feed` (25 levels).
- A chroma-only build (`LUMA = 0`: no 8- or 14-tap filter; suites `lf_filt_chroma`, `lf_stream_chroma`) is
  15,404 LUTs with 4.5 ns of slack. One luma and two chroma stages: about 64,000 LUTs, against 19,000 for today's
  whole deblocking, super-res and mode-info block.
- Not done: back-pressure on the output, the stage on inter frames end to end (the edge decision alone is), a
  picture wider than the test frames, overlapping pass B's start with pass A (the 0.5 clock a block).

Open points for this stage:
- the row store's word: a 4x4 block (16 samples) read at once, written by reconstruction four samples at a time;
- 128-line superblocks double the window height (or the band is taken as two 64-line halves: the stage model can
  already be fed 64-line bands of a 128 superblock stream, and that case is identical too);
- 4:2:2 and 4:4:4 chroma heights.

## CDEF stage: built so far and the plan (2026-11-13, branch `filter-chain`)

Built (simulation only), each lifted from `cdef_top` and fuzzed against `cdef_model`, each with deliberate faults
that fail:
- `rtl/cdef_row.sv`: the sample filter for N samples of a line (8 lanes: a line of an 8x8 luma block a clock; 4: a
  4:2:0 chroma line), four clocks. Suites `cdef_row_unit`, `cdef_row_4`.
- `rtl/cdef_dir.sv`: the direction search of an 8x8 luma block, four clocks, a block a clock. Suite `cdef_dir_unit`.
- `rtl/cdef_str.sv`: direction, strengths, the constrain() shifts and the tap-weight bit of a plane's block from the
  frame's strengths, cdef_idx, the luma direction and variance. Suite `cdef_str_unit`.

Plan for the stage around them (not built):
- It takes the deblock stage's output as it comes: 4x4 blocks column by column, the band shifted up by 8 lines.
  That shift is one 8x8 block row, so the blocks arrive aligned to CDEF's 8x8 grid.
- Five rotating column buffers as in `lf_stream`, each a column of the band plus the lines carried from the band
  above. The 8x8 block column C needs the 4x4 columns 2C - 1 .. 2C + 2 (12 luma samples wide, as the model's column
  window): it is filtered top to bottom when column 2C + 2 has arrived.
- A block row needs two lines below it, which the deblock stage sends with the next band: the CDEF stage runs one
  more 8x8 row behind and carries the last 8x8 row plus two lines of each column (the model's 18 luma lines).
- Per 8x8 block: skip flags and cdef_idx from `mi_store`, `cdef_dir` on the block, `cdef_str`, then its eight lines
  through `cdef_row`, one a clock; a block that is not filtered is passed through.
- Chroma: its own instances fed by the chroma deblock stages; the luma direction and the filter decision of every
  8x8 block go into a small memory that the chroma stages read (they wait for luma).
- Out: 8x8 blocks (or their lines) in column order to the super-res / restoration stage, with the deblocked stripe
  boundary lines the restoration stage needs taken from the same stream.
