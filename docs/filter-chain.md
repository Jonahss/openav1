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

- `rtl/cdef_stream.sv`: the stage for the luma plane, first cut (the plan below, with the block's steps in sequence:
  about 18 clocks an 8x8 block). Its test (`tb/test_cdef_stream.py`) feeds the deblocked picture in `lf_stream`'s
  order, answers the skip and cdef_idx reads, and requires every luma sample once and equal to `cdef_model`: frame 0
  of four real streams (suite `cdef_stream_unit`) and six inter frames dumped by `dec_inter_lr` (`tb/cdef_vec.py`,
  `tb/vectors/cdef_stream_inter`, suite `cdef_stream_inter`: 837 skipped blocks, blocks without a cdef_idx, blocks
  where the fourth skip flag decides). Five deliberate faults fail; the two about unfiltered blocks only on the inter
  frames.
- The same module takes a chroma plane (`luma = 0`): its CDEF block is one 4x4 block wide and / or high by the
  subsampling, the direction comes from a port (the luma stage's result for the 8x8 block), and it steps by its own
  block size. `cdef_stream_unit` drives all three planes of the four real frames (16 chroma runs, 4:2:0): identical;
  three chroma faults (direction ignored, the band start, the skip address) fail.
- Synthesis of the luma first cut (out of context, 10 ns): 0.9 ns of slack (the direction search's cost sums),
  38,647 LUTs (direction search 10,218, line filter 16,951).
- Look-ahead: inside a block column the next block's rows, direction search, skip flags and parameters are
  prepared during the current block's eight lines, so luma blocks follow each other every eight clocks. Measured
  on the six inter frames with the input never held back: 10.6 clocks an 8x8 block including each column's start
  (the first block in sequence, the carried rows read and saved): about 6 luma samples a clock on these small
  pictures, where a band is only 8 or 16 blocks high. `cdef_stream_fmt` adds 4:4:4 and 4:2:2 frames.
- A test bug worth remembering: the test's stand-in for `mi_store` answered in the same clock instead of the next.
  The first cut read all four skip flags anyway and passed; the look-ahead read one twice, and only the dumped inter
  frames showed it. The stand-in now has the memory's latency.
- Chroma from luma's record (2026-11-14): whether a block is filtered, its cdef_idx and its direction are those of
  the luma 8x8 block over it in every format. The luma stage now gives all three as it decides them (`dw_info`), and
  a chroma stage reads that one record a block (`yd`) instead of four skip flags, the cdef_idx and the direction:
  it reads nothing from `mi_store`. With one read to wait for, blocks one 4x4 row high (vertically subsampled
  chroma) got their own look-ahead: the next block's row and record are fetched during the current block's four
  lines, so they follow each other every four clocks (they took about 14 in sequence). The tests give a chroma
  stage noise on the `mi_store` ports. 15 faults in the new parts: 13 fail (the one-row look-ahead's six only on
  4:2:0 frames, the record's three only in `fc_top`, where chroma reads what luma wrote), one was not a valid
  fault (it did not compile), and one passes and is a simplification: a chroma column's first block could take
  its record a clock earlier than it does.
  The whole chain on frame 0 of the six real streams, three planes at once: 4.4 samples a clock before, 5.4
  after; luma CDEF at its 2.7 clocks a 4x4 block is now what sets the pace.
- Not done: two lines a clock (the planned rate), the per-column overhead, chroma on inter frames (the dumped
  frames hold luma only).

Plan for the stage around them (the luma part is built as described):
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

## Restoration stage: built so far (2026-11-14, simulation only)

- `rtl/lr_wiener_h.sv`, `rtl/lr_wiener_v.sv` (`lr_wiener_pkg.sv`): the Wiener filter's horizontal and vertical
  passes for N samples of a line, two clocks each, lifted from `lr_top`. Suites `lr_wiener_unit` (8 lanes),
  `lr_wiener_4`.
- `rtl/lr_stream.sv`: the stage for one plane, all three unit types (self-guided units below). It takes the CDEF stage's
  line segments as they come (segment column by segment column, top to bottom):
  - four rotating column buffers (block RAM); segment column C is filtered when column C + 1 has arrived, since a
    line's horizontal pass reads three samples either side;
  - the column's lines go through `lr_wiener_h` one a clock; the last six intermediate lines sit in a shift
    register and `lr_wiener_v` gives the line three above the newest;
  - the stripe rule becomes pushes of extra lines: before a stripe's first line the deblocked lines two and one
    above it (the shift register starts afresh), after its last line the two deblocked lines below; the plane's
    first and last lines are repeated three times. The four deblocked lines around a stripe boundary come from the
    boundary line store (below) through a port;
  - the unit record (type, coefficients) is read from `mi_store`'s port at every stripe start;
  - between bands it carries, per column, the six newest intermediate lines (16 bits a sample hold them at every
    bit depth) and the three newest source lines: 1056 bits a segment column;
  - samples and lines of the coded area outside the plane leave unchanged.
- Its test (`tb/test_lr_stream.py`, suite `lr_stream_unit`) makes up pictures (CDEF and deblocked planes, any width
  and height, 4:2:0 / 4:4:4 / 4:2:2 / mono, 8 / 10 / 12 bit) and unit maps, feeds them band by band for 64 and 128
  line superblocks, and requires every sample of the coded area once and equal to `lr_model`. 17 deliberate faults
  fail. One (the clamp of the lines below a stripe when the plane ends one line below the boundary) passed at first:
  no picture had such a height; the test now draws heights around the stripe boundaries and counts them.
- Rate, measured on 960-wide 4:2:0 pictures with the input never held back: 1.3 clocks a line over the three
  planes (a luma segment line is 8 samples, a chroma one 4). A stripe boundary costs eight clocks a column (two for
  the unit read, six extra lines), the copy of the boundary lines six on the arrival side.
- Synthesis (out of context, 10 ns, 1024 segment columns): 3.4 ns of slack, 3,106 LUTs, 2,005 registers, 126 DSP
  blocks, 35.5 block RAM tiles (30 of them the carried lines; half would do for 4K luma's 480 columns). The worst
  path is the carried lines' memory into the vertical pass's multipliers.
- Self-guided units, correct but slow (`rtl/lr_sgr_ab.sv`, `rtl/lr_sgr_f.sv`, the arithmetic lifted from `lr_top`):
  the same pushes of lines, but a line's 14 samples go to a sequential engine and the column waits for it. The
  engine keeps the last six lines, walks the 14 sample columns (the column sums of both radii a clock), slides the
  box sums over them and sends the ten box positions of the line through two `lr_sgr_ab` (a2 and b2, one per
  pass); `lr_sgr_f` then gives the line three above the newest from three rows of a2 / b2. Between bands a
  self-guided column carries its six newest source lines in place of the Wiener intermediates and pushes them
  again at the column's start to rebuild the rows. Suite `lr_sgr_ab_unit` (20,000 fuzzed boxes against the model's
  arithmetic); `lr_stream_unit` now draws all three unit types (124 self-guided units in its 40 pictures).
  - Rate, same 960-wide pictures: 30.8 clocks a line with every unit self-guided, against 1.37 with every unit
    Wiener. That is about 0.2 samples a clock: fine for being correct, far from 4K60. No YouTube sample uses
    restoration at all, and the encoders that use it at 4K choose Wiener or (rav1e) a mix
    (`docs/4k60-budget.md`), so the fast version waits until a stream needs it. The way there is known: the box
    sums are already running sums, so ten box positions a clock instead of one is more multipliers, not a new
    design.
  - Synthesis with the self-guided path (out of context, 10 ns): 3.4 ns of slack (unchanged, still the carried
    lines into the vertical pass), 11,339 LUTs, 6,453 registers, 183 DSP blocks, 42 block RAM tiles.
  - 50 deliberate faults across the stage and the two arithmetic modules: 44 fail by a sample mismatch. The six
    that pass change nothing a legal picture can show: the parity of a line that never makes the engine give a line
    (three of them), the weight-0 pass standing in as the source when the bitstream forces that weight to 0 (two),
    and a2 = 1 for z = 0, which the table's default gives as well.
- `rtl/lr_bstore.sv`: the boundary line store. It watches the deblock stage's finished 4x4 blocks go by and keeps,
  per 4x4 column, the two lines above each stripe boundary and the two from it on (block rows B/4 - 1 and B/4),
  for four boundaries at a time: in a 128-line band restoration reads two boundaries while the deblock stage of the
  same band is already writing the next one, so three are alive at once. 768 bits a 4x4 column; as declared for
  1024 columns 1.18 Mbit (about 33 block RAM tiles), because the memory of a segment's first four samples is sized
  for a subsampled plane, where every 4x4 column starts a segment (not synthesised yet). The deblock stage must not run a whole band ahead of restoration's reads.
  Suite `lr_stage_unit`: the stage and the store together (`lr_stage_tb_top`), the test writing the blocks as
  `lf_stream` sends them (two context rows first, the band's last two rows held back) and filling the store with
  noise before each plane so that a line it fails to take cannot pass as an old copy. 16 deliberate faults in
  the store: 15 fail by a sample mismatch (among them a store of two boundaries instead of four, on a 128-line
  band); the one that passes (the block row taken as unsigned) lands on the same boundary slot, since 128 rows are
  a multiple of four boundaries.
- Not done: super-resolution in front of the stage, output back-pressure, the fast self-guided path.

## The stages joined (2026-11-14, simulation only)

- `rtl/fc_plane.sv`: one plane of the chain. `lf_feed` + `lf_stream` take the band's blocks from the row store,
  `cdef_stream` takes the deblocked blocks, `lr_bstore` copies the stripe boundary lines out of the same stream, and
  `lr_stream` takes CDEF's lines. One `band_start` for all of it; the band is done when restoration is and the
  stages before it are idle.
  - All three stages always run. A filter that is off for the frame or the plane passes its samples through
    (`lf_on`, `cdef_on`, the plane's restoration type), so a plane's output always has the same shape and shift.
  - CDEF's output cannot wait line by line and a self-guided line takes restoration about 30 clocks, so a FIFO of
    256 lines (`rtl/fc_lfifo.sv`) sits between them and CDEF starts a block column only when the FIFO has room for
    all of that column's lines (`col_hold`). A dropped line would raise `err_ovf`; none has.
- `rtl/fc_top.sv`: the planes of a picture, each an `fc_plane`, working on the same band at the same time. Luma's
  CDEF decisions (filtered or not, cdef_idx, direction) go into a small memory (64 rows of 8x8 blocks, a copy per
  chroma plane) as luma's stage makes them; a chroma plane holds a block column until luma has finished the columns over the same picture area.
  Monochrome leaves the chroma planes idle. Each plane has its own read ports into the row store and the mode
  info: the stores behind them must answer three readers at once (the test wrapper uses three `mi_store`).
- Tests against the frame-at-once models (`lf_model`, then `cdef_model`, then `lr_model`), every sample of the coded
  area exactly once, both band sizes, with nothing supplied but the headers, `mi_store`'s contents and each band's
  pre-filter blocks: `tb/test_fc_plane.py` (suites `fc_plane_unit`, `fc_plane_odd`, `fc_plane_real`; a chroma
  plane's luma directions answered by the test; every frame once more with deblocking and CDEF switched off by the
  switches alone) and `tb/test_fc_top.py` (`fc_top_unit`, `fc_top_odd`, `fc_top_real`; nothing answered by the
  test). Identical on 62 frames each in a first wide run: generated frames at 128x96, 200x136 and 72x264 in all
  four formats and three bit depths, and frame 0 of six real streams. Frames with super-resolution are skipped.
- Faults: 25 in `fc_plane`, the FIFO and CDEF's column hold. 17 failed on the first set of frames; the eight that
  passed showed what the frames lacked, and the tests gained it: the pass with the filters switched off (the two
  deblock switches and the CDEF switch then fail), odd picture sizes with 4:2:0 chroma restoration (plane width and
  height rounding, the unit-column count, the plane's own unit size and the boundary store's row then fail). One
  still passes and is equivalent as the stages are built: reporting the band done on restoration's word alone,
  without waiting for the stages before it, which are always idle by then. 13 in `fc_top`: all fail (chroma's hold
  removed or short by a column, the direction memory's row, column, second copy and depth, the chroma plane's
  columns and rows, monochrome, the band's end, the luma build, the per-plane deblock switch; the last two needed
  a monochrome frame and a pass with chroma's deblocking alone switched off).
- Synthesis of `fc_top` (out of context, 10 ns, 4096 columns): 0.9 ns of slack, 246,512 LUTs, 77,494 registers, 640
  DSP blocks, 255 block RAM tiles; the luma plane is 97,534 LUTs of it (CDEF 41,783, restoration 13,328). That is
  large, and the stages were written to be right first. Per plane: deblock 31,492 (luma) / 15,470 (the chroma-only
  build), CDEF 41,783 / 31,494 (synthesis drops the direction search from the chroma planes, whose `luma` input is
  constant), restoration 13,328, the boundary store about 9,000, of which 7,168 are LUT RAM. That last figure is an
  artefact of the part this script synthesises for (the K26, 144 block RAM tiles): the chain asks for more than the
  part has, and synthesis moved two of the store's four memories and the line FIFOs to LUT RAM (alone the store is
  33 block RAM tiles and 64 LUTs). About 40,000 of the 246,512 LUTs are such LUT RAM. CDEF's window multiplexers
  are the first place to look for real savings.
- Rate as it stands, a frame's planes run at the same time, generated 128x96 frames with self-guided units: about
  1 sample a clock. Per plane on the real streams: about 7 clocks a 4x4 block over the three planes one after
  another, which is luma at CDEF's 2.7 clocks a block and 4:2:0 chroma at about 14 (CDEF blocks one 4x4 row high
  took `cdef_stream`'s sequential path; see the CDEF section for the look-ahead that replaced it). The stages'
  planned rate is one block a clock.
- The output can be held (`out_hold`, per plane): restoration then pushes no further line into its filters, on the
  stall it already had for a self-guided line. Lines already inside still come out: 5 at most in the tests, which
  hold for random stretches and fail above 16 (`OUT_SLACK`). A first attempt held restoration's input instead and
  saw 56 lines follow, a whole buffered column. Three faults (the push not stopped, the sequencer not stopped, the
  hold not connected) fail.
- Not done: super-resolution between CDEF and restoration, the stores shared by three readers in the decoder (mode
  info for the deblock stages, the unit records), the row store, the writer to DDR, and the switch in `dec_top`.
