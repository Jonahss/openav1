# Frame memory for the FPGA builds

Written 2026-10-13 after the first place-and-route of the core build (`docs/itx-level-engine.md` has the
transform side). Status: design; no RTL yet.

## What the decoder asks of it

`rtl/frame_mem.sv` is a simulation model: three planes of `(1 << FBY) x (1 << FBX)` samples with NRD read and
NWR write ports, one-cycle read latency, every port independent. Each read returns the sample at x *and* the
aligned group of four samples containing x; each write carries four lanes with lane enables (a single-sample
write is one enabled lane). `dec_top` instantiates three of them:

| buffer | readers | writers |
|---|---|---|
| fb0, pre-filter / deblocked | recon, LF, CDEF, LR source 0, SR | recon, LF, SR |
| fb1, CdefFrame | LR source 1, SR | CDEF, SR |
| fb2, LrFrame | host only | LR |

No port has back-pressure: a stage issues a read and expects the data the next clock. That is the constraint
every hardware version has to honour, or the stages have to learn to stall.

For the **core build** (CDEF, LR and SR parameterised out, the staging Jonah approved on 2026-10-13) only fb0
is live, with two readers (recon, LF) and two writers (recon, LF). fb1 and fb2 are unused.

## On-chip version (first bitstream)

One URAM copy of the picture, **double-pumped**: the URAM runs at 2x the decoder clock (200 MHz, URAM288 is
rated well above that) and its two ports serve the two readers and the two writers in alternate half-cycles.
No arbitration, no FIFO, no forwarding, and every port keeps its one-cycle latency at the decoder clock.

- **Word = one aligned group of four samples**, 4 x 18 bits = 72 bits, the URAM width. A sample sits in the
  low 12 bits of its 18-bit lane; the lane is two URAM bytes, so URAM byte-write enables give the per-lane write
  enables the interface needs. Address = plane base + y * (W/4) + x/4. The group read is one word; the
  single-sample read is a lane select on it.
- **Timing:** read 0 is issued on the first 2x edge of a decoder cycle, read 1 on the second; the URAM's output
  register delivers each a 2x cycle later, and a holding register presents both at the next decoder edge. Writes
  likewise on the other port. The decoder-side registers are in the 100 MHz domain; the 2x domain is phase-
  aligned to it from the same MMCM, so the crossing is a known multicycle relationship, not an asynchronous one.
- **Host port:** the testbench's / PS-side read takes a write slot when the writer is idle (writes are sparse),
  or a third half-cycle slot at 3x if ever needed.

Size at PW = 12 with 18-bit lanes (a 2026-10-13 miscount said 11 URAM for 512 x 512; it is 24):

| picture | samples (4:2:0) | words | URAM288 |
|---|---|---|---|
| 256 x 256 | 98k | 24.6k | 6 |
| 512 x 512 | 393k | 98.3k | 24 |
| 1024 x 512 | 786k | 196.6k | 48 |

The core build leaves 38 URAM free, so **512 x 512 is the first-bitstream geometry** (FBX = FBY = 9), one
copy, double-pumped. 1024 x 512 needs the DDR version. Two copies with a merged write FIFO (the earlier idea)
would need 48 URAM at 512 x 512 and forwarding logic; the double-pump is both smaller and exact.

The frame memory needs a second clock input (`clk2x`); the simulation model ignores it.

Implemented (`rtl/frame_mem_uram.sv`, 2026-10-13): Vivado infers UltraRAM only for full-word no-change writes
(read-first and per-byte enables are both "invalid write mode"), so synthesis instantiates the URAM288 through
`xpm_memory_tdpram` with 8-bit byte enables (lanes are byte pairs, 16 bits, sample in the low 12; byte 8 of
the 72-bit word is unused) and simulation uses a behavioural model. At 512 x 512: 24 URAM, 265 LUT, 246 FF.
Schedule (2026-10-14): both ports read at the clk-aligned clk2x edge (the edge at which the model samples
a request), the write requests are captured into clk2x registers at that edge and applied at the mid-cycle
edge. So every decoder-side path is a full clk cycle and the read data is the URAM output register itself.
Constraints: `synth/vivado/synth.tcl` writes the clk <-> clk2x multicycles (2 cycles each way; the
fast -> slow one with `-start`), and `impl.tcl` adds URAM -> clk2x as a 2-cycle path once the primitives
exist (the read data -> recon add -> write-request capture round trip is a clk cycle, not a clk2x one). In the
core build at 512 x 512 the memory adds no path worse than the decoder's own: the URAM output cascade
(8 deep: 1.8 + 7 x 0.26 ns) to the ipred LUTRAMs is the only one near the edge (-0.4 ns at 100 MHz). The fuzz
(`tb/test_frame_mem_uram.py`) checks the model's contract on random traffic; the gate-level replay checks the
XPM build against it (block level and the whole core decoder, `tb/gate/replay_frame_mem_uram.sv`).

## DDR version (full-size pictures, all stages)

Same port interface, pictures in the PS DDR through the AXI HP ports. The stages' access patterns are all
row-local (recon: current block and the row above; LF: the rows at an edge; CDEF: a 2-line neighbourhood;
LR: a stripe), so each port gets a line buffer of a few rows of its plane in BRAM, filled and written back by a
DMA engine per port that runs ahead by superblock row. The forwarding problem becomes a coherence problem
between ports that share a buffer (recon writes what LF reads one superblock row later; the sequencer already
orders the stages by superblock row, which is what makes the line buffers sufficient). This is the real
design; the on-chip version is its test oracle: same interface, same decoder, pictures small enough to keep
whole.

## Verification

The simulation model stays the oracle: the hardware frame memory replays the whole-decoder vectors (the
`tb/gate` flow records the frame-memory ports as part of dec_top's ports) and the decoder's regressions run
against both models through a parameter. Gate-level replay of the memory itself: record its ports in a
dec_top run, replay against the netlist with the URAM primitives (xsim has them).

## Window version: full-size pictures through DDR (design 2026-10-14 evening)

The core build is intra-only without intra block copy, so no stage ever reads far from where it is working:
reconstruction reads the row above and the column to the left of the block, the deblocking filter reads and
writes a few lines either side of the superblock-row boundary. In the pipelined frame mode (`pipe_en`) the
filter trails reconstruction by one superblock row, so at any moment the picture rows in play are three: the
row being reconstructed (r), the row being filtered (r-1), and the row whose last lines that filtering still
changes (r-2). Everything older is final.

So the frame memory becomes a **window**: the same double-pumped UltraRAM with the same ports and the same
one-cycle latency, holding `FRAME_WIN` superblock rows (3) of the picture's full width as a ring indexed by the
superblock row modulo 3. Rows that become final are **flushed to DDR** by the host interface's AXI master (the
picture-DMA engine, now also triggered by hardware), in the same planar layout the picture DMA used; the PS
finds the finished picture in DDR exactly as before, nothing is ever read back from DDR by the decoder. The
only new control is a hold: reconstruction may not start superblock row R before row R-3 (the previous occupant
of its slot) has been flushed. `dec_top` gates the superblock event into the reconstruction stage on
`flushed_upto >= R - FRAME_WIN`; in practice a flush (a few hundred microseconds a row at 1080p) finishes long
before reconstruction gets there, so the hold is a safety net.

Row release: `lf_done` for superblock row r makes row r-1 final (LF(r) wrote into its last lines); the last
filter pass releases the last two rows. `dec_top` emits `row_final` pulses with the row index; `dec_host`
queues them and writes each row's lines (all planes) to the plane bases / strides already programmed for the
picture DMA, then reports `flush_done`, which advances `flushed_upto`. Software programs the plane registers
once per frame (before `START.frame`), sets the auto-flush bit, and after `frame_done` waits for the flush
queue to drain.

Geometry: at 4:2:0 the window is `3 x 64 x W` luma samples plus two chroma planes of `3 x 32 x W/2`, one
72-bit word per aligned group of four samples:

| width | window words | URAM288 |
|---|---|---|
| 1280 (720p) | 92k | 23 |
| 1920 (1080p) | 138k | 35 |

against 24 for the 512 x 512 full-picture memory, so 1080p fits the K26 with ~20 UltraRAMs to spare. Limits of
this version, all flagged as `unsupported` by the RTL (sticky for the frame) and refused by the driver rather than
silently wrong: 64-pixel superblocks only (128-pixel rows would need a 6-row window), 4:2:0 or monochrome,
pipelined frame mode only, **one tile column** (a tile column spans every superblock row, so with several the
window of the full width could never release a row before the next column started: the first run on a four-column
stream deadlocked on the hold), no intra block copy (it reads anywhere in the picture), and CDEF / loop restoration
stay out (they would lengthen the window by the rows they lag). Tile rows are fine. Full-picture builds
(`FRAME_WIN = 0`) are unchanged.

Two things the first simulations taught (2026-10-14 evening): the UltraRAM serves the host port only in the
filter's read slot when that port is idle, and the flush now runs while the filter works, so the memory reports
whether a host read was served (`h_ok`) and the DMA engine refetches a group that was not; and a superblock event
held by the window still has to count as the end of the previous row (once reconstruction is idle), since the
filter that releases the row the hold waits for is itself waiting for that count. Synthesis of the 1080p
configuration (`FBX = FBY = 11`, three rows): 102k LUT, 94 block RAM, 50 UltraRAM, once the per-4x4 stores that
scale with the picture (motion info, the filters' info, reconstruction's mode map) were narrowed to the rows in
play (32, 64 and 64 4x4 rows, rings by row truncation).

On the board (2026-10-17): the 1080p configuration at 50 MHz decodes the 20-frame 1080p demo stream bit-exact
against dav1d through the picture DMA into a reserved DDR region (`docs/kv260-bringup.md`). Its gate-level
replays: the whole decoder at the core configuration (`FRAME_WIN = 0`) on two recordings, and `dec_host` at
`FRAME_WIN = 3` on two window recordings, 256 x 224 with seeds 1 and 4 (monochrome and 4:2:0 8-bit, 567,588
cycles) and 128 x 512 with seeds 6 and 8 (4:2:0 12-bit and 8-bit, 823,354 cycles), both 0 mismatches (the AXI
attribute outputs compared only while their valid is high). The seeds are ones the board build accepts: a
recording with CDEF or loop restoration on shows those blocks busy in the STATUS register, which the netlist
with them nulled cannot (seed 7: 281,257 STATUS-read mismatches and not one anywhere else, including the DMA).
The first of those replays disagreed on `awsize` / `awburst` on every write address: not the netlist, the
recording.

### Timing at 80 MHz (2026-10-18)

The 80 MHz 1080p window build missed timing by 1.03 ns under Vivado's default implementation strategy (1053
failing endpoints; `Performance_ExtraTimingOpt` closed it with 38 ps to spare, which is a strategy's result rather
than the design's). The report had three path families, all fixed in RTL on the `timing80` branch:

- **Window address** (-1.03 ns, 39 logic levels, the ten worst paths): from reconstruction's transform-size register
  through the availability flags and the edge-gather limit clamps into the frame memory, whose `row % FRAME_WIN` and
  `line * words-per-row + plane base` were synthesised as a divider and adders in front of the UltraRAM address.
  Now the window line is a table over the superblock row (two LUT levels), the address is a concatenation of line
  and group with only the plane base added (its low bits are zero), and `recon_top` registers the two limits in
  `R_SETUP` (they are constant over a transform block and used only by the edge gather). Same addresses for every
  legal coordinate: the memory fuzz, the window suite and the gate-level replays are unchanged in result.
- **Intra edge filter / upsampler** (-0.56 ns, 26 levels, 70 % routing): the read addresses of the edge banks were
  computed from the step counter with clamps in the same cycle as the LUT-RAM read, the filter arithmetic and the
  write-request mux. `ipred` now computes the six indices the next cycle reads where the sequencer sets the next
  step (a lookahead register, as the pixel loop already had).
- **Palette sort** (-0.56 ns, 24 levels): the 8-round compare-exchange network of the palette colours ran in one
  cycle; it is split over two states (same network, one more cycle per palette plane).

With those three fixed the default strategy closed (+0.025 ns), but the next build of the same design missed by 1.09 ns
on the intra predictor alone: at 89 % LUT utilisation the placement moves a routing-dominated family by a nanosecond
between runs. Two more read-modify-write paths were therefore cut at the write, so that the arithmetic and the routing
to the memory are no longer in series:

- **`ipred` edge-bank writes** are registered and land one cycle after they are posted (the filter / upsample
  arithmetic behind a bank read, then the request mux and the routing to the eight copies of each bank). The
  sequencer never reads an entry the cycle after writing it except across two transitions, which take a one-cycle
  bubble (`S_EF_PRE` after the corner write, `S_UP_DONE` after the last upsample step); the edge filter already
  bypasses its own last two writes through `o0` / `o1`, the upsampler reads only positions it has not yet written.
- **`recon_top` residual add**: the sum of the group read last cycle is registered and written the cycle after
  (`R_ADD_LAST2` drains the last write before `R_FIN`). Groups are each written once and read before their own add.

Result (default strategy, 80 MHz, 1080p window): WNS +0.051 ns, no failing endpoint of 151,087, 104.8k LUT, 50
UltraRAM; on the board 1080p 20/20 and foreman 120/120 bit-exact (10.9 s / 4.2 s). The worst paths are now the
transform engine's transpose pack (`col_reg -> pk_in`, 0.05 ns), the palette delta into the CDF store's address
(0.10 ns) and the level engine's DSP cascade (0.16 ns): the design is at the device's ceiling for this clock, and
more margin means either lower utilisation or a pipeline stage in those three. Netlist replays and the board run
are the verification; the cycle counts of the recordings grow by the bubbles and the drain state (g4: 51,717 to
51,764).
 Verilator gives one VCD identifier to nets it finds equal (the write and read channels' constant size
and burst), and `vcd2vec` kept one name per identifier, so the write channel's constants were recorded as zero.
