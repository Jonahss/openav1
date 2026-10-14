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

Two URAM copies of the picture, one per read port, both receiving every write:

- **Word = one aligned group of four samples**, 4 x 18 bits = 72 bits, the URAM width. A sample sits in the low
  12 bits of its 18-bit lane; the 18-bit lane is two URAM bytes, so URAM byte-write enables give the per-lane
  write enables the interface needs. Address = plane base + y * (W/4) + x/4. The group read is one word; the
  single-sample read is a lane select on it.
- **Reads:** copy A's read port serves recon, copy B's serves LF, each exactly as today (one-cycle latency, no
  arbitration, no stall).
- **Writes:** recon's and LF's writes go through a small FIFO (depth 4) into the second port of *both* copies.
  Both stages write at most one group per clock and on average far less than one between them (recon writes a
  4-sample group every few clocks while predicting and adding; LF writes a filtered line segment), so a
  collision costs a clock of FIFO latency, not bandwidth. The hazard this creates — a read of a sample whose
  write is still in the FIFO — is closed by forwarding: a read compares its word address against the FIFO
  entries and takes the newest pending lanes. With depth 4 that is four 20-bit comparators per read port.
- **Host port:** the testbench's read port and the future PS-side read share copy A's write port when idle
  (writes are sparse) or get a third copy if that ever matters.

Size at PW = 12: a 1024 x 512 luma plane plus two 4:2:0 chroma planes is 6.3 Mbit per copy, 22 URAM288 each,
44 for two: more than the K26's 64 minus the 26 the decoder already uses (CDF store, mode info). Two ways down,
both fine for a first bitstream:

- 512 x 512 pictures (FBX = FBY = 9): 3.1 Mbit, 11 URAM per copy, 22 for two.
- 8-bit storage for 8-bit streams (two bytes per lane still, samples in 8 of them): nothing saved per lane
  because the lane is the URAM byte pair; so this is not a lever. Halving the picture is.

So: **core build, 512 x 512, two URAM copies, merged write FIFO with forwarding.** That is the first picture
on the board.

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
