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
