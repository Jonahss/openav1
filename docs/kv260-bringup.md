# KV260 bring-up: the core decoder as a bitstream

Status (2026-10-14): **the core decoder decodes on the board, bit-exact.** First bitstream (50 MHz decoder /
100 MHz memory, 99.5k LUT, 40 URAM, WNS +0.7 ns): seed 1 and seeds 2-7 (4:2:0 / 4:4:4 / mono, 8/10/12-bit,
a four-tile frame, loop filter on) replayed through the register file with 0 mismatches over 126,976 pixels.
Two board-only lessons, below.

## What is in the bitstream

`fpga/kv260/build.sh` builds a block design (`build.tcl`) for the Kria K26 (xck26-sfvc784-2LV-c):

- `zynq_ultra_ps_e`: only HPM0 FPD (32-bit AXI master), `pl_clk0` and `pl_resetn0`. The PS is configured by the
  boot firmware; the bitstream is loaded under a running Linux, so nothing else of the PS matters here.
- `clk_wiz` (MMCM) from `pl_clk0`: `clk` (the decoder clock, `KV260_MHZ`, 50 by default) and `clk2x`, aligned.
  Without the KV260 board files the PS IP computes `pl_clk0` as 96.97 MHz and the MMCM is analysed from that;
  the overlay asks Linux for 100 MHz, so the real clocks run ~3 % above the analysed ones.
- `phase_gen` (`rtl/phase_gen.sv`): the frame memory's phase flag, a clk2x toggle aligned by the clk reset.
- `dec_host` (`rtl/dec_host.sv`, through the Verilog shell `dec_host_v.v`): the AXI-Lite register file around
  `dec_top` in the core configuration (FBX = FBY = 9, two transform lanes, `FRAME_URAM = 1`; CDEF and LR are null
  stubs, super-resolution is not run). At 0xA000_0000, 4 KB. The register map is in the header of `dec_host.sv`.
- `smartconnect` between the two, `proc_sys_reset` for the resets.

Constraints: `sys.xdc` (clk <-> clk2x multicycles by the MMCM output pins) and `pre_opt.tcl` (the URAM -> clk2x
multicycle and the phase generator's one-period alignment path; both need post-synthesis cells, so they run as the
`opt_design` pre-hook). Same relationships as `synth/vivado/synth.tcl` + `impl.tcl` use out of context.

## Driving it

Software talks to the register file as `tb/test_dec_host.py` does: headers (`hdr_t`, `rec_hdr_t`, `lf_hdr_t`
packed little-endian into the 16-word windows), the CDF default rows (8 data words + the address write that
strobes `def_we`), `START.tile`, the tile bytes into the FIFO (`DATA4`, free count in `STATUS[31:16]`), `CTRL.eos`,
poll `STATUS.tile_done_lvl`; then `START.lf` and poll `STATUS.lf_done`; then the picture through `HADDR` / `HDATA`.

The simplest board run needs no model on the board: `TS_TRANSCRIPT=<file> python tb/runner.py dec_host`
(with `FBX=9 FBY=9 ITX_LANES=2 FRAME_URAM=1 TS_STAGE=lf`) records every register write, every poll and the expected
value of every pixel read, and `tools/kv260/play.py <file>` replays it over `/dev/mem` and reports mismatches.

## Board lessons (2026-10-14)

- **The PS port width is the boot firmware's, not the bitstream's.** The K26 firmware's psu_init sets
  `FPD_SLCR.AFI_FS` (0xFD615000, bits [9:8] HPM0, [11:10] HPM1) to 128 bits. A PL built for a 32-bit HPM0 only
  sees byte lane 0: registers at offsets 4, 8, 12 mod 16 read 0 and ignore writes. The design now asks for
  128 bits (`PSU__MAXIGP0__DATA_WIDTH 128`; the smartconnect narrows to the AXI-Lite slave). Runtime check /
  workaround for an old bitstream: read the register; clearing bits [9:8] switches HPM0 to 32 bits until reboot.
- **One register write must be one bus transfer.** Through `/dev/mem`, Python's `struct.pack_into` issues three
  stores per 32-bit word and a slice assignment two (measured with the FIFO's free count); only
  `ctypes.c_uint32(...).value = v` is a single store. The split writes each pushed a junk word into the tile FIFO
  and the entropy decoder diverged gracefully (small errors first, garbage later; identical at a slower clock,
  which is how it was told from a timing problem). `play.py` uses ctypes; `dec_host` drops a push whose write
  strobe is not the whole word and sets STATUS bit 14.

## Loading on the board

`build.sh` leaves `fw/openav1/` with `openav1.bit.bin` (bootgen), `openav1.dtbo` (from `openav1.dts`: programs the
PL, releases the PL resets, enables `pl_clk0` at 100 MHz) and `shell.json`. On the KV260 (Ubuntu, `xmutil`):

```
sudo mkdir -p /lib/firmware/xilinx/openav1 && sudo cp fw/openav1/* /lib/firmware/xilinx/openav1/
sudo xmutil unloadapp; sudo xmutil loadapp openav1
sudo tools/kv260/play.py host-seed1.txt
```

With the first bitstream (32-bit port), first `devmem`-style clear bits [9:8] of 0xFD615000 (see above).

(`xmutil listapps` shows it; `dmesg` shows the FPGA manager load. Without `xmutil`: `fpgautil -b openav1.bit.bin
-o openav1.dtbo`.)
