# KV260 bring-up: the core decoder as a bitstream

Status (2026-10-14): host interface verified in simulation (seeds through the register map, pictures identical
with the loop filter); first bitstream build in progress; nothing has run on the board yet.

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

## Loading on the board

`build.sh` leaves `fw/openav1/` with `openav1.bit.bin` (bootgen), `openav1.dtbo` (from `openav1.dts`: programs the
PL, releases the PL resets, enables `pl_clk0` at 100 MHz) and `shell.json`. On the KV260 (Ubuntu, `xmutil`):

```
sudo mkdir -p /lib/firmware/xilinx/openav1 && sudo cp fw/openav1/* /lib/firmware/xilinx/openav1/
sudo xmutil unloadapp; sudo xmutil loadapp openav1
sudo tools/kv260/play.py host-seed1.txt
```

(`xmutil listapps` shows it; `dmesg` shows the FPGA manager load. Without `xmutil`: `fpgautil -b openav1.bit.bin
-o openav1.dtbo`.)
