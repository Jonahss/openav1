# The memory twin

The card's memory path in simulation: the DDR4 controller exactly as the image build configured it, Xilinx's DDR4
model behind it, and in front of it either nothing, the image's own memory interconnect (its post-synthesis simulation
netlist) or `rf_ddr_bridge`. A traffic player issues read and write bursts from text files and reports what the path
did with them. It runs in xsim; one run of 2,000 reads takes four to eight minutes, calibration included.

It exists because the card showed reference fetch waiting 1.51 M of a 4K frame's 2.71 M clocks for its read address
to be taken (image `u200-k4lnx80-0ca8453`, 2026-11-27, `K60-108`), and nothing in the decoder's own simulation could
say why: its memory model answers at once.

`bin/memtwin.sh` runs everything (`memtwin.sh` alone prints its verbs). `setup` needs a finished image build
directory: the controller's `.xci`, and the interconnect's simulation netlist and stub, are taken from there.

## What it measured (2026-11-27)

Reads are patch rows: one two-beat burst per row, 64 bytes a beat, 2,000 rows, issued back to back.

| path | stride 3840 | stride 3904 | rows 128 bytes apart |
|---|---|---|---|
| controller alone, controller clocks (300 MHz) a read | 10.0 | 3.7 | 2.1 |
| through the image's interconnect, decoder clocks (80 MHz) a read | 4.02 | 4.02 | 4.02 |
| through `rf_ddr_bridge`, decoder clocks a read | 2.66 | 2.08 | 2.08 |

- The interconnect takes one read address every 4 decoder clocks, at 50, 80 and 100 MHz alike, whatever lies behind
  it. The card agrees: 3.86 clocks a read while it asks, and no change when the picture stride was changed.
- A handshake trace (`memtwin.sh trace`) puts it in the slave-side converter (`si_converter`): it takes an address,
  passes it on the next clock and is not ready for two more. Its source is encrypted.
- Declaring 32 reads outstanding, no narrow bursts or a burst limit of 16 on the port changes nothing (`mksc`,
  `scx`: the interconnect made again from the build's recorded parameters reproduces the image's netlist to the
  clock, with or without those changes).
- Bursts of 16 beats pass at about a beat a clock: the cost is per address, not per beat.
- Behind the bridge the player itself is the limit at 2.08 (it offers an address every second clock); at stride 3840
  the controller is (every row falls in the same bank group: the address map puts the group in address bits 7:6).

## The bridge's check

`memtwin.sh check`: a second player in XDMA's place writes two regions through the interconnect and the bridge, every
beat carrying its own address, then reads one back while the first player reads rows of the other through the
bridge; both compare every beat. Then the same with a mutant bridge that sends one burst the wrong way, which must be
reported. `MEMTWIN_CHECK PASS` needs both.

## Files

- `mk_example.tcl`, `smoke.tcl`: the controller's example design from an image's configuration; its stock test.
- `twin_player.sv`: the player and the twin's clocks. `patch_top.py`: puts it into the example design (three modes).
- `run.tcl`, `trace.tcl`, `probe_br.tcl`: runs; the handshake trace; a stepped run that prints the second player's state.
- `mk_sc.tcl`: the interconnect made again as a block design of its own, with interface parameters overridden.
- `synth_patterns.py`, `bridge_patterns.py`: the traffic files.
- `ooc_bridge.tcl`: out-of-context synthesis of the bridge. `mk_bridge_v.py`: writes `fpga/u200/rf_ddr_bridge_v.v`.

The decoder's own simulations got one thing from this: `TS_AXI_ARGAP=4` makes `tb/axi_mem.py` take a reference read
address every 4 clocks, as the interconnect does (`bin/whatif_frame.sh ... TS_AXI_ARGAP=4`).
