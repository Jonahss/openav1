# Warnings: the rule, the census, the waivers

The rule (2026-11-16): every warning any stage of the pipeline prints is either fixed, or waived in
`waivers.txt` with a reason that points at a section of this file. A build or a nightly that prints a warning
with no waiver fails. Nothing is counted without being read.

Why: two bugs in one week sat in warnings that were counted, not read. A port connection used before its
declaration (Synth 8-6901, 2026-11-09) tied a hold signal to 0 through three green test rungs. A hold exception
anchored at the wrong clock (TIMING-29 in the methodology report, 2026-11-16) left 245 real hold violations
behind a clean timing summary, and two images decoded wrongly for two days.

## Stages and how each is checked

| stage | what prints | how it is checked | state |
|---|---|---|---|
| Verilator simulation builds (`tb/runner.py`) | `-Wall`, warnings are errors | three kinds switched off globally: UNUSEDSIGNAL, UNUSEDPARAM, PINCONNECTEMPTY | to do: switch them on, see "Simulator" |
| declaration order | `tools/check_decl_order.py` | hard gate in the nightly and the gate script since 2026-11-09 | clean |
| Vivado synthesis, every run | `WARNING: [Synth 8-...]` in each `runme.log` | `tools/warn_census.py --gate` | census below, no waivers yet |
| Vivado implementation | opt / place / route messages, `impl_1/runme.log` | same | same |
| methodology (`report_methodology`) | TIMING-*, CLKC-*, LUTAR-* ... | same; TIMING-29 and a non-zero clk <-> clk2x hold requirement stop the build by themselves (`HOLD_ANCHOR` in `fpga/u200/build.tcl`) | same |
| routed DRC | DPIP-*, DPOP-*, REQP-* ... | same | same |
| out-of-context synthesis for the netlist gate (`synth/vivado/run.sh`) | Synth 8-* | to do: the census reads image builds only | open |
| Tiny Tapeout flow (separate repository) | Yosys / OpenROAD / magic | to do | open |
| Python and shell tooling | interpreter warnings | to do | open |

Vivado stops printing a message kind after 100 instances per process. Until 2026-11-16 that made every count
below a lower bound (45 kinds were cut off in the first census). The limit is lifted for every Vivado process on the
build machine by `~/.Xilinx/Vivado/2025.2.1/Vivado_init.tcl` (`set_param messaging.defaultLimit 1000000`); a hook
set from `build.tcl` reaches `synth_1` and `impl_1` only, because the out-of-context runs do not exist yet when it
is set. The census reports any kind that was still cut off, and `--gate` fails on it.

## Owners

`tools/warn_census.py` sorts every message by who can fix it:

- **decoder**: our RTL (the `sys_host_0`, `sys_pg_0`, `sys_clkgen_0` runs, and report lines that name `sys_i/host/`).
- **shell**: our block design, constraints and flow (`synth_1`, `impl_1`, the reports).
- **vendor**: AMD's XDMA, DDR4 controller, SmartConnect, reset and debug blocks. Their RTL is not ours to change;
  these are waived by kind after one reading each, never fixed.

## First census: u200-fc3rf16te2e100p-c86eb7f (2026-11-16, lower bounds, no waivers)

decoder 3462 printed, shell 27, vendor 1148. The file is `census-u200-fc3rf16te2e100p-c86eb7f.md`.

### Decoder kinds and what happens to each

| kind | printed | reading | treatment |
|---|---:|---|---|
| Synth 8-324 index out of range | 14 | `lf_top.sv` picked filter taps through `w8[int'(k) + 3]`, out of range for unmodified positions | FIXED 81fdcd4 (constant indices; 7 suites pass). A cast index is what Vivado built differently in `lr_stream` on 2026-11-15 |
| REQP-1860 / REQP-1861 | 384 + 384 | `frame_mem_uram` sets `USE_EXT_CE_A/B = "FALSE"` and still drives `OREG_CE` with the phase: in that mode the pin is ignored and the output register follows the read access. Function is the same while reads happen only at the mid-cycle edge, and the netlist replays pass, but the comment describes a mechanism that is not the one in use | OPEN: either `USE_EXT_CE = "TRUE"` (what the comment and the simulation model say) or tie the pins high and correct the comment; needs a netlist replay and an image |
| DPIP-2, DPOP-3, DPOP-4 | 1126, 524, 602 | DSP blocks without input / output / multiplier registers: performance advice | to read per block: where the path is timing-critical, register; elsewhere waive by block |
| DPREG-7 | 1 | `u_rc/p_1_out`: dynamic OPMODE may form an unregistered feedback path | OPEN: to read |
| Synth 8-6014, 8-3332, 8-3936, 8-3917 | cut off at 100 each | registers and bits removed or tied because nothing reads them | each one is either dead RTL (delete) or a configuration-dependent leftover; follows the simulator's UNUSEDSIGNAL work |
| Synth 8-4767 RAM built from registers | 6 | `mi_store.sv:163/164`, `rec_fifo.sv:88` (two), `fb_fetch.sv:82` (two): the known six, small | to waive with sizes, or give them `ram_style`; a NEW line of this kind stays a stop (2026-11-14) |
| Synth 8-6841 byte-wide write enable not used | 24 | block RAMs written with one enable | to read |
| Synth 8-6057, 8-6013 UltraRAM without pipeline registers | 12 + 1 | performance advice on `mi_store` and `u_mvm` | to read against timing |
| Synth 8-11357 | 1 | `line_reg` with 15360 registers (the same `fb_fetch` RAM as 8-4767) | with 8-4767 |
| CLKC-30, CLKC-56 | 1 + 1 | the MMCM takes its input from a global buffer of the DDR4 controller | to read |

### Shell kinds

| kind | printed | reading | treatment |
|---|---:|---|---|
| TIMING-29 inconsistent multicycle pair | 1 | the clk -> clk2x hold exception lacked `-end` | FIXED 09af9e6; the build refuses an image that has it |
| Vivado 12-1008, Project 1-498 | 2 + 1 | `sys.xdc` was read at synthesis, before the MMCM clocks exist | FIXED e062b8f (implementation only); to confirm on the next census |
| Runs 36-537 | 1 | hook script outside the project | FIXED e062b8f; to confirm |
| TIMING-9 unknown CDC logic | 1 | an asynchronous crossing behind a false path, without a recognised synchroniser | OPEN: to read, this is the kind that hides bugs |
| IP_Flow 19-5661 / 19-11770, BD 41-3281 / 41-2671, filemgmt 56-443 | 4, 4, 2, 2, 4 | block-design bookkeeping on our module references | to read: most can be fixed by declaring the interface parameters |
| Timing 38-436 | 1 | asks for `report_bus_skew` | add the report to the build and read it |
| Power 33-332, Vivado 12-7122, Vivado 12-1815 | 1, 1, 2 | power estimate note, no incremental reference, IOSTANDARD on GT pins | to read |

### Vendor kinds

1148 lines in 17 kinds (unconnected ports, trimmed registers, waiver bookkeeping of the IP's own constraint files,
PCIe core timing notes). To be read once each and waived by kind and owner.

## Simulator (Verilator 5.053, `--lint-only -Wall`, top `dec_host`, CHAIN=1; 2026-11-16)

With the three switched-off kinds enabled: 312 UNUSEDSIGNAL, 196 UNUSEDPARAM, 8 PINCONNECTEMPTY, nothing else.

- UNUSEDSIGNAL: 116 signals never used and 17 neither driven nor used (dead RTL: delete, each one read first; an
  error flag nobody reads is a finding, not noise), 142 with unused bits and 37 function variables (per site: narrow
  the signal, or a `lint_off` on that line with the reason). Most in `dec_top` (67), `recon_top` (51), `dec_host` (19).
- UNUSEDPARAM: 185 of 196 in the generated table package `cdf_map_pkg` (one file-level waiver); the other 11 read one by one.
- PINCONNECTEMPTY: 8 deliberately open output pins; written as explicit open connections with a comment, per site.
- A signal used in one build configuration and not in another (CHAIN 0 / 1, INTER, core stubs) is waived at its
  declaration with the configuration named. The lint runs for every configuration the nightly builds.

## Order of work

1. Complete census (message limit lifted) from the next image build.
2. The open findings that could be functional: REQP-1860/1861, TIMING-9, DPREG-7.
3. Simulator: dead signals out, per-site waivers, the three `-Wno` flags removed from `tb/runner.py`.
4. Synthesis leftovers that remain after 3.
5. Waiver file complete; `warn_census.py --gate` in `fpga/u200/build.sh`; lint of every configuration in the nightly.
6. The out-of-context gate synthesis, the Tiny Tapeout flow, the tooling.

## Log

### 2026-11-16, afternoon

- **Simulator lint per configuration** (`tools/lint_all.sh`): a default-parameter lint called error flags unused that
  the image configuration reads (`fsl_err`, `pj_err`), so the lint runs in four configurations. Before the cleanup:
  image CHAIN=1 294 UNUSEDSIGNAL, image CHAIN=0 244, intra-only small 301, defaults CHAIN=1 312; 75 signals unused in
  all four.
- **66 of those 75 removed** (f38b037): 24 submodule outputs became explicitly open pins. The other 9 are module input
  ports or are read by a testbench or the ILA probe list. After: 229 / 179 / 236 / 247. Full nightly on that commit:
  63 pass, 0 fail.
- **PINCONNECTEMPTY** flags `.port()`, which is the idiom that states an output is left open on purpose. It will be
  waived as a kind (switched off in `tb/runner.py` with this paragraph as the reason), not per site.
- **Second census, u200-fc6clte2s90p-e062b8f**: decoder 3455, shell 24, vendor 785. Gone as intended: Vivado 12-1008,
  Project 1-498, Runs 36-537, TIMING-29. `impl_1` printed everything (no cut-off); the synthesis runs of this build
  started before the init file existed and are still cut off (41 kinds). New in the shell list, both worth reading:
  Place 46-14 (the placer calls the design highly congested) and Route 35-468 (17 pins that are both setup- and
  hold-critical).
- **TIMING-9 read** (`report_cdc` on the routed design): one Critical, CDC-11, the XDMA's `user_reset_reg` fanning out
  to more than one synchroniser, one of them the decoder reset block's first stage (`rst_dec ... syncstages_ff_reg[0]`),
  behind a false path. It is the PCIe core's reset entering our reset block; 23 Warning rows not yet read.
- **Open finding, `dec_host.sv` auto-flush**: `if (row_final && !fq_full) fq_w <= fq_w + 1` drops a released row
  without a trace when eight flushes are outstanding. Whether that can occur is not known. Either show it cannot, or
  give it a sticky error bit in STATUS like the chain's FIFO overflow flags.
- **Open finding, `blk_syntax`**: the removed `a_ok` / `l_ok` (neighbour available AND written) were never read; the
  context logic uses availability alone. Probably equivalent inside a tile; to confirm against the MV store.

### 2026-11-18

- **Three lint configurations, not four** (`tools/lint_all.sh`): the `CHAIN` parameter and the frame-wide filter stages
  are gone, so there is one image configuration (`dec_host-image`), the intra-only small one and the defaults. Counts
  of the new `dec_top` / `dec_host` against the last four-configuration run are in `docs/warnings/lint/`; two notes
  are new and waived as benign: `dec_host` `start[5:2]` (the START bits of the removed stages: written by older host
  software, read by nothing) and unused fields of `dec_top`'s `cd_rd_info` (the chain takes only the skip flag from the
  second mode-info read).
