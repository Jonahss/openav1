# openav1 — an open-source AV1 hardware decoder

**Goal:** a complete, spec-exact AV1 video decoder in synthesizable SystemVerilog, developed in the open,
verified against the reference decoders at every stage, and brought up on a low-cost FPGA (AMD Kria KV260)
before anyone talks about silicon. Year one targets **intra-only 1080p** (key frames / still pictures /
AVIF): the arithmetic decoder, the whole tile syntax, prediction, transforms, reconstruction and the
in-loop filters. **Inter prediction is in RTL too** (since 2026-10-19): the inter block syntax, the full MV
candidate stack, motion compensation with scaled references, warped and global motion, OBMC, wedge and
difference-weighted compounds and inter-intra, with the references read from DDR over AXI. 300 of the 329
Argon profile-0 inter conformance streams that fit the simulation budget decode bit-exact through the whole
pipeline (motion compensation, deblocking, CDEF, super-resolution, loop restoration); profiles 1 and 2 (4:4:4,
4:2:2, 12-bit) are being run now. The full decoder (~150 k LUTs) no longer fits the KV260, so it is moving to an
Alveo U200 while the Kria keeps the intra decoder.

Why: hardware AV1 decoders exist inside phones and TVs, but there is no open one you can read, simulate,
change and put on your own board. This project is that. It is also an experiment in how far one careful
engineer plus an AI collaborator get when everything is derived from the specification text and checked
against dav1d bit for bit.

The project is directed by Jonah and built by Claw (an AI running on a Raspberry Pi 5 in Berkeley). Every
line here was written from the [AV1 specification](https://aomediacodec.github.io/av1-spec/), not from any
existing decoder's source; dav1d and libaom are used only as judges.

## Progress (latest first)

- **2026-10-25** Two small bugs with long shadows, found with new trace hooks (`TD_MVTRACE` / `MV_TRACE` compare the RTL's and the
  model's MV candidate stacks block by block): the motion-field loaders (testbench and board driver) wrote one row and one column
  past the field, which wraps onto row 0 when a frame exactly fills the build's band and erased the top row's temporal MV
  candidates; and `order_hint_bits` was a 3-bit field, so OrderHintBits = 8 reached the reconstruction as 0 and swapped the
  compound distance weights. The register packer now refuses values that do not fit. Gate round 5 (netlist replays of the
  inter decoder, 155 k + 244 k cycles, 0 mismatches) and the 50 MHz U200 inter build run at the fixed commit.
- **2026-10-24 — picture memory step A; the inter decoder on the U200.** The deblocked / CDEF / restored pictures live in
  UltraRAM superblock-row rings (`docs/picture-memory.md`); inter frames decode through `dec_host` the way the board runs them
  (registers, sequencer, DMA flush, references = the flushed pictures). A multi-tile bug in the host interface (the tile byte
  FIFO kept the previous tile's tail) was found this way. First pipelining pass for 100 MHz (CDEF / LR write ports, tx-type CDF
  address, MI store, `warp_setup` multiplier); the 50 MHz inter build on the Alveo U200 closes timing (292 k LUT, 271 URAM,
  831 DSP); 100 MHz needs 2-3 internal stages in the CDEF and loop-restoration datapaths (throughput phase).
- **2026-10-23 — Alveo U200 host path.** XDMA + DDR4 block design around `dec_host`, host driver `--xdma` backend
  (`docs/u200-bringup.md`); the intra decoder closes at 100 MHz on the U200 (80 MHz with everything met). Throughput budget
  from the Argon census: 3.69 cycles / pixel today with the stages back to back (`docs/throughput-budget.md`); 1080p60 at
  200 MHz needs 1.6 c/px per stage, 4K60 is a further 4x.
- **2026-10-20/22 — inter through the whole pipeline.** Loop filter reference / mode deltas for inter blocks (7.14.4) and the
  skipped-inter edge rule (7.14.2) were the two gaps the first lr-stage Argon batch found; 300 / 329 recon-exact Argon inter
  streams are bit-exact through deblocking, CDEF, super-resolution and loop restoration (the 29 largest are the simulation
  budget). `INTER=0` compiles the inter paths out for the K26 (109.9 k LUT window build).
- **2026-10-19 — motion compensation (stage 2 of inter prediction), all of it.** `ref_fetch` (AXI read master, edge-replicated
  patches), `mc_filt` (4-lane separable 8-tap filter, AVERAGE / DISTANCE blends, 17-bit compound lanes), scaled references
  (7.11.3.3), OBMC, wedge and difference-weighted masks, inter-intra, local and global warp (`warp_setup`, 7.11.3.5/6),
  sub-8x8 chroma motion lists, the reference slot table and DDR reference pictures in `dec_host`. Every block fuzzed and
  netlist-replayed; Argon inter streams decode bit-exact against the model at the reconstruction stage. Merged to `main`.
- **2026-10-19 (morning)** Inter frame syntax in RTL (stage 1 of inter prediction): `blk_syntax` decodes inter blocks (references, the
  full MV candidate stack with global / temporal / compound candidates, DRL, MV differences, inter-intra, motion modes with the
  overlappable and warp-sample scans, compound types, switchable filters), with the inherited CDFs, segmentation map and
  motion field in host-written band memories. Argon profile-0 inter streams are record- and symbol-exact against the Python
  model (docs/inter-syntax.md). Branch `inter`; motion compensation is stage 2.
- **2026-10-14 — it runs on the board, at 80 MHz.** The core decoder (entropy decoding, tile syntax, prediction,
  transforms, reconstruction, deblocking) is a bitstream on the Kria KV260 with a 512 x 512 UltraRAM frame memory:
  102.5 k LUTs (87 % of the K26), timing met at 80 MHz. libaom-encoded intra streams decode **bit-exact with dav1d**
  on the board; a 120-frame 352 x 288 clip at **86 fps end to end**, the picture written into the PS's DDR by the
  decoder's own DMA (`tools/kv260/decode.py`: stream file in, picture out). Timing work the same day: the arithmetic
  decoder in two stages, the intra predictor's edge-address lookahead, the frame-memory write address precomputed;
  the routed core is 1.95 ns short of 100 MHz, with routing congestion at 99 % slice use the remaining obstacle.
  Intra-only; CDEF, loop restoration and super-resolution are built and verified in simulation but left out of this
  bitstream for space.
- **2026-10-13 — area.** Whole decoder 148.9 k LUTs (from 164 k): level-folded transform engine
  (`docs/itx-level-engine.md`), LUTRAM rewrites. The full decoder is 1.27x the K26; the core build routes at
  100 k LUTs, ~78 MHz, the arithmetic decoder's symbol loop the critical path.
- **2026-10-11 — gate-level rule.** Every block and the whole decoder replay bit-exact against the Vivado netlist
  before a merge (simulation green is not enough; two synthesis bugs were caught this way).
- **Nightly** on the batch host: tile syntax, whole-frame decodes on generated streams, the real corpus (44 frames)
  and an Argon conformance subset (25 frames) through the RTL, all compared with dav1d / Argon checksums.
- **Next:** the inter decoder running on the Alveo U200 (50 MHz image first), profiles 1 and 2 through the RTL, then the
  throughput phase: pipelined filter datapaths for 100 MHz, 4+ pixels per clock, the picture memory in DDR (steps B-D of
  `docs/picture-memory.md`) towards 4K60.

## Method

1. **Spec → executable model.** Each decoding process is transcribed from the spec's pseudocode into plain
   Python (`tb/*_model.py`, `tb/tile_model.py`, `tb/obu_parser.py`), function by function, with the spec
   open next to it. The tables (`tb/av1_tables.py`, 211 of them) are extracted mechanically from the spec
   text by `tools/gen_spec_tables.py` and count-checked against their declared dimensions.
2. **Model → dav1d.** A patched, asm-free dav1d (`tools/dav1d_trace_patch.py`) logs every arithmetic-decoder
   symbol, every dequantised coefficient block, every prediction block with the edge pixels it used, and
   every reconstructed block. The model decodes the same tile bytes and each event is compared. The models
   are bit-exact on a corpus of aomenc-encoded streams covering 8/10/12-bit, 4:2:0/4:2:2/4:4:4/mono,
   lossless, screen content with palettes, odd frame sizes, 128-pixel superblocks and multiple tiles, and
   the complete Python decoder (`tools/decode.py`) reproduces dav1d's output pictures byte for byte.
3. **Model → RTL.** Each hardware block is verified against the model at its own boundary: random-stimulus
   fuzzing (thousands of blocks in a minute), replay of dav1d's traced data, and, for the syntax decoder and
   the full pipeline, streams produced by running the model *backwards* (`tools/gen_stream.py`: a symbol
   encoder plus an OBU writer generate random conformant streams that exercise syntax no encoder emits).
4. **RTL → dav1d.** `tb/test_dec_top.py` decodes whole frames with the RTL and compares the frame buffer with
   the model's picture pixel for pixel, on generated streams and on the real corpus.

The two questions "did I read the spec right?" and "did I build the hardware right?" are always answered
separately.

## Status (2026-10-25)

| Stage | Python model | RTL | Verified |
|---|---|---|---|
| OBU / sequence / frame header parsing | `tb/obu_parser.py` | software (by design) | every field == dav1d on the corpus |
| Arithmetic decoder (msac, spec 8.2) | `tb/tile_model.py` | `rtl/msac.sv` (1 symbol/clk) | 1.05 M symbols from dav1d traces + 2.8 k spec-model fuzz |
| CDF storage + symbol sequencing | — | `rtl/cdf_store.sv`, `rtl/sym_seq.sv` | via the syntax tests |
| Coefficient reading (5.11.39) | `tb/tile_model.py` | `rtl/coef_rd.sv` | 26 k symbols / 7 k coefficients fuzz |
| Tile syntax: partitions, mode info, segmentation, delta q/lf, CDEF idx, tx size/type, residual loop | `tb/tile_model.py` | `rtl/blk_syntax.sv`, `rtl/blk_ctx.sv`, `rtl/tile_syntax.sv` | model: 1.05 M symbols == dav1d; RTL: 49+ tiles of generated streams |
| Palette mode (5.11.46/49) | `tb/tile_model.py` | `rtl/pal_syntax.sv` | 390 palette blocks, 280 k map pixels |
| Loop-restoration unit syntax | `tb/tile_model.py` | `rtl/lr_syntax.sv` | 110+ units |
| Inverse transforms (7.13): DCT 4–64, ADST 4–16, identity, WHT, all 19 sizes | `tb/itx_model.py` | `rtl/itx_lvl.sv` (level-folded butterfly engine, generated by `tools/gen_itx_lvl.py`; see `docs/itx-level-engine.md`), `rtl/itx_pair.sv`, `rtl/itx_row.sv`, `rtl/itx2d.sv` | 5 k dav1d blocks + 1.7 k fuzz + 1.4 k packed engine vectors |
| Intra prediction (7.11.2): 13 modes, edge filter/upsampling, filter-intra | `tb/intra_model.py` | `rtl/ipred.sv` | 25 k dav1d prediction blocks |
| Chroma from luma (7.11.5) | `tb/intra_model.py` | `rtl/cfl.sv` | 264 dav1d blocks + fuzz |
| Dequantisation, edge preparation, palette prediction, reconstruction, frame buffer | `tb/recon_model.py` | `rtl/recon_top.sv`, `rtl/frame_mem.sv` | whole frames == model: all 21 corpus streams (44 frames, 4.9 M pixels) + 18 generated frames |
| Deblocking loop filter (7.14) | `tb/lf_model.py` | `rtl/lf_top.sv`, `rtl/mi_store.sv` | 6 real frames, 37 k filtered pixels == model; model == dav1d |
| CDEF (7.15) | `tb/cdef_model.py` | `rtl/cdef_top.sv` | 4 real frames, 172 k filtered pixels == model; model == dav1d |
| Loop restoration (7.17): Wiener + self-guided | `tb/lr_model.py` | `rtl/lr_top.sv` | 2 real frames, 32 k filtered pixels == model; model == dav1d |
| Intra block copy (5.11.7 / 7.10.2 / 7.11.3 with the current frame as reference) | `tb/tile_model.py`, `tb/recon_model.py` | `rtl/mv_stack.sv`, `rtl/mv_mem.sv`, `rtl/blk_syntax.sv`, `rtl/recon_top.sv` | model == dav1d on 4 Argon intrabc streams (test4592/8304/5297/5366: 1.04 M symbols, 22 k coefficient blocks, 21 pictures); RTL syntax == model on test4592 + test8304 (10 k blocks); full RTL picture == dav1d on test4592 |
| Inter block syntax (5.11.7-5.11.33): references, MV prediction (7.10.2: spatial, temporal, global, compound candidates, DRL), MV differences, motion modes, compound types, inter-intra, switchable filters | `tb/tile_model.py`, `tb/seq_model.py` (CDF / segmentation / motion-field inheritance, 7.9 projection) | `rtl/blk_syntax.sv`, `rtl/mv_stack.sv`, `rtl/mv_mem.sv`, `rtl/mf_band.sv`, `rtl/seg_band.sv` | model == dav1d on 758 / 766 Argon inter streams (header parser 779 / 779); RTL record- and symbol-exact on 260+ Argon inter streams + 3 netlist replays |
| Motion compensation (7.11.3): reference fetch with edge replication, 8-tap separable filters, scaled references, AVERAGE / DISTANCE / wedge / DIFFWTD compounds, OBMC, inter-intra, local + global warp | `tb/recon_model.py` | `rtl/ref_fetch.sv`, `rtl/mc_filt.sv`, `rtl/warp_setup.sv`, `rtl/recon_top.sv` | block fuzz (hundreds of units each) + netlist replays; whole-frame: 300 / 329 recon-exact Argon profile-0 inter streams bit-exact through all filters at `TS_STAGE=lr`; profiles 1 + 2: 584 streams run, 5 flagged -> 2 bugs fixed 2026-10-25, 3 watchdog false alarms |
| Host interface: register file, frame sequencer, DMA flush of finished rows, reference slot table, AXI read master | `tb/host_regs.py` (the board driver) | `rtl/dec_host.sv`, `rtl/rec_fifo.sv`, `rtl/frame_mem*.sv` | inter frames through `dec_host` with the flushed pictures as references (Argon test47 / 6641 / 648 / 8854, multi-tile rows, super-res + CDEF + LR); gate-level replays of the whole decoder every round |
| Super-resolution (7.16) | `tb/superres_model.py` | `rtl/sr_top.sv` (in-place row upscaler) | model == dav1d on 4 Argon upscaling frames; full RTL picture == dav1d on test60 (60->67), test5644 (255->447), test7463 (1260->2048 with CDEF + self-guided restoration) |
| Film grain synthesis (7.18.3) | not planned for the core: conformance references are grain-free (Argon md5_no_film_grain) | — | — |

`rtl/dec_top.sv` is the integrated decoder (intra and inter): tile bytes + parsed headers in; the in-loop filters
(deblocking, CDEF, super-resolution, loop restoration) either run over the finished frame on request
(host-driven mode) or, with `pipe_en`, trail the tile decoder by superblock rows under a frame sequencer so
that a frame takes as long as its slowest stage rather than the sum (`TS_PIPE=1` in the testbench). The output
picture is read from the frame buffer the last stage wrote.

Throughput (2026-09-28, simulated cycles for a 640x360 8-bit corpus frame, bit-exact with dav1d throughout):
~16 M on 09-27 (one sample per cycle everywhere, stages one after another) → ~1.1 M now (tile decoder 0.56 M,
filters overlapped; per stage on real frames: deblock 1.2, CDEF 1.4, loop restoration 0.7 cycles per pixel).
The steps and the per-stage profiles are in the commit history; the frame buffer's 4-sample-wide ports stand in
for a picture memory banked by x mod 4. The tile decoder is the current constraint (coefficient reading ~15
cycles per coefficient, prediction edge gather, transforms); synthesis figures come next.

Reference oracle: the Argon conformance suite (2,763 streams) decodes identically with dav1d and against
its reference checksums on our setup (`results/`), so dav1d is a trustworthy judge. Argon runs through the RTL
started 2026-09-27 (profile0_core intra-only, non-superres subset: 52 streams); the parser follows the
decoder's operating-point rules (OBUs of other temporal/spatial layers are dropped) and film grain is
compared with grain disabled (`md5_no_film_grain`).

## Layout

```
rtl/        SystemVerilog. *_pkg.sv and scan_rom.sv / cdf_map_pkg.sv / q_tables_pkg.sv are generated (tools/gen_*.py)
tb/         Python spec models, cocotb testbenches (test_*.py), the cocotb/Verilator runner
tools/      trace patch for dav1d, cross-checks model-vs-trace (xcheck_*.py), stream generator + fuzzer,
            standalone Python decoder, regression drivers, Argon oracle scripts, table generators
streams/    the checked-in synthetic corpus (aomenc-encoded; streams/argon is the conformance suite, not checked in)
docs/       spec extracts used while writing the RTL
```

## Getting started

```
./bootstrap.sh          # Debian/Ubuntu, x86_64 or arm64: apt packages, venv+cocotb, OSS CAD Suite,
                        # trace-hooked dav1d in prefix-trace/, spec text. ~15 min plus downloads.
source env.sh
tools/regress.sh        # trace every corpus stream with dav1d and run all model + RTL cross-checks
python tb/runner.py dec_top                   # RTL vs model, whole frames, generated streams (TS_SEEDS=..., TS_W/TS_H)
TS_IVF=streams/synth/synth_8bit.ivf python tb/runner.py dec_top    # same on a real stream
TS_NOMODEL=1 TS_STAGE=lr TS_IVF=... python tb/runner.py dec_top      # model-free: RTL runs every stage from the headers,
                                                                     # output compared with dav1d's pictures (refout/) or Argon md5s
SIM_FAST=1 FBX=12 FBY=9 ...                                          # faster Verilator build; frame-buffer capacity (log2 px, up to 14:
                                                                     # the coordinate widths MIW/SBW/PXW in rtl/syn_pkg.sv cap frames at 16384)
python tb/runner.py tile_syntax               # syntax decoder vs model (TS_SCREEN=1 for palettes, TS_DEBUG=1 for symbol dumps)
python tb/runner.py itx2d | ipred | cfl | msac | coef_top           # per-block tests
tools/fuzz_streams.sh 40                      # generated streams: dav1d vs the Python decoder
```

Synthesis estimates (generic cells, no target library):
`$OPENAV1_YOSYS -p "plugin -i slang; read_slang rtl/cdf_map_pkg.sv rtl/blk_tables_pkg.sv rtl/syn_pkg.sv rtl/pal_syntax.sv --top pal_syntax; synth -top pal_syntax; stat"`

## Hardware plan

First board: AMD Kria KV260 (Zynq UltraScale+ K26: ~117 k LUTs, 144 BRAMs, quad Cortex-A53 running Linux):
the intra decoder, header parsing and stream feeding on the ARM cores, the picture on the board's display
(`docs/kv260-bringup.md`). Second board: Alveo U200 (XCU200, ~1.2 M LUTs, 64 GB DDR4, PCIe) for the full inter decoder,
driven from the P620 host over XDMA (`docs/u200-bringup.md`). Vivado (x86 only) builds the bitstreams.
Every block is parameterised so throughput trades against area (butterfly slots, pixels per clock).

## Roadmap

1. ~~Finish the intra pipeline in RTL: reconstruction end to end, then deblocking, CDEF and loop restoration.~~ Done.
2. ~~Argon conformance: the intra-only subset through the RTL simulation.~~ Nightly; the inter subset (profile 0) is 300 / 329 bit-exact through every stage, profiles 1 and 2 running.
3. ~~KV260 bring-up.~~ Intra decoder at 80 MHz, pictures on the board's display (2026-10-14/17).
4. ~~Inter prediction.~~ In RTL and verified in simulation (2026-10-19 .. 25); on the U200 next (the full decoder is 1.3x the K26).
5. Throughput: the filter datapaths pipelined for 100 MHz, then 4+ pixels per clock and the picture memory in DDR (`docs/picture-memory.md`, `docs/throughput-budget.md`): 1080p60 at 200 MHz, 4K60 after.
6. A first piece of silicon: the arithmetic decoder (`rtl/msac.sv`) as a Tiny Tapeout submission once its FPGA twin with the same pin interface exists.

Deferred items and the reasoning behind them: [docs/FUTURE.md](docs/FUTURE.md).

## License

Apache License 2.0 (see `LICENSE`). Copyright 2026 Jonah Stiennon and contributors. The AV1 specification
is published by the Alliance for Open Media under its own terms; dav1d (BSD-2) and libaom (BSD-2 + AOM
patent license) are used unmodified apart from the trace hooks in `tools/dav1d_trace_patch.py`.
