# openav1 — an open-source AV1 hardware decoder

**Goal:** a complete, spec-exact AV1 video decoder in synthesizable SystemVerilog, developed in the open,
verified against the reference decoders at every stage, and brought up on a low-cost FPGA (AMD Kria KV260)
before anyone talks about silicon. Year one targets **intra-only 1080p** (key frames / still pictures /
AVIF): the arithmetic decoder, the whole tile syntax, prediction, transforms, reconstruction and the
in-loop filters. Inter prediction (motion compensation, reference buffers) is the next mountain.

Why: hardware AV1 decoders exist inside phones and TVs, but there is no open one you can read, simulate,
change and put on your own board. This project is that. It is also an experiment in how far one careful
engineer plus an AI collaborator get when everything is derived from the specification text and checked
against dav1d bit for bit.

The project is directed by Jonah and built by Claw (an AI running on a Raspberry Pi 5 in Berkeley). Every
line here was written from the [AV1 specification](https://aomediacodec.github.io/av1-spec/), not from any
existing decoder's source; dav1d and libaom are used only as judges.

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

## Status (2026-09-27)

| Stage | Python model | RTL | Verified |
|---|---|---|---|
| OBU / sequence / frame header parsing | `tb/obu_parser.py` | software (by design) | every field == dav1d on the corpus |
| Arithmetic decoder (msac, spec 8.2) | `tb/tile_model.py` | `rtl/msac.sv` (1 symbol/clk) | 1.05 M symbols from dav1d traces + 2.8 k spec-model fuzz |
| CDF storage + symbol sequencing | — | `rtl/cdf_store.sv`, `rtl/sym_seq.sv` | via the syntax tests |
| Coefficient reading (5.11.39) | `tb/tile_model.py` | `rtl/coef_rd.sv` | 26 k symbols / 7 k coefficients fuzz |
| Tile syntax: partitions, mode info, segmentation, delta q/lf, CDEF idx, tx size/type, residual loop | `tb/tile_model.py` | `rtl/blk_syntax.sv`, `rtl/blk_ctx.sv`, `rtl/tile_syntax.sv` | model: 1.05 M symbols == dav1d; RTL: 49+ tiles of generated streams |
| Palette mode (5.11.46/49) | `tb/tile_model.py` | `rtl/pal_syntax.sv` | 390 palette blocks, 280 k map pixels |
| Loop-restoration unit syntax | `tb/tile_model.py` | `rtl/lr_syntax.sv` | 110+ units |
| Inverse transforms (7.13): DCT 4–64, ADST 4–16, identity, WHT, all 19 sizes | `tb/itx_model.py` | `rtl/itx1d.sv`, `rtl/itx2d.sv` (microcoded butterfly engine) | 5 k dav1d blocks + 1.7 k fuzz |
| Intra prediction (7.11.2): 13 modes, edge filter/upsampling, filter-intra | `tb/intra_model.py` | `rtl/ipred.sv` | 25 k dav1d prediction blocks |
| Chroma from luma (7.11.5) | `tb/intra_model.py` | `rtl/cfl.sv` | 264 dav1d blocks + fuzz |
| Dequantisation, edge preparation, palette prediction, reconstruction, frame buffer | `tb/recon_model.py` | `rtl/recon_top.sv`, `rtl/frame_mem.sv` | whole frames == model: all 21 corpus streams (44 frames, 4.9 M pixels) + 18 generated frames |
| Deblocking loop filter (7.14) | `tb/lf_model.py` | `rtl/lf_top.sv`, `rtl/mi_store.sv` | 6 real frames, 37 k filtered pixels == model; model == dav1d |
| CDEF (7.15) | `tb/cdef_model.py` | `rtl/cdef_top.sv` | 4 real frames, 172 k filtered pixels == model; model == dav1d |
| Loop restoration (7.17): Wiener + self-guided | `tb/lr_model.py` | `rtl/lr_top.sv` | 2 real frames, 32 k filtered pixels == model; model == dav1d |
| Super-resolution, film grain | not yet | not yet | — |

`rtl/dec_top.sv` is the integrated intra decoder: tile bytes + parsed headers in, then the in-loop filters
(deblocking, CDEF, loop restoration) run over the finished frame on request; the output picture is read
from the frame buffer the last stage wrote. It is slow-and-correct (one pixel or coefficient per cycle or two, no overlap between
stages); throughput work starts once the pipeline is complete and the numbers are measured.

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
SIM_FAST=1 FBX=12 FBY=9 ...                                          # faster Verilator build; frame-buffer capacity (log2 px)
python tb/runner.py tile_syntax               # syntax decoder vs model (TS_SCREEN=1 for palettes, TS_DEBUG=1 for symbol dumps)
python tb/runner.py itx2d | ipred | cfl | msac | coef_top           # per-block tests
tools/fuzz_streams.sh 40                      # generated streams: dav1d vs the Python decoder
```

Synthesis estimates (generic cells, no target library):
`$OPENAV1_YOSYS -p "plugin -i slang; read_slang rtl/cdf_map_pkg.sv rtl/blk_tables_pkg.sv rtl/syn_pkg.sv rtl/pal_syntax.sv --top pal_syntax; synth -top pal_syntax; stat"`

## Hardware plan

First board: AMD Kria KV260 (Zynq UltraScale+ K26: ~117 k LUTs, 144 BRAMs, quad Cortex-A53 running Linux).
Header parsing and stream feeding run on the ARM cores; the fabric holds the tile decoder; the picture goes
out over HDMI/DisplayPort. Vivado (x86 only) builds the bitstream; the board is driven over Ethernet.
Every block is parameterised so throughput trades against area (butterfly slots, pixels per clock).

## Roadmap

1. Finish the intra pipeline in RTL: reconstruction end to end, then deblocking, CDEF and loop restoration.
2. Argon conformance: run the intra-only subset through the RTL simulation.
3. Throughput: measure, then pipeline (2 clk/symbol target for the syntax decoder, 4+ px/clk reconstruction).
4. KV260 bring-up: software header parser and driver on the ARM side, first pictures on a screen.
5. Inter prediction.

## License

Apache License 2.0 (see `LICENSE`). Copyright 2026 Jonah Stiennon and contributors. The AV1 specification
is published by the Alliance for Open Media under its own terms; dav1d (BSD-2) and libaom (BSD-2 + AOM
patent license) are used unmodified apart from the trace hooks in `tools/dav1d_trace_patch.py`.
