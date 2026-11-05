# Future improvements

Deferred work: things we decided not to do yet, with the reasoning, so the decision does not have to be remade.

## Entropy decoder: symbol loop timing (done 2026-10-14)

The msac symbol loop (CDF read -> range multiply -> compare -> value / range update) was the decoder's last timing
path: -2.8 ns routed at 100 MHz in the core build, ~78 MHz. Deferred on 2026-10-12 ("leave as is"); done on
2026-10-14, but not the way sketched here (an early CDF read with dependency detection). What was built instead:
the decoder in two stages. The symbol sequencer already took two cycles per symbol (go, request), leaving msac idle
every other cycle, so the accept cycle now finds the symbol (multiply, compare chain) and the next cycle
renormalises, consumes the stream bits and adapts the row. The symbol still arrives one cycle after the request;
the adapted row two cycles after (resp_wb_valid), written back by the sequencer a cycle later than before, with
forwarding for a re-read of that row (sym_seq) and for any other requester (cdf_store, wb_pre). Cycle counts are
unchanged on every suite; the CDF store moved from UltraRAM to block RAM (a third of the clock-to-out). The loop
has +3.2 ns of margin at 100 MHz out of context and left the top of the in-context paths. The speculative early
read stays as an idea for a one-symbol-per-cycle decoder (3x the throughput of this one), which would need the
syntax FSMs to request ahead.

## xsim vs Verilator RTL evaluation (open since 2026-10-11)

xsim's RTL evaluation of lr_top and tile_syntax differs from Verilator's (the Vivado netlists match Verilator).
Likely xsim's always_comb is not sensitive to module-scope reads inside functions. Check what Yosys does before
the ASIC flow.

## Transform stage: transpose rotators and pack/unpack muxes (noted 2026-10-13)

After the level-folded engine (`docs/itx-level-engine.md`) the transform stage is ~51k LUTs in the decoder:
engine 18k, transpose memory ~20k (of which the two 64-lane log rotators ~15k), pack/unpack shift muxes ~5k,
load/store paths ~4k, streamed-row unit 1k. The rotators are the inherent cost of one-cycle row and column
access to the skewed 64-bank memory; a different memory organisation (e.g. half-width banks with two-cycle
column access, or keeping blocks of up to 64 coefficients inside the engine and transposing by lane permutation)
would trade cycles for those LUTs. The wrapper is also sequential per group (load, run, store); the memory has
separate read and write ports, so the store of group k could overlap the load of group k+1.

## warp_setup timing (2026-10-19)
The warp parameter unit (rtl/warp_setup.sv) multiplies on a combinational 64 x 64 path straight into the round-shift,
clamp and parameter registers: 61 logic levels, 16.5 ns out of context (synth warp_setup-2c2, 3.4k LUT / 14 DSP). It
is ~18 cycles per warp block and sequential, so the fix is mechanical: register the product (one bubble state per
product) and, if the multiplier alone still misses 10 ns, split it 32 x 64 + shift-add. Deferred until the warp stage
is bit-exact end to end; whole-decoder timing with warp will show it first.

## Product features: metadata, cropping, film grain, DRM (asked for 2026-11-04)

At the AOMedia Research Symposium (Jonah's talk, 2026-11-04) Alexis Tourapis (Apple) named what a decoder needs beyond
conformance to be considered for a product: accepting metadata, the cropping and stereo signalling that is coming, and
DRM "if we want to be a serious chip"; Fabien (NVIDIA) asked for film grain synthesis in hardware after all. Decisions:

- **Metadata OBUs** (`OBU_METADATA`: HDR_CLL, HDR_MDCV, ITU-T T.35 for HDR10+ / Dolby Vision, timecode, scalability):
  today `tb/obu_parser.py` and the host skip them. A decoder should pass them through untouched, in a sideband next
  to the picture they belong to, and leave interpretation to the display path; new types (stereo / multiview and
  whatever the next spec revision adds) ride the same path. The stance: syntax-level things flow around the decoder,
  pixel-level things go in it. On the card: a metadata FIFO the host drains per frame; on a chip: the same FIFO on the
  output side. No decode-loop change; deferred until the throughput work is done.
- **Cropping:** `render_size` (the intended display rectangle when it differs from the coded size, 5.9.6 / 7.21) is
  parsed and ignored; the coded frame is output whole. Honour it in the output stage (an output-crop register pair
  written from the frame header), not in the decode loop. Trivial; with the metadata work.
- **Film grain synthesis** (7.18.3): deferred on 2026-10-26 to the software driver, since it must not touch the
  references. The driver answer fails once the decoded pictures live in protected memory (below): the driver cannot
  read them. So grain becomes an **output-stage block after the decoder**: the two noise templates (a 73 x 82 luma
  and 38 x 44 chroma block from the LFSR), the scaling look-up tables from the film_grain_params piecewise function,
  one blend per output pixel at display rate, applied on the way out and never to the stored reference. ~15k gates;
  a Tiny Tapeout-sized block on its own, and a good second chip.
- **DRM / secure video path:** not decoder RTL but an SoC-integration feature: the bitstream is decrypted (AES-CTR
  Common Encryption) inside a trusted boundary before the OBU parser, the decoded pictures live in memory the CPU
  cannot read, only the display path (with HDCP) fetches them, and keys never leave a key ladder. Widevine L1 /
  FairPlay content needs exactly this. What it asks of the decoder now, so the core is secure-path capable later:
  (1) no pixel read-back through the register file except a debug mode that can be fused off (`dec_host`'s read-back
  DMA and the Kria register path are debug paths, not product paths), (2) control registers separate from data paths
  (one AXI-Lite control port, picture traffic on its own AXI master), (3) the picture bus able to sit behind a
  memory firewall (addresses from a base register, never from the stream). The current design already has (2) and
  (3) by construction; (1) is a build parameter to add when the output stage exists.
