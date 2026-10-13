# Future improvements

Deferred work: things we decided not to do yet, with the reasoning, so the decision does not have to be remade.

## Entropy decoder: speculative CDF fetch (deferred 2026-10-12)

The msac symbol loop (u_cdf UltraRAM read → range multiply → compare → val/range update) is the last timing
path in the whole decoder: WNS −1.5 ns at a 100 MHz target, so the chip tops out near 87 MHz while every other
block meets 100 MHz standalone. The loop is serial by nature: each symbol's result feeds the next symbol's state.

Decision (Jonah, 2026-10-12): leave it at one symbol per cycle, ~87 MHz, for the first KV260 builds.

Preferred fix when we come back to it: issue the next symbol's CDF read one cycle early whenever its address does
not depend on the symbol currently being decoded (most symbols), and take a one-cycle bubble when it does. That
takes the UltraRAM read off the critical path and keeps ~1 symbol/cycle. Estimated effort: one to two days plus
the usual unit tests → nightly gate → synth → netlist replay. The risk is in the dependency detection (a wrong
"independent" verdict silently decodes garbage, so the Argon pass and gate replay are the acceptance test).

Alternatives considered and rejected for now:
- Two cycles per symbol: 100 MHz clock but 50 M symbols/s; fine at normal bitrates, bottleneck on very
  high-bitrate or intra-heavy streams.
- Separate ~85 MHz clock for tile_syntax with CDC FIFOs at the boundary: more machinery than it is worth.

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
