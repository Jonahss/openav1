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
