# The level-folded inverse transform engine (`rtl/itx_lvl.sv`)

Written 2026-10-13, when this engine replaced the microcoded `itx1d` lanes inside `itx2d`.

## Why

The microcoded engine kept the transform vector in 64 registers and let any of its 2·NSLOTS slot results land
in any register. That indexed register file was the cost: ~15k LUTs of write-back muxes and ~3k of read muxes
per lane, against ~2k for the arithmetic. Two lanes plus their input logic were 44k LUTs of the 164k decoder.

A fully unrolled 64-wide butterfly network is not smaller: DCT64 is 241 in-place pair operations (81 rotations,
160 Hadamards), each with two 21-bit adders and clamps, so ~25–30k LUTs and ~300 DSPs for one vector per
clock, which is 50× more throughput than the decoder needs. The level-folded engine keeps the fixed wiring of a
network (no indexed register file) but executes one *level* of it per clock on 32 physical pair units.

## Structure

`tools/gen_itx_lvl.py` records the spec's pair operations from `tb/itx_model.py` for every DCT and ADST size,
schedules them ASAP into levels (an op depends on the previous ops touching its two lanes), packs 64/N rows of
an N-point transform into the 64 lanes, assigns the ops of each level to units greedily (minimising the number
of distinct lanes each unit input ever reads), and emits the engine as SystemVerilog `case` arms: for every
(program, level) the unit inputs, the constants (cos128/sin128 of the fixed angle, B or H, flip) and the lane
write-back are constants. Vivado turns the arms into ≤16:1 muxes (measured fan-in: unit inputs ≤ 14, write-back
per lane ≤ 13).

| program | rows per vector | levels | ops |
|---|---|---|---|
| 0 DCT4 | 16 | 2 | 64 |
| 1 DCT8 | 8 | 4 | 104 |
| 2 DCT16 | 4 | 6 | 148 |
| 3 DCT32 | 2 | 8 | 194 |
| 4 DCT64 | 1 | 10 | 241 |
| 5 ADST8 | 8 | 5 | 128 |
| 6 ADST16 | 4 | 7 | 176 |

Level 0 reads `in_vec` directly with the spec's input permutation (DCT bit reversal, ADST input permutation)
folded into its sources; later levels read the register vector. The ADST output permutation and its per-lane
negation are the output map. Every program's level 0 touches all 64 lanes, so there is no separate load path.
`rtl/itx_pair.sv` is the unit: the same B/H arithmetic as the old slot datapath (products in DSPs, Round2 12,
Hadamard with clamp to r bits). ADST4, identity and WHT are not butterfly networks; `rtl/itx_row.sv` does one
row of those per clock and `itx2d` streams such rows through it.

The DCT sizes nest: DCT-n is the sub-network of DCT64 on lanes below 2^n, and the extra ops of the larger sizes
never corrupt the lower lanes when the upper lanes are zero (every cross op is a Hadamard reading a zero lane).
The packed programs are the sub-networks replicated per row group, so nothing size-specific is bypassed.

## The 2D wrapper (`rtl/itx2d.sv` v4)

Rows of a group shift into `pk_in` one per clock from the top (row g ends in slot g from the top), the engine
runs, the results shift out of `pk_out` one row per clock, top slot first, so rows are stored in reverse
order (addressing only). Zero row groups (rows ≥ 32) store directly. Streamed rows (ADST4 / identity / WHT)
go through a three-stage pipeline: read row ri → `ld_row`, `itx_row` → `st_row`, write row st_i. The transpose
memory (64 skewed LUTRAM banks with two log rotators), the coefficient port and the residual read port are
unchanged, so `recon_top` is untouched.

Cycles per group: G loads + L levels + 1 + G stores. Rows of a 4-pt transform load at one per clock, DCT64
rows take 12 clocks each (10 levels); the old engine took 65 per row per lane.

## Verification ladder (all green on the merge commit)

1. `tools/gen_itx_lvl.py --check`: the emitted tables simulated in Python against the model, 300 random packed
   vectors per program.
2. `tb/test_itx_lvl.py`: the RTL fuzzed in Verilator, 1400 packed vectors, every row of every vector checked.
3. `tb/test_itx2d.py`: the 2D block, all 19 sizes and allowed types, coefficients loaded as the decoder loads
   them (clear, then only the non-zero ones — a dense load hid a wrong-row valid-bit index in the streamed path).
4. `tb/runner.py dec_top` seeds 2–8 at stage lr, the nightly gate (6 suites incl. the corpus vs dav1d and the
   Argon feature streams), and gate-level netlist replay of `itx2d` and of the whole decoder (`tb/gate/`).

## Numbers (Vivado 2025.2.1, xck26, out-of-context synthesis)

| | LUT | FF | DSP | WNS @100 MHz |
|---|---|---|---|---|
| `itx_lvl` alone | 19,247 | 1,294 | 128 | +2.9 ns |
| `itx2d` v4 alone | 47,367 | 6,194 | 194 | +2.2 ns |
| `itx2d` v3, NLANES=4 alone | 85,110 | 11,664 | 416 | +1.0 ns |
| decoder, v4 | 156,643 | 45,438 | 678 | −1.5 ns (msac loop, unchanged) |
| decoder, v3 two lanes | 164,332 | 45,542 | 705 | −1.5 ns |

Inside the decoder the transform stage went from 57.9k to 51.0k LUTs; the engine replaced 37.7k of lanes with
18.2k, the pack/unpack shift muxes (~5k) and the streamed-row pipeline (~2k) took part of that back, and the
transpose memory's rotators (~15k) are untouched. Whole-decoder regressions run ~12 % faster.

## What is left on the table

- The two 64-lane log rotators of the transpose memory (~15k) are the inherent cost of one-cycle row and
  column access; a cheaper transpose would need a different memory organisation.
- The pack/unpack muxes (~5k) are the cost of five transform sizes in one engine.
- Load/run/store of consecutive groups could overlap (the memory has separate read and write ports); the
  wrapper is sequential per group today.
