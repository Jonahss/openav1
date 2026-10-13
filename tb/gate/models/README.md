# Gate-level replay: simulation models and the driver script

`gate_block.sh` is the atium-side driver (`~/gate/gate_block.sh`): it writes a funcsim netlist from a Vivado
checkpoint, compiles it with the replay harness (`tb/gate/replay_<block>.sv`) and replays recorded vectors
(`tools/vcd2vec.py`) against it; optionally the RTL as a control. Copied here so the flow is versioned.

The whole-decoder netlist is flat and black-boxes the frame memory (`synth/vivado/frame_mem_bb.sv`), so the
funcsim contains empty stub modules, one per parameterisation, under Vivado's names with no parameter overrides:
`frame_mem` (u_fb, NRD=5/NWR=3), `frame_mem__parameterized0` (u_fb1, 2/2), `frame_mem__parameterized1` (u_fb2, 1/1)
for dec_top at fb10x9. `frame_mem_sim.sv` is `rtl/frame_mem.sv` renamed (plus `+fminit=zero|rand` and an X-write
reporter) and `frame_mem_variants.sv` wraps each netlist name with that instance's parameters baked in. Both are
compiled after the netlist so they override the stubs. Check the `frame_mem ... init=... N=` banners in the xsim
log: one per instance, with the right N. If the geometry or the port counts change, update the wrappers.

History (2026-10-12): without the wrappers u_fb ran at the model defaults and u_fb1/u_fb2 stayed stubs; the
seed-1 replays passed anyway (port 0 only, no filters, 128-px frames). A loop-filter stream (seeds 2-8 at
TS_STAGE=lr) exposed it. Any whole-decoder replay must include such a stream.
