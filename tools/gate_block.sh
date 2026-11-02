#!/usr/bin/env bash
# gate_block.sh <block> <tag> <dcp|-> <vec.hex> [rtl files for the RTL control...]
# Replays <vec.hex> (+ .out) with ~/gate/<block>/replay_<block>.sv (tools/gen_replay.py) against the RTL (control) and the
# funcsim netlist of <dcp>: the gate rung of the verification ladder (docs/code-tour.md).
#   SIM=xsim (default)   the netlist under Vivado's xsim with unisims_ver / secureip (hours to days on dec_host)
#   SIM=verilator        the netlist under Verilator through the external vgate flow (VGATE=<dir>, default ~/vgate:
#                        vgate_build.sh / vgate_run.sh with its own primitive library); the harness takes +ncyc there
# NB: a funcsim is reused only when newer than its checkpoint (2026-10-14: a stale netlist under a reused tag
# elaborated against a new harness, failed, and the old xsim log was reported as a pass).
# NB: never add -L xpm to the xsim netlist elaboration: with it, UltraRAM reads simulate as zeros (2026-10-14); a flattened
# funcsim needs only unisims_ver / secureip even when the RTL used an XPM macro.
set -u
source /opt/Xilinx/2025.2.1/Vivado/settings64.sh
blk=$1; tag=$2; dcp=$3; vec=$4; shift 4
SIM=${SIM:-xsim}; VGATE=${VGATE:-$HOME/vgate}
cd ~/gate/$blk
G=/opt/Xilinx/2025.2.1/Vivado/data/verilog/src/glbl.v
if [ $# -gt 0 ]; then
  xvlog -sv -d RTL_CONTROL "$@" replay_$blk.sv > rtl_$tag.xvlog 2>&1 && xelab ${XELAB_OPT:-} -timescale 1ns/1ps replay_$blk -s rtl_$tag > rtl_$tag.xelab 2>&1 && xsim rtl_$tag -R -testplusarg vec=$vec -testplusarg xmask=1 > rtl_$tag.xsim 2>&1
  echo "RTL control: $(grep -E '^REPLAY' rtl_$tag.xsim || tail -n 3 rtl_$tag.xelab rtl_$tag.xvlog | tr '\n' ' ' | cut -c1-200)"
fi
if [ "$dcp" != "-" ]; then
  printf "open_checkpoint %s\nwrite_verilog -force -mode funcsim %s/%s_%s.v\nexit\n" "$dcp" "$PWD" "$blk" "$tag" > fs_$tag.tcl
  { [ -f ${blk}_$tag.v ] && [ ${blk}_$tag.v -nt "$dcp" ]; } || vivado -mode batch -nojournal -nolog -source fs_$tag.tcl > fs_$tag.log 2>&1 || { echo "funcsim failed"; exit 1; }
  # Black-boxed memories (frame_mem for dec_top): compile the sim models AFTER the netlist so they override the stubs.
  models=""; for m in frame_mem.sv frame_mem_w.sv frame_mem_variants.sv; do [ -f "$m" ] && models="$models $m"; done
  if [ "$SIM" = verilator ]; then
    $VGATE/vgate_build.sh ${blk}_$tag $PWD/${blk}_$tag.v $PWD/replay_$blk.sv $models > vg_$tag.build 2>&1 || { echo "netlist: verilator build failed: $(grep -m1 '%Error' $VGATE/../vgate/${blk}_$tag/build.log 2>/dev/null | cut -c1-160)"; exit 1; }
    $VGATE/vgate_run.sh ${blk}_$tag $PWD $vec +fminit=zero > vg_$tag.run 2>&1
    cp $HOME/vgate/${blk}_$tag/run_$(basename $vec).log g_$tag.xsim      # the verdict where the ladder scripts look for it
    echo "netlist (verilator): $(grep -E '^REPLAY' g_$tag.xsim || tail -n 3 vg_$tag.run | tr '\n' ' ' | cut -c1-200)"
  else
    xvlog -sv -d NETLIST ${blk}_$tag.v $models replay_$blk.sv $G > g_$tag.xvlog 2>&1 && xelab ${XELAB_OPT:-} -L unisims_ver -L secureip replay_$blk glbl -s gs_$tag > g_$tag.xelab 2>&1 && xsim gs_$tag -R -testplusarg vec=$vec -testplusarg fminit=zero > g_$tag.xsim 2>&1
    echo "netlist: $(grep -E '^REPLAY' g_$tag.xsim || tail -n 3 g_$tag.xelab g_$tag.xvlog | tr '\n' ' ' | cut -c1-200)"
  fi
fi
