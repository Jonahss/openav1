#!/usr/bin/env bash
# gate_block.sh <block> <tag> <dcp|-> <vec.hex> [rtl files for the RTL control...]
# Replays <vec.hex> (+ .out) with ~/gate/<block>/replay_<block>.sv against the RTL (control) and the funcsim netlist of <dcp>.
set -u
source /opt/Xilinx/2025.2.1/Vivado/settings64.sh
blk=$1; tag=$2; dcp=$3; vec=$4; shift 4
cd ~/gate/$blk
G=/opt/Xilinx/2025.2.1/Vivado/data/verilog/src/glbl.v
if [ $# -gt 0 ]; then
  xvlog -sv -d RTL_CONTROL "$@" replay_$blk.sv > rtl_$tag.xvlog 2>&1 && xelab -timescale 1ns/1ps replay_$blk -s rtl_$tag > rtl_$tag.xelab 2>&1 && xsim rtl_$tag -R -testplusarg vec=$vec -testplusarg xmask=1 > rtl_$tag.xsim 2>&1
  echo "RTL control: $(grep -E '^REPLAY' rtl_$tag.xsim || tail -3 rtl_$tag.xelab rtl_$tag.xvlog | tr '\n' ' ' | cut -c1-200)"
fi
if [ "$dcp" != "-" ]; then
  printf "open_checkpoint %s\nwrite_verilog -force -mode funcsim %s/%s_%s.v\nexit\n" "$dcp" "$PWD" "$blk" "$tag" > fs_$tag.tcl
  [ -f ${blk}_$tag.v ] || vivado -mode batch -nojournal -nolog -source fs_$tag.tcl > fs_$tag.log 2>&1 || { echo "funcsim failed"; exit 1; }
  # Black-boxed memories (frame_mem for dec_top): compile the sim models AFTER the netlist so they override the stubs.
  models=""; for m in frame_mem.sv frame_mem_w.sv frame_mem_variants.sv; do [ -f "$m" ] && models="$models $m"; done
  xvlog -sv -d NETLIST ${blk}_$tag.v $models replay_$blk.sv $G > g_$tag.xvlog 2>&1 && xelab -L unisims_ver -L secureip replay_$blk glbl -s gs_$tag > g_$tag.xelab 2>&1 && xsim gs_$tag -R -testplusarg vec=$vec > g_$tag.xsim 2>&1
  echo "netlist: $(grep -E '^REPLAY' g_$tag.xsim || tail -3 g_$tag.xelab g_$tag.xvlog | tr '\n' ' ' | cut -c1-200)"
fi
