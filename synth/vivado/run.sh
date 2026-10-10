#!/usr/bin/env bash
# Synthesise dec_top for the Kria K26 with Vivado: synth/vivado/run.sh <FBX> <FBY> [tag]
# Needs Vivado on PATH (source /opt/Xilinx/<ver>/Vivado/settings64.sh). Reports land in
# ${OPENAV1_SYNTH_OUT:-$HOME/openav1-synth}/dec_top-<tag>/ (util.rpt, util_hier.rpt, ram.rpt, timing.rpt,
# post_synth.dcp); the log is next to them. The frame buffer is a black box (frame_mem_bb.sv): see that file.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
fbx=${1:?FBX}; fby=${2:?FBY}
tag=${3:-$(git -C "$root" rev-parse --short HEAD)-fb${fbx}x${fby}}
out=${OPENAV1_SYNTH_OUT:-$HOME/openav1-synth}/dec_top-$tag
mkdir -p "$out"
# rtl in dependency order; keep in step with the dec_top entry of tb/runner.py (packages first, leaves before users)
files="cdf_map_pkg tx_tables_pkg blk_tables_pkg q_tables_pkg syn_pkg scan_rom cdf_store msac sym_seq coef_rd blk_ctx
       pal_syntax mv_mem mv_stack blk_syntax lr_syntax tile_syntax itx_ucode cos128_lut itx1d itx2d ipred cfl qm_rom
       frame_mem_w recon_top cdef_pkg lr_pkg mi_store rec_fifo lf_top cdef_top lr_top sr_top dec_top"
sources="$here/frame_mem_bb.sv"
for f in $files; do [ -f "$root/rtl/$f.sv" ] || { echo "missing rtl/$f.sv" >&2; exit 1; }; sources="$sources $root/rtl/$f.sv"; done
export SYNTH_ROOT=$root SYNTH_OUT=$out SYNTH_FBX=$fbx SYNTH_FBY=$fby SYNTH_SOURCES=$sources
# qm_rom loads rtl/qm_rom.hex by a path relative to a simulation build dir (../../rtl/): give Vivado the same shape
mkdir -p "$out/work/run"; ln -sfn "$root/rtl" "$out/rtl"
echo "dec_top FBX=$fbx FBY=$fby -> $out"
cd "$out/work/run"
exec vivado -mode batch -nojournal -log "$out/vivado.log" -source "$here/dec_top.tcl"
