#!/usr/bin/env bash
# Synthesise dec_top (or one block: -t lr_top etc.) for the Kria K26 with Vivado:
#   synth/vivado/run.sh [-t top] <FBX> <FBY> [tag]
# Needs Vivado on PATH (source /opt/Xilinx/<ver>/Vivado/settings64.sh). Reports land in
# ${OPENAV1_SYNTH_OUT:-$HOME/openav1-synth}/<top>-<tag>/ (util.rpt, util_hier.rpt, ram.rpt, timing.rpt,
# post_synth.dcp); the log is next to them. The frame buffer is a black box (frame_mem_bb.sv): see that file.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
top=dec_top
if [ "${1:-}" = "-t" ]; then top=$2; shift 2; fi
fbx=${1:?FBX}; fby=${2:?FBY}
tag=${3:-$(git -C "$root" rev-parse --short HEAD)-fb${fbx}x${fby}}
out=${OPENAV1_SYNTH_OUT:-$HOME/openav1-synth}/$top-$tag
# generics the block takes, in the convention of tb/runner.py: FBX/FBY = log2 pixels, ML2R/ML2C = log2 4x4 units
generics=""
grep -q "parameter int FBX" "$root/rtl/$top.sv" && generics="$generics -generic FBX=$fbx -generic FBY=$fby"
grep -q "parameter int ML2R" "$root/rtl/$top.sv" && generics="$generics -generic ML2R=$((fby - 2)) -generic ML2C=$((fbx - 2))"
mkdir -p "$out"
# rtl in dependency order; keep in step with the dec_top entry of tb/runner.py (packages first, leaves before users)
files="cdf_map_pkg tx_tables_pkg blk_tables_pkg q_tables_pkg syn_pkg scan_rom cdf_store msac sym_seq coef_rd blk_ctx
       pal_syntax mv_mem mv_stack blk_syntax lr_syntax tile_syntax itx_ucode cos128_lut itx1d itx2d ipred cfl qm_rom
       frame_mem_w recon_top cdef_pkg lr_pkg mi_store rec_fifo lf_top cdef_top lr_top sr_top dec_top"
sources="$here/frame_mem_bb.sv"
for f in $files; do [ -f "$root/rtl/$f.sv" ] || { echo "missing rtl/$f.sv" >&2; exit 1; }; sources="$sources $root/rtl/$f.sv"; done
export SYNTH_ROOT=$root SYNTH_OUT=$out SYNTH_TOP=$top SYNTH_GENERICS=$generics SYNTH_SOURCES=$sources
# qm_rom loads rtl/qm_rom.hex by a path relative to a simulation build dir (../../rtl/): give Vivado the same shape
mkdir -p "$out/work/run"; ln -sfn "$root/rtl" "$out/rtl"
echo "$top FBX=$fbx FBY=$fby ($generics) -> $out"
cd "$out/work/run"
exec vivado -mode batch -nojournal -log "$out/vivado.log" -source "$here/synth.tcl"
