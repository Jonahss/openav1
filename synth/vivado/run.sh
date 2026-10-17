#!/usr/bin/env bash
# Synthesise dec_top (or one block: -t lr_top etc.) for the Kria K26 with Vivado:
#   synth/vivado/run.sh [-t top] [-b mod,mod,...] [-n mod,mod,...] [-f bb|pr] [-g NAME=VAL ...] <FBX> <FBY> [tag]
# -b replaces the named modules by black-box stubs (stub.py), to measure a block without its children;
# -n replaces them by null stubs (outputs low) and -f pr uses the frame-memory stand-in, for implementation
# trials (synth/vivado/impl.sh) where black boxes are not allowed.
# Needs Vivado on PATH (source /opt/Xilinx/<ver>/Vivado/settings64.sh). Reports land in
# ${OPENAV1_SYNTH_OUT:-$HOME/openav1-synth}/<top>-<tag>/ (util.rpt, util_hier.rpt, ram.rpt, timing.rpt,
# post_synth.dcp); the log is next to them. The frame buffer is a black box (frame_mem_bb.sv): see that file.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
top=dec_top; bb=""; nb=""; extra=""; fm=bb
while [ "${1:-}" = "-t" ] || [ "${1:-}" = "-b" ] || [ "${1:-}" = "-n" ] || [ "${1:-}" = "-g" ] || [ "${1:-}" = "-f" ]; do
    case "$1" in -t) top=$2 ;; -b) bb=${2//,/ } ;; -n) nb=${2//,/ } ;; -g) extra="$extra -generic $2" ;; -f) fm=$2 ;; esac; shift 2
done
fbx=${1:?FBX}; fby=${2:?FBY}
tag=${3:-$(git -C "$root" rev-parse --short HEAD)-fb${fbx}x${fby}${bb:+-bb}${nb:+-nb}}
out=${OPENAV1_SYNTH_OUT:-$HOME/openav1-synth}/$top-$tag
# generics the block takes, in the convention of tb/runner.py: FBX/FBY = log2 pixels, ML2R/ML2C = log2 4x4 units
generics=""
grep -q "parameter int FBX" "$root/rtl/$top.sv" && generics="$generics -generic FBX=$fbx -generic FBY=$fby"
grep -q "parameter int ML2R" "$root/rtl/$top.sv" && generics="$generics -generic ML2R=$((fby - 2)) -generic ML2C=$((fbx - 2))"
generics="$generics$extra"
mkdir -p "$out"
# rtl in dependency order; keep in step with the dec_top entry of tb/runner.py (packages first, leaves before users)
files="cdf_map_pkg tx_tables_pkg blk_tables_pkg q_tables_pkg syn_pkg scan_rom cdf_store msac sym_seq sym_path coef_rd blk_ctx
       pal_syntax mv_mem mv_stack blk_syntax lr_syntax tile_syntax itx_pair itx_lvl itx_row itx2d ipred cfl qm_rom
       frame_mem_w frame_mem_uram recon_top cdef_pkg lr_pkg mi_store rec_fifo lf_top cdef_top lr_top sr_top dec_top"
# -t dec_host: the AXI-Lite register file + DMA engine around dec_top (its sources plus itself), e.g. for a
# whole-decoder netlist replay of a window configuration, where only dec_host's engine can drive flush_done.
[ "$top" = dec_host ] && files="$files dec_host"
sources="$here/frame_mem_$fm.sv"                                   # bb: black box (default); pr: place-and-route stand-in
mkdir -p "$out/stubs"
for f in $files; do
    [ -f "$root/rtl/$f.sv" ] || { echo "missing rtl/$f.sv" >&2; exit 1; }
    case " $bb " in
        *" $f "*) python3 "$here/stub.py" "$root/rtl/$f.sv" > "$out/stubs/$f.sv"; sources="$sources $out/stubs/$f.sv"; continue ;;
    esac
    case " $nb " in
        *" $f "*) python3 "$here/stub.py" --null "$root/rtl/$f.sv" > "$out/stubs/$f.sv"; sources="$sources $out/stubs/$f.sv" ;;
        *) sources="$sources $root/rtl/$f.sv" ;;
    esac
done
export SYNTH_ROOT=$root SYNTH_OUT=$out SYNTH_TOP=$top SYNTH_GENERICS=$generics SYNTH_SOURCES=$sources
grep -qE "input +logic +clk2x" "$root/rtl/$top.sv" && export SYNTH_CLK2X=1   # a double-pumped memory: 5 ns clock on clk2x
# qm_rom loads rtl/qm_rom.hex by a path relative to a simulation build dir (../../rtl/): give Vivado the same shape
mkdir -p "$out/work/run"; ln -sfn "$root/rtl" "$out/rtl"
echo "$top FBX=$fbx FBY=$fby ($generics) -> $out"
cd "$out/work/run"
exec vivado -mode batch -nojournal -log "$out/vivado.log" -source "$here/synth.tcl"
