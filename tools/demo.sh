#!/usr/bin/env bash
# Showcase demo: decode one AV1 picture with the RTL (Verilator), compare it pixel for pixel with dav1d,
# and render the two pictures side by side.
#
#   tools/demo.sh [stream.ivf] [out-dir]
#
# Defaults: streams/demo/wyldcard_devkit.ivf, out-dir build/demo. The reference picture is
# refout/<stream>.yuv (dav1d's output, made by tools/regress.sh or tools/trace.sh). Needs `source env.sh`.
# What it prints is meant to be read aloud: the picture size, the simulated cycles, and the comparison.
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT=$PWD
STREAM=${1:-$ROOT/streams/demo/wyldcard_devkit.ivf}
OUT=${2:-$ROOT/build/demo}
[ "${STREAM#/}" = "$STREAM" ] && STREAM=$ROOT/$STREAM
name=$(basename "$STREAM" .ivf)
REF=${TS_REF_DIR:-$ROOT/refout}
if [ ! -f "$REF/$name.yuv" ]; then           # dav1d's picture is the reference; refout/ is not checked in
  mkdir -p "$REF"
  prefix-trace/bin/dav1d -q -i "$STREAM" -o "$REF/$name.yuv" --threads 1 || { echo "dav1d failed on $STREAM"; exit 1; }
fi
mkdir -p "$OUT"
LOG=$OUT/$name.log

bold() { printf '\033[1m%s\033[0m\n' "$*"; }
bold "openav1: AV1 intra decoder in SystemVerilog, simulated with Verilator on this machine"
echo "stream:    ${STREAM#$ROOT/} ($(stat -c %s "$STREAM") bytes of AV1)"
echo "reference: dav1d's decoded picture, ${REF#$ROOT/}/$name.yuv"
echo "mode:      the in-loop filters trail the tile decoder by superblock rows (pipelined)"
echo
t0=$(date +%s)
TS_NOMODEL=1 TS_PIPE=1 TS_STAGE=lr SIM_FAST=1 TS_REF_DIR="$REF" TS_DUMP="$OUT" TS_IVF="$STREAM" TS_VIS="$OUT/$name.vis.txt" \
  python tb/runner.py dec_top > "$LOG" 2>&1 &
pid=$!
# progress: the testbench logs each tile and each frame; echo those lines as they appear
tail -n0 -F "$LOG" 2>/dev/null | grep --line-buffered -oE "$name[^:]*: (tile done in ~[0-9]+ cycles|pipelined filters finished ~[0-9]+ cycles after the last tile|identical to dav1d .*\]\))|FAIL .*" &
tp=$!
wait $pid || { kill $tp 2>/dev/null; echo; echo "decode FAILED, see $LOG"; exit 1; }
sleep 0.5; kill $tp 2>/dev/null; wait $tp 2>/dev/null || true
t1=$(date +%s)
echo
tile=$(grep -oE "tile done in ~[0-9]+" "$LOG" | grep -oE "[0-9]+$" | paste -sd+ | bc)
tail_c=$(grep -oE "pipelined filters finished ~[0-9]+" "$LOG" | grep -oE "[0-9]+$" | paste -sd+ | bc)
size=$(grep -oE "identical to dav1d \([0-9]+x[0-9]+ bd[0-9]+" "$LOG" | head -1 | grep -oE "[0-9]+x[0-9]+ bd[0-9]+")
W=${size%%x*}; H=${size#*x}; H=${H%% *}; BD=${size##*bd}
px=$((W * H)); total=$((tile + tail_c))
bold "result"
printf '  picture:          %sx%s, %s-bit 4:2:0\n' "$W" "$H" "$BD"
printf '  simulated cycles: %s (tile decoder %s + filter drain %s) = %.2f cycles per pixel\n' "$total" "$tile" "$tail_c" "$(echo "$total / $px" | bc -l)"
printf '  wall time:        %ds of Verilator simulation\n' "$((t1 - t0))"
python3 tools/cmp_dump_refout.py "$OUT" "$REF" | sed 's/^/  /'
# pictures: RTL | dav1d
rtl=$OUT/$name.ivf_frame_0.rtl.yuv
if [ -f "$rtl" ]; then
  fmt=yuv420p; [ "$BD" != 8 ] && fmt=yuv420p16le
  ffmpeg -v error -y -f rawvideo -pixel_format $fmt -video_size ${W}x${H} -i "$rtl" -frames:v 1 "$OUT/$name.rtl.png"
  ffmpeg -v error -y -f rawvideo -pixel_format $fmt -video_size ${W}x${H} -i "$REF/$name.yuv" -frames:v 1 "$OUT/$name.dav1d.png"
  ffmpeg -v error -y -i "$OUT/$name.rtl.png" -i "$OUT/$name.dav1d.png" \
    -filter_complex "[0:v]pad=iw+8:ih:0:0:color=white[l];[l][1:v]hstack" "$OUT/$name.side.png"
  echo "  pictures:         ${OUT#$ROOT/}/$name.side.png  (left: RTL, right: dav1d)"
fi
