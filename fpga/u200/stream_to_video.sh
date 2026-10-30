#!/usr/bin/env bash
# fpga/u200/stream_to_video.sh <stream.ivf|.obu> <out.mp4> [decode.py options...]
# An AV1 stream through the decoder on the Alveo U200 and out as a watchable video: every shown picture the card
# returns over PCIe is written as YUV4MPEG2 (tools/kv260/decode.py --y4m), then ffmpeg encodes that to H.264 so it
# plays anywhere. The pictures in the file are the card's own output, not dav1d's: pass --ref <dav1d .yuv> to also
# compare them frame by frame on the way (2026-10-30, for the AOM symposium demo).
set -euo pipefail
[ $# -ge 2 ] || { echo "usage: $0 <stream> <out.mp4> [decode.py options]" >&2; exit 2; }
stream=$1; out=$2; shift 2
here=$(cd "$(dirname "$0")" && pwd); root=$(cd "$here/../.." && pwd)
cd "$root"; source env.sh > /dev/null 2>&1 || true
export PYTHONPATH=$root/tools:$root/tb:$root/tools/kv260
y4m=${out%.*}.y4m
echo "== $stream -> U200 (decode.py --xdma 0 --inter) -> $y4m"
PY=$(command -v python || command -v python3)
"$PY" tools/kv260/decode.py "$stream" --xdma 0 --inter --frames 100000 --y4m "$y4m" "$@"
echo "== $y4m -> ffmpeg (libx264 crf 18) -> $out"
ffmpeg -hide_banner -loglevel error -y -i "$y4m" -c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p "$out"
ls -la "$y4m" "$out"
echo "play: ffplay $y4m   (the card's raw pictures)   or   $out"
