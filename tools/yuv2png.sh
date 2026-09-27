#!/usr/bin/env bash
# Render a dumped frame (tb/test_dec_top.py with TS_DUMP=<dir>) to PNG with ffmpeg.
# Usage: tools/yuv2png.sh <dump-dir>/<tag>   -> <tag>.rtl.png, <tag>.model.png, <tag>.side.png (RTL | model)
# The <tag>.txt file holds: width height bitdepth ssx ssy planes.
set -euo pipefail
base="$1"
read -r W H BD SSX SSY NP < "$base.txt"
if [ "$NP" = 1 ]; then fmt=gray; [ "$BD" != 8 ] && fmt=gray16le
elif [ "$SSX" = 1 ] && [ "$SSY" = 1 ]; then fmt=yuv420p; [ "$BD" != 8 ] && fmt=yuv420p16le
elif [ "$SSX" = 1 ]; then fmt=yuv422p; [ "$BD" != 8 ] && fmt=yuv422p16le
else fmt=yuv444p; [ "$BD" != 8 ] && fmt=yuv444p16le
fi
# for >8-bit the samples sit in the low BD bits of 16-bit words: scale them up to full range for viewing
vf="null"; [ "$BD" != 8 ] && vf="lutyuv=y=val*$((1 << (16 - BD))):u=val*$((1 << (16 - BD))):v=val*$((1 << (16 - BD)))"
for side in rtl model; do
  ffmpeg -v error -y -f rawvideo -pixel_format "$fmt" -video_size "${W}x${H}" -i "$base.$side.yuv" -vf "$vf" -frames:v 1 "$base.$side.png"
done
ffmpeg -v error -y -i "$base.rtl.png" -i "$base.model.png" -filter_complex "[0:v]pad=iw+8:ih:0:0:color=white[l];[l][1:v]hstack" "$base.side.png"
echo "$base.rtl.png $base.model.png $base.side.png"
