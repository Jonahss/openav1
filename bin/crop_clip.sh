#!/usr/bin/env bash
# crop_clip.sh <pictures.yuv> <width> <height> <out.ivf> [x y w h frames cq]: a small clip with the character of a big
# one, for simulation. A window is cut out of the decoded pictures of a stream (not scaled, so blocks, motion and
# references keep their sizes) and encoded again with warped / global motion and OBMC off (aomenc, good quality, sb64).
# Check it with tools/fetch_census.py (block sizes, compound share) against the original before trusting it.
# 2026-11-21: a 1024x576 window of a 2160p YouTube stream ran at 804 clocks and 386 read beats a strip in simulation,
# the card on the whole stream at 806 and 383. The clips stay outside the repository (they are cut from other
# people's video): ~/measure4k/crop on the build machine.
set -eu
[ $# -ge 4 ] || { echo "usage: crop_clip.sh <pictures.yuv> <width> <height> <out.ivf> [x y w h frames cq]" >&2; exit 2; }
yuv=$1; W=$2; H=$3; out=$4; x=${5:-1408}; y=${6:-768}; w=${7:-1024}; h=${8:-576}; n=${9:-12}; cq=${10:-44}
tmp=$(mktemp --suffix=.y4m)
ffmpeg -v error -y -f rawvideo -pix_fmt yuv420p -s ${W}x${H} -r 60 -i "$yuv" -frames:v $n -vf "crop=$w:$h:$x:$y" -pix_fmt yuv420p "$tmp"
aomenc "$tmp" -o "$out" --ivf --good --cpu-used=3 --passes=2 --lag-in-frames=19 --auto-alt-ref=1 --end-usage=q --cq-level=$cq --sb-size=64 \
    --enable-warped-motion=0 --enable-global-motion=0 --enable-obmc=0 --tile-columns=0 --threads=8 --limit=$n 2>&1 | tail -1 | tr '\r' '\n' | tail -1
rm -f "$tmp"; ls -l "$out"
