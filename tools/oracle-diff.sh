#!/usr/bin/env bash
# Decode one AV1 stream with dav1d and with libaom; compare the raw YUV output.
# Usage: tools/oracle-diff.sh <stream.obu|.ivf> [outdir]
# Exit 0 = both decoders agree byte-for-byte, 1 = mismatch, 2 = a decoder failed.
set -uo pipefail
source "$(dirname "$0")/../env.sh"
in="$1"; out="${2:-$(mktemp -d)}"; mkdir -p "$out"
base="$(basename "${in%.*}")"

t0=$(date +%s.%N)
dav1d -q -i "$in" -o "$out/$base.dav1d.yuv" --muxer yuv --threads 1 >"$out/$base.dav1d.log" 2>&1
rc1=$?; t1=$(date +%s.%N)
# Annex B streams (Argon) have the forbidden bit set in byte 0 when read as an OBU header
annexb=""; [ $(( $(od -An -tu1 -N1 "$in") & 0x80 )) -ne 0 ] && annexb="--annexb"
aomdec $annexb --rawvideo -o "$out/$base.aom.yuv" "$in" >"$out/$base.aom.log" 2>&1
rc2=$?; t2=$(date +%s.%N)

if [ $rc1 -ne 0 ] || [ $rc2 -ne 0 ]; then
  echo "FAIL $base dav1d_rc=$rc1 aom_rc=$rc2"; exit 2
fi
m1=$(md5sum < "$out/$base.dav1d.yuv" | cut -c1-32)
m2=$(md5sum < "$out/$base.aom.yuv" | cut -c1-32)
sz=$(stat -c %s "$out/$base.dav1d.yuv")
printf "%s %s bytes=%s dav1d=%.2fs aom=%.2fs md5=%s\n" \
  "$([ "$m1" = "$m2" ] && echo MATCH || echo MISMATCH)" "$base" "$sz" \
  "$(echo "$t1 - $t0" | bc)" "$(echo "$t2 - $t1" | bc)" "$m1"
[ "$m1" = "$m2" ]
