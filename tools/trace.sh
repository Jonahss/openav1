#!/usr/bin/env bash
# Produce the golden trace for one stream with the trace-hooked, asm-free dav1d (prefix-trace/).
# Usage: tools/trace.sh <stream.ivf|.obu> <trace.txt> [mask] [out.yuv]
#   mask bits: 1 symbols, 2 coefficients, 4 prediction, 8 reconstruction (default 15 = everything)
#   out.yuv: also keep dav1d's decoded picture (raw planar YUV), for the frame-level checks
# Format: see the docstring of tools/dav1d_trace_patch.py.
set -euo pipefail
source "$(dirname "$0")/../env.sh"
out="${4:-/dev/null}"
muxer="yuv"; [ "$out" = "/dev/null" ] && muxer="null"
# DAV1D_EXTRA: extra dav1d options, e.g. "--filmgrain 0" to write the picture without film grain (Argon md5_no_film_grain)
DAV1D_TRACE="$2" DAV1D_TRACE_MASK="${3:-15}" "$DAV1D_TRACE_BIN" -q -i "$1" -o "$out" --muxer "$muxer" --threads 1 ${DAV1D_EXTRA:-}
