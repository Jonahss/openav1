#!/usr/bin/env bash
# Regression over the synthetic corpus: trace each stream with the patched dav1d (Pi build), then
#   1. xcheck_trace.py  : intra / CfL / inverse-transform models vs dav1d, block by block
#   2. msac replay      : arithmetic decoder RTL vs every symbol
# Usage: tools/regress.sh [stream.ivf ...]   (default: streams/synth/*.ivf). Bit depth from the file name.
set -uo pipefail
cd "$(dirname "$0")/.."
export LD_LIBRARY_PATH=$PWD/prefix-trace/lib/aarch64-linux-gnu:$PWD/prefix-trace/lib
source venv/bin/activate
mkdir -p traces
rc=0
[ $# -eq 0 ] && set -- streams/synth/*.ivf
for st in "$@"; do
  name=$(basename "$st" .ivf); bd=8; [[ $name == *10bit* ]] && bd=10; [[ $name == *12bit* ]] && bd=12
  tr=traces/$name.txt
  mkdir -p refout
  DAV1D_TRACE=$tr DAV1D_TRACE_MASK=15 prefix-trace/bin/dav1d -q -i "$st" -o refout/$name.yuv --threads 1 >/dev/null 2>&1 || { echo "FAIL $name: dav1d"; rc=1; continue; }
  x=$(python3 tools/xcheck_trace.py "$tr" $bd 2>&1); xr=$?
  tl=$(timeout 3600 python3 tools/xcheck_tile.py "$tr" 2>&1 | tail -1)
  tn=$(sed -nE 's#^([0-9]+)/([0-9]+) tiles bit-exact.*#\1 \2#p' <<<"$tl")
  if [ -z "$tn" ] || [ "${tn% *}" != "${tn#* }" ]; then xr=1; x="$x$tl"; fi
  rc_=$(timeout 3600 python3 tools/xcheck_recon.py "$tr" 2>&1 | tail -1); [ "$rc_" = "OK" ] || { xr=1; x="$x recon:$rc_"; }
  fr_="n/a"
  if [[ $name == *lf_only* ]]; then fr_=$(timeout 3600 python3 tools/xcheck_frame.py "$tr" refout/$name.yuv --stage lf 2>&1 | tail -1); [ "$fr_" = "OK" ] || { xr=1; x="$x frame:$fr_"; }; fi
  summary=$(grep -E "^(pred|cfl|itx):" <<<"$x" | sed 's/ checked, / chk /; s/ mismatches/ bad/' | paste -sd';'); summary="$summary; tile: $tl; recon: $rc_; frame: $fr_"
  m=$(MSAC_TRACE=$PWD/$tr timeout 3600 python tb/runner.py msac 2>&1)
  ms=$(grep -oE "OK: [0-9]+ symbols" <<<"$m" | head -1); [ -n "$ms" ] || { ms="msac FAIL"; xr=1; }
  ip=$(IPRED_TRACE=$PWD/$tr IPRED_BD=$bd timeout 3600 python tb/runner.py ipred test_ipred_trace 2>&1)
  ips=$(grep -oE "OK: [0-9]+ prediction blocks" <<<"$ip" | head -1); [ -n "$ips" ] || { ips="ipred FAIL"; xr=1; }
  ms="$ms; ipred $ips"; m="$m$ip"
  if [ $xr -eq 0 ]; then echo "PASS $name ($bd-bit): $summary; $ms"; else echo "FAIL $name ($bd-bit): $summary; $ms"; grep -m3 "MISMATCH\|BAD\|Assertion" <<<"$x$m"; rc=1; fi
done
exit $rc
