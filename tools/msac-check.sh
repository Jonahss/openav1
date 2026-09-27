#!/usr/bin/env bash
# Replay dav1d symbol traces through the msac RTL.  Usage: tools/msac-check.sh <stream.obu>...
# Traces are cached in traces/<name>.s1.txt.  Prints one PASS/FAIL line per stream.
set -uo pipefail
cd "$(dirname "$0")/.."
source env.sh
mkdir -p traces
rc=0
for st in "$@"; do
  name=$(basename "$st" .obu)
  tr="$PWD/traces/$name.s1.txt"
  [ -s "$tr" ] || tools/trace.sh "$st" "$tr" 1
  nsym=$(grep -c '^S' "$tr"); ntile=$(grep -c '^T' "$tr"); nupd=$(grep -c '^T [0-9]* 0 ' "$tr")
  if [ "$nsym" -gt "${MSAC_MAX_TRACE_SYMS:-2000000}" ]; then echo "SKIP $name symbols=$nsym (raise MSAC_MAX_TRACE_SYMS to force)"; continue; fi
  out=$(MSAC_TRACE="$tr" timeout 3600 python tb/runner.py msac 2>&1)
  if grep -q 'TESTS=1 PASS=1' <<<"$out"; then
    secs=$(grep 'replay_trace *PASS' <<<"$out" | awk '{print $(NF-2)}')
    echo "PASS $name tiles=$ntile adapt_tiles=$nupd symbols=$nsym sim=${secs}s"
  else
    echo "FAIL $name tiles=$ntile symbols=$nsym"; grep -m1 -A3 'AssertionError\|Error' <<<"$out" | head -6; rc=1
  fi
done
exit $rc
