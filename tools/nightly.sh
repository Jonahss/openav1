#!/usr/bin/env bash
# Nightly regression (run from cron on the batch host): RTL syntax decoder on generated streams, the full decoder
# on generated frames, and the corpus through the whole RTL pipeline compared with dav1d's pictures (model-free).
# Writes logs/nightly-<date>.txt with one line per suite and exits non-zero on any failure.
# Usage: tools/nightly.sh [tag]      (needs bootstrap.sh done and refout/ populated by tools/regress.sh once)
set -uo pipefail
cd "$(dirname "$0")/.."
source env.sh
tag="${1:-$(date +%Y-%m-%d)}"
out="logs/nightly-$tag.txt"
mkdir -p logs
: > "$out"
rc=0
run() {  # name, env..., -- command
  local name="$1"; shift
  local t0=$(date +%s)
  local log="logs/nightly-$tag-$name.log"
  if env "$@" > "$log" 2>&1 && grep -q "OK:" "$log"; then
    printf "PASS %-22s %5ss  %s\n" "$name" "$(( $(date +%s) - t0 ))" "$(grep -o 'OK: {.*}' "$log" | tail -1 | cut -c1-140)" >> "$out"
  else
    printf "FAIL %-22s %5ss  %s\n" "$name" "$(( $(date +%s) - t0 ))" "$(grep -m1 'AssertionError\|%Error' "$log" | cut -c1-160)" >> "$out"
    rc=1
  fi
}
run tile_syntax_plain  TS_SCREEN=0 TS_SEEDS=12 python tb/runner.py tile_syntax
run tile_syntax_screen TS_SCREEN=1 TS_SEEDS=24 python tb/runner.py tile_syntax
run dec_top_small      TS_SEEDS=12 python tb/runner.py dec_top
run dec_top_large      TS_SEEDS=101,102,103,104,105,106 TS_W=256 TS_H=160 python tb/runner.py dec_top
IVFS=$(ls "$PWD"/streams/synth/*.ivf "$PWD"/streams/synth2/*.ivf | paste -sd,)
run corpus_full_rtl    TS_NOMODEL=1 TS_IVF="$IVFS" python tb/runner.py dec_top
echo "DONE rc=$rc $(date -Is)" >> "$out"
cat "$out"
exit $rc
