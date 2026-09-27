#!/usr/bin/env bash
# Run oracle-diff over every .obu under an Argon set dir; also check dav1d output vs md5_ref / md5_no_film_grain.
# Usage: tools/oracle-all.sh <argon-set-dir> <results.txt>
set -uo pipefail
dir="$1"; res="$2"; : > "$res"
out=/tmp/openav1-oracle; mkdir -p /tmp/openav1-oracle
t0=$(date +%s)
for f in "$dir"/streams/*.obu; do
  b=$(basename "${f%.obu}")
  line=$("$(dirname "$0")/oracle-diff.sh" "$f" "$out" 2>&1 | tail -1)
  got=$(md5sum < "$out/$b.dav1d.yuv" 2>/dev/null | cut -c1-32)
  ref=$(cut -d" " -f1 "$dir/md5_ref/$b.md5" 2>/dev/null)
  nfg=$(cut -d" " -f1 "$dir/md5_no_film_grain/$b.md5" 2>/dev/null)
  tag="ref=?"; [ -n "$ref" ] && { [ "$got" = "$ref" ] && tag="ref=OK" || tag="ref=DIFF"; }
  [ -n "$nfg" ] && [ "$got" = "$nfg" ] && tag="$tag nfg=OK"
  echo "$line $tag" >> "$res"
  find /tmp/openav1-oracle -maxdepth 1 -name "*.yuv" -delete
done
echo "TOTAL_SECONDS $(( $(date +%s) - t0 )) streams=$(wc -l < "$res") match=$(grep -c ^MATCH "$res") mismatch=$(grep -c ^MISMATCH "$res") fail=$(grep -c ^FAIL "$res") refok=$(grep -c "ref=OK" "$res")" >> "$res"
echo ORACLE_ALL_DONE >> "$res"
