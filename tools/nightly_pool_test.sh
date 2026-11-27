#!/usr/bin/env bash
# tools/nightly_pool_test.sh: checks tools/nightly_pool.sh without simulators (2026-11-27, WF-18). About 20 s.
# Two tokens; two holders take them; then waiters ask in a known order with known expected seconds (one of them dies
# while waiting). Checks: never more than two at once; tokens go longest expected first (unknown = first), ties in ask
# order; a dead waiter does not block the pool; expect / expect_build read the newest times file; PAR=1 hands out at once.
set -u
pool_sh="$(cd "$(dirname "$0")" && pwd)/nightly_pool.sh"
d=$(mktemp -d); trap 'rm -rf "$d"' EXIT; cd "$d"; mkdir logs
printf 'w50 - 10\nw20a - 3\nsuite b 7\nsuite 0 40\nsuite 1 90\n' > logs/nightly-old-times.txt; sleep 1.1
printf 'w50 - 50\nw20a - 20\nw20b - 20\nw5 - 5\nw1 - 1\nwdead - 99\n' > logs/nightly-new-times.txt
fail=0; check() { if [[ "$2" == "$3" ]]; then echo "ok   $1"; else echo "FAIL $1: got '$2', want '$3'"; fail=1; fi; }
source "$pool_sh"
pool_init 2 logs/nightly-test-times.txt
trap 'rm -rf "$d" "$pool"' EXIT   # pool_init set its own
check "expect, newer file wins"  "$(expect w50 -)" 50
check "expect, older file kept"  "$(expect suite 1)" 90
check "expect, unknown"          "$(expect nosuch -)" "$POOL_UNKNOWN"
check "expect_build = build + longest shard" "$(expect_build suite)" 97
check "expect_build, unknown"    "$(expect_build nosuch)" "$POOL_UNKNOWN"
job() {  # name expected hold
  tok_get "$2"; echo "get $1 $(date +%s%N)" >> events; sleep "$3"; echo "put $1 $(date +%s%N)" >> events; tok_put; record "$1" - "$3"
}
: > events
job h1 1 3 & job h2 1 3 & sleep 0.5
for w in w5 w50 w1 wunknown wdead w20a w20b; do
  if [[ $w == wdead ]]; then ( tok_get "$(expect wdead -)"; echo "get wdead" >> events ) & dead=$!
  else job "$w" "$(expect "$w" -)" 1 & fi
  sleep 0.2
done
sleep 0.5; kill "$dead" 2>/dev/null; pkill -P "$dead" 2>/dev/null
wait
order=$(awk '$1 == "get" && $2 != "h1" && $2 != "h2" {print $2}' events | tr '\n' ' ')
check "grant order" "$order" "wunknown w50 w20a w20b w5 w1 "
max=$(sort -k3,3n events | awk '$1 == "get" {n++} $1 == "put" {n--} n > m {m = n} END {print m}')
check "at most two at once" "$max" 2
check "tokens all back" "$(cat "$pool/free")" 2
check "times recorded" "$(grep -c . logs/nightly-test-times.txt)" 8
check "no waiter left" "$(ls "$pool" | grep -c '^w\.')" 0
pool_par=1; t0=$(date +%s%N); tok_get 5; tok_put
check "PAR=1 does not wait" "$(( ($(date +%s%N) - t0) / 1000000000 ))" 0
[[ $fail -eq 0 ]] && echo "nightly_pool_test: PASS" || { echo "nightly_pool_test: FAIL"; exit 1; }
