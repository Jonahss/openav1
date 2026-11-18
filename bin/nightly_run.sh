#!/usr/bin/env bash
# nightly_run.sh <worktree> <tag> [VAR=value ...]  (2026-11-17): every run of tools/nightly.sh goes through here, so it
# has a ticket on the Builds board: opened at the start, closed with the PASS / FAIL counts. Extra VAR=value words are
# the nightly's environment (NIGHTLY_ONLY=..., NIGHTLY_PAR=..., ...). CHG=CHG-n in the environment links the change under test.
set -u
W=$1; tag=$2; shift 2
cd "$W" || exit 1
h=$(git rev-parse --short HEAD); mkdir -p logs
BLD=$($(dirname "$0")/vk build-open nightly "nightly $tag" --commit "$h" --config "${*:-all suites} NIGHTLY_PAR=${NIGHTLY_PAR:-12}" --output "atium $W/logs/nightly-$tag.txt" --changes "${CHG:-}")
env NIGHTLY_PAR="${NIGHTLY_PAR:-12}" "$@" tools/nightly.sh "$tag" > "logs/nightly-$tag.out" 2>&1
rc=$?
p=$(grep -c ^PASS "logs/nightly-$tag.txt" 2>/dev/null); f=$(grep -c ^FAIL "logs/nightly-$tag.txt" 2>/dev/null)
if [ -n "$BLD" ]; then
  if [ "$rc" = 0 ] && [ "${f:-1}" = 0 ] && [ "${p:-0}" -gt 0 ] && grep -q "^DONE rc=0" "logs/nightly-$tag.txt"; then st=Passed; else st=Failed; fi
  $(dirname "$0")/vk build-close "$BLD" $st "PASS ${p:-0} FAIL ${f:-?} rc=$rc; $(grep ^DONE "logs/nightly-$tag.txt" | tail -1); failing: $(grep ^FAIL "logs/nightly-$tag.txt" | awk '{print $2}' | tr '\n' ' ' | cut -c1-200)" > /dev/null
fi
exit $rc
