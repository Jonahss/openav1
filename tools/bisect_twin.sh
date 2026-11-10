#!/usr/bin/env bash
# bisect_twin.sh <stream> <frame|all> <sha> [sha...]  (2026-11-07, Claw)
# Fan out the dec_host inter twin on one Argon stream over several commits in parallel: one worktree per commit under
# ~/bisect/wt-<sha> (wt-setup.sh makes it runnable: venv/toolchain links), the card-like geometry (FBX/FBY 11, sb128,
# pool 10), stage B unless STAGE_A=1 (the stage-B twin cannot model a frame inheriting CDFs after a frame that
# disabled the frame-end update; use STAGE_A=1 then). frame = N runs only that frame on the RTL (TS_FRAMES; the
# rest model-only), 'all' the whole stream. Verdicts in ~/measure1080/bisect-<stream>-<tag>.txt, one line per commit.
# Env: EXTRA="..." extra env for the twin, TAG=<name> for the verdict file (default: date-time).
set -uo pipefail
stream=$1; frame=$2; shift 2
A=/home/jonahss/openav1/streams/argon/profile0_core/streams
[ -f $A/$stream.obu ] || { echo "no stream $A/$stream.obu"; exit 1; }
tag=${TAG:-$(date +%H%M%S)}; V=~/measure1080/bisect-$stream-$tag.txt; : > $V
# TE=2 REF_RING=8 = the card image (2026-11-10: three census-15 streams passed the TE=1 twin on the broken commit and failed with the card configuration)
ENV="FBX=11 FBY=11 ITX_LANES=2 FRAME_URAM=1 FRAME_WIN=5 FRAME_WIN1=3 FRAME_WIN2=3 TS_POOL=10 SB128=1 TE=${TE:-2} REF_RING=${REF_RING:-8} ${EXTRA:-}"
[ "${STAGE_A:-0}" = 1 ] || ENV="$ENV TS_FSTATE_B=1"
[ "$frame" = all ] || ENV="$ENV TS_FRAMES=$frame"
echo "bisect $stream frame $frame: $* ($(date -u +%H:%MZ), env $ENV)" >> $V
for sha in "$@"; do
  wt=~/bisect/wt-$sha
  ~/bin/wt-setup.sh $wt $sha > ~/logs/bisect-setup-$sha.log 2>&1 || { echo "$sha: SETUP FAILED ($(tail -n 1 ~/logs/bisect-setup-$sha.log | cut -c1-80))" >> $V; continue; }
  ( cd $wt && source env.sh && t0=$(date +%s) && env $ENV TS_IVF=$A/$stream.obu python tb/runner.py dec_host test_dec_host_inter > ~/logs/bisect-$stream-$sha.log 2>&1;
    rc=$?; log=~/logs/bisect-$stream-$sha.log
    if grep -q "OK: {" $log; then v=PASS; elif grep -q "samples differ" $log; then v="FAIL $(grep -m1 -oE 'frame [0-9]+: flushed picture: [0-9]+ samples differ' $log)"; else v="ERROR $(grep -m1 -oE 'AssertionError.*|%Error.*|env: .*' $log | cut -c1-100)"; fi
    echo "$sha: $v ($(( $(date +%s) - t0 )) s, rc $rc)" >> $V ) &
  sleep 3
done
wait
echo "BISECT_DONE $(date -u +%H:%MZ)" >> $V
cat $V
