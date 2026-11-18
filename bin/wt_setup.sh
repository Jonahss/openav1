#!/usr/bin/env bash
# Make (or refresh) an openav1 worktree on atium with everything the suites need that is not in git:
#   bin/wt_setup.sh <dir> <branch-or-ref>
# venv / toolchain / dav1d trace build / spec text from ~/openav1, the full reference pictures (~/refout-fb14,
# 93 files; ~/openav1/refout is incomplete), the Argon set, the natural streams, the msac traces.
set -euo pipefail
d=$1; ref=$2
git -C ~/openav1 fetch -q origin
if [ ! -e "$d/.git" ]; then git -C ~/openav1 worktree prune; git -C ~/openav1 worktree add -q "$d" "origin/$ref" 2>/dev/null || git -C ~/openav1 worktree add -q "$d" "$ref"; fi
cd "$d" && git checkout -q --detach "origin/$ref" 2>/dev/null || git checkout -q --detach "$ref"
for x in venv prefix-trace toolchain traces; do [ -e $x ] || ln -s ~/openav1/$x $x; done
[ -e refout ] && [ ! -L refout ] && mv refout refout.local; [ -L refout ] && [ "$(readlink refout)" != "/home/jonahss/refout-fb14" ] && rm refout; [ -e refout ] || ln -s ~/refout-fb14 refout
[ -e streams/argon ] || ln -s ~/openav1/streams/argon streams/argon
for f in ~/openav1-wt-pffm2c/streams/inter/mf_small_8bit.ivf; do [ -e streams/inter/$(basename $f) ] || cp $f streams/inter/; done   # untracked inter streams the gate rung records
cp -n ~/openav1/streams/synth/natural_lf_only_*.ivf streams/synth/ 2>/dev/null || true
mkdir -p logs
echo "$d at $(git rev-parse --short HEAD): refout $(ls refout | wc -l) files, argon $(ls streams/argon/profile0_core/streams 2>/dev/null | wc -l) streams"
