#!/usr/bin/env bash
# postroute.sh <source image> <tag>: a new image u200-<tag>-<sha> from the ROUTED CHECKPOINT of <source image>
# (u200-<x>-<sha> in ~/openav1-synth), repaired by Vivado's post-route phys_opt_design (postroute.tcl). For an image
# that missed timing by a few hundredths of a ns outside the decoder; anything more wants a new build. Same output
# layout as build.sh (impl_*.rpt, sys_wrapper.bit, proj/u200.runs/impl_1/sys_wrapper.bit for bin/card_measure.sh) and
# the same line in ~/logs/u200-builds.log, its configuration marked "postroute of <source image>".
# An image made this way is an edited-checkpoint image: loading it on the card is asked of Jonah per case.
set -euo pipefail
[ $# -eq 2 ] || { echo "usage: postroute.sh <source image> <tag>" >&2; exit 2; }
here=$(cd "$(dirname "$0")" && pwd); root=$(cd "$here/../.." && pwd)
src=$1; sha=${src##*-}; img=u200-$2-$sha; out=$HOME/openav1-synth/$img
dcp=$HOME/openav1-synth/$src/proj/u200.runs/impl_1/sys_wrapper_routed.dcp
[ -f "$dcp" ] || { echo "postroute.sh: no routed checkpoint $dcp" >&2; exit 1; }
[ ! -e "$out" ] || { echo "postroute.sh: $out exists" >&2; exit 1; }
mkdir -p "$out/work"; cd "$out/work"
export U200_ROOT=$root U200_OUT=$out U200_SRC_DCP=$dcp
rc=0
vivado -mode batch -nojournal -log "$out/vivado.log" -source "$here/postroute.tcl" > ~/logs/$img.log 2>&1 || rc=$?
grep -q BUILD_DONE "$out/vivado.log" || rc=1
grep -h "^decoder: FBX" ~/logs/$src.log | tail -1 >> ~/logs/$img.log      # the configuration is the source image's
[ $rc = 0 ] && cp "$out/proj/u200.runs/impl_1/sys_wrapper.bit" "$out/"
cfg=$(grep " $src rc=" ~/logs/u200-builds.log | tail -1 | sed -E 's/.*\[(.*)\]$/\1/')
echo "$(date -Is) $img rc=$rc $(grep -A6 "Design Timing Summary" $out/impl_timing.rpt 2>/dev/null | tail -1 | awk '{print "WNS",$1,"TNS",$2,"failing",$3,"WHS",$5,"WPWS",$9}') $(grep -q "All user specified timing constraints are met" $out/impl_timing.rpt 2>/dev/null && echo MET || echo NOT-MET) [postroute of $src: $cfg]" >> ~/logs/u200-builds.log
tail -1 ~/logs/u200-builds.log
exit $rc
