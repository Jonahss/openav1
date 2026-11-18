#!/usr/bin/env bash
# image.sh <worktree> <tag> <TE> <MHz> <strategy>  (2026-11-07): one U200 image of the window/inter configuration
# (FBX/FBY 11, fb0 window 5 rows, fb2 3 rows, sb128, lr_pblock) with the given tile engines, clock and implementation strategy; the usual line in
# ~/logs/u200-builds.log. The knob experiments for TE=2 at 100 MHz run through this.
# 2026-11-15: the build runs through ramq (kind u200-image, protected: waits for its memory, first in the queue, own ceiling).
set -uo pipefail
W=$1; tag=$2; te=$3; mhz=$4; strat=$5
# its ticket on the Builds board (vk never fails the build; CHG=CHG-n links the change under test)
BLD=$($(dirname "$0")/vk build-open image "image u200-$tag-$(git -C $W rev-parse --short HEAD)" --commit "$(git -C $W rev-parse --short HEAD)" --config "asked for: TE=$te RING=${U200_REF_RING:-16} $mhz MHz $strat" --output "atium ~/openav1-synth/u200-$tag-$(git -C $W rev-parse --short HEAD)" --changes "${CHG:-}")
cd $W && h=$(git rev-parse --short HEAD)
source /opt/Xilinx/2025.2.1/Vivado/settings64.sh >/dev/null && source env.sh
U200_INTER=1 U200_FBX=11 U200_FBY=11 U200_WIN=5 U200_WIN2=3 U200_MHZ=$mhz U200_SB128=1 U200_TE=$te U200_STRATEGY=$strat U200_PBLOCK=1 U200_XDC=fpga/u200/lr_pblock.xdc $HOME/ramq/bin/ramq run u200-image --name $tag-$h -- fpga/u200/build.sh $tag-$h > ~/logs/u200-$tag-$h.log 2>&1
rc=$?
echo "$(date -Is) u200-$tag-$h rc=$rc $(grep -A6 "Design Timing Summary" ~/openav1-synth/u200-$tag-$h/impl_timing.rpt 2>/dev/null | tail -1 | awk "{print \"WNS\",\$1,\"TNS\",\$2,\"failing\",\$3,\"WHS\",\$5}") [$W TE=$te RING=${U200_REF_RING:-16} $mhz MHz $strat]" >> ~/logs/u200-builds.log
line=$(tail -1 ~/logs/u200-builds.log); built=$(grep -h "^decoder: FBX" ~/logs/u200-$tag-$h.log | tail -1)
if [ -n "$BLD" ]; then
  echo "$BLD" > ~/openav1-synth/u200-$tag-$h/.bld 2>/dev/null
  $(dirname "$0")/vk comment "$BLD" "built: ${built:-no decoder line in the log}"
  case "$line" in *"WNS -"*) $(dirname "$0")/vk label "$BLD" timing:miss;; *"WNS "[0-9]*) $(dirname "$0")/vk label "$BLD" timing:clean;; esac
  [ "$rc" = 0 ] && [ -f ~/openav1-synth/u200-$tag-$h/proj/u200.runs/impl_1/sys_wrapper.bit ] && st=Passed || st=Failed
  $(dirname "$0")/vk build-close "$BLD" $st "$line" > /dev/null
fi
