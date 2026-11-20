#!/usr/bin/env bash
# image.sh <worktree> <tag> [build.sh options]: one U200 image of the worktree's commit (fpga/u200/build.sh: the one
# decoder configuration; --te, --mhz, --ring, --strategy choose the rest, defaults = the standing image's), through
# ramq (kind u200-image, protected: waits for its memory, first in the queue, own ceiling), with its ticket on the
# Builds board and the usual line in ~/logs/u200-builds.log. Image name: u200-<tag>-<commit>.
set -uo pipefail
[ $# -ge 2 ] || { echo "usage: image.sh <worktree> <tag> [build.sh options]" >&2; exit 2; }
W=$1; tag=$2; shift 2
cfg=$($W/fpga/u200/build.sh --print "$@") || exit 2
# its ticket on the Builds board (vk never fails the build; CHG=CHG-n links the change under test)
BLD=$($(dirname "$0")/vk build-open image "image u200-$tag-$(git -C $W rev-parse --short HEAD)" --commit "$(git -C $W rev-parse --short HEAD)" --config "asked for: $cfg" --output "atium ~/openav1-synth/u200-$tag-$(git -C $W rev-parse --short HEAD)" --changes "${CHG:-}")
cd $W && h=$(git rev-parse --short HEAD)
source /opt/Xilinx/2025.2.1/Vivado/settings64.sh >/dev/null && source env.sh
$HOME/ramq/bin/ramq run u200-image --name $tag-$h -- fpga/u200/build.sh $tag-$h "$@" > ~/logs/u200-$tag-$h.log 2>&1
rc=$?
echo "$(date -Is) u200-$tag-$h rc=$rc $(grep -A6 "Design Timing Summary" ~/openav1-synth/u200-$tag-$h/impl_timing.rpt 2>/dev/null | tail -1 | awk "{print \"WNS\",\$1,\"TNS\",\$2,\"failing\",\$3,\"WHS\",\$5}") [$W $cfg]" >> ~/logs/u200-builds.log
line=$(tail -1 ~/logs/u200-builds.log); built=$(grep -h "^decoder: FBX" ~/logs/u200-$tag-$h.log | tail -1)
if [ -n "$BLD" ]; then
  echo "$BLD" > ~/openav1-synth/u200-$tag-$h/.bld 2>/dev/null
  $(dirname "$0")/vk comment "$BLD" "built: ${built:-no decoder line in the log}"
  case "$line" in *"WNS -"*) $(dirname "$0")/vk label "$BLD" timing:miss;; *"WNS "[0-9]*) $(dirname "$0")/vk label "$BLD" timing:clean;; esac
  [ "$rc" = 0 ] && [ -f ~/openav1-synth/u200-$tag-$h/proj/u200.runs/impl_1/sys_wrapper.bit ] && st=Passed || st=Failed
  $(dirname "$0")/vk build-close "$BLD" $st "$line" > /dev/null
fi
