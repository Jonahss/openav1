#!/usr/bin/env bash
# whatif_frame.sh <frame> <tag> <axi_lat> <ring> [VAR=value ...]: one frame of the 1080p clip through dec_top in simulation
# with the per-state profile (TS_PERF: the FSM histograms and the motion-compensation counters), at a given AXI read
# latency and reference ring depth, in this checkout. The what-if run before a fetch-side redesign: how much a frame
# shortens when one cost is taken away (2026-11-12: three "biggest shares" were each worth 3 %). Earlier frames are
# decoded by the model only. Output: logs/whatif-<tag>.log; the frame's "tile done in ~N cycles" line is the figure.
set -u
[ $# -ge 4 ] || { echo "usage: whatif_frame.sh <frame> <tag> <axi_lat> <ring> [VAR=value ...]" >&2; exit 2; }
W=$(cd "$(dirname "$0")/.." && pwd); cd $W && source env.sh >/dev/null 2>&1
f=$1; tag=$2; lat=$3; ring=$4; shift 4
mkdir -p logs
echo "launch $(date -Is) $tag: frame $f, AXI latency $lat, ring $ring, $* $(git log --oneline -1 | cut -c1-50)" > logs/whatif-$tag.log
exec env FBX=11 FBY=11 TS_FRAMES=$f TS_IVF=$HOME/measure1080/in_to_tree-aom.ivf TS_AXI_OUT=16 TS_AXI_LAT=$lat TS_PERF=1 TS_STAGE=lr \
    TS_TEST_DIR=/tmp/whatif_$tag TE=2 REF_RING=$ring TS_BUILD_TAG=whatif-$tag "$@" timeout 43200 python tb/runner.py dec_top test_dec_inter >> logs/whatif-$tag.log 2>&1
