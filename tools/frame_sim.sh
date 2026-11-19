#!/usr/bin/env bash
# tools/frame_sim.sh <frame> <tag> [axi_lat=48] [ring=8] [stream]: one frame of a 1080p clip through dec_top in
# simulation (the card geometry: FBX/FBY 11, two tile engines), the other frames decoded by the model alone. The log
# (logs/frame_<tag>.log) carries the testbench perf lines and dec_top[rate] lines: the clocks of every band of the
# filter chain and how long reconstruction was held. Hours for a heavy frame. Run from the repository root.
set -u
frame=${1:?frame}; tag=${2:?tag}; lat=${3:-48}; ring=${4:-8}; ivf=${5:-$HOME/measure1080/in_to_tree-aom.ivf}
source env.sh >/dev/null 2>&1; mkdir -p logs
export FBX=11 FBY=11 TE=2 REF_RING=$ring TS_FRAMES=$frame TS_IVF=$ivf TS_AXI_OUT=16 TS_AXI_LAT=$lat TS_PERF=1 TS_STAGE=lr
export TS_TEST_DIR=/tmp/frame_sim_$tag TS_BUILD_TAG=frame_$tag
echo "launch $(date -Is) frame $frame of $(basename $ivf), AXI latency $lat, ring $ring, $(git log --oneline -1 | cut -c1-50)" > logs/frame_$tag.log
timeout 28800 python tb/runner.py dec_top test_dec_inter >> logs/frame_$tag.log 2>&1
echo "frame_sim rc=$?" >> logs/frame_$tag.log
