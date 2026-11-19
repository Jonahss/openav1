#!/usr/bin/env bash
# tools/stream_sim.sh <stream.ivf> <ref dir> <tag> [FBX=12] [FBY=12]: a whole stream through dec_top alone (no model),
# every output picture compared with dav1d (<ref dir>/<stream name>.yuv). For pictures larger than the suites use
# (4K: FBX = FBY = 12). The log (logs/stream_<tag>.log) carries dec_top[rate] lines. Run from the repository root.
set -u
ivf=${1:?stream}; ref=${2:?reference directory}; tag=${3:?tag}; fbx=${4:-12}; fby=${5:-12}
source env.sh >/dev/null 2>&1; mkdir -p logs
export FBX=$fbx FBY=$fby TS_NOMODEL=1 TS_STAGE=lr TS_IVF=$ivf TS_REF_DIR=$ref SIM_FAST=1 SIM_THREADS=8 TS_BUILD_TAG=stream_$tag
echo "launch $(date -Is) $(basename $ivf) FBX $fbx FBY $fby, $(git log --oneline -1 | cut -c1-50)" > logs/stream_$tag.log
timeout 86400 python tb/runner.py dec_top >> logs/stream_$tag.log 2>&1
echo "stream_sim rc=$?" >> logs/stream_$tag.log
