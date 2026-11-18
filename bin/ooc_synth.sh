#!/usr/bin/env bash
# ooc_synth.sh <top> <tag> <worktree>: out-of-context synthesis of one module for the U200 part (every new module gets
# one, and its utilisation is read: a RAM built from flip-flops passes every simulation). Log: /tmp/ooc_synth_<top>_<tag>.log
cd ${3:?worktree} && source /opt/Xilinx/*/Vivado/settings64.sh 2>/dev/null
SYNTH_PART=xcu200-fsgd2104-2-e synth/vivado/run.sh -t $1 11 11 $2 > /tmp/ooc_synth_$1_$2.log 2>&1
echo "synth rc=$?" >> /tmp/ooc_synth_$1_$2.log
