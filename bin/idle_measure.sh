#!/usr/bin/env bash
# idle_measure.sh <bitfile> <tag>: the scoreboard measurement wants an idle host. Stops (SIGSTOP) every simulation /
# Vivado build / Verilator / compiler process of this user, waits for the load to fall, runs card_measure (checked +
# unchecked; it keeps a good image on the card), and continues them (SIGCONT) whatever happens. The stopped jobs lose
# only time.
#
# Refuses to start while anything is talking to the card (a measurement, a decode, a programming step): on 2026-11-19
# it stopped another measurement's programming Vivado in mid-JTAG and the cable stayed jammed until that was resumed
# by hand. Simulators are found by where their binary lives (a sim_build or obj_dir directory), not by name.
# `idle_measure.sh --list` prints what would be stopped and stops nothing.
set -u
busy=$(pgrep -u $USER -af "bin/card_measure.s[h]|tools/host/decode.p[y]|fpga/u200/program.s[h]|program.tc[l]" | grep -v "idle_measure" | cut -c1-160)
[ -z "$busy" ] || { echo "idle_measure: the card is in use, not starting:" >&2; echo "$busy" >&2; exit 1; }
pids=""
for d in /proc/[0-9]*; do
    p=${d#/proc/}; [ -O $d ] && [ $p != $$ ] || continue
    comm=$(cat $d/comm 2>/dev/null); exe=$(readlink $d/exe 2>/dev/null)
    case "$comm" in vivado|Vtop*|vreplay*|verilator*|cc1plus|make|xelab|xsim*|xsc|ld|ccache) pids="$pids $p"; continue;; esac
    case "$exe" in */sim_build*|*/obj_dir*) pids="$pids $p"; continue;; esac
    tr "\0" " " < $d/cmdline 2>/dev/null | grep -qE "tb/runner\.py|sim_build|obj_dir/V" && pids="$pids $p"
done
if [ "${1:-}" = "--list" ]; then for p in $pids; do ps -o pid=,stat=,comm=,args= -p $p | cut -c1-150; done; exit 0; fi
[ $# -eq 2 ] || { echo "usage: idle_measure.sh <bitfile> <tag> | --list" >&2; exit 2; }
bit=$1; tag=$2
L=~/measure1080/idle_measure_$tag.txt; : > $L
resume() { for p in $pids; do kill -CONT $p 2>/dev/null; done; echo "resumed $(date -u +%H:%M:%S)" >> $L; }
trap resume EXIT
echo "stopping $(echo $pids | wc -w) processes $(date -u +%H:%M:%S), load $(cut -d" " -f1 /proc/loadavg)" >> $L
for p in $pids; do kill -STOP $p 2>/dev/null; done
sleep 45
echo "load after 45 s: $(cut -d" " -f1-2 /proc/loadavg); running now: $(ps -u $USER -o stat=,comm= | awk "\$1 ~ /^R/ {print \$2}" | sort | uniq -c | tr "\n" " ")" >> $L
$(dirname "$0")/card_measure.sh $bit $tag >> $L 2>&1
grep -E "identical|UNCHECKED|PF_CARD|restored|kept|NOT GOOD|mismatch" ~/measure1080/pf_card_$tag.txt | cut -c1-200 >> $L
echo "IDLE_MEASURE_DONE" >> $L
