#!/usr/bin/env bash
# idle_measure.sh <bitfile> <tag>  (2026-11-17): the scoreboard measurement wants an idle host. Stops (SIGSTOP) every
# simulation / Vivado / Verilator / compiler process of this user, waits for the load to fall, runs card_measure (checked +
# unchecked, restores the previous image), and continues them (SIGCONT) whatever happens. The stopped jobs lose only time.
set -u
bit=$1; tag=$2
L=~/measure1080/idle_measure_$tag.txt; : > $L
pids=$(ps -u $USER -o pid=,comm= | awk "\$2 ~ /^(vivado|Vtop|vreplay|verilator_bin|verilator|cc1plus|make|xelab|xsim)/ {print \$1}")
pids="$pids $(pgrep -u $USER -f "tb/runner.py|sim_build|obj_dir/V" | grep -v "^$$\$" | tr "\n" " ")"
resume() { for p in $pids; do kill -CONT $p 2>/dev/null; done; echo "resumed $(date -u +%H:%M:%S)" >> $L; }
trap resume EXIT
echo "stopping $(echo $pids | wc -w) processes $(date -u +%H:%M:%S), load $(cut -d" " -f1 /proc/loadavg)" >> $L
for p in $pids; do kill -STOP $p 2>/dev/null; done
sleep 45
echo "load after 45 s: $(cut -d" " -f1-2 /proc/loadavg); running now: $(ps -u $USER -o stat=,comm= | awk "\$1 ~ /^R/ {print \$2}" | sort | uniq -c | tr "\n" " ")" >> $L
$(dirname "$0")/card_measure.sh $bit $tag >> $L 2>&1
grep -E "identical|UNCHECKED|PF_CARD|restored" ~/measure1080/pf_card_$tag.txt | cut -c1-200 >> $L
echo "IDLE_MEASURE_DONE" >> $L
