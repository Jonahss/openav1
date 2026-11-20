#!/usr/bin/env bash
# tools/lint_all.sh [outdir]: Verilator --lint-only -Wall of dec_host in the configurations docs/warnings reads
# (the card image and the defaults), every warning kind on. Prints the counts;
# the logs go to outdir (default docs/warnings/lint, as .txt: *.log is ignored by git). Run from the repository root
# after `source env.sh`.
set -u
O=${1:-docs/warnings/lint}; mkdir -p $O
F=$(grep -A3 "^files=" fpga/u200/build.sh | tr -d '"' | sed "s/files=//" | tr "\n" " "); L=""
for f in $F; do [ -f rtl/$f.sv ] && L="$L rtl/$f.sv"; done
IMG="-GFBX=11 -GFBY=11 -GFRAME_URAM=1 -GFRAME_WIN=5 -GFRAME_WIN2=3 -GSB128=1 -GREF_RING=16 -GITX_LANES=2 -GTE=2"
run() { verilator --lint-only -Wall -Wno-fatal --top-module dec_host $2 -Irtl $L > $O/$1.txt 2>&1; echo "$1 rc=$? $(grep -oE "^%(Warning|Error)-?[A-Z_]*" $O/$1.txt | sort | uniq -c | tr -s " " | tr "\n" ";")"; }
run dec_host-image "$IMG" &
run dec_host-defaults "" &
wait
# signals no configuration uses: dead RTL
for f in $O/dec_host-*.txt; do grep -E "Signal is not used|Signal is not driven, nor used" $f | sed -E "s/^%Warning-UNUSEDSIGNAL: //; s/:[0-9]+: /: /" | sort -u > $f.dead; done
sort $O/dec_host-*.txt.dead | uniq -c | awk -v n=$(ls $O/dec_host-*.txt.dead | wc -l) '$1 == n { $1 = ""; print substr($0, 2) }' > $O/dead-in-all.txt
rm -f $O/dec_host-*.txt.dead
echo "unused in every configuration: $(wc -l < $O/dead-in-all.txt) signals ($O/dead-in-all.txt)"
