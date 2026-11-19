#!/usr/bin/env bash
# card_measure.sh <bitstream> <tag> (2026-11-03 evening): program an image, decode the 1080p clips against their dav1d references
# (tools/host/decode.py --inter --pool 12): the libaom single-tile clip (in_to_tree-aom.ivf, 548 frames) and the
# two-tile-column clip (in_to_tree_1080p50_20.ivf, aomenc --tile-columns=1), record per-frame and total times, restore the
# census image. Log ~/measure1080/pf_card_<tag>.txt (PF_CARD_DONE / PF_CARD_ABORT), raw logs pf_card_<tag>_<clip>_raw.log.
set -u
bit=$1; tag=$2; img=$(basename $(dirname $(dirname $(dirname $(dirname $bit))))); W=~/openav1-wt-main; M=~/measure1080; L=$M/pf_card_$tag.txt; : > $L
cd $W && git log --oneline -1 | cut -c1-60 >> $L
source /opt/Xilinx/2025.2.1/Vivado/settings64.sh >/dev/null && source env.sh >/dev/null 2>&1
export PYTHONPATH=$W/tools:$W/tb:$W/tools/host
pgrep -f "tools/host/decode.p[y]|tools/kv260/decode.p[y]" > /dev/null && { echo "card busy, not programming" >> $L; echo PF_CARD_ABORT >> $L; exit 1; }
prev=$(cat $M/card_image.txt)
fpga/u200/program.sh $bit 2>&1 | grep -E "PROGRAMMED|error|no xcu" >> $L
grep -q PROGRAMMED $L || { echo PF_CARD_ABORT >> $L; exit 1; }
echo "$img" > $M/card_image.txt; echo "card image: $img $(date -Is)" >> $L
for clip in in_to_tree-aom in_to_tree_1080p50_20; do
    ivf=$M/$clip.ivf; ref=$M/refout/$clip.yuv; raw=$M/pf_card_${tag}_${clip}_raw.log
    [ -f $ref ] || prefix-trace/bin/dav1d -q -i $ivf -o $ref --threads 1 --filmgrain 0 > /dev/null 2>&1
    echo "== $clip (ref $(stat -c %s $ref) bytes)" >> $L
    t0=$(date +%s)
    timeout 1800 python tools/host/decode.py $ivf --xdma 0 --inter --pool 12 --frames 100000 --max-bad 3 --ref $ref > $raw 2>&1
    echo "rc=$? wall $(( $(date +%s) - t0 ))s" >> $L
    grep -vE "^\[win\]" $raw | grep -E "^decoder|identical|differ|error|Error|Traceback|HostError|refus|fps|total|tiles" | tail -4 | cut -c1-330 >> $L
    echo "frames: $(grep -c "^frame" $raw); identical: $(grep -c "identical" $raw); differ: $(grep -c "samples differ" $raw)" >> $L
    grep -E "^frame ([0-9]|1[0-3]):" $raw | cut -c1-120 >> $L
    # the headline number (Jonah 2026-11-10: "report the unchecked fps number. that's the performance of the real card we're
    # building. checking is just how we verify"): the same clip again without --ref, nothing read back or compared
    timeout 1800 python tools/host/decode.py $ivf --xdma 0 --inter --pool 12 --frames 100000 > ${raw%.log}_unchecked.log 2>&1
    echo "UNCHECKED $clip: $(grep -oE "in [0-9.]+s \([0-9.]+ fps\)" ${raw%.log}_unchecked.log | tail -1)" >> $L
done
prevbit=$(ls -d ~/openav1-synth/$prev/sys_wrapper.bit 2>/dev/null)
if [ -n "$prevbit" ]; then fpga/u200/program.sh $prevbit 2>&1 | grep -E "PROGRAMMED|error" >> $L; echo "$prev" > $M/card_image.txt; echo "restored $prev" >> $L; fi
echo PF_CARD_DONE >> $L
