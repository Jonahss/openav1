#!/usr/bin/env bash
# card_measure.sh <bitstream> <tag>: program an image, decode the 1080p clips against their dav1d references
# (tools/host/decode.py): the libaom single-tile clip (in_to_tree-aom.ivf, 548 frames) and the
# two-tile-column clip (in_to_tree_1080p50_20.ivf, aomenc --tile-columns=1), each checked and then unchecked.
# Log ~/measure1080/pf_card_<tag>.txt (PF_CARD_DONE / PF_CARD_ABORT), raw logs pf_card_<tag>_<clip>_raw.log.
#
# The host software is the image's own: tools/ and tb/ of the commit in the image's name (u200-<tag>-<sha>) are
# extracted into a temporary directory and run from there, wherever this script is started (2026-11-19: a new image
# driven by the previous day's decode.py read as "broken on the card").
#
# Afterwards (Jonah 2026-11-19, WF-21) the new image STAYS as the standing image when it is bit-exact (both clips ran,
# pictures compared, none differs) and its build met timing (WNS >= 0 in ~/logs/u200-builds.log); otherwise the image
# that was on the card before is programmed back. ~/measure1080/card_image.txt names what is on the card.
set -u
[ $# -eq 2 ] || { echo "usage: card_measure.sh <bitstream> <tag>" >&2; exit 2; }
bit=$1; tag=$2; img=$(basename $(dirname $(dirname $(dirname $(dirname $bit))))); sha=${img##*-}
W=$(cd "$(dirname "$0")/.." && pwd); M=~/measure1080; L=$M/pf_card_$tag.txt; : > $L
H=$(mktemp -d); trap 'rm -rf "$H"' EXIT
abort() { echo "$1" >> $L; echo PF_CARD_ABORT >> $L; echo "card_measure: $1" >&2; exit 1; }
cd $W || abort "no checkout at $W"
[ -f "$bit" ] || abort "no bitstream $bit"
git rev-parse -q --verify "$sha^{commit}" > /dev/null || abort "image $img: commit $sha is not in $W, its host software cannot be taken"
git archive "$sha" tools tb | tar -x -C "$H"; [ -f $H/tools/host/decode.py ] && [ -f $H/tb/host_regs.py ] || abort "could not extract tools/ and tb/ of $sha"
echo "host software: tools/ and tb/ of $(git log --oneline -1 "$sha" | cut -c1-60)" >> $L
source /opt/Xilinx/2025.2.1/Vivado/settings64.sh >/dev/null && source env.sh >/dev/null 2>&1
export PYTHONPATH=$H/tools:$H/tb:$H/tools/host
bash $H/tools/host/build_hostlib.sh 2>&1 | sed "s|$H/||" >> $L; [ -f $H/tools/host/libhostregs.so ] || abort "tools/host/build_hostlib.sh failed"
pgrep -f "tools/host/decode.p[y]|fpga/u200/program.s[h]" > /dev/null && abort "card busy, not programming"
prev=$(cat $M/card_image.txt)
fpga/u200/program.sh $bit 2>&1 | grep -E "PROGRAMMED|error|no xcu" >> $L
grep -q PROGRAMMED $L || abort "programming failed"
echo "$img" > $M/card_image.txt; echo "card image: $img $(date -Is)" >> $L
good=1
for clip in in_to_tree-aom in_to_tree_1080p50_20; do
    ivf=$M/$clip.ivf; ref=$M/refout/$clip.yuv; raw=$M/pf_card_${tag}_${clip}_raw.log
    [ -f $ref ] || prefix-trace/bin/dav1d -q -i $ivf -o $ref --threads 1 --filmgrain 0 > /dev/null 2>&1
    echo "== $clip (ref $(stat -c %s $ref) bytes)" >> $L
    t0=$(date +%s)
    timeout 1800 python $H/tools/host/decode.py $ivf --max-bad 3 --ref $ref > $raw 2>&1
    rc=$?; echo "rc=$rc wall $(( $(date +%s) - t0 ))s" >> $L
    grep -vE "^\[win\]" $raw | grep -E "^decoder|identical|differ|error|Error|Traceback|HostError|refus|fps|total|tiles" | tail -4 | cut -c1-330 >> $L
    ni=$(grep -c "identical" $raw); nd=$(grep -c "samples differ" $raw)
    echo "frames: $(grep -c "^frame" $raw); identical: $ni; differ: $nd" >> $L
    [ $rc -eq 0 ] && [ $ni -gt 0 ] && [ $nd -eq 0 ] || good=0
    grep -E "^frame ([0-9]|1[0-3]):" $raw | cut -c1-120 >> $L
    # the headline number (Jonah 2026-11-10: "report the unchecked fps number. that's the performance of the real card we're
    # building. checking is just how we verify"): the same clip again without --ref, nothing read back or compared
    timeout 1800 python $H/tools/host/decode.py $ivf > ${raw%.log}_unchecked.log 2>&1
    echo "UNCHECKED $clip: $(grep -oE "in [0-9.]+s \([0-9.]+ fps\)" ${raw%.log}_unchecked.log | tail -1)" >> $L
done
wns=$(grep " $img rc=0 WNS " ~/logs/u200-builds.log | tail -1 | sed -E "s/.* WNS ([-0-9.]+) .*/\1/")
case "$wns" in "") why="no rc=0 build line for it in u200-builds.log";; -*) why="timing not met (WNS $wns)";; *) why="";; esac
[ $good -eq 1 ] || why="not bit-exact${why:+, $why}"
if [ -z "$why" ]; then
    echo "kept $img: bit-exact on both clips, WNS $wns" >> $L
elif [ "$prev" = "$img" ]; then
    echo "NOT GOOD ($why) and it was already the standing image: $img left on the card" >> $L
else
    prevbit=$(ls -d ~/openav1-synth/$prev/sys_wrapper.bit 2>/dev/null)
    if [ -n "$prevbit" ] && fpga/u200/program.sh $prevbit 2>&1 | grep -E "PROGRAMMED|error" >> $L && grep -q "PROGRAMMED $prevbit" $L; then
        echo "$prev" > $M/card_image.txt; echo "restored $prev ($img: $why)" >> $L
    else
        echo "NOT GOOD ($why) and the previous image $prev could not be put back: $img is still on the card" >> $L
    fi
fi
echo PF_CARD_DONE >> $L
