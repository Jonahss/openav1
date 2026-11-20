#!/usr/bin/env bash
# after_build.sh <name> <sha> <tag> [<tag> ...]  (2026-11-17; the general form of fo_/ha_/ig_after_build.sh, which were
# copies with the tags and the commit written in). Waits for the images u200-<tag>-<sha> (their rc line in
# ~/logs/u200-builds.log), then runs the two-clip card test on each that built (card_measure: checked + unchecked;
# a good image stays on the card, a bad one is replaced by the previous). Summary: ~/measure1080/<name>_after_build.txt, last line AFTER_BUILD_DONE.
set -u
name=$1; sha=$2; shift 2
S=~/measure1080/${name}_after_build.txt; : > $S
for t in "$@"; do
  until grep -q "u200-$t-$sha rc=" ~/logs/u200-builds.log; do sleep 60; done
  grep "u200-$t-$sha rc=" ~/logs/u200-builds.log | tail -1 >> $S
  grep -h "^HOLD_ANCHOR" ~/openav1-synth/u200-$t-$sha/vivado.log >> $S
  grep -h "^decoder: FBX" ~/logs/u200-$t-$sha.log | tail -1 >> $S          # the configuration that was really built
done
while pgrep -f "card_measure.sh" > /dev/null; do sleep 30; done
for t in "$@"; do
  B=~/openav1-synth/u200-$t-$sha/proj/u200.runs/impl_1/sys_wrapper.bit
  grep -q "u200-$t-$sha rc=0" ~/logs/u200-builds.log && [ -f $B ] || { echo "$t: no image to test" >> $S; continue; }
  $(dirname "$0")/card_measure.sh $B $t
  echo "---- $t" >> $S; grep -E "identical|UNCHECKED|PF_CARD|restored|kept|NOT GOOD|mismatch" ~/measure1080/pf_card_$t.txt | cut -c1-200 >> $S
  BLD=$(cat ~/openav1-synth/u200-$t-$sha/.bld 2>/dev/null)
  if [ -n "$BLD" ]; then
    $(dirname "$0")/vk comment "$BLD" "card: $(grep -E "^frames:|UNCHECKED|PF_CARD" ~/measure1080/pf_card_$t.txt | tr "\n" ";" | cut -c1-400)"
    # bit-exact: both clips ran, no picture differs, and pictures were in fact compared (a run that died early has
    # "identical: 0; differ: 0"; until 2026-11-19 the differ count matched a text decode.py never prints and was always 0)
    if grep -q PF_CARD_DONE ~/measure1080/pf_card_$t.txt && [ "$(grep -cE "^frames:" ~/measure1080/pf_card_$t.txt)" = 2 ] && ! grep -E "^frames:" ~/measure1080/pf_card_$t.txt | grep -qvE "identical: [1-9][0-9]*; differ: 0$"; then $(dirname "$0")/vk label "$BLD" card:bit-exact; else $(dirname "$0")/vk label "$BLD" card:differs-or-aborted; fi
  fi
done
echo "AFTER_BUILD_DONE $(date -Is)" >> $S
