#!/usr/bin/env bash
# blk_gate.sh <block> <test env...> : netlist replay of one chain block: record the block's unit test (RTL, vcd), generate the BG_GEN="--gate v:sig,.." passes options to gen_replay (cdef_stream: --gate dw_we:dw_row,dw_col,dw_info).
# replay harness, synthesise the block out of context for the card's part, replay the vectors against the RTL (control) and
# the funcsim netlist. Result lines in ~/gate/<block>/blk_gate.txt
set -u
blk=$1; shift
W=${BGW:-~/openav1-wt-card}; G=~/gate/$blk; mkdir -p $G; L=$G/blk_gate.txt; : > $L
cd $W && source env.sh && source /opt/Xilinx/2025.2.1/Vivado/settings64.sh >/dev/null
echo "worktree $(git log --oneline -1 | cut -c1-50) $(date -u +%H:%M)" >> $L
rm -rf build/${blk}_bg
env "$@" TS_BUILD_TAG=bg WAVES=1 WAVE_FMT=vcd WAVE_DEPTH=1 timeout 3000 python tb/runner.py $blk ${BGTEST:-} > $G/rec.log 2>&1
echo "twin rc=$? $(grep -oE 'OK: \{.*|AssertionError.*' $G/rec.log | tail -1 | cut -c1-140)" >> $L
B=$(ls -d build/${blk}*_bg 2>/dev/null | head -1); [ -f "$B/dump.vcd" ] || { echo "no dump.vcd in $B" >> $L; exit 1; }
python tools/gen_replay.py rtl/$blk.sv --maxv 400000 ${BG_GEN:-} > $G/replay_$blk.sv 2> $G/gen_replay.lists || { echo "gen_replay failed: $(tail -2 $G/gen_replay.lists)" >> $L; exit 1; }
IN=$(grep -o -- "--in [^ ]*" $G/gen_replay.lists | head -1 | cut -c6-); OUT=$(grep -o -- "--out [^ ]*" $G/gen_replay.lists | head -1 | cut -c7-)
python tools/vcd2vec.py $B/dump.vcd TOP.$blk clk --in $IN --out $OUT > $G/raw.hex 2> $G/vec.err
cut -d" " -f1 $G/raw.hex | head -n 400000 > $G/vec.hex; cut -d" " -f2 $G/raw.hex | head -n 400000 > $G/vec.hex.out
echo "vectors: $(wc -l < $G/vec.hex) ($(tail -1 $G/vec.err | cut -c1-80))" >> $L
rm -f $B/dump.vcd $G/raw.hex
SYNTH_PART=xcu200-fsgd2104-2-e synth/vivado/run.sh -t $blk 11 11 bg > $G/synth.log 2>&1
S=~/openav1-synth/$blk-bg
echo "synth rc=$? $(grep -E '^\| CLB LUTs' $S/util.rpt | awk -F'|' '{print $3}' | tr -d ' ') LUT; 8-4767: $(grep -c 'Synth 8-4767' $S/vivado.log); 8-6901: $(grep -c 'Synth 8-6901' $S/vivado.log)" >> $L
rtl=$(grep -oE "rtl/[a-z_0-9]+\.sv" $S/vivado.log | sort -u | sed "s#^#$W/#" | tr '\n' ' ')
[ -n "$rtl" ] || rtl="$W/rtl/$blk.sv"
~/gate/gate_block.sh $blk bg $S/post_synth.dcp vec.hex $(for f in $BLK_RTL; do echo -n "$W/rtl/$f.sv "; done) >> $L 2>&1
echo BLK_GATE_DONE >> $L
