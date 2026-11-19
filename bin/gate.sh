#!/usr/bin/env bash
# gate.sh (2026-11-18; before: pf_gate_chain.sh and seven per-branch copies of it): the gate rung of a change. The whole
# decoder (dec_host: FBX/FBY 9, TE 1, REF_RING 8, FRAME_WIN 5, FRAME_WIN2 3) is synthesised out of context, its funcsim
# netlist is built under Verilator (vgate), and five recorded shards of 450k cycles are replayed against it: every
# output of every cycle must equal what the RTL gave. The shards run with the card-side load of the frame state
# (TS_FSTATE_B=1: segment map, CDF rows and the motion-field projection read from the areas in DDR).
#   PFW=<detached worktree at the commit> PFTAG=<name> [PFM=<synth tag>] [CHG=CHG-n] nohup setsid bin/gate.sh &
# Verdict and steps in ~/measure1080/pf_gate<PFTAG>.txt (last line PF_GATE_DONE or PF_GATE_ABORT); a ticket on the Builds
# board, closed however the script ends. How:
#   - the recordings run against the card-like memory (TS_AXI_LAT=48 TS_AXI_OUT=16): up to 16 reads in flight, which only the
#     new engine issues, so the netlist replay exercises the outstanding logic;
#   - harnesses generated WITH --genphase phase (the 2026-11-01 lesson), the IN list must equal dec_host9m2's (else abort);
#   - every shard cut to 450k vectors (the harness array; a longer recording makes Verilator's $readmemh abort);
#   - the netlist replays under Verilator (vgate): ONE build of the funcsim netlist (VOPT / VTHR: g++ opt and --threads), the
#     shards in parallel. The shards run through ramq (kind vreplay).
set -u
L=~/measure1080/pf_gate${PFTAG:?PFTAG=<name of this rung>}.txt; : > $L
W=${PFW:?PFW=<worktree at the commit>}; G=${PFG:-~/gate/dec_host9fc}; SP=${PFSP:-q2}; PFM=${PFM:-fc1}; BLK=${BLK:-dec_host9fc}; VJ=${VJ:-40}; S=~/openav1-synth/dec_host-$PFM; mkdir -p $G
VOPT=${VOPT:--O0}; VTHR=${VTHR:-}
source /opt/Xilinx/2025.2.1/Vivado/settings64.sh >/dev/null
cd $W || { echo "no worktree $W" >> $L; echo PF_GATE_ABORT >> $L; exit 1; }
echo "worktree $(git log --oneline -1 | cut -c1-60) $(date -u +%H:%M) vgate OPT=$VOPT $VTHR" >> $L
source env.sh
# no identifier used before its declaration (Vivado: implicit 1-bit net, Synth 8-6901; 2026-11-09 v_hold tied to 0)
python3 tools/check_decl_order.py rtl/*.sv >> $L 2>&1 || { echo "ABORT: identifiers used before their declaration" >> $L; echo PF_GATE_ABORT >> $L; exit 1; }
echo "decl order clean" >> $L
# its ticket on the Builds board, closed when the script ends, however it ends (CHG=CHG-n links the change under test)
BLD=$($(dirname "$0")/vk build-open gate "gate $PFTAG $(git rev-parse --short HEAD)" --commit "$(git rev-parse --short HEAD)" --config "FBX/FBY 9, TE 1, REF_RING 8, FRAME_WIN 5, FRAME_WIN2 3; 5 shards x 450000 cycles" --output "atium $L" --changes "${CHG:-}")
gate_close() { [ -n "$BLD" ] || return 0; if [ "$(tail -1 $L)" = PF_GATE_DONE ]; then st=Passed; else st=Failed; fi; $(dirname "$0")/vk build-close "$BLD" $st "$(grep -cE "0 mismatches -> PASS" $L) of 5 shards PASS; last lines: $(tail -3 $L | tr "\n" ";" | cut -c1-300)" > /dev/null; }
trap gate_close EXIT
# the synth directory and the recorded shards belong to ONE commit (2026-11-09: a relaunch after a fix kept the killed
# synth of the previous commit and waited on it forever, and reused two of its recordings)
SHA=$(git rev-parse --short HEAD)
if [ "$(cat $G/.sha 2>/dev/null)" != "$SHA" ]; then
  pgrep -f "dec_host-$PFM" > /dev/null && { echo "ABORT: a process still runs in $S" >> $L; echo PF_GATE_ABORT >> $L; exit 1; }
  rm -rf $S; rm -f $G/*_vec.hex $G/*_vec.hex.out $G/rec_*.log $G/run_*.log; echo $SHA > $G/.sha
  echo "new commit $SHA: synth directory and recorded shards cleared" >> $L
fi
# A. the OOC synth (background)
if [ -f $S/vivado.log ]; then echo "synth pfm already launched/done: keeping $S" >> $L; else
rm -rf $S
$HOME/ramq/bin/ramq run synth-ooc --detach --name gate-synth-$PFM --log ~/openav1-synth/launch-$PFM.log -- synth/vivado/run.sh -t dec_host -g FRAME_URAM=1 -g ITX_LANES=2 -g FRAME_WIN=5 -g FRAME_WIN2=3 -g TE=1 -g REF_RING=8 9 9 $PFM >> $L 2>&1
echo "synth pfm launched $(date -u +%H:%M)" >> $L
fi
# B. the harness at the same parameters (generated clk2x AND phase)
python tools/gen_replay.py rtl/dec_host.sv --param FBX=9 --param FBY=9 --param ITX_LANES=2 --param FRAME_URAM=1 --param FRAME_WIN=5 --param FRAME_WIN2=3 --param TE=1 --param SB128=0 --param REF_RING=8 --maxv 450000 --gen2x clk2x --genphase phase \
  --gate m_axi_awvalid:m_axi_awaddr,m_axi_awlen,m_axi_awsize,m_axi_awburst,m_axi_awcache,m_axi_awprot \
  --gate m_axi_wvalid:m_axi_wdata,m_axi_wstrb,m_axi_wlast \
  --gate m_axi_arvalid:m_axi_araddr,m_axi_arlen,m_axi_arsize,m_axi_arburst,m_axi_arcache,m_axi_arprot \
  > $G/replay_dec_host9fc.sv 2> $G/gen_replay.lists || { echo "gen_replay failed: $(tail -2 $G/gen_replay.lists)" >> $L; echo PF_GATE_ABORT >> $L; exit 1; }
sed -i "s/module replay_dec_host;/module replay_dec_host9fc;/" $G/replay_dec_host9fc.sv
IN=$(grep -o -- "--in [^ ]*" $G/gen_replay.lists | head -1 | cut -c6-); OUT=$(grep -o -- "--out [^ ]*" $G/gen_replay.lists | head -1 | cut -c7-)
[ -n "$IN" ] && [ -n "$OUT" ] || { echo "empty port lists" >> $L; echo PF_GATE_ABORT >> $L; exit 1; }
echo "$IN" | grep -qw phase && { echo "harness records phase" >> $L; echo PF_GATE_ABORT >> $L; exit 1; }
if diff <(grep -o -- "--in [^ ]*" ~/gate/dec_host9m2/gen_replay.lists | head -1) <(echo "--in $IN") >> $L; then echo "IN list == dec_host9m2 (generated phase)" >> $L
else echo "IN LIST CHANGED vs dec_host9m2: abort, not a note" >> $L; echo PF_GATE_ABORT >> $L; exit 1; fi
# C. the recordings, one after another (they share the build directory), against the card-like memory
A=$W/streams/argon/profile0_core/streams
P="TS_FSTATE_B=1 REF_RING=8 TE=1 SB128=0 FBX=9 FBY=9 ITX_LANES=2 FRAME_URAM=1 FRAME_WIN=5 FRAME_WIN2=3 WAVES=1 WAVE_FMT=vcd WAVE_DEPTH=1 TS_AXI_LAT=48 TS_AXI_OUT=16"
rm -rf $W/build/dec_host_fb9x9*
shards=""
for spec in "${SP}a::$W/streams/inter/mf_small_8bit.ivf" "${SP}b::$W/streams/inter/inter2a_8bit.ivf" "${SP}c::$W/streams/inter/inter_obmc_8bit.ivf" "${SP}d::$A/test47.obu" "${SP}e::$A/test7588_6663.obu"; do
  T=${spec%%:*}; rest=${spec#*:}; EXTRA=${rest%%:*}; IVF=${rest#*:}
  if [ -f $G/${T}_vec.hex ] && [ "$(wc -l < $G/${T}_vec.hex)" -ge 100000 ]; then echo "shard $T already recorded ($(wc -l < $G/${T}_vec.hex) vectors)" >> $L; shards="$shards $T"; continue; fi
  [ -f "$IVF" ] || { echo "ABORT: stream $IVF missing for $T" >> $L; echo PF_GATE_ABORT >> $L; exit 1; }
  t0=$(date +%s)
  env $P $EXTRA TS_IVF=$IVF python tb/runner.py dec_host test_dec_host_inter > $G/rec_$T.log 2>&1
  TRC=$?; echo "twin $T rc=$TRC $(( $(date +%s) - t0 ))s $(grep -oE "OK: \{.*|AssertionError.*" $G/rec_$T.log | tail -1 | cut -c1-160) $(date -u +%H:%M)" >> $L
  B=$(ls -d $W/build/dec_host_fb9x9* 2>/dev/null | while read d; do [ -f $d/dump.vcd ] && echo $d; done | head -1)
  [ -n "$B" ] || { echo "no dump.vcd under $W/build for $T" >> $L; echo PF_GATE_ABORT >> $L; exit 1; }
  python tools/vcd2vec.py $B/dump.vcd TOP.dec_host clk --in $IN --out $OUT > $G/${T}_raw.hex 2> $G/${T}_vec.err
  cut -d" " -f1 $G/${T}_raw.hex | head -n 450000 > $G/${T}_vec.hex; cut -d" " -f2 $G/${T}_raw.hex | head -n 450000 > $G/${T}_vec.hex.out
  NV=$(wc -l < $G/${T}_vec.hex); NR=$(wc -l < $G/${T}_raw.hex); echo "vectors $T: $NV of $NR recorded cycles ($(tail -1 $G/${T}_vec.err | cut -c1-80))" >> $L
  [ "$TRC" = 0 ] && grep -q "OK: {" $G/rec_$T.log && ! grep -q "AssertionError" $G/rec_$T.log && [ "$NV" -ge 100000 ] || { echo "ABORT: twin $T rc=$TRC, $NV cycles recorded (need rc 0, an OK line, no AssertionError, >= 100000)" >> $L; echo PF_GATE_ABORT >> $L; exit 1; }
  rm -f $B/dump.vcd $G/${T}_raw.hex
  shards="$shards $T"
done
# D. the synth, the funcsim netlist, one Verilator build, the shards in parallel
# ramq says how the synth ended (2026-11-17: this was a loop that looked for ^ERROR in vivado.log, and waited nine
# hours on a synth that had been killed). A kept directory of an earlier launch has no live job: wait then
# answers from the ledger, or 3 for a synth ramq never ran, and SYNTH_DONE in the log decides.
$HOME/ramq/bin/ramq wait gate-synth-$PFM >> $L 2>&1; SRC=$?
grep -q SYNTH_DONE $S/vivado.log 2>/dev/null || { echo "synth ended (ramq wait rc=$SRC) without SYNTH_DONE: $(grep -m1 -iE '^ERROR' $S/vivado.log 2>/dev/null | cut -c1-200) $(grep -E '^ramq: ' ~/openav1-synth/launch-$PFM.log | tail -1 | cut -c1-200)" >> $L; echo PF_GATE_ABORT >> $L; exit 1; }
echo "synth pfm done $(date -u +%H:%M): $(grep -E '^\| CLB LUTs' $S/util.rpt | awk -F'|' '{print $3}' | tr -d ' ') LUT, URAM $(grep -E '^\| URAM' $S/util.rpt | awk -F'|' '{print $3}' | tr -d ' '); synth 8-6014/6901/8895/4767: $(grep -cE "Synth 8-(6014|6901|8895|4767)" $S/vivado.log)" >> $L
cd $G
printf "open_checkpoint %s\nwrite_verilog -force -mode funcsim %s/$BLK.v\nexit\n" "$S/post_synth.dcp" "$G" > fs_pf.tcl
vivado -mode batch -nojournal -nolog -source fs_pf.tcl > fs_pf.log 2>&1 || { echo "funcsim failed" >> $L; echo PF_GATE_ABORT >> $L; exit 1; }
echo "funcsim $(wc -l < $BLK.v) lines $(date -u +%H:%M)" >> $L
TIED=$(grep -c "\.v_hold(1'b[01])" $BLK.v); [ "$TIED" = 0 ] || { echo "ABORT: mc_filt v_hold is tied to a constant in the netlist ($TIED)" >> $L; echo PF_GATE_ABORT >> $L; exit 1; }
echo "netlist: v_hold driven; constant-tied input pins of u_mc: $(awk '/^  mc_filt /{f=1} f&&/\);/{f=0} f' $BLK.v | grep -c "(1'b[01])")" >> $L
OPT=$VOPT J=$VJ ~/vgate/vgate_build.sh $BLK $G/$BLK.v $G/replay_dec_host9fc.sv $VTHR > $G/vg_build.out 2>&1
grep -q BUILD_RC=0 ~/vgate/$BLK/build.log || { echo "verilator build failed: $(grep -m1 '%Error' ~/vgate/$BLK/build.log | cut -c1-160)" >> $L; echo PF_GATE_ABORT >> $L; exit 1; }
echo "verilator build done $(date -u +%H:%M): $(grep -E '^real' ~/vgate/$BLK/build.log | tr '\n' ' ')" >> $L
for T in $shards; do ( $HOME/ramq/bin/ramq run vreplay --name vr-$T -- ~/vgate/vgate_run.sh $BLK $G ${T}_vec.hex +fminit=zero > run_$T.log 2>&1 ) & sleep 2; done
wait
echo "replays done $(date -u +%H:%M)" >> $L
for T in $shards; do echo "$T: $(grep -E '^REPLAY|^wall' run_$T.log | tr '\n' ' ' | cut -c1-200)" >> $L; done
if [ "$(grep -cE "^${SP}[a-e]: .*0 mismatches -> PASS" $L)" = "$(echo $shards | wc -w)" ]; then echo PF_GATE_DONE >> $L; else echo "PF_GATE_DONE (NOT ALL PASS: read the lines above)" >> $L; fi
