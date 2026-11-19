#!/usr/bin/env bash
# gate_record.sh <worktree> <gate dir> <build tag> <shard> <in list> <out list> [VAR=value ...]: one recorded shard of the
# gate rung (bin/gate.sh step C). Runs the RTL twin (tb/runner.py dec_host test_dec_host_inter) on the RTL build that
# gate.sh made beforehand (TS_BUILD_TAG=<build tag>, TS_BUILD_ONLY=1), in a test directory of its own
# (build/dec_host_fb9x9_rec_<shard>), so the five shards run side by side; the VAR=value arguments are the twin's
# environment (parameters, TS_IVF, TS_FSTATE_B ...). Then the port VCD -> per-cycle vectors (tools/vcd2vec.py), cut to the
# harness's 450,000. Writes <gate dir>/rec_<shard>.log (the twin), <shard>_vec.hex / .out, and <gate dir>/rec_<shard>.status:
# the "twin ..." and "vectors ..." lines for the gate's verdict file and a last line REC_OK or REC_FAIL <why>.
set -u
W=$1; G=$2; RT=$3; T=$4; IN=$5; OUT=$6; shift 6
S=$G/rec_$T.status; : > $S
fail() { echo "REC_FAIL $*" >> $S; exit 1; }
cd "$W" || fail "no worktree $W"
source env.sh
TD=$W/build/dec_host_fb9x9_rec_$T; rm -rf $TD
t0=$(date +%s)
env "$@" TS_BUILD_TAG=$RT TS_TEST_DIR=$TD python tb/runner.py dec_host test_dec_host_inter > $G/rec_$T.log 2>&1
TRC=$?
echo "twin $T rc=$TRC $(( $(date +%s) - t0 ))s $(grep -oE "OK: \{.*|AssertionError.*" $G/rec_$T.log | tail -1 | cut -c1-160) $(date -u +%H:%M)" >> $S
V=$TD/sim/run/dump.vcd
[ -f $V ] || fail "no dump.vcd in $TD/sim/run for $T"
python tools/vcd2vec.py $V TOP.dec_host clk --in $IN --out $OUT > $G/${T}_raw.hex 2> $G/${T}_vec.err
cut -d" " -f1 $G/${T}_raw.hex | head -n 450000 > $G/${T}_vec.hex; cut -d" " -f2 $G/${T}_raw.hex | head -n 450000 > $G/${T}_vec.hex.out
NV=$(wc -l < $G/${T}_vec.hex); NR=$(wc -l < $G/${T}_raw.hex)
echo "vectors $T: $NV of $NR recorded cycles ($(tail -1 $G/${T}_vec.err | cut -c1-80))" >> $S
[ "$TRC" = 0 ] && grep -q "OK: {" $G/rec_$T.log && ! grep -q "AssertionError" $G/rec_$T.log && [ "$NV" -ge 100000 ] \
  || fail "twin $T rc=$TRC, $NV cycles recorded (need rc 0, an OK line, no AssertionError, >= 100000)"
rm -f $V $G/${T}_raw.hex
echo REC_OK >> $S
