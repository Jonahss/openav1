#!/usr/bin/env bash
# Nightly regression (run from cron on the batch host): RTL syntax decoder on generated streams, the full decoder
# on generated frames, and the corpus through the whole RTL pipeline compared with dav1d's pictures (model-free).
# Writes logs/nightly-<date>.txt with one line per suite and exits non-zero on any failure.
# Usage: tools/nightly.sh [tag]      (needs bootstrap.sh done and refout/ populated by tools/regress.sh once)
set -uo pipefail
cd "$(dirname "$0")/.."
source env.sh
tag="${1:-$(date +%Y-%m-%d)}"
out="logs/nightly-$tag.txt"
mkdir -p logs
: > "$out"
rc=0
run() {  # name, env..., -- command
  local name="$1"; shift
  local t0=$(date +%s)
  local log="logs/nightly-$tag-$name.log"
  if env "$@" > "$log" 2>&1 && grep -q "OK:" "$log"; then
    printf "PASS %-22s %5ss  %s\n" "$name" "$(( $(date +%s) - t0 ))" "$(grep -o 'OK: {.*}' "$log" | tail -1 | cut -c1-140)" >> "$out"
  else
    printf "FAIL %-22s %5ss  %s\n" "$name" "$(( $(date +%s) - t0 ))" "$(grep -m1 'AssertionError\|%Error' "$log" | cut -c1-160)" >> "$out"
    rc=1
  fi
}
run tile_syntax_plain  TS_SCREEN=0 TS_SEEDS=12 python tb/runner.py tile_syntax
run tile_syntax_screen TS_SCREEN=1 TS_SEEDS=24 python tb/runner.py tile_syntax
run dec_top_small      TS_SEEDS=12 python tb/runner.py dec_top
run dec_top_large      TS_SEEDS=101,102,103,104,105,106 TS_W=256 TS_H=160 python tb/runner.py dec_top
# the core FPGA configuration through its AXI-Lite host interface (rtl/dec_host.sv): registers, FIFO, read-back
run dec_host_core      FBX=9 FBY=9 ITX_LANES=2 FRAME_URAM=1 TS_SEEDS=1,2,3,4 TS_W=128 TS_H=96 TS_STAGE=lf python tb/runner.py dec_host
run dec_host_dma       FBX=9 FBY=9 ITX_LANES=2 FRAME_URAM=1 TS_SEEDS=5,6,7 TS_W=120 TS_H=72 TS_STAGE=lf TS_DMA=1 python tb/runner.py dec_host
run dec_host_pipe      FBX=9 FBY=9 ITX_LANES=2 FRAME_URAM=1 TS_SEEDS=8,9 TS_W=128 TS_H=96 TS_STAGE=lf TS_PIPE=1 TS_DMA=1 python tb/runner.py dec_host
IVFS=$(ls "$PWD"/streams/synth/*.ivf "$PWD"/streams/synth2/*.ivf | paste -sd,)
run corpus_full_rtl    TS_NOMODEL=1 TS_IVF="$IVFS" python tb/runner.py dec_top
# Argon feature streams (intra block copy, super-resolution, re-shown frames) when the Argon set is present
ARGON=$PWD/streams/argon/profile0_core/streams
if [ -d "$ARGON" ]; then
  AIVFS=$(for s in 4592 8304 5297 5366 60 5644 6617 7463; do echo -n "$ARGON/test$s.obu,"; done | sed 's/,$//')
  run argon_features     TS_NOMODEL=1 FBX=12 FBY=9 TS_IVF="$AIVFS" python tb/runner.py dec_top
  # inter frames through the RTL tile syntax vs the model (the frame sequence: inherited CDFs, segmentation map, motion field)
  IIVFS=$(for s in 10046 10001 10041; do echo -n "$ARGON/test$s.obu,"; done | sed 's/,$//')
  run tile_syntax_inter  TS_IVF="$IIVFS" python tb/runner.py tile_syntax
  # inter frames through the whole decoder (stage 2a: translational MC, average / distance compound), references =
  # the model's pictures in the AXI memory model (tb/test_dec_inter.py); test10046 is the stage-2a-clean Argon stream
  run dec_inter_argon    FBX=12 FBY=9 TS_STAGE=recon TS_IVF="$ARGON/test10046.obu" python tb/runner.py dec_top test_dec_inter
fi
# the stage-2a / 2b streams (OBMC, wedge + DIFFWTD masks, inter-intra) encoded from streams/gen/motion_8bit.y4m (tools/gen_y4m_motion.py + aomenc, features restricted)
GIVFS=$(for s in inter2a_8bit inter2b_8bit inter2c_8bit inter2d_8bit inter_obmc_8bit inter_mask_8bit inter_mask2_8bit inter_ii_8bit; do echo -n "$PWD/streams/inter/$s.ivf,"; done | sed 's/,$//')
run dec_inter            TS_STAGE=recon TS_IVF="$GIVFS" python tb/runner.py dec_top test_dec_inter
echo "DONE rc=$rc $(date -Is)" >> "$out"
cat "$out"
exit $rc
