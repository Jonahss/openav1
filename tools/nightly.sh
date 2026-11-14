#!/usr/bin/env bash
# Nightly regression (run from cron on the batch host): RTL syntax decoder on generated streams, the full decoder
# on generated frames, and the corpus through the whole RTL pipeline compared with dav1d's pictures (model-free).
# Writes logs/nightly-<date>.txt with one line per suite and exits non-zero on any failure.
# Usage: [NIGHTLY_ONLY=regex] tools/nightly.sh [tag]   (needs bootstrap.sh done and refout/ populated by tools/regress.sh once)
set -uo pipefail
cd "$(dirname "$0")/.."
source env.sh
tag="${1:-$(date +%Y-%m-%d)}"
out="logs/nightly-$tag.txt"
mkdir -p logs
: > "$out"
rc=0
# NIGHTLY_PAR=<n> (default 1: one suite after another, as before): up to n suites at the same time, each with its own
# build directory (TS_BUILD_TAG = the suite's name). The lines of the result file then come in the order the suites
# finish; rc is counted from the FAIL lines at the end. The suites were sequential from habit and from the days the
# CPU ran hot; 22 of them took 3.4 h on a 64-thread machine (2026-11-11).
PAR=${NIGHTLY_PAR:-1}
slot() { [[ $PAR -gt 1 ]] || return 0; while (( $(jobs -rp | wc -l) >= PAR )); do sleep 2; done; }
run() {
  [[ -n "${NIGHTLY_ONLY:-}" && ! "$1" =~ ${NIGHTLY_ONLY} ]] && return 0
  if [[ $PAR -gt 1 ]]; then slot; local n="$1"; shift; ( run_one "$n" TS_BUILD_TAG="$n" "$@" ) & sleep 1
  else run_one "$@"; fi
}
runsh() {
  [[ -n "${NIGHTLY_ONLY:-}" && ! "$1" =~ ${NIGHTLY_ONLY} ]] && return 0
  if [[ $PAR -gt 1 ]]; then slot; local n="$1"; shift; ( runsh_one "$n" TS_BUILD_TAG="$n" "$@" ) & sleep 1
  else runsh_one "$@"; fi
}
run_one() {  # name, env..., -- command  (NIGHTLY_ONLY=<regex> runs just the suites whose name matches: reruns after a fix)
  local name="$1"; shift
  [[ -n "${NIGHTLY_ONLY:-}" && ! "$name" =~ ${NIGHTLY_ONLY} ]] && return 0
  local t0=$(date +%s)
  local log="logs/nightly-$tag-$name.log"
  # Every step is capped (NIGHTLY_STEP_TIMEOUT seconds, default 4 h; the longest step, dec_inter, takes ~2.7 h): on
  # 2026-10-31 a hung simulation (test4592, intrabc at the top edge) held argon_features for 10.5 h and the rest of
  # the night with it. SIGINT lets cocotb print what it had; the FAIL line says it was a timeout.
  local step_rc=0
  timeout --signal=INT --kill-after=60 "${NIGHTLY_STEP_TIMEOUT:-14400}" env "$@" > "$log" 2>&1 || step_rc=$?
  if [[ $step_rc -eq 0 ]] && grep -q "OK:" "$log"; then
    printf "PASS %-22s %5ss  %s\n" "$name" "$(( $(date +%s) - t0 ))" "$(grep -o 'OK: {.*}' "$log" | tail -1 | cut -c1-140)" >> "$out"
  else
    local why
    if [[ $step_rc -eq 124 || $step_rc -eq 137 ]]; then why="TIMEOUT after ${NIGHTLY_STEP_TIMEOUT:-14400}s: $(grep -E '^\[rc\]' "$log" | tail -1 | cut -c1-120)"
    else why=$(grep -m1 'AssertionError\|%Error' "$log" | cut -c1-160); fi
    printf "FAIL %-22s %5ss  %s\n" "$name" "$(( $(date +%s) - t0 ))" "$why" >> "$out"
    rc=1
  fi
}
# runsh: like run, but the streams of TS_IVF are split round-robin over NIGHTLY_SHARDS processes (default 4) after one
# build (TS_BUILD_ONLY=1), each shard in its own run directory (TS_TEST_DIR). The suites that loop over many streams
# (dec_inter 3 h, corpus_full_rtl 2 h, argon_features 1 h) were the nightly's wall; 2026-11-07. PASS needs every shard OK.
#   runsh name env... TS_IVF="a,b,c" python tb/runner.py top [test]
runsh_one() {
  local name="$1"; shift
  [[ -n "${NIGHTLY_ONLY:-}" && ! "$name" =~ ${NIGHTLY_ONLY} ]] && return 0
  local t0=$(date +%s) n=${NIGHTLY_SHARDS:-4} envs=() ivf="" cmd=()
  while [[ $# -gt 0 ]]; do case "$1" in TS_IVF=*) ivf="${1#TS_IVF=}";; python) cmd=("$@"); break;; *) envs+=("$1");; esac; shift; done
  IFS=, read -r -a list <<< "$ivf"
  [[ ${#list[@]} -lt $n ]] && n=${#list[@]}
  local log="logs/nightly-$tag-$name.log" step_rc=0
  env "${envs[@]}" TS_IVF="${list[0]}" TS_BUILD_ONLY=1 "${cmd[@]}" > "$log" 2>&1 || step_rc=$?
  if [[ $step_rc -ne 0 ]]; then printf "FAIL %-22s %5ss  build: %s\n" "$name" "$(( $(date +%s) - t0 ))" "$(grep -m1 '%Error' "$log" | cut -c1-140)" >> "$out"; rc=1; return; fi
  local i pids=() shard_ivf
  for ((i = 0; i < n; i++)); do
    shard_ivf=$(for ((j = i; j < ${#list[@]}; j += n)); do echo -n "${list[j]},"; done | sed 's/,$//')
    mkdir -p "logs/shards/$tag-$name-$i"
    ( timeout --signal=INT --kill-after=60 "${NIGHTLY_STEP_TIMEOUT:-14400}" env "${envs[@]}" TS_IVF="$shard_ivf" TS_TEST_DIR="$PWD/logs/shards/$tag-$name-$i" "${cmd[@]}" > "$log.$i" 2>&1 ) &
    pids+=($!); sleep 2
  done
  local fails=0 oks=""
  for ((i = 0; i < n; i++)); do
    wait "${pids[i]}" || fails=$((fails + 1))
    grep -q "OK:" "$log.$i" || fails=$((fails + 1))
    oks="$oks $i:$(grep -o 'OK: {.*}' "$log.$i" | tail -1 | grep -oE "'frames': [0-9]+" | grep -oE '[0-9]+' || echo FAIL)"
  done
  if [[ $fails -eq 0 ]]; then printf "PASS %-22s %5ss  OK: %d shards, frames per shard:%s\n" "$name" "$(( $(date +%s) - t0 ))" "$n" "$oks" >> "$out"
  else local why; why=$(grep -h -m1 'AssertionError\|%Error\|fatal' "$log".[0-9]* | head -1 | cut -c1-150); printf "FAIL %-22s %5ss  %d of %d shards failed: %s\n" "$name" "$(( $(date +%s) - t0 ))" "$fails" "$n" "$why" >> "$out"; rc=1; fi
}
# identifiers used before their declaration: Verilator accepts them, Vivado makes implicit 1-bit nets (2026-11-09: mc_filt's
# v_hold was tied to 0 in three netlists while every suite and gate rung passed)
run decl_order         bash -c 'python3 tools/check_decl_order.py rtl/*.sv && echo "OK: {decl_order: clean}" || { echo "%Error: identifiers used before their declaration"; exit 1; }'
run lf_edge_unit       TS_SEEDS=2,3,7,8,20,47,70 TS_W=192 TS_H=160 python tb/runner.py lf_edge   # the deblocking edge decision (apply, size, strength) vs lf_model, every position of every plane
run lf_edge_inter      TS_LF_VEC="$PWD/tb/vectors/lf_edge_inter" python tb/runner.py lf_edge   # six inter frames of Argon test47 / test6491 / test11 dumped by dec_inter_lr (TS_LF_VEC_OUT): reference and mode deltas, the skip-and-inter rule
run lf_filt_chroma     LFF_PIPE=1 LF_CHROMA_ONLY=1 python tb/runner.py lf_filt   # the chroma-only build (LUMA = 0)
run lf_stream_chroma   LF_CHROMA_ONLY=1 TS_SEEDS=2,3,7,20,47,70 TS_W=192 TS_H=160 python tb/runner.py lf_stream   # the chroma-only stage on the chroma planes
run lf_stream_short    TS_SEEDS=2,3,7,20,47 TS_W=192 TS_H=136 python tb/runner.py lf_stream   # a last band of two luma block rows (one chroma row in 4:2:0): bands shorter than the stage's pipeline
run lf_stage_short     TS_SEEDS=2,3,7,20 TS_W=176 TS_H=136 python tb/runner.py lf_stage_tb_top test_lf_stage
run lf_stage_unit      TS_SEEDS=2,3,7,20,47,70 TS_W=192 TS_H=160 python tb/runner.py lf_stage_tb_top test_lf_stage   # mi_store + lf_feed + lf_edge + lf_stream vs lf_model: the deblock stage with nothing supplied but what the decoder has
run cdef_row_unit      python tb/runner.py cdef_row         # the CDEF line filter (8 lanes) of the streaming chain vs cdef_model
run cdef_row_4         CR_N=4 python tb/runner.py cdef_row   # the 4-lane build (4:2:0 chroma lines)
run lf_filt_unit       python tb/runner.py lf_filt          # the deblocking sample filter of the streaming chain vs lf_model (one line across an edge)
run lf_filt_pipe       LFF_PIPE=1 python tb/runner.py lf_filt   # the same filter in its two-stage form (PIPE = 1): outputs from the stage registers alone
run lf_stream_unit     TS_SEEDS=2,3,7,8,9,10,20,47,70 TS_W=192 TS_H=160 python tb/runner.py lf_stream   # the streaming deblock stage vs lf_model, band by band (seeds 7, 20, 47, 70: samples that depend on the last column's vertical edge)
run edge_line_unit     python tb/runner.py edge_line        # the unfiltered last-line copy (intra edge reads; LF a row earlier) vs a model
run mc_filt_unit       python tb/runner.py mc_filt          # the motion-compensation filter vs the model: fuzz + back-to-back (overlapped) runs
run tile_syntax_plain  TS_SCREEN=0 TS_SEEDS=12 python tb/runner.py tile_syntax
run tile_syntax_screen TS_SCREEN=1 TS_SEEDS=24 python tb/runner.py tile_syntax
run dec_top_small      TS_SEEDS=12 python tb/runner.py dec_top
run dec_top_intra0     INTER=0 TS_SEEDS=12 python tb/runner.py dec_top          # the intra-only build (INTER=0: the K26 board)
run dec_top_large      TS_SEEDS=101,102,103,104,105,106 TS_W=256 TS_H=160 python tb/runner.py dec_top
# the core FPGA configuration through its AXI-Lite host interface (rtl/dec_host.sv): registers, FIFO, read-back
run dec_host_core      FBX=9 FBY=9 ITX_LANES=2 FRAME_URAM=1 TS_SEEDS=1,2,3,4 TS_W=128 TS_H=96 TS_STAGE=lf python tb/runner.py dec_host
run dec_host_dma       FBX=9 FBY=9 ITX_LANES=2 FRAME_URAM=1 TS_SEEDS=5,6,7 TS_W=120 TS_H=72 TS_STAGE=lf TS_DMA=1 python tb/runner.py dec_host
run dec_host_pipe      FBX=9 FBY=9 ITX_LANES=2 FRAME_URAM=1 TS_SEEDS=8,9 TS_W=128 TS_H=96 TS_STAGE=lf TS_PIPE=1 TS_DMA=1 python tb/runner.py dec_host
# the RING build (docs/picture-memory.md step A): every frame buffer an UltraRAM ring (fb0 + mirror, fb0s, fb1, fb2), CDEF / SR / LR
# under the frame sequencer, the final picture flushed row by row; seeds 7 / 17 / 20 have CDEF + LR, 8 neither; 5 rows at 320 wrap the rings
# (fb0 needs 3 + CDEF + LR = 5 rows: recon, LF, CDEF, LR in flight plus the hold slot; a shallower ring flags the frame unsupported)
run dec_host_ring      FBX=9 FBY=9 ITX_LANES=2 FRAME_URAM=1 FRAME_WIN=5 FRAME_WIN1=3 FRAME_WIN2=3 TS_SEEDS=7,8,17,20 TS_W=128 TS_H=96 TS_STAGE=lr TS_PIPE=1 TS_DMA=1 TS_WIN=1 python tb/runner.py dec_host
run dec_host_ring_tall FBX=9 FBY=9 ITX_LANES=2 FRAME_URAM=1 FRAME_WIN=5 FRAME_WIN1=3 FRAME_WIN2=3 TS_SEEDS=28,32 TS_W=128 TS_H=320 TS_STAGE=lr TS_PIPE=1 TS_DMA=1 TS_WIN=1 python tb/runner.py dec_host
# inter streams through dec_host (registers, sequencer, flush; references = the flushed pictures, as on the board): tb/test_dec_host_inter.py
run dec_host_inter     FBX=10 FBY=10 ITX_LANES=2 FRAME_URAM=1 FRAME_WIN=5 FRAME_WIN1=3 FRAME_WIN2=3 TS_IVF="$PWD/streams/inter/inter2a_8bit.ivf,$PWD/streams/inter/inter_obmc_8bit.ivf" python tb/runner.py dec_host test_dec_host_inter
IVFS=$(ls "$PWD"/streams/synth/*.ivf "$PWD"/streams/synth2/*.ivf | paste -sd,)
runsh corpus_full_rtl  TS_NOMODEL=1 TS_IVF="$IVFS" python tb/runner.py dec_top
# Argon feature streams (intra block copy, super-resolution, re-shown frames) when the Argon set is present
ARGON=$PWD/streams/argon/profile0_core/streams
if [ -d "$ARGON" ]; then
  AIVFS=$(for s in 4592 8304 5297 5366 60 5644 6617 7463; do echo -n "$ARGON/test$s.obu,"; done | sed 's/,$//')
  runsh argon_features   TS_NOMODEL=1 FBX=12 FBY=9 TS_IVF="$AIVFS" python tb/runner.py dec_top
  # inter frames through the RTL tile syntax vs the model (the frame sequence: inherited CDFs, segmentation map, motion field)
  IIVFS=$(for s in 10046 10001 10041; do echo -n "$ARGON/test$s.obu,"; done | sed 's/,$//')
  run tile_syntax_inter  TS_IVF="$IIVFS" python tb/runner.py tile_syntax
  # inter frames through the whole decoder (stage 2a: translational MC, average / distance compound), references =
  # the model's pictures in the AXI memory model (tb/test_dec_inter.py); test10046 is the stage-2a-clean Argon stream
  run lf_edge_argon      TS_IVF="$ARGON/test10963.obu,$ARGON/test1057_6368.obu,$ARGON/test10446_512_421.obu" python tb/runner.py lf_edge   # frame 0s where the level fallback to the neighbour changes the picture (delta LF / segment features)
  run dec_inter_argon    FBX=12 FBY=9 TS_STAGE=recon TS_IVF="$ARGON/test10046.obu" python tb/runner.py dec_top test_dec_inter
  # inter frames through the WHOLE pipeline (deblocking with the inter ref / mode deltas and the skipped-inter edge rule,
  # CDEF, super-resolution, loop restoration): the corpus runs stop at recon, which hid both lf_top gaps until 2026-10-20
  run dec_inter_lr       FBX=12 FBY=12 TS_STAGE=lr TS_IVF="$ARGON/test47.obu,$ARGON/test6491.obu,$ARGON/test11.obu" python tb/runner.py dec_top test_dec_inter
  # Argon inter streams through dec_host's RING build: test47 multi-tile rows + super-res, test6641 / test8854 super-res + CDEF + LR (test11 is sb128: model only there)
  run dec_host_inter_argon FBX=10 FBY=10 ITX_LANES=2 FRAME_URAM=1 FRAME_WIN=5 FRAME_WIN1=3 FRAME_WIN2=3 TS_IVF="$ARGON/test47.obu,$ARGON/test6641.obu,$ARGON/test8854.obu" python tb/runner.py dec_host test_dec_host_inter
  # stage B (docs/frame-state.md: the card loads / saves the frame state itself) with the host's 10-buffer pool and recycling rule
  # (TS_POOL): test5439 is the stream whose state areas aliased on the card (2026-11-01), test47 the multi-tile-row one
  run dec_host_cardload  FBX=10 FBY=10 ITX_LANES=2 FRAME_URAM=1 FRAME_WIN=5 FRAME_WIN1=3 FRAME_WIN2=3 TS_FSTATE_B=1 TS_POOL=10 TS_IVF="$ARGON/test5439.obu,$ARGON/test47.obu" python tb/runner.py dec_host test_dec_host_inter
  # the card's own configuration (two tile engines, an 8-deep reference ring, sb128, FBX 11) on two streams of card census 15
  # (2026-11-10): the run-record reuse under the blend pipelines (rung B) passed every suite above and the gate rung and
  # showed only on the card, in 30 Argon streams; test5530 fails on 86c2847 only with TE=2, test6687 with one engine as well
  run dec_host_census15  FBX=11 FBY=11 ITX_LANES=2 FRAME_URAM=1 FRAME_WIN=5 FRAME_WIN1=3 FRAME_WIN2=3 TS_FSTATE_B=1 TS_POOL=10 SB128=1 TE=2 REF_RING=8 TS_IVF="$ARGON/test5530.obu,$ARGON/test6687.obu" python tb/runner.py dec_host test_dec_host_inter
fi
# the stage-2a / 2b / 2c.1 streams (OBMC, wedge + DIFFWTD masks, inter-intra, scaled references via aomenc --resize-mode) encoded from streams/gen/motion_8bit.y4m (tools/gen_y4m_motion.py + aomenc, features restricted)
GIVFS=$(for s in inter2a_8bit inter2b_8bit inter2c_8bit inter2d_8bit inter_obmc_8bit inter_mask_8bit inter_mask2_8bit inter_ii_8bit inter_scaled_8bit inter_scaled2_8bit; do echo -n "$PWD/streams/inter/$s.ivf,"; done | sed 's/,$//')
runsh dec_inter          TS_STAGE=recon TS_IVF="$GIVFS" python tb/runner.py dec_top test_dec_inter
wait
[[ $PAR -gt 1 ]] && rc=$(( $(grep -c '^FAIL' "$out") > 0 ? 1 : 0 ))
echo "DONE rc=$rc $(date -Is)" >> "$out"
cat "$out"
exit $rc
