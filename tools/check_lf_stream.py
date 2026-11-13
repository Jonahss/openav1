#!/usr/bin/env python3
"""check_lf_stream.py [seeds] [W] [H]: the streaming-order deblocking model (tb/lf_stream_model.py) against the
specification-order model on generated frames (as tb/test_lf_top.py makes them), for superblock rows of 64 and 128
lines and horizontal-edge lags of 0..3 columns. Lag 2 must be identical everywhere; the smaller lags are run to show
that the check can fail (a lag of 0 filters horizontal edges before the vertical edges to their right)."""
import copy, random, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "tb")); sys.path.insert(0, str(HERE))
import recon_model as rm, test_tile_syntax as TT, xcheck_frame as xf, lf_stream_model as ls, lf_stage_model as lst, lf_model as lfm

seeds = [int(x) for x in (sys.argv[1] if len(sys.argv) > 1 else "1,2,3,4,5,6").split(",")]
W = int(sys.argv[2]) if len(sys.argv) > 2 else 192
H = int(sys.argv[3]) if len(sys.argv) > 3 else 160
fails = {0: 0, 1: 0, 2: 0, 3: 0}; runs = 0; total_changed = 0; skipped = 0; stage_runs = 0; stage_fail = 0
for seed in seeds:
    fmt = random.Random(seed * 7).choice(["420", "444", "mono", "422"])
    try:
        d, args = TT.gen(seed, W, H, fmt)
        frame = {}; decs = []
        for th, data in d.tiles:
            dec = rm.FrameRecon(th, data, frame); dec.decode_tile(); decs.append(dec)
    except Exception as e:                                   # a generated stream the models reject (as the RTL suites skip)
        print(f"seed {seed} {fmt}: skipped ({type(e).__name__}: {str(e)[:60]})"); skipped += 1
        continue
    hdr = d.tiles[0][0]; state = xf.FrameState(hdr)
    for _d in decs: state.add(_d)
    pre = frame["planes"]
    for sb in (64, 128):
        for lag in (0, 1, 2, 3):
            ok, bad, changed = ls.check(hdr, state, pre, sb, lag)
            runs += 1
            if not ok: fails[lag] += 1
            if lag == 2:
                # the stage as it will see the picture: band by band, CTX lines carried, any access outside raises
                ref = copy.deepcopy(pre); lfm.LoopFilter(hdr, state, ref).apply()
                try:
                    got = lst.run_frame(hdr, state, pre, sb, lag)
                    stage_ok = all(got[p] == ref[p] for p in range(hdr.NumPlanes))
                except (IndexError, AssertionError) as e:
                    stage_ok = False; print(f"  stage: {type(e).__name__}: {e}")
                stage_runs += 1; stage_fail += 0 if stage_ok else 1
                total_changed += changed if ok else 0
                print(f"seed {seed} {fmt} bd{args['bd']} {W}x{H} sb{sb} lag 2: {'identical' if ok else 'DIFFERS ' + str(bad[:3])}, {changed if ok else '?'} samples changed by the filter, levels {hdr.loop_filter_level}")
print(f"stage model (band by band, {lst.CTX} context lines): {stage_runs} runs, {stage_fail} differ or reach outside the window")
print(f"runs {runs}; seeds skipped {skipped}; frames differing by lag: {fails}; samples changed (lag 2 runs) {total_changed}")
sys.exit(0 if fails[2] == 0 and fails[3] == 0 and total_changed > 0 and stage_fail == 0 else 1)
