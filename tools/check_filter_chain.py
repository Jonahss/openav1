#!/usr/bin/env python3
"""check_filter_chain.py <stream.ivf>... | --seeds a,b,c [W H]: the stage models of the streaming filter chain
(tb/lf_stage_model.py -> tb/cdef_stage_model.py -> tb/lr_stage_model.py) run band by band, against the frame-at-once models (lf_model,
cdef_model, lr_model) on the intra frames the Python reconstruction can make: generated frames or the first frame(s) of real
streams. Reports per frame whether the chain's output is identical, how much the filters changed, and the most lines
each stage held."""
import copy, random, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "tb")); sys.path.insert(0, str(HERE))
import recon_model as rm, test_tile_syntax as TT, xcheck_frame as xf
import lf_model as lfm, cdef_model as cdm, lr_model as lrm, lf_stage_model as lst, cdef_stage_model as cst, lr_stage_model as rst

def frames(argv):
    if argv and argv[0] == "--seeds":
        seeds = [int(x) for x in argv[1].split(",")]
        W = int(argv[2]) if len(argv) > 2 else 192; H = int(argv[3]) if len(argv) > 3 else 160
        for seed in seeds:
            fmt = random.Random(seed * 7).choice(["420", "444", "mono", "422"])
            try:
                d, args = TT.gen(seed, W, H, fmt)
                yield f"seed {seed} {fmt} bd{args['bd']} {W}x{H}", d.tiles
            except Exception as e:
                print(f"seed {seed}: skipped ({type(e).__name__})")
    else:
        import obu_parser as op
        for path in argv:
            d = op.Decoder(); d.feed_ivf(open(path, "rb").read())
            yield f"{Path(path).name} frame 0", [d.tiles[i] for i in d.frames[0]]

def chain(hdr, state, pre, sb_lines):
    """Band by band through the deblock, CDEF and restoration stage models; returns (deblocked, cdef, restored, held)."""
    cdef_on = bool(hdr.enable_cdef and not hdr.CodedLossless and not hdr.allow_intrabc)
    lr_on = lr_wanted(hdr)
    heights = [len(pl) for pl in pre]
    lf = lst.DeblockStage(hdr, state)
    cd = cst.CdefStage(hdr, state, heights)
    lr = rst.LrStage(hdr, state, heights)
    n = hdr.NumPlanes
    dbl = [[None] * heights[p] for p in range(n)]; out = [[None] * heights[p] for p in range(n)]; fin_lr = [[None] * heights[p] for p in range(n)]
    band = sb_lines // 4
    for row0 in range(0, hdr.MiRows, band):
        row1 = min(row0 + band, hdr.MiRows); last = row1 >= hdr.MiRows
        lines = []
        for p in range(n):
            sy = hdr.subsampling_y if p else 0
            y0 = (row0 * 4) >> sy; y1 = len(pre[p]) if last else (row1 * 4) >> sy
            lines.append([list(r) for r in pre[p][y0:y1]])
        fin = lf.band(row0, row1, lines, last)
        for p in range(n):
            for y, ln in fin[p]: dbl[p][y] = ln[:]
        res = cd.feed(fin, last) if cdef_on else [[(y, ln[:]) for y, ln in fin[p]] for p in range(n)]
        for p in range(n):
            for y, ln in res[p]:
                assert out[p][y] is None; out[p][y] = ln[:]
        if lr_on:
            r2 = lr.feed(fin, res, last)
            for p in range(n):
                for y, ln in r2[p]:
                    assert fin_lr[p][y] is None; fin_lr[p][y] = ln
    return dbl, out, (fin_lr if lr_on else None), (cd.held_max, lr.held_max, lr.cur_kept_max)

def lr_wanted(hdr):
    return (not all(t == 0 for t in hdr.FrameRestorationType[:hdr.NumPlanes])) and not getattr(hdr, "use_superres", 0)

bad = 0; n_frames = 0; cdef_frames = 0; lr_frames = 0; sr_skipped = 0; cdef_changed = 0; lr_changed = 0; held = [[0] * 3 for _ in range(3)]
for tag, tiles in frames(sys.argv[1:]):
    try:
        frame = {}; decs = []
        for th, data in tiles:
            dec = rm.FrameRecon(th, data, frame); dec.decode_tile(); decs.append(dec)
    except Exception as e:
        print(f"{tag}: skipped ({type(e).__name__}: {str(e)[:60]})"); continue
    hdr = tiles[0][0]; state = xf.FrameState(hdr)
    for _d in decs: state.add(_d)
    pre = frame["planes"]; n = hdr.NumPlanes
    ref_lf = copy.deepcopy(pre); lfm.LoopFilter(hdr, state, ref_lf).apply()
    cdef_on = bool(hdr.enable_cdef and not hdr.CodedLossless and not hdr.allow_intrabc)
    ref_cd = cdm.Cdef(hdr, state, copy.deepcopy(ref_lf)).apply() if cdef_on else copy.deepcopy(ref_lf)
    lr_on = lr_wanted(hdr)
    if getattr(hdr, "use_superres", 0): sr_skipped += 1
    ref_lr = lrm.LoopRestoration(hdr, state, ref_lf, ref_cd).apply() if lr_on else None
    for sb in (64, 128):
        n_frames += 1
        try:
            dbl, out, flr, hm = chain(hdr, state, pre, sb)
            ok_lf = all(dbl[p] == ref_lf[p] for p in range(n))
            ok_cd = all(out[p] == ref_cd[p] for p in range(n))
            ok_lr = (not lr_on) or all(flr[p] == ref_lr[p] for p in range(n))
            msg = ""
        except (IndexError, AssertionError) as e:
            ok_lf = ok_cd = ok_lr = False; hm = None; msg = f" [{type(e).__name__}: {e}]"
        ok = ok_lf and ok_cd and ok_lr
        chc = sum(1 for p in range(n) for y in range(len(ref_cd[p])) if ref_cd[p][y] != ref_lf[p][y]) if cdef_on else 0
        chl = sum(1 for p in range(n) for y in range(len(ref_lr[p])) if ref_lr[p][y] != ref_cd[p][y]) if lr_on else 0
        if ok:
            cdef_frames += 1 if cdef_on else 0; lr_frames += 1 if lr_on else 0; cdef_changed += chc; lr_changed += chl
            for i in range(3):
                held[i] = [max(a, b) for a, b in zip(held[i], list(hm[i]) + [0, 0, 0])]
        bad += 0 if ok else 1
        d = lambda on, okk, ch: "off" if not on else (f"identical ({ch} lines changed)" if okk else "DIFFERS")
        print(f"{tag} sb{sb}: deblock {'identical' if ok_lf else 'DIFFERS'}, cdef {d(cdef_on, ok_cd, chc)}, restoration {d(lr_on, ok_lr, chl)}{msg}")
print(f"frames x superblock sizes {n_frames}, failing {bad}; CDEF on in {cdef_frames} ({cdef_changed} lines changed), restoration on in {lr_frames} ({lr_changed} lines changed), super-res frames (restoration not modelled) {sr_skipped}")
print(f"carried between superblock rows, per plane: CDEF stage {held[0]} deblocked lines; restoration stage {held[1]} CDEF lines and {held[2]} deblocked boundary lines")
sys.exit(1 if bad or not n_frames else 0)
