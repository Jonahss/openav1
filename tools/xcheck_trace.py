#!/usr/bin/env python3
"""Cross-check the spec models against a dav1d v3 trace.

  intra: every P event -> intra_model.predict_intra(edges, mode, ...) must equal dav1d's pixels
  cfl:   every Q event -> DC prediction + Round2Signed(alpha * ac, 6), clipped
  itx:   every R event -> Clip1(pred + flip(itx_model(Dequant from the matching C event)))

Usage: tools/xcheck_trace.py <trace.txt> <bit_depth> [--max N] [-v]
"""
import sys
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tb"))
import intra_model as im  # noqa: E402
import itx_model as tm    # noqa: E402

# dav1d DSP mode indices (levels.h)
LEFT_DC, TOP_DC, DC_128, Z1, Z2, Z3 = 3, 4, 5, 6, 7, 8
FILTER_PRED = 13
WHT_WHT = 16
FLIP_UD = {tm.FLIPADST_DCT, tm.FLIPADST_ADST, tm.V_FLIPADST, tm.FLIPADST_FLIPADST}
FLIP_LR = {tm.DCT_FLIPADST, tm.ADST_FLIPADST, tm.H_FLIPADST, tm.FLIPADST_FLIPADST}


def edges_from_dav1d(vals, w, h):
    """vals = edge[-2h .. 2w] as dumped. Returns (above[0..w+h-1], left[0..w+h-1], topleft) per spec
    replication: entries beyond dav1d's 2w / 2h copies repeat the last one (spec 7.11.2.1 Min(limit))."""
    off = 2 * h  # index of edge[0]
    tl = vals[off]
    above = [vals[off + 1 + min(i, 2 * w - 1)] for i in range(w + h)]
    left = [vals[off - 1 - min(i, 2 * h - 1)] for i in range(w + h)]
    return above, left, tl


def check_pred(f, bd, verbose):
    plane, x4, y4, w, h = f[1:6]
    mode, m, angle, maxw, maxh = f[6:11]
    n_edge = 2 * h + 2 * w + 1
    edge = f[11:11 + n_edge]
    pix = f[11 + n_edge:11 + n_edge + w * h]
    assert len(pix) == w * h, "short P line"
    above, left, tl = edges_from_dav1d(edge, w, h)
    log2w, log2h = w.bit_length() - 1, h.bit_length() - 1
    p_angle = angle & 511
    is_sm = (angle >> 9) & 1
    edge_en = angle >> 10
    have_left = 0 if m in (TOP_DC, DC_128) else 1
    have_above = 0 if m in (LEFT_DC, DC_128) else 1
    kw = dict(have_left=have_left, have_above=have_above, filter_type=is_sm,
              enable_intra_edge_filter=edge_en, above_px=min(w, maxw), left_px=min(h, maxh))
    if mode == FILTER_PRED:
        exp = im.predict_intra(above, left, tl, 0, log2w, log2h, bd, use_filter_intra=True,
                               filter_intra_mode=p_angle, **kw)
        tag = "filter%d" % p_angle
    elif im.is_directional(mode):
        delta = (p_angle - im.MODE_TO_ANGLE[mode]) // im.ANGLE_STEP
        # dav1d pre-filters the Z2 corner inside edge preparation; the dumped corner is already filtered
        corner_done = (m == Z2 and (w + h) >= 24 and edge_en)
        exp = im.predict_intra(above, left, tl, mode, log2w, log2h, bd, angle_delta=delta,
                               corner_prefiltered=corner_done, **kw)
        tag = "dir%d/%+d" % (mode, delta)
    else:
        exp = im.predict_intra(above, left, tl, mode, log2w, log2h, bd, **kw)
        tag = ["dc", "v", "h", "", "", "", "", "", "", "smooth", "smooth_v", "smooth_h", "paeth"][mode]
        if mode == 0:
            tag += ["", "", "", "_left", "_top", "_128"][m] if m in (3, 4, 5) else ""
    got = [v for row in exp for v in row]
    ok = got == pix
    if not ok and verbose:
        print(f"PRED MISMATCH plane {plane} ({x4},{y4}) {w}x{h} mode {mode} m {m} angle {angle} maxw {maxw} maxh {maxh}")
        print("  exp", got[:16]); print("  got", pix[:16])
    return tag, ok, (plane, x4, y4), pix


def check_cfl(f, bd, verbose):
    plane, x4, y4, w, h, m, alpha = f[1:8]
    n_edge = 2 * h + 2 * w + 1
    edge = f[8:8 + n_edge]
    ac = f[8 + n_edge:8 + n_edge + w * h]
    pix = f[8 + n_edge + w * h:8 + n_edge + 2 * w * h]
    assert len(pix) == w * h, "short Q line"
    above, left, tl = edges_from_dav1d(edge, w, h)
    log2w, log2h = w.bit_length() - 1, h.bit_length() - 1
    have_left = 0 if m in (TOP_DC, DC_128) else 1
    have_above = 0 if m in (LEFT_DC, DC_128) else 1
    dc = im.predict_intra(above, left, tl, im.DC_PRED, log2w, log2h, bd, have_left=have_left, have_above=have_above)
    exp = []
    for i in range(h):
        for j in range(w):
            exp.append(im.clip1(dc[i][j] + im.round2signed(alpha * ac[i * w + j], 6), bd))
    ok = exp == pix
    if not ok and verbose:
        print(f"CFL MISMATCH plane {plane} ({x4},{y4}) {w}x{h} m {m} alpha {alpha}")
        print("  exp", exp[:16]); print("  got", pix[:16])
    return ok, (plane, x4, y4), pix


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    verbose = "-v" in sys.argv
    max_n = None
    if "--max" in sys.argv:
        max_n = int(sys.argv[sys.argv.index("--max") + 1])
    path, bd = args[0], int(args[1])
    preds = {}          # (plane,x4,y4) -> pixels of the latest prediction
    coefs = {}          # (plane,x4,y4) -> (tx, txtp, w, h, dequant rows)
    stats = defaultdict(Counter)
    n = 0
    with open(path) as fh:
        for line in fh:
            if not line or line[0] in "#STU":
                continue
            f = line.split()
            kind = f[0]
            f = [f[0]] + [int(x) for x in f[1:]]
            if kind == "P":
                tag, ok, key, pix = check_pred(f, bd, verbose)
                stats["pred"][("ok" if ok else "BAD", tag)] += 1
                preds[key] = pix
            elif kind == "Q":
                ok, key, pix = check_cfl(f, bd, verbose)
                stats["cfl"]["ok" if ok else "BAD"] += 1
                preds[key] = pix
            elif kind == "C":
                plane, x4, y4, tx, txtp, eob, w, h = f[1:9]
                cw, ch = min(w, 32), min(h, 32)
                vals = f[9:9 + cw * ch]
                rows = [vals[r * cw:(r + 1) * cw] for r in range(ch)]
                coefs[(plane, x4, y4)] = (tx, txtp, w, h, rows)
            elif kind == "R":
                plane, x4, y4, w, h, tx, txtp = f[1:8]
                pix = f[8:8 + w * h]
                key = (plane, x4, y4)
                if key not in coefs or key not in preds:
                    stats["itx"]["skipped(no pred/coef)"] += 1
                    continue
                ctx, ctxtp, cw_, ch_, rows = coefs.pop(key)
                pred = preds[key]
                lossless = (txtp == WHT_WHT)
                ttype = tm.DCT_DCT if lossless else txtp
                deq = [[0] * w for _ in range(h)]
                for i, r in enumerate(rows):
                    for j, v in enumerate(r):
                        deq[i][j] = v
                try:
                    res = tm.inverse_transform_2d(deq, tx, ttype, bd, lossless)
                except tm.Nonconformant as e:
                    stats["itx"][("nonconformant", str(e)[:30])] += 1
                    continue
                flip_ud = ttype in FLIP_UD
                flip_lr = ttype in FLIP_LR
                exp = []
                for i in range(h):
                    for j in range(w):
                        yy = h - 1 - i if flip_ud else i
                        xx = w - 1 - j if flip_lr else j
                        exp.append(im.clip1(pred[i * w + j] + res[yy][xx], bd))
                ok = exp == pix
                stats["itx"][("ok" if ok else "BAD", "tx%d/type%d" % (tx, txtp))] += 1
                if not ok and verbose:
                    print(f"ITX MISMATCH plane {plane} ({x4},{y4}) {w}x{h} tx {tx} txtp {txtp}")
                    print("  exp", exp[:16]); print("  got", pix[:16])
                preds[key] = pix   # the reconstructed block is what later CfL / neighbours see
            n += 1
            if max_n and n >= max_n:
                break
    for k in ("pred", "cfl", "itx"):
        c = stats[k]
        tot = sum(c.values())
        bad = sum(v for kk, v in c.items() if (kk[0] if isinstance(kk, tuple) else kk) == "BAD")
        print(f"{k}: {tot} checked, {bad} mismatches")
        for kk, v in sorted(c.items(), key=lambda kv: -kv[1]):
            print(f"    {kk}: {v}")
    total_bad = sum(v for c in stats.values() for kk, v in c.items() if (kk[0] if isinstance(kk, tuple) else kk) == "BAD")
    sys.exit(1 if total_bad else 0)


if __name__ == "__main__":
    main()
