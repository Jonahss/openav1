#!/usr/bin/env python3
"""Whole-frame check: run tb/recon_model.py on every tile of a v3 trace and compare each predicted
block (P / Q events) and each reconstructed block (R events) with dav1d, pixel for pixel.

Usage: tools/xcheck_recon.py <trace.txt> [-v]
"""
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tb"))
import tile_model as tm     # noqa: E402
import recon_model as rm    # noqa: E402


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    verbose = "-v" in sys.argv
    path = args[0]
    tiles = []
    cur = None
    pending = []
    active = {}
    with open(path) as fh:
        for line in fh:
            if not line or line[0] in "#SUC":
                continue
            f = line.split()
            k = f[0]
            if k == "T":
                cur = dict(hdr=None, data=bytes.fromhex(f[3]) if len(f) > 3 else b"", P={}, Q={}, R={})
                tiles.append(cur)
                pending.append(cur)
            elif k == "H":
                cur["hdr"] = [int(x) for x in f[1:]]
            elif k == "D":
                if pending:
                    active = {(t["hdr"][0], t["hdr"][1]): t for t in pending}
                    pending = []
                cur = active[(int(f[1]), int(f[2]))]
            elif k == "P":
                v = [int(x) for x in f[1:]]
                plane, x4, y4, w, h = v[:5]
                n_edge = 2 * h + 2 * w + 1
                pix = v[11 + n_edge:11 + n_edge + w * h]
                cur["P"][(plane, x4, y4)] = (w, h, pix)
            elif k == "Q":
                v = [int(x) for x in f[1:]]
                plane, x4, y4, w, h = v[:5]
                n_edge = 2 * h + 2 * w + 1
                pix = v[7 + n_edge + w * h:7 + n_edge + 2 * w * h]
                cur["Q"][(plane, x4, y4)] = (w, h, pix)
            elif k == "R":
                v = [int(x) for x in f[1:]]
                plane, x4, y4, w, h = v[:5]
                cur["R"][(plane, x4, y4)] = (w, h, v[7:7 + w * h])
    stats = Counter()
    frame = None
    bad = 0
    for ti, t in enumerate(tiles):
        hdr = tm.FrameHeader(t["hdr"])
        if frame is None or (hdr.tile_row == 0 and hdr.tile_col == 0):
            frame = {}
        dec = rm.FrameRecon(hdr, t["data"], frame)
        try:
            dec.decode_tile()
        except Exception as e:
            print(f"tile {ti}: MODEL ERROR {type(e).__name__}: {e}")
            bad += 1
            continue
        # compare
        for kind, plane, x4, y4, w, h, pix in dec.pred_events:
            ref = t[kind].get((plane, x4, y4))
            if ref is None:
                stats["pred missing in trace"] += 1
                continue
            if ref[2] != pix:
                bad += 1
                stats[f"{kind} BAD"] += 1
                if verbose or stats[f"{kind} BAD"] <= 3:
                    d = [(i // w, i % w, pix[i], ref[2][i]) for i in range(w * h) if pix[i] != ref[2][i]]
                    print(f"tile {ti} {kind} plane {plane} ({x4},{y4}) {w}x{h}: {len(d)} bad px, first {d[:5]}")
            else:
                stats[f"{kind} ok"] += 1
        for kind, plane, x4, y4, w, h, pix in dec.recon_events:
            ref = t["R"].get((plane, x4, y4))
            if ref is None:
                stats["recon missing in trace"] += 1
                continue
            if ref[2] != pix:
                bad += 1
                stats["R BAD"] += 1
                if verbose or stats["R BAD"] <= 3:
                    d = [(i // w, i % w, pix[i], ref[2][i]) for i in range(w * h) if pix[i] != ref[2][i]]
                    print(f"tile {ti} R plane {plane} ({x4},{y4}) {w}x{h}: {len(d)} bad px, first {d[:5]}")
            else:
                stats["R ok"] += 1
        nP = len(t["P"]) + len(t["Q"]); nR = len(t["R"])
        if len(dec.pred_events) != nP or len(dec.recon_events) != nR:
            print(f"tile {ti}: event counts model P/Q {len(dec.pred_events)} R {len(dec.recon_events)} vs dav1d P/Q {nP} R {nR}")
    print(dict(stats))
    print("OK" if bad == 0 else f"{bad} mismatching blocks")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
