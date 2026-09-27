#!/usr/bin/env python3
"""Run tb/tile_model.py on every tile of a dav1d v3 trace and compare, symbol by symbol, with dav1d.

For each tile (H + T events) the model decodes the tile bytes from scratch. Every symbol the model
reads is matched against the next S/U event dav1d logged: symbol kind, alphabet size, decoded value,
the arithmetic decoder's range afterwards, and (for adaptive symbols) the CDF row before decoding.
Dequantised coefficient blocks (C events) are compared as well. The first mismatch stops the tile
and names the syntax element, so context-derivation mistakes are located exactly.

Usage: tools/xcheck_tile.py <trace.txt> [--max-tiles N] [-v]
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tb"))
import tile_model as tm  # noqa: E402


class Mismatch(Exception):
    pass


def run_tile(hdr_fields, data, sym_events, c_events, verbose):
    hdr = tm.FrameHeader(hdr_fields)
    idx = [0]
    counts = {"S": 0}

    def hook(kind, N, symbol, rng, before, after, name):
        counts["S"] += 1
        if idx[0] >= len(sym_events):
            raise Mismatch(f"model read {name} (N={N}) but dav1d logged no more symbols in this tile")
        ev = sym_events[idx[0]]
        idx[0] += 1
        ekind = ev[0]
        if ekind == "A":
            n, val, erng, cnt, icdf = ev[1], ev[2], ev[3], ev[4], ev[5]
            if n != N - 1 or val != symbol:
                raise Mismatch(f"{name}: model N={N} sym={symbol}; dav1d A n={n} val={val} (event #{idx[0]})")
            model_icdf = [32768 - before[i] for i in range(N - 1)]
            if model_icdf != icdf or before[N] != cnt:
                raise Mismatch(f"{name}: CDF differs. model icdf={model_icdf} cnt={before[N]}; dav1d icdf={icdf} cnt={cnt} (event #{idx[0]})")
        elif ekind == "B":
            f, bit, erng, upd = ev[1], ev[2], ev[3], ev[4]
            if N != 2 or bit != symbol:
                raise Mismatch(f"{name}: model N={N} sym={symbol}; dav1d B bit={bit} (event #{idx[0]})")
            if 32768 - before[0] != f:
                raise Mismatch(f"{name}: bool prob differs. model f={32768 - before[0]} dav1d f={f} (event #{idx[0]})")
            if upd is not None and before[2] != upd[0]:
                raise Mismatch(f"{name}: bool counter differs. model {before[2]} dav1d {upd[0]} (event #{idx[0]})")
        else:  # E
            bit, erng = ev[1], ev[2]
            if N != 2 or before[0] != 16384 or bit != symbol:
                raise Mismatch(f"{name}: model N={N} cdf0={before[0]} sym={symbol}; dav1d E bit={bit} (event #{idx[0]})")
        if rng != erng:
            raise Mismatch(f"{name}: range after symbol differs: model {rng} dav1d {erng} (event #{idx[0]})")

    dec = tm.TileDecoder(hdr, data, hook)
    try:
        dec.decode_tile()
    except Mismatch as m:
        b = dec.blocks[-1] if dec.blocks else None
        cur = f"MiRow={getattr(dec, 'MiRow', '?')} MiCol={getattr(dec, 'MiCol', '?')} MiSize={getattr(dec, 'MiSize', '?')}"
        raise Mismatch(f"{m} | at {cur}; last completed block {b}") from None
    if idx[0] != len(sym_events):
        raise Mismatch(f"model consumed {idx[0]} symbols, dav1d logged {len(sym_events)}")
    # coefficient blocks
    model_c = [e for e in dec.events if e[0] == "C"]
    if len(model_c) != len(c_events):
        raise Mismatch(f"model produced {len(model_c)} coefficient blocks, dav1d {len(c_events)}")
    for k, (m, d) in enumerate(zip(model_c, c_events)):
        _, plane, x4, y4, tx, txtp, eob, rows = m
        dplane, dx4, dy4, dtx, dtxtp, deob, dw, dh, dvals = d
        if (plane, x4, y4, tx, txtp) != (dplane, dx4, dy4, dtx, dtxtp):
            raise Mismatch(f"C #{k}: model (plane {plane} x4 {x4} y4 {y4} tx {tx} type {txtp}) vs dav1d (plane {dplane} x4 {dx4} y4 {dy4} tx {dtx} type {dtxtp})")
        flat = [v for r in rows for v in r]
        if flat != dvals:
            bad = [(i, flat[i], dvals[i]) for i in range(min(len(flat), len(dvals))) if flat[i] != dvals[i]]
            raise Mismatch(f"C #{k} plane {plane} ({x4},{y4}) tx {tx}: dequant differs at {bad[:6]} (model eob {eob}, dav1d eob {deob})")
    return counts["S"], len(model_c), len(dec.blocks)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    verbose = "-v" in sys.argv
    max_tiles = int(sys.argv[sys.argv.index("--max-tiles") + 1]) if "--max-tiles" in sys.argv else None
    path = args[0]
    tiles = []
    cur = None
    with open(path) as fh:
        for line in fh:
            if not line or line[0] == "#":
                continue
            f = line.split()
            k = f[0]
            if k == "T":                       # a tile starts with T (msac init), then its H (header)
                cur = dict(hdr=None, data=bytes.fromhex(f[3]) if len(f) > 3 else b"", syms=[], coefs=[])
                tiles.append(cur)
            elif k == "H":
                cur["hdr"] = [int(x) for x in f[1:]]
            elif k == "S":
                if f[1] == "A":
                    n = int(f[2])
                    cur["syms"].append(("A", n, int(f[3]), int(f[4]), int(f[5]), [int(x) for x in f[6:6 + n]]))
                elif f[1] == "B":
                    cur["syms"].append(["B", int(f[2]), int(f[3]), int(f[4]), None])
                else:
                    cur["syms"].append(("E", int(f[2]), int(f[3])))
            elif k == "U":
                last = cur["syms"][-1]
                assert last[0] == "B"
                last[4] = (int(f[1]), int(f[2]), int(f[3]))
            elif k == "C":
                plane, x4, y4, tx, txtp, eob, w, h = [int(x) for x in f[1:9]]
                n = min(w, 32) * min(h, 32)
                cur["coefs"].append((plane, x4, y4, tx, txtp, eob, w, h, [int(x) for x in f[9:9 + n]]))
    ok = 0
    tot_s = tot_c = tot_b = 0
    for ti, t in enumerate(tiles[:max_tiles] if max_tiles else tiles):
        try:
            ns, nc, nb = run_tile(t["hdr"], t["data"], t["syms"], t["coefs"], verbose)
            ok += 1
            tot_s += ns; tot_c += nc; tot_b += nb
            if verbose:
                print(f"tile {ti}: OK {ns} symbols, {nb} blocks, {nc} coefficient blocks")
        except Mismatch as m:
            print(f"tile {ti}: MISMATCH: {m}")
        except Exception as e:  # model crash
            import traceback
            print(f"tile {ti}: MODEL ERROR: {type(e).__name__}: {e}")
            if verbose:
                traceback.print_exc()
    print(f"{ok}/{len(tiles)} tiles bit-exact; {tot_s} symbols, {tot_b} blocks, {tot_c} coefficient blocks")
    sys.exit(0 if ok == len(tiles) else 1)


if __name__ == "__main__":
    main()
