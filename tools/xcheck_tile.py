#!/usr/bin/env python3
"""Run tb/tile_model.py on every tile of a dav1d v3 trace and compare, symbol by symbol, with dav1d.

For each tile (H + T events) the model decodes the tile bytes from scratch. Every symbol the model
reads is matched against the next S/U event dav1d logged: symbol kind, alphabet size, decoded value,
the arithmetic decoder's range afterwards, and (for adaptive symbols) the CDF row before decoding.
Dequantised coefficient blocks (C events) are compared as well. The first mismatch stops the tile
and names the syntax element, so context-derivation mistakes are located exactly.

Usage: tools/xcheck_tile.py <trace.txt> [stream.ivf|.obu] [--max-tiles N] [-v]
With the stream, the frames are decoded in order with the sequence state (tb/seq_model.py): CDFs inherited from
reference slots, segmentation maps, motion fields for temporal MV candidates. Inter streams need it; the header
parser (tb/obu_parser.py) supplies the per-frame values the trace header does not carry (context_update_tile_id,
refresh flags, show_existing_frame).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tb"))
import tile_model as tm  # noqa: E402
import seq_model as sm   # noqa: E402
import obu_parser as op  # noqa: E402


class Mismatch(Exception):
    pass


def run_tile(hdr_fields, data, sym_events, c_events, verbose, frame=None, check_coefs=True):
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

    dec = tm.TileDecoder(hdr, data, hook, frame=frame)
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
    if not check_coefs:
        return counts["S"], len(model_c), len(dec.blocks), dec
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
            # the block holding this transform block (luma 4x4 units), for the syntax that feeds the dequantisation
            sx = hdr.subsampling_x if plane else 0
            sy = hdr.subsampling_y if plane else 0
            blk = [b for b in dec.blocks if b["r"] <= (y4 << sy) < b["r"] + tm.T.Num_4x4_Blocks_High[b["size"]]
                   and b["c"] <= (x4 << sx) < b["c"] + tm.T.Num_4x4_Blocks_Wide[b["size"]]]
            raise Mismatch(f"C #{k} plane {plane} ({x4},{y4}) tx {tx} type {txtp}: dequant differs at {bad[:6]} (model eob {eob}, "
                           f"dav1d eob {deob}); block {blk[-1] if blk else None}; qindex {dec.get_qindex(0, blk[-1]['seg']) if blk else '?'} "
                           f"base_q_idx {hdr.base_q_idx} delta_q_present {hdr.delta_q_present} lossless {hdr.LosslessArray}")
    return counts["S"], len(model_c), len(dec.blocks), dec


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    if "--max-tiles" in sys.argv:
        args = [a for a in args if a != sys.argv[sys.argv.index("--max-tiles") + 1]]
    verbose = "-v" in sys.argv
    max_tiles = int(sys.argv[sys.argv.index("--max-tiles") + 1]) if "--max-tiles" in sys.argv else None
    path = args[0]
    stream = args[1] if len(args) > 1 else None

    def frames_of(path):
        """Yield the trace's frames one at a time as lists of tiles (dict hdr/data/syms/coefs), so memory holds one
        frame's events rather than the whole stream (a large Argon stream parsed whole took 15 GB)."""
        pending = []        # tiles of the frame being set up (T/H come first for all tiles of a frame)
        active = {}         # (tile_row, tile_col) -> tile, for the frame being decoded (D routes events)
        order = []          # the active frame's tiles in T order
        cur = None
        with open(path) as fh:
            for line in fh:
                if not line or line[0] == "#":
                    continue
                f = line.split()
                k = f[0]
                if k == "T":                       # msac init: T then H, for every tile, before decoding starts
                    if active and not pending:     # a new frame starts: the decoded one is complete
                        yield order
                        active, order = {}, []
                    cur = dict(hdr=None, data=bytes.fromhex(f[3]) if len(f) > 3 else b"", syms=[], coefs=[])
                    pending.append(cur)
                elif k == "H":
                    cur["hdr"] = [int(x) for x in f[1:]]
                elif k == "D":                     # a tile's superblock row starts decoding
                    if pending:
                        active = {(t["hdr"][0], t["hdr"][1]): t for t in pending}
                        order = pending
                        pending = []
                    cur = active[(int(f[1]), int(f[2]))]
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
        if pending:                                # tiles announced but never decoded (should not happen)
            active = {(t["hdr"][0], t["hdr"][1]): t for t in pending}
            order = pending
        if order:
            yield order
    ok = 0
    tot_s = tot_c = tot_b = 0
    n_tiles = 0
    has_coefs = None                                        # decided from the first frame (a mask-1 trace has no C events)
    stopped = False
    if stream is None:
        for ftiles in frames_of(path):
            if has_coefs is None:
                has_coefs = any(t["coefs"] for t in ftiles)
            for t in ftiles:
                n_tiles += 1
                if max_tiles is not None and n_tiles > max_tiles:
                    stopped = True
                    break
                try:
                    ns, nc, nb, _ = run_tile(t["hdr"], t["data"], t["syms"], t["coefs"], verbose, check_coefs=has_coefs)
                    ok += 1
                    tot_s += ns; tot_c += nc; tot_b += nb
                except Mismatch as m:
                    print(f"tile {n_tiles - 1}: MISMATCH: {m}")
                    stopped = True
                    break
            if stopped:
                break
    else:
        d = op.parse_file(stream)
        seq = sm.SequenceState()
        tot_inter = [0, 0]                                      # inter blocks, of which compound
        hdr_iter = iter(d.frame_headers)
        fi = 0                                                  # decoded frames consumed
        for ftiles in frames_of(path):
            if has_coefs is None:
                has_coefs = any(t["coefs"] for t in ftiles)
            # the parser's header of this decoded frame, applying show_existing_frame events on the way
            h = None
            for h in hdr_iter:
                if h.show_existing_frame:
                    seq.show_existing(h)
                    continue
                break
            if h is None or h.show_existing_frame:
                print(f"frame {fi}: the trace has more frames than the parsed stream")
                stopped = True
                break
            if len(d.frames[fi]) != len(ftiles):
                print(f"frame {fi}: parser {len(d.frames[fi])} tiles vs trace {len(ftiles)}")
            ctx = seq.start_frame(h)
            decoders = []
            for t in ftiles:
                n_tiles += 1
                if max_tiles is not None and n_tiles > max_tiles:
                    stopped = True
                    break
                try:
                    ns, nc, nb, dec = run_tile(t["hdr"], t["data"], t["syms"], t["coefs"], verbose, frame=ctx,
                                               check_coefs=has_coefs)
                    ok += 1
                    tot_s += ns; tot_c += nc; tot_b += nb
                    tot_inter[0] += sum(1 for b in dec.blocks if b.get("inter") and not b.get("intrabc"))
                    tot_inter[1] += sum(1 for b in dec.blocks if b.get("inter") and len(b.get("mvs", ())) == 2)
                    # keep only what the sequence state needs; the decoder's full-frame arrays go (memory)
                    decoders.append(sm.TileSummary(dec, keep_cdf=(len(decoders) == h.context_update_tile_id)))
                    del dec
                except Mismatch as m:
                    kind = "intra" if h.FrameIsIntra else "inter"
                    print(f"tile {n_tiles - 1} (frame {fi}, {kind}, order_hint {h.order_hint}): MISMATCH: {m}")
                    stopped = True
                    break
            if stopped:
                break
            seq.end_frame(h, decoders, frame_cdfs=ctx.cdfs)
            fi += 1
        if not stopped and fi != len(d.frames):
            print(f"parser has {len(d.frames)} decoded frames, trace {fi}")
            stopped = True
    inter = f", {tot_inter[0]} inter blocks ({tot_inter[1]} compound)" if stream is not None else ""
    print(f"OK: {ok}/{n_tiles} tiles, {tot_s} symbols, {tot_c} coefficient blocks, {tot_b} blocks{inter}" if not stopped and ok == n_tiles
          else f"{ok}/{n_tiles} tiles OK ({tot_s} symbols, {tot_c} coefficient blocks, {tot_b} blocks{inter} before the mismatch)")
    sys.exit(0 if (not stopped and ok == n_tiles) else 1)

if __name__ == "__main__":
    main()
