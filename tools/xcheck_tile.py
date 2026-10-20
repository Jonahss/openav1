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


class TileEvents:
    """A tile's trace events, read lazily from the byte spans of its D sections (frames_of indexes them): the symbol
    events are pulled one at a time as the model reads symbols, the C events collect as they pass. Holding a frame's
    events as Python lists took tens of GB on a 10-million-line Argon frame."""
    def __init__(self, fh, spans):
        self.fh, self.spans = fh, spans
        self.coefs = []
        self.nsym = 0                           # symbol events handed out
        self._lines = self._iter_lines()
        self._peek = None

    def _iter_lines(self):
        for start, end in self.spans:
            self.fh.seek(start)
            for line in self.fh.read(end - start).decode().split("\n"):
                if line and line[0] in "SUC":
                    yield line

    def _next_line(self):
        if self._peek is not None:
            line, self._peek = self._peek, None
            return line
        return next(self._lines, None)

    def next_sym(self):
        """The next S event (A/B/E tuple as before), with a following U merged into a B; None when the tile has no more.
        C events met on the way are appended to self.coefs."""
        while True:
            line = self._next_line()
            if line is None:
                return None
            f = line.split()
            k = f[0]
            if k == "S":
                self.nsym += 1
                if f[1] == "A":
                    n = int(f[2])
                    return ("A", n, int(f[3]), int(f[4]), int(f[5]), [int(x) for x in f[6:6 + n]])
                if f[1] == "B":
                    ev = ["B", int(f[2]), int(f[3]), int(f[4]), None]
                    nxt = self._next_line()
                    if nxt is not None and nxt[0] == "U":
                        g = nxt.split()
                        ev[4] = (int(g[1]), int(g[2]), int(g[3]))
                    else:
                        self._peek = nxt
                    return ev
                return ("E", int(f[2]), int(f[3]))
            if k == "C":
                plane, x4, y4, tx, txtp, eob, w, h = [int(x) for x in f[1:9]]
                n = min(w, 32) * min(h, 32)
                self.coefs.append((plane, x4, y4, tx, txtp, eob, w, h, [int(x) for x in f[9:9 + n]]))
            # a stray U (not after a B) cannot happen; ignored

    def drain(self):
        """Read the rest of the tile: returns the number of symbol events the model did not consume."""
        extra = 0
        while self.next_sym() is not None:
            extra += 1
        return extra


def run_tile(hdr_fields, data, events, verbose, frame=None, check_coefs=True):
    hdr = tm.FrameHeader(hdr_fields)
    counts = {"S": 0}

    def hook(kind, N, symbol, rng, before, after, name):
        counts["S"] += 1
        ev = events.next_sym()
        if ev is None:
            raise Mismatch(f"model read {name} (N={N}) but dav1d logged no more symbols in this tile")
        idx = events.nsym
        ekind = ev[0]
        if ekind == "A":
            n, val, erng, cnt, icdf = ev[1], ev[2], ev[3], ev[4], ev[5]
            if n != N - 1 or val != symbol:
                raise Mismatch(f"{name}: model N={N} sym={symbol}; dav1d A n={n} val={val} (event #{idx})")
            model_icdf = [32768 - before[i] for i in range(N - 1)]
            if model_icdf != icdf or before[N] != cnt:
                raise Mismatch(f"{name}: CDF differs. model icdf={model_icdf} cnt={before[N]}; dav1d icdf={icdf} cnt={cnt} (event #{idx})")
        elif ekind == "B":
            f, bit, erng, upd = ev[1], ev[2], ev[3], ev[4]
            if N != 2 or bit != symbol:
                raise Mismatch(f"{name}: model N={N} sym={symbol}; dav1d B bit={bit} (event #{idx})")
            if 32768 - before[0] != f:
                raise Mismatch(f"{name}: bool prob differs. model f={32768 - before[0]} dav1d f={f} (event #{idx})")
            if upd is not None and before[2] != upd[0]:
                raise Mismatch(f"{name}: bool counter differs. model {before[2]} dav1d {upd[0]} (event #{idx})")
        else:  # E
            bit, erng = ev[1], ev[2]
            if N != 2 or before[0] != 16384 or bit != symbol:
                raise Mismatch(f"{name}: model N={N} cdf0={before[0]} sym={symbol}; dav1d E bit={bit} (event #{idx})")
        if rng != erng:
            raise Mismatch(f"{name}: range after symbol differs: model {rng} dav1d {erng} (event #{idx})")

    dec = tm.TileDecoder(hdr, data, hook, frame=frame)
    try:
        dec.decode_tile()
    except Mismatch as m:
        b = dec.blocks[-1] if dec.blocks else None
        cur = f"MiRow={getattr(dec, 'MiRow', '?')} MiCol={getattr(dec, 'MiCol', '?')} MiSize={getattr(dec, 'MiSize', '?')}"
        if str(m).startswith("segment_id") or str(m).startswith("seg_id_predicted"):
            # the segmentation context: the current frame's ids around the block, the previous map, the pred contexts,
            # and the three segment_id CDF rows the model holds (icdf form, as dav1d logs them)
            r, c = dec.MiRow, dec.MiCol
            def sid(rr, cc):
                try:
                    return dec.SegmentIds[rr][cc] if rr >= 0 and cc >= 0 else None
                except IndexError:
                    return None
            prev = dec.PrevSegmentIds
            pv = [[(prev[rr][cc] if prev is not None and 0 <= rr < len(prev) and 0 <= cc < len(prev[0]) else None)
                   for cc in range(c - 1, c + 3)] for rr in range(r - 1, r + 3)]
            rows = ["[" + ", ".join(str(32768 - v) for v in row[:-2]) + f"] cnt={row[-1]}" for row in dec.cdf["Segment_Id"]]
            cur += (f"; AvailU={dec.AvailU} AvailL={dec.AvailL} prevUL={sid(r - 1, c - 1)} prevU={sid(r - 1, c)} prevL={sid(r, c - 1)} "
                    f"skip={getattr(dec, 'skip', '?')}; PrevSegmentIds[r-1..r+2][c-1..c+2]={pv}; "
                    f"AboveSegPred[c]={dec.AboveSegPredContext[c]} LeftSegPred[r]={dec.LeftSegPredContext[r]}; "
                    f"Segment_Id rows: ctx0 {rows[0]} ctx1 {rows[1]} ctx2 {rows[2]}")
        raise Mismatch(f"{m} | at {cur}; last completed block {b}") from None
    extra = events.drain()
    if extra:
        raise Mismatch(f"model consumed {events.nsym - extra} symbols, dav1d logged {events.nsym}")
    # coefficient blocks
    model_c = [e for e in dec.events if e[0] == "C"]
    c_events = events.coefs
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

    fh = open(path, "rb")

    def frames_of(path):
        """Yield the trace's frames one at a time as lists of tiles (dict hdr/data/spans): an index pass over the
        file that records, per tile, the byte spans of its D sections; the events themselves are read by TileEvents
        when the tile is checked. (Events held as Python lists took 15 GB for a frame, then 30+ GB on the 10-million-
        line Argon frames; the index is a few numbers per superblock row.)"""
        pending = []        # tiles of the frame being set up (T/H come first for all tiles of a frame)
        active = {}         # (tile_row, tile_col) -> tile, for the frame being decoded (D routes events)
        order = []          # the active frame's tiles in T order
        cur = None          # the tile whose T/H is being read
        span_tile = None    # the tile whose D section is open, and where it started
        span_start = 0
        pos = 0
        with open(path, "rb") as idx:
            for line in idx:
                n = len(line)
                c0 = line[:1]
                if c0 == b"T":
                    if span_tile is not None:
                        span_tile["spans"].append((span_start, pos))
                        span_tile = None
                    if active and not pending:     # a new frame starts: the decoded one is complete
                        yield order
                        active, order = {}, []
                    f = line.split()
                    cur = dict(hdr=None, data=bytes.fromhex(f[3].decode()) if len(f) > 3 else b"", spans=[])
                    pending.append(cur)
                elif c0 == b"H":
                    cur["hdr"] = [int(x) for x in line.split()[1:]]
                elif c0 == b"D":                   # a tile's superblock row starts decoding
                    if span_tile is not None:
                        span_tile["spans"].append((span_start, pos))
                    if pending:
                        active = {(t["hdr"][0], t["hdr"][1]): t for t in pending}
                        order = pending
                        pending = []
                    f = line.split()
                    span_tile = active[(int(f[1]), int(f[2]))]
                    span_start = pos + n
                pos += n
        if span_tile is not None:
            span_tile["spans"].append((span_start, pos))
        if pending:                                # tiles announced but never decoded (should not happen)
            active = {(t["hdr"][0], t["hdr"][1]): t for t in pending}
            order = pending
        if order:
            yield order

    def events_of(t):
        return TileEvents(fh, t["spans"])

    def tile_has_coefs(t):
        """Whether the trace carries C events (a mask-1 trace has none): decided from the first tile's first span."""
        for start, end in t["spans"][:1]:
            fh.seek(start)
            return b"\nC " in b"\n" + fh.read(end - start)
        return False

    ok = 0
    tot_s = tot_c = tot_b = 0
    n_tiles = 0
    has_coefs = None                                        # decided from the first frame (a mask-1 trace has no C events)
    stopped = False
    if stream is None:
        for ftiles in frames_of(path):
            if has_coefs is None:
                has_coefs = any(tile_has_coefs(t) for t in ftiles)
            for t in ftiles:
                n_tiles += 1
                if max_tiles is not None and n_tiles > max_tiles:
                    stopped = True
                    break
                try:
                    ns, nc, nb, _ = run_tile(t["hdr"], t["data"], events_of(t), verbose, check_coefs=has_coefs)
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
                has_coefs = any(tile_has_coefs(t) for t in ftiles)
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
                    ns, nc, nb, dec = run_tile(t["hdr"], t["data"], events_of(t), verbose, frame=ctx,
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
