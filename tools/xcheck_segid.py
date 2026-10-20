#!/usr/bin/env python3
"""Compare the model's per-block segment ids with dav1d's, from a trace with the MVS bit ('G by4 bx4 bs seg_id skip
seg_pred' after every block's mode info). Tiles are matched in stream order (one H + T pair per tile), blocks by
position. Reports the first tile whose ids differ, with the model's view of that block (skip, predicted, neighbours,
previous map). Usage: tools/xcheck_segid.py <trace.txt> <stream.obu|.ivf> [-v]"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tb"))
import tile_model as tm   # noqa: E402
import seq_model as sm    # noqa: E402
import obu_parser as op   # noqa: E402


DAV1D_DIMS = [(128, 128), (128, 64), (64, 128), (64, 64), (64, 32), (64, 16), (32, 64), (32, 32), (32, 16), (32, 8), (16, 64),
              (16, 32), (16, 16), (16, 8), (16, 4), (8, 32), (8, 16), (8, 8), (8, 4), (4, 16), (4, 8), (4, 4)]


def parse_trace(path):
    """[(events of frame k)] in decode order: dav1d writes every T line of a frame before decoding it, so the events
    follow the frame's last T line; a maximal run of T lines starts a new frame group."""
    frames, cur, in_t = [], None, False
    with open(path) as fh:
        for line in fh:
            if line.startswith("T ") or line.startswith("H "):
                if not in_t:                      # a frame's T / H lines come together, its events after them
                    cur = []
                    frames.append(cur)
                in_t = True
            elif line.startswith("G ") and cur is not None:
                in_t = False
                f = [int(v) for v in line.split()[1:]]
                cur.append(tuple(f))
            elif line[:2] in ("S ", "A ", "B ", "M ", "G ", "C ", "P ", "Q ", "R "):
                in_t = False
    return frames


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    verbose = "-v" in sys.argv
    dframes = parse_trace(args[0])
    d = op.parse_file(args[1])
    seq = sm.SequenceState()
    hdr_iter = iter(d.frame_headers)
    k = 0
    nblocks = 0
    missing = 0
    for fi, tl in enumerate(d.frames):
        h = None
        for h in hdr_iter:
            if h.show_existing_frame:
                seq.show_existing(h)
                continue
            break
        if h is None:
            break
        ctx = seq.start_frame(h)
        decs = []
        dav = {(e[0], e[1]): e for e in dframes[k]} if k < len(dframes) else {}
        for ti_k, ti in enumerate(tl):
            th, data = d.tiles[ti]
            dec = tm.TileDecoder(th, data, None, frame=ctx)
            dec.decode_tile()
            decs.append(sm.TileSummary(dec, keep_cdf=(ti_k == h.context_update_tile_id)))
            for b in dec.blocks:
                nblocks += 1
                e = dav.pop((b["r"], b["c"]), None)
                if e is None:
                    missing += 1
                    if verbose:
                        print(f"frame {fi} tile {ti_k}: model block ({b['r']},{b['c']}) size {b['size']} has no dav1d block")
                    continue
                # dav1d's BlockSize enum runs 128x128 .. 4x4; the spec's sizes are indexed differently: compare dimensions
                dw, dh = DAV1D_DIMS[e[2]]
                if (dw, dh) != (tm.T.Block_Width[b["size"]], tm.T.Block_Height[b["size"]]) or e[3] != b["seg"] or e[4] != b["skip"]:
                    r, c = b["r"], b["c"]
                    sid = lambda rr, cc: dec.SegmentIds[rr][cc] if rr >= 0 and cc >= 0 else None  # noqa: E731
                    prev = dec.PrevSegmentIds
                    pv = [[(prev[rr][cc] if prev is not None and 0 <= rr < len(prev) and 0 <= cc < len(prev[0]) else None)
                           for cc in range(c, c + 4)] for rr in range(r, r + 4)]
                    print(f"frame {fi} tile {ti_k} (order_hint {h.order_hint}, seg upd {h.segmentation_update_map} tmp "
                          f"{h.segmentation_temporal_update} preskip {h.SegIdPreSkip} last {h.LastActiveSegId}) block ({r},{c}) "
                          f"size {b['size']} (dav1d bs {e[2]}) skip {b['skip']}: model seg {b['seg']} | dav1d seg {e[3]} skip {e[4]} seg_pred {e[5]}; "
                          f"model neighbours UL {sid(r - 1, c - 1)} U {sid(r - 1, c)} L {sid(r, c - 1)}; prev map [r..r+3][c..c+3] {pv}")
                    if not verbose:
                        print(f"{k + 1} frames, {nblocks} blocks compared before the first difference")
                        return 1
        if dav:
            missing += len(dav)
            print(f"frame {fi}: {len(dav)} dav1d blocks without a model block, e.g. {list(dav.values())[:3]}")
        k += 1
        seq.end_frame(h, decs, frame_cdfs=ctx.cdfs)
    print(f"{k} frames, {nblocks} blocks compared, no differences; {missing} blocks unmatched on either side")
    return 0


if __name__ == "__main__":
    sys.exit(main())
