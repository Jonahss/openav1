#!/usr/bin/env python3
"""Compare the model's MV stack (tb/tile_model.py find_mv_stack, 7.10.2) with dav1d's refmvs_find, block by block, from a
dav1d v3 trace recorded with the MVS bit (mask 16 + 1: 'M by4 bx4 bs ref0 ref1 cnt ctx [y0 x0 y1 x1 weight]*').
Tiles are matched in stream order (one H + T pair per tile in the trace, the M events of a tile follow its T line);
within a tile the blocks are matched by position and references. Reports the first tile whose stacks differ.

Usage: tools/xcheck_mvstack.py <trace.txt> <stream.obu|.ivf> [--max-tiles N] [-v]"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tb"))
import tile_model as tm   # noqa: E402
import seq_model as sm    # noqa: E402
import obu_parser as op   # noqa: E402

# dav1d's BlockSize enum runs from BS_128x128 (0) down to BS_4x4 (21): map to the spec's MiSize for the report
DAV1D_BS = ["128x128", "128x64", "64x128", "64x64", "64x32", "64x16", "32x64", "32x32", "32x16", "32x8", "16x64", "16x32",
            "16x16", "16x8", "16x4", "8x32", "8x16", "8x8", "8x4", "4x16", "4x8", "4x4"]


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
            elif line.startswith("M ") and cur is not None:
                in_t = False
                f = [int(v) for v in line.split()[1:]]
                ents = [tuple(f[7 + 5 * i: 12 + 5 * i]) for i in range(f[5])]
                cur.append((f[0], f[1], f[2], f[3], f[4], f[5], f[6], ents))
            elif line[:2] in ("S ", "A ", "B ", "M ", "G ", "C ", "P ", "Q ", "R "):
                in_t = False
    return frames


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    verbose = "-v" in sys.argv
    max_tiles = None
    if "--max-tiles" in sys.argv:
        max_tiles = int(sys.argv[sys.argv.index("--max-tiles") + 1])
    trace, stream = args[0], args[1]
    dframes = parse_trace(trace)
    d = op.parse_file(stream)
    seq = sm.SequenceState()
    hdr_iter = iter(d.frame_headers)
    stacks = []       # the model's stacks of the tile being decoded

    orig = tm.TileDecoder.find_mv_stack

    def hooked(self, isCompound):
        orig(self, isCompound)
        n = self.NumMvFound
        ents = []
        for i in range(n):
            e = self.RefStackMv[i]
            ents.append((e[0][0], e[0][1], e[1][0] if isCompound else 0, e[1][1] if isCompound else 0, self.WeightStack[i]))
        stacks.append((self.MiRow, self.MiCol, self.MiSize, self.RefFrame[0], self.RefFrame[1] if isCompound else -1, n, ents))
    tm.TileDecoder.find_mv_stack = hooked

    k = 0
    bad = 0
    nblocks = 0
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
        dav = {(e[0], e[1], e[3], e[4]): e for e in dframes[k]} if k < len(dframes) else {}
        for ti_k, ti in enumerate(tl):
            th, data = d.tiles[ti]
            stacks.clear()
            dec = tm.TileDecoder(th, data, None, frame=ctx)
            dec.decode_tile()
            decs.append(sm.TileSummary(dec, keep_cdf=(ti_k == h.context_update_tile_id)))
            # match by (row, col, ref0, ref1); dav1d's cnt counts the found candidates like NumMvFound
            for (r, c, sz, r0, r1, n, ents) in stacks:
                nblocks += 1
                e = dav.pop((r, c, r0, r1), None)
                if e is None:
                    continue      # intrabc / a different call pattern: not compared
                w = 4 if r1 > 0 else 2                       # single reference: dav1d leaves list 1 uninitialised
                d_ents = [tuple(x[:w]) for x in e[7]]
                m_ents = [tuple(x[:w]) for x in ents]
                if e[5] != n or d_ents != m_ents:
                    bad += 1
                    print(f"frame {fi} tile {ti_k} (order_hint {h.order_hint}) block ({r},{c}) {tm.T.Block_Width[sz]}x{tm.T.Block_Height[sz]} "
                          f"refs ({r0},{r1}): model NumMvFound {n} {m_ents} weights {[x[4] for x in ents]} | "
                          f"dav1d cnt {e[5]} {d_ents} weights {[x[4] for x in e[7]]} ctx {e[6]}")
                    if not verbose:
                        print(f"{k + 1} frames, {nblocks} inter blocks compared before the first difference")
                        return 1
            if verbose:
                print(f"frame {fi} tile {ti_k}: {len(stacks)} model stacks")
        k += 1
        if max_tiles and k >= max_tiles:
            seq.end_frame(h, decs, frame_cdfs=ctx.cdfs)
            print(f"stopped after {k} frames, {nblocks} inter blocks, {bad} differences")
            return 1 if bad else 0
        seq.end_frame(h, decs, frame_cdfs=ctx.cdfs)
    print(f"{k} frames, {nblocks} inter blocks compared, {bad} differences")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
