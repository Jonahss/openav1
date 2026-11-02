#!/usr/bin/env python3
"""Reference-fetch census per frame (syntax model only, no reconstruction): what ref_fetch as built asks of DDR
(docs/reference-prefetch.md, 2026-11-02).
Per inter block (not intrabc): lists = 1 + compound; luma patch rows = h + 7, chroma (4:2:0) 2 planes x (h/2 + 7)
rows (blocks < 8 wide/high in chroma count as 8); every patch row is one AXI read (one AR, ceil((w + 7 + 7) / 8)
8-byte beats for luma, chroma half). OBMC blocks add the neighbour strips (approximated as one extra luma + chroma
fetch of half height). For docs/picture-memory.md section 4 it also counts the DDR traffic three ways: linear rows
in 64-byte words (what the bursts touch), the distinct 64-byte tiles (16 x 4 samples at 8-bit, 8 x 4 at 16-bit)
a tiled layout would read = the compulsory traffic of a perfect cache, and the tiles a direct-mapped cache of
TILE_CACHE entries per list would miss (sweep order = decode order). Output: one line per frame.
Usage: fetch_census.py stream.ivf first last [cache entries]"""
import sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tb"))
import tile_model as tm, seq_model as sm, obu_parser as op
path, lo, hi = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
cache_n = int(sys.argv[4]) if len(sys.argv) > 4 else 128
d = op.parse_file(path)
seq = sm.SequenceState()
hdr_iter = iter(d.frame_headers)
BW = [4 * n for n in tm.T.Num_4x4_Blocks_Wide]
BH = [4 * n for n in tm.T.Num_4x4_Blocks_High]
print("size tables: T.Num_4x4_Blocks_Wide", flush=True)
for fi, tl in enumerate(d.frames):
    h = None
    for h in hdr_iter:
        if h.show_existing_frame:
            seq.show_existing(h); continue
        break
    if h is None or fi > hi:
        break
    ctx = seq.start_frame(h)
    t0 = time.monotonic()
    n = rows = beats = lists = obmc = comp = 0
    hist = {}
    bps = 1 if h.BitDepth == 8 else 2
    tw = 64 // (4 * bps)                                  # tile width in samples (64-byte tile, 4 rows)
    cnt = dict(words=0, misses=0); tiles = set(); cache = {}
    def touch(plane, ref, x0, y0, w, hh, W, H):
        """the linear 64-byte words and the 64-byte tiles a (w + 7) x (hh + 7) patch at (x0 - 3, y0 - 3) reads, clamped to the plane"""
        ya, yb = max(0, min(y0 - 3, H - 1)), max(0, min(y0 + hh + 3, H - 1))
        xa, xb = max(0, min(x0 - 3, W - 1)), max(0, min(x0 + w + 4, W - 1))
        cnt["words"] += (yb - ya + 1) * ((xb * bps) // 64 - (xa * bps) // 64 + 1)
        for ty in range(ya // 4, yb // 4 + 1):
            for tx in range(xa // tw, xb // tw + 1):
                key = (ref, plane, ty, tx); tiles.add(key)
                slot = hash(key) % cache_n
                if cache.get(slot) != key:
                    cnt["misses"] += 1; cache[slot] = key
    summaries = []
    for k, ti in enumerate(tl):
        th, data = d.tiles[ti]
        dec = tm.TileDecoder(th, data, None, frame=ctx)
        dec.decode_tile()
        summaries.append(sm.TileSummary(dec, keep_cdf=(k == h.context_update_tile_id)))
        if fi < lo:
            continue
        for b in dec.blocks:
            if not b.get("inter") or b.get("intrabc"):
                continue
            w, hh = BW[b["size"]], BH[b["size"]]
            L = 1 + (1 if b["ref"][1] > 0 else 0)
            n += 1; lists += L; comp += (L == 2)
            key = f"{w}x{hh}"; hist[key] = hist.get(key, 0) + 1
            cw, ch = max(w // 2, 8), max(hh // 2, 8)
            r_blk = (hh + 7) + 2 * (ch + 7)
            b_blk = (hh + 7) * ((w + 14 + 7) // 8) + 2 * (ch + 7) * ((cw + 14 + 7) // 8)
            if b.get("motion_mode", 0) == 1:
                obmc += 1; r_blk += r_blk // 2; b_blk += b_blk // 2
            rows += L * r_blk; beats += L * b_blk
            for li in range(L):
                mvy, mvx = b["mvs"][li][0] >> 3, b["mvs"][li][1] >> 3          # 1/8-pel to integer samples
                px, py = b["c"] * 4, b["r"] * 4
                touch(0, b["ref"][li], px + mvx, py + mvy, w, hh, h.UpscaledWidth, h.FrameHeight)
                for pl in (1, 2):
                    touch(pl, b["ref"][li], (px >> 1) + (mvx >> 1), (py >> 1) + (mvy >> 1), cw, ch, (h.UpscaledWidth + 1) >> 1, (h.FrameHeight + 1) >> 1)
    seq.end_frame(h, summaries, frame_cdfs=ctx.cdfs)
    if fi >= lo:
        top = sorted(hist.items(), key=lambda kv: -kv[1])[:6]
        print(f"frame {fi} {'intra' if h.FrameIsIntra else 'inter'} {h.UpscaledWidth}x{h.FrameHeight}: blocks {n} lists {lists} compound {comp} obmc {obmc} "
              f"rows {rows} beats {beats} ddr64B linear {cnt['words']} tiled {len(tiles)} cache{cache_n}-miss {cnt['misses']} sizes {top} ({time.monotonic() - t0:.0f} s)", flush=True)
