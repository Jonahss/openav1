"""The streaming filter chain as a model (docs/filter-chain.md): the three stage models chained band by band, and a
check of that chain against the frame-at-once models' pictures. tools/check_filter_chain.py runs it on intra frames;
the decoder test benches call check_frame() on every frame they filter when TS_CHAIN_CHECK=1, which covers inter
frames and the Argon streams. Super-resolution frames are checked up to CDEF only (the upscale and the restoration
behind it are not in the chain model yet)."""
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lf_stage_model as lst      # noqa: E402
import cdef_stage_model as cst    # noqa: E402
import lr_stage_model as rst      # noqa: E402

ENABLED = os.environ.get("TS_CHAIN_CHECK", "0") == "1"
stats = {"frames": 0, "cdef": 0, "lr": 0, "sr_partial": 0}


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


def check_frame(hdr, state, pre, deblocked, cdef, final, tag="", sb_lines=None):
    """pre: the pre-filter planes; deblocked / cdef / final: the frame-at-once models' pictures before any upscale
    (cdef None when CDEF is off, final None when restoration is off or the frame uses super-res). Raises
    AssertionError naming the first differing stage."""
    n = hdr.NumPlanes
    for sb in ([sb_lines] if sb_lines else [128 if getattr(hdr, "use_128x128_superblock", 0) else 64]):
        dbl, out, flr, _ = chain(hdr, state, pre, sb)
        for p in range(n):
            assert dbl[p] == deblocked[p], f"{tag}: chain deblock differs from lf_model, plane {p}"
            if cdef is not None:
                assert out[p] == cdef[p], f"{tag}: chain CDEF differs from cdef_model, plane {p}"
            if final is not None and flr is not None:
                assert flr[p] == final[p], f"{tag}: chain restoration differs from lr_model, plane {p}"
    stats["frames"] += 1; stats["cdef"] += cdef is not None; stats["lr"] += (final is not None and lr_wanted(hdr))
    stats["sr_partial"] += bool(getattr(hdr, "use_superres", 0))
