"""Frames for the streaming CDEF stage's test (tb/test_cdef_stream.py), cocotb-free: the deblocked luma plane, the
skip flags, cdef_idx, the frame's CDEF parameters and cdef_model's luma plane, cropped to the coded area. The decoder
test benches call dump() on every frame they run CDEF on when TS_CDEF_VEC_OUT=<dir> is set: that is how inter frames
(whole 8x8 blocks skipped, 64x64 units without a cdef_idx) reach the stage's test."""
import gzip
import os
import pickle
import types
from pathlib import Path

DUMP_DIR = os.environ.get("TS_CDEF_VEC_OUT", "")
_n = [0]


def pack(hdr, state, deblocked, cdef, tag):
    H, W = hdr.MiRows * 4, hdr.MiCols * 4
    skips = [[int(state.Skips[r][c]) for c in range(hdr.MiCols)] for r in range(hdr.MiRows)]
    n8 = 0; skipped8 = 0; noidx8 = 0; fourth = 0
    for r in range(0, hdr.MiRows, 2):
        for c in range(0, hdr.MiCols, 2):
            n8 += 1
            r1 = min(r + 1, hdr.MiRows - 1); c1 = min(c + 1, hdr.MiCols - 1)
            f = (skips[r][c], skips[r1][c], skips[r][c1], skips[r1][c1])
            if state.cdef_idx.get((r & ~15, c & ~15), -1) == -1:
                noidx8 += 1
            elif all(f):
                skipped8 += 1
            elif f[0] and f[1] and f[2]:
                fourth += 1
    return dict(tag=tag, BitDepth=hdr.BitDepth, MiRows=hdr.MiRows, MiCols=hdr.MiCols, cdef_damping=hdr.cdef_damping,
                cdef_y_strengths=list(hdr.cdef_y_strengths), cdef_uv_strengths=list(hdr.cdef_uv_strengths),
                dbl=[list(row[:W]) for row in deblocked[0][:H]], post=[list(row[:W]) for row in cdef[0][:H]],
                skips=skips, cdef_idx=dict(state.cdef_idx), blocks8=n8, skipped8=skipped8, noidx8=noidx8, fourth_decides=fourth)


def unpack(v):
    hdr = types.SimpleNamespace(BitDepth=v["BitDepth"], MiRows=v["MiRows"], MiCols=v["MiCols"], cdef_damping=v["cdef_damping"],
                                cdef_y_strengths=v["cdef_y_strengths"], cdef_uv_strengths=v["cdef_uv_strengths"])
    state = types.SimpleNamespace(Skips=v["skips"], cdef_idx=v["cdef_idx"])
    return hdr, state, v["dbl"], v["post"]


def dump(hdr, state, deblocked, cdef, tag):
    if not DUMP_DIR:
        return
    Path(DUMP_DIR).mkdir(parents=True, exist_ok=True)
    _n[0] += 1
    with gzip.open(Path(DUMP_DIR) / f"cdefvec_{os.getpid()}_{_n[0]:04d}.pkl.gz", "wb") as fh:
        pickle.dump(pack(hdr, state, deblocked, cdef, tag), fh, protocol=4)
