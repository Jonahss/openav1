"""Stimulus and expected results for the deblocking edge decision (rtl/lf_edge.sv), cocotb-free: for a frame's header
and per-4x4 state, every (plane, pass, position) with the mode-info words and transform sizes the module is given and
what lf_model decides there. tb/test_lf_edge.py builds these for the frames it reconstructs itself (intra) and replays
files written by dump(); the decoder test benches call dump() on every frame they filter when TS_LF_VEC=<dir> is set,
which is how inter frames (reference and mode deltas, the skip-and-inter rule) reach the module's test."""
import os
import pickle
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lf_model as lfm                                                    # noqa: E402
from host_regs import HDR_FIELDS, LF_FIELDS, hdr_vals, lf_vals, pack      # noqa: E402

MI_FIELDS = [("bsize", 5), ("skip", 1), ("seg", 3), ("delta_lf", 28), ("ref0", 3), ("mode_type", 1)]
DUMP_DIR = os.environ.get("TS_LF_VEC_OUT", "")


class _Params(lfm.LoopFilter):
    def sample_filter(self, x, y, plane, limit, blimit, thresh, dx, dy, filterSize):
        self.got = (filterSize, limit, blimit, thresh)

    def params(self, plane, pss, row, col):
        self.got = None
        self.edge(plane, pss, row, col)
        return self.got


def mi_word(state, r, c):
    dlf = state.DeltaLFs[r][c] or [0, 0, 0, 0]
    rf = getattr(state, "RefFrames", None)
    ref0 = rf[r][c][0] if rf is not None and rf[r][c] is not None and rf[r][c][0] > 0 else 0
    ym = state.YModes[r][c] if getattr(state, "YModes", None) is not None else 0
    mode_type = 1 if (ym >= 13 and ym not in (15, 23)) else 0
    return pack(MI_FIELDS, dict(bsize=state.MiSizes[r][c], skip=state.Skips[r][c], seg=state.SegmentIds[r][c],
                                delta_lf=sum((dlf[i] & 0x7F) << (7 * i) for i in range(4)), ref0=ref0, mode_type=mode_type))


def vectors(hdr, state, dec0, tag=""):
    """None when the frame's loop filter is off; else {"tag", "hdr", "lh", "vec": [(plane, pss, row, col, cur, prv,
    cur_tx, prv_tx, exp)]} with exp = (size, limit, blimit, thresh) or None, and counts of inter positions."""
    if hdr.loop_filter_level[0] == 0 and hdr.loop_filter_level[1] == 0:
        return None
    par = _Params(hdr, state, None)
    vec = []
    words = {}
    for plane in range(hdr.NumPlanes):
        if not (plane == 0 or hdr.loop_filter_level[1 + plane]):
            continue
        sx = hdr.subsampling_x if plane else 0
        sy = hdr.subsampling_y if plane else 0
        lts = state.LoopfilterTxSizes[plane]
        for pss in (0, 1):
            for row in range(0, hdr.MiRows, 1 << sy):
                for col in range(0, hdr.MiCols, 1 << sx):
                    ra = row | sy; ca = col | sx
                    pr = ra - ((1 << sy) if pss else 0); pc = ca - (0 if pss else (1 << sx))
                    has_prev = pr >= 0 and pc >= 0
                    for k in ((ra, ca), (pr, pc)):
                        if k not in words and k[0] >= 0 and k[1] >= 0:
                            words[k] = mi_word(state, k[0], k[1])
                    vec.append((plane, pss, row, col, words[(ra, ca)], words[(pr, pc)] if has_prev else 0,
                                lts[ra >> sy][ca >> sx], lts[pr >> sy][pc >> sx] if has_prev else 0,
                                par.params(plane, pss, row, col)))
    inter = sum(1 for w in words.values() if (w >> 1) & 7)
    return dict(tag=tag, hdr=pack(HDR_FIELDS, hdr_vals(hdr, dec0)), lh=pack(LF_FIELDS, lf_vals(hdr)), vec=vec,
                inter_positions=inter, positions=len(words))


_n = [0]


def dump(hdr, state, dec0, tag):
    """Write the frame's vectors to TS_LF_VEC_OUT (called from the decoder test benches' filter step)."""
    if not DUMP_DIR:
        return
    v = vectors(hdr, state, dec0, tag)
    if v is None:
        return
    Path(DUMP_DIR).mkdir(parents=True, exist_ok=True)
    _n[0] += 1
    with open(Path(DUMP_DIR) / f"lfvec_{os.getpid()}_{_n[0]:04d}.pkl", "wb") as fh:
        pickle.dump(v, fh, protocol=4)
