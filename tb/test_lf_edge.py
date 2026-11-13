"""The deblocking edge decision (rtl/lf_edge.sv, combinational) against lf_model on whole frames: for every plane,
pass and 4x4 position the module gets the frame header, the mode info of the position and of its neighbour across the
edge and the two transform sizes, as the streaming stage will supply them; its {apply, size, limit, blimit, thresh}
must equal what lf_model would call the sample filter with (or apply = 0 where the model does not filter).
Env: TS_SEEDS / TS_W / TS_H / TS_FMT (generated intra frames), TS_IVF (every frame of real streams the Python
reconstruction can decode alone, i.e. their intra frames)."""
import os
import random
import sys
from pathlib import Path

import cocotb
from cocotb.triggers import Timer

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "tools"))
import lf_model as lfm                  # noqa: E402
import recon_model as rm                # noqa: E402
import test_tile_syntax as TT           # noqa: E402
import xcheck_frame as xf               # noqa: E402
from host_regs import LF_FIELDS, lf_vals   # noqa: E402

MI_FIELDS = [("bsize", 5), ("skip", 1), ("seg", 3), ("delta_lf", 28), ("ref0", 3), ("mode_type", 1)]


class _Params(lfm.LoopFilter):
    def sample_filter(self, x, y, plane, limit, blimit, thresh, dx, dy, filterSize):
        self.got = (filterSize, limit, blimit, thresh)

    def params(self, plane, pss, row, col):
        self.got = None
        self.edge(plane, pss, row, col)
        if self.got is None:
            return None
        return self.got


def mi_word(state, r, c):
    dlf = state.DeltaLFs[r][c] or [0, 0, 0, 0]
    rf = getattr(state, "RefFrames", None)
    ref0 = rf[r][c][0] if rf is not None and rf[r][c] is not None and rf[r][c][0] > 0 else 0
    ym = state.YModes[r][c] if getattr(state, "YModes", None) is not None else 0
    mode_type = 1 if (ym >= 13 and ym not in (15, 23)) else 0
    return TT.pack(MI_FIELDS, dict(bsize=state.MiSizes[r][c], skip=state.Skips[r][c], seg=state.SegmentIds[r][c],
                                   delta_lf=sum((dlf[i] & 0x7F) << (7 * i) for i in range(4)), ref0=ref0, mode_type=mode_type))


async def one_frame(dut, tiles, tag, stats):
    frame = {}; decs = []
    for th, data in tiles:
        dec = rm.FrameRecon(th, data, frame); dec.decode_tile(); decs.append(dec)
    hdr = tiles[0][0]
    state = xf.FrameState(hdr)
    for _d in decs:
        state.add(_d)
    if hdr.loop_filter_level[0] == 0 and hdr.loop_filter_level[1] == 0:
        stats["skipped"] += 1
        return
    par = _Params(hdr, state, None)
    dut.hdr.value = TT.pack(TT.HDR_FIELDS, TT.hdr_vals(hdr, decs[0]))
    dut.lh.value = TT.pack(LF_FIELDS, lf_vals(hdr))
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
                    dut.plane.value = plane; dut.pss.value = pss; dut.row.value = row; dut.col.value = col
                    dut.cur.value = mi_word(state, ra, ca)
                    dut.prv.value = mi_word(state, pr, pc) if has_prev else 0
                    dut.cur_tx.value = lts[ra >> sy][ca >> sx]
                    dut.prv_tx.value = lts[pr >> sy][pc >> sx] if has_prev else 0
                    await Timer(1, "ns")
                    got = int(dut.par.value)
                    exp = par.params(plane, pss, row, col)
                    if exp is None:
                        assert (got >> 29) == 0, f"{tag} plane {plane} pass {pss} ({row}, {col}): apply set, the model does not filter"
                        stats["off"] += 1
                    else:
                        fs, limit, blimit, thresh = exp
                        e = (1 << 29) | (fs << 24) | (limit << 16) | (blimit << 8) | thresh
                        assert got == e, (f"{tag} plane {plane} pass {pss} ({row}, {col}): par {got:08x} "
                                          f"(apply {got >> 29} size {(got >> 24) & 31} limit {(got >> 16) & 255} blimit {(got >> 8) & 255} thresh {got & 255}), "
                                          f"model size {fs} limit {limit} blimit {blimit} thresh {thresh}")
                        stats["on"] += 1
                        stats[f"size{fs}"] = stats.get(f"size{fs}", 0) + 1
    stats["frames"] += 1


@cocotb.test()
async def lf_edge_vs_model(dut):
    seeds = [int(x) for x in os.environ.get("TS_SEEDS", "2,3,7,20").split(",") if x]
    W = int(os.environ.get("TS_W", "128")); H = int(os.environ.get("TS_H", "96"))
    stats = dict(frames=0, on=0, off=0, skipped=0)
    ivfs = [p for p in os.environ.get("TS_IVF", "").split(",") if p]
    if ivfs:
        import obu_parser as op
        for path in ivfs:
            d = op.Decoder(); d.feed_ivf(open(path, "rb").read())
            await one_frame(dut, [d.tiles[i] for i in d.frames[0]], f"{Path(path).name} frame 0", stats)
    else:
        for seed in seeds:
            fmt = os.environ.get("TS_FMT") or random.Random(seed * 7).choice(["420", "444", "mono", "422"])
            try:
                d, args = TT.gen(seed, W, H, fmt)
            except Exception:
                stats["skipped"] += 1
                continue
            await one_frame(dut, d.tiles, f"seed {seed} {fmt} bd{args['bd']} {W}x{H}", stats)
    assert stats["frames"] > 0 and stats["on"] > 1000 and stats["off"] > 1000, stats
    dut._log.info(f"OK: {stats}")
