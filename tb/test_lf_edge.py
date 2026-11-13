"""The deblocking edge decision (rtl/lf_edge.sv, combinational) against lf_model on whole frames: for every plane,
pass and 4x4 position the module gets the frame header, the mode info of the position and of its neighbour across the
edge and the two transform sizes, as the streaming stage will supply them; its {apply, size, limit, blimit, thresh}
must equal what lf_model would call the sample filter with (or apply = 0 where the model does not filter).
Env: TS_SEEDS / TS_W / TS_H / TS_FMT (generated intra frames), TS_IVF (frame 0 of real streams), TS_LF_VEC=<dir>
(frames dumped by a decoder test bench run with TS_LF_VEC_OUT=<dir>: inter frames; see tb/lf_edge_vec.py)."""
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

import lf_edge_vec as lv                # noqa: E402


async def run_vectors(dut, v, stats):
    tag = v["tag"]
    dut.hdr.value = v["hdr"]; dut.lh.value = v["lh"]
    for (plane, pss, row, col, cur, prv, cur_tx, prv_tx, exp) in v["vec"]:
        dut.plane.value = plane; dut.pss.value = pss; dut.row.value = row; dut.col.value = col
        dut.cur.value = cur; dut.prv.value = prv; dut.cur_tx.value = cur_tx; dut.prv_tx.value = prv_tx
        await Timer(1, "ns")
        got = int(dut.par.value)
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
    stats["inter_positions"] += v["inter_positions"]; stats["positions"] += v["positions"]


async def one_frame(dut, tiles, tag, stats):
    frame = {}; decs = []
    for th, data in tiles:
        dec = rm.FrameRecon(th, data, frame); dec.decode_tile(); decs.append(dec)
    hdr = tiles[0][0]
    state = xf.FrameState(hdr)
    for _d in decs:
        state.add(_d)
    v = lv.vectors(hdr, state, decs[0], tag)
    if v is None:
        stats["skipped"] += 1
        return
    await run_vectors(dut, v, stats)


@cocotb.test()
async def lf_edge_vs_model(dut):
    seeds = [int(x) for x in os.environ.get("TS_SEEDS", "2,3,7,20").split(",") if x]
    W = int(os.environ.get("TS_W", "128")); H = int(os.environ.get("TS_H", "96"))
    stats = dict(frames=0, on=0, off=0, skipped=0, inter_positions=0, positions=0)
    ivfs = [p for p in os.environ.get("TS_IVF", "").split(",") if p]
    vdir = os.environ.get("TS_LF_VEC", "")
    if vdir:                                                 # frames dumped by a decoder test bench (lf_edge_vec.dump)
        import pickle
        files = sorted(Path(vdir).glob("lfvec_*.pkl"))
        assert files, f"no vector files in {vdir}"
        for f in files:
            with open(f, "rb") as fh:
                await run_vectors(dut, pickle.load(fh), stats)
        assert stats["inter_positions"] > 0, f"the dumped frames have no inter position: {stats}"
    elif ivfs:
        import obu_parser as op
        for path in ivfs:
            d = op.parse_file(path)                          # .ivf, or Argon's .obu
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
