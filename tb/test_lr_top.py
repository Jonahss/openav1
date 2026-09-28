"""Loop restoration RTL (lr_top via lr_tb_top) vs the Python model (lr_model.LoopRestoration).

Per frame: Python reconstruction + deblocking (+ CDEF when enabled) give the two input pictures and the unit
records; they are preloaded, restoration runs into the third frame buffer, which is compared with the model's
output over the visible picture. Frames without restoration are skipped.
Env: TS_IVF (real streams), TS_SEEDS etc. as elsewhere.
"""
import copy
import os
import random
import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge, Timer

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "tools"))
import cdef_model as cdm                # noqa: E402
import lf_model as lfm                  # noqa: E402
import lr_model as lrm                  # noqa: E402
import recon_model as rm                # noqa: E402
import test_tile_syntax as TT           # noqa: E402
import xcheck_frame as xf               # noqa: E402


def lr_rec_vals(plane, row, col, state):
    key = (plane, row, col)
    t = state.LrType.get(key, 0)
    wien = state.LrWiener.get(key, [[0, 0, 0], [0, 0, 0]]) if t == 1 else [[0, 0, 0], [0, 0, 0]]
    xqd = state.LrSgrXqd.get(key, [0, 0]) if t == 2 else [0, 0]
    return dict(plane=plane, unit_row=row, unit_col=col, lr_type=t,
                wiener=sum((wien[p][j] & 0x7F) << (7 * (3 * p + j)) for p in range(2) for j in range(3)),
                sgr_set=state.LrSgrSet.get(key, 0) if t == 2 else 0,
                xqd=(xqd[0] & 0xFF) | ((xqd[1] & 0xFF) << 8))


async def preload(dut, hdr, deblocked, cdef, state):
    await Timer(1, "ns")
    for buf, planes in ((0, deblocked), (1, cdef)):
        dut.h_buf.value = buf
        for plane in range(hdr.NumPlanes):
            sx = hdr.subsampling_x if plane else 0
            sy = hdr.subsampling_y if plane else 0
            W = min(((hdr.MiCols * 4) >> sx) + 8, len(planes[plane][0]), 1 << 10)
            H = min(((hdr.MiRows * 4) >> sy) + 8, len(planes[plane]), 1 << 9)
            dut.h_plane.value = plane
            for y in range(H):
                row = planes[plane][y]
                for x in range(W):
                    dut.h_we.value = 1
                    dut.h_x.value = x
                    dut.h_y.value = y
                    dut.h_wdata.value = row[x]
                    await RisingEdge(dut.clk)
            dut.h_we.value = 0
    for (plane, row, col) in state.LrType:
        dut.lr_we.value = 1
        dut.lr_rec.value = TT.pack(TT.LR_FIELDS, lr_rec_vals(plane, row, col, state))
        await RisingEdge(dut.clk)
        dut.lr_we.value = 0
    await Timer(1, "ns")


async def run_and_compare(dut, hdr, pre, post, tag, stats):
    dut.start.value = 1
    await RisingEdge(dut.clk)
    await Timer(1, "ns")
    dut.start.value = 0
    cycles = 0
    while True:
        await RisingEdge(dut.clk)
        await ReadOnly()
        cycles += 1
        if int(dut.done.value):
            break
        if cycles > 80_000_000:
            raise AssertionError(f"{tag}: restoration did not finish (st={int(dut.u_lr.st.value)} ly={int(dut.u_lr.ly.value)} lx={int(dut.u_lr.lx.value)})")
    await Timer(1, "ns")
    stats["cycles"] += cycles
    for plane in range(hdr.NumPlanes):
        sx = hdr.subsampling_x if plane else 0
        sy = hdr.subsampling_y if plane else 0
        W = (hdr.UpscaledWidth + sx) >> sx
        H = (hdr.FrameHeight + sy) >> sy
        bad = []
        changed = 0
        for y in range(H):
            for x in range(W):
                dut.h_plane.value = plane
                dut.h_x.value = x
                dut.h_y.value = y
                await RisingEdge(dut.clk)
                await ReadOnly()
                v = int(dut.h_rdata.value)
                if v != post[plane][y][x]:
                    bad.append((y, x, v, post[plane][y][x], pre[plane][y][x]))
                if post[plane][y][x] != pre[plane][y][x]:
                    changed += 1
                await Timer(1, "ns")
        stats["pixels"] += W * H
        stats["filtered"] += changed
        assert not bad, f"{tag} plane {plane} type {hdr.FrameRestorationType[plane]}: {len(bad)} / {W * H} mismatches (y, x, rtl, model, cdef-input), first {bad[:10]}"
    dut._log.info(f"{tag}: identical ({cycles} cycles, {stats['filtered']} pixels changed by restoration so far)")


async def one_frame(dut, tiles, tag, stats):
    frame = {}
    decs = []
    for th, data in tiles:
        dec = rm.FrameRecon(th, data, frame)
        dec.decode_tile()
        decs.append(dec)
    hdr = tiles[0][0]
    planes = frame["planes"]
    state = xf.FrameState(hdr, decs)
    lfm.LoopFilter(hdr, state, planes).apply()
    if all(t == 0 for t in hdr.FrameRestorationType[:hdr.NumPlanes]):
        dut._log.info(f"{tag}: restoration off, skipped")
        stats["skipped"] += 1
        return
    deblocked = planes
    if hdr.enable_cdef and not hdr.CodedLossless and not hdr.allow_intrabc:
        cdef = cdm.Cdef(hdr, state, planes).apply()
    else:
        cdef = copy.deepcopy(planes)
    post = lrm.LoopRestoration(hdr, state, deblocked, cdef).apply()
    dut.hdr.value = TT.pack(TT.HDR_FIELDS, TT.hdr_vals(hdr, decs[0]))
    await preload(dut, hdr, deblocked, cdef, state)
    await run_and_compare(dut, hdr, cdef, post, tag, stats)
    stats["frames"] += 1


@cocotb.test()
async def lr_vs_model(dut):
    seeds_env = os.environ.get("TS_SEEDS", "1,2,3")
    seeds = [int(x) for x in seeds_env.split(",") if x] if "," in seeds_env else list(range(1, int(seeds_env) + 1))
    W = int(os.environ.get("TS_W", "128"))
    H = int(os.environ.get("TS_H", "96"))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("start", "h_we", "h_buf", "h_plane", "h_x", "h_y", "h_wdata", "lr_we", "lr_rec", "hdr"):
        getattr(dut, s).value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    stats = dict(frames=0, skipped=0, pixels=0, filtered=0, cycles=0)
    ivfs = [p for p in os.environ.get("TS_IVF", "").split(",") if p]
    if ivfs:
        import obu_parser as op
        for path in ivfs:
            d = op.Decoder()
            d.feed_ivf(open(path, "rb").read())
            for fi, tile_idx in enumerate(d.frames):
                await one_frame(dut, [d.tiles[i] for i in tile_idx], f"{Path(path).name} frame {fi}", stats)
    else:
        for seed in seeds:
            fmt = os.environ.get("TS_FMT") or random.Random(seed * 7).choice(["420", "444", "mono", "422"])
            d, args = TT.gen(seed, W, H, fmt)
            await one_frame(dut, d.tiles, f"seed {seed} {fmt} bd{args['bd']} {W}x{H}", stats)
    dut._log.info(f"OK: {stats}")
