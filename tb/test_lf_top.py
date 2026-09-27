"""Deblocking loop filter RTL (lf_top via lf_tb_top) vs the Python model (lf_model.LoopFilter).

Per frame: the Python reconstruction produces the pre-filter picture and the per-4x4 state; both are preloaded
into the wrapper (frame_mem + mi_store), the filter runs, and the frame buffer is compared with the model's
filtered picture over the MI-aligned area.
Env: TS_SEEDS / TS_W / TS_H / TS_FMT / TS_SCREEN (generated streams, as in test_tile_syntax), TS_IVF (real streams).
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
import lf_model as lfm                  # noqa: E402
import recon_model as rm                # noqa: E402
import test_tile_syntax as TT           # noqa: E402
import xcheck_frame as xf               # noqa: E402

LF_FIELDS = [  # syn_pkg::lf_hdr_t order (MSB first)
    ("frame_width", 13), ("frame_height", 13), ("level", 24), ("sharpness", 3), ("delta_enabled", 1),
    ("ref_delta_intra", 7), ("delta_lf_multi", 1), ("seg_lf_en", 32), ("seg_lf_data", 224)]
MI_FIELDS = [("bsize", 5), ("skip", 1), ("seg", 3), ("delta_lf", 28)]


def lf_vals(th):
    return dict(frame_width=th.FrameWidth, frame_height=th.FrameHeight,
                level=sum((th.loop_filter_level[i] & 63) << (6 * i) for i in range(4)),
                sharpness=th.loop_filter_sharpness, delta_enabled=th.loop_filter_delta_enabled,
                ref_delta_intra=th.loop_filter_ref_deltas[0], delta_lf_multi=th.delta_lf_multi,
                seg_lf_en=sum((th.FeatureEnabled[s][1 + i] & 1) << (8 * i + s) for i in range(4) for s in range(8)),
                seg_lf_data=sum((th.FeatureData[s][1 + i] & 0x7F) << (7 * (8 * i + s)) for i in range(4) for s in range(8)))


async def preload(dut, hdr, planes, state):
    """Frame pixels (MI area + 16 px margin) and the per-4x4 state into the wrapper."""
    await Timer(1, "ns")
    for plane in range(hdr.NumPlanes):
        sx = hdr.subsampling_x if plane else 0
        sy = hdr.subsampling_y if plane else 0
        W = min(((hdr.MiCols * 4) >> sx) + 16, len(planes[plane][0]), 1 << 10)
        H = min(((hdr.MiRows * 4) >> sy) + 16, len(planes[plane]), 1 << 9)
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
    for r in range(hdr.MiRows):
        for c in range(hdr.MiCols):
            dlf = state.DeltaLFs[r][c] or [0, 0, 0, 0]
            dut.blk_we.value = 1
            dut.blk_r.value = r
            dut.blk_c.value = c
            dut.blk_bw4.value = 1
            dut.blk_bh4.value = 1
            dut.blk_data.value = TT.pack(MI_FIELDS, dict(bsize=state.MiSizes[r][c], skip=state.Skips[r][c], seg=state.SegmentIds[r][c],
                                                        delta_lf=sum((dlf[i] & 0x7F) << (7 * i) for i in range(4))))
            await RisingEdge(dut.clk)
            dut.blk_we.value = 0
            await RisingEdge(dut.clk)
    for plane in range(hdr.NumPlanes):
        lts = state.LoopfilterTxSizes[plane]
        for r in range(len(lts)):
            for c in range(len(lts[0])):
                dut.tx_we.value = 1
                dut.tx_plane.value = plane
                dut.tx_row.value = r
                dut.tx_col.value = c
                dut.tx_w4.value = 1
                dut.tx_h4.value = 1
                dut.tx_sz.value = lts[r][c]
                await RisingEdge(dut.clk)
                dut.tx_we.value = 0
                await RisingEdge(dut.clk)
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
        if cycles > 30_000_000:
            raise AssertionError(f"{tag}: loop filter did not finish (st={int(dut.u_lf.st.value)} plane={int(dut.u_lf.plane.value)} "
                                 f"row={int(dut.u_lf.row.value)} col={int(dut.u_lf.col.value)})")
    await Timer(1, "ns")
    stats["cycles"] += cycles
    for plane in range(hdr.NumPlanes):
        sx = hdr.subsampling_x if plane else 0
        sy = hdr.subsampling_y if plane else 0
        W = (hdr.MiCols * 4) >> sx
        H = (hdr.MiRows * 4) >> sy
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
        assert not bad, f"{tag} plane {plane}: {len(bad)} / {W * H} mismatches (y, x, rtl, model, pre-filter), first {bad[:10]}"
    dut._log.info(f"{tag}: identical ({cycles} cycles, {stats['filtered']} pixels changed by the filter so far)")


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
    pre = copy.deepcopy(planes)
    lfm.LoopFilter(hdr, state, planes).apply()
    dut.hdr.value = TT.pack(TT.HDR_FIELDS, TT.hdr_vals(hdr, decs[0]))
    dut.lh.value = TT.pack(LF_FIELDS, lf_vals(hdr))
    await preload(dut, hdr, pre, state)
    await run_and_compare(dut, hdr, pre, planes, tag, stats)
    stats["frames"] += 1


@cocotb.test()
async def lf_vs_model(dut):
    seeds_env = os.environ.get("TS_SEEDS", "1,2,3")
    seeds = [int(x) for x in seeds_env.split(",") if x] if "," in seeds_env else list(range(1, int(seeds_env) + 1))
    W = int(os.environ.get("TS_W", "128"))
    H = int(os.environ.get("TS_H", "96"))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("start", "h_we", "h_plane", "h_x", "h_y", "h_wdata", "blk_we", "tx_we", "hdr", "lh"):
        getattr(dut, s).value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    stats = dict(frames=0, pixels=0, filtered=0, cycles=0)
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
