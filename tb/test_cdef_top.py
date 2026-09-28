"""CDEF RTL (cdef_top via cdef_tb_top) vs the Python model (cdef_model.Cdef).

Per frame: Python reconstruction + deblocking give the CDEF input picture and the per-4x4 / per-64x64 state; they are
preloaded (src frame_mem + mi_store), CDEF runs into the dst frame_mem, which is compared with the model's output over
the MI-aligned area. Frames where CDEF is off (enable_cdef == 0, coded lossless) are skipped.
Env: TS_IVF (real streams; the generated streams are noise and rarely exercise the filter), TS_SEEDS etc. as elsewhere.
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
import recon_model as rm                # noqa: E402
import test_lf_top as TL                # noqa: E402
import test_tile_syntax as TT           # noqa: E402
import xcheck_frame as xf               # noqa: E402

CDEF_FIELDS = [("damping", 3), ("bits", 2), ("y_str", 48), ("uv_str", 48)]


def cdef_vals(th):
    return dict(damping=th.cdef_damping, bits=th.cdef_bits,
                y_str=sum((th.cdef_y_strengths[i] & 63) << (6 * i) for i in range(8)),
                uv_str=sum((th.cdef_uv_strengths[i] & 63) << (6 * i) for i in range(8)))


async def preload(dut, hdr, planes, state):
    await Timer(1, "ns")
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
    for r in range(hdr.MiRows):
        for c in range(hdr.MiCols):
            dut.blk_we.value = 1
            dut.blk_r.value = r
            dut.blk_c.value = c
            dut.blk_bw4.value = 1
            dut.blk_bh4.value = 1
            dut.blk_data.value = TT.pack(TL.MI_FIELDS, dict(bsize=state.MiSizes[r][c], skip=state.Skips[r][c], seg=state.SegmentIds[r][c], delta_lf=0))
            await RisingEdge(dut.clk)
            dut.blk_we.value = 0
            await RisingEdge(dut.clk)
    # cdef_idx per 64x64 unit (the model keys them by the unit's top-left 4x4 position; -1 = not coded)
    for (r, c), v in state.cdef_idx.items():
        dut.cd_row64.value = r >> 4
        dut.cd_col64.value = c >> 4
        dut.cd_sb128.value = 0
        dut.cd_mask.value = 1
        if v == -1:
            dut.cd_clr.value = 1
        else:
            dut.cd_we.value = 1
            dut.cd_idx.value = v
        await RisingEdge(dut.clk)
        dut.cd_clr.value = 0
        dut.cd_we.value = 0
    await Timer(1, "ns")


async def monitor_writes(dut, log):
    while True:
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.u_cdef.dst_we.value):
            log["n"] += 1
            if len(log["first"]) < 24:
                log["first"].append((int(dut.u_cdef.dst_plane.value), int(dut.u_cdef.dst_x.value), int(dut.u_cdef.dst_y.value),
                                     int(dut.u_cdef.dst_wdata.value), str(dut.u_cdef.st.value)))
        st = int(dut.u_cdef.st.value)
        log["st"][st] = log["st"].get(st, 0) + 1
        if st == 11 and len(log["dirs"]) < 6:      # C_STR: best_dir / best_cost / cost[] valid
            log["dirs"].append((int(dut.u_cdef.r.value), int(dut.u_cdef.c.value), int(dut.u_cdef.best_dir.value), int(dut.u_cdef.best_cost.value),
                                [int(dut.u_cdef.cost[i].value) for i in range(8)]))


async def run_and_compare(dut, hdr, pre, post, tag, stats):
    wlog = dict(n=0, first=[], st={}, dirs=[])
    if os.environ.get("TD_DEBUG"):
        cocotb.start_soon(monitor_writes(dut, wlog))
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
        if cycles > 60_000_000:
            raise AssertionError(f"{tag}: CDEF did not finish (st={int(dut.u_cdef.st.value)} r={int(dut.u_cdef.r.value)} c={int(dut.u_cdef.c.value)})")
    await Timer(1, "ns")
    stats["cycles"] += cycles
    if os.environ.get("TD_DEBUG"):
        dut._log.info(f"DBG writes={wlog['n']} cycles={cycles} state histogram={wlog['st']}")
        dut._log.info(f"DBG first writes (plane,x,y,val,st): {wlog['first'][:8]}")
        for (r, c, bd_, bc_, costs) in wlog["dirs"]:
            dut._log.info(f"DBG RTL block ({r},{c}): best_dir={bd_} best_cost={bc_} costs={costs}")
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
        assert not bad, f"{tag} plane {plane}: {len(bad)} / {W * H} mismatches (y, x, rtl, model, input), first {bad[:10]}"
    dut._log.info(f"{tag}: identical ({cycles} cycles, {stats['filtered']} pixels changed by CDEF so far)")


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
    if not (hdr.enable_cdef and not hdr.CodedLossless and not hdr.allow_intrabc):
        dut._log.info(f"{tag}: CDEF off, skipped")
        stats["skipped"] += 1
        return
    pre = copy.deepcopy(planes)
    cd = cdm.Cdef(hdr, state, planes)
    post = cd.apply()
    if os.environ.get("TD_DEBUG"):
        for (r, c) in ((0, 0), (0, 2), (0, 4), (2, 0)):
            idx = state.cdef_idx.get((r & ~15, c & ~15), -1)
            sk = [state.Skips[min(r + i, hdr.MiRows - 1)][min(c + j, hdr.MiCols - 1)] for i in (0, 1) for j in (0, 1)]
            d = cd.direction(r, c) if idx != -1 else None
            dut._log.info(f"DBG model block ({r},{c}): cdef_idx={idx} skips={sk} dir/var={d} y_str={hdr.cdef_y_strengths} damping={hdr.cdef_damping} bits={hdr.cdef_bits}")
    dut.hdr.value = TT.pack(TT.HDR_FIELDS, TT.hdr_vals(hdr, decs[0]))
    dut.ch.value = TT.pack(CDEF_FIELDS, cdef_vals(hdr))
    await preload(dut, hdr, pre, state)
    await run_and_compare(dut, hdr, pre, post, tag, stats)
    stats["frames"] += 1


@cocotb.test()
async def cdef_vs_model(dut):
    seeds_env = os.environ.get("TS_SEEDS", "1,2,3")
    seeds = [int(x) for x in seeds_env.split(",") if x] if "," in seeds_env else list(range(1, int(seeds_env) + 1))
    W = int(os.environ.get("TS_W", "128"))
    H = int(os.environ.get("TS_H", "96"))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("start", "h_we", "h_plane", "h_x", "h_y", "h_wdata", "blk_we", "cd_clr", "cd_we", "cd_row64", "cd_col64", "cd_sb128", "cd_mask", "cd_idx", "hdr", "ch"):
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
