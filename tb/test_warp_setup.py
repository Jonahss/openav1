"""Fuzz rtl/warp_setup.sv against tb/recon_model.py's warp_estimation (7.11.3.8) and setup_shear (7.11.3.6):
random warp-sample lists (local mode; the DUT receives the least-squares sums the syntax stage accumulates) and
random global motion parameters (global mode). Env: WS_ITERS (default 300), WS_SEED."""
import os
import random
import types
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, Timer

import recon_model as rm
import av1_tables as T

LS_MV_MAX = rm.LS_MV_MAX


def ls_sums(cands, mi_size, mi_row, mi_col, mv):
    """The seven least-squares sums of warp_estimation over the (valid) samples, as blk_syntax accumulates them."""
    w4 = T.Num_4x4_Blocks_Wide[mi_size]
    h4 = T.Num_4x4_Blocks_High[mi_size]
    midY = mi_row * 4 + h4 * 2 - 1
    midX = mi_col * 4 + w4 * 2 - 1
    suy, sux = midY * 8, midX * 8
    duy, dux = suy + mv[0], sux + mv[1]
    a00 = a01 = a11 = bx0 = bx1 = by0 = by1 = 0

    def lsp(a, b):
        return ((a * b) >> 2) + (a + b)
    for c in cands:
        sy, sx = c[0] - suy, c[1] - sux
        dy, dx = c[2] - duy, c[3] - dux
        if abs(sx - dx) < LS_MV_MAX and abs(sy - dy) < LS_MV_MAX:
            a00 += lsp(sx, sx) + 8
            a01 += lsp(sx, sy) + 4
            a11 += lsp(sy, sy) + 8
            bx0 += lsp(sx, dx) + 8
            bx1 += lsp(sy, dx) + 4
            by0 += lsp(sx, dy) + 4
            by1 += lsp(sy, dy) + 8
    return (a00, a01, a11, bx0, bx1, by0, by1), midX, midY


def s16(v):
    return v - 65536 if v >= 32768 else v


def s24(v):
    return v - (1 << 24) if v >= (1 << 23) else v


async def run(dut, mode, sums, mv, midx, midy, gm):
    for name, v in zip(("ls_a00", "ls_a01", "ls_a11", "ls_bx0", "ls_bx1", "ls_by0", "ls_by1"), sums):
        assert -(1 << 31) <= v < (1 << 31), f"sum {name} = {v} outside 32 bits"
        getattr(dut, name).value = v & 0xFFFFFFFF
    dut.mv_row.value = mv[0] & 0xFFFF; dut.mv_col.value = mv[1] & 0xFFFF
    dut.mid_x.value = midx; dut.mid_y.value = midy
    gmv = 0
    for i in range(6):
        gmv |= (gm[i] & 0xFFFFFF) << (24 * i)
    dut.gm_p.value = gmv
    dut.mode.value = mode
    dut.start.value = 1
    await RisingEdge(dut.clk)
    await Timer(1, "ns")
    dut.start.value = 0
    for cyc in range(64):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.done.value):
            pv = int(dut.p.value)
            p = [s24((pv >> (24 * i)) & 0xFFFFFF) for i in range(6)]
            sh = tuple(s16(int(getattr(dut, n).value)) for n in ("alpha", "beta", "gamma", "delta"))
            v = int(dut.valid.value)
            await Timer(1, "ns")
            return v, p, sh, cyc
        await Timer(1, "ns")
    raise AssertionError("no done")


@cocotb.test()
async def warp_setup_vs_model(dut):
    iters = int(os.environ.get("WS_ITERS", "300"))
    rng = random.Random(int(os.environ.get("WS_SEED", "1")))
    cocotb.start_soon(Clock(dut.clk, 10, "ns").start())
    dut.rst.value = 1; dut.start.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    stats = {"local": 0, "local_valid": 0, "global": 0, "global_valid": 0, "cycles_max": 0}
    for it in range(iters):
        if rng.random() < 0.6:
            # ---- local: a block, its MV and 1..8 samples around it (positions in 1/8 luma samples)
            mi_size = rng.randint(3, 21)        # BLOCK_8X8 and up (warp needs w, h >= 8)
            w4 = T.Num_4x4_Blocks_Wide[mi_size]; h4 = T.Num_4x4_Blocks_High[mi_size]
            mi_row, mi_col = rng.randint(0, 300), rng.randint(0, 500)
            mv = (rng.randint(-2048, 2047), rng.randint(-2048, 2047))
            midY = mi_row * 4 + h4 * 2 - 1
            midX = mi_col * 4 + w4 * 2 - 1
            n = rng.randint(1, 8)
            cands = []
            for _ in range(n):
                # neighbour centres within a couple of block sizes, their MVs near the block's (or wild)
                cy = midY * 8 + rng.randint(-8 * (h4 * 4 + 64), 8 * (h4 * 4 + 64))
                cx = midX * 8 + rng.randint(-8 * (w4 * 4 + 64), 8 * (w4 * 4 + 64))
                spread = 32 if rng.random() < 0.7 else 600
                cands.append([cy, cx, cy + mv[0] + rng.randint(-spread, spread), cx + mv[1] + rng.randint(-spread, spread)])
            fake = types.SimpleNamespace(MiSize=mi_size, MiRow=mi_row, MiCol=mi_col, Mv=[list(mv)], NumSamples=n, CandList=cands,
                                         resolve_divisor=rm.FrameRecon.resolve_divisor, setup_shear=lambda wp: rm.FrameRecon.setup_shear(fake, wp))
            rm.FrameRecon.warp_estimation(fake)
            if fake.LocalValid:
                exp_valid, *exp_sh = rm.FrameRecon.setup_shear(fake, fake.LocalWarpParams)
                exp_p = fake.LocalWarpParams
            else:
                exp_valid, exp_sh, exp_p = 0, None, None
            sums, midx, midy = ls_sums(cands, mi_size, mi_row, mi_col, mv)
            v, p, sh, cyc = await run(dut, 0, sums, mv, midx, midy, [0] * 6)
            stats["local"] += 1
            assert v == exp_valid, f"iter {it} local: valid {v} vs model {exp_valid} (sums {sums}, p {p}, model p {exp_p})"
            if exp_valid:
                stats["local_valid"] += 1
                assert p == exp_p, f"iter {it} local: params {p} vs model {exp_p} (sums {sums})"
                assert sh == tuple(exp_sh), f"iter {it} local: shear {sh} vs model {tuple(exp_sh)} (p {p})"
        else:
            # ---- global: parameters around identity, sometimes wild
            wide = rng.random() < 0.3
            gm = [rng.randint(-(1 << 20), (1 << 20)), rng.randint(-(1 << 20), (1 << 20)),
                  65536 + rng.randint(-(1 << 13 if wide else 1 << 11), (1 << 13 if wide else 1 << 11)),
                  rng.randint(-(1 << 13 if wide else 1 << 11), (1 << 13 if wide else 1 << 11)),
                  rng.randint(-(1 << 13 if wide else 1 << 11), (1 << 13 if wide else 1 << 11)),
                  65536 + rng.randint(-(1 << 13 if wide else 1 << 11), (1 << 13 if wide else 1 << 11))]
            exp_valid, *exp_sh = rm.FrameRecon.setup_shear(types.SimpleNamespace(resolve_divisor=rm.FrameRecon.resolve_divisor), gm)
            v, p, sh, cyc = await run(dut, 1, [0] * 7, (0, 0), 0, 0, gm)
            stats["global"] += 1
            assert v == exp_valid, f"iter {it} global: valid {v} vs model {exp_valid} (gm {gm}, shear {sh} vs {exp_sh})"
            assert p == gm, f"iter {it} global: params passed through wrong: {p} vs {gm}"
            if exp_valid:
                stats["global_valid"] += 1
            assert sh == tuple(exp_sh), f"iter {it} global: shear {sh} vs model {tuple(exp_sh)} (gm {gm})"
        stats["cycles_max"] = max(stats["cycles_max"], cyc)
    dut._log.info(f"OK: {stats}")
