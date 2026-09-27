"""Fuzz the CfL RTL against intra_model.cfl_subsample_luma + predict_cfl. Env: CFL_ITERS, CFL_SEED."""
import os
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, Timer, ClockCycles

import intra_model as im


@cocotb.test()
async def fuzz_vs_model(dut):
    iters = int(os.environ.get("CFL_ITERS", "150"))
    rng = random.Random(int(os.environ.get("CFL_SEED", "1")))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1; dut.start.value = 0; dut.luma_we.value = 0; dut.dc_we.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    for it in range(iters):
        bd = rng.choice([8, 10])
        mx = (1 << bd) - 1
        sx, sy = rng.choice([(1, 1), (1, 1), (1, 0), (0, 0)])
        log2w = rng.randint(2, 5 - sx); log2h = rng.randint(2, 5 - sy)
        if abs(log2w - log2h) > 2: continue
        w, h = 1 << log2w, 1 << log2h
        lw, lh = w << sx, h << sy
        # luma region, possibly cut by the frame edge (availability in luma samples, multiple of the subsampling step)
        aw = rng.choice([lw, lw, lw, max(1 << sx, lw - (1 << sx) * rng.randint(1, max(1, (lw >> sx) - 1)))])
        ah = rng.choice([lh, lh, lh, max(1 << sy, lh - (1 << sy) * rng.randint(1, max(1, (lh >> sy) - 1)))])
        luma = [[rng.randint(0, mx) for _ in range(32)] for _ in range(32)]
        dc_val = rng.randint(0, mx)
        dc = [[dc_val] * w for _ in range(h)]
        alpha = rng.choice([rng.randint(-16, 16), 16, -16, 1, -1])
        L = im.cfl_subsample_luma(luma, 0, 0, w, h, sx, sy, aw, ah)
        exp = im.predict_cfl(dc, L, alpha, log2w, log2h, bd)
        # load
        for y in range(32):
            for x in range(32):
                dut.luma_we.value = 1; dut.luma_addr.value = y * 32 + x; dut.luma_data.value = luma[y][x]
                await RisingEdge(dut.clk)
        dut.luma_we.value = 0
        for i in range(h):
            for j in range(w):
                dut.dc_we.value = 1; dut.dc_addr.value = i * 32 + j; dut.dc_data.value = dc[i][j]
                await RisingEdge(dut.clk)
        dut.dc_we.value = 0
        await Timer(1, "ns")
        dut.log2w.value = log2w; dut.log2h.value = log2h; dut.sub_x.value = sx; dut.sub_y.value = sy
        dut.alpha.value = alpha & 63; dut.bit_depth.value = bd; dut.luma_avail_w.value = aw; dut.luma_avail_h.value = ah
        dut.start.value = 1
        await RisingEdge(dut.clk); await Timer(1, "ns"); dut.start.value = 0
        got = [[None] * w for _ in range(h)]
        n = 0; cyc = 0
        while True:
            await RisingEdge(dut.clk); cyc += 1
            await ReadOnly()
            if int(dut.out_valid.value):
                got[int(dut.out_y.value)][int(dut.out_x.value)] = int(dut.out_pix.value); n += 1
            if int(dut.done.value): break
            await Timer(1, "ns")
            assert cyc < 5000
        await Timer(1, "ns")
        assert n == w * h
        if got != exp:
            bad = [(i, j, got[i][j], exp[i][j]) for i in range(h) for j in range(w) if got[i][j] != exp[i][j]]
            raise AssertionError(f"case {it} {w}x{h} ss({sx},{sy}) bd{bd} alpha {alpha} avail({aw},{ah}) dc {dc_val}: {len(bad)} bad, first {bad[:6]}")
    dut._log.info(f"OK: {iters} CfL blocks match the model")
