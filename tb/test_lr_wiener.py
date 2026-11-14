"""Fuzz the Wiener line modules (rtl/lr_wiener.sv: seven lr_wiener_h into lr_wiener_v, through lr_wiener_tb_top)
against lr_model.LoopRestoration: random small pictures (deblocked and CDEF planes), bit depth, coefficients in the
ranges the bitstream allows and a random 4-line block; the model restores the block (two neighbouring blocks for the
8-lane build) and the module is given, for each of the block's lines, the seven source lines of N + 6 samples the
model's source rule (7.17.6: plane clamp, stripe rule) returns. So the arithmetic is checked here, with real border
and stripe cases in the samples; the rule itself belongs to the stage.
Env: LW_ITERS (blocks, default 1500), LW_SEED, LW_N (lanes of the build: 8 default, 4)."""
import os
import random
import types
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge, Timer

import lr_model as lrm

N = int(os.environ.get("LW_N", "8"))


@cocotb.test()
async def lr_wiener_vs_model(dut):
    iters = int(os.environ.get("LW_ITERS", "1500"))
    rng = random.Random(int(os.environ.get("LW_SEED", "1")))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1; dut.v_in.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk); await Timer(1, "ns")
    expect = []; seen = [0]; last_bd = [0]
    stats = dict(blocks=0, lines=0, changed=0, stripe_edge=0, plane_edge=0, clamped_out=0)

    async def monitor():
        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.v_out.value):
                assert expect, "v_out with nothing expected"
                exp, tag = expect.pop(0)
                got = [(int(dut.out.value) >> (12 * l)) & 0xFFF for l in range(N)]
                assert got == exp, f"{tag}: rtl {got}, model {exp}"
                seen[0] += 1
    cocotb.start_soon(monitor())
    for it in range(iters):
        bd = rng.choice([8, 10, 12]); mx = (1 << bd) - 1
        W = rng.choice([16, 24, 64, 136]); H = rng.choice([8, 16, 72, 136])
        base = rng.randint(0, mx); nz = rng.choice([2, 8, 40, 300]) << (bd - 8)
        mk = lambda: [[max(0, min(mx, base + rng.randint(-nz, nz))) for _ in range(W)] for _ in range(H)]
        if rng.random() < 0.15:                              # extremes, to reach the clamps
            mk = lambda: [[rng.choice([0, mx]) for _ in range(W)] for _ in range(H)]
        cur, cdef = mk(), mk()
        hdr = types.SimpleNamespace(BitDepth=bd, FrameHeight=H, UpscaledWidth=W, subsampling_x=0, subsampling_y=0, NumPlanes=1,
                                    LoopRestorationSize=[64], FrameRestorationType=[1])
        vc = [rng.randint(-5, 10), rng.randint(-23, 8), rng.randint(-17, 46)]
        hc = [rng.randint(-5, 10), rng.randint(-23, 8), rng.randint(-17, 46)]
        keys = {(0, ur, uc): 1 for ur in range(4) for uc in range(4)}
        state = types.SimpleNamespace(LrType=keys, LrWiener={k: [vc, hc] for k in keys})
        m = lrm.LoopRestoration(hdr, state, [cur], [cdef])
        m.lr = [[row[:] for row in cdef]]
        m.InterRound0 = 3 + (2 if bd == 12 else 0); m.InterRound1 = 11 - (2 if bd == 12 else 0)
        row = rng.randrange(H // 4); col = rng.randrange(0, W // 4, N // 4)
        for b in range(N // 4):
            m.block(0, row, col + b)
        x, y = col * 4, row * 4
        pk = lambda c3: sum((v & 0x7F) << (7 * i) for i, v in enumerate(c3))
        if bd != last_bd[0]:                                 # the bit depth belongs to the frame: let the lines inside leave first
            dut.v_in.value = 0
            await ClockCycles(dut.clk, 6); await Timer(1, "ns")
            last_bd[0] = bd
        dut.bd.value = bd; dut.hcoef.value = pk(hc); dut.vcoef.value = pk(vc)
        stats["blocks"] += 1
        stats["stripe_edge"] += (y - 3 < m.StripeStartY) or (y + 6 > m.StripeEndY)
        stats["plane_edge"] += (x < 3) or (x + N + 2 >= W) or (y < 3) or (y + 6 >= H)
        for r in range(4):
            px = 0
            for rr in range(7):
                for i in range(N + 6):
                    px |= m.src(0, x + i - 3, y + r + rr - 3) << (12 * (rr * (N + 6) + i))
            exp = m.lr[0][y + r][x:x + N]
            dut.v_in.value = 1; dut.px.value = px
            expect.append((exp, f"block {it} bd{bd} {W}x{H} at ({y}, {x}) line {r} vcoef {vc} hcoef {hc}"))
            stats["lines"] += 1; stats["changed"] += exp != cdef[y + r][x:x + N]
            stats["clamped_out"] += any(v in (0, mx) for v in exp)
            await RisingEdge(dut.clk); await Timer(1, "ns")
            if rng.random() < 0.2:
                dut.v_in.value = 0
                await RisingEdge(dut.clk); await Timer(1, "ns")
    dut.v_in.value = 0
    await ClockCycles(dut.clk, 8)
    assert not expect and seen[0] == stats["lines"], f"{seen[0]} lines came out, {stats['lines']} went in"
    assert stats["changed"] > stats["lines"] // 2 and stats["stripe_edge"] > 100 and stats["plane_edge"] > 100 and stats["clamped_out"] > 100, stats
    dut._log.info(f"OK: {stats}")
