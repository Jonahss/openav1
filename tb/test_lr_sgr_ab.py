"""Fuzz rtl/lr_sgr_ab.sv (a2 / b2 of a self-guided box position, four clocks) against lr_model.LoopRestoration.sgr_ab:
boxes of 25 or 9 random samples (flat, noisy, extreme: all the ranges of z), every parameter set's s, 8 / 10 / 12 bit,
positions back to back and with gaps.
Env: SA_ITERS (default 20000), SA_SEED."""
import os
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge, Timer

import av1_tables as T
import lr_model as lrm


@cocotb.test()
async def lr_sgr_ab_vs_model(dut):
    iters = int(os.environ.get("SA_ITERS", "20000"))
    rng = random.Random(int(os.environ.get("SA_SEED", "1")))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1; dut.v_in.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk); await Timer(1, "ns")
    expect = []; seen = [0]
    stats = dict(n=0, a2_1=0, a2_256=0, a2_mid=0, p_floor=0)

    async def monitor():
        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.v_out.value):
                assert expect, "v_out with nothing expected"
                exp, tag = expect.pop(0)
                got = (int(dut.a2.value), int(dut.b2.value))
                assert got == exp, f"{tag}: rtl {got}, model {exp}"
                seen[0] += 1
    cocotb.start_soon(monitor())
    bd = 8
    for it in range(iters):
        if it % 500 == 0:                                    # the bit depth belongs to the frame
            dut.v_in.value = 0
            await ClockCycles(dut.clk, 6); await Timer(1, "ns")
            bd = rng.choice([8, 10, 12])
        mx = (1 << bd) - 1
        st = rng.randrange(16); pss = rng.randrange(2)
        r = T.Sgr_Params[st][pss * 2]; eps = T.Sgr_Params[st][pss * 2 + 1]
        if r == 0:
            pss ^= 1; r = T.Sgr_Params[st][pss * 2]; eps = T.Sgr_Params[st][pss * 2 + 1]
        n = (2 * r + 1) ** 2
        base = rng.randint(0, mx); nz = rng.choice([0, 1, 2, 4, 12, 40, 200, 4095]) << (bd - 8)
        kind = rng.random()
        if kind < 0.1:
            c = [rng.choice([0, mx]) for _ in range(n)]
        else:
            c = [max(0, min(mx, base + rng.randint(-nz, nz))) for _ in range(n)]
        a = sum(v * v for v in c); b = sum(c)
        n2e = n * n * eps
        s = ((1 << 20) + n2e // 2) // n2e
        one_over_n = ((1 << 12) + n // 2) // n
        exp = lrm.LoopRestoration.sgr_ab(a, b, bd, n, s, one_over_n)
        ar = lrm.round2(a, 2 * (bd - 8)); d = lrm.round2(b, bd - 8)
        stats["n"] += 1; stats["a2_1"] += exp[0] == 1; stats["a2_256"] += exp[0] == 256; stats["a2_mid"] += 1 < exp[0] < 256
        stats["p_floor"] += ar * n < d * d
        dut.bd.value = bd; dut.r1.value = int(r == 1); dut.s.value = s; dut.box_a.value = a; dut.box_b.value = b
        dut.v_in.value = 1
        expect.append((exp, f"position {it} bd{bd} set {st} pass {pss} r {r} s {s}: a {a} b {b}"))
        await RisingEdge(dut.clk); await Timer(1, "ns")
        if rng.random() < 0.2:
            dut.v_in.value = 0
            await RisingEdge(dut.clk); await Timer(1, "ns")
    dut.v_in.value = 0
    await ClockCycles(dut.clk, 8)
    assert not expect and seen[0] == stats["n"], f"{seen[0]} positions came out, {stats['n']} went in"
    assert stats["a2_1"] > 100 and stats["a2_256"] > 100 and stats["a2_mid"] > 1000 and stats["p_floor"] > 20, stats
    dut._log.info(f"OK: {stats}")
