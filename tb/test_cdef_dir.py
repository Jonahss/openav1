"""Fuzz the CDEF direction search (rtl/cdef_dir.sv) against cdef_model.Cdef.direction: random 8x8 luma blocks (flat,
noisy, and with an oriented edge so that every direction wins), bit depths 8 / 10 / 12, a block every clock or with
gaps. Env: CD_ITERS (default 4000), CD_SEED."""
import math
import os
import random
import types
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge, Timer

import cdef_model as cdm


@cocotb.test()
async def cdef_dir_vs_model(dut):
    iters = int(os.environ.get("CD_ITERS", "4000"))
    rng = random.Random(int(os.environ.get("CD_SEED", "1")))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1; dut.v_in.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk); await Timer(1, "ns")
    expect = []; seen = [0]
    stats = dict(blocks=0, var0=0, dirs={})

    async def monitor():
        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.v_out.value):
                assert expect, "v_out with nothing expected"
                (d, v), tag = expect.pop(0)
                got = (int(dut.dir.value), int(dut.var_o.value))
                assert got == (d, v & 0xFFFFFFFF), f"{tag}: rtl dir {got[0]} var {got[1]}, model dir {d} var {v}"
                seen[0] += 1
    cocotb.start_soon(monitor())
    for it in range(iters):
        bd = rng.choice([8, 10, 12]); cs = bd - 8; mx = (1 << bd) - 1
        kind = rng.random()
        if kind < 0.15:
            blk = [[rng.randint(0, mx)] * 8 for _ in range(1)] * 8                 # flat
            blk = [row[:] for row in blk]
        elif kind < 0.35:
            blk = [[rng.randint(0, mx) for _ in range(8)] for _ in range(8)]       # noise
        else:                                                                       # an oriented ramp / edge plus a little noise
            ang = rng.random() * math.pi; ca, sa = math.cos(ang), math.sin(ang)
            amp = rng.choice([4, 20, 60, 200]) << cs; base = rng.randint(0, mx); nz = rng.choice([0, 1, 3]) << cs
            sharp = rng.random() < 0.5
            blk = []
            for i in range(8):
                row = []
                for j in range(8):
                    t = (j - 3.5) * ca + (i - 3.5) * sa
                    v = base + int(amp * ((1 if t > 0 else -1) if sharp else t / 4)) + rng.randint(-nz, nz)
                    row.append(max(0, min(mx, v)))
                blk.append(row)
        hdr = types.SimpleNamespace(BitDepth=bd, cdef_y_strengths=[0], cdef_uv_strengths=[0])
        d, v = cdm.Cdef(hdr, None, [blk]).direction(0, 0)
        dut.v_in.value = 1; dut.cs.value = cs
        dut.px.value = sum(blk[i][j] << (12 * (8 * i + j)) for i in range(8) for j in range(8))
        expect.append(((d, v), f"block {it} bd{bd}"))
        stats["blocks"] += 1; stats["var0"] += v == 0; stats["dirs"][d] = stats["dirs"].get(d, 0) + 1
        await RisingEdge(dut.clk); await Timer(1, "ns")
        if rng.random() < 0.2:
            dut.v_in.value = 0
            await RisingEdge(dut.clk); await Timer(1, "ns")
    dut.v_in.value = 0
    await ClockCycles(dut.clk, 8)
    assert not expect and seen[0] == stats["blocks"], f"{seen[0]} results for {stats['blocks']} blocks"
    assert len(stats["dirs"]) == 8 and min(stats["dirs"].values()) > 50 and stats["var0"] > 50, stats
    dut._log.info(f"OK: {stats}")
