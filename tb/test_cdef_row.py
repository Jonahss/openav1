"""Fuzz the CDEF line filter (rtl/cdef_row.sv, N lanes) against cdef_model.Cdef.filter: random small pictures of
8x8 blocks, a random block (so the picture's borders, where taps are absent, come up often), random direction,
strengths, damping and bit depth; each of the block's eight lines goes through the module as N-sample pieces with
its 5 x (N + 4) neighbourhood of deblocked samples and their presence, and must equal the model's line.
The module's constrain() shifts and tap-weight bit are formed here as cdef_top forms them.
Env: CR_ITERS (blocks, default 600), CR_SEED, CR_N (lanes of the build: 8 default, 4)."""
import os
import random
import types
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge, Timer

import cdef_model as cdm

N = int(os.environ.get("CR_N", "8"))


def flog2(v):
    return v.bit_length() - 1


@cocotb.test()
async def cdef_row_vs_model(dut):
    iters = int(os.environ.get("CR_ITERS", "600"))
    rng = random.Random(int(os.environ.get("CR_SEED", "1")))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1; dut.v_in.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk); await Timer(1, "ns")
    stats = dict(blocks=0, lines=0, changed=0, absent_taps=0, pri0=0, sec0=0, dirs=set())
    expect = []                                              # (expected N samples, tag) in input order
    seen = [0]

    async def monitor():                                     # every v_out is the next line that went in
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
        bd = rng.choice([8, 10, 12]); cs = bd - 8; mx = (1 << bd) - 1
        mr, mc = rng.choice([2, 4, 6]), rng.choice([2, 4, 6])     # picture size in 4x4 units
        H, W = mr * 4, mc * 4
        base = rng.randint(0, mx); noise = rng.choice([1, 4, 16, 64, 255]) << cs
        src = [[max(0, min(mx, base + rng.randint(-noise, noise))) for _ in range(W)] for _ in range(H)]
        if rng.random() < 0.3:                               # an edge through the picture
            e = rng.randrange(W); step = rng.choice([-1, 1]) * rng.randint(8, 80) << cs
            src = [[max(0, min(mx, v + (step if x >= e else 0))) for x, v in enumerate(row)] for row in src]
        r, c = rng.randrange(0, mr, 2), rng.randrange(0, mc, 2)
        pri = rng.choice([0, 0, 1, 2, 3, 5, 9, 15]) << cs; sec = rng.choice([0, 1, 2, 4]) << cs
        damping = rng.randint(3, 6) + cs - rng.choice([0, 1]); dr = rng.randrange(8)
        hdr = types.SimpleNamespace(BitDepth=bd, subsampling_x=0, subsampling_y=0, MiRows=mr, MiCols=mc,
                                    cdef_y_strengths=[0], cdef_uv_strengths=[0])
        m = cdm.Cdef(hdr, None, [src])
        m.out = [[row[:] for row in src]]
        m.filter(0, r, c, pri, sec, damping, dr)
        adjp = max(0, damping - flog2(pri)) if pri else 0
        adjs = max(0, damping - flog2(sec)) if sec else 0
        priodd = (pri >> cs) & 1
        y0, x0 = r * 4, c * 4
        stats["blocks"] += 1; stats["dirs"].add(dr); stats["pri0"] += pri == 0; stats["sec0"] += sec == 0
        for i in range(8):
            for part in range(8 // N):
                xs = x0 + part * N
                nb = 0; ok = 0
                for rr in range(5):
                    for cc in range(N + 4):
                        yy, xx = y0 + i + rr - 2, xs + cc - 2
                        inside = 0 <= yy < H and 0 <= xx < W
                        if inside:
                            nb |= src[yy][xx] << (12 * (rr * (N + 4) + cc)); ok |= 1 << (rr * (N + 4) + cc)
                        else:
                            nb |= rng.getrandbits(12) << (12 * (rr * (N + 4) + cc))      # an absent sample's value must not matter
                            stats["absent_taps"] += 1
                dut.v_in.value = 1; dut.nb.value = nb; dut.nbok.value = ok; dut.dr.value = dr
                dut.pri.value = pri; dut.sec.value = sec; dut.adjp.value = adjp; dut.adjs.value = adjs; dut.priodd.value = priodd
                exp = m.out[0][y0 + i][xs:xs + N]
                expect.append((exp, f"block {it} bd{bd} {W}x{H} at ({r},{c}) line {i} part {part} dr {dr} pri {pri} sec {sec} damping {damping}"))
                stats["lines"] += 1; stats["changed"] += exp != src[y0 + i][xs:xs + N]
                await RisingEdge(dut.clk); await Timer(1, "ns")
                if rng.random() < 0.2:                        # gaps in the input
                    dut.v_in.value = 0
                    await RisingEdge(dut.clk); await Timer(1, "ns")
    dut.v_in.value = 0
    await ClockCycles(dut.clk, 8)
    assert not expect, f"{len(expect)} lines never came out"
    stats["dirs"] = len(stats["dirs"])
    assert seen[0] == stats["lines"], f"{seen[0]} lines came out, {stats['lines']} went in"
    assert stats["changed"] > stats["lines"] // 4 and stats["absent_taps"] > 1000 and stats["dirs"] == 8 and stats["pri0"] > 20 and stats["sec0"] > 20, stats
    dut._log.info(f"OK: {stats}")
