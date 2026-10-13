"""Fuzz the level-folded 1D inverse transform engine (rtl/itx_lvl.sv) against the spec model.

Every program (DCT4 x16, DCT8 x8, DCT16 x4, DCT32 x2, DCT64, ADST8 x8, ADST16 x4) with random packed rows and
realistic clamp ranges; every row of the packed vector is checked. Env: ITX_FUZZ_ITERS per program (default
200), ITX_FUZZ_SEED.
"""
import os
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, Timer, ClockCycles

import itx_model as m

TW = int(os.environ.get("ITX_TW", "20"))
MASK = (1 << TW) - 1
PROGRAMS = [(0, 2, 0), (1, 3, 0), (2, 4, 0), (3, 5, 0), (4, 6, 0), (5, 3, 1), (6, 4, 1)]   # (pid, n, cls)


def pack(vec):
    v = 0
    for i, x in enumerate(vec):
        v |= (x & MASK) << (i * TW)
    return v


def unpack(v, count):
    out = []
    for i in range(count):
        x = (v >> (i * TW)) & MASK
        if x >> (TW - 1):
            x -= 1 << TW
        out.append(x)
    return out


@cocotb.test()
async def fuzz_vs_model(dut):
    iters = int(os.environ.get("ITX_FUZZ_ITERS", "200"))
    rng = random.Random(int(os.environ.get("ITX_FUZZ_SEED", "1")))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.start.value = 0
    dut.in_vec.value = 0
    dut.prog.value = 0
    dut.r.value = 16
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)

    checked = 0
    cycles = {}
    for pid, n, cls in PROGRAMS:
        N = 1 << n
        copies = 64 // N
        good = 0
        while good < iters:
            r = rng.choice([16, 16, 18, 20])
            amp = rng.choice([50, 500, 3000, 20000])
            rows = [[rng.randint(-amp, amp) for _ in range(N)] for _ in range(copies)]
            if rng.random() < 0.3:                        # sparse, like real coefficient rows
                rows = [[x if rng.random() < 0.15 else 0 for x in row] for row in rows]
            try:
                exp = []
                for row in rows:
                    T = list(row)
                    (m.inv_dct if cls == 0 else m.inv_adst)(T, n, r)
                    exp += T
            except m.Nonconformant:
                continue
            vec = [x for row in rows for x in row]
            if any(not m.fits(x, TW) for x in exp) or any(not m.fits(x, TW) for x in vec):
                continue
            dut.in_vec.value = pack(vec)
            dut.prog.value = pid
            dut.r.value = r
            dut.start.value = 1
            await RisingEdge(dut.clk)
            await Timer(1, "ns")
            dut.start.value = 0
            dut.in_vec.value = 0                          # level 0 must have consumed it in the start cycle
            ncyc = 0
            while True:
                await RisingEdge(dut.clk)
                ncyc += 1
                await ReadOnly()
                if int(dut.done.value) == 1:
                    got = unpack(int(dut.out_vec.value), 64)
                    break
                await Timer(1, "ns")
                assert ncyc < 40, f"pid {pid}: no done after {ncyc} cycles"
            await Timer(1, "ns")
            assert got == exp, (f"pid {pid} (n={n} cls={cls} r={r})\n rows={rows}\n exp={exp}\n got={got}")
            cycles[pid] = ncyc
            good += 1
            checked += 1
    dut._log.info(f"OK: {checked} packed vectors across {len(PROGRAMS)} programs match the spec model; "
                  f"cycles per program: {cycles}")
