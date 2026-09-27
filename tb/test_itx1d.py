"""Fuzz the 1D inverse transform engine against the spec model (tb/itx_model.py).

Every program (DCT 4..64, ADST 4..16, identity 4..32, WHT) with random vectors and realistic
clamp ranges. Env: ITX_FUZZ_ITERS per program (default 150), ITX_FUZZ_SEED.
"""
import os
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, Timer, ClockCycles

import itx_model as m

TW = 20
MASK = (1 << TW) - 1

PROGRAMS = [(pid, n, cls) for pid, (n, cls) in enumerate(
    [(n, 0) for n in range(2, 7)] + [(n, 1) for n in range(2, 5)] + [(n, 2) for n in range(2, 6)] + [(2, 3)])]


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


def model_run(vec, n, cls, r, shift):
    T = list(vec)
    if cls == 0:
        m.inv_dct(T, n, r)
    elif cls == 1:
        m.inv_adst(T, n, r)
    elif cls == 2:
        m.inv_identity(T, n)
    else:
        m.inv_wht(T, shift)
    return T


@cocotb.test()
async def fuzz_vs_model(dut):
    iters = int(os.environ.get("ITX_FUZZ_ITERS", "150"))
    rng = random.Random(int(os.environ.get("ITX_FUZZ_SEED", "1")))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.start.value = 0
    dut.in_vec.value = 0
    dut.prog.value = 0
    dut.n.value = 2
    dut.r.value = 16
    dut.wht_shift.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)

    checked = 0
    cycles = {}
    for pid, n, cls in PROGRAMS:
        N = 1 << n
        good = 0
        while good < iters:
            r = rng.choice([16, 16, 18, 20])
            shift = rng.choice([0, 2])
            amp = rng.choice([50, 500, 3000, 20000])
            vec = [rng.randint(-amp, amp) for _ in range(N)]
            if rng.random() < 0.2:                        # sparse, like real coefficient rows
                vec = [x if rng.random() < 0.15 else 0 for x in vec]
            if cls == 3 and rng.random() < 0.3:           # lossless coefficients are small
                vec = [rng.randint(-1024, 1023) for _ in range(N)]
            try:
                exp = model_run(vec, n, cls, r, shift)
            except m.Nonconformant:
                continue
            if any(not m.fits(x, TW) for x in exp) or any(not m.fits(x, TW) for x in vec):
                continue
            dut.in_vec.value = pack(vec + [0] * (64 - N))
            dut.prog.value = pid
            dut.n.value = n
            dut.r.value = r
            dut.wht_shift.value = shift
            dut.start.value = 1
            await RisingEdge(dut.clk)
            await Timer(1, "ns")
            dut.start.value = 0
            ncyc = 0
            while True:
                await RisingEdge(dut.clk)
                ncyc += 1
                await ReadOnly()
                if int(dut.done.value) == 1:
                    got = unpack(int(dut.out_vec.value), N)
                    break
                await Timer(1, "ns")
                assert ncyc < 200, f"pid {pid}: no done after {ncyc} cycles"
            await Timer(1, "ns")
            assert got == exp, (f"pid {pid} (n={n} cls={cls} r={r} shift={shift})\n vec={vec}\n exp={exp}\n got={got}")
            cycles[pid] = ncyc
            good += 1
            checked += 1
    dut._log.info(f"OK: {checked} vectors across {len(PROGRAMS)} programs match the spec model; cycles per program: {cycles}")
