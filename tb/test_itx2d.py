"""Fuzz the 2D inverse transform block against the spec model, over all 19 transform sizes and the
transform types the spec allows for each. Env: ITX2D_ITERS per size (default 6), ITX2D_SEED.
"""
import os
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, Timer, ClockCycles

import itx_model as m

TW = int(os.environ.get("ITX_TW", "20"))
MASK = (1 << TW) - 1


def allowed_types(tx_sz):
    lw, lh = m.TX_WIDTH_LOG2[tx_sz], m.TX_HEIGHT_LOG2[tx_sz]
    types = []
    for t in range(16):
        row_cls = 0 if t in m.ROW_DCT else 1 if t in m.ROW_ADST else 2
        col_cls = 0 if t in m.COL_DCT else 1 if t in m.COL_ADST else 2
        ok = True
        if row_cls == 1 and lw > 4: ok = False      # ADST exists up to 16
        if col_cls == 1 and lh > 4: ok = False
        if row_cls == 2 and lw > 5: ok = False      # identity up to 32
        if col_cls == 2 and lh > 5: ok = False
        if ok:
            types.append(t)
    return types


def to_signed(x):
    return x - (1 << TW) if x >> (TW - 1) else x


@cocotb.test()
async def fuzz_vs_model(dut):
    iters = int(os.environ.get("ITX2D_ITERS", "6"))
    rng = random.Random(int(os.environ.get("ITX2D_SEED", "1")))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.start.value = 0
    dut.coef_we.value = 0
    dut.coef_addr.value = 0
    dut.coef_data.value = 0
    dut.res_addr.value = 0
    dut.tx_sz.value = 0
    dut.tx_type.value = 0
    dut.bit_depth.value = 8
    dut.lossless.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)

    checked = 0
    for tx_sz in range(19):
        lw, lh = m.TX_WIDTH_LOG2[tx_sz], m.TX_HEIGHT_LOG2[tx_sz]
        w, h = 1 << lw, 1 << lh
        types = allowed_types(tx_sz)
        good = 0
        while good < iters:
            lossless = (tx_sz == 0 and rng.random() < 0.25)
            tx_type = 0 if lossless else rng.choice(types)
            bd = rng.choice([8, 10, 10, 12] if TW >= 22 else [8, 10, 10])
            lim = (1 << (7 + bd)) - 1
            amp = rng.choice([30, 300, 3000, lim])
            nz_w, nz_h = min(w, 32), min(h, 32)
            deq = [[0] * w for _ in range(h)]
            density = rng.choice([0.05, 0.3, 1.0])
            for i in range(nz_h):
                for j in range(nz_w):
                    if rng.random() < density:
                        deq[i][j] = max(-lim - 1, min(lim, rng.randint(-amp, amp)))
            if lossless:
                for i in range(4):
                    for j in range(4):
                        deq[i][j] = rng.randint(-600, 600)
            try:
                exp = m.inverse_transform_2d(deq, tx_sz, tx_type, bd, lossless)
            except m.Nonconformant:
                continue
            if any(not m.fits(v, TW) for row in exp for v in row):
                continue

            # load coefficients (only the non-zero 32x32 corner exists in hardware)
            for i in range(nz_h):
                for j in range(nz_w):
                    dut.coef_we.value = 1
                    dut.coef_addr.value = i * 32 + j
                    dut.coef_data.value = deq[i][j] & MASK
                    await RisingEdge(dut.clk)
            dut.coef_we.value = 0
            await Timer(1, "ns")
            dut.tx_sz.value = tx_sz
            dut.tx_type.value = tx_type
            dut.bit_depth.value = bd
            dut.lossless.value = 1 if lossless else 0
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
                    break
                await Timer(1, "ns")
                assert ncyc < 20000, f"tx_sz {tx_sz}: no done after {ncyc} cycles"
            await Timer(1, "ns")
            got = [[0] * w for _ in range(h)]
            for i in range(h):
                for j in range(w):
                    dut.res_addr.value = i * 64 + j
                    await Timer(1, "ns")
                    got[i][j] = to_signed(int(dut.res_data.value))
            if got != exp:
                bad = [(i, j, got[i][j], exp[i][j]) for i in range(h) for j in range(w) if got[i][j] != exp[i][j]]
                raise AssertionError(f"tx_sz {tx_sz} ({w}x{h}) type {tx_type} bd {bd} lossless {lossless}: "
                                     f"{len(bad)} mismatches, first {bad[:6]}")
            good += 1
            checked += 1
            dut._log.info(f"tx_sz {tx_sz:2d} {w:2d}x{h:2d} type {tx_type:2d} bd {bd} ll {int(lossless)} ok in {ncyc} cycles")
    dut._log.info(f"OK: {checked} blocks across 19 sizes match the spec model")
