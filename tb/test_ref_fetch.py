"""Fuzz the reference fetch (rtl/ref_fetch.sv) against the region it must deliver: random reference planes in the
AXI memory model (tb/axi_mem.py, read channels with random stalls), both sample formats, unit positions crossing every
picture edge, sizes 2 .. 64. The patch writes are captured and compared sample by sample with 7.11.3.4's clamped
region (edge replication). Env: RF_ITERS (default 150), RF_SEED.
"""
import os
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, Timer

from axi_mem import AxiMem
import intra_model as im

SIZES = [2, 4, 8, 16, 32, 64]


@cocotb.test()
async def ref_fetch_regions(dut):
    iters = int(os.environ.get("RF_ITERS", "150"))
    seed = int(os.environ.get("RF_SEED", "1"))
    rng = random.Random(seed)
    cocotb.start_soon(Clock(dut.clk, 10, "ns").start())
    mem = AxiMem(dut, dut.clk, seed=seed, stall=0.3)
    cocotb.start_soon(mem.run_read())
    dut.rst.value = 1
    dut.req_valid.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    stats = {"units": 0, "words": 0, "clamped": 0, "fmt16": 0}
    for it in range(iters):
        fmt16 = rng.random() < 0.5
        bd = 12 if fmt16 else 8
        mx = (1 << bd) - 1
        W, H = rng.randint(1, 90), rng.randint(1, 90)
        bps = 2 if fmt16 else 1
        stride = ((W * bps + 7) // 8) * 8 + rng.choice([0, 8, 64])
        base = rng.randrange(0, 1 << 20, 8) + (rng.choice([0, 1 << 32]))     # exercise 40-bit addresses
        pic = [[rng.randint(0, mx) for _ in range(W)] for _ in range(H)]
        mem.mem.clear()
        for y in range(H):
            row = bytearray()
            for v in pic[y]:
                row += v.to_bytes(bps, "little")
            mem.load(base + y * stride, bytes(row))
        w, h = rng.choice(SIZES), rng.choice(SIZES)
        x0 = rng.randint(-12, W + 4) if rng.random() < 0.6 else rng.randint(0, max(0, W - w))
        y0 = rng.randint(-12, H + 4) if rng.random() < 0.6 else rng.randint(0, max(0, H - h))
        lst = rng.randint(0, 1)
        ngrp = (w + 3) >> 2
        if x0 - 3 < 0 or x0 - 3 + 4 * (ngrp + 2) > W or y0 - 3 < 0 or y0 + h + 4 > H:
            stats["clamped"] += 1
        stats["fmt16"] += int(fmt16)
        # request
        dut.req_lst.value = lst; dut.req_fmt16.value = int(fmt16); dut.req_base.value = base; dut.req_stride.value = stride
        dut.req_lastx.value = W - 1; dut.req_lasty.value = H - 1
        dut.req_x0.value = x0; dut.req_y0.value = y0; dut.req_w.value = w; dut.req_h.value = h
        dut.req_valid.value = 1
        await RisingEdge(dut.clk)
        await Timer(1, "ns")
        dut.req_valid.value = 0
        got = {}
        cycles = 0
        while True:
            await RisingEdge(dut.clk)
            cycles += 1
            await ReadOnly()
            if int(dut.patch_we.value):
                key = (int(dut.patch_list.value), int(dut.patch_row.value), int(dut.patch_w4.value))
                assert key not in got, f"iter {it}: patch word {key} written twice"
                got[key] = int(dut.patch_d4.value)
            if int(dut.done.value):
                break
            await Timer(1, "ns")
            assert cycles < 200000, f"iter {it}: no done"
        await Timer(1, "ns")
        assert not mem.errors, f"iter {it}: AXI errors {mem.errors[:3]}"
        assert int(dut.err.value) == 0, f"iter {it}: err set"
        # expected
        exp = {}
        for r in range(h + 7):
            srow = pic[im.clip3(0, H - 1, y0 - 3 + r)]
            for w4 in range(ngrp + 2):
                v = 0
                for l in range(4):
                    v |= srow[im.clip3(0, W - 1, x0 - 3 + 4 * w4 + l)] << (12 * l)
                exp[(lst, r, w4)] = v
        missing = [k for k in exp if k not in got]
        extra = [k for k in got if k not in exp]
        assert not missing and not extra, f"iter {it}: {w}x{h} at ({x0},{y0}) in {W}x{H}: missing {missing[:4]} extra {extra[:4]}"
        for k in exp:
            if got[k] != exp[k]:
                gl = [(got[k] >> (12 * l)) & 0xFFF for l in range(4)]
                el = [(exp[k] >> (12 * l)) & 0xFFF for l in range(4)]
                raise AssertionError(f"iter {it}: {w}x{h} at ({x0},{y0}) in {W}x{H} fmt16={fmt16} stride {stride}: word {k} rtl {gl} exp {el}")
        stats["units"] += 1
        stats["words"] += len(exp)
    dut._log.info(f"OK: {stats} read bursts {getattr(mem, 'read_bursts', 0)} beats {getattr(mem, 'read_beats', 0)}")
