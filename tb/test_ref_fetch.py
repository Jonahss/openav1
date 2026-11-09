"""Fuzz the reference fetch (rtl/ref_fetch.sv) against the region it must deliver: random reference planes in the
AXI memory model (tb/axi_mem.py, read channels with random stalls), both sample formats, unit positions crossing every
picture edge, sizes 2 .. 64. The patch writes are captured and compared sample by sample with 7.11.3.4's clamped
region (edge replication). Env: RF_ITERS (default 150), RF_SEED.

ref_fetch_pipelined (2026-11-09, step C1 of docs/reference-prefetch.md section 8): requests issued back to back, each
accepted as soon as the engine's address side is idle, while the previous requests' rows are still in the ring or
being written; the patch buffer of each request alternates and buf_busy is driven randomly. Checks: every request's
words land, in request order, tagged with the request's buffer, no row of a busy buffer is taken, one done per request
in order. Env: RF_PIPE_REQS (default 400).
"""
import os
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, Timer

from axi_mem import AxiMem
import intra_model as im

SIZES = [2, 4, 8, 16, 32, 64]


def expected_words(pic, W, H, x0, y0, w, h, lst):
    ngrp = (w + 3) >> 2
    exp = {}
    for r in range(h + 7):
        srow = pic[im.clip3(0, H - 1, y0 - 3 + r)]
        for w4 in range(ngrp + 2):
            v = 0
            for l in range(4):
                v |= srow[im.clip3(0, W - 1, x0 - 3 + 4 * w4 + l)] << (12 * l)
            exp[(lst, r, w4)] = v
    return exp


def random_picture(rng, mem, fmt16):
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
    return pic, W, H, base, stride


def random_unit(rng, W, H):
    w, h = rng.choice(SIZES), rng.choice(SIZES)
    x0 = rng.randint(-12, W + 4) if rng.random() < 0.6 else rng.randint(0, max(0, W - w))
    y0 = rng.randint(-12, H + 4) if rng.random() < 0.6 else rng.randint(0, max(0, H - h))
    if os.environ.get("RF_FAR"):
        # windows far outside the picture (Argon: test10505 frame 1 starts 35 px beyond the right edge of a
        # scaled reference): every sample clamps to an edge column / row
        x0 = rng.choice([rng.randint(W + 5, W + 90), rng.randint(-90, -13), x0])
        y0 = rng.choice([rng.randint(H + 5, H + 90), rng.randint(-90, -13), y0])
    return w, h, x0, y0


def drive_request(dut, fmt16, base, stride, W, H, x0, y0, w, h, lst, buf):
    dut.req_lst.value = lst; dut.req_fmt16.value = int(fmt16); dut.req_base.value = base; dut.req_stride.value = stride
    dut.req_lastx.value = W - 1; dut.req_lasty.value = H - 1
    dut.req_x0.value = x0; dut.req_y0.value = y0; dut.req_w.value = w; dut.req_h.value = h
    dut.req_r_lo.value = 0; dut.req_r_hi.value = 0
    dut.req_buf.value = buf


async def reset(dut):
    dut.rst.value = 1
    dut.req_valid.value = 0
    dut.req_buf.value = 0
    dut.buf_busy.value = 0
    dut.req_r_lo.value = 0
    dut.req_r_hi.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)


@cocotb.test()
async def ref_fetch_regions(dut):
    iters = int(os.environ.get("RF_ITERS", "150"))
    seed = int(os.environ.get("RF_SEED", "1"))
    rng = random.Random(seed)
    cocotb.start_soon(Clock(dut.clk, 10, "ns").start())
    mem = AxiMem(dut, dut.clk, seed=seed, stall=0.3)
    cocotb.start_soon(mem.run_read())
    await reset(dut)
    stats = {"units": 0, "words": 0, "clamped": 0, "fmt16": 0}
    for it in range(iters):
        fmt16 = rng.random() < 0.5
        pic, W, H, base, stride = random_picture(rng, mem, fmt16)
        w, h, x0, y0 = random_unit(rng, W, H)
        lst = rng.randint(0, 1)
        buf = rng.randint(0, 1)
        ngrp = (w + 3) >> 2
        if x0 - 3 < 0 or x0 - 3 + 4 * (ngrp + 2) > W or y0 - 3 < 0 or y0 + h + 4 > H:
            stats["clamped"] += 1
        stats["fmt16"] += int(fmt16)
        # request
        drive_request(dut, fmt16, base, stride, W, H, x0, y0, w, h, lst, buf)
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
                assert int(dut.patch_buf.value) == buf, f"iter {it}: patch write names buffer {int(dut.patch_buf.value)}, request said {buf}"
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
        exp = expected_words(pic, W, H, x0, y0, w, h, lst)
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


@cocotb.test()
async def ref_fetch_pipelined(dut):
    nreq = int(os.environ.get("RF_PIPE_REQS", "400"))
    seed = int(os.environ.get("RF_SEED", "1")) + 1000
    rng = random.Random(seed)
    cocotb.start_soon(Clock(dut.clk, 10, "ns").start())
    mem = AxiMem(dut, dut.clk, seed=seed, stall=0.3)
    cocotb.start_soon(mem.run_read())
    await reset(dut)
    stats = {"groups": 0, "units": 0, "words": 0, "accept_early": 0, "cycles": 0}
    k = 0
    while k < nreq:
        # one picture per group, several requests on it issued back to back (one memory image at a time)
        fmt16 = rng.random() < 0.5
        pic, W, H, base, stride = random_picture(rng, mem, fmt16)
        n = min(rng.randint(2, 12), nreq - k)
        reqs = []
        for i in range(n):
            w, h, x0, y0 = random_unit(rng, W, H)
            lst = rng.randint(0, 1)
            buf = (k + i) & 1 if rng.random() < 0.8 else rng.randint(0, 1)
            reqs.append((w, h, x0, y0, lst, buf))
        # the issuer: present the next request the cycle after the previous one was accepted
        state = {"accepted": 0, "dones": 0}

        async def issuer():
            for (w, h, x0, y0, lst, buf) in reqs:
                drive_request(dut, fmt16, base, stride, W, H, x0, y0, w, h, lst, buf)
                dut.req_valid.value = 1
                while True:
                    await RisingEdge(dut.clk)
                    await ReadOnly()
                    acc = int(dut.req_valid.value) and int(dut.req_ready.value)
                    await Timer(1, "ns")
                    if acc:
                        if state["accepted"] > state["dones"]:
                            stats["accept_early"] += 1       # accepted while an earlier request had not pulsed done yet
                        state["accepted"] += 1
                        break
            dut.req_valid.value = 0

        # buf_busy: each bit flips with a small probability a cycle (holds a buffer for a few dozen cycles at a time)
        stop = {"v": False}

        async def busy_driver():
            b = 0
            while not stop["v"]:
                await RisingEdge(dut.clk)
                await Timer(2, "ns")
                for bit in (0, 1):
                    if rng.random() < 0.03:
                        b ^= 1 << bit
                dut.buf_busy.value = b

        cocotb.start_soon(issuer())
        cocotb.start_soon(busy_driver())
        got = [dict() for _ in reqs]
        dones = 0
        cycles = 0
        while dones < n:
            await RisingEdge(dut.clk)
            cycles += 1
            await ReadOnly()
            busy_now = int(dut.buf_busy.value)
            if int(dut.patch_we.value):
                pb = int(dut.patch_buf.value)
                assert dones < n, f"group {stats['groups']}: patch write after the last done"
                if int(dut.patch_w4.value) == 0:
                    # the row was taken at this edge (wr_busy rises with the first word): the busy value the engine saw at
                    # that edge is the one set after the previous edge, i.e. the value read now
                    assert not (busy_now >> pb) & 1, f"group {stats['groups']} request {dones}: row taken for buffer {pb} while busy"
                assert pb == reqs[dones][5], f"group {stats['groups']} request {dones}: buffer {pb}, request said {reqs[dones][5]}"
                key = (int(dut.patch_list.value), int(dut.patch_row.value), int(dut.patch_w4.value))
                assert key not in got[dones], f"group {stats['groups']} request {dones}: patch word {key} written twice"
                got[dones][key] = int(dut.patch_d4.value)
            if int(dut.done.value):
                dones += 1
                state["dones"] = dones
            await Timer(1, "ns")
            assert cycles < 400000, f"group {stats['groups']}: no done for request {dones} after {cycles} cycles"
        stop["v"] = True
        dut.buf_busy.value = 0
        await Timer(1, "ns")
        assert state["accepted"] == n, f"group {stats['groups']}: {state['accepted']} of {n} requests accepted"
        assert not mem.errors, f"group {stats['groups']}: AXI errors {mem.errors[:3]}"
        assert int(dut.err.value) == 0, f"group {stats['groups']}: err set"
        for i, (w, h, x0, y0, lst, buf) in enumerate(reqs):
            exp = expected_words(pic, W, H, x0, y0, w, h, lst)
            missing = [kk for kk in exp if kk not in got[i]]
            extra = [kk for kk in got[i] if kk not in exp]
            assert not missing and not extra, (f"group {stats['groups']} request {i}: {w}x{h} at ({x0},{y0}) in {W}x{H} buf {buf}: "
                                               f"missing {missing[:4]} extra {extra[:4]}")
            for kk in exp:
                if got[i][kk] != exp[kk]:
                    gl = [(got[i][kk] >> (12 * l)) & 0xFFF for l in range(4)]
                    el = [(exp[kk] >> (12 * l)) & 0xFFF for l in range(4)]
                    raise AssertionError(f"group {stats['groups']} request {i}: {w}x{h} at ({x0},{y0}) in {W}x{H} fmt16={fmt16}: word {kk} rtl {gl} exp {el}")
            stats["units"] += 1
            stats["words"] += len(exp)
        stats["groups"] += 1
        stats["cycles"] += cycles
        k += n
        # a few idle cycles between pictures (the memory image changes)
        for _ in range(4):
            await RisingEdge(dut.clk)
    assert stats["accept_early"] > 0, "no request was ever accepted while the previous one's rows were in flight"
    dut._log.info(f"OK: {stats} read bursts {getattr(mem, 'read_bursts', 0)} beats {getattr(mem, 'read_beats', 0)}")
