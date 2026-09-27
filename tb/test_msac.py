"""Replay a dav1d golden trace (v2, mask=1) through the msac RTL and check every symbol.

Env: MSAC_TRACE=<trace file> (from tools/trace.sh <stream> <out> 1).  MSAC_MAX_SYMS caps the run.
"""
import os
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, Timer, ClockCycles

KIND_ADAPT, KIND_BOOL, KIND_EQUI = 0, 1, 2


def parse_trace(path):
    """Yield tiles one at a time as (data, disable_cdf_update, requests).

    Streams the file so a million-symbol trace does not need gigabytes of RAM.
    A request is the tuple (kind, n, cdf_tuple, cnt, exp_sym, exp_rng, post_tuple_or_None, postcnt_or_None).
    """
    data = dis = None
    reqs = []
    pending = None  # an "S B" waiting to see whether a "U" line follows
    with open(path) as fh:
        for line in fh:
            if not line or line[0] == "#":
                continue
            f = line.split()
            if f[0] == "U":
                if pending is not None:
                    k, n, cdf, cnt, sym, rng, _, _ = pending
                    pending = (KIND_ADAPT, 1, cdf, int(f[1]), sym, rng, (int(f[2]),), int(f[3]))
                continue
            if pending is not None:
                reqs.append(pending); pending = None
            if f[0] == "T":
                if data is not None:
                    yield data, dis, reqs
                data = bytes.fromhex(f[3]) if len(f) > 3 else b""
                assert len(data) == int(f[1])
                dis = int(f[2]); reqs = []
            elif f[0] == "S":
                assert data is not None, "S before T"
                if f[1] == "A":
                    n, sym, rng, cnt = int(f[2]), int(f[3]), int(f[4]), int(f[5])
                    pre = tuple(int(x) for x in f[6:6 + n])
                    post = postcnt = None
                    if not dis:
                        post = tuple(int(x) for x in f[6 + n:6 + 2 * n]); postcnt = int(f[6 + 2 * n])
                    reqs.append((KIND_ADAPT, n, pre, cnt, sym, rng, post, postcnt))
                elif f[1] == "B":
                    pending = (KIND_BOOL, 1, (int(f[2]),), 0, int(f[3]), int(f[4]), None, None)
                elif f[1] == "E":
                    reqs.append((KIND_EQUI, 1, (), 0, int(f[2]), int(f[3]), None, None))
    if pending is not None:
        reqs.append(pending)
    if data is not None:
        yield data, dis, reqs


async def feed_bytes(dut, data):
    dut.in_eos.value = 0
    for b in data:
        dut.in_data.value = b
        dut.in_valid.value = 1
        while True:
            await ReadOnly()
            ok = int(dut.in_ready.value) == 1
            await RisingEdge(dut.clk)
            if ok:
                break
        await Timer(1, "ns")
    dut.in_valid.value = 0
    dut.in_eos.value = 1


def pack_cdf(cdf):
    v = 0
    for k, c in enumerate(cdf):
        v |= (c & 0xFFFF) << (16 * k)
    return v


@cocotb.test()
async def replay_trace(dut):
    path = os.environ["MSAC_TRACE"]
    max_syms = int(os.environ.get("MSAC_MAX_SYMS", "0")) or None
    dut._log.info(f"replaying {path}")

    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.init.value = 0
    dut.in_valid.value = 0
    dut.in_eos.value = 0
    dut.req_valid.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)

    done = ntiles = 0
    for ti, (data, dis, reqs) in enumerate(parse_trace(path)):
        ntiles += 1
        dut.cdf_update_en.value = 0 if dis else 1
        dut.init.value = 1
        await RisingEdge(dut.clk)
        await Timer(1, "ns")
        dut.init.value = 0
        feeder = cocotb.start_soon(feed_bytes(dut, data))

        for ri, (kind, n, cdf, cnt, exp_sym, exp_rng, post, postcnt) in enumerate(reqs):
            dut.req_kind.value = kind
            dut.req_n.value = n
            dut.req_cdf.value = pack_cdf(cdf)
            dut.req_cnt.value = cnt
            dut.req_valid.value = 1
            while True:
                await ReadOnly()
                ok = int(dut.req_ready.value) == 1
                await RisingEdge(dut.clk)
                if ok:
                    break
            await ReadOnly()
            assert int(dut.resp_valid.value) == 1, f"tile {ti} sym {ri}: no response"
            got_sym = int(dut.resp_sym.value)
            got_rng = int(dut.resp_rng.value)
            where = f"tile {ti} req {ri} (kind={kind},n={n} cdf={cdf} cnt={cnt})"
            assert got_sym == exp_sym, f"{where}: symbol {got_sym} != expected {exp_sym}"
            assert got_rng == exp_rng, f"{where}: rng {got_rng} != expected {exp_rng}"
            if post is not None:
                got_cdf = int(dut.resp_cdf.value)
                got = tuple((got_cdf >> (16 * k)) & 0xFFFF for k in range(n))
                assert got == post, f"{where}: cdf after update {got} != expected {post}"
                assert int(dut.resp_cnt.value) == postcnt, f"{where}: cnt {int(dut.resp_cnt.value)} != {postcnt}"
            await Timer(1, "ns")
            dut.req_valid.value = 0
            done += 1
            if done % 200000 == 0:
                dut._log.info(f"... {done} symbols")
            if max_syms and done >= max_syms:
                break
        if max_syms and done >= max_syms:
            break
        if not feeder.done():
            feeder.cancel()
        dut.in_valid.value = 0
        dut.in_eos.value = 0
        await ClockCycles(dut.clk, 2)
    dut._log.info(f"OK: {done} symbols matched across {ntiles} tiles")
