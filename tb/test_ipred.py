"""Fuzz the intra predictor RTL against tb/intra_model.py.

Random edges, all modes (incl. filter-intra), all angle deltas, block sizes 4..64 (aspect <= 4:1),
8/10-bit, random availability and edge-filter settings. Env: IPRED_ITERS (default 300), IPRED_SEED.
"""
import os
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, Timer, ClockCycles

import intra_model as im


async def load_edges(dut, above, left, tl):
    for idx, v in enumerate(above):
        dut.edge_we.value = 1; dut.edge_side.value = 0; dut.edge_idx.value = idx; dut.edge_data.value = v
        await RisingEdge(dut.clk)
    for idx, v in enumerate(left):
        dut.edge_we.value = 1; dut.edge_side.value = 1; dut.edge_idx.value = idx; dut.edge_data.value = v
        await RisingEdge(dut.clk)
    dut.edge_we.value = 1; dut.edge_side.value = 2; dut.edge_idx.value = 0; dut.edge_data.value = tl
    await RisingEdge(dut.clk)
    dut.edge_we.value = 0
    await Timer(1, "ns")


async def run_block(dut, params, w, h):
    for k, v in params.items():
        getattr(dut, k).value = v
    dut.start.value = 1
    await RisingEdge(dut.clk)
    await Timer(1, "ns")
    dut.start.value = 0
    got = [[None] * w for _ in range(h)]
    n = 0
    cycles = 0
    while True:
        await RisingEdge(dut.clk)
        cycles += 1
        await ReadOnly()
        if int(dut.out_valid.value):
            x, y = int(dut.out_x.value), int(dut.out_y.value)
            got[y][x] = int(dut.out_pix.value)
            n += 1
        if int(dut.done.value):
            break
        await Timer(1, "ns")
        assert cycles < 20000, "no done"
    await Timer(1, "ns")
    assert n == w * h, f"got {n} pixels, expected {w*h}"
    return got, cycles


def gen_case(rng):
    log2w, log2h = rng.randint(2, 6), rng.randint(2, 6)
    while abs(log2w - log2h) > 2:
        log2h = rng.randint(2, 6)
    w, h = 1 << log2w, 1 << log2h
    bd = rng.choice([8, 10])
    mx = (1 << bd) - 1
    style = rng.random()
    if style < 0.3:      # smooth ramp + noise
        base = rng.randint(0, mx); slope = rng.uniform(-3, 3)
        above = [max(0, min(mx, int(base + slope * i + rng.gauss(0, 3)))) for i in range(w + h)]
        left = [max(0, min(mx, int(base + slope * 0.7 * i + rng.gauss(0, 3)))) for i in range(w + h)]
        tl = max(0, min(mx, int(base + rng.gauss(0, 3))))
    elif style < 0.6:    # random
        above = [rng.randint(0, mx) for _ in range(w + h)]
        left = [rng.randint(0, mx) for _ in range(w + h)]
        tl = rng.randint(0, mx)
    else:                # extremes
        vals = [0, mx, mx // 2, 1, mx - 1]
        above = [rng.choice(vals) for _ in range(w + h)]
        left = [rng.choice(vals) for _ in range(w + h)]
        tl = rng.choice(vals)
    use_fi = (rng.random() < 0.12) and log2w <= 5 and log2h <= 5
    mode = rng.randint(0, 12)
    delta = rng.randint(-3, 3) if im.is_directional(mode) else 0
    fimode = rng.randint(0, 4)
    have_left, have_above = rng.choice([(1, 1), (1, 1), (1, 1), (1, 0), (0, 1), (0, 0)])
    # spec edge replication when a side is unavailable (the caller's job; mirror it here)
    if not have_above and have_left:
        above = [left[0]] * (w + h); tl = left[0]
    elif have_above and not have_left:
        left = [above[0]] * (w + h); tl = above[0]
    elif not have_above and not have_left:
        above = [(1 << (bd - 1)) - 1] * (w + h); left = [(1 << (bd - 1)) + 1] * (w + h); tl = 1 << (bd - 1)
    ft = rng.randint(0, 1)
    efe = rng.choice([1, 1, 1, 0])
    apx = rng.choice([w, w, w, max(4, w - 4 * rng.randint(1, w // 4))])
    lpx = rng.choice([h, h, h, max(4, h - 4 * rng.randint(1, h // 4))])
    return dict(log2w=log2w, log2h=log2h, w=w, h=h, bd=bd, above=above, left=left, tl=tl, use_fi=use_fi,
                mode=mode, delta=delta, fimode=fimode, have_left=have_left, have_above=have_above,
                ft=ft, efe=efe, apx=apx, lpx=lpx)


@cocotb.test()
async def fuzz_vs_model(dut):
    iters = int(os.environ.get("IPRED_ITERS", "300"))
    rng = random.Random(int(os.environ.get("IPRED_SEED", "1")))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.start.value = 0
    dut.edge_we.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    from collections import Counter
    seen = Counter()
    for it in range(iters):
        c = gen_case(rng)
        exp = im.predict_intra(c["above"], c["left"], c["tl"], c["mode"], c["log2w"], c["log2h"], c["bd"],
                               have_left=c["have_left"], have_above=c["have_above"], angle_delta=c["delta"],
                               enable_intra_edge_filter=c["efe"], filter_type=c["ft"],
                               use_filter_intra=c["use_fi"], filter_intra_mode=c["fimode"],
                               above_px=c["apx"], left_px=c["lpx"])
        await load_edges(dut, c["above"], c["left"], c["tl"])
        params = dict(mode=c["mode"], use_filter_intra=int(c["use_fi"]), filter_intra_mode=c["fimode"],
                      angle_delta=c["delta"] & 7, log2w=c["log2w"], log2h=c["log2h"], bit_depth=c["bd"],
                      have_left=c["have_left"], have_above=c["have_above"], filter_type=c["ft"],
                      edge_filter_en=c["efe"], above_px=c["apx"], left_px=c["lpx"])
        got, cycles = await run_block(dut, params, c["w"], c["h"])
        tag = "fi%d" % c["fimode"] if c["use_fi"] else ("m%d/%+d" % (c["mode"], c["delta"]) if im.is_directional(c["mode"]) else "m%d" % c["mode"])
        if got != exp:
            bad = [(i, j, got[i][j], exp[i][j]) for i in range(c["h"]) for j in range(c["w"]) if got[i][j] != exp[i][j]]
            raise AssertionError(f"case {it} {tag} {c['w']}x{c['h']} bd{c['bd']} hl{c['have_left']} ha{c['have_above']} "
                                 f"ft{c['ft']} efe{c['efe']} apx{c['apx']} lpx{c['lpx']}: {len(bad)} bad, first {bad[:8]}\n"
                                 f"above={c['above'][:12]}.. left={c['left'][:12]}.. tl={c['tl']}")
        seen[tag] += 1
    dut._log.info(f"OK: {iters} blocks match the model; kinds: {dict(seen)}")
