"""Fuzz the motion compensation filter (rtl/mc_filt.sv) against tb/recon_model.py's block_inter_prediction and the
compound blend of predict_inter (7.11.3.1 AVERAGE / DISTANCE).

Random reference pictures, unit positions that cross every picture edge (the testbench does ref_fetch's job: it
writes the clamped patch), all interpolation filters and phases, unit sizes 2 .. 64 (chroma of a 4xN luma block
predicts 2-wide units), 8 / 10 / 12-bit, single predictions and compound AVERAGE / DISTANCE blends with every
Quant_Dist weight pair. Env: MC_ITERS (default 200), MC_SEED.
"""
import os
import random
import types
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, Timer

import recon_model as rm
import intra_model as im
import av1_tables as T

SIZES = [2, 4, 8, 16, 32, 64]


def model_pred(ref, refW, refH, x0, y0, fx, fy, w, h, interp_x, interp_y, bd, compound):
    """block_inter_prediction of the model on reference plane `ref` (rows of ints), unit at integer position
    (x0, y0) with subpel phases (fx, fy), as predict_inter calls it for an unscaled reference."""
    fake = types.SimpleNamespace(h=types.SimpleNamespace(subsampling_x=0, subsampling_y=0, BitDepth=bd),
                                 InterpFilters=[[[interp_y, interp_x]]])
    r0 = 3 + (2 if bd == 12 else 0)
    r1 = 7 if compound else (11 - (2 if bd == 12 else 0))
    sx = (((x0 << 4) + fx) << 6) + 32
    sy = (((y0 << 4) + fy) << 6) + 32
    return rm.FrameRecon.block_inter_prediction(fake, 0, [ref], sx, sy, 1024, 1024, w, h, 0, 0, refW, refH, r0, r1)


async def write_patch(dut, lst, ref, refW, refH, x0, y0, w, h):
    ngrp = (w + 3) >> 2
    for r in range(h + 7):
        srow = ref[im.clip3(0, refH - 1, y0 - 3 + r)]
        for w4 in range(ngrp + 2):
            v = 0
            for l in range(4):
                v |= srow[im.clip3(0, refW - 1, x0 - 3 + 4 * w4 + l)] << (12 * l)
            dut.patch_we.value = 1; dut.patch_list.value = lst; dut.patch_row.value = r; dut.patch_w4.value = w4
            dut.patch_d4.value = v
            await RisingEdge(dut.clk)
    dut.patch_we.value = 0
    await Timer(1, "ns")


async def run(dut, lst, w, h, interp_x, interp_y, fx, fy, bd, mode, wt0, wt1):
    dut.lst.value = lst; dut.w.value = w; dut.h.value = h
    dut.interp_x.value = interp_x; dut.interp_y.value = interp_y; dut.fx.value = fx; dut.fy.value = fy
    dut.bit_depth.value = bd; dut.mode.value = mode; dut.wt0.value = wt0; dut.wt1.value = wt1
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
            v4 = int(dut.out_pix4.value)
            m = int(dut.out_mask.value)
            for l in range(4):
                if (m >> l) & 1:
                    assert x + l < w and y < h, f"output outside the unit: ({x + l}, {y}) in {w}x{h}"
                    assert got[y][x + l] is None, f"({x + l}, {y}) written twice"
                    got[y][x + l] = (v4 >> (12 * l)) & 0xFFF
                    n += 1
        if int(dut.done.value):
            break
        await Timer(1, "ns")
        assert cycles < 20000, "no done"
    await Timer(1, "ns")
    return got, n, cycles


@cocotb.test()
async def mc_filt_vs_model(dut):
    iters = int(os.environ.get("MC_ITERS", "200"))
    seed = int(os.environ.get("MC_SEED", "1"))
    rng = random.Random(seed)
    cocotb.start_soon(Clock(dut.clk, 10, "ns").start())
    dut.rst.value = 1
    dut.patch_we.value = 0; dut.start.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    stats = {"single": 0, "avg": 0, "dist": 0, "samples": 0, "clamped": 0}
    for it in range(iters):
        bd = rng.choice([8, 10, 12])
        mx = (1 << bd) - 1
        w, h = rng.choice(SIZES), rng.choice(SIZES)
        kind = rng.choice(["single", "single", "avg", "dist"])
        compound = kind != "single"
        r0 = 3 + (2 if bd == 12 else 0)
        r1 = 7 if compound else (11 - (2 if bd == 12 else 0))
        post = 14 - r0 - r1
        refW, refH = rng.randint(8, 80), rng.randint(8, 80)
        preds = []
        for lst in range(2 if compound else 1):
            ref = [[rng.randint(0, mx) for _ in range(refW)] for _ in range(refH)]
            # positions: inside, or crossing any edge (clamping = edge replication)
            x0 = rng.randint(-12, refW + 4) if rng.random() < 0.5 else rng.randint(0, max(0, refW - w))
            y0 = rng.randint(-12, refH + 4) if rng.random() < 0.5 else rng.randint(0, max(0, refH - h))
            if x0 - 3 < 0 or x0 + w + 4 > refW or y0 - 3 < 0 or y0 + h + 4 > refH:
                stats["clamped"] += 1
            fx, fy = rng.randint(0, 15), rng.randint(0, 15)
            interp_x, interp_y = rng.randint(0, 3), rng.randint(0, 3)
            await write_patch(dut, lst, ref, refW, refH, x0, y0, w, h)
            preds.append(model_pred(ref, refW, refH, x0, y0, fx, fy, w, h, interp_x, interp_y, bd, compound))
            if compound and lst == 0:
                mode, wt0, wt1 = 1, 0, 0
            elif kind == "avg":
                mode, wt0, wt1 = 2, 0, 0
            elif kind == "dist":
                mode = 3
                row = rng.choice(T.Quant_Dist_Lookup)
                wt0, wt1 = (row[0], row[1]) if rng.random() < 0.5 else (row[1], row[0])
            else:
                mode, wt0, wt1 = 0, 0, 0
            got, n, cycles = await run(dut, lst, w, h, interp_x, interp_y, fx, fy, bd, mode, wt0, wt1)
        # expected (predict_inter's blend lines)
        clip1 = lambda v: 0 if v < 0 else mx if v > mx else v  # noqa: E731
        if kind == "single":
            exp = [[clip1(preds[0][i][j]) for j in range(w)] for i in range(h)]
        elif kind == "avg":
            exp = [[clip1(im.round2(preds[0][i][j] + preds[1][i][j], 1 + post)) for j in range(w)] for i in range(h)]
        else:
            exp = [[clip1(im.round2(wt0 * preds[0][i][j] + wt1 * preds[1][i][j], 4 + post)) for j in range(w)]
                   for i in range(h)]
        assert n == w * h, f"iter {it}: got {n} samples, expected {w * h} ({w}x{h} {kind})"
        for i in range(h):
            for j in range(w):
                assert got[i][j] == exp[i][j], (f"iter {it}: {kind} {w}x{h} bd {bd} interp ({interp_x},{interp_y}) phase "
                                                f"({fx},{fy}): ({j},{i}) rtl {got[i][j]} model {exp[i][j]}; row {got[i][:8]} vs {exp[i][:8]}")
        stats[kind] += 1
        stats["samples"] += w * h
    dut._log.info(f"OK: {stats}")
