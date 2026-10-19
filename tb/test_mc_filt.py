"""Fuzz the motion compensation filter (rtl/mc_filt.sv) against tb/recon_model.py's block_inter_prediction and the
compound blend of predict_inter (7.11.3.1 AVERAGE / DISTANCE).

Random reference pictures, unit positions that cross every picture edge (the testbench does ref_fetch's job: it
writes the clamped patch), all interpolation filters and phases, unit sizes 2 .. 64 (chroma of a 4xN luma block
predicts 2-wide units), 8 / 10 / 12-bit, single predictions and compound AVERAGE / DISTANCE blends with every
Quant_Dist weight pair; scaled references (steps 64 .. 2048, units <= 32 x 32 when downscaling) on a share
MC_SCALED (default 0.4) of the units. Env: MC_ITERS (default 200), MC_SEED.
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


def model_pred(ref, refW, refH, x0, y0, xf, yf, xs, ys, w, h, interp_x, interp_y, bd, compound):
    """block_inter_prediction of the model on reference plane `ref` (rows of ints), unit at position
    (x0 * 1024 + xf, y0 * 1024 + yf) in 1/1024 samples with steps (xs, ys) (1024 = unscaled), as predict_inter
    calls it after motion_vector_scaling."""
    fake = types.SimpleNamespace(h=types.SimpleNamespace(subsampling_x=0, subsampling_y=0, BitDepth=bd),
                                 InterpFilters=[[[interp_y, interp_x]]])
    r0 = 3 + (2 if bd == 12 else 0)
    r1 = 7 if compound else (11 - (2 if bd == 12 else 0))
    sx = (x0 << 10) + xf
    sy = (y0 << 10) + yf
    return rm.FrameRecon.block_inter_prediction(fake, 0, [ref], sx, sy, xs, ys, w, h, 0, 0, refW, refH, r0, r1)


def patch_extent(w, h, xs, ys):
    """(rows, words) of the patch mc_filt reads: rows = intermediateHeight, words = ceil(cols / 4) with
    cols = (((w - 1) xs + 1023) >> 10) + 8, at least the unscaled ngrp + 2."""
    rows = (((h - 1) * ys + 1023) >> 10) + 8
    cols = (((w - 1) * xs + 1023) >> 10) + 8
    words = max(((w + 3) >> 2) + 2, (cols + 3) >> 2)
    assert rows <= 72 and words <= 18, f"patch {rows} rows x {words} words: scaled units must be <= 32 x 32"
    return rows, words


async def write_patch(dut, lst, ref, refW, refH, x0, y0, w, h, xs=1024, ys=1024):
    rows, words = patch_extent(w, h, xs, ys)
    for r in range(rows):
        srow = ref[im.clip3(0, refH - 1, y0 - 3 + r)]
        for w4 in range(words):
            v = 0
            for l in range(4):
                v |= srow[im.clip3(0, refW - 1, x0 - 3 + 4 * w4 + l)] << (12 * l)
            dut.patch_we.value = 1; dut.patch_list.value = lst; dut.patch_row.value = r; dut.patch_w4.value = w4
            dut.patch_d4.value = v
            await RisingEdge(dut.clk)
    dut.patch_we.value = 0
    await Timer(1, "ns")


async def run(dut, lst, w, h, interp_x, interp_y, xf, yf, bd, mode, wt0, wt1, xs=1024, ys=1024):
    dut.lst.value = lst; dut.w.value = w; dut.h.value = h
    dut.interp_x.value = interp_x; dut.interp_y.value = interp_y
    dut.x_frac0.value = xf; dut.y_frac0.value = yf; dut.x_step.value = xs; dut.y_step.value = ys
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
            pr = int(dut.out_pair.value)
            m = int(dut.out_mask.value)
            for l in range(4):
                if (m >> l) & 1:
                    assert x + l < w and y < h, f"output outside the unit: ({x + l}, {y}) in {w}x{h}"
                    assert got[y][x + l] is None, f"({x + l}, {y}) written twice"
                    if mode == 4:
                        s0 = (pr >> (16 * l)) & 0xFFFF
                        s1 = (pr >> (64 + 16 * l)) & 0xFFFF
                        got[y][x + l] = (s0 - 65536 if s0 >> 15 else s0, s1 - 65536 if s1 >> 15 else s1)
                    else:
                        got[y][x + l] = (v4 >> (12 * l)) & 0xFFF
                    n += 1
        if int(dut.done.value):
            break
        await Timer(1, "ns")
        assert cycles < 40000, "no done"
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
    stats = {"single": 0, "avg": 0, "dist": 0, "pair": 0, "scaled": 0, "samples": 0, "clamped": 0}
    scaled_share = float(os.environ.get("MC_SCALED", "0.4"))
    for it in range(iters):
        bd = rng.choice([8, 10, 12])
        mx = (1 << bd) - 1
        w, h = rng.choice(SIZES), rng.choice(SIZES)
        # scaled references (7.11.3.3): steps 64 .. 2048 per axis (1/16x .. 2x), units <= 32 x 32 when a step > 1024
        if rng.random() < scaled_share:
            pick = lambda: rng.choice([1024, rng.randint(64, 2048), rng.randint(512, 2048), 2048, 64])  # noqa: E731
            xs, ys = pick(), pick()
            if xs == 1024 and ys == 1024:
                ys = 1536
            if xs > 1024 or ys > 1024:
                w, h = min(w, 32), min(h, 32)
        else:
            xs, ys = 1024, 1024
        kind = rng.choice(["single", "single", "avg", "dist", "pair"])
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
            if xs == 1024 and ys == 1024:
                fx, fy = (rng.randint(0, 15) << 6) | 32, (rng.randint(0, 15) << 6) | 32   # the phases of an unscaled MV
            else:
                fx, fy = rng.randint(0, 1023), rng.randint(0, 1023)
            interp_x, interp_y = rng.randint(0, 3), rng.randint(0, 3)
            await write_patch(dut, lst, ref, refW, refH, x0, y0, w, h, xs, ys)
            preds.append(model_pred(ref, refW, refH, x0, y0, fx, fy, xs, ys, w, h, interp_x, interp_y, bd, compound))
            if compound and lst == 0:
                mode, wt0, wt1 = 1, 0, 0
            elif kind == "avg":
                mode, wt0, wt1 = 2, 0, 0
            elif kind == "dist":
                mode = 3
                row = rng.choice(T.Quant_Dist_Lookup)
                wt0, wt1 = (row[0], row[1]) if rng.random() < 0.5 else (row[1], row[0])
            elif kind == "pair":
                mode, wt0, wt1 = 4, 0, 0
            else:
                mode, wt0, wt1 = 0, 0, 0
            got, n, cycles = await run(dut, lst, w, h, interp_x, interp_y, fx, fy, bd, mode, wt0, wt1, xs, ys)
        # expected (predict_inter's blend lines)
        clip1 = lambda v: 0 if v < 0 else mx if v > mx else v  # noqa: E731
        if kind == "single":
            exp = [[clip1(preds[0][i][j]) for j in range(w)] for i in range(h)]
        elif kind == "avg":
            exp = [[clip1(im.round2(preds[0][i][j] + preds[1][i][j], 1 + post)) for j in range(w)] for i in range(h)]
        elif kind == "dist":
            exp = [[clip1(im.round2(wt0 * preds[0][i][j] + wt1 * preds[1][i][j], 4 + post)) for j in range(w)]
                   for i in range(h)]
        else:
            exp = [[(preds[0][i][j], preds[1][i][j]) for j in range(w)] for i in range(h)]
        assert n == w * h, f"iter {it}: got {n} samples, expected {w * h} ({w}x{h} {kind})"
        for i in range(h):
            for j in range(w):
                assert got[i][j] == exp[i][j], (f"iter {it}: {kind} {w}x{h} bd {bd} interp ({interp_x},{interp_y}) frac "
                                                f"({fx},{fy}) step ({xs},{ys}): ({j},{i}) rtl {got[i][j]} model {exp[i][j]}; "
                                                f"row {got[i][:8]} vs {exp[i][:8]}")
        stats[kind] += 1
        stats["scaled"] += (xs, ys) != (1024, 1024)
        stats["samples"] += w * h
    dut._log.info(f"OK: {stats}")
