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


def model_warp(ref, refW, refH, params, x, y, bd, compound):
    """block_warp of the model: the 8x8 at plane position (x, y) through the affine params (luma = plane)."""
    fake = types.SimpleNamespace(h=types.SimpleNamespace(subsampling_x=0, subsampling_y=0, BitDepth=bd),
                                 resolve_divisor=rm.FrameRecon.resolve_divisor)
    fake.setup_shear = lambda wp: rm.FrameRecon.setup_shear(fake, wp)
    r0 = 3 + (2 if bd == 12 else 0)
    r1 = 7 if compound else (11 - (2 if bd == 12 else 0))
    pred = [[0] * 8 for _ in range(8)]
    rm.FrameRecon.block_warp(fake, params, 0, [ref], refW, refH, x, y, 0, 0, 8, 8, pred, r0, r1)
    return pred


def warp_position(params, x, y):
    """ix4 / iy4 / sx4 / sy4 of block_warp for the 8x8 at (x, y), luma (no subsampling)."""
    srcX, srcY = x + 4, y + 4
    dstX = params[2] * srcX + params[3] * srcY + params[0]
    dstY = params[4] * srcX + params[5] * srcY + params[1]
    return dstX >> 16, dstY >> 16, dstX & 0xFFFF, dstY & 0xFFFF


def random_warp_params(rng):
    """Affine parameters with a valid shear (setup_shear), identity-ish with random translation."""
    fake = types.SimpleNamespace(resolve_divisor=rm.FrameRecon.resolve_divisor)
    while True:
        wide = rng.random() < 0.3
        r = (1 << 13) if wide else (1 << 10)
        params = [rng.randint(-(1 << 19), (1 << 19)), rng.randint(-(1 << 19), (1 << 19)),
                  65536 + rng.randint(-r, r), rng.randint(-r, r), rng.randint(-r, r), 65536 + rng.randint(-r, r)]
        valid, alpha, beta, gamma, delta = rm.FrameRecon.setup_shear(fake, params)
        if valid:
            return params, (alpha, beta, gamma, delta)


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


async def run(dut, lst, w, h, interp_x, interp_y, xf, yf, bd, mode, wt0, wt1, xs=1024, ys=1024, warp=None):
    dut.lst.value = lst; dut.w.value = w; dut.h.value = h
    dut.interp_x.value = interp_x; dut.interp_y.value = interp_y
    dut.x_frac0.value = xf; dut.y_frac0.value = yf; dut.x_step.value = xs; dut.y_step.value = ys
    if warp is None:
        dut.warp.value = 0
    else:
        shear, sx4, sy4 = warp
        dut.warp.value = 1
        dut.w_alpha.value = shear[0] & 0xFFFF; dut.w_beta.value = shear[1] & 0xFFFF
        dut.w_gamma.value = shear[2] & 0xFFFF; dut.w_delta.value = shear[3] & 0xFFFF
        dut.w_sx4.value = sx4; dut.w_sy4.value = sy4
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
                        s0 = (pr >> (17 * l)) & 0x1FFFF
                        s1 = (pr >> (68 + 17 * l)) & 0x1FFFF
                        got[y][x + l] = (s0 - (1 << 17) if s0 >> 16 else s0, s1 - (1 << 17) if s1 >> 16 else s1)
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
    stats = {"single": 0, "avg": 0, "dist": 0, "pair": 0, "scaled": 0, "warp": 0, "samples": 0, "clamped": 0}
    scaled_share = float(os.environ.get("MC_SCALED", "0.4"))
    warp_share = float(os.environ.get("MC_WARP", "0.3"))
    for it in range(iters):
        bd = rng.choice([8, 10, 12])
        mx = (1 << bd) - 1
        w, h = rng.choice(SIZES), rng.choice(SIZES)
        # warped motion (7.11.3.5): one 8x8 per run, every list through its own affine parameters
        use_warp = rng.random() < warp_share
        if use_warp:
            w, h = 8, 8
        # scaled references (7.11.3.3): steps 64 .. 2048 per axis (1/16x .. 2x), units <= 32 x 32 when a step > 1024
        if not use_warp and rng.random() < scaled_share:
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
            warp = None
            if use_warp:
                # the 8x8 sits at a plane position; its projection lands anywhere, inside or across the edges
                params, shear = random_warp_params(rng)
                px, py = rng.randint(0, 64), rng.randint(0, 64)
                ix4, iy4, sx4, sy4 = warp_position(params, px, py)
                if not (-40 <= ix4 - refW <= 40 and -40 <= iy4 - refH <= 40):
                    params[0] -= (ix4 - refW // 2) << 16            # pull the projection back near the picture
                    params[1] -= (iy4 - refH // 2) << 16
                    ix4, iy4, sx4, sy4 = warp_position(params, px, py)
                x0, y0 = ix4 - 4, iy4 - 4
                warp = (shear, sx4, sy4)
                await write_patch(dut, lst, ref, refW, refH, x0, y0, 8, 8)
                preds.append(model_warp(ref, refW, refH, params, px, py, bd, compound))
            else:
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
            got, n, cycles = await run(dut, lst, w, h, interp_x, interp_y, fx, fy, bd, mode, wt0, wt1, xs, ys, warp)
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
                                                f"({fx},{fy}) step ({xs},{ys}) warp {use_warp}: ({j},{i}) rtl {got[i][j]} model {exp[i][j]}; "
                                                f"row {got[i][:8]} vs {exp[i][:8]}")
        stats[kind] += 1
        stats["scaled"] += (xs, ys) != (1024, 1024)
        stats["warp"] += use_warp
        stats["samples"] += w * h
    dut._log.info(f"OK: {stats}")


# ---------------------------------------------------------------- overlapped runs (2026-11-08)
# Since 2026-11-08 the horizontal pass of run n + 1 overlaps the vertical pass of run n (two sides with a one-run
# handoff, double-buffered strip memory, outputs tagged with run_tag). This test drives the filter the way recon_top
# does: the next run's patch goes into the other patch buffer while a run is in flight, start comes as soon as h_ready,
# and every output word is sorted by its tag and checked against the model when that run's done arrives.

async def write_patch_buf(dut, pb, lst, ref, refW, refH, x0, y0, w, h, xs=1024, ys=1024):
    dut.patch_buf.value = pb
    await write_patch(dut, lst, ref, refW, refH, x0, y0, w, h, xs, ys)


async def start_when_ready(dut, lst, w, h, interp_x, interp_y, xf, yf, bd, mode, wt0, wt1, xs, ys, warp, rbuf, tag):
    """Wait for h_ready, then one start pulse with the run's parameters (the registered handshake recon_top uses)."""
    waited = 0
    while True:
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.h_ready.value):
            break
        waited += 1
        assert waited < 40000, "h_ready never came"
    await Timer(1, "ns")
    dut.lst.value = lst; dut.w.value = w; dut.h.value = h
    dut.interp_x.value = interp_x; dut.interp_y.value = interp_y
    dut.x_frac0.value = xf; dut.y_frac0.value = yf; dut.x_step.value = xs; dut.y_step.value = ys
    if warp is None:
        dut.warp.value = 0
    else:
        shear, sx4, sy4 = warp
        dut.warp.value = 1
        dut.w_alpha.value = shear[0] & 0xFFFF; dut.w_beta.value = shear[1] & 0xFFFF
        dut.w_gamma.value = shear[2] & 0xFFFF; dut.w_delta.value = shear[3] & 0xFFFF
        dut.w_sx4.value = sx4; dut.w_sy4.value = sy4
    dut.bit_depth.value = bd; dut.mode.value = mode; dut.wt0.value = wt0; dut.wt1.value = wt1
    dut.rbuf.value = rbuf; dut.run_tag.value = tag
    dut.start.value = 1
    await RisingEdge(dut.clk)
    await Timer(1, "ns")
    dut.start.value = 0


class RunCollector:
    """Sorts the output words by tag; at every done pulse the oldest pending run is checked against its expectation."""

    def __init__(self, dut):
        self.dut = dut
        self.pending = []                      # dicts: tag, label, w, h, mode, exp (None for a parked list-0 run)
        self.buckets = {0: [], 1: []}
        self.dones = 0
        self.failures = []
        self.overlap_cycles = 0                # cycles with both sides busy
        self.busy_cycles = 0

    async def run(self):
        dut = self.dut
        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.out_valid.value):
                self.buckets[int(dut.out_tag.value)].append((int(dut.out_x.value), int(dut.out_y.value), int(dut.out_pix4.value),
                                                            int(dut.out_pair.value), int(dut.out_mask.value)))
            h_busy = int(dut.hst.value) != 0
            v_busy = int(dut.vst.value) != 0
            self.busy_cycles += h_busy or v_busy
            self.overlap_cycles += h_busy and v_busy
            if int(dut.done.value):
                self.dones += 1
                if not self.pending:
                    self.failures.append("done without a pending run")
                else:
                    self.check(self.pending.pop(0))

    def check(self, r):
        words = self.buckets[r["tag"]]
        self.buckets[r["tag"]] = []
        w, h, mode, exp, label = r["w"], r["h"], r["mode"], r["exp"], r["label"]
        if mode == 1:                          # parked list-0 prediction: no output words
            if words:
                self.failures.append(f"{label}: {len(words)} output words from a parked run")
            return
        got = [[None] * w for _ in range(h)]
        n = 0
        for x, y, v4, pr, m in words:
            for l in range(4):
                if (m >> l) & 1:
                    if not (x + l < w and y < h):
                        self.failures.append(f"{label}: output outside the unit ({x + l}, {y})"); return
                    if got[y][x + l] is not None:
                        self.failures.append(f"{label}: ({x + l}, {y}) written twice"); return
                    if mode == 4:
                        s0 = (pr >> (17 * l)) & 0x1FFFF
                        s1 = (pr >> (68 + 17 * l)) & 0x1FFFF
                        got[y][x + l] = (s0 - (1 << 17) if s0 >> 16 else s0, s1 - (1 << 17) if s1 >> 16 else s1)
                    else:
                        got[y][x + l] = (v4 >> (12 * l)) & 0xFFF
                    n += 1
        if n != w * h:
            self.failures.append(f"{label}: {n} samples, expected {w * h}"); return
        for i in range(h):
            for j in range(w):
                if got[i][j] != exp[i][j]:
                    self.failures.append(f"{label}: ({j},{i}) rtl {got[i][j]} model {exp[i][j]}; row {got[i][:8]} vs {exp[i][:8]}")
                    return


@cocotb.test()
async def mc_filt_overlap(dut):
    iters = int(os.environ.get("MC_OVERLAP_ITERS", "150"))
    seed = int(os.environ.get("MC_SEED", "1")) + 1000
    rng = random.Random(seed)
    cocotb.start_soon(Clock(dut.clk, 10, "ns").start())
    dut.rst.value = 1
    dut.patch_we.value = 0; dut.start.value = 0; dut.patch_buf.value = 0; dut.rbuf.value = 0; dut.run_tag.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    col = RunCollector(dut)
    cocotb.start_soon(col.run())
    scaled_share = float(os.environ.get("MC_SCALED", "0.4"))
    warp_share = float(os.environ.get("MC_WARP", "0.3"))
    runs = 0
    stats = {"single": 0, "avg": 0, "dist": 0, "pair": 0, "scaled": 0, "warp": 0, "samples": 0}
    for it in range(iters):
        bd = rng.choice([8, 10, 12])
        mx = (1 << bd) - 1
        w, h = rng.choice(SIZES), rng.choice(SIZES)
        use_warp = rng.random() < warp_share
        if use_warp:
            w, h = 8, 8
        if not use_warp and rng.random() < scaled_share:
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
        # the expectation first (both lists' predictions), then the runs back to back
        plan = []
        preds = []
        for lst in range(2 if compound else 1):
            ref = [[rng.randint(0, mx) for _ in range(refW)] for _ in range(refH)]
            x0 = rng.randint(-12, refW + 4) if rng.random() < 0.5 else rng.randint(0, max(0, refW - w))
            y0 = rng.randint(-12, refH + 4) if rng.random() < 0.5 else rng.randint(0, max(0, refH - h))
            if xs == 1024 and ys == 1024:
                fx, fy = (rng.randint(0, 15) << 6) | 32, (rng.randint(0, 15) << 6) | 32
            else:
                fx, fy = rng.randint(0, 1023), rng.randint(0, 1023)
            interp_x, interp_y = rng.randint(0, 3), rng.randint(0, 3)
            warp = None
            if use_warp:
                params, shear = random_warp_params(rng)
                px, py = rng.randint(0, 64), rng.randint(0, 64)
                ix4, iy4, sx4, sy4 = warp_position(params, px, py)
                if not (-40 <= ix4 - refW <= 40 and -40 <= iy4 - refH <= 40):
                    params[0] -= (ix4 - refW // 2) << 16
                    params[1] -= (iy4 - refH // 2) << 16
                    ix4, iy4, sx4, sy4 = warp_position(params, px, py)
                x0, y0 = ix4 - 4, iy4 - 4
                warp = (shear, sx4, sy4)
                preds.append(model_warp(ref, refW, refH, params, px, py, bd, compound))
            else:
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
            plan.append(dict(lst=lst, ref=ref, x0=x0, y0=y0, fx=fx, fy=fy, interp_x=interp_x, interp_y=interp_y,
                             mode=mode, wt0=wt0, wt1=wt1, warp=warp))
        clip1 = lambda v: 0 if v < 0 else mx if v > mx else v  # noqa: E731
        if kind == "single":
            exp = [[clip1(preds[0][i][j]) for j in range(w)] for i in range(h)]
        elif kind == "avg":
            exp = [[clip1(im.round2(preds[0][i][j] + preds[1][i][j], 1 + post)) for j in range(w)] for i in range(h)]
        elif kind == "dist":
            wt0, wt1 = plan[1]["wt0"], plan[1]["wt1"]
            exp = [[clip1(im.round2(wt0 * preds[0][i][j] + wt1 * preds[1][i][j], 4 + post)) for j in range(w)]
                   for i in range(h)]
        else:
            exp = [[(preds[0][i][j], preds[1][i][j]) for j in range(w)] for i in range(h)]
        for k, r in enumerate(plan):
            pb = runs & 1
            label = (f"iter {it} run {k}: {kind} {w}x{h} bd {bd} interp ({r['interp_x']},{r['interp_y']}) frac ({r['fx']},{r['fy']}) "
                     f"step ({xs},{ys}) warp {use_warp}")
            # the patch of this run goes into the buffer the run before the previous one used: that run's horizontal pass
            # is over (the previous run could only start once the horizontal side was free)
            if use_warp:
                await write_patch_buf(dut, pb, r["lst"], r["ref"], refW, refH, r["x0"], r["y0"], 8, 8)
            else:
                await write_patch_buf(dut, pb, r["lst"], r["ref"], refW, refH, r["x0"], r["y0"], w, h, xs, ys)
            col.pending.append(dict(tag=pb, label=label, w=w, h=h, mode=r["mode"], exp=exp if k == len(plan) - 1 else None))
            await start_when_ready(dut, r["lst"], w, h, r["interp_x"], r["interp_y"], r["fx"], r["fy"], bd, r["mode"], r["wt0"], r["wt1"],
                                   xs, ys, r["warp"], pb, pb)
            runs += 1
            assert not col.failures, col.failures[0]
        stats[kind] += 1
        stats["scaled"] += (xs, ys) != (1024, 1024)
        stats["warp"] += use_warp
        stats["samples"] += w * h
    waited = 0
    while col.dones < runs:
        await RisingEdge(dut.clk)
        waited += 1
        assert waited < 200000, f"{runs - col.dones} runs never finished"
    await Timer(1, "ns")
    assert not col.failures, col.failures[0]
    assert col.overlap_cycles > 0, "the two sides never overlapped"
    dut._log.info(f"OK: {runs} runs back to back, {stats}; both sides busy in {col.overlap_cycles} of {col.busy_cycles} busy cycles "
                  f"({100 * col.overlap_cycles // max(col.busy_cycles, 1)}%)")
