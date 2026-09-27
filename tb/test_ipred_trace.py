"""Replay dav1d's P (intra prediction) events from a v3 trace through the ipred RTL.

Uses the same edge/availability interpretation as tools/xcheck_trace.py (which is verified against
the Python model), so any mismatch here is an RTL bug. Env: IPRED_TRACE=<trace>, IPRED_BD=<8|10>,
IPRED_MAX (optional cap on the number of blocks).
"""
import os
import sys
from pathlib import Path
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import intra_model as im
import xcheck_trace as xc
from test_ipred import load_edges, run_block


def iter_p_events(path):
    with open(path) as fh:
        for line in fh:
            if line.startswith("P "):
                f = line.split()
                yield [f[0]] + [int(x) for x in f[1:]]


@cocotb.test()
async def replay_pred(dut):
    path = os.environ["IPRED_TRACE"]
    bd = int(os.environ.get("IPRED_BD", "8"))
    max_n = int(os.environ.get("IPRED_MAX", "0")) or None
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.start.value = 0
    dut.edge_we.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)

    n = 0
    total_cycles = 0
    from collections import Counter
    kinds = Counter()
    for f in iter_p_events(path):
        plane, x4, y4, w, h = f[1:6]
        mode, m, angle, maxw, maxh, rawtl = f[6:12]
        n_edge = 2 * h + 2 * w + 1
        edge = list(f[12:12 + n_edge])
        edge[2 * h] = rawtl
        pix = f[12 + n_edge:12 + n_edge + w * h]
        log2w, log2h = w.bit_length() - 1, h.bit_length() - 1
        p_angle = angle & 511
        is_sm = (angle >> 9) & 1
        edge_en = angle >> 10
        if mode == xc.FILTER_PRED or im.is_directional(mode):
            have_left = 0 if (m == 2 and p_angle > 180) else 1
            have_above = 0 if (m == 1 and p_angle < 90) else 1
            above, left, tl = xc.edges_from_dav1d(edge, w, h, 1, 1, bd)
            if not have_above: above = [above[0]] * (w + h); tl = above[0]
            if not have_left:  left = [left[0]] * (w + h);   tl = left[0]
        else:
            have_left, have_above = xc.availability(m)
            above, left, tl = xc.edges_from_dav1d(edge, w, h, have_left, have_above, bd)
        if mode == xc.FILTER_PRED:
            use_fi, fimode, mode_rtl, delta = 1, p_angle, 0, 0
        elif im.is_directional(mode):
            use_fi, fimode, mode_rtl = 0, 0, mode
            delta = (p_angle - im.MODE_TO_ANGLE[mode]) // im.ANGLE_STEP
        else:
            use_fi, fimode, mode_rtl, delta = 0, 0, mode, 0
        await load_edges(dut, above, left, tl)
        params = dict(mode=mode_rtl, use_filter_intra=use_fi, filter_intra_mode=fimode, angle_delta=delta & 7,
                      log2w=log2w, log2h=log2h, bit_depth=bd, have_left=have_left, have_above=have_above,
                      filter_type=is_sm, edge_filter_en=edge_en, above_px=min(w, maxw), left_px=min(h, maxh))
        got, cycles = await run_block(dut, params, w, h)
        total_cycles += cycles
        flat = [v for row in got for v in row]
        if flat != pix:
            bad = [(i // w, i % w, flat[i], pix[i]) for i in range(w * h) if flat[i] != pix[i]]
            raise AssertionError(f"plane {plane} ({x4},{y4}) {w}x{h} mode {mode} m {m} angle {angle} maxw {maxw} maxh {maxh}: "
                                 f"{len(bad)} bad, first {bad[:8]}")
        kinds["fi" if mode == xc.FILTER_PRED else ("dir" if im.is_directional(mode) else "other")] += 1
        n += 1
        if max_n and n >= max_n:
            break
    dut._log.info(f"OK: {n} prediction blocks match dav1d ({dict(kinds)}); {total_cycles} cycles total")
