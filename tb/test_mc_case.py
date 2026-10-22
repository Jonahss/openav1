"""mc_filt on one recorded case (MC_CASE=<json from tools/mc_cases.py>): the exact reference patch, start position, steps,
filters and bit depth of a real block, RTL against the model's block_inter_prediction. Strips of <= 32 rows like
recon_top's scaled path (the start moves by ys * 32 per strip)."""
import json
import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

import test_mc_filt as MF


@cocotb.test()
async def mc_case(dut):
    cases = json.load(open(os.environ["MC_CASE"]))
    if isinstance(cases, dict): cases = [cases]
    cocotb.start_soon(Clock(dut.clk, 10, "ns").start())
    dut.rst.value = 1
    dut.patch_we.value = 0; dut.start.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    total_bad = 0; bad_cases = []; n_strips = 0; rec_bad_cases = []
    for ci, case in enumerate(cases):
        patch = case["patch"]; pw, ph_ = len(patch[0]), len(patch)
        w, h = case["block"]["w"], case["block"]["h"]
        xs, ys, bd = case["xs"], case["ys"], case["bd"]
        iy = case["iy"]
        interp_x, interp_y = case["block"]["interp"][1], case["block"]["interp"][0]   # interp = (y filter, x filter) as the block record stores it
        for sy0 in range(0, h, 32):
            for sx0 in range(0, w, 32):
                sh, sw = min(32, h - sy0), min(32, w - sx0)
                start_y = case["startY"] + ys * sy0; start_x = case["startX"] + xs * sx0
                iy_s, fy_s = start_y >> 10, start_y & 1023; ix_s, fx_s = start_x >> 10, start_x & 1023
                py0 = iy_s - iy + 3; px0 = ix_s - case["ix"] + 3
                await MF.write_patch(dut, 0, patch, pw, ph_, px0, py0, sw, sh, xs, ys)
                got, n, cycles = await MF.run(dut, 0, sw, sh, interp_x, interp_y, fx_s, fy_s, bd, 0, 0, 0, xs, ys)
                exp = MF.model_pred(patch, pw, ph_, px0, py0, fx_s, fy_s, xs, ys, sw, sh, interp_x, interp_y, bd, False)
                mx = (1 << bd) - 1
                exp = [[0 if v < 0 else mx if v > mx else v for v in row] for row in exp]      # a single prediction is clipped to a pixel
                bad = [(y, x, got[y][x], exp[y][x]) for y in range(sh) for x in range(sw) if got[y][x] != exp[y][x]]
                n_strips += 1
                if bad:
                    total_bad += len(bad); bad_cases.append((ci, case["block"], sx0, sy0, len(bad), bad[:4]))
                if not case["block"].get("interintra"):
                    rec = [case["pred"][sy0 + y][sx0:sx0 + sw] for y in range(sh)]
                    badr = [(y, x, got[y][x], rec[y][x]) for y in range(sh) for x in range(sw) if got[y][x] != rec[y][x]]
                    if badr: rec_bad_cases.append((ci, case["block"], sx0, sy0, len(badr), badr[:4]))
        if ci % 200 == 0:
            dut._log.info(f"case {ci}/{len(cases)}: {n_strips} strips, {total_bad} bad vs model_pred, {len(rec_bad_cases)} strips differ from the model recon")
    dut._log.info(f"DONE {len(cases)} cases, {n_strips} strips: {total_bad} samples differ vs model_pred in {len(bad_cases)} strips; {len(rec_bad_cases)} strips differ from the model's recon")
    for e in bad_cases[:10]: dut._log.info(f"  bad vs model_pred: {e}")
    for e in rec_bad_cases[:10]: dut._log.info(f"  bad vs recon: {e}")
    assert total_bad == 0 and not rec_bad_cases, f"{total_bad} samples differ from the model; {len(rec_bad_cases)} strips differ from the recon"
