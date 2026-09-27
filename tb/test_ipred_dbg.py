import cocotb, json, os
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, Timer, ClockCycles
import intra_model as im
from test_ipred import load_edges, run_block

@cocotb.test()
async def dbg(dut):
    c = json.loads(os.environ["IPRED_CASE"])
    w, h = 1 << c["log2w"], 1 << c["log2h"]
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1; dut.start.value = 0; dut.edge_we.value = 0
    await ClockCycles(dut.clk, 3); dut.rst.value = 0; await RisingEdge(dut.clk)
    # model with visible edges
    above = im.Edge(c["above"], c["tl"]); left = im.Edge(c["left"], c["tl"])
    exp = im.pred_directional(above, left, w, h, c["mode"], c["delta"], c["bd"], c["have_left"], c["have_above"], c["efe"], c["ft"], c["apx"], c["lpx"])
    await load_edges(dut, c["above"], c["left"], c["tl"])
    params = dict(mode=c["mode"], use_filter_intra=0, filter_intra_mode=0, angle_delta=c["delta"] & 7, log2w=c["log2w"], log2h=c["log2h"],
                  bit_depth=c["bd"], have_left=c["have_left"], have_above=c["have_above"], filter_type=c["ft"], edge_filter_en=c["efe"], above_px=c["apx"], left_px=c["lpx"])
    got, cycles = await run_block(dut, params, w, h)
    rA = [int(dut.A[k + 2].value) for k in range(-2, 2 * (w + h) + 1)]
    rL = [int(dut.L[k + 2].value) for k in range(-2, 2 * (w + h) + 1)]
    mA = [above[k] for k in range(-2, 2 * (w + h) + 1)]
    mL = [left[k] for k in range(-2, 2 * (w + h) + 1)]
    dut._log.info(f"up_a={int(dut.up_a.value)} up_l={int(dut.up_l.value)} p_angle={int(dut.p_angle.value)} dx={int(dut.dx.value)} dy={int(dut.dy.value)}")
    dut._log.info(f"A rtl  [-2..]: {rA}")
    dut._log.info(f"A model[-2..]: {mA}")
    dut._log.info(f"L rtl  [-2..]: {rL}")
    dut._log.info(f"L model[-2..]: {mL}")
    bad = [(i, j, got[i][j], exp[i][j]) for i in range(h) for j in range(w) if got[i][j] != exp[i][j]]
    dut._log.info(f"pixels bad: {bad[:10]}")
