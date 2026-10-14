"""Fuzz the double-pumped on-chip frame memory (rtl/frame_mem_uram.sv) against a Python picture: random reads
and writes on both port pairs every clock, host reads when read port 1 is idle. Checks the model's contract:
one-cycle read latency, the sample and its aligned group, lane write enables, a read in the same cycle as a
write to the same word sees the old data. Env: FM_ITERS (default 4000), FM_SEED.
"""
import os
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, Timer

FBX = int(os.environ.get("FBX", "9"))
FBY = int(os.environ.get("FBY", "9"))
PW = 12
MASK = (1 << PW) - 1


@cocotb.test()
async def fuzz_vs_model(dut):
    iters = int(os.environ.get("FM_ITERS", "4000"))
    rng = random.Random(int(os.environ.get("FM_SEED", "1")))
    W, H = 1 << FBX, 1 << FBY
    dims = [(W, H), (W // 2, H // 2), (W // 2, H // 2)]
    pic = [[[0] * w for _ in range(h)] for (w, h) in dims]     # [plane][y][x]
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    cocotb.start_soon(Clock(dut.clk2x, 5, unit="ns").start())
    # phase: 1 during the first half of each clk cycle (sampled by the clk2x edges)
    async def phase_gen():                                   # 1 at the mid-cycle clk2x edge, 0 at the clk-aligned one
        while True:
            await RisingEdge(dut.clk)
            await Timer(2.5, "ns")
            dut.phase.value = 1
            await Timer(5, "ns")
            dut.phase.value = 0
    cocotb.start_soon(phase_gen())
    dbg_from = int(os.environ.get("FM_DBG_FROM", "-1"))
    cyc = {"n": -1}
    async def mon():                                         # per clk2x edge: what the RAM saw
        while True:
            await RisingEdge(dut.clk2x)
            await ReadOnly()
            if dbg_from >= 0 and dbg_from <= cyc["n"] <= dbg_from + 3:
                dut._log.info(f"  cyc {cyc['n']} t={cocotb.utils.get_sim_time('ns')} phase={int(dut.phase.value)} a_en={int(dut.a_en.value)} a_we={int(dut.a_we.value)} a_addr={int(dut.a_addr.value)} a_q={int(dut.a_q.value) & 0xfff} b_en={int(dut.b_en.value)} b_we={int(dut.b_we.value)} b_addr={int(dut.b_addr.value)} q0={int(dut.q0.value) & 0xfff}")
    cocotb.start_soon(mon())
    dut.re.value = 0; dut.we.value = 0; dut.w_be.value = 0
    dut.r_plane.value = 0; dut.r_x.value = 0; dut.r_y.value = 0
    dut.w_plane.value = 0; dut.w_x.value = 0; dut.w_y.value = 0; dut.w_data.value = 0
    dut.h_plane.value = 0; dut.h_x.value = 0; dut.h_y.value = 0
    for _ in range(4):
        await RisingEdge(dut.clk)

    def rnd_pos(p):
        w, h = dims[p]
        return rng.randrange(w), rng.randrange(h)

    hist = []
    pending = None          # expectations for the reads issued last cycle: [(port, plane, x, y, exp_sample, exp_group)]
    host_pending = None
    checked = 0
    for it in range(iters):
        cyc["n"] = it
        # ---- drive this cycle's requests (randomly, with bursts of locality)
        re = [rng.random() < 0.7, rng.random() < 0.7]
        we = [rng.random() < 0.4, rng.random() < 0.4]
        hp = rng.randrange(3); hx, hy = rnd_pos(hp)
        wr = []
        for i in range(2):
            p = rng.randrange(3); x, y = rnd_pos(p)
            be = rng.randrange(1, 16) if rng.random() < 0.5 else 0xF
            data = [rng.randrange(1 << PW) for _ in range(4)]
            wr.append((p, x, y, be, data))
        rp = []
        for i in range(2):
            p = rng.randrange(3); x, y = rnd_pos(p)
            rp.append((p, x, y))
        dut.re.value = (re[1] << 1) | re[0]
        dut.r_plane.value = (rp[1][0] << 2) | rp[0][0]
        dut.r_x.value = (rp[1][1] << FBX) | rp[0][1]
        dut.r_y.value = (rp[1][2] << FBY) | rp[0][2]
        dut.we.value = (we[1] << 1) | we[0]
        dut.w_be.value = (wr[1][3] << 4) | wr[0][3]
        dut.w_plane.value = (wr[1][0] << 2) | wr[0][0]
        dut.w_x.value = (wr[1][1] << FBX) | wr[0][1]
        dut.w_y.value = (wr[1][2] << FBY) | wr[0][2]
        wd = 0
        for i in range(2):
            for l in range(4):
                wd |= wr[i][4][l] << ((i * 4 + l) * PW)
        dut.w_data.value = wd
        dut.h_plane.value = hp; dut.h_x.value = hx; dut.h_y.value = hy
        # ---- expectations for this cycle's reads: the picture BEFORE this cycle's writes
        exp = []
        for i in range(2):
            if re[i]:
                p, x, y = rp[i]
                g = [pic[p][y][(x & ~3) + l] for l in range(4)]
                exp.append((i, p, x, y, pic[p][y][x], g))
        hexp = None if re[1] else (hp, hx, hy, pic[hp][hy][hx])
        # ---- apply the writes to the picture (both ports; the stages never write the same sample together)
        for i in range(2):
            if we[i]:
                p, x, y, be, data = wr[i]
                for l in range(4):
                    if be >> l & 1:
                        pic[p][y][(x & ~3) + l] = data[l]
        hist.append((it, re, [(p, x, y) for (p, x, y) in rp], [(p, x, y, be, data) for (p, x, y, be, data) in wr if True], we))
        if len(hist) > 4:
            hist.pop(0)
        await RisingEdge(dut.clk)
        await ReadOnly()
        # ---- check this cycle's reads: their data is valid after the edge that ends the cycle (one-cycle latency)
        pending = exp
        host_pending = hexp
        if pending:
            rd = int(dut.rdata.value); rd4 = int(dut.rdata4.value)
            for (i, p, x, y, es, eg) in pending:
                gs = (rd >> (i * PW)) & MASK
                gg = [(rd4 >> ((i * 4 + l) * PW)) & MASK for l in range(4)]
                if not (gs == es and gg == eg):
                    for h in hist:
                        dut._log.error(f"  cycle {h[0]}: re={h[1]} reads={h[2]} we={h[4]} writes={[(w[0], w[1], w[2], w[3], w[4]) for w in h[3]]}")
                assert gs == es and gg == eg, (f"iter {it}: port {i} plane {p} ({x},{y}): sample {gs} vs {es}, group {gg} vs {eg}")
                checked += 1
        if host_pending:
            hp_, hx_, hy_, eh = host_pending
            gh = int(dut.h_rdata.value)
            assert gh == eh, f"iter {it}: host plane {hp_} ({hx_},{hy_}): {gh} vs {eh}"
            checked += 1
        await Timer(1, "ns")
    dut._log.info(f"OK: {checked} reads matched the picture over {iters} cycles ({W}x{H})")
