"""Fuzz the unfiltered last-line copy (rtl/edge_line.sv) against a Python model: random reconstruction writes
(single lanes and groups with lane enables, lines biased towards superblock rows' last lines) and random edge reads
every clock, in each of the four (sb128, ssy) settings. Checks the contract dec_top and recon_top rely on: only writes
to a last line are kept; one copy per superblock-row parity and plane; a read is answered two cycles after the request
with sel = "the line was a last line", the aligned group and the requested lane; a read in the same cycle as a write
to the same group sees the old data. Env: EL_ITERS (default 6000 per setting), EL_SEED.
"""
import os
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, Timer

FBX = int(os.environ.get("FBX", "9"))
FBY = int(os.environ.get("FBY", "9"))
PW = 12


def lg_h(plane, sb128, ssy):
    return (7 if sb128 else 6) - (1 if (plane != 0 and ssy) else 0)


def is_last(plane, y, sb128, ssy):
    m = (1 << lg_h(plane, sb128, ssy)) - 1
    return (y & m) == m


def par(plane, y, sb128, ssy):
    return (y >> lg_h(plane, sb128, ssy)) & 1


@cocotb.test()
async def edge_line_vs_model(dut):
    iters = int(os.environ.get("EL_ITERS", "6000"))
    rng = random.Random(int(os.environ.get("EL_SEED", "1")))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.we.value = 0; dut.re.value = 0; dut.be.value = 0; dut.sb128.value = 0; dut.ssy.value = 0
    stats = {"writes_kept": 0, "writes_dropped": 0, "reads_sel": 0, "reads_other": 0, "same_cycle": 0, "cross_parity": 0}
    XG = 12                                                  # groups in play per setting: few, so reads meet earlier writes
    setting = 0
    for sb128 in (0, 1):
        for ssy in (0, 1):
            g0 = 16 * setting; setting += 1                  # each setting its own groups: nothing stale from the last one
            mem = {}                                         # (parity, plane, group) -> [4 lanes]
            last_par = {}                                    # (plane, group) -> parity written last
            pend = []                                        # (due edge, sel, group as read, lane)

            def rand_line(plane):
                h = 1 << lg_h(plane, sb128, ssy)
                row = rng.randrange((1 << FBY) // h)
                return row * h + (h - 1 if rng.random() < 0.6 else rng.randrange(h))

            for cyc in range(iters + 3):
                await RisingEdge(dut.clk)
                await ReadOnly()
                # ---- the read requested two cycles ago is on the outputs now
                while pend and pend[0][0] == cyc:
                    _, sel, exp4, lane = pend.pop(0)
                    assert int(dut.sel.value) == sel, f"edge {cyc} (sb128 {sb128} ssy {ssy}): sel {int(dut.sel.value)}, expected {sel}"
                    if sel:
                        got4 = int(dut.rdata4.value); got = int(dut.rdata.value)
                        e4 = sum(v << (PW * l) for l, v in enumerate(exp4))
                        assert got4 == e4, f"edge {cyc} (sb128 {sb128} ssy {ssy}): group {got4:012x}, expected {e4:012x}"
                        assert got == exp4[lane], f"edge {cyc}: lane {lane} sample {got:03x}, expected {exp4[lane]:03x}"
                await Timer(1, "ns")
                # ---- this cycle's requests (sampled at the next edge)
                dut.sb128.value = sb128; dut.ssy.value = ssy
                if cyc >= iters:
                    dut.we.value = 0; dut.re.value = 0
                    continue
                w = None
                if rng.random() < 0.6:
                    pl = rng.randrange(3); g = g0 + rng.randrange(XG); y = rand_line(pl)
                    if rng.random() < 0.3:
                        lane = rng.randrange(4); be = 1 << lane; x = 4 * g + lane        # a single-sample write
                    else:
                        be = rng.randrange(1, 16); x = 4 * g
                    data = [rng.randrange(1 << PW) for _ in range(4)]
                    dut.we.value = 1; dut.be.value = be; dut.wplane.value = pl; dut.wx.value = x; dut.wy.value = y
                    dut.wdata.value = sum(v << (PW * l) for l, v in enumerate(data))
                    w = (pl, g, y, be, data)
                else:
                    dut.we.value = 0
                if rng.random() < 0.7:
                    pl = rng.randrange(3); g = g0 + rng.randrange(XG); y = rand_line(pl); lane = rng.randrange(4)
                    dut.re.value = 1; dut.rplane.value = pl; dut.rx.value = 4 * g + lane; dut.ry.value = y
                    sel = 1 if is_last(pl, y, sb128, ssy) else 0
                    key = (par(pl, y, sb128, ssy), pl, g)
                    exp4 = list(mem.get(key, [0, 0, 0, 0]))  # as before this cycle's write: read-first
                    pend.append((cyc + 2, sel, exp4, lane))
                    stats["reads_sel" if sel else "reads_other"] += 1
                    if sel and w and is_last(w[0], w[2], sb128, ssy) and (par(w[0], w[2], sb128, ssy), w[0], w[1]) == key:
                        stats["same_cycle"] += 1
                    if sel and last_par.get((pl, g)) not in (None, key[0]):
                        stats["cross_parity"] += 1           # the other parity was written last: its data must not show
                else:
                    dut.re.value = 0
                if w:
                    pl, g, y, be, data = w
                    if is_last(pl, y, sb128, ssy):
                        key = (par(pl, y, sb128, ssy), pl, g)
                        cur = mem.setdefault(key, [0, 0, 0, 0])
                        for l in range(4):
                            if be >> l & 1:
                                cur[l] = data[l]
                        last_par[(pl, g)] = key[0]
                        stats["writes_kept"] += 1
                    else:
                        stats["writes_dropped"] += 1
            assert not pend
    assert stats["reads_sel"] > 1000 and stats["same_cycle"] > 20 and stats["cross_parity"] > 100 and stats["writes_dropped"] > 100, stats
    dut._log.info(f"OK: {stats}")
