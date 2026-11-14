"""rtl/fc_out.sv (the chain's line segments into two frame-memory write ports) against a plain picture: bands of
random segments for three planes (two groups for luma, one or two for chroma, lines above the band's first line),
sent as the chain sends them (a plane stops within OUT_SLACK = 16 segments of its out_hold rising, here a random
number of them); every group must be written exactly once, to the right plane, place and with the right samples,
and the hold must keep every FIFO from overflowing.
Env: FO_SEED, FO_BANDS (default 30)."""
import os
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge, Timer


@cocotb.test()
async def fc_out_vs_picture(dut):
    rng = random.Random(int(os.environ.get("FO_SEED", "1")))
    bands = int(os.environ.get("FO_BANDS", "30"))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("y0", "in_valid", "in_line", "in_n4", "in_col", "in_y"):
        getattr(dut, s).value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk); await Timer(1, "ns")
    cw = len(dut.in_col) // 3; fby = len(dut.y0) // 3; fbx = len(dut.wx) // 2
    expect = {}                                               # (plane, y, x4) -> 48-bit group, to be written once
    stats = dict(segments=0, groups=0, both_ports=0, hold_clocks=0)
    written = [0]

    async def monitor():
        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()
            w = int(dut.we.value)
            if w:
                pl = int(dut.wplane.value); x = int(dut.wx.value); y = int(dut.wy.value); d = int(dut.wdata.value)
                for k in range(2):
                    if (w >> k) & 1:
                        key = ((pl >> (2 * k)) & 3, (y >> (fby * k)) & ((1 << fby) - 1), (x >> (fbx * k)) & ((1 << fbx) - 1))
                        assert key in expect, f"a group written that was not sent, or twice: plane {key[0]} line {key[1]} x {key[2]}"
                        got = (d >> (48 * k)) & ((1 << 48) - 1)
                        assert got == expect[key], f"plane {key[0]} line {key[1]} x {key[2]}: written {got:012x}, sent {expect[key]:012x}"
                        del expect[key]; written[0] += 1
                stats["both_ports"] += w == 3
            assert int(dut.err_ovf.value) == 0, "a segment was dropped at a FIFO"
    cocotb.start_soon(monitor())
    for b in range(bands):
        ssx = rng.random() < 0.6
        y0 = [rng.randrange(40, 900) & ~3 for _ in range(3)]
        dut.y0.value = sum(v << (fby * p) for p, v in enumerate(y0))
        todo = []
        for p in range(3):
            n4 = 2 if (p == 0 or not ssx) else 1
            segs = []
            for c in range(0, rng.choice([8, 24, 60]), n4):
                for yr in range(rng.randint(-19, -1), rng.randint(8, 70)):
                    segs.append((c, yr, n4, rng.getrandbits(48 * n4)))
            todo.append(segs)
        idx = [0, 0, 0]; slack = [None, None, None]; rate = [rng.choice([1.0, 1.0, 0.5, 0.15]) for _ in range(3)]
        while any(idx[p] < len(todo[p]) for p in range(3)):
            await ReadOnly()
            hold = int(dut.out_hold.value)
            await Timer(1, "ns")
            v = 0; line = 0; n4s = 0; col = 0; yy = 0
            for p in range(3):
                if (hold >> p) & 1:
                    if slack[p] is None:
                        slack[p] = rng.randint(0, 16)         # the chain sends up to OUT_SLACK more
                    stats["hold_clocks"] += 1
                else:
                    slack[p] = None
                can = slack[p] is None or slack[p] > 0
                if idx[p] < len(todo[p]) and can and rng.random() < rate[p]:
                    c, yr, n4, d = todo[p][idx[p]]; idx[p] += 1
                    if slack[p] is not None:
                        slack[p] -= 1
                    v |= 1 << p; line |= d << (96 * p); n4s |= n4 << (2 * p); col |= c << (cw * p); yy |= (yr & 0x1FF) << (9 * p)
                    for g in range(n4):
                        key = (p, y0[p] + yr, 4 * c + 4 * g)
                        assert key not in expect
                        expect[key] = (d >> (48 * g)) & ((1 << 48) - 1)
                    stats["segments"] += 1; stats["groups"] += n4
            dut.in_valid.value = v; dut.in_line.value = line; dut.in_n4.value = n4s; dut.in_col.value = col; dut.in_y.value = yy
            await RisingEdge(dut.clk)
        await Timer(1, "ns")
        dut.in_valid.value = 0
        n = 0
        while expect:                                          # y0 may change only when the band's segments are all in: they are
            await RisingEdge(dut.clk); n += 1
            assert n < 20000, f"{len(expect)} groups never written"
        await Timer(1, "ns")
        assert int(dut.busy.value) == 0
    assert written[0] == stats["groups"] and stats["both_ports"] > 1000 and stats["hold_clocks"] > 100, stats
    dut._log.info(f"OK: {stats}")
