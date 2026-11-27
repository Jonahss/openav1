"""rtl/fc_rstore.sv (the chain's row store: a block-organised mirror of reconstruction's frame-memory writes) against
a plain picture kept by the test. Superblock rows are written one after another as reconstruction writes them
(aligned groups of four with lane enables: whole groups, single samples, pairs; samples written twice), in a random
order inside the row and for all planes; after each row random blocks of that row and of the rows the ring still
holds are read back through the band interface (row0 + bk_row) and compared, all planes at once and one at a time.
Env: RS_SEED, RS_ROWS (superblock rows, default 9: more than the ring), RS_W (luma width, default 144)."""
import os
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge, Timer


@cocotb.test()
async def fc_rstore_vs_picture(dut):
    rng = random.Random(int(os.environ.get("RS_SEED", "1")))
    n_sb = int(os.environ.get("RS_ROWS", "9")); W = int(os.environ.get("RS_W", "144"))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("we", "be", "wplane", "wx", "wy", "wdata", "we_b", "we_c", "we_d", "row0", "bk_re", "bk_row", "bk_col", "ssy"):
        getattr(dut, s).value = 0
    await ClockCycles(dut.clk, 2); await Timer(1, "ns")
    stats = dict(writes=0, partial=0, blocks=0, old_rows=0, together=0, second=0, lane=0, lane_with_luma=0)
    for ssy, sb in ((1, 64), (0, 64), (1, 128)):
        dut.ssy.value = ssy
        ssx = 1                                               # the store does not care about the chroma width
        pw = [W, W >> ssx, W >> ssx]; sbh = [sb, sb >> ssy, sb >> ssy]
        ring = [64 * 4, 32 * 4, 32 * 4]                       # lines the ring holds (RBY = 6, RBC = 5)
        pic = [dict() for _ in range(3)]                      # (y, x) -> sample
        for r in range(n_sb):
            todo = []
            for p in range(3):
                for y in range(r * sbh[p], (r + 1) * sbh[p]):
                    for g in range(pw[p] // 4):
                        kind = rng.random()
                        if kind < 0.7:
                            todo.append((p, y, g, 0xF))
                        elif kind < 0.85:
                            todo.append((p, y, g, 0x3)); todo.append((p, y, g, 0xC))
                        else:
                            for i in rng.sample(range(4), 4):
                                todo.append((p, y, g, 1 << i))
                        if rng.random() < 0.1:
                            todo.append((p, y, g, rng.randrange(1, 16)))   # a second write over some of the samples
            # reconstruction's order is not raster: shuffle, keeping the writes to one group in their order
            keyed = {}
            for i, t in enumerate(todo):
                keyed.setdefault(t[:3], []).append(t)
            groups = list(keyed.values()); rng.shuffle(groups)
            # Two streams of writes a clock: reconstruction's (port a, with its second word b) and the chroma lane's
            # (port c, with its second word d: chroma planes only). In a cycle in which the lane writes, port a writes
            # luma or nothing; chroma groups also come through port a in cycles without the lane.
            lum = [t for grp in groups for t in grp if t[0] == 0]
            chr_ = [t for grp in groups for t in grp if t[0] != 0]

            def drive(t, first, second):
                """One write on a port pair: first = (we, be, plane, x, y, data) signal names, second likewise without be."""
                (p, y, g, be) = t
                d = [rng.getrandbits(12) for _ in range(4)]
                getattr(dut, first[0]).value = 1; getattr(dut, first[1]).value = be; getattr(dut, first[2]).value = p
                getattr(dut, first[3]).value = 4 * g + rng.randrange(4); getattr(dut, first[4]).value = y
                getattr(dut, first[5]).value = sum(v << (12 * i) for i, v in enumerate(d))
                for i in range(4):
                    if (be >> i) & 1:
                        pic[p][(y, 4 * g + i)] = d[i]
                stats["writes"] += 1; stats["partial"] += be != 0xF
                if g + 1 < pw[p] // 4 and rng.random() < 0.3:          # the second write of a cycle: the next group, all lanes
                    d2 = [rng.getrandbits(12) for _ in range(4)]
                    getattr(dut, second[0]).value = 1; getattr(dut, second[2]).value = 4 * (g + 1); getattr(dut, second[4]).value = sum(v << (12 * i) for i, v in enumerate(d2))
                    if second[1]:
                        getattr(dut, second[1]).value = p; getattr(dut, second[3]).value = y
                    for i in range(4):
                        pic[p][(y, 4 * (g + 1) + i)] = d2[i]
                    stats["second"] += 1

            A = ("we", "be", "wplane", "wx", "wy", "wdata"); B = ("we_b", "wplane_b", "wx_b", "wy_b", "wdata_b")
            C = ("we_c", "be_c", "wplane_c", "wx_c", "wy_c", "wdata_c"); D = ("we_d", None, "wx_d", None, "wdata_d")
            while lum or chr_:
                for sig in ("we", "we_b", "we_c", "we_d"):
                    getattr(dut, sig).value = 0
                if chr_ and (not lum or rng.random() < 0.5):
                    drive(chr_.pop(0), C, D); stats["lane"] += 1
                    if lum and rng.random() < 0.7:
                        drive(lum.pop(0), A, B); stats["lane_with_luma"] += 1
                else:
                    drive((lum or chr_).pop(0), A, B)
                await RisingEdge(dut.clk); await Timer(1, "ns")
                if rng.random() < 0.1:
                    for sig in ("we", "we_b", "we_c", "we_d"):
                        getattr(dut, sig).value = 0
                    dut.wdata.value = rng.getrandbits(48)
                    await RisingEdge(dut.clk); await Timer(1, "ns")
            dut.we_c.value = 0; dut.we_d.value = 0
            dut.we.value = 0; dut.we_b.value = 0
            # one clock for the write register to empty: the chain reads a band only after its reconstruction is over, and
            # the store stops the simulation on a read of a word still in that register
            await RisingEdge(dut.clk); await Timer(1, "ns")
            # read back: bands = this row and older rows that are still inside every plane's ring
            for back in range(0, 4):
                rr = r - back
                if rr < 0 or any((r + 1) * sbh[p] - rr * sbh[p] > ring[p] for p in range(3)):
                    continue
                dut.row0.value = (rr * sb) // 4
                for _ in range(40):
                    which = rng.choice([[0], [1], [2], [0, 1, 2], [0, 1, 2]])
                    req = {}
                    for p in which:
                        req[p] = (rng.randrange(sbh[p] // 4), rng.randrange(pw[p] // 4))
                    dut.bk_re.value = sum(1 << p for p in which)
                    dut.bk_row.value = sum(req[p][0] << (6 * p) for p in which)
                    cw = len(dut.bk_col) // 3
                    dut.bk_col.value = sum(req[p][1] << (cw * p) for p in which)
                    await RisingEdge(dut.clk); await Timer(1, "ns")
                    dut.bk_re.value = 0
                    if rng.random() < 0.3:                     # the data is held while no read is asked, whatever the address
                        dut.bk_row.value = rng.getrandbits(18); dut.bk_col.value = rng.getrandbits(cw * 3)
                        await RisingEdge(dut.clk); await Timer(1, "ns")
                    got = int(dut.bk_data.value)
                    for p in which:
                        br, bc = req[p]
                        y0 = rr * sbh[p] + 4 * br
                        v = (got >> (192 * p)) & ((1 << 192) - 1)
                        for l in range(4):
                            for c in range(4):
                                g = (v >> (12 * (4 * l + c))) & 0xFFF
                                e = pic[p][(y0 + l, 4 * bc + c)]
                                assert g == e, f"sb{sb} ssy{ssy} plane {p} superblock row {rr} (read after row {r}): sample ({y0 + l}, {4 * bc + c}) = {g}, written {e}"
                        stats["blocks"] += 1
                    stats["old_rows"] += back > 0; stats["together"] += len(which) == 3
    assert stats["old_rows"] > 50 and stats["together"] > 50 and stats["partial"] > 1000, stats
    dut._log.info(f"OK: {stats}")
