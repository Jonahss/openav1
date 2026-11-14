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
    for s in ("we", "be", "wplane", "wx", "wy", "wdata", "row0", "bk_re", "bk_row", "bk_col", "ssy"):
        getattr(dut, s).value = 0
    await ClockCycles(dut.clk, 2); await Timer(1, "ns")
    stats = dict(writes=0, partial=0, blocks=0, old_rows=0, together=0)
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
            for grp in groups:
                for (p, y, g, be) in grp:
                    d = [rng.getrandbits(12) for _ in range(4)]
                    dut.we.value = 1; dut.be.value = be; dut.wplane.value = p; dut.wx.value = 4 * g + rng.randrange(4); dut.wy.value = y
                    dut.wdata.value = sum(v << (12 * i) for i, v in enumerate(d))
                    for i in range(4):
                        if (be >> i) & 1:
                            pic[p][(y, 4 * g + i)] = d[i]
                    stats["writes"] += 1; stats["partial"] += be != 0xF
                    await RisingEdge(dut.clk); await Timer(1, "ns")
                    if rng.random() < 0.1:
                        dut.we.value = 0; dut.wdata.value = rng.getrandbits(48)
                        await RisingEdge(dut.clk); await Timer(1, "ns")
            dut.we.value = 0
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
