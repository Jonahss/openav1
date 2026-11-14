"""The streaming loop restoration stage (rtl/lr_stream.sv) against lr_model on whole pictures: made-up pictures (a
CDEF plane and a deblocked plane per plane, any width and height, 4:2:0 / 4:4:4 / 4:2:2 / mono, 8 / 10 / 12 bit) with a
made-up map of restoration units (Wiener with coefficients in the bitstream's ranges, or none; unit sizes as the
header allows). The CDEF plane is fed band by band in the order cdef_stream sends it (line segments column by
column), the test answers the stage's unit reads and its reads of the deblocked lines around the stripe boundaries,
and every line segment the stage sends must equal the model's picture: every sample of the coded area exactly once.
The arithmetic is fuzzed in test_lr_wiener; this test is about the stage's structure (stripes, units, picture
borders, bands, the column window).
Env: LS_ITERS (pictures, default 10), LS_SEED, LS_W / LS_H (fix the picture's size), LS_FMT."""
import os
import random
import types

import cocotb
import cocotb.utils
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, ReadOnly, RisingEdge, Timer

import lr_model as lrm

SBW = 9


def make_picture(rng):
    bd = rng.choice([8, 10, 12]); mx = (1 << bd) - 1
    fmt = os.environ.get("LS_FMT") or rng.choice(["420", "420", "444", "422", "mono"])
    sx = 1 if fmt in ("420", "422") else 0
    sy = 1 if fmt == "420" else 0
    W = int(os.environ.get("LS_W", "0")) or rng.choice([rng.randint(9, 40), rng.randint(41, 150), rng.choice([64, 72, 128, 136])])
    # heights: any, multiples of the superblock, and planes that end within a few lines of a stripe boundary (56 mod 64
    # in luma, 28 mod 32 in subsampled chroma), where the lines below a stripe are clamped to the plane
    near = rng.choice([64 * rng.randint(1, 4) - 8 + rng.randint(-2, 4), 2 * (32 * rng.randint(1, 4) - 4 + rng.randint(-2, 4))])
    H = int(os.environ.get("LS_H", "0")) or rng.choice([rng.randint(9, 70), rng.randint(71, 300), rng.choice([64, 120, 128, 184, 192, 248, 256]), near, near])
    mi_cols = 2 * ((W + 7) >> 3); mi_rows = 2 * ((H + 7) >> 3)
    n = 1 if fmt == "mono" else 3
    us = rng.choice([64, 64, 128, 256])
    uv = us >> (1 if (sx and sy and rng.random() < 0.5 and us > 64) or (sx and sy and us == 64 and rng.random() < 0.5) else 0)
    hdr = types.SimpleNamespace(BitDepth=bd, FrameHeight=H, UpscaledWidth=W, subsampling_x=sx, subsampling_y=sy, NumPlanes=n,
                                LoopRestorationSize=[us, uv, uv], FrameRestorationType=[1 if rng.random() < 0.9 else 0 for _ in range(3)],
                                MiRows=mi_rows, MiCols=mi_cols)
    cdef = []; dbl = []; lr_type = {}; wien = {}
    for p in range(n):
        psx = sx if p else 0; psy = sy if p else 0
        cw = (mi_cols * 4) >> psx; ch = (mi_rows * 4) >> psy
        base = rng.randint(0, mx); nz = rng.choice([4, 20, 80, 300]) << (bd - 8)
        if rng.random() < 0.1:
            c = [[rng.choice([0, mx]) for _ in range(cw)] for _ in range(ch)]
        else:
            c = [[max(0, min(mx, base + rng.randint(-nz, nz))) for _ in range(cw)] for _ in range(ch)]
        dz = rng.choice([1, 8, 60]) << (bd - 8)
        d = [[max(0, min(mx, v + rng.randint(-dz, dz))) for v in row] for row in c]
        cdef.append(c); dbl.append(d)
        usz = hdr.LoopRestorationSize[p]
        urows = lrm.count_units_in_frame(usz, lrm.round2(H, psy)); ucols = lrm.count_units_in_frame(usz, lrm.round2(W, psx))
        for ur in range(urows):
            for uc in range(ucols):
                k = (p, ur, uc)
                lr_type[k] = 1 if rng.random() < 0.75 else 0
                wien[k] = [[rng.randint(-5, 10), rng.randint(-23, 8), rng.randint(-17, 46)] for _ in range(2)]
    state = types.SimpleNamespace(LrType=lr_type, LrWiener=wien)
    return hdr, state, dbl, cdef, fmt


def pack_line(row, x, n):
    v = 0
    for i in range(n):
        if x + i < len(row):
            v |= row[x + i] << (12 * i)
    return v


async def one_plane(dut, hdr, state, dbl, cdef, post, p, sb_lines, rng, tag, stats):
    sx = hdr.subsampling_x if p else 0; sy = hdr.subsampling_y if p else 0
    cw = (hdr.MiCols * 4) >> sx; ch = (hdr.MiRows * 4) >> sy
    cols = cw // 4
    segw = 4 if sx else 8; nseg = (cw + segw - 1) // segw
    usz = hdr.LoopRestorationSize[p]
    pl_w = lrm.round2(hdr.UpscaledWidth, sx); pl_h = lrm.round2(hdr.FrameHeight, sy)
    urows = lrm.count_units_in_frame(usz, pl_h); ucols = lrm.count_units_in_frame(usz, pl_w)
    en = hdr.FrameRestorationType[p] != 0
    dut.en.value = int(en); dut.plane.value = p; dut.ssx.value = sx; dut.ssy.value = sy
    dut.pl_w.value = pl_w; dut.pl_h.value = pl_h; dut.u_log2.value = usz.bit_length() - 1
    dut.u_rows.value = urows; dut.u_cols.value = ucols
    seen = [[0] * cw for _ in range(ch)]
    out = []
    band = sb_lines // 4
    sh_stripe = 64 >> sy; so = 8 >> sy

    async def monitor():
        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.out_valid.value):
                y = int(dut.out_y.value); y = y - 512 if y >= 256 else y
                out.append((int(dut.out_col.value), y, int(dut.out_line.value), int(dut.out_n4.value)))

    async def stores():                                      # registered reads: the address of one clock is answered in the next
        while True:
            await FallingEdge(dut.clk)
            pl = int(dut.lrr_plane.value); ur = int(dut.lrr_row.value); uc = int(dut.lrr_col.value)
            k = int(dut.bl_k.value); c4 = int(dut.bl_col.value)
            await RisingEdge(dut.clk)
            await Timer(1, "ns")
            key = (pl, ur, uc)
            t = state.LrType.get(key, 0); w = state.LrWiener.get(key, [[0, 0, 0], [0, 0, 0]])
            wv = sum((v & 0x7F) << (7 * i) for i, v in enumerate(w[0])) | (sum((v & 0x7F) << (7 * i) for i, v in enumerate(w[1])) << 21)
            stats["unit_reads"] += 1
            dut.lrr_rec.value = (t << 62) | (wv << 20) | rng.getrandbits(20)
            b = k * sh_stripe - so
            v = 0
            for i in range(4):
                yy = b - 2 + i
                if 0 <= yy < ch:
                    v |= pack_line(dbl[p][yy], c4 * 4, segw) << (96 * i)
                else:
                    v |= rng.getrandbits(96) << (96 * i)       # never a line of the picture: must not be used
            dut.bl_rd.value = v
    mon = cocotb.start_soon(monitor()); st = cocotb.start_soon(stores())
    sh = 12 if sy else 16
    for row0 in range(0, hdr.MiRows, band):
        row1 = min(row0 + band, hdr.MiRows)
        n_blk = len(range(row0, row1, 1 << sy))
        first = row0 == 0; last = row1 >= hdr.MiRows
        y0 = (row0 * 4) >> sy
        ya = 0 if first else y0 - sh
        yb = ch if last else y0 + n_blk * 4 - sh
        del out[:]
        await Timer(2, "ns")
        dut.row0.value = row0 >> sy; dut.pic_rows.value = ch // 4; dut.n_blk.value = n_blk; dut.n_col.value = cols
        dut.first_band.value = int(first); dut.last_band.value = int(last)
        dut.band_start.value = 1
        await RisingEdge(dut.clk); await Timer(2, "ns")
        dut.band_start.value = 0
        gap = rng.choice([0.0, 0.0, 0.3])
        t_start = cocotb.utils.get_sim_time("ns")
        for ci in range(nseg):
            for y in range(ya, yb):
                while rng.random() < gap:
                    dut.in_valid.value = 0
                    await RisingEdge(dut.clk); await Timer(2, "ns")
                dut.in_valid.value = 1
                dut.in_line.value = pack_line(cdef[p][y], ci * segw, segw)
                dut.in_y.value = (y - y0) & 0x1FF
                n = 0
                while True:
                    await ReadOnly()
                    rdy = int(dut.in_ready.value)
                    await RisingEdge(dut.clk); await Timer(2, "ns")
                    if rdy:
                        break
                    n += 1
                    assert n < 100000, f"{tag} band at line {y0}: the stage stopped taking lines at column {ci} line {y}"
        dut.in_valid.value = 0
        n = 0
        while True:
            await ReadOnly()
            done = int(dut.band_done.value)
            await RisingEdge(dut.clk); await Timer(2, "ns")
            n += 1
            if done:
                break
            assert n < 400000, f"{tag} band at line {y0}: no band_done"
        if gap == 0.0:
            stats["nogap_clocks"] += int((cocotb.utils.get_sim_time("ns") - t_start) // 10)
            stats["nogap_lines"] += nseg * (yb - ya)
        await ClockCycles(dut.clk, 2); await Timer(2, "ns")
        exp_lo = 0 if first else ya - 3
        exp_hi = ch if last else yb - 3
        for (c4, yr, v, n4) in out:
            y = y0 + yr
            assert exp_lo <= y < exp_hi and 0 <= c4 * 4 < cw, f"{tag} plane {p} sb{sb_lines}: line {y} 4x4 column {c4} sent in the band of lines {exp_lo}..{exp_hi - 1}"
            for k in range(4 * n4):
                if 4 * c4 + k >= cw:
                    continue
                got = (v >> (12 * k)) & 0xFFF
                exp = post[p][y][4 * c4 + k]
                assert got == exp, (f"{tag} plane {p} sb{sb_lines}: sample ({y}, {4 * c4 + k}) = {got}, model {exp} (cdef {cdef[p][y][4 * c4 + k]}; "
                                    f"plane {pl_w}x{pl_h} of {cw}x{ch}, unit {usz}, band lines {ya}..{yb - 1})")
                seen[y][4 * c4 + k] += 1
        stats["lines"] += len(out)
    mon.cancel(); st.cancel()
    missing = sum(1 for y in range(ch) for x in range(cw) if seen[y][x] != 1)
    assert missing == 0, f"{tag} plane {p} sb{sb_lines}: {missing} samples not sent out exactly once"
    stats["changed"] += sum(1 for y in range(ch) for x in range(cw) if post[p][y][x] != cdef[p][y][x])
    stats["planes"] += 1
    stats["planes_off"] += not en
    stats["stripes"] += (pl_h + so + sh_stripe - 1) // sh_stripe
    stats["odd_w"] += pl_w != cw
    stats["odd_h"] += pl_h != ch
    stats["end_below_stripe"] += (pl_h + so) % sh_stripe == 2      # the plane's last line is a stripe's second: the line below it is clamped


@cocotb.test()
async def lr_stream_vs_model(dut):
    iters = int(os.environ.get("LS_ITERS", "10"))
    rng = random.Random(int(os.environ.get("LS_SEED", "1")))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("band_start", "in_valid", "in_line", "in_y", "row0", "pic_rows", "n_blk", "n_col", "first_band", "last_band", "bit_depth", "en", "plane",
              "ssx", "ssy", "pl_w", "pl_h", "u_log2", "u_rows", "u_cols", "lrr_rec", "bl_rd"):
        getattr(dut, s).value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    stats = dict(pictures=0, planes=0, planes_off=0, lines=0, changed=0, stripes=0, odd_w=0, odd_h=0, end_below_stripe=0, unit_reads=0, nogap_clocks=0, nogap_lines=0)
    for it in range(iters):
        hdr, state, dbl, cdef, fmt = make_picture(rng)
        m = lrm.LoopRestoration(hdr, state, dbl, cdef)
        post = m.apply()
        tag = f"picture {it} {fmt} bd{hdr.BitDepth} {hdr.UpscaledWidth}x{hdr.FrameHeight} units {hdr.LoopRestorationSize}"
        dut.bit_depth.value = hdr.BitDepth
        await ClockCycles(dut.clk, 8)
        for p in range(hdr.NumPlanes):
            for sb in (64, 128):
                await one_plane(dut, hdr, state, dbl, cdef, post, p, sb, rng, tag, stats)
        stats["pictures"] += 1
        dut._log.info(f"{tag}: identical")
    del stats["unit_reads"]
    assert stats["changed"] > 0 and stats["odd_w"] > 0 and stats["odd_h"] > 0 and stats["end_below_stripe"] > 0, stats
    if stats["nogap_lines"]:
        stats["clocks_per_line"] = round(stats["nogap_clocks"] / stats["nogap_lines"], 3)
    dut._log.info(f"OK: {stats}")
