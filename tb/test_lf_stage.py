"""The deblock stage as it will sit in the decoder (rtl/lf_stage_tb_top.sv: mi_store + a block memory + lf_feed +
lf_stream) against lf_model on whole frames. Unlike test_lf_stream nothing is supplied but what the decoder has: the
frame header, the mode info and transform sizes in mi_store, and the band's pre-filter blocks; the edge strengths are
the RTL's own (lf_edge inside lf_feed). Per plane the frame goes band by band (64- and 128-line superblock rows);
every block sent out must equal the model's final picture and every sample must come out exactly once.
Env: TS_SEEDS / TS_W / TS_H / TS_FMT (generated frames), TS_IVF (frame 0 of real streams, .ivf or .obu)."""
import copy
import os
import random
import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge, Timer

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "tools"))
import lf_model as lfm                  # noqa: E402
import recon_model as rm                # noqa: E402
import test_tile_syntax as TT           # noqa: E402
import xcheck_frame as xf               # noqa: E402
import lf_edge_vec as lv                # noqa: E402
from host_regs import LF_FIELDS, lf_vals   # noqa: E402


def blk_of(plane_rows, y, x):
    v = 0
    for l in range(4):
        row = plane_rows[y + l]
        for c in range(4):
            v |= row[x + c] << (12 * (4 * l + c))
    return v


async def load_mi(dut, hdr, state):
    await Timer(1, "ns")
    for r in range(hdr.MiRows):
        for c in range(hdr.MiCols):
            dut.blk_we.value = 1; dut.blk_r.value = r; dut.blk_c.value = c; dut.blk_bw4.value = 1; dut.blk_bh4.value = 1
            dut.blk_data.value = lv.mi_word(state, r, c)
            await RisingEdge(dut.clk); await Timer(1, "ns")
            dut.blk_we.value = 0
            await RisingEdge(dut.clk); await Timer(1, "ns")
    for plane in range(hdr.NumPlanes):
        lts = state.LoopfilterTxSizes[plane]
        for r in range(len(lts)):
            for c in range(len(lts[0])):
                dut.tx_we.value = 1; dut.tx_plane.value = plane; dut.tx_row.value = r; dut.tx_col.value = c
                dut.tx_w4.value = 1; dut.tx_h4.value = 1; dut.tx_sz.value = lts[r][c]
                await RisingEdge(dut.clk); await Timer(1, "ns")
                dut.tx_we.value = 0
                await RisingEdge(dut.clk); await Timer(1, "ns")


async def one_plane(dut, hdr, pre, post, plane, sb_lines, tag, stats):
    sx = hdr.subsampling_x if plane else 0
    sy = hdr.subsampling_y if plane else 0
    cols = list(range(0, hdr.MiCols, 1 << sx))
    W = len(cols) * 4; H = ((hdr.MiRows * 4) >> sy)
    seen = [[0] * W for _ in range(H)]
    dut.plane.value = plane
    band = sb_lines // 4
    out = []

    async def monitor():
        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.out_valid.value):
                r = int(dut.out_row.value); r = r - 128 if r >= 64 else r
                out.append((int(dut.out_col.value), r, int(dut.out_blk.value)))
    mon = cocotb.start_soon(monitor())
    for row0 in range(0, hdr.MiRows, band):
        row1 = min(row0 + band, hdr.MiRows)
        rows = list(range(row0, row1, 1 << sy))
        y0 = (row0 * 4) >> sy
        await Timer(1, "ns")
        for ci in range(len(cols)):                          # the band's blocks into the row store's stand-in
            for ri in range(len(rows)):
                dut.bm_we.value = 1; dut.bm_row.value = ri; dut.bm_col.value = ci
                dut.bm_data.value = blk_of(pre[plane], y0 + 4 * ri, 4 * ci)
                await RisingEdge(dut.clk); await Timer(1, "ns")
        dut.bm_we.value = 0
        del out[:]
        dut.row0.value = row0; dut.n_blk.value = len(rows); dut.n_col.value = len(cols)
        dut.first_band.value = int(row0 == 0); dut.last_band.value = int(row1 >= hdr.MiRows)
        dut.band_start.value = 1
        await RisingEdge(dut.clk); await Timer(1, "ns")
        dut.band_start.value = 0
        n = 0
        while True:
            await ReadOnly()
            done = int(dut.band_done.value)
            await RisingEdge(dut.clk); await Timer(1, "ns")
            n += 1
            if done:
                break
            assert n < 400000, f"{tag} plane {plane} band at MI row {row0}: no band_done"
        await RisingEdge(dut.clk); await Timer(1, "ns")
        stats["cycles"] += n; stats["in_blocks"] += len(rows) * len(cols)
        for (c, r, v) in out:
            y = y0 + 4 * r
            assert 0 <= y < H and 0 <= c < len(cols), f"{tag} plane {plane}: block ({c}, row {r}) of the band at line {y0} is outside the plane"
            for l in range(4):
                for k in range(4):
                    got = (v >> (12 * (4 * l + k))) & 0xFFF
                    exp = post[plane][y + l][4 * c + k]
                    assert got == exp, (f"{tag} plane {plane} sb{sb_lines}: sample ({y + l}, {4 * c + k}) = {got}, model {exp} "
                                        f"(pre-filter {pre[plane][y + l][4 * c + k]}; block column {c}, row {r} of the band at line {y0})")
                    seen[y + l][4 * c + k] += 1
        stats["blocks"] += len(out)
    mon.cancel()
    missing = sum(1 for y in range(H) for x in range(W) if seen[y][x] != 1)
    assert missing == 0, f"{tag} plane {plane} sb{sb_lines}: {missing} samples not sent out exactly once"
    stats["filtered"] += sum(1 for y in range(H) for x in range(W) if post[plane][y][x] != pre[plane][y][x])


async def one_frame(dut, tiles, tag, stats):
    frame = {}; decs = []
    for th, data in tiles:
        dec = rm.FrameRecon(th, data, frame); dec.decode_tile(); decs.append(dec)
    hdr = tiles[0][0]
    state = xf.FrameState(hdr)
    for _d in decs:
        state.add(_d)
    pre = frame["planes"]
    post = copy.deepcopy(pre)
    lfm.LoopFilter(hdr, state, post).apply()
    if hdr.loop_filter_level[0] == 0 and hdr.loop_filter_level[1] == 0:
        stats["skipped"] += 1
        return
    dut.hdr.value = TT.pack(TT.HDR_FIELDS, TT.hdr_vals(hdr, decs[0]))
    dut.lh.value = TT.pack(LF_FIELDS, lf_vals(hdr))
    await load_mi(dut, hdr, state)
    for plane in range(hdr.NumPlanes):
        if plane == 0 or hdr.loop_filter_level[1 + plane]:
            for sb in (64, 128):
                await one_plane(dut, hdr, pre, post, plane, sb, tag, stats)
                stats["planes"] += 1
    stats["frames"] += 1
    dut._log.info(f"{tag}: identical ({stats})")


@cocotb.test()
async def lf_stage_vs_model(dut):
    seeds = [int(x) for x in os.environ.get("TS_SEEDS", "2,3").split(",") if x]
    W = int(os.environ.get("TS_W", "128")); H = int(os.environ.get("TS_H", "96"))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("band_start", "bm_we", "bm_row", "bm_col", "bm_data", "blk_we", "tx_we", "hdr", "lh", "plane", "row0", "n_blk", "n_col",
              "first_band", "last_band", "blk_r", "blk_c", "blk_bw4", "blk_bh4", "blk_data", "tx_plane", "tx_row", "tx_col", "tx_w4", "tx_h4", "tx_sz"):
        getattr(dut, s).value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    stats = dict(frames=0, planes=0, blocks=0, in_blocks=0, filtered=0, cycles=0, skipped=0)
    ivfs = [p for p in os.environ.get("TS_IVF", "").split(",") if p]
    if ivfs:
        import obu_parser as op
        for path in ivfs:
            d = op.parse_file(path)
            await one_frame(dut, [d.tiles[i] for i in d.frames[0]], f"{Path(path).name} frame 0", stats)
    else:
        for seed in seeds:
            fmt = os.environ.get("TS_FMT") or random.Random(seed * 7).choice(["420", "444", "mono", "422"])
            try:
                d, args = TT.gen(seed, W, H, fmt)
            except Exception as e:
                dut._log.info(f"seed {seed}: generation rejected ({type(e).__name__})"); stats["skipped"] += 1
                continue
            await one_frame(dut, d.tiles, f"seed {seed} {fmt} bd{args['bd']} {W}x{H}", stats)
    assert stats["frames"] > 0 and stats["filtered"] > 0, stats
    stats["clocks_per_block"] = round(stats["cycles"] / stats["in_blocks"], 3)
    dut._log.info(f"OK: {stats}")
