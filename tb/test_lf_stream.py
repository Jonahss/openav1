"""The streaming deblock stage (rtl/lf_stream.sv) against lf_model on whole frames: per plane the frame is fed band by
band (superblock rows of 64 and of 128 lines), block column by block column, with the edge strengths the model uses;
every block the stage sends out must equal the model's final picture, and every sample of the plane's coded area must
come out exactly once. Input valid is dropped at random.
Env: TS_SEEDS / TS_W / TS_H / TS_FMT (generated frames, as test_lf_top), TS_IVF (frame 0 of real streams)."""
import copy
import os
import random
import sys
from pathlib import Path

import cocotb
import cocotb.utils
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge, Timer

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "tools"))
import lf_model as lfm                  # noqa: E402
import recon_model as rm                # noqa: E402
import test_tile_syntax as TT           # noqa: E402
import xcheck_frame as xf               # noqa: E402


class _Params(lfm.LoopFilter):
    """lf_model's edge decision without the filtering: what sample_filter would be called with, or None."""
    def sample_filter(self, x, y, plane, limit, blimit, thresh, dx, dy, filterSize):
        self.got = (filterSize, limit, blimit, thresh)

    def params(self, plane, pss, row, col):
        self.got = None
        self.edge(plane, pss, row, col)
        if self.got is None:
            return 0
        fs, limit, blimit, thresh = self.got
        return (1 << 29) | (fs << 24) | (limit << 16) | (blimit << 8) | thresh


def blk_of(plane_rows, y, x):
    v = 0
    for l in range(4):
        row = plane_rows[y + l]
        for c in range(4):
            v |= row[x + c] << (12 * (4 * l + c))
    return v


async def one_plane(dut, hdr, state, pre, post, plane, sb_lines, rng, tag, stats):
    sx = hdr.subsampling_x if plane else 0
    sy = hdr.subsampling_y if plane else 0
    par = _Params(hdr, state, None)
    cols = list(range(0, hdr.MiCols, 1 << sx))
    W = len(cols) * 4; H = ((hdr.MiRows * 4) >> sy)
    seen = [[0] * W for _ in range(H)]
    dut.bit_depth.value = hdr.BitDepth; dut.chroma.value = 1 if plane else 0
    band = sb_lines // 4
    out = []
    orng = random.Random(plane * 7 + sb_lines)
    stall = [0.0]

    async def monitor():
        while True:
            await RisingEdge(dut.clk)
            await Timer(2, "ns")                             # the taker is ready at random
            dut.out_ready.value = 1 if (orng.random() >= stall[0]) else 0
            await Timer(1, "ns")
            # out_valid and out_ready as the next clock edge will see them: that edge takes the block
            if int(dut.out_valid.value) and int(dut.out_ready.value):
                r = int(dut.out_row.value); r = r - 128 if r >= 64 else r
                out.append((int(dut.out_col.value), r, int(dut.out_blk.value)))
    mon = cocotb.start_soon(monitor())
    for row0 in range(0, hdr.MiRows, band):
        row1 = min(row0 + band, hdr.MiRows)
        rows = list(range(row0, row1, 1 << sy))
        y0 = (row0 * 4) >> sy
        del out[:]
        await Timer(1, "ns")
        dut.n_blk.value = len(rows); dut.n_col.value = len(cols)
        dut.first_band.value = int(row0 == 0); dut.last_band.value = int(row1 >= hdr.MiRows)
        dut.band_start.value = 1
        await RisingEdge(dut.clk); await Timer(1, "ns")
        dut.band_start.value = 0
        gap = rng.choice([0.0, 0.0, 0.2, 0.6])
        stall[0] = 0.0 if gap == 0.0 else rng.choice([0.0, 0.3, 0.7])   # unthrottled bands stay unthrottled at both ends
        t_start = cocotb.utils.get_sim_time("ns")
        for ci, col in enumerate(cols):
            for ri, row in enumerate(rows):
                while rng.random() < gap:
                    dut.in_valid.value = 0
                    await RisingEdge(dut.clk); await Timer(1, "ns")
                dut.in_valid.value = 1
                dut.in_blk.value = blk_of(pre[plane], y0 + 4 * ri, 4 * ci)
                dut.in_vpar.value = par.params(plane, 0, row, col)
                dut.in_hpar.value = par.params(plane, 1, row, col)
                while True:
                    await ReadOnly()
                    rdy = int(dut.in_ready.value)
                    await RisingEdge(dut.clk); await Timer(1, "ns")
                    if rdy:
                        break
        dut.in_valid.value = 0
        n = 0
        while True:
            await ReadOnly()
            done = int(dut.band_done.value)
            await RisingEdge(dut.clk); await Timer(1, "ns")
            n += 1
            if done:
                break
            assert n < 200000, f"{tag} plane {plane} band at MI row {row0}: no band_done"
        if gap == 0.0:                                       # input never held back: the stage's own rate
            stats["nogap_cycles"] += int((cocotb.utils.get_sim_time("ns") - t_start) // 10)
            stats["nogap_blocks"] += len(rows) * len(cols)
        await RisingEdge(dut.clk); await Timer(1, "ns")
        stats["cycles"] += n
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


async def one_frame(dut, tiles, tag, stats, rng):
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
    chroma_only = os.environ.get("LF_CHROMA_ONLY", "0") == "1"        # the LUMA = 0 build takes the chroma planes only
    for plane in range(hdr.NumPlanes):
        if (plane == 0 and not chroma_only) or (plane and hdr.loop_filter_level[1 + plane]):
            for sb in (64, 128):
                await one_plane(dut, hdr, state, pre, post, plane, sb, rng, tag, stats)
                stats["planes"] += 1
    stats["frames"] += 1
    dut._log.info(f"{tag}: identical ({stats})")


@cocotb.test()
async def lf_stream_vs_model(dut):
    seeds_env = os.environ.get("TS_SEEDS", "2,3")
    seeds = [int(x) for x in seeds_env.split(",") if x]
    W = int(os.environ.get("TS_W", "128")); H = int(os.environ.get("TS_H", "96"))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("band_start", "out_ready", "in_valid", "in_blk", "in_vpar", "in_hpar", "n_blk", "n_col", "first_band", "last_band", "bit_depth", "chroma"):
        getattr(dut, s).value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    rng = random.Random(1)
    stats = dict(frames=0, planes=0, blocks=0, filtered=0, cycles=0, skipped=0, nogap_cycles=0, nogap_blocks=0)
    ivfs = [p for p in os.environ.get("TS_IVF", "").split(",") if p]
    if ivfs:
        import obu_parser as op
        for path in ivfs:
            d = op.parse_file(path)                          # .ivf, or Argon's .obu
            await one_frame(dut, [d.tiles[i] for i in d.frames[0]], f"{Path(path).name} frame 0", stats, rng)
    else:
        for seed in seeds:
            fmt = os.environ.get("TS_FMT") or random.Random(seed * 7).choice(["420", "444", "mono", "422"])
            try:
                d, args = TT.gen(seed, W, H, fmt)
            except Exception as e:
                dut._log.info(f"seed {seed}: generation rejected ({type(e).__name__})"); stats["skipped"] += 1
                continue
            await one_frame(dut, d.tiles, f"seed {seed} {fmt} bd{args['bd']} {W}x{H}", stats, rng)
    assert stats["frames"] > 0 and stats["filtered"] > 0, stats
    if stats["nogap_blocks"]:
        stats["clocks_per_block"] = round(stats["nogap_cycles"] / stats["nogap_blocks"], 3)
    dut._log.info(f"OK: {stats}")
