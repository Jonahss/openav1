"""The streaming CDEF stage (rtl/cdef_stream.sv, luma) against cdef_model on whole frames: the deblocked picture
(lf_model) is fed band by band in the order the deblock stage sends it (4x4 blocks column by column, the band shifted
up by 8 lines), the test answers the stage's skip and cdef_idx reads from the frame's state, and every line segment
the stage sends must equal the model's CDEF picture, every luma sample of the coded area exactly once.
Env: TS_IVF (frame 0 of real streams; generated frames rarely exercise CDEF), TS_SEEDS / TS_W / TS_H / TS_FMT,
TS_CDEF_VEC=<dir> (frames dumped by a decoder test bench run with TS_CDEF_VEC_OUT=<dir>: inter frames, tb/cdef_vec.py)."""
import copy
import os
import random
import sys
from pathlib import Path

import cocotb
import cocotb.utils
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, ReadOnly, RisingEdge, Timer

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "tools"))
import cdef_model as cdm                # noqa: E402
import lf_model as lfm                  # noqa: E402
import recon_model as rm                # noqa: E402
import test_tile_syntax as TT           # noqa: E402
import xcheck_frame as xf               # noqa: E402


def blk_of(plane_rows, y, x):
    v = 0
    for l in range(4):
        row = plane_rows[y + l]
        for c in range(4):
            v |= row[x + c] << (12 * (4 * l + c))
    return v


async def one_plane(dut, hdr, state, dbl, post, sb_lines, rng, tag, stats, plane=0, ydir=None):
    sx = getattr(hdr, "subsampling_x", 0) if plane else 0
    sy = getattr(hdr, "subsampling_y", 0) if plane else 0
    cols = len(range(0, hdr.MiCols, 1 << sx)); W = cols * 4; H = (hdr.MiRows * 4) >> sy
    seen = [[0] * W for _ in range(H)]
    band = sb_lines // 4
    out = []
    dut.luma.value = 0 if plane else 1; dut.ssx.value = sx; dut.ssy.value = sy
    dut.mi_rows.value = hdr.MiRows; dut.mi_cols.value = hdr.MiCols

    async def monitor():
        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.out_valid.value):
                y = int(dut.out_y.value); y = y - 512 if y >= 256 else y
                out.append((int(dut.out_col.value), y, int(dut.out_line.value), int(dut.out_n4.value)))

    async def mode_info():                                   # mi_store's registered reads: the address of one clock is answered in the next
        while True:
            await FallingEdge(dut.clk)                       # the address is stable here; the coming edge samples it
            r = int(dut.rd_row.value); c = int(dut.rd_col.value)
            r64 = int(dut.cdr_row64.value); c64 = int(dut.cdr_col64.value)
            yr = int(dut.yd_row.value); yc = int(dut.yd_col.value)
            await RisingEdge(dut.clk)
            await Timer(1, "ns")
            dut.yd.value = ydir(yr, yc) if (ydir and yr < hdr.MiRows and yc < hdr.MiCols) else 0
            dut.rd_skip.value = int(state.Skips[min(r, hdr.MiRows - 1)][min(c, hdr.MiCols - 1)]) if (r < 4096 and c < 4096) else 0
            idx = state.cdef_idx.get((r64 << 4, c64 << 4), -1)
            dut.cdr_val.value = 0 if idx == -1 else (8 | idx)
            # the taker's room: block columns are held back at random in the bands whose input has gaps
            h = 1 if hrng.random() < hold_p[0] else 0
            dut.col_hold.value = h; stats["hold_clocks"] += h
    hrng = random.Random(1000 * plane + sb_lines); hold_p = [0.0]
    mon = cocotb.start_soon(monitor()); mi = cocotb.start_soon(mode_info())
    for row0 in range(0, hdr.MiRows, band):
        row1 = min(row0 + band, hdr.MiRows)
        n_blk = len(range(row0, row1, 1 << sy))
        first = row0 == 0; last = row1 >= hdr.MiRows
        y0 = (row0 * 4) >> sy
        del out[:]
        await Timer(2, "ns")
        dut.row0.value = row0 >> sy; dut.pic_rows.value = H // 4; dut.n_blk.value = n_blk; dut.n_col.value = cols
        dut.first_band.value = int(first); dut.last_band.value = int(last)
        dut.band_start.value = 1
        await RisingEdge(dut.clk); await Timer(2, "ns")
        dut.band_start.value = 0
        gap = rng.choice([0.0, 0.0, 0.3])
        hold_p[0] = 0.6 if gap else 0.0
        t_start = cocotb.utils.get_sim_time("ns")
        for ci in range(cols):
            for r4 in range(0 if first else -2, (n_blk if last else n_blk - 2)):
                while rng.random() < gap:
                    dut.in_valid.value = 0
                    await RisingEdge(dut.clk); await Timer(2, "ns")
                dut.in_valid.value = 1
                dut.in_blk.value = blk_of(dbl, y0 + 4 * r4, 4 * ci)
                n = 0
                while True:
                    await ReadOnly()
                    rdy = int(dut.in_ready.value)
                    await RisingEdge(dut.clk); await Timer(2, "ns")
                    if rdy:
                        break
                    n += 1
                    assert n < 100000, f"{tag} band at MI row {row0}: the stage stopped taking blocks at column {ci} row {r4}"
        dut.in_valid.value = 0
        n = 0
        while True:
            await ReadOnly()
            done = int(dut.band_done.value)
            await RisingEdge(dut.clk); await Timer(2, "ns")
            n += 1
            if done:
                break
            assert n < 400000, f"{tag} band at MI row {row0}: no band_done"
        if gap == 0.0:                                       # input never held back: clocks a CDEF block (the band's start and end included)
            stats["nogap_clocks"] += int((cocotb.utils.get_sim_time("ns") - t_start) // 10)
            stats["nogap_blocks"] += len(range(0, cols, 2 >> sx)) * len(range(0, n_blk, 2 >> sy))
        await ClockCycles(dut.clk, 2); await Timer(2, "ns")
        stats["cycles"] += n
        for (c4, yr, v, n4) in out:
            y = y0 + yr
            assert 0 <= y < H and 0 <= c4 * 4 < W, f"{tag} plane {plane}: line {yr} of the band at line {y0}, 4x4 column {c4}: outside the plane"
            for k in range(4 * n4):
                if 4 * c4 + k >= W:
                    continue                                 # an odd number of 4x4 columns: the block's second half is outside
                got = (v >> (12 * k)) & 0xFFF
                exp = post[y][4 * c4 + k]
                assert got == exp, (f"{tag} plane {plane} sb{sb_lines}: sample ({y}, {4 * c4 + k}) = {got}, model {exp} (deblocked {dbl[y][4 * c4 + k]}; "
                                    f"4x4 column {c4}, line {yr} of the band at line {y0})")
                seen[y][4 * c4 + k] += 1
        stats["lines"] += len(out)
    mon.cancel(); mi.cancel()
    missing = sum(1 for y in range(H) for x in range(W) if seen[y][x] != 1)
    assert missing == 0, f"{tag} plane {plane} sb{sb_lines}: {missing} samples not sent out exactly once"
    stats["changed"] += sum(1 for y in range(H) for x in range(W) if post[y][x] != dbl[y][x])


async def run_frame(dut, hdr, state, dbl, post, tag, stats, rng):
    dut.bit_depth.value = hdr.BitDepth
    dut.ch.value = ((hdr.cdef_damping & 7) << 98) | (sum(v << (6 * i) for i, v in enumerate(hdr.cdef_y_strengths)) << 48) \
        | sum(v << (6 * i) for i, v in enumerate(hdr.cdef_uv_strengths))
    for sb in (64, 128):
        await one_plane(dut, hdr, state, dbl, post, sb, rng, tag, stats)
        stats["runs"] += 1
    stats["frames"] += 1
    dut._log.info(f"{tag}: identical ({stats})")


async def one_frame(dut, tiles, tag, stats, rng):
    frame = {}; decs = []
    for th, data in tiles:
        dec = rm.FrameRecon(th, data, frame); dec.decode_tile(); decs.append(dec)
    hdr = tiles[0][0]
    state = xf.FrameState(hdr)
    for _d in decs:
        state.add(_d)
    planes = frame["planes"]
    lfm.LoopFilter(hdr, state, planes).apply()
    if not (hdr.enable_cdef and not hdr.CodedLossless and not hdr.allow_intrabc):
        stats["skipped"] += 1
        return
    cd = cdm.Cdef(hdr, state, planes)
    post = cd.apply()
    await run_frame(dut, hdr, state, planes[0], post[0], tag, stats, rng)
    dirs = {}

    def ydir(r, c):                                          # the luma direction of the 8x8 block, as the luma stage would store it
        if (r, c) not in dirs:
            dirs[(r, c)] = cd.direction(r, c)[0]
        return dirs[(r, c)]
    for plane in range(1, hdr.NumPlanes):
        for sb in (64, 128):
            await one_plane(dut, hdr, state, planes[plane], post[plane], sb, rng, tag, stats, plane, ydir)
            stats["chroma_runs"] += 1


@cocotb.test()
async def cdef_stream_vs_model(dut):
    seeds = [int(x) for x in os.environ.get("TS_SEEDS", "2,3").split(",") if x]
    W = int(os.environ.get("TS_W", "128")); H = int(os.environ.get("TS_H", "96"))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("band_start", "in_valid", "in_blk", "row0", "pic_rows", "n_blk", "n_col", "first_band", "last_band", "bit_depth", "ch", "rd_skip", "cdr_val",
              "luma", "ssx", "ssy", "mi_rows", "mi_cols", "yd", "col_hold"):
        getattr(dut, s).value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    rng = random.Random(1)
    stats = dict(frames=0, runs=0, chroma_runs=0, lines=0, changed=0, cycles=0, skipped=0, hold_clocks=0, nogap_clocks=0, nogap_blocks=0)
    ivfs = [p for p in os.environ.get("TS_IVF", "").split(",") if p]
    vdir = os.environ.get("TS_CDEF_VEC", "")
    if vdir:                                                 # frames dumped by a decoder test bench (cdef_vec.dump): inter frames
        import gzip, pickle
        import cdef_vec as cdv
        files = sorted(Path(vdir).glob("cdefvec_*.pkl.gz"))
        assert files, f"no frame files in {vdir}"
        cov = dict(blocks8=0, skipped8=0, noidx8=0, fourth_decides=0)
        for f in files:
            with gzip.open(f, "rb") as fh:
                v = pickle.load(fh)
            for k in cov: cov[k] += v[k]
            hdr, state, dbl, post = cdv.unpack(v)
            await run_frame(dut, hdr, state, dbl, post, v["tag"], stats, rng)
        stats.update(cov)
        assert cov["skipped8"] > 0 and cov["noidx8"] > 0 and cov["fourth_decides"] > 0, f"the frames lack unfiltered blocks: {cov}"
    elif ivfs:
        import obu_parser as op
        for path in ivfs:
            d = op.parse_file(path)
            await one_frame(dut, [d.tiles[i] for i in d.frames[0]], f"{Path(path).name} frame 0", stats, rng)
    else:
        for seed in seeds:
            fmt = os.environ.get("TS_FMT") or random.Random(seed * 7).choice(["420", "444", "mono", "422"])
            try:
                d, args = TT.gen(seed, W, H, fmt)
            except Exception:
                stats["skipped"] += 1
                continue
            await one_frame(dut, d.tiles, f"seed {seed} {fmt} bd{args['bd']} {W}x{H}", stats, rng)
    assert stats["frames"] > 0 and stats["changed"] > 0, stats
    if stats["nogap_blocks"]:
        stats["clocks_per_block"] = round(stats["nogap_clocks"] / stats["nogap_blocks"], 2)
    dut._log.info(f"OK: {stats}")
