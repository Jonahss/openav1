"""The streaming filter chain for all planes of a picture (rtl/fc_top_tb_top.sv: three mi_store, three block memories
and fc_top = an fc_plane per plane with the luma CDEF directions handed to chroma inside) against the frame-at-once
models on whole frames. As test_fc_plane, but the planes run at the same time on each band and nothing about CDEF is
answered by the test. Frames with super-resolution are skipped.
Env: TS_SEEDS / TS_W / TS_H / TS_FMT (generated frames), TS_IVF (frame 0 of real streams), FC_SB (64 or 128 only)."""
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
import cdef_model as cdm                # noqa: E402
import lf_model as lfm                  # noqa: E402
import lr_model as lrm                  # noqa: E402
import recon_model as rm                # noqa: E402
import test_tile_syntax as TT           # noqa: E402
import xcheck_frame as xf               # noqa: E402
from host_regs import LF_FIELDS, lf_vals, CDEF_FIELDS, cdef_vals   # noqa: E402
from test_lf_stage import blk_of, load_mi   # noqa: E402
from test_fc_plane import load_units    # noqa: E402


async def one_size(dut, hdr, pre, post, sb_lines, tag, stats):
    n = hdr.NumPlanes
    sxy = [((hdr.subsampling_x if p else 0), (hdr.subsampling_y if p else 0)) for p in range(n)]
    cols = [len(range(0, hdr.MiCols, 1 << sxy[p][0])) for p in range(n)]
    W = [cols[p] * 4 for p in range(n)]; H = [(hdr.MiRows * 4) >> sxy[p][1] for p in range(n)]
    seen = [[[0] * W[p] for _ in range(H[p])] for p in range(n)]
    band = sb_lines // 4
    out = [[] for _ in range(n)]

    async def monitor():
        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()
            ov = int(dut.out_valid.value)
            if ov:
                ol = int(dut.out_line.value); oc = int(dut.out_col.value); oy = int(dut.out_y.value); on = int(dut.out_n4.value)
                cw = len(dut.out_col) // 3
                for p in range(3):
                    if (ov >> p) & 1:
                        assert p < n, f"{tag}: plane {p} sent a line"
                        y = (oy >> (9 * p)) & 0x1FF; y = y - 512 if y >= 256 else y
                        out[p].append(((oc >> (cw * p)) & ((1 << cw) - 1), y, (ol >> (96 * p)) & ((1 << 96) - 1), (on >> (2 * p)) & 3))
    mon = cocotb.start_soon(monitor())
    for row0 in range(0, hdr.MiRows, band):
        row1 = min(row0 + band, hdr.MiRows)
        first = row0 == 0; last = row1 >= hdr.MiRows
        await Timer(1, "ns")
        for p in range(n):                                   # the band's blocks into the row store's stand-ins
            rows = list(range(row0, row1, 1 << sxy[p][1])); y0 = (row0 * 4) >> sxy[p][1]
            dut.bm_plane.value = p
            for ci in range(cols[p]):
                for ri in range(len(rows)):
                    dut.bm_we.value = 1; dut.bm_row.value = ri; dut.bm_col.value = ci
                    dut.bm_data.value = blk_of(pre[p], y0 + 4 * ri, 4 * ci)
                    await RisingEdge(dut.clk); await Timer(1, "ns")
            dut.bm_we.value = 0
            del out[p][:]
        dut.row0.value = row0; dut.n_blk.value = row1 - row0
        dut.first_band.value = int(first); dut.last_band.value = int(last)
        dut.band_start.value = 1
        await RisingEdge(dut.clk); await Timer(1, "ns")
        dut.band_start.value = 0
        k = 0
        while True:
            await ReadOnly()
            done = int(dut.band_done.value)
            await RisingEdge(dut.clk); await Timer(1, "ns")
            k += 1
            if done:
                break
            assert k < 3000000, f"{tag} sb{sb_lines} band at MI row {row0}: no band_done"
        await ClockCycles(dut.clk, 2); await Timer(1, "ns")
        assert int(dut.err_ovf.value) == 0, f"{tag} sb{sb_lines}: a CDEF line was dropped at a FIFO"
        stats["cycles"] += k; stats["in_samples"] += sum(16 * cols[p] * len(range(row0, row1, 1 << sxy[p][1])) for p in range(n))
        for p in range(n):
            sy = sxy[p][1]; sh = 12 if sy else 16
            y0 = (row0 * 4) >> sy; nb = len(range(row0, row1, 1 << sy))
            ya = 0 if first else y0 - sh
            yb = H[p] if last else y0 + nb * 4 - sh
            exp_lo = 0 if first else ya - 3
            exp_hi = H[p] if last else yb - 3
            for (c4, yr, v, n4) in out[p]:
                y = y0 + yr
                assert exp_lo <= y < exp_hi and 0 <= c4 * 4 < W[p], f"{tag} plane {p} sb{sb_lines}: line {y} 4x4 column {c4} sent in the band of lines {exp_lo}..{exp_hi - 1}"
                for i in range(4 * n4):
                    if 4 * c4 + i >= W[p]:
                        continue
                    got = (v >> (12 * i)) & 0xFFF
                    exp = post[p][y][4 * c4 + i]
                    assert got == exp, (f"{tag} plane {p} sb{sb_lines}: sample ({y}, {4 * c4 + i}) = {got}, model {exp} "
                                        f"(pre-filter {pre[p][y][4 * c4 + i]}; band at line {y0}, lines {exp_lo}..{exp_hi - 1})")
                    seen[p][y][4 * c4 + i] += 1
            stats["lines"] += len(out[p])
    mon.cancel()
    for p in range(n):
        missing = sum(1 for y in range(H[p]) for x in range(W[p]) if seen[p][y][x] != 1)
        assert missing == 0, f"{tag} plane {p} sb{sb_lines}: {missing} samples not sent out exactly once"


async def one_frame(dut, tiles, tag, stats):
    frame = {}; decs = []
    try:
        for th, data in tiles:
            dec = rm.FrameRecon(th, data, frame); dec.decode_tile(); decs.append(dec)
    except Exception as e:
        if type(e).__name__ != "Nonconformant":
            raise
        dut._log.info(f"{tag}: not a conformant stream ({e}), skipped"); stats["skipped"] += 1
        return
    hdr = tiles[0][0]
    if getattr(hdr, "use_superres", 0):
        stats["skipped_superres"] += 1
        return
    state = xf.FrameState(hdr)
    for _d in decs:
        state.add(_d)
    pre = frame["planes"]
    dbl = copy.deepcopy(pre)
    lfm.LoopFilter(hdr, state, dbl).apply()
    cdef_on = bool(hdr.enable_cdef and not hdr.CodedLossless and not hdr.allow_intrabc)
    lr_on = not all(t == 0 for t in hdr.FrameRestorationType[:hdr.NumPlanes])
    cdef = cdm.Cdef(hdr, state, dbl).apply() if cdef_on else copy.deepcopy(dbl)
    post = lrm.LoopRestoration(hdr, state, dbl, cdef).apply() if lr_on else cdef
    dut.hdr.value = TT.pack(TT.HDR_FIELDS, TT.hdr_vals(hdr, decs[0]))
    dut.lh.value = TT.pack(LF_FIELDS, lf_vals(hdr))
    dut.ch.value = TT.pack(CDEF_FIELDS, cdef_vals(hdr))
    dut.cdef_on.value = int(cdef_on)
    lvl = hdr.loop_filter_level
    dut.lf_on.value = sum(int(bool(lvl[0] or lvl[1]) and (p == 0 or bool(lvl[1 + p]))) << p for p in range(3))
    await load_mi(dut, hdr, state)
    await load_units(dut, hdr, state)
    only_sb = int(os.environ.get("FC_SB", "0"))
    for sb in ((only_sb,) if only_sb else (64, 128)):
        await one_size(dut, hdr, pre, post, sb, tag, stats)
    for p in range(hdr.NumPlanes):
        stats["by_lf"] += sum(1 for a, b in zip(pre[p], dbl[p]) if a != b)
        stats["by_cdef"] += sum(1 for a, b in zip(dbl[p], cdef[p]) if a != b)
        stats["by_lr"] += sum(1 for a, b in zip(cdef[p], post[p]) if a != b)
        stats["chroma_by_cdef"] += sum(1 for a, b in zip(dbl[p], cdef[p]) if a != b) if p else 0
    stats["frames"] += 1; stats["cdef_frames"] += cdef_on; stats["lr_frames"] += lr_on; stats["mono"] += hdr.NumPlanes == 1
    dut._log.info(f"{tag}: identical ({stats})")


@cocotb.test()
async def fc_top_vs_models(dut):
    seeds = [int(x) for x in os.environ.get("TS_SEEDS", "2,3").split(",") if x]
    W = int(os.environ.get("TS_W", "128")); H = int(os.environ.get("TS_H", "96"))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("band_start", "bm_we", "bm_plane", "bm_row", "bm_col", "bm_data", "blk_we", "tx_we", "hdr", "lh", "ch", "lf_on", "cdef_on", "row0", "n_blk",
              "first_band", "last_band", "blk_r", "blk_c", "blk_bw4", "blk_bh4", "blk_data", "tx_plane", "tx_row", "tx_col", "tx_w4", "tx_h4",
              "tx_sz", "cd_clr", "cd_we", "cd_row64", "cd_col64", "cd_sb128", "cd_mask", "cd_idx", "lr_we", "lr_rec"):
        getattr(dut, s).value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    stats = dict(frames=0, cdef_frames=0, lr_frames=0, mono=0, lines=0, by_lf=0, by_cdef=0, chroma_by_cdef=0, by_lr=0, in_samples=0, cycles=0,
                 skipped=0, skipped_superres=0)
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
    assert stats["frames"] > 0 and stats["by_lf"] > 0, stats
    stats["samples_per_clock"] = round(stats["in_samples"] / max(1, stats["cycles"]), 3)
    dut._log.info(f"OK: {stats}")
