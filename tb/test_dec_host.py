"""The decoder through its AXI-Lite host interface (rtl/dec_host.sv): the same generated streams and model as
test_dec_top, but every header, CDF default row, tile byte, start and pixel read goes through the register map,
exactly as software on the board does it. With TS_TRANSCRIPT=<file> the register traffic is written out
(tb/axil.py format) for tools/kv260/play.py to replay on the KV260 against the bitstream.
Env: TS_SEEDS, TS_W / TS_H (default 64x64), TS_FMT, TS_STAGE (recon | lf), TS_TRANSCRIPT; or TS_IVF=<a.ivf,b.ivf>
to run real streams (intra, no CDEF / LR: the core build) against dav1d's pictures in refout/ (TS_REF_DIR).
"""
import os
import random
import sys
from pathlib import Path
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "tools"))
import cdf_map as M                     # noqa: E402
import lf_model as lfm                  # noqa: E402
import test_dec_top as TD               # noqa: E402
import test_lf_top as TL                # noqa: E402
import test_tile_syntax as TT           # noqa: E402
import xcheck_frame as xf               # noqa: E402
from axil import AxiLite, write_transcript   # noqa: E402
from axi_mem import AxiMem                    # noqa: E402

STAGE = os.environ.get("TS_STAGE", "recon")
PIPE = os.environ.get("TS_PIPE", "0") == "1" and STAGE == "lf"      # the deblocking filter under the frame sequencer
WIN = os.environ.get("TS_WIN", "0") == "1"                           # window build: rows auto-flushed into the AXI memory


async def pipe_start(ax, hdr, n_tile_cols, base=None, mem=None):
    if WIN:                                                  # the planes and auto-flush before the frame
        seq, layout, total = dma_regs(hdr, base, visible=True)
        st = await ax.read(R_DMA_STATUS, record=False)
        ax.dut._log.info(f"pipe_start: DMA_STATUS 0x{st:08x} before the frame; {len(seq)} plane register writes")
        for a, v in seq:
            await ax.write(a, (v | DMA_CFG_AUTO) if a == R_DMA_CFG else v)
        mem.mem.clear()
    await write_wide(ax, R_LH, TT.pack(TL.LF_FIELDS, TL.lf_vals(hdr)), LH_WORDS)
    await ax.write(R_CTRL, CTRL_PIPE | (n_tile_cols << 16))
    await ax.write(R_START, START_FRAME)
    ax.dut._log.info("pipe_start: frame started")


async def pipe_finish(ax, tag):
    await ax.poll(R_STATUS, ST_FRAME_DONE, ST_FRAME_DONE, 120_000_000, tag)
    await ax.write(R_CTRL, 0)

from host_regs import (HostDecoder, R_ID, R_CFG, R_CTRL, R_START, R_STATUS, R_DATA1, R_DATA4, R_HADDR, R_HDATA, R_DEF_ADDR,   # noqa: E402
                       R_HDATA2, R_HDATA4, R_DEF, R_HDR, R_RH, R_LH, R_DMA_BEATS, ST_TILE_DONE, ST_UNSUP, ST_LF_DONE, START_TILE, START_LF, CTRL_RST, CTRL_EOS,
                       HDR_WORDS, RH_WORDS, LH_WORDS, words, plane_size, dma_regs, unpack_dma, R_DMA_START, R_DMA_STATUS,
                       DMA_BUSY, DMA_DONE, DMA_ERR, START_FRAME, ST_FRAME_DONE, CTRL_PIPE, DMA_FLUSH_BUSY, DMA_CFG_AUTO, R_DMA_CFG,
                       dma_layout)



async def write_wide(ax, base, v, nwords):
    for i, w in enumerate(words(v, nwords)):
        await ax.write(base + 4 * i, w)


async def run_tile(ax, th, dec, data, tag, stats):
    await write_wide(ax, R_HDR, TT.pack(TT.HDR_FIELDS, TT.hdr_vals(th, dec, 0)), HDR_WORDS)
    await write_wide(ax, R_RH, TT.pack(TD.REC_FIELDS, TD.rec_vals(th)), RH_WORDS)
    for i, row in enumerate(M.default_rows(th.base_q_idx)):
        await write_wide(ax, R_DEF, row, 8)
        await ax.write(R_DEF_ADDR, i)
    await ax.write(R_START, START_TILE)
    # the bytes: whole words, the FIFO's free count read before each burst
    pos = 0
    while pos < len(data):
        st = await ax.read(R_STATUS, record=False)
        free = st >> 16
        if free == 0:
            await ClockCycles(ax.clk, 64)
            continue
        n = min(free, (len(data) - pos + 3) // 4)
        for _ in range(n):
            chunk = data[pos:pos + 4]
            if len(chunk) == 4:
                await ax.write(R_DATA4, int.from_bytes(chunk, "little"))
            else:
                for b in chunk:
                    await ax.write(R_DATA1, b)
            pos += 4
    ctrl = await ax.read(R_CTRL, record=False)
    await ax.write(R_CTRL, ctrl | CTRL_EOS)
    st = await ax.poll(R_STATUS, ST_TILE_DONE | ST_UNSUP, ST_TILE_DONE, 400_000_000, tag)
    assert not (st & ST_UNSUP), f"{tag}: RTL flagged unsupported syntax"
    stats["tiles"] += 1
    stats["bytes"] += len(data)


DMA_BASE = 0x7_1234_5678 & ~7                                # an odd-looking, 8-aligned 40-bit address


async def read_back(ax, hdr, planes, tag, stats, visible=False, mem=None):
    """The picture through the host port: packed words (HDATA4 for 8-bit, HDATA2 otherwise, the address
    advancing by itself) unless TS_READ=single, which reads every sample through HADDR / HDATA; with TS_DMA=1
    (mem: the AxiMem) through the DMA engine into memory, and the memory image is compared."""
    packed = os.environ.get("TS_READ", "packed") != "single"
    if os.environ.get("TS_DMA", "0") == "1":
        base = DMA_BASE + (stats["frames"] % 3) * 0x100000      # a different base per frame
        if WIN:                                                  # the rows were flushed as they finished: wait for the last
            base = DMA_BASE
            seq, layout, total = dma_regs(hdr, base, visible=True)
            st = await ax.poll(R_DMA_STATUS, DMA_FLUSH_BUSY, 0, 20_000_000, tag)
            visible = True
        else:
            seq, layout, total = dma_regs(hdr, base, visible)
            for a, v in seq:
                await ax.write(a, v)
            mem.mem.clear()
            await ax.write(R_DMA_START, 1)
            st = await ax.poll(R_DMA_STATUS, DMA_DONE | DMA_BUSY, DMA_DONE, 20_000_000, tag)
        assert not (st & DMA_ERR), f"{tag}: DMA error flag"
        assert not mem.errors, f"{tag}: AXI protocol errors: {mem.errors[:4]}"
        got = unpack_dma(hdr, mem.read(base, total), layout)
        bad = []
        for plane in range(hdr.NumPlanes):
            _, _, W, H = layout[plane]
            for y in range(H):
                if got[plane][y] != planes[plane][y][:W]:
                    bad += [(plane, y, x, got[plane][y][x], planes[plane][y][x]) for x in range(W) if got[plane][y][x] != planes[plane][y][x]]
            stats["pixels"] += W * H
        expect_bytes = sum(((W * (1 if hdr.BitDepth == 8 else 2)) * H) for _, _, W, H in layout)
        if mem.written() != expect_bytes:
            per = []
            for b, stride, W, H in layout:
                lo, hi = b, b + stride * H
                per.append(f"plane@0x{b:x} stride {stride} {W}x{H}: {sum(1 for a in mem.mem if lo <= a < hi)} bytes")
            others = sum(1 for a in mem.mem if not any(b <= a < b + stride * H for b, stride, W, H in layout))
            st2 = await ax.read(R_DMA_STATUS, record=False)
            beats = await ax.read(R_DMA_BEATS, record=False)
            assert False, (f"{tag}: DMA wrote {mem.written()} bytes, picture is {expect_bytes}; " + "; ".join(per)
                           + f"; outside: {others}; max addr 0x{max(mem.mem):x}; DMA_STATUS 0x{st2:08x} (rows flushed {st2 >> 16 & 0xFF}), beats {beats}, bursts {mem.bursts}")
        assert not bad, f"{tag}: DMA image: {len(bad)} samples differ, first (plane, y, x, dma, model): {bad[:8]}"
        return
    for plane in range(hdr.NumPlanes):
        W, H = plane_size(hdr, plane, visible)                # visible: dav1d's pictures hold the visible area only
        n = 4 if hdr.BitDepth == 8 else 2
        reg = R_HDATA4 if n == 4 else R_HDATA2
        bad = []
        for y in range(H):
            x = 0
            if packed:
                await ax.write(R_HADDR, (plane << 28) | (y << 14))
                while x + n <= W:
                    exp = sum(planes[plane][y][x + i] << (8 * i if n == 4 else 16 * i) for i in range(n))
                    v = await ax.read(reg, expect=exp)
                    for i in range(n):
                        got = (v >> (8 * i)) & 0xFF if n == 4 else (v >> (16 * i)) & 0xFFF
                        if got != planes[plane][y][x + i]:
                            bad.append((y, x + i, got, planes[plane][y][x + i]))
                    x += n
            for x in range(x, W):
                await ax.write(R_HADDR, (plane << 28) | (y << 14) | x)
                v = await ax.read(R_HDATA, expect=planes[plane][y][x])
                if v != planes[plane][y][x]:
                    bad.append((y, x, v, planes[plane][y][x]))
        stats["pixels"] += W * H
        assert not bad, f"{tag}: plane {plane}: {len(bad)} / {W * H} samples differ, first (y, x, rtl, model): {bad[:8]}"


@cocotb.test()
async def dec_host_streams(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    if hasattr(dut, "clk2x"):
        cocotb.start_soon(Clock(dut.clk2x, 5, unit="ns").start())

        async def phase_gen():
            while True:
                await RisingEdge(dut.clk)
                await Timer(2.5, "ns")
                dut.phase.value = 1
                await Timer(5, "ns")
                dut.phase.value = 0
        dut.phase.value = 0
        cocotb.start_soon(phase_gen())
    transcript = [] if os.environ.get("TS_TRANSCRIPT") else None
    ax = AxiLite(dut, dut.clk, transcript=transcript)
    mem = AxiMem(dut, dut.clk)
    cocotb.start_soon(mem.run())
    dut.aresetn.value = 0
    await ClockCycles(dut.clk, 4)
    dut.aresetn.value = 1
    await ClockCycles(dut.clk, 2)
    ident = await ax.read(R_ID, expect=0xA71D0001)
    assert ident == 0xA71D0001, f"ID register reads 0x{ident:08x}"
    cfg = await ax.read(R_CFG)
    dut._log.info(f"dec_host ID 0x{ident:08x} CFG 0x{cfg:08x}: FBX {cfg >> 16 & 0xFF} FBY {cfg >> 8 & 0xFF} lanes {cfg >> 4 & 0xF} uram {cfg & 1}")
    seeds = [int(s) for s in os.environ.get("TS_SEEDS", "1").split(",")]
    W = int(os.environ.get("TS_W", "64"))
    H = int(os.environ.get("TS_H", "64"))
    stats = dict(frames=0, tiles=0, bytes=0, pixels=0, stage=STAGE)
    ivfs = [p for p in os.environ.get("TS_IVF", "").split(",") if p]
    if ivfs:
        import obu_parser as op
        import tile_model as tm
        ref_dir = os.environ.get("TS_REF_DIR", str(HERE.parent / "refout"))
        for path in ivfs:
            d = op.Decoder()
            with open(path, "rb") as fh:
                d.feed_ivf(fh.read())
            shown = d.shown_frames()
            pic_offset, off = [], 0
            for k in shown:
                h_k = d.tiles[d.frames[k][0]][0]
                pic_offset.append(off)
                off += sum(((h_k.UpscaledWidth + (h_k.subsampling_x if p_ else 0)) >> (h_k.subsampling_x if p_ else 0)) *
                           ((h_k.FrameHeight + (h_k.subsampling_y if p_ else 0)) >> (h_k.subsampling_y if p_ else 0))
                           for p_ in range(h_k.NumPlanes)) * (1 if h_k.BitDepth == 8 else 2)
            for fi, tile_idx in enumerate(d.frames):
                tag = f"{Path(path).name} frame {fi}"
                hdr0 = d.tiles[tile_idx[0]][0]
                assert not (hdr0.enable_cdef and not hdr0.CodedLossless and not hdr0.allow_intrabc), f"{tag}: CDEF is not in the core build"
                assert not any(t != 0 for t in hdr0.FrameRestorationType[:hdr0.NumPlanes]), f"{tag}: loop restoration is not in the core build"
                await ax.write(R_CTRL, CTRL_RST)
                await ClockCycles(dut.clk, 4)
                await ax.write(R_CTRL, 0)
                if PIPE:
                    await pipe_start(ax, hdr0, len({d.tiles[ti][0].MiColStart for ti in tile_idx}), DMA_BASE, mem)
                for ti in tile_idx:
                    th, data = d.tiles[ti]
                    dec = tm.TileDecoder(th, data, None)      # header helper only (get_qindex); nothing is decoded
                    await run_tile(ax, th, dec, data, f"{tag} tile ({th.MiColStart},{th.MiRowStart})", stats)
                if PIPE:
                    await pipe_finish(ax, tag)
                else:
                    await write_wide(ax, R_LH, TT.pack(TL.LF_FIELDS, TL.lf_vals(hdr0)), LH_WORDS)
                    await ax.write(R_START, START_LF)
                    await ax.poll(R_STATUS, ST_LF_DONE, ST_LF_DONE, 50_000_000, tag)
                outs = [o for o, k in enumerate(shown) if k == fi]
                if not outs:
                    continue
                ref, _ = TD.ref_frame(os.path.join(ref_dir, Path(path).stem + ".yuv"), hdr0, fi, pic_offset[outs[0]])
                await read_back(ax, hdr0, ref, tag, stats, visible=True, mem=mem)
                stats["frames"] += 1
                dut._log.info(f"{tag}: identical to dav1d ({len(tile_idx)} tiles, {hdr0.UpscaledWidth}x{hdr0.FrameHeight} bd{hdr0.BitDepth}, {len(transcript) if transcript else 0} register ops so far)")
        if transcript is not None:
            write_transcript(os.environ["TS_TRANSCRIPT"], transcript)
            dut._log.info(f"transcript: {len(transcript)} ops -> {os.environ['TS_TRANSCRIPT']}")
        dut._log.info(f"OK: {stats}")
        return
    for seed in seeds:
        fmt = os.environ.get("TS_FMT") or random.Random(seed * 7).choice(["420", "444", "mono", "422"])
        d, args = TT.gen(seed, W, H, fmt)
        tag = f"seed {seed} {fmt} bd{args['bd']}"
        await ax.write(R_CTRL, CTRL_RST)
        await ClockCycles(dut.clk, 4)
        await ax.write(R_CTRL, 0)
        frame = {}
        hdr0 = None
        decs = []
        dut._log.info(f"{tag} {W}x{H}: {len(d.tiles)} tiles, sb128 {d.tiles[0][0].use_128x128_superblock}, decoding the model")
        if WIN and not HostDecoder.window_ok(d.tiles[0][0], len({th.MiColStart for th, _ in d.tiles})):
            dut._log.info(f"{tag}: outside the window build's limits (sb128 / non-4:2:0 / tile columns / intrabc), skipped")
            continue
        if PIPE:
            await pipe_start(ax, d.tiles[0][0], len({th.MiColStart for th, _ in d.tiles}), DMA_BASE, mem)
        for ti, (th, data) in enumerate(d.tiles):
            dec = TD.RecFrame(th, data, frame)
            dec.decode_tile()
            decs.append(dec)
            hdr0 = th
            await run_tile(ax, th, dec, data, f"{tag} tile {ti} ({th.MiColStart},{th.MiRowStart})", stats)
        planes = frame["planes"]
        if STAGE == "lf":
            state = xf.FrameState(hdr0, decs)
            lfm.LoopFilter(hdr0, state, planes).apply()
            if PIPE:
                await pipe_finish(ax, tag)
            else:
                await write_wide(ax, R_LH, TT.pack(TL.LF_FIELDS, TL.lf_vals(hdr0)), LH_WORDS)
                await ax.write(R_START, START_LF)
                await ax.poll(R_STATUS, ST_LF_DONE, ST_LF_DONE, 50_000_000, tag)
        await read_back(ax, hdr0, planes, tag, stats, mem=mem)
        stats["frames"] += 1
        dut._log.info(f"{tag} {W}x{H}: frame identical ({len(d.tiles)} tiles, {len(transcript) if transcript else 0} register ops so far)")
    if transcript is not None:
        write_transcript(os.environ["TS_TRANSCRIPT"], transcript)
        dut._log.info(f"transcript: {len(transcript)} ops -> {os.environ['TS_TRANSCRIPT']}")
    dut._log.info(f"OK: {stats}")
