"""Helpers for the decoder behind its AXI-Lite host interface (rtl/dec_host.sv) under cocotb, used by
tb/test_dec_host_inter.py: a frame under the frame sequencer with the row flush (pipe_start / pipe_finish), the tile
registers and bytes (run_tile, run_tile_row), the model's in-loop filters (model_filters) and the CDF read-back check.
Until 2026-11-21 this file was test_dec_host.py and also held the whole-picture test that read pictures back through
HADDR / HDATA and the DMA engine's software mode; the card only ever used the row flush.
"""
import copy
import os
import sys
from pathlib import Path
import cocotb
from cocotb.triggers import ClockCycles
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "tools"))
import cdf_map as M                     # noqa: E402
import lf_model as lfm                  # noqa: E402
import filter_chain_model as fcm       # noqa: E402
import cdef_model as cdm                # noqa: E402
import lr_model as lrm                  # noqa: E402
import superres_model as srm            # noqa: E402
import test_dec_top as TD               # noqa: E402
import test_tile_syntax as TT           # noqa: E402
import xcheck_frame as xf               # noqa: E402



async def pipe_start(ax, hdr, n_tile_cols, base=None, mem=None, clear=True, visible=True):
    """Pipelined frame under the frame sequencer; the flush target (base) and auto-flush are programmed first.
    clear=False keeps the AXI memory's contents (inter streams: earlier frames' pictures are the references)."""
    seq, layout, total = dma_regs(hdr, base, visible=visible)            # intrabc frames flush the whole decoded (MI) area
    st = await ax.read(R_DMA_STATUS, record=False)
    ax.dut._log.info(f"pipe_start: DMA_STATUS 0x{st:08x} before the frame; {len(seq)} plane register writes")
    for a, v in seq:
        await ax.write(a, (v | DMA_CFG_AUTO) if a == R_DMA_CFG else v)
    if clear:
        mem.mem.clear()
    await write_wide(ax, R_LH, TT.pack(HR.LF_FIELDS, HR.lf_vals(hdr)), LH_WORDS)
    if hdr.enable_cdef and not hdr.CodedLossless and not hdr.allow_intrabc:        # the CDEF header: the sequencer starts CDEF itself
        await write_wide(ax, HR.R_CH, TT.pack(HR.CDEF_FIELDS, HR.cdef_vals(hdr)), HR.CH_WORDS)
    await ax.write(R_CTRL, CTRL_PIPE | (n_tile_cols << 16))
    await ax.write(R_START, START_FRAME)
    ax.dut._log.info("pipe_start: frame started")


def model_filters(hdr0, decs, planes):
    """The model's in-loop filters after the tiles: deblocking in place, CDEF, the super-resolution upscale of both
    LR inputs, loop restoration. Returns the planes the RTL's final buffer holds."""
    state = xf.FrameState(hdr0)
    for _d in decs:
        state.add(_d)
    pre_chain = copy.deepcopy(planes) if fcm.ENABLED else None
    lfm.LoopFilter(hdr0, state, planes).apply()
    deblocked = planes
    deblocked_chain = planes; cdef_chain = None; final_chain = None
    cdef_on = hdr0.enable_cdef and not hdr0.CodedLossless and not hdr0.allow_intrabc
    if cdef_on:
        planes = cdm.Cdef(hdr0, state, planes).apply()
        cdef_chain = planes
    if hdr0.use_superres:
        deblocked = srm.upscale(hdr0, deblocked)
        planes = srm.upscale(hdr0, planes) if cdef_on else deblocked
    if any(t != 0 for t in hdr0.FrameRestorationType[:hdr0.NumPlanes]):
        planes_cdef = planes if cdef_on else [[row[:] for row in p] for p in planes]
        planes = lrm.LoopRestoration(hdr0, state, deblocked, planes_cdef).apply()
        final_chain = planes
    if fcm.ENABLED:
        fcm.check_frame(hdr0, state, pre_chain, deblocked_chain, cdef_chain, final_chain, "dec_host")
    return planes


async def pipe_finish(ax, tag):
    await ax.poll(R_STATUS, ST_FRAME_DONE, ST_FRAME_DONE, 120_000_000, tag, stop=ST_UNSUP)
    await ax.write(R_CTRL, 0)

import host_regs as HR                   # noqa: E402
from host_regs import (R_CTRL, R_START, R_STATUS, R_DATA1, R_DATA4, R_DEF_ADDR, R_DEF, R_HDR, R_RH, R_LH,   # noqa: E402
                       ST_TILE_DONE, ST_UNSUP, START_TILE, CTRL_EOS, HDR_WORDS, RH_WORDS, LH_WORDS, words, dma_regs, R_DMA_STATUS,
                       START_FRAME, ST_FRAME_DONE, CTRL_PIPE, DMA_CFG_AUTO, R_DMA_CFG)



async def write_wide(ax, base, v, nwords):
    for i, w in enumerate(words(v, nwords)):
        await ax.write(base + 4 * i, w)


async def run_tile(ax, th, dec, data, tag, stats, rows=None):
    """One tile through the register map. rows: the CDF rows to load (default: the defaults for the tile's base_q_idx;
    inter streams pass the inherited frame CDFs, tb/seq_model.py)."""
    await write_wide(ax, R_HDR, TT.pack(TT.HDR_FIELDS, TT.hdr_vals(th, dec, 0)), HDR_WORDS)
    await write_wide(ax, R_RH, TT.pack(TD.REC_FIELDS, TD.rec_vals(th)), RH_WORDS)
    await write_wide(ax, HR.R_IH, TT.pack(HR.IHDR_FIELDS, HR.ihdr_vals(th)), HR.IH_WORDS)
    for i, row in enumerate([] if rows is False else (M.default_rows(th.base_q_idx) if rows is None else rows)):   # False: the loader wrote them (stage B)
        await write_wide(ax, R_DEF, row, 8)
        await ax.write(R_DEF_ADDR, i)
    await ax.write(R_START, START_TILE)
    data = data + bytes(int(os.environ.get("TS_PAD", "0")))    # TS_PAD=N: N explicit zero bytes after the tile (padding experiment)
    # the bytes: whole words, the FIFO's free count read before each burst
    pos = 0; idle = 0
    while pos < len(data):
        st = await ax.read(R_STATUS, record=False)
        free = st >> 16
        if free == 0:
            idle += 1                                            # (a decoder that stopped taking bytes: say so instead of waiting for ever)
            assert not (st & ST_UNSUP), f"{tag}: RTL flagged unsupported syntax with {len(data) - pos} bytes to push (status 0x{st:08x})"
            assert idle < 20_000, f"{tag}: the tile FIFO never freed a word ({pos} of {len(data)} bytes pushed, status 0x{st:08x})"
            await ClockCycles(ax.clk, 64)
            continue
        idle = 0
        # every push is one FIFO word, a single tail byte included: the budget counts pushes, not bytes (2026-10-15:
        # a 3-byte tail budgeted as one word overflowed a nearly full FIFO and the last byte of the tile was lost)
        for _ in range(free):
            if pos >= len(data):
                break
            chunk = data[pos:pos + 4]
            if len(chunk) == 4:
                await ax.write(R_DATA4, int.from_bytes(chunk, "little"))
                pos += 4
            else:
                await ax.write(R_DATA1, chunk[0])
                pos += 1
    ctrl = await ax.read(R_CTRL, record=False)
    await ax.write(R_CTRL, ctrl | CTRL_EOS)
    st = await ax.poll(R_STATUS, ST_TILE_DONE | ST_UNSUP, ST_TILE_DONE, 400_000_000, tag, stop=ST_UNSUP)   # (unsupported ends the wait)
    assert not (st & (1 << 14)), f"{tag}: the tile FIFO dropped a push (overflow or a split write)"
    stats["tiles"] += 1
    stats["bytes"] += len(data)


async def run_tile_row(ax, tiles, tag, stats, rows=None, chunk=256):
    """The tiles of one tile row ([(th, dec, data), ...] in column order) through the tile engines at once
    (rtl/tile_engines.sv, HostDecoder.run_tile_row): headers once, each engine's columns and the engine mask, the
    CDF rows broadcast, one start, the bytes round-robin in chunks so no engine starves, end of stream, tile done."""
    if len(tiles) == 1:
        th, dec, data = tiles[0]
        await run_tile(ax, th, dec, data, tag, stats, rows=rows)
        return
    th0, dec0, _ = tiles[0]
    await write_wide(ax, R_HDR, TT.pack(TT.HDR_FIELDS, TT.hdr_vals(th0, dec0, 0)), HDR_WORDS)
    await write_wide(ax, R_RH, TT.pack(TD.REC_FIELDS, TD.rec_vals(th0)), RH_WORDS)
    await write_wide(ax, HR.R_IH, TT.pack(HR.IHDR_FIELDS, HR.ihdr_vals(th0)), HR.IH_WORDS)
    for e, (th, _, _) in enumerate(tiles):
        await ax.write(HR.R_TILE_COL + 4 * e, (th.MiColEnd << 16) | th.MiColStart)
    await ax.write(HR.R_TILE_ENG, (1 << len(tiles)) - 1)
    for i, row in enumerate([] if rows is False else (M.default_rows(th0.base_q_idx) if rows is None else rows)):   # False: the loader wrote them (stage B)
        await write_wide(ax, R_DEF, row, 8)
        await ax.write(R_DEF_ADDR, i)
    ctrl0 = (await ax.read(R_CTRL, record=False)) & ~(HR.CTRL_ENG_MASK | CTRL_EOS)   # the previous tile row's EOS must not ride along
    await ax.write(R_CTRL, ctrl0)
    await ax.write(R_START, START_TILE)
    cocotb.log.info(f"{tag}: {len(tiles)} engines started, cols {[(th.MiColStart, th.MiColEnd) for th, _, _ in tiles]}, bytes {[len(d) for _, _, d in tiles]}, ctrl 0x{ctrl0:x}")
    pos = [0] * len(tiles)
    idle = 0
    loops = 0
    while any(pos[e] < len(d) for e, (_, _, d) in enumerate(tiles)):
        progressed = False
        loops += 1
        for e, (_, _, data) in enumerate(tiles):
            if pos[e] >= len(data):
                continue
            if loops <= 3:
                cocotb.log.info(f"{tag}: loop {loops} reading engine {e}'s FIFO status")
            fst = await ax.read(HR.R_TILE_FIFO + 4 * e, record=False)
            free = fst >> 16
            if loops <= 3 or loops % 200 == 0:
                cocotb.log.info(f"{tag}: loop {loops} engine {e} fifo 0x{fst:08x} pos {pos} status 0x{await ax.read(R_STATUS, record=False):08x}")
            if free == 0:
                continue
            await ax.write(R_CTRL, ctrl0 | (e << HR.CTRL_ENG_SHIFT))
            # every push is one FIFO word, a tail byte included: the budget counts pushes, not bytes (the single-engine
            # loop's 2026-10-15 lesson; this loop budgeted free * 4 bytes, and test5413 frame 0's 3-byte tail into one
            # free word overflowed engine 0's FIFO, 2026-10-28)
            q = pos[e]
            for _ in range(min(free, chunk // 4)):
                if q >= len(data):
                    break
                c = data[q:q + 4]
                if len(c) == 4:
                    await ax.write(R_DATA4, int.from_bytes(c, "little")); q += 4
                else:
                    await ax.write(R_DATA1, c[0]); q += 1
            pos[e] = q
            progressed = True
            if q >= len(data):                                       # this engine's stream ends now (EOS per engine, CFG bit 2)
                await ax.write(R_CTRL, ctrl0 | (e << HR.CTRL_ENG_SHIFT) | CTRL_EOS)
        if not progressed:
            idle += 1
            assert idle < 100_000, f"{tag}: no engine FIFO freed a word"
            await ClockCycles(ax.clk, 64)
    for e in range(len(tiles)):                                      # (every engine's EOS went out with its last bytes; an empty tile's here)
        await ax.write(R_CTRL, ctrl0 | (e << HR.CTRL_ENG_SHIFT) | CTRL_EOS)
    cocotb.log.info(f"{tag}: all bytes pushed ({loops} loops), EOS set, status 0x{await ax.read(R_STATUS, record=False):08x}")
    # the done poll with a running log of STATUS (debugging the two-engine rows: the plain poll never returned)
    reads = 0
    while True:
        st = await ax.read(R_STATUS, record=False)
        reads += 1
        if reads <= 5 or reads % 2000 == 0:
            cocotb.log.info(f"{tag}: done poll read {reads}: status 0x{st:08x}")
        if st & (ST_TILE_DONE | ST_UNSUP):
            break
        assert reads < 2_000_000, f"{tag}: no tile done after {reads} status reads (last 0x{st:08x})"   # ~5 s of simulated time: LR-heavy tall frames take > 8 ms after the last byte
        await ClockCycles(ax.clk, 256)
    cocotb.log.info(f"{tag}: done poll finished after {reads} reads: status 0x{st:08x}")
    await ax.write(HR.R_TILE_ENG, 0)
    assert not (st & ST_UNSUP), f"{tag}: RTL flagged unsupported syntax"
    assert not (st & (1 << 14)), f"{tag}: a tile FIFO dropped a push (overflow or a split write)"
    stats["tiles"] += len(tiles)
    stats["bytes"] += sum(len(d) for _, _, d in tiles)


async def check_cdf_readback(ax, dec, tag, stats, rows=None):
    """DEF_ADDR bit 31: the working store's rows after the tile must be the model's adapted CDFs (counters too)."""
    want = M.rows_of(dec.cdf)
    if rows is None:
        rows = range(M.ROWS)
    bad = 0
    for i in rows:
        await ax.write(R_DEF_ADDR, i | (1 << 31))
        v = 0
        for k in range(8):
            v |= (await ax.read(R_DEF + 4 * k, record=False)) << (32 * k)
        if v != want[i]:
            bad += 1
            if bad <= 5:
                name = next(n for n in M.TABLES if M.MAP[n]["base"] <= i < M.MAP[n]["base"] + M.MAP[n]["rows"])
                cocotb.log.info(f"{tag}: CDF row {i} ({name}+{i - M.MAP[name]['base']}) RTL {v:062x} model {want[i]:062x}")
    assert bad == 0, f"{tag}: {bad} CDF rows differ after the tile"
    stats["rb_checks"] = stats.get("rb_checks", 0) + 1
