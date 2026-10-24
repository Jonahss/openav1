"""Inter streams through dec_host: the register map, the frame sequencer and the row flush, against the model
(tb/test_dec_inter.py is the same check through dec_top's ports). Every frame is decoded pipelined under the
sequencer, its final picture (after CDEF / super-res / loop restoration) is flushed row by row into the AXI
memory, and the references of later frames are read by ref_fetch from THOSE flushed pictures: the slot table
points at the flush targets, exactly as the board software will do (docs/picture-memory.md, step A). With
FRAME_URAM=1 FRAME_WIN=5 FRAME_WIN1=3 FRAME_WIN2=3 this is the U200 inter build's memory system in simulation.

The frame sequence follows tb/seq_model.py (inherited CDFs through the DEF window, segmentation map and motion
field through the band registers, slot table and ref_frame_idx through 0x400 / 0x600). Frames the window build
cannot take (sb128, 4:4:4 / 4:2:2, several tile columns, intra block copy) and frames outside TS_FRAMES are
decoded by the model only; a reference the RTL did not produce is loaded into the AXI memory at a slot address.
Env: TS_IVF=a.obu[,b.obu] (required), TS_FRAMES, TD_DEBUG. TS_STAGE is lr, TS_PIPE / TS_WIN / TS_DMA are 1.
"""
import os
from pathlib import Path
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer

os.environ["TS_STAGE"] = "lr"
os.environ["TS_PIPE"] = os.environ["TS_WIN"] = os.environ["TS_DMA"] = "1"
import test_dec_host as TH                                   # noqa: E402
import test_dec_top as TD                                    # noqa: E402
import test_dec_inter as TI                                  # noqa: E402
import test_tile_syntax as TT                                # noqa: E402
import seq_model as sm                                       # noqa: E402
import superres_model as srm                                 # noqa: E402
import cdf_map as M                                          # noqa: E402
import obu_parser as op                                      # noqa: E402
import host_regs as HR                                       # noqa: E402
from host_regs import (HostDecoder, R_ID, R_CTRL, R_CFG, R_DMA_STATUS, CTRL_RST, DMA_FLUSH_BUSY, DMA_ERR, dma_regs, unpack_dma)   # noqa: E402
from axil import AxiLite                                     # noqa: E402
from axi_mem import AxiMem                                   # noqa: E402

FLUSH_BASE = 0x80_0000_0000                                  # frame fi's picture at FLUSH_BASE + fi * FLUSH_STRIDE (40-bit space)
FLUSH_STRIDE = TI.SLOT_STRIDE


async def write_seq(ax, seq):
    for a, v in seq:
        await ax.write(a, v)


def store_entry(hdr, ph, planes, layout):
    """What a slot holds: the model's picture (for the frames the model decodes alone) and where the RTL finds it."""
    return dict(planes=planes, upscaled_width=ph.UpscaledWidth, frame_height=ph.FrameHeight, hdr=hdr,
                mem=[(b, stride) for b, stride, _, _ in layout] if layout else None)


async def flush_compare(ax, mem, hdr, base, planes, tag, stats):
    """After frame_done: wait for the row flush, then the memory image against the model's final picture."""
    seq, layout, total = dma_regs(hdr, base, visible=True)
    st = await ax.poll(R_DMA_STATUS, DMA_FLUSH_BUSY, 0, 40_000_000, tag)
    assert not (st & DMA_ERR), f"{tag}: DMA error flag (DMA_STATUS 0x{st:08x})"
    assert not mem.errors, f"{tag}: AXI protocol errors: {mem.errors[:4]}"
    got = unpack_dma(hdr, mem.read(base, total), layout)
    bad = []
    for plane in range(hdr.NumPlanes):
        _, _, W, H = layout[plane]
        assert len(planes[plane]) >= H and len(planes[plane][0]) >= W, f"{tag}: model picture {len(planes[plane][0])}x{len(planes[plane])} smaller than the flush {W}x{H}"
        for y in range(H):
            if got[plane][y] != planes[plane][y][:W]:
                bad += [(plane, y, x, got[plane][y][x], planes[plane][y][x]) for x in range(W) if got[plane][y][x] != planes[plane][y][x]]
        stats["pixels"] += W * H
    if bad:
        st2 = await ax.read(R_DMA_STATUS, record=False)
        ax.dut._log.info(f"{tag}: DMA_STATUS 0x{st2:08x} (rows flushed {st2 >> 16 & 0xFF}), {len(bad)} samples differ")
    assert not bad, f"{tag}: flushed picture: {len(bad)} samples differ, first (plane, y, x, rtl, model): {bad[:8]}"
    return layout


@cocotb.test()
async def dec_host_inter_vs_model(dut):
    debug = bool(os.environ.get("TD_DEBUG"))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    if hasattr(dut, "clk2x"):
        cocotb.start_soon(Clock(dut.clk2x, 5, unit="ns").start())

        async def phase_gen():
            while True:
                await cocotb.triggers.RisingEdge(dut.clk)
                await Timer(2.5, "ns")
                dut.phase.value = 1
                await Timer(5, "ns")
                dut.phase.value = 0
        dut.phase.value = 0
        cocotb.start_soon(phase_gen())
    ax = AxiLite(dut, dut.clk)
    mem = AxiMem(dut, dut.clk, seed=1, stall=0.2)
    cocotb.start_soon(mem.run())
    cocotb.start_soon(mem.run_read())
    dut.aresetn.value = 0
    await ClockCycles(dut.clk, 4)
    dut.aresetn.value = 1
    await ClockCycles(dut.clk, 2)
    ident = await ax.read(R_ID, expect=0xA71D0001)
    assert ident == 0xA71D0001, f"ID register reads 0x{ident:08x}"
    cfg = await ax.read(R_CFG)
    dut._log.info(f"dec_host ID 0x{ident:08x} CFG 0x{cfg:08x}: FBX {cfg >> 16 & 0xFF} FBY {cfg >> 8 & 0xFF} lanes {cfg >> 4 & 0xF} uram {cfg & 1}")
    stats = dict(frames=0, inter_frames=0, model_only=0, tiles=0, bytes=0, pixels=0, ref_loads=0, stage="lr")
    ivfs = [p for p in os.environ.get("TS_IVF", "").split(",") if p]
    assert ivfs, "TS_IVF=<stream.obu>[,...] is required"
    frames_sel = {int(x) for x in os.environ.get("TS_FRAMES", "").split(",") if x}
    fi_global = 0                                            # flush regions are per decoded frame across streams
    for path in ivfs:
        d = op.Decoder()
        with open(path, "rb") as fh:
            raw = fh.read()
        if path.endswith(".obu"):
            d.feed_annexb(raw)
        else:
            d.feed_ivf(raw)
        name = Path(path).name
        seq = sm.SequenceState()
        store = {}                                           # slot -> store_entry
        loaded = {}                                          # slot -> id(planes) written at the slot's own address
        hdr_iter = iter(d.frame_headers)
        for fi, tile_idx in enumerate(d.frames):
            ph = None
            for ph in hdr_iter:
                if ph.show_existing_frame:
                    seq.show_existing(ph)
                    if ph.frame_type == 0 and ph.frame_to_show_map_idx in store:
                        for i in range(8):
                            store[i] = store[ph.frame_to_show_map_idx]
                    continue
                break
            assert ph is not None and not ph.show_existing_frame, f"{name}: frame {fi} has no parsed header"
            ctx = seq.start_frame(ph)
            th0 = d.tiles[tile_idx[0]][0]
            n_tile_cols = len({d.tiles[ti][0].MiColStart for ti in tile_idx})
            tag = f"{name} frame {fi}"
            model_only = (frames_sel and fi not in frames_sel) or not HostDecoder.window_ok(th0, n_tile_cols)
            if model_only and not frames_sel:
                dut._log.info(f"{tag}: outside the window build's limits (sb128 / non-4:2:0 / tile columns / intrabc): model only")
            # ---- the model decodes the frame (the RTL follows tile by tile unless model_only)
            frame = {"refs": store}
            decs, summaries = [], []
            rows = M.rows_of(ctx.cdfs) if ctx.cdfs is not None else None
            if not model_only:
                await ax.write(R_CTRL, CTRL_RST)
                await ClockCycles(dut.clk, 4)
                await ax.write(R_CTRL, 0)
                fmt16 = th0.BitDepth > 8
                if not ph.FrameIsIntra:
                    # the bands and the slot table before the frame; references the RTL never flushed go in at a slot address
                    await write_seq(ax, HR.seg_map_seq(ctx.prev_segment_ids, th0.MiRows, th0.MiCols))
                    await write_seq(ax, HR.motion_field_seq(ctx.motion_field, th0.MiRows, th0.MiCols))
                    for i in range(7):
                        slot = ph.ref_frame_idx[i]
                        assert slot in store, f"{tag}: reference {i + 1} points at empty slot {slot}"
                        st = store[slot]
                        if st["mem"] is None:
                            if loaded.get(slot) != id(st["planes"]):
                                TI.load_slot(mem, st["hdr"], slot, st["planes"])
                                loaded[slot] = id(st["planes"])
                                stats["ref_loads"] += 1
                            st["mem"] = [(TI.SLOT_BASE + slot * TI.SLOT_STRIDE + p * TI.PLANE_STRIDE, stride)
                                         for p, (w, h, stride) in enumerate(TI.plane_geometry(st["hdr"], st["planes"]))]
                            st["mem_at_slot"] = True
                    await write_seq(ax, HR.ref_seq(ph, {s: dict(planes=e["mem"], upscaled_width=e["upscaled_width"], frame_height=e["frame_height"])
                                                        for s, e in store.items() if e.get("mem")}, fmt16))
                    stats["inter_frames"] += 1
                elif ctx.prev_segment_ids is not None or True:
                    pass                                     # intra frames read neither band
                flush_base = FLUSH_BASE + fi_global * FLUSH_STRIDE
                await TH.pipe_start(ax, th0, n_tile_cols, flush_base, mem, clear=False)
            for k, ti in enumerate(tile_idx):
                th, data = d.tiles[ti]
                dec = TD.RecFrame(th, data, frame, ctx=ctx)
                dec.decode_tile()
                decs.append(dec)
                summaries.append(sm.TileSummary(dec, keep_cdf=(k == ph.context_update_tile_id)))
                if not model_only:
                    await TH.run_tile(ax, th, dec, data, f"{tag} tile ({th.MiColStart},{th.MiRowStart})", stats, rows=rows)
            # ---- the in-loop filters: the sequencer runs them, the model follows
            planes = TH.model_filters(th0, decs, frame["planes"])
            layout = None
            if not model_only:
                await TH.pipe_finish(ax, tag)
                layout = await flush_compare(ax, mem, th0, flush_base, planes, tag, stats)
                fi_global += 1
                stats["frames"] += 1
                dut._log.info(f"{tag}: identical ({len(tile_idx)} tiles, {th0.UpscaledWidth}x{th0.FrameHeight} bd{th0.BitDepth}, "
                              f"{'inter' if not ph.FrameIsIntra else 'intra'}, {sum(1 for dd in decs for b in dd.blocks if b.get('inter'))} inter blocks)")
            else:
                stats["model_only"] += 1
            # ---- the frame becomes a reference (model picture + where the flush put it)
            seq.end_frame(ph, summaries, frame_cdfs=ctx.cdfs)
            planes_ref = TI.ref_planes_of(th0, planes)
            entry = store_entry(th0, ph, planes_ref, layout)
            for i in range(8):
                if (ph.refresh_frame_flags >> i) & 1:
                    store[i] = entry
            if debug:
                dut._log.info(f"{tag}: picture {len(planes_ref[0][0])}x{len(planes_ref[0])} -> slots {[i for i in range(8) if (ph.refresh_frame_flags >> i) & 1]}"
                              f"{' at ' + hex(layout[0][0]) if layout else ' (model only)'}")
            del decs
    assert not mem.errors, f"AXI errors: {mem.errors[:4]}"
    dut._log.info(f"OK: {stats}")
