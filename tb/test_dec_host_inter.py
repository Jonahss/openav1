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
import json
import os
import time
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
from host_regs import (R_START, HostDecoder, R_ID, R_CTRL, R_CFG, R_DMA_STATUS, CTRL_RST, DMA_FLUSH_BUSY, DMA_ERR, dma_regs, unpack_dma, plane_size)   # noqa: E402
from axil import AxiLite                                     # noqa: E402
from axi_mem import AxiMem                                   # noqa: E402

FLUSH_BASE = 0x80_0000_0000                                  # frame fi's picture at FLUSH_BASE + fi * FLUSH_STRIDE (40-bit space)
FLUSH_STRIDE = TI.SLOT_STRIDE


async def write_seq(ax, seq):
    for a, v in seq:
        await ax.write(a, v)


def store_entry(hdr, ph, planes, layout, gen):
    """What a slot holds: the model's picture (for the frames the model decodes alone) and where the RTL finds it.
    mem = the flushed picture's (base, stride) per plane, or None for a picture the RTL never produced: that one is
    loaded at the referencing SLOT's own address (slot_mem below), never cached here, because one entry sits in every
    slot the frame refreshed (test10316_84 frame 12: the key frame's entry, cached at slot 0's address, kept pointing
    there after the next model-only frame was loaded into slot 0). gen = the decoded-frame index that made the entry."""
    return dict(planes=planes, upscaled_width=ph.UpscaledWidth, frame_height=ph.FrameHeight, hdr=hdr, gen=gen,
                mem=[(b, stride) for b, stride, _, _ in layout] if layout else None)


def block_at(blocks, hdr, plane, x, y):
    """The model block holding sample (x, y) of a plane, as xcheck_frame reports it."""
    import av1_tables as T
    sx = hdr.subsampling_x if plane else 0
    sy = hdr.subsampling_y if plane else 0
    for b in blocks:
        if (b["c"] * 4) >> sx <= x < ((b["c"] * 4) + T.Block_Width[b["size"]]) >> sx and (b["r"] * 4) >> sy <= y < ((b["r"] * 4) + T.Block_Height[b["size"]]) >> sy:
            return (f"block r{b['r']} c{b['c']} size {b['size']} skip {b['skip']} inter {b.get('inter')} ymode {b['ymode']} ref {b.get('ref')} "
                    f"mvs {b.get('mvs')} mm {b.get('motion_mode')} ct {b.get('compound_type')} interp {b.get('interp')} seg {b['seg']}")
    return "no block"


async def cdf_save(ax, dut, cdf_area, engine, tag, stats):
    """Stage B: engine `engine`'s CDF rows to cdf_area (START_SAVE), right after the context tile's row."""
    await write_seq(ax, HR.sv_regs(cdf_area, engine))
    await ax.write(R_START, HR.START_SAVE)
    for _ in range(20000):
        stv = await ax.read(HR.R_FS_STATUS)
        if stv & HR.FS_SV_DONE:
            break
        await ClockCycles(dut.clk, 20)
    else:
        raise AssertionError(f"{tag}: the CDF save never finished (FS_STATUS 0x{stv:08x})")
    assert not (stv & HR.FS_SV_ERR), f"{tag}: the CDF save reported a write error"
    stats["sv_frames"] = stats.get("sv_frames", 0) + 1


async def flush_compare(ax, mem, hdr, base, planes, tag, stats, pre=None, deblocked=None, blocks=None, visible=True):
    """After frame_done: wait for the row flush, then the memory image against the model's final picture (the visible
    picture; an intrabc frame's flush holds the whole decoded MI area, its own copy source, visible=False)."""
    seq, layout, total = dma_regs(hdr, base, visible=visible)
    st = await ax.poll(R_DMA_STATUS, DMA_FLUSH_BUSY, 0, 40_000_000, tag)
    assert not (st & DMA_ERR), f"{tag}: DMA error flag (DMA_STATUS 0x{st:08x})"
    assert not mem.errors, f"{tag}: AXI protocol errors: {mem.errors[:4]}"
    got = unpack_dma(hdr, mem.read(base, total), layout)
    bad = []
    for plane in range(hdr.NumPlanes):
        W, H = plane_size(hdr, plane, True)                              # compared: the visible picture
        assert len(planes[plane]) >= H and len(planes[plane][0]) >= W, f"{tag}: model picture {len(planes[plane][0])}x{len(planes[plane])} smaller than the picture {W}x{H}"
        for y in range(H):
            if got[plane][y] != planes[plane][y][:W]:
                bad += [(plane, y, x, got[plane][y][x], planes[plane][y][x]) for x in range(W) if got[plane][y][x] != planes[plane][y][x]]
        stats["pixels"] += W * H
    if bad:
        st2 = await ax.read(R_DMA_STATUS, record=False)
        ax.dut._log.info(f"{tag}: DMA_STATUS 0x{st2:08x} (rows flushed {st2 >> 16 & 0xFF}), {len(bad)} samples differ")
        # the pictures themselves, for a look offline (tools/mismatch.py): the flush, the model's final picture and, when
        # the caller has them, its pre-filter and deblocked pictures, cropped to the flush
        dpath = os.path.join(os.environ.get("TS_TEST_DIR") or ".", "mismatch_" + "".join(c if c.isalnum() else "_" for c in tag) + ".json")
        def crop(pic):
            return None if pic is None else [[list(map(int, pic[p_][y_][:layout[p_][2]])) for y_ in range(min(layout[p_][3], len(pic[p_])))]
                                             for p_ in range(hdr.NumPlanes)]
        with open(dpath, "w") as fh:
            json.dump({"tag": tag, "wh": [[int(layout[p_][2]), int(layout[p_][3])] for p_ in range(hdr.NumPlanes)],
                       "rtl": [[list(map(int, r_)) for r_ in got[p_]] for p_ in range(hdr.NumPlanes)], "model": crop(planes),
                       "pre": crop(pre), "deblocked": crop(deblocked), "bad": [list(map(int, b_)) for b_ in bad]}, fh)
        ax.dut._log.info(f"{tag}: mismatch dump {dpath}")
        # where: per (plane, superblock row, line within the row) counts, and what every frame buffer holds at the first ones
        hist = {}
        for plane, y, x, _, _ in bad:
            k = (plane, (y << (hdr.subsampling_y if plane else 0)) >> 6)
            hist[k] = hist.get(k, 0) + 1
        ax.dut._log.info(f"{tag}: differing samples per (plane, superblock row): {sorted(hist.items())}")
        lines = {}
        for plane, y, x, _, _ in bad:
            if plane == 0:
                lines[y] = lines.get(y, 0) + 1
        ax.dut._log.info(f"{tag}: plane 0 lines hit (y: count), first 24: {sorted(lines.items())[:24]}")
        if blocks:
            seen = set()
            for plane, y, x, got_v, exp_v in bad:
                blk = block_at(blocks, hdr, plane, x, y)
                if blk in seen:
                    continue
                seen.add(blk)
                ax.dut._log.info(f"{tag}: plane {plane} ({x},{y}) rtl {got_v} model {exp_v} pre {pre[plane][y][x] if pre is not None else None}: {blk}")
                if len(seen) >= 10:
                    break
        if pre is not None:
            ax.dut._log.info(f"{tag}: model pre-filter {[pre[p_][y_][x_] if y_ < len(pre[p_]) and x_ < len(pre[p_][0]) else None for p_, y_, x_, _, _ in bad[:6]]}, "
                             f"deblocked {[deblocked[p_][y_][x_] if y_ < len(deblocked[p_]) and x_ < len(deblocked[p_][0]) else None for p_, y_, x_, _, _ in bad[:6]]}, "
                             f"final {[e for _, _, _, _, e in bad[:6]]}")
        for buf in range(4):                                 # 0 fb0, 1 CdefFrame, 2 LrFrame, 3 SR's upscaled rows
            await ax.write(R_CTRL, buf << 4)
            got_b = []
            for plane, y, x, _, _ in bad[:6]:
                await ax.write(HR.R_HADDR, (plane << 28) | (y << 14) | x)
                got_b.append(await ax.read(HR.R_HDATA, record=False))
            ax.dut._log.info(f"{tag}: fb{buf} holds {got_b} at {[(p_, y_, x_) for p_, y_, x_, _, _ in bad[:6]]}")
        await ax.write(R_CTRL, 0)
    assert not bad, f"{tag}: flushed picture: {len(bad)} samples differ, first (plane, y, x, rtl, model): {bad[:8]}"
    return layout


# The ILA probe set of the ila-u200 debug build (fpga/u200/pre_opt_ila.tcl): hierarchical name under dec_host -> how to read it.
# Struct columns (ev_tx / t) are the tx_rec_t fields (TT.TX_FIELDS); the ILA exports them as <bus>[<field>].
ILA_PROBES = [
    "start", "u_dec/ev_kind", "u_dec/ev_pop", "u_dec/ev_sb_c", "u_dec/ev_sb_r", "u_dec/ev_tx", "u_dec/ev_blk", "u_dec/ev_valid", "u_dec/tx_inflight",
    "u_dec/u_rc/a_valid", "u_dec/u_rc/aadd_x_r", "u_dec/u_rc/aadd_y_r", "u_dec/u_rc/add_r", "u_dec/u_rc/ci", "u_dec/u_rc/cj",
    "u_dec/u_rc/dq_ci2", "u_dec/u_rc/dq_cj2", "u_dec/u_rc/dq_out", "u_dec/u_rc/dq_v2", "u_dec/u_rc/fb4_q", "u_dec/u_rc/itx_addr",
    "u_dec/u_rc/itx_clr", "u_dec/u_rc/itx_data", "u_dec/u_rc/itx_done", "u_dec/u_rc/itx_start", "u_dec/u_rc/itx_we", "u_dec/u_rc/p_valid",
    "u_dec/u_rc/pci", "u_dec/u_rc/pcj", "u_dec/u_rc/q_valid", "u_dec/u_rc/res_addr", "u_dec/u_rc/res_data4", "u_dec/u_rc/rs",
    "u_dec/u_rc/sd_k", "u_dec/u_rc/sd_valid", "u_dec/u_rc/sk", "u_dec/u_rc/t",
    "u_dec/u_ts/u_coef/c", "u_dec/u_ts/u_coef/cul", "u_dec/u_ts/u_coef/eob", "u_dec/u_ts/u_coef/glen", "u_dec/u_ts/u_coef/gx",
    "u_dec/u_ts/u_coef/level", "u_dec/u_ts/u_coef/nz_we", "u_dec/u_ts/u_coef/pos", "u_dec/u_ts/u_coef/q_nz", "u_dec/u_ts/u_coef/q_raw",
    "u_dec/u_ts/u_coef/sign", "u_dec/u_ts/u_coef/st",
]
ILA_STRUCTS = {"u_dec/ev_tx": TT.TX_FIELDS, "u_dec/u_rc/t": TT.TX_FIELDS, "u_dec/ev_blk": TT.BLK_FIELDS}   # struct -> fields (twin-only columns are fine: the diff skips them)


async def ila_log(dut, path):
    """TS_ILA_LOG=<file>: the ILA probe set of the ila-u200 debug build, sampled every clk from the first ev_pop of the
    RTL-decoded frame for TS_ILA_N cycles (default 8192, the ILA depth), one hex value per column, so a capture off
    the card (the ILA CSV export) can be diffed against the twin cycle by cycle: tools/ila_diff.py <ila.csv> <this>."""
    n_max = int(os.environ.get("TS_ILA_N", "8192"))
    handles = []
    for name in ILA_PROBES:
        h = dut
        try:
            for part in name.split("/"):
                h = getattr(h, part)
            int(h.value)
        except Exception as e:                                   # not reachable through VPI: logged as NA
            dut._log.warning(f"ila_log: {name} unavailable ({e})")
            h = None
        handles.append((name, h))
    cols = []
    for name, h in handles:
        if name in ILA_STRUCTS:
            cols += [f"{name}[{f}]" for f, _ in ILA_STRUCTS[name]]
        else:
            cols.append(name)
    out = open(path, "w")
    out.write(",".join(cols) + "\n")
    armed = False
    n = 0
    while n < n_max:
        await cocotb.triggers.RisingEdge(dut.clk)
        if not armed:
            if int(dut.u_dec.ev_pop.value) != 1:
                continue
            armed = True
        row = []
        for name, h in handles:
            if h is None:
                row.append("NA" if name not in ILA_STRUCTS else ",".join(["NA"] * len(ILA_STRUCTS[name])))
                continue
            v = int(h.value)
            if name in ILA_STRUCTS:
                d = TT.unpack(ILA_STRUCTS[name], v)
                row += [f"{d[f]:x}" for f, _ in ILA_STRUCTS[name]]
            else:
                row.append(f"{v:x}")
        out.write(",".join(row) + "\n")
        n += 1
    out.close()
    dut._log.info(f"ila_log: {n} cycles -> {path}")


@cocotb.test()
async def dec_host_inter_vs_model(dut):
    debug = bool(os.environ.get("TD_DEBUG"))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    if os.environ.get("TD_MCTRACE"):
        cocotb.start_soon(TI.mc_trace(dut, dut.u_dec.u_rc))      # the same MC launch trace as test_dec_inter
    if os.environ.get("TD_ARTRACE"):
        cocotb.start_soon(TI.ar_trace(dut))
    if os.environ.get("TS_ILA_LOG"):
        cocotb.start_soon(ila_log(dut, os.environ["TS_ILA_LOG"]))
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
        fsb = os.environ.get("TS_FSTATE_B") == "1"           # stage B: the card loads the segment map and the CDF rows itself
        slot_areas = {}                                      # slot -> dict(seg=(base, rowb) | None, cdf=base | None)
        slot_mem = {}                                        # slot -> (gen, [(base, stride) per plane]) of the model picture loaded at the slot's own address
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
            dut._log.info(f"{tag}: {th0.FrameWidth}x{th0.FrameHeight} (upscaled {th0.UpscaledWidth}) bd{th0.BitDepth} {'intra' if ph.FrameIsIntra else 'inter'}, "
                          f"refs {list(ph.ref_frame_idx) if not ph.FrameIsIntra else []}, refresh 0x{ph.refresh_frame_flags:02x}, cdef {HR.cdef_on(th0)} sr {th0.use_superres} "
                          f"lr {[t for t in th0.FrameRestorationType[:th0.NumPlanes]]}")
            model_only = (frames_sel and fi not in frames_sel) or not HostDecoder.window_ok(th0, n_tile_cols, sb128=bool(int(os.environ.get("SB128", "0"))), fbx=int(os.environ.get("FBX", "9")), fby=int(os.environ.get("FBY", "9")), te=int(os.environ.get("TE", "1")))
            if model_only and not frames_sel:
                dut._log.info(f"{tag}: outside the window build's limits (sb128 / non-4:2:0 / tile columns): model only")
            # ---- the model decodes the frame (the RTL follows tile by tile unless model_only)
            frame = {"refs": store}
            decs, summaries = [], []
            loaded_cdf = False
            cdf_area = None
            if not model_only:
                await ax.write(R_CTRL, CTRL_RST)
                await ClockCycles(dut.clk, 4)
                await ax.write(R_CTRL, 0)
                fmt16 = th0.BitDepth > 8
                if fsb:
                    # stage B: the loader reads the primary slot's areas (written by the card in earlier frames)
                    prim = slot_areas.get(ph.ref_frame_idx[ph.primary_ref_frame]) if ph.primary_ref_frame != sm.PRIMARY_REF_NONE else None
                    seg_ld = prim["seg"] if (ctx.prev_segment_ids is not None and prim and prim.get("seg")) else None
                    cdf_ld = prim["cdf"] if (ctx.cdfs is not None and prim and prim.get("cdf") is not None) else None
                    assert (ctx.prev_segment_ids is None) == (seg_ld is None), f"{tag}: the model has a previous segment map but no slot area holds one"
                    assert (ctx.cdfs is None) == (cdf_ld is None), f"{tag}: the model inherits CDFs but no slot area holds them"
                    await write_seq(ax, HR.ld_regs(seg=seg_ld, cdf_base=cdf_ld, zero_seg=(seg_ld is None and not ph.FrameIsIntra)))
                    await ax.write(R_START, HR.START_LOAD)
                    for _ in range(20000):
                        stv = await ax.read(HR.R_FS_STATUS)
                        if stv & HR.FS_LD_DONE:
                            break
                        await ClockCycles(dut.clk, 20)
                    else:
                        raise AssertionError(f"{tag}: the frame-state loader never finished (FS_STATUS 0x{stv:08x})")
                    assert not (stv & HR.FS_LD_ERR), f"{tag}: the frame-state loader reported a read error"
                    loaded_cdf = cdf_ld is not None
                    stats["ld_frames"] = stats.get("ld_frames", 0) + 1
                if not ph.FrameIsIntra:
                    # the bands and the slot table before the frame; references the RTL never flushed go in at a slot address
                    if not fsb:
                        await write_seq(ax, HR.seg_map_seq(ctx.prev_segment_ids, th0.MiRows, th0.MiCols))
                    await write_seq(ax, HR.motion_field_seq(ctx.motion_field, th0.MiRows, th0.MiCols))
                    ref_store = {}
                    for i in range(7):
                        slot = ph.ref_frame_idx[i]
                        assert slot in store, f"{tag}: reference {i + 1} points at empty slot {slot}"
                        st = store[slot]
                        if st["mem"] is None:
                            # a picture the RTL never flushed: in the AXI memory at this slot's own address (once per entry per slot)
                            if slot_mem.get(slot, (None,))[0] != st["gen"]:
                                TI.load_slot(mem, st["hdr"], slot, st["planes"])
                                slot_mem[slot] = (st["gen"], [(TI.SLOT_BASE + slot * TI.SLOT_STRIDE + p * TI.PLANE_STRIDE, stride)
                                                              for p, (w, h, stride) in enumerate(TI.plane_geometry(st["hdr"], st["planes"]))])
                                stats["ref_loads"] += 1
                            planes_at = slot_mem[slot][1]
                        else:
                            planes_at = st["mem"]
                        ref_store[slot] = dict(planes=planes_at, upscaled_width=st["upscaled_width"], frame_height=st["frame_height"])
                    await write_seq(ax, HR.ref_seq(ph, ref_store, fmt16))
                    stats["inter_frames"] += 1
                elif ctx.prev_segment_ids is not None or True:
                    pass                                     # intra frames read neither band
                flush_base = FLUSH_BASE + fi_global * FLUSH_STRIDE
                # the frame-state sideband areas (docs/frame-state.md) in the frame's own region, past any picture
                fs_seq, fsL = HR.fs_regs(th0, flush_base + 0x1800_0000)
                await write_seq(ax, fs_seq)
                await TH.pipe_start(ax, th0, n_tile_cols, flush_base, mem, clear=False, visible=not th0.allow_intrabc)
            rows = False if loaded_cdf else (M.rows_of(ctx.cdfs) if ctx.cdfs is not None else None)   # False: the loader wrote them
            cdf_area = flush_base + 0x1C00_0000 if not model_only else None      # stage B: this frame's saved CDF rows
            pending_k = []
            pending = []                                      # the tiles of a tile row go to the engines together
            for k, ti in enumerate(tile_idx):
                th, data = d.tiles[ti]
                dec = TD.RecFrame(th, data, frame, ctx=ctx)
                dec.decode_tile()
                decs.append(dec)
                summaries.append(sm.TileSummary(dec, keep_cdf=(k == ph.context_update_tile_id)))
                if not model_only:
                    if pending and th.MiRowStart != pending[0][0].MiRowStart:
                        await TH.run_tile_row(ax, pending, f"{tag} tile row {pending[0][0].MiRowStart}", stats, rows=rows)
                        if fsb and ph.context_update_tile_id in pending_k and not getattr(ph, "disable_frame_end_update_cdf", 0):
                            await cdf_save(ax, dut, cdf_area, pending_k.index(ph.context_update_tile_id), tag, stats)
                        pending, pending_k = [], []
                    pending.append((th, dec, data)); pending_k.append(k)
                    if k == len(tile_idx) - 1:
                        await TH.run_tile_row(ax, pending, f"{tag} tile row {pending[0][0].MiRowStart}", stats, rows=rows)
                        if fsb and ph.context_update_tile_id in pending_k and not getattr(ph, "disable_frame_end_update_cdf", 0):
                            await cdf_save(ax, dut, cdf_area, pending_k.index(ph.context_update_tile_id), tag, stats)
                        pending, pending_k = [], []
            # ---- the in-loop filters: the sequencer runs them, the model follows (LF in place: frame["planes"] = deblocked)
            pre = [[row[:] for row in p_] for p_ in frame["planes"]] if not model_only else None
            t_mf = time.monotonic()                       # pure-Python LF/CDEF/SR/LR: minutes at 1080p, hours with LR on every plane
            planes = TH.model_filters(th0, decs, frame["planes"])
            dut._log.info(f"{tag}: model filters done in {time.monotonic() - t_mf:.0f} s (simulated time stood still)")
            layout = None
            if not model_only:
                await TH.pipe_finish(ax, tag)
                layout = await flush_compare(ax, mem, th0, flush_base, planes, tag, stats, pre=pre, deblocked=frame["planes"],
                                             blocks=[b for dd in decs for b in dd.blocks], visible=not th0.allow_intrabc)
                fi_global += 1
                stats["frames"] += 1
                dut._log.info(f"{tag}: identical ({len(tile_idx)} tiles, {th0.UpscaledWidth}x{th0.FrameHeight} bd{th0.BitDepth}, "
                              f"{'inter' if not ph.FrameIsIntra else 'intra'}, {sum(1 for dd in decs for b in dd.blocks if b.get('inter'))} inter blocks)")
            else:
                stats["model_only"] += 1
            # ---- the frame becomes a reference (model picture + where the flush put it)
            seq.end_frame(ph, summaries, frame_cdfs=ctx.cdfs)
            if not model_only and ph.refresh_frame_flags:
                # the card's frame-state sideband against the model's saved motion field and segmentation map (7.19 / 7.20)
                for _ in range(2000):
                    if not (await ax.read(HR.R_FS_STATUS)) & 1:
                        break
                    await ClockCycles(dut.clk, 50)
                else:
                    raise AssertionError(f"{tag}: the frame-state sideband DMA never went idle")
                saved = seq.slots[next(i for i in range(8) if (ph.refresh_frame_flags >> i) & 1)]
                c_refs, c_mvs = HR.fs_unpack_mv(th0, mem.read(fsL["mv_base"], fsL["mv_bytes"]), fsL["mv_rowb"])
                bad = []
                for y8 in range(th0.MiRows >> 1):
                    for x8 in range(th0.MiCols >> 1):
                        mr = saved.ref_frames[2 * y8 + 1][2 * x8 + 1]
                        mm = saved.mvs[2 * y8 + 1][2 * x8 + 1] if mr > 0 else (0, 0)
                        cr, cm = c_refs[y8][x8], c_mvs[y8][x8]
                        if (cr != (mr if mr > 0 else 0)) or (cr and tuple(cm) != tuple(mm)):
                            bad.append((y8, x8, (cr, cm), (mr, mm)))
                assert not bad, f"{tag}: {len(bad)} MV sideband cells differ (y8, x8, card, model): {bad[:6]}"
                if not (getattr(ph, "segmentation_enabled", 0) and not getattr(ph, "segmentation_update_map", 1)):
                    c_seg = HR.fs_unpack_seg(th0, mem.read(fsL["seg_base"], fsL["seg_bytes"]), fsL["seg_rowb"])
                    badseg = [(r, c, c_seg[r][c], saved.segment_ids[r][c]) for r in range(th0.MiRows) for c in range(th0.MiCols)
                              if c_seg[r][c] != saved.segment_ids[r][c]]
                    assert not badseg, f"{tag}: {len(badseg)} segment-id sideband cells differ (row, col, card, model): {badseg[:6]}"
                stats["fs_frames"] = stats.get("fs_frames", 0) + 1
            planes_ref = TI.ref_planes_of(th0, planes)
            entry = store_entry(th0, ph, planes_ref, None if os.environ.get("TS_REFS_FROM_MODEL") == "1" else layout, fi)
            if fsb and not model_only:
                # the slot's areas for later frames (7.20): the map is this frame's unless it kept the previous one verbatim;
                # the CDF rows are the saved ones unless the frame-end update was disabled (then the ones it started with)
                primA = slot_areas.get(ph.ref_frame_idx[ph.primary_ref_frame]) if ph.primary_ref_frame != sm.PRIMARY_REF_NONE else None
                seg_a = (primA["seg"] if primA else None) if (getattr(ph, "segmentation_enabled", 0) and not getattr(ph, "segmentation_update_map", 1)) else (fsL["seg_base"], fsL["seg_rowb"])
                cdf_a = (primA["cdf"] if primA else None) if getattr(ph, "disable_frame_end_update_cdf", 0) else cdf_area
                areas = dict(seg=seg_a, cdf=cdf_a)
            for i in range(8):
                if (ph.refresh_frame_flags >> i) & 1:
                    store[i] = entry
                    if fsb and not model_only:
                        slot_areas[i] = areas
            if debug:
                dut._log.info(f"{tag}: picture {len(planes_ref[0][0])}x{len(planes_ref[0])} -> slots {[i for i in range(8) if (ph.refresh_frame_flags >> i) & 1]}"
                              f"{' at ' + hex(layout[0][0]) if layout else ' (model only)'}")
            del decs
    assert not mem.errors, f"AXI errors: {mem.errors[:4]}"
    dut._log.info(f"OK: {stats}")
