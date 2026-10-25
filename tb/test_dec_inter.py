"""Inter streams through the whole decoder (dec_top) against the model, frame by frame (stage 2a).

The frame sequence follows tb/seq_model.py (inherited CDFs, segmentation map, motion field: loaded into the DUT's
header / band ports as the host will program them). The reference pictures are the model's own reconstructed
frames (STAGE deep: TS_STAGE=recon compares and references the pre-filter pictures, upscaled when super-res so the reference geometry is the stream's, lr the final ones): they are
written into the AXI memory model that ref_fetch reads, at one base per (slot, plane), and the slot table
registers point the RTL at them. So a motion compensation bug cannot hide in a filter bug and vice versa.
Env: TS_IVF=a.obu,b.obu (required), TS_STAGE (recon | lf | cdef | lr), TS_FRAMES, TD_DEBUG.
"""
import os
from pathlib import Path
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer

import test_dec_top as TD
import test_tile_syntax as TT
import seq_model as sm
import superres_model as srm
import cdf_map as M
import obu_parser as op
from axi_mem import AxiMem

SLOT_BASE = 1 << 32                 # slot s, plane p at SLOT_BASE + s * SLOT_STRIDE + p * PLANE_STRIDE (40-bit addresses)
# A plane must fit its window: the largest picture a build stores is (1 << FBX) x (1 << FBY) samples of 2 bytes, 512 MB
# at fb14x14. With 16 MB planes (until 2026-10-22) an 8192 x 1088 10-bit luma plane (17 MB; test10505's super-res
# reference) ran into its own chroma plane, so the RTL read chroma for luma rows 1024 and up: 18% of the frame wrong.
PLANE_STRIDE = 0x2000_0000
SLOT_STRIDE = 3 * PLANE_STRIDE
assert 8 * SLOT_STRIDE + SLOT_BASE < (1 << 40)


def ref_planes_of(hdr, planes):
    """The picture a frame leaves in the reference store. Below the lr stage a super-res frame's pictures are still
    FrameWidth wide, but the stream's headers (RefUpscaledWidth, so is_scaled in the model, ref_scaled in the RTL and
    the warped-motion syntax that depends on it) describe UpscaledWidth-wide references: upscale the stage's picture
    (7.16) so the geometry both sides see matches the stream. (test11 frame 7: 989-wide frame, 719-wide pre-upscale
    references under affine global motion; the RTL took the unscaled path, the model the scaled one, both 'right'.)"""
    if TD.STAGE == "lr" or not hdr.use_superres:          # at lr finish_frame already upscaled the picture (7.16)
        return planes
    return srm.upscale(hdr, planes)


def plane_geometry(hdr, planes):
    """(width, height, stride bytes) per plane of a stored picture, in the DMA flush layout."""
    bps = 2 if hdr.BitDepth > 8 else 1
    out = []
    for p in range(hdr.NumPlanes):
        w, h = len(planes[p][0]), len(planes[p])
        out.append((w, h, ((w * bps + 7) // 8) * 8))
    return out


def load_slot(mem, hdr, slot, planes):
    """Write a reference picture into the AXI memory at its slot's bases."""
    bps = 2 if hdr.BitDepth > 8 else 1
    for p, (w, h, stride) in enumerate(plane_geometry(hdr, planes)):
        base = SLOT_BASE + slot * SLOT_STRIDE + p * PLANE_STRIDE
        assert h * stride <= PLANE_STRIDE, f"slot {slot} plane {p}: {w}x{h} ({h * stride} bytes) overruns its {PLANE_STRIDE}-byte window"
        for y in range(h):
            row = planes[p][y]
            if bps == 1:
                data = bytes(row)
            else:
                data = b"".join(v.to_bytes(2, "little") for v in row)
            mem.load(base + y * stride, data)


async def mc_trace(dut, rc):
    """TD_MCTRACE: every motion-compensation launch of the reconstruction (rc = the recon_top instance)."""
    # every motion-compensation run the reconstruction launches: the strip's geometry and reference position
    while True:
        await RisingEdge(dut.clk)
        if int(rc.mc_start.value):
            bb = TT.unpack(TT.BLK_FIELDS, int(rc.b.value))
            dut._log.info(f"MC pl{int(rc.mc_pl.value)} l{int(rc.mc_l.value)} at ({int(rc.mc_px.value)},{int(rc.mc_py.value)}) "
                          f"{int(rc.mc_sw.value)}x{int(rc.mc_sh.value)} mode {int(rc.mc_mode.value)} warp {int(rc.mc_warp.value)} "
                          f"ref x0 {int(rc.rf_x0.value.signed_integer)} y0 {int(rc.rf_y0.value.signed_integer)} "
                          f"w {int(rc.rf_w.value)} h {int(rc.rf_h.value)} steps {int(rc.sc_xs.value)},{int(rc.sc_ys.value)} "
                          f"frac {int(rc.mc_xf0.value)},{int(rc.mc_yf0.value)} unit ({int(rc.mc_ux.value)},{int(rc.mc_uy.value)}) "
                          f"{int(rc.mc_uw.value)}x{int(rc.mc_uh.value)} strips {int(rc.mc_nsc.value)}x{int(rc.mc_nsr.value)} "
                          f"blk r{bb['r']} c{bb['c']} bs{bb['bsize']} mm{bb['motion_mode']} "
                          f"ob {int(rc.ob_active.value)}/{int(rc.ob_pass.value)}/{int(rc.ob_k.value)} comp {int(rc.mc_comp.value)} "
                          f"cmv {int(rc.mc_cmv.value)} uw_warp {int(rc.unit_warp.value)} "
                          f"blkmv ({TT.to_signed(bb['mv_row'], 18)},{TT.to_signed(bb['mv_col'], 18)}) ref {bb['ref0']}/{bb['ref1']} "
                          f"umv0 ({int(rc.u_mv0r.value.signed_integer)},{int(rc.u_mv0c.value.signed_integer)}) umv1 ({int(rc.u_mv1r.value.signed_integer)},{int(rc.u_mv1c.value.signed_integer)}) "
                          f"uref {int(rc.u_ref0.value)}/{int(rc.u_ref1.value)} wt {int(rc.mc_wt0.value)},{int(rc.mc_wt1.value)}")



@cocotb.test()
async def dec_inter_vs_model(dut):
    debug = bool(os.environ.get("TD_DEBUG"))
    if os.environ.get("TD_MCTRACE"):
        cocotb.start_soon(mc_trace(dut, dut.u_rc))
    if os.environ.get("TD_MVTRACE"):
        async def mv_trace():
            # every find_mv_stack result of the syntax block: position, refs, NumMvFound, RefStackMv[0..1], GlobalMvs
            blk = dut.u_ts.u_blk; mvs = blk.u_mvs
            def arr(name, i):
                try:
                    return int(getattr(mvs, name)[i].value.signed_integer)
                except Exception:
                    return None
            while True:
                await RisingEdge(dut.clk)
                if int(mvs.done.value):
                    dut._log.info(f"MVS blk r{int(blk.br.value)} c{int(blk.bc.value)} bs{int(blk.bs.value)} ref {int(blk.rf0.value)}/{int(blk.rf1.value)} "
                                  f"n_found {int(mvs.n_found.value)} st0 ({arr('st0_row', 0)},{arr('st0_col', 0)}) ({arr('st0_row', 1)},{arr('st0_col', 1)}) "
                                  f"st1 ({arr('st1_row', 0)},{arr('st1_col', 0)}) gmv ({arr('gmv_row', 0)},{arr('gmv_col', 0)}) "
                                  f"pred ({int(mvs.pred_row.value.signed_integer)},{int(mvs.pred_col.value.signed_integer)}) "
                                  f"ctx new {int(mvs.new_mv_ctx.value)} ref {int(mvs.ref_mv_ctx.value)} zero {int(mvs.zero_mv_ctx.value)}")
        cocotb.start_soon(mv_trace())
    if os.environ.get("TD_TXTRACE"):
        async def tx_trace():
            # every transform-block record the syntax layer hands the reconstruction (at the tx_done / tx_ack handshake):
            # plane, position, size, type, eob. TD_TXTRACE=<plane>[,<ymax>] narrows it to a plane (and rows below ymax).
            sel = os.environ["TD_TXTRACE"].split(",")
            pl_sel = int(sel[0]) if sel[0] not in ("", "all") else None
            ymax = int(sel[1]) if len(sel) > 1 else None
            while True:
                await RisingEdge(dut.clk)
                if int(dut.tx_done.value) and int(dut.tx_ack.value):
                    t = TT.unpack(TT.TX_FIELDS, int(dut.tx_rec.value))
                    if (pl_sel is None or t["plane"] == pl_sel) and (ymax is None or t["y"] < ymax):
                        dut._log.info(f"TX pl{t['plane']} at ({t['x']},{t['y']}) txsz {t['txsz']} txtype {t['txtype']} eob {t['eob']} "
                                      f"skip {t['skip']} lossless {t['lossless']} slot {t['slot']}")
        cocotb.start_soon(tx_trace())
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
    for s in ("in_valid", "in_eos", "def_we", "tile_start", "hdr", "ihdr", "rh", "lh", "ch", "lf_start", "cdef_start", "lr_start",
              "pipe_en", "frame_start", "n_tile_cols", "lr_from_deblocked", "h_buf", "h_plane", "h_x", "h_y", "mf_we", "sp_we",
              "ref_slot", "ref_planes", "ref_dims", "ref_scales", "ref_fmt16"):
        if hasattr(dut, s):
            getattr(dut, s).value = 0
    mem = AxiMem(dut, dut.clk, seed=1, stall=0.2)
    cocotb.start_soon(mem.run_read())
    dut.rst.value = 1
    TD.run_tile._perf_prev = None
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    stats = dict(frames=0, inter_frames=0, tiles=0, blocks=0, txblocks=0, pixels=0, cycles=0, lf_cycles=0, cdef_cycles=0,
                 lr_cycles=0, stage=TD.STAGE, ref_loads=0)
    ivfs = [p for p in os.environ.get("TS_IVF", "").split(",") if p]
    assert ivfs, "TS_IVF=<stream.obu>[,...] is required"
    frames_sel = {int(x) for x in os.environ.get("TS_FRAMES", "").split(",") if x}
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
        store = {}                              # slot -> dict(planes, upscaled_width, frame_height, hdr)
        loaded = {}                             # slot -> id(planes) written into the AXI memory
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
            if frames_sel and fi not in frames_sel:
                # keep the sequence state and the reference store in step without running the RTL
                frame = {"refs": store}
                summaries = []
                for k, ti in enumerate(tile_idx):
                    th, data = d.tiles[ti]
                    dec = TD.RecFrame(th, data, frame, ctx=ctx)
                    dec.decode_tile()
                    summaries.append(sm.TileSummary(dec, keep_cdf=(k == ph.context_update_tile_id)))
                    del dec
                planes = srm.upscale(th0, frame["planes"]) if ph.use_superres else frame["planes"]   # model-only frame: pre-filter, stream geometry
                seq.end_frame(ph, summaries, frame_cdfs=ctx.cdfs)
                for i in range(8):
                    if (ph.refresh_frame_flags >> i) & 1:
                        store[i] = dict(planes=planes, upscaled_width=ph.UpscaledWidth, frame_height=ph.FrameHeight, hdr=th0)
                continue
            tag = f"{name} frame {fi}"
            # ---- the frame-level state into the DUT: inter header, bands, and the references
            dut.ihdr.value = TT.pack(TT.IHDR_FIELDS, TT.ihdr_vals(th0))
            await TT.load_frame_state(dut, ph, ctx)
            if not ph.FrameIsIntra:
                ref_slot = 0
                ref_planes = 0
                ref_dims = 0
                ref_scales = 0
                for i in range(7):
                    slot = ph.ref_frame_idx[i]
                    ref_slot |= slot << (3 * i)
                    assert slot in store, f"{tag}: reference {i + 1} points at empty slot {slot}"
                    st = store[slot]
                    if loaded.get(slot) != id(st["planes"]):
                        load_slot(mem, st["hdr"], slot, st["planes"])
                        loaded[slot] = id(st["planes"])
                        stats["ref_loads"] += 1
                    for p, (w, h, stride) in enumerate(plane_geometry(st["hdr"], st["planes"])):
                        base = SLOT_BASE + slot * SLOT_STRIDE + p * PLANE_STRIDE
                        ref_planes |= (base | (stride << 40)) << (72 * (3 * slot + p))
                    ref_dims |= (st["upscaled_width"] | (st["frame_height"] << 16)) << (32 * slot)
                    # 7.11.3.3 scale factors of this reference for the current frame (REF_SCALE_SHIFT 14)
                    xsc = ((st["upscaled_width"] << 14) + ph.FrameWidth // 2) // ph.FrameWidth
                    ysc = ((st["frame_height"] << 14) + ph.FrameHeight // 2) // ph.FrameHeight
                    ref_scales |= (xsc | (ysc << 16)) << (32 * slot)
                dut.ref_slot.value = ref_slot
                dut.ref_planes.value = ref_planes
                dut.ref_dims.value = ref_dims
                dut.ref_scales.value = ref_scales
                dut.ref_fmt16.value = 1 if th0.BitDepth > 8 else 0
                stats["inter_frames"] += 1
            rows = M.rows_of(ctx.cdfs) if ctx.cdfs is not None else M.default_rows(th0.base_q_idx)
            # ---- tiles: the model decodes each, the RTL follows, records compared
            frame = {"refs": store}
            events, blocks_all, decs, summaries = [], [], [], []
            for k, ti in enumerate(tile_idx):
                th, data = d.tiles[ti]
                ttag = f"{tag} tile ({th.MiColStart},{th.MiRowStart})"
                dec = TD.RecFrame(th, data, frame, ctx=ctx)
                dec.decode_tile()
                decs.append(dec)
                summaries.append(sm.TileSummary(dec, keep_cdf=(k == ph.context_update_tile_id)))
                events.extend(dec.pred_events)
                events.extend(dec.recon_events)
                blocks_all.extend(dec.blocks)
                stats["blocks"] += len(dec.blocks)
                await TD.run_tile(dut, th, dec, data, ttag, stats, debug, fi, rows=rows)
                await ClockCycles(dut.clk, 4)
                assert not int(dut.unsupported.value), f"{ttag}: RTL flagged an unsupported feature (inter beyond stage 2a?)"
            planes_cmp, buf = await TD.finish_frame(dut, th0, decs, frame["planes"], tag, stats)
            dut.h_buf.value = buf
            await TD.compare_frame(dut, th0, planes_cmp, tag, stats, events, blocks_all)
            stats["frames"] += 1
            dut._log.info(f"{tag}: identical ({len(tile_idx)} tiles, {th0.MiCols * 4}x{th0.MiRows * 4} MI area, bd{th0.BitDepth}, "
                          f"{'inter' if not ph.FrameIsIntra else 'intra'}, {sum(1 for b in blocks_all if b.get('inter'))} inter blocks)")
            # ---- the frame becomes a reference (the stage's pictures: the RTL reads the same ones)
            seq.end_frame(ph, summaries, frame_cdfs=ctx.cdfs)
            if debug:
                dut._log.info(f"{tag}: stored picture {len(planes_cmp[0][0])}x{len(planes_cmp[0])} (upscaled {ph.UpscaledWidth}, frame {ph.FrameWidth}x{ph.FrameHeight}, "
                              f"MI {th0.MiCols * 4}x{th0.MiRows * 4}, superres {ph.use_superres}) -> slots {[i for i in range(8) if (ph.refresh_frame_flags >> i) & 1]}")
            planes_ref = ref_planes_of(th0, planes_cmp)
            for i in range(8):
                if (ph.refresh_frame_flags >> i) & 1:
                    store[i] = dict(planes=planes_ref, upscaled_width=ph.UpscaledWidth, frame_height=ph.FrameHeight, hdr=th0)
            del decs
    assert not mem.errors, f"AXI errors: {mem.errors[:4]}"
    dut._log.info(f"OK: {stats}")
