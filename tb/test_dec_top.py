"""End-to-end intra decoder test: dec_top (tile_syntax + recon_top + frame_mem) vs the Python reconstruction
model on generated streams. Every tile of a frame is decoded by both; then the RTL frame buffer is read back
through the host port and compared with the model's CurrFrame (pre loop filter) over the MI-aligned area.

Env: TS_SEEDS (as in test_tile_syntax), TS_W / TS_H (frame size, default 128x96), TS_FMT, TS_SCREEN (1/0/rand),
TS_NOLR, TD_DEBUG=1 (per-tile record counting + hang dump).
"""
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
import cdf_map as M                     # noqa: E402
import cdef_model as cdm                # noqa: E402
import lf_model as lfm                  # noqa: E402
import lr_model as lrm                  # noqa: E402
import test_cdef_top as TC              # noqa: E402
import recon_model as rm                # noqa: E402
import test_lf_top as TL                # noqa: E402
import test_tile_syntax as TT           # noqa: E402
import xcheck_frame as xf               # noqa: E402

STAGE = os.environ.get("TS_STAGE", "recon")      # recon | lf | cdef | lr : how far the RTL and the model go before comparing

REC_FIELDS = [  # syn_pkg::rec_hdr_t order (MSB first)
    ("enable_intra_edge_filter", 1), ("dq_ydc", 7), ("dq_udc", 7), ("dq_uac", 7), ("dq_vdc", 7), ("dq_vac", 7),
    ("seg_altq_en", 8), ("seg_altq", 72), ("using_qmatrix", 1), ("qm_level", 96)]


def rec_vals(th):
    return dict(enable_intra_edge_filter=th.enable_intra_edge_filter,
                dq_ydc=th.DeltaQYDc, dq_udc=th.DeltaQUDc, dq_uac=th.DeltaQUAc, dq_vdc=th.DeltaQVDc, dq_vac=th.DeltaQVac if hasattr(th, "DeltaQVac") else th.DeltaQVAc,
                seg_altq_en=sum((th.FeatureEnabled[s][0] & 1) << s for s in range(8)),
                seg_altq=sum((th.FeatureData[s][0] & 0x1FF) << (9 * s) for s in range(8)),
                using_qmatrix=th.using_qmatrix,
                qm_level=sum((th.SegQMLevel[p][s] & 0xF) << ((p * 8 + s) * 4) for p in range(3) for s in range(8)))


rtl_pal = {}
ft_seen = set()


class RecFrame(rm.FrameRecon):
    """FrameRecon that also records each block's palette colours / maps for mismatch diagnostics."""

    ft_log = {}      # (r, c, plane) -> filter type the model used

    def get_filter_type(self, plane):
        ft = super().get_filter_type(plane)
        RecFrame.ft_log[(self.MiRow, self.MiCol, plane)] = ft
        return ft

    def decode_block(self, r, c, subSize):
        super().decode_block(r, c, subSize)
        b = self.blocks[-1]
        b["col_y"] = list(self.palette_colors_y[:self.PaletteSizeY])
        b["col_u"] = list(self.palette_colors_u[:self.PaletteSizeUV])
        b["col_v"] = list(self.palette_colors_v[:self.PaletteSizeUV])
        b["cmap_y"], b["cmap_uv"] = self.ColorMapY, self.ColorMapUV
        b["qidx"] = self.CurrentQIndex



_feed_task = [None]


def start_feed(dut, coro):
    """Start a byte feeder for a tile, cancelling the previous tile's feeder first: a tile may carry trailing
    bytes the decoder never fetches (Argon streams do), which would leave the old feeder blocked and its bytes
    interleaving with the next tile's."""
    if _feed_task[0] is not None and not _feed_task[0].done():
        _feed_task[0].cancel()
    dut.in_valid.value = 0
    _feed_task[0] = cocotb.start_soon(coro)
    return _feed_task[0]

async def run_tile(dut, th, dec, data, tag, stats, debug):
    dut.hdr.value = TT.pack(TT.HDR_FIELDS, TT.hdr_vals(th, dec))
    dut.rh.value = TT.pack(REC_FIELDS, rec_vals(th))
    for i, row in enumerate(M.default_rows(th.base_q_idx)):
        dut.def_we.value = 1
        dut.def_addr.value = i
        dut.def_data.value = row
        await RisingEdge(dut.clk)
    dut.def_we.value = 0
    dut.tile_start.value = 1
    await RisingEdge(dut.clk)
    await Timer(1, "ns")
    dut.tile_start.value = 0
    feeder = start_feed(dut, TT.feed_bytes(dut, data))
    cycles = 0
    idle = 0
    blocks = txb = 0
    if os.environ.get("TS_NOMODEL"):
        # coarse wait: the records are not compared, so poll the held tile_done every 128 cycles
        while True:
            await ClockCycles(dut.clk, 128)
            cycles += 128
            await ReadOnly()
            if int(dut.tile_done_lvl.value):
                break
            assert not int(dut.unsupported.value), f"{tag}: RTL flagged unsupported syntax"
            assert cycles < 400_000_000, f"{tag}: tile did not finish"
        await Timer(1, "ns")
        stats["tiles"] += 1
        stats["cycles"] += cycles
        dut._log.info(f"{tag}: tile done in ~{cycles} cycles")
        return
    while True:
        await RisingEdge(dut.clk)
        await ReadOnly()
        cycles += 1
        idle += 1
        if int(dut.blk_done.value):
            blocks += 1
            idle = 0
            got = TT.unpack(TT.BLK_FIELDS, int(dut.blk_rec.value))
            if got["pal_y"] or got["pal_uv"]:
                rtl_pal[(got["r"], got["c"])] = ([(got["col_y"] >> (12 * k)) & 0xFFF for k in range(got["pal_y"])],
                                                 [(got["col_u"] >> (12 * k)) & 0xFFF for k in range(got["pal_uv"])],
                                                 [(got["col_v"] >> (12 * k)) & 0xFFF for k in range(got["pal_uv"])])
        if int(dut.tx_done.value) and int(dut.u_ts.tx_ack.value):
            txb += 1
            idle = 0
            if debug:
                bb = TT.unpack(TT.BLK_FIELDS, int(dut.u_rc.b.value))
                key = (bb["r"], bb["c"])
                if key not in ft_seen:
                    ft_seen.add(key)
                    rtl_fty, rtl_ftuv = int(dut.u_rc.ft_y.value), int(dut.u_rc.ft_uv.value)
                    m_fty = RecFrame.ft_log.get((bb["r"], bb["c"], 0))
                    m_ftuv = RecFrame.ft_log.get((bb["r"], bb["c"], 1))
                    if (m_fty is not None and m_fty != rtl_fty) or (m_ftuv is not None and m_ftuv != rtl_ftuv):
                        dut._log.info(f"FT MISMATCH block ({bb['r']},{bb['c']}) size {bb['bsize']} ymode {bb['ymode']} uvmode {bb['uvmode']}: "
                                      f"rtl y/uv {rtl_fty}/{rtl_ftuv} model {m_fty}/{m_ftuv}")
        if int(dut.unsupported.value):
            raise AssertionError(f"{tag}: RTL flagged unsupported syntax")
        if idle > 300_000:
            state = (f"ts st={int(dut.u_ts.st.value)} blk st={int(dut.u_ts.u_blk.st.value)} rc rs={int(dut.u_rc.rs.value)} "
                     f"bst={int(dut.u_rc.bst.value)} blk_ready={int(dut.u_rc.blk_ready.value)} tx_done={int(dut.tx_done.value)} "
                     f"ip_busy={int(dut.u_rc.u_ip.busy.value)} itx_busy={int(dut.u_rc.u_itx.busy.value)} cfl_busy={int(dut.u_rc.u_cfl.busy.value)}")
            raise AssertionError(f"{tag}: hang after {blocks} blocks / {txb} tx blocks (model {len(dec.recon_events) + len(dec.pred_events)} events); {state}")
        if int(dut.tile_done.value):
            break
    stats["tiles"] += 1
    stats["blocks"] += blocks
    stats["txblocks"] += txb
    stats["cycles"] += cycles
    if debug:
        dut._log.info(f"{tag}: tile done, {blocks} blocks, {txb} tx blocks, {cycles} cycles")


async def pulse_and_wait(dut, start_sig, done_sig, tag, limit=60_000_000, busy_sig=None):
    """Pulse start, wait for the stage. With busy_sig, poll the busy level every 256 cycles instead of every
    clock (the per-cycle Python await is the bottleneck of frame-level stages)."""
    await Timer(1, "ns")
    start_sig.value = 1
    await RisingEdge(dut.clk)
    await Timer(1, "ns")
    start_sig.value = 0
    cycles = 0
    if busy_sig is not None:
        await ClockCycles(dut.clk, 2)
        cycles = 2
        while True:
            await ReadOnly()
            if not int(busy_sig.value):
                break
            await ClockCycles(dut.clk, 256)
            cycles += 256
            assert cycles < limit, f"{tag}: stage did not finish"
        await Timer(1, "ns")
        return cycles
    while True:
        await RisingEdge(dut.clk)
        await ReadOnly()
        cycles += 1
        if int(done_sig.value):
            break
        assert cycles < limit, f"{tag}: stage did not finish"
    await Timer(1, "ns")
    return cycles


async def finish_frame(dut, hdr, decs, planes, tag, stats):
    """After all tiles: run the in-loop filters up to STAGE in both the model and the RTL. Returns the planes to
    compare and which frame buffer holds the RTL result (0 = deblocked frame, 1 = CdefFrame)."""
    buf = 0
    if STAGE in ("lf", "cdef", "lr"):
        state = xf.FrameState(hdr, decs)
        lfm.LoopFilter(hdr, state, planes).apply()
        dut.lh.value = TT.pack(TL.LF_FIELDS, TL.lf_vals(hdr))
        stats["lf_cycles"] += await pulse_and_wait(dut, dut.lf_start, dut.lf_done, tag)
        deblocked = planes
        if STAGE in ("cdef", "lr") and hdr.enable_cdef and not hdr.CodedLossless and not hdr.allow_intrabc:
            planes = cdm.Cdef(hdr, state, planes).apply()
            dut.ch.value = TT.pack(TC.CDEF_FIELDS, TC.cdef_vals(hdr))
            stats["cdef_cycles"] += await pulse_and_wait(dut, dut.cdef_start, dut.cdef_done, tag)
            buf = 1
        if STAGE == "lr" and any(t != 0 for t in hdr.FrameRestorationType[:hdr.NumPlanes]):
            # CDEF off: CdefFrame == deblocked frame (the RTL reads fb0 for both sources via lr_from_deblocked)
            planes_cdef = planes if buf == 1 else [[row[:] for row in p] for p in planes]
            dut.lr_from_deblocked.value = 0 if buf == 1 else 1
            planes = lrm.LoopRestoration(hdr, state, deblocked, planes_cdef).apply()
            stats["lr_cycles"] += await pulse_and_wait(dut, dut.lr_start, dut.lr_done_o, tag, 120_000_000)
            buf = 2
    return planes, buf


def dump_yuv(path, hdr, get_pixel):
    """Write the visible picture (FrameWidth x FrameHeight, planar, 8-bit or 16-bit LE) using get_pixel(plane, x, y)."""
    import struct
    with open(path, "wb") as fh:
        for plane in range(hdr.NumPlanes):
            sx = hdr.subsampling_x if plane else 0
            sy = hdr.subsampling_y if plane else 0
            W = (hdr.FrameWidth + sx) >> sx
            H = (hdr.FrameHeight + sy) >> sy
            for y in range(H):
                row = [get_pixel(plane, x, y) for x in range(W)]
                fh.write(bytes(row) if hdr.BitDepth == 8 else struct.pack("<%dH" % W, *row))


def ref_frame(path, hdr, fi, offset=None):
    """Frame fi of a dav1d raw-yuv output (planar, visible size, 8-bit bytes or 16-bit LE) as plane arrays.
    offset: byte offset of this frame (frames may differ in size); defaults to fi * this frame's size."""
    import struct
    sizes = []
    for plane in range(hdr.NumPlanes):
        sx = hdr.subsampling_x if plane else 0
        sy = hdr.subsampling_y if plane else 0
        sizes.append(((hdr.FrameWidth + sx) >> sx, (hdr.FrameHeight + sy) >> sy))
    bps = 1 if hdr.BitDepth == 8 else 2
    frame_bytes = sum(w * h for w, h in sizes) * bps
    with open(path, "rb") as fh:
        fh.seek(frame_bytes * fi if offset is None else offset)
        data = fh.read(frame_bytes)
    assert len(data) == frame_bytes, f"{path}: frame {fi} missing (file too short)"
    planes = []
    pos = 0
    for (w, h) in sizes:
        n = w * h
        vals = list(data[pos:pos + n]) if bps == 1 else list(struct.unpack("<%dH" % n, data[pos:pos + n * 2]))
        pos += n * bps
        planes.append([vals[y * w:(y + 1) * w] for y in range(h)])
    return planes, sizes


async def rtl_only(dut, ivfs, stats):
    """Every stream is attempted; failures are collected and reported at the end (one bad stream must not hide
    the others' results in a conformance batch)."""
    failures = []
    for path in ivfs:
        try:
            await rtl_only_stream(dut, path, stats)
        except AssertionError as e:
            failures.append(str(e)[:300])
            stats["failed"] = stats.get("failed", 0) + 1
            dut._log.error(f"FAIL {Path(path).name}: {str(e)[:300]}")
            # the decoder may be mid-frame: reset it before the next stream
            dut.rst.value = 1
            await ClockCycles(dut.clk, 4)
            dut.rst.value = 0
            await RisingEdge(dut.clk)
    dut._log.info(f"streams: {len(ivfs)}, failed: {len(failures)}")
    assert not failures, f"{len(failures)} / {len(ivfs)} streams failed: " + " | ".join(failures)


async def rtl_only_stream(dut, path, stats):
    import obu_parser as op
    import tile_model as tm
    ref_dir = os.environ.get("TS_REF_DIR", str(HERE.parent / "refout"))
    if True:
        d = op.Decoder()
        raw = open(path, "rb").read()
        if path.endswith(".obu"):
            d.feed_annexb(raw)                 # Argon conformance streams (Annex B)
        else:
            d.feed_ivf(raw)
        ref_path = os.path.join(ref_dir, Path(path).stem + ".yuv")
        # Argon layout: <set>/streams/<name>.obu with <set>/md5_ref/<name>.md5 (and md5_no_film_grain); no yuv
        md5_ref = None
        if not os.path.exists(ref_path):
            import hashlib
            setdir = Path(path).resolve().parent.parent
            for sub in ("md5_no_film_grain", "md5_ref"):
                mf = setdir / sub / (Path(path).stem + ".md5")
                if mf.exists():
                    md5_ref = mf.read_text().split()[0]
                    break
            assert md5_ref, f"{path}: no reference yuv ({ref_path}) and no md5 file"
            md5 = hashlib.md5()
        ref_offset = 0                                     # running byte offset into the reference yuv (frame sizes vary)
        for fi, tile_idx in enumerate(d.frames):
            hdr0 = None
            for ti in tile_idx:
                th, data = d.tiles[ti]
                dec = tm.TileDecoder(th, data, None)          # header helper only (get_qindex); nothing is decoded
                dec.recon_events = []
                dec.pred_events = []
                hdr0 = th
                await run_tile(dut, th, dec, data, f"{Path(path).name} frame {fi} tile ({th.MiColStart},{th.MiRowStart})", stats, False)
                await ClockCycles(dut.clk, 4)
            hdr = hdr0
            tag = f"{Path(path).name} frame {fi}"
            # in-loop filters, driven by the frame header alone
            buf = 0
            dut.lh.value = TT.pack(TL.LF_FIELDS, TL.lf_vals(hdr))
            c = await pulse_and_wait(dut, dut.lf_start, dut.lf_done, tag, busy_sig=dut.lf_busy)
            stats["lf_cycles"] += c
            dut._log.info(f"{tag}: deblock ~{c} cycles")
            if hdr.enable_cdef and not hdr.CodedLossless and not hdr.allow_intrabc:
                dut.ch.value = TT.pack(TC.CDEF_FIELDS, TC.cdef_vals(hdr))
                c = await pulse_and_wait(dut, dut.cdef_start, dut.cdef_done, tag, busy_sig=dut.cdef_busy)
                stats["cdef_cycles"] += c
                dut._log.info(f"{tag}: cdef ~{c} cycles")
                buf = 1
            if any(t != 0 for t in hdr.FrameRestorationType[:hdr.NumPlanes]):
                dut.lr_from_deblocked.value = 0 if buf == 1 else 1
                c = await pulse_and_wait(dut, dut.lr_start, dut.lr_done_o, tag, 120_000_000, busy_sig=dut.lr_busy)
                stats["lr_cycles"] += c
                dut._log.info(f"{tag}: restoration ~{c} cycles")
                buf = 2
            dut.h_buf.value = buf
            await Timer(1, "ns")
            if md5_ref is None:
                ref, sizes = ref_frame(ref_path, hdr, fi, ref_offset)
                ref_offset += sum(w * h for w, h in sizes) * (1 if hdr.BitDepth == 8 else 2)
            else:
                ref = None
                sizes = [(((hdr.FrameWidth + (hdr.subsampling_x if p_ else 0)) >> (hdr.subsampling_x if p_ else 0)),
                          ((hdr.FrameHeight + (hdr.subsampling_y if p_ else 0)) >> (hdr.subsampling_y if p_ else 0))) for p_ in range(hdr.NumPlanes)]
            import struct
            for plane in range(hdr.NumPlanes):
                W, H = sizes[plane]
                bad = []
                for y in range(H):
                    row = []
                    for x in range(W):
                        dut.h_plane.value = plane
                        dut.h_x.value = x
                        dut.h_y.value = y
                        await RisingEdge(dut.clk)
                        await ReadOnly()
                        v = int(dut.h_rdata.value)
                        row.append(v)
                        if ref is not None and v != ref[plane][y][x]:
                            bad.append((y, x, v, ref[plane][y][x]))
                        await Timer(1, "ns")
                    if md5_ref is not None:
                        md5.update(bytes(row) if hdr.BitDepth == 8 else struct.pack("<%dH" % W, *row))
                stats["pixels"] += W * H
                assert not bad, f"{tag} plane {plane}: {len(bad)} / {W * H} pixels differ from dav1d (y, x, rtl, dav1d), first {bad[:10]}"
            stats["frames"] += 1
            if md5_ref is None:
                dut._log.info(f"{tag}: identical to dav1d ({hdr.FrameWidth}x{hdr.FrameHeight} bd{hdr.BitDepth}, stages up to {['deblock', 'cdef', 'lr'][buf]})")
            else:
                dut._log.info(f"{tag}: decoded ({hdr.FrameWidth}x{hdr.FrameHeight} bd{hdr.BitDepth}, stages up to {['deblock', 'cdef', 'lr'][buf]}); md5 pending")
        if md5_ref is not None:
            got = md5.hexdigest()
            assert got == md5_ref, f"{Path(path).name}: md5 {got} != reference {md5_ref}"
            stats["md5_ok"] = stats.get("md5_ok", 0) + 1
            dut._log.info(f"{Path(path).name}: all {len(d.frames)} frames, md5 == reference")


async def compare_frame(dut, hdr, planes, tag, stats, decs_events=(), blocks_all=()):
    await Timer(1, "ns")
    dump_dir = os.environ.get("TS_DUMP")
    rtl_planes = []
    for plane in range(hdr.NumPlanes):
        sx = hdr.subsampling_x if plane else 0
        sy = hdr.subsampling_y if plane else 0
        W = (hdr.MiCols * 4) >> sx
        H = (hdr.MiRows * 4) >> sy
        bad = []
        got = [[0] * W for _ in range(H)]
        for y in range(H):
            for x in range(W):
                dut.h_plane.value = plane
                dut.h_x.value = x
                dut.h_y.value = y
                await RisingEdge(dut.clk)
                await ReadOnly()
                v = int(dut.h_rdata.value)
                got[y][x] = v
                if v != planes[plane][y][x]:
                    bad.append((y, x, v, planes[plane][y][x]))
                await Timer(1, "ns")
        rtl_planes.append(got)
        if dump_dir and plane == hdr.NumPlanes - 1:
            os.makedirs(dump_dir, exist_ok=True)
            base = tag.replace(" ", "_").replace("/", "_")
            dump_yuv(os.path.join(dump_dir, f"{base}.rtl.yuv"), hdr, lambda p, x, y: rtl_planes[p][y][x])
            dump_yuv(os.path.join(dump_dir, f"{base}.model.yuv"), hdr, lambda p, x, y: planes[p][y][x])
            with open(os.path.join(dump_dir, f"{base}.txt"), "w") as fh:
                fh.write(f"{hdr.FrameWidth} {hdr.FrameHeight} {hdr.BitDepth} {hdr.subsampling_x} {hdr.subsampling_y} {hdr.NumPlanes}\n")
        stats["pixels"] += W * H
        if bad:
            # locate the first mismatches in the model's per-block events: prediction value vs final value
            for (y, x, v, mv) in bad[:4]:
                for blk in blocks_all:
                    bw = TT.T.Block_Width[blk["size"]] >> sx
                    bh = TT.T.Block_Height[blk["size"]] >> sy
                    bx = (blk["c"] * 4) >> sx
                    by = (blk["r"] * 4) >> sy
                    if bx <= x < bx + bw and by <= y < by + bh:
                        dut._log.info(f"  ({y},{x}) block r={blk['r']} c={blk['c']} size={blk['size']} skip={blk['skip']} ymode={blk['ymode']} uvmode={blk['uvmode']} "
                                      f"pal={blk['pal']} cfl={blk['cfl']} fi={blk['fi']} tx={blk['tx']} seg={blk['seg']} qidx={blk.get('qidx')}")
                        if blk["pal"][plane > 0]:
                            cm = blk["cmap_y"] if plane == 0 else blk["cmap_uv"]
                            cols = blk["col_y"] if plane == 0 else (blk["col_u"] if plane == 1 else blk["col_v"])
                            dut._log.info(f"     model colours {cols}; map row {y - by}: {cm[y - by][:16]}; rtl colours {rtl_pal.get((blk['r'], blk['c']))}")
                for ev in decs_events:
                    kind, pl, x4, y4, w, h, pix = ev
                    if pl == plane and x4 * 4 <= x < x4 * 4 + w and y4 * 4 <= y < y4 * 4 + h:
                        dut._log.info(f"  ({y},{x}) rtl {v} model {mv}: in {kind} event block at ({y4 * 4},{x4 * 4}) {w}x{h}, value {pix[(y - y4 * 4) * w + (x - x4 * 4)]}")
        assert not bad, f"{tag} plane {plane}: {len(bad)} / {W * H} pixel mismatches (y, x, rtl, model), first {bad[:10]}"


@cocotb.test()
async def dec_vs_model(dut):
    seeds_env = os.environ.get("TS_SEEDS", "1,2,3")
    seeds = [int(x) for x in seeds_env.split(",") if x] if "," in seeds_env else list(range(1, int(seeds_env) + 1))
    W = int(os.environ.get("TS_W", "128"))
    H = int(os.environ.get("TS_H", "96"))
    debug = bool(os.environ.get("TD_DEBUG"))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("in_valid", "in_eos", "def_we", "tile_start", "hdr", "rh", "lh", "ch", "lf_start", "cdef_start", "lr_start", "lr_from_deblocked", "h_buf", "h_plane", "h_x", "h_y"):
        getattr(dut, s).value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    stats = dict(frames=0, tiles=0, blocks=0, txblocks=0, pixels=0, cycles=0, lf_cycles=0, cdef_cycles=0, lr_cycles=0, stage=STAGE)
    ivfs = [p for p in os.environ.get("TS_IVF", "").split(",") if p]
    if ivfs and os.environ.get("TS_NOMODEL"):
        # model-free: the RTL runs every stage on its own and the output picture is compared with dav1d's decoded
        # frames (refout/<stream>.yuv, or TS_REF_DIR), so this scales to the conformance sets
        await rtl_only(dut, ivfs, stats)
        dut._log.info(f"OK: {stats}")
        return
    if ivfs:
        # real streams (aomenc / dav1d-verified corpus): every frame, every tile
        import obu_parser as op
        for path in ivfs:
            d = op.Decoder()
            raw = open(path, "rb").read()
            if path.endswith(".obu"):
                d.feed_annexb(raw)
            else:
                d.feed_ivf(raw)
            for fi, tile_idx in enumerate(d.frames):
                frame = {}
                events = []
                blocks_all = []
                hdr0 = None
                decs = []
                for ti in tile_idx:
                    th, data = d.tiles[ti]
                    tag = f"{Path(path).name} frame {fi} tile ({th.MiColStart},{th.MiRowStart})"
                    dec = RecFrame(th, data, frame)
                    dec.decode_tile()
                    decs.append(dec)
                    hdr0 = th
                    events.extend(dec.pred_events)
                    events.extend(dec.recon_events)
                    blocks_all.extend(dec.blocks)
                    await run_tile(dut, th, dec, data, tag, stats, debug)
                    await ClockCycles(dut.clk, 4)
                planes_cmp, buf = await finish_frame(dut, hdr0, decs, frame["planes"], f"{Path(path).name} frame {fi}", stats)
                dut.h_buf.value = buf
                await compare_frame(dut, hdr0, planes_cmp, f"{Path(path).name} frame {fi}", stats, events, blocks_all)
                stats["frames"] += 1
                dut._log.info(f"{Path(path).name} frame {fi}: identical ({len(tile_idx)} tiles, {hdr0.MiCols * 4}x{hdr0.MiRows * 4} MI area, bd{hdr0.BitDepth})")
        dut._log.info(f"OK: {stats}")
        return
    for seed in seeds:
        fmt = os.environ.get("TS_FMT") or random.Random(seed * 7).choice(["420", "444", "mono", "422"])
        d, args = TT.gen(seed, W, H, fmt)
        frame = {}
        hdr0 = None
        events = []
        blocks_all = []
        decs = []
        for ti, (th, data) in enumerate(d.tiles):
            tag = f"seed {seed} {fmt} bd{args['bd']} tile {ti} ({th.MiColStart},{th.MiRowStart})"
            dec = RecFrame(th, data, frame)
            dec.decode_tile()
            decs.append(dec)
            hdr0 = th
            events.extend(dec.pred_events)
            events.extend(dec.recon_events)
            blocks_all.extend(dec.blocks)
            await run_tile(dut, th, dec, data, tag, stats, debug)
            await ClockCycles(dut.clk, 4)
        if debug:
            for blk in blocks_all:
                if blk["pal"] != (0, 0):
                    dut._log.info(f"PAL block r={blk['r']} c={blk['c']} size={blk['size']} model Y {blk['col_y']} U {blk['col_u']} | rtl {rtl_pal.get((blk['r'], blk['c']))}")
        planes_cmp, buf = await finish_frame(dut, hdr0, decs, frame["planes"], f"seed {seed} {fmt} bd{args['bd']}", stats)
        dut.h_buf.value = buf
        await compare_frame(dut, hdr0, planes_cmp, f"seed {seed} {fmt} bd{args['bd']}", stats, events, blocks_all)
        stats["frames"] += 1
        dut._log.info(f"seed {seed} {fmt} bd{args['bd']} {W}x{H}: frame identical ({len(d.tiles)} tiles)")
    dut._log.info(f"OK: {stats}")
