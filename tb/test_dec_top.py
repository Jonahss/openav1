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
import recon_model as rm                # noqa: E402
import test_tile_syntax as TT           # noqa: E402

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
    cocotb.start_soon(TT.feed_bytes(dut, data))
    cycles = 0
    idle = 0
    blocks = txb = 0
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


async def compare_frame(dut, hdr, planes, tag, stats, decs_events=(), blocks_all=()):
    await Timer(1, "ns")
    for plane in range(hdr.NumPlanes):
        sx = hdr.subsampling_x if plane else 0
        sy = hdr.subsampling_y if plane else 0
        W = (hdr.MiCols * 4) >> sx
        H = (hdr.MiRows * 4) >> sy
        bad = []
        for y in range(H):
            for x in range(W):
                dut.h_plane.value = plane
                dut.h_x.value = x
                dut.h_y.value = y
                await RisingEdge(dut.clk)
                await ReadOnly()
                v = int(dut.h_rdata.value)
                if v != planes[plane][y][x]:
                    bad.append((y, x, v, planes[plane][y][x]))
                await Timer(1, "ns")
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
    for s in ("in_valid", "in_eos", "def_we", "tile_start", "hdr", "rh", "h_plane", "h_x", "h_y"):
        getattr(dut, s).value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    stats = dict(frames=0, tiles=0, blocks=0, txblocks=0, pixels=0, cycles=0)
    ivfs = [p for p in os.environ.get("TS_IVF", "").split(",") if p]
    if ivfs:
        # real streams (aomenc / dav1d-verified corpus): every frame, every tile
        import obu_parser as op
        for path in ivfs:
            d = op.Decoder()
            d.feed_ivf(open(path, "rb").read())
            for fi, tile_idx in enumerate(d.frames):
                frame = {}
                events = []
                blocks_all = []
                hdr0 = None
                for ti in tile_idx:
                    th, data = d.tiles[ti]
                    tag = f"{Path(path).name} frame {fi} tile ({th.MiColStart},{th.MiRowStart})"
                    dec = RecFrame(th, data, frame)
                    dec.decode_tile()
                    hdr0 = th
                    events.extend(dec.pred_events)
                    events.extend(dec.recon_events)
                    blocks_all.extend(dec.blocks)
                    await run_tile(dut, th, dec, data, tag, stats, debug)
                    await ClockCycles(dut.clk, 4)
                await compare_frame(dut, hdr0, frame["planes"], f"{Path(path).name} frame {fi}", stats, events, blocks_all)
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
        for ti, (th, data) in enumerate(d.tiles):
            tag = f"seed {seed} {fmt} bd{args['bd']} tile {ti} ({th.MiColStart},{th.MiRowStart})"
            dec = RecFrame(th, data, frame)
            dec.decode_tile()
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
        await compare_frame(dut, hdr0, frame["planes"], f"seed {seed} {fmt} bd{args['bd']}", stats, events, blocks_all)
        stats["frames"] += 1
        dut._log.info(f"seed {seed} {fmt} bd{args['bd']} {W}x{H}: frame identical ({len(d.tiles)} tiles)")
    dut._log.info(f"OK: {stats}")
