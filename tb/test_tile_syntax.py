"""tile_syntax (the whole intra tile syntax decoder in RTL) vs tb/tile_model.py.

Streams come from tools/gen_stream.py (random syntax, no palette / no loop restoration yet), are parsed by
tb/obu_parser.py, decoded by the model (recording every block and transform block), and replayed through
the RTL tile by tile. Compared: every block record (position, size, skip, segment, modes, angles, CfL
alphas, filter-intra, tx size, CurrentQIndex, DeltaLF) and every transform block (plane, position, tx
size, tx type, eob, all Quant coefficients).

Env: TS_SEEDS (comma list or count, default "1,2,3"), TS_W/TS_H (default 128x96), TS_FMT (default random).
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
import av1_tables as T          # noqa: E402
import cdf_map as M             # noqa: E402
import gen_stream as g          # noqa: E402
import obu_parser as op         # noqa: E402
import tile_model as tm         # noqa: E402

ACK_DELAY = int(os.environ.get("TS_ACK_DELAY", "0"))
HDR_FIELDS = [  # (name, width) in syn_pkg::hdr_t order (MSB first)
    ("mi_rows", 11), ("mi_cols", 11), ("mi_row_start", 11), ("mi_row_end", 11), ("mi_col_start", 11), ("mi_col_end", 11),
    ("ssx", 1), ("ssy", 1), ("mono", 1), ("sb128", 1), ("bit_depth", 4), ("seg_enabled", 1), ("seg_preskip", 1),
    ("last_active_segid", 3), ("seg_skip_en", 8), ("lossless", 8), ("seg_qidx", 64), ("base_q_idx", 8), ("tx_mode", 2),
    ("reduced_tx_set", 1), ("allow_sct", 1), ("enable_filter_intra", 1), ("enable_cdef", 1), ("cdef_bits", 2),
    ("coded_lossless", 1), ("delta_q_present", 1), ("delta_q_res", 2), ("delta_lf_present", 1), ("delta_lf_res", 2),
    ("delta_lf_multi", 1), ("disable_cdf_update", 1), ("lr_type", 6), ("lr_size", 6), ("frame_height", 13), ("upscaled_width", 13)]
BLK_FIELDS = [
    ("r", 11), ("c", 11), ("bsize", 5), ("skip", 1), ("seg", 3), ("lossless", 1), ("has_chroma", 1), ("ymode", 4), ("uvmode", 4),
    ("angle_y", 3), ("angle_uv", 3), ("cfl_u", 6), ("cfl_v", 6), ("use_fi", 1), ("fi_mode", 3), ("txsz", 5), ("qidx", 8),
    ("delta_lf", 28), ("cdef_valid", 1), ("cdef_idx", 3), ("cdef_units", 4),
    ("pal_y", 4), ("pal_uv", 4), ("col_y", 96), ("col_u", 96), ("col_v", 96)]
TX_FIELDS = [("plane", 2), ("x", 13), ("y", 13), ("txsz", 5), ("txtype", 4), ("eob", 11), ("skip", 1), ("lossless", 1)]
LR_FIELDS = [("plane", 2), ("unit_row", 8), ("unit_col", 8), ("lr_type", 2), ("wiener", 42), ("sgr_set", 4), ("xqd", 16)]


def pack(fields, vals):
    v = 0
    for name, w in fields:
        x = vals[name] & ((1 << w) - 1)
        v = (v << w) | x
    return v


def unpack(fields, v):
    out = {}
    total = sum(w for _, w in fields)
    pos = total
    for name, w in fields:
        pos -= w
        out[name] = (v >> pos) & ((1 << w) - 1)
    return out


def to_signed(v, bits):
    return v - (1 << bits) if v >> (bits - 1) else v


class RecDecoder(tm.TileDecoder):
    """Records what the RTL emits: per block and per transform block."""

    def __init__(self, hdr, data):
        super().__init__(hdr, data, None)
        self.tx_recs = []
        self.lr_recs = []
        self.symlog = []
        self.idmap = {}
        for name in M.INTRA_TABLES:
            e = M.MAP[name]

            def walk(v, depth, idx):
                if depth == len(e["dims"]):
                    self.idmap[id(v)] = (M.addr(name, *idx), e["N"])
                else:
                    for i, sub in enumerate(v):
                        walk(sub, depth + 1, idx + [i])
            walk(self.cdf[name], 0, [])

    def sym(self, cdf, name):
        v = super().sym(cdf, name)
        self.symlog.append((name, self.idmap.get(id(cdf), ("?", len(cdf) - 1)), v))
        return v

    def L(self, n, name):
        v = super().L(n, name)
        self.symlog.append((name, "L%d" % n, v))
        return v

    def NS(self, n, name):
        # read_ns through L so the literal bits land in the symbol log (same bit sequence as the spec)
        w = n.bit_length()
        m = (1 << w) - n
        v = self.L(w - 1, name)
        if v < m:
            return v
        return (v << 1) - m + self.L(1, name)

    def predict_block(self, plane, startX, startY, txSz, x, y, subX, subY, sbMiRow, sbMiCol, stepX, stepY):
        self.tx_recs.append(dict(plane=plane, x=startX, y=startY, txsz=txSz, skip=self.skip, txtype=0, eob=0, quant=[],
                                 lossless=self.Lossless))

    def coeffs(self, plane, startX, startY, txSz):
        eob = super().coeffs(plane, startX, startY, txSz)
        segEob = 512 if txSz in (tm.TX_16X64, tm.TX_64X16) else min(1024, T.Tx_Width[txSz] * T.Tx_Height[txSz])
        rec = self.tx_recs[-1]
        rec["txtype"] = self.PlaneTxType if eob > 0 else 0
        rec["eob"] = eob
        rec["quant"] = list(self.Quant[:segEob]) if eob > 0 else [0] * segEob
        return eob

    def read_lr_unit(self, plane, unitRow, unitCol):
        super().read_lr_unit(plane, unitRow, unitCol)
        key = (plane, unitRow, unitCol)
        t = self.LrType[key]
        self.lr_recs.append(dict(plane=plane, unit_row=unitRow, unit_col=unitCol, lr_type=t,
                                 wiener=self.LrWiener.get(key, [[0, 0, 0], [0, 0, 0]]) if t == tm.RESTORE_WIENER else [[0, 0, 0], [0, 0, 0]],
                                 sgr_set=self.LrSgrSet.get(key, 0) if t == tm.RESTORE_SGRPROJ else 0,
                                 xqd=self.LrSgrXqd.get(key, [0, 0]) if t == tm.RESTORE_SGRPROJ else [0, 0]))

    def decode_block(self, r, c, subSize):
        super().decode_block(r, c, subSize)
        self.blocks[-1]["qidx"] = self.CurrentQIndex
        self.blocks[-1]["dlf"] = list(self.DeltaLF)
        self.blocks[-1]["lossless"] = self.Lossless
        self.blocks[-1]["has_chroma"] = self.HasChroma
        self.blocks[-1]["fim"] = self.filter_intra_mode
        b = self.blocks[-1]
        b["col_y"] = list(self.palette_colors_y[:self.PaletteSizeY])
        b["col_u"] = list(self.palette_colors_u[:self.PaletteSizeUV])
        b["col_v"] = list(self.palette_colors_v[:self.PaletteSizeUV])
        b["cmap_y"], b["cmap_uv"] = self.ColorMapY, self.ColorMapUV
        h = self.h
        b["os"] = (min(T.Block_Width[subSize], (h.MiCols - c) * 4), min(T.Block_Height[subSize], (h.MiRows - r) * 4))


def hdr_vals(th, dec):
    v = dict(mi_rows=th.MiRows, mi_cols=th.MiCols, mi_row_start=th.MiRowStart, mi_row_end=th.MiRowEnd,
             mi_col_start=th.MiColStart, mi_col_end=th.MiColEnd, ssx=th.subsampling_x, ssy=th.subsampling_y,
             mono=1 if th.NumPlanes == 1 else 0, sb128=th.use_128x128_superblock, bit_depth=th.BitDepth,
             seg_enabled=th.segmentation_enabled, seg_preskip=th.SegIdPreSkip, last_active_segid=th.LastActiveSegId,
             seg_skip_en=sum((th.FeatureEnabled[s][6] & 1) << s for s in range(8)),
             lossless=sum((th.LosslessArray[s] & 1) << s for s in range(8)),
             seg_qidx=sum((dec.get_qindex(1, s) & 0xFF) << (8 * s) for s in range(8)),
             base_q_idx=th.base_q_idx, tx_mode=th.TxMode, reduced_tx_set=th.reduced_tx_set,
             allow_sct=th.allow_screen_content_tools, enable_filter_intra=th.enable_filter_intra, enable_cdef=th.enable_cdef,
             cdef_bits=th.cdef_bits, coded_lossless=th.CodedLossless, delta_q_present=th.delta_q_present,
             delta_q_res=th.delta_q_res, delta_lf_present=th.delta_lf_present, delta_lf_res=th.delta_lf_res,
             delta_lf_multi=th.delta_lf_multi, disable_cdf_update=th.disable_cdf_update,
             lr_type=sum(th.FrameRestorationType[p] << (2 * p) for p in range(3)),
             lr_size=sum((th.LoopRestorationSize[p].bit_length() - 1 - 6) << (2 * p) for p in range(3)),
             frame_height=th.FrameHeight, upscaled_width=th.UpscaledWidth)
    return v


async def monitor_syms(dut, log):
    """Every accepted msac request: (row addr of the active sequencer, n, kind) and its symbol."""
    # requests may be accepted in the same cycle the previous response arrives: keep a FIFO, respond first
    pend = []
    while True:
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.resp_valid.value) and pend:
            log.append(pend.pop(0) + (int(dut.resp_sym.value),))
        if int(dut.req_valid.value) and int(dut.req_ready.value):
            k = int(dut.sel_k.value)
            addr = int(dut.u_sq_k.l_addr.value) if k else int(dut.u_sq_c.l_addr.value)
            pend.append(("k" if k else "c", addr, int(dut.req_n.value), int(dut.req_kind.value)))


async def monitor_nb(dut, log):
    """Every answered neighbour query: position/size and what blk_ctx returned."""
    while True:
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.nb_valid.value):
            c = dut.u_ctx
            log.append(dict(r=int(c.q_r.value), c=int(c.q_c.value), bs=int(c.q_bs.value), au=int(dut.avail_u.value), al=int(dut.avail_l.value),
                            a_ymode=int(dut.a_ymode.value), l_ymode=int(dut.l_ymode.value), a_skip=int(dut.a_skip.value), l_skip=int(dut.l_skip.value),
                            a_misize=int(dut.a_misize.value), l_misize=int(dut.l_misize.value), a_txsz=int(dut.a_txsz.value), l_txsz=int(dut.l_txsz.value),
                            seg=(int(dut.seg_ul.value), int(dut.seg_u.value), int(dut.seg_l.value)),
                            lword=hex(int(c.l_word.value)), aword=hex(int(c.a_word.value))))


async def feed_bytes(dut, data):
    dut.in_eos.value = 0
    for b in data:
        dut.in_data.value = b
        dut.in_valid.value = 1
        while True:
            await ReadOnly()
            ok = int(dut.in_ready.value) == 1
            await RisingEdge(dut.clk)
            if ok:
                break
        await Timer(1, "ns")
    dut.in_valid.value = 0
    dut.in_eos.value = 1


def gen(seed, w, h, fmt):
    rng = random.Random(seed)
    screen_env = os.environ.get("TS_SCREEN", "rand")      # 1 / 0 / rand
    args = dict(fmt=fmt, bd=rng.choice([8, 10, 12]), w=w, h=h, sb128=rng.random() < 0.3, tiles=rng.random() < 0.3,
                screen=(screen_env == "1") or (screen_env == "rand" and rng.random() < 0.5),
                lossless=rng.random() < 0.15, nolr=bool(os.environ.get("TS_NOLR")), frames=1)
    if fmt == "422":
        args["bd"] = rng.choice([10, 12])
    data, info = g.generate(seed, args)
    d = op.Decoder()
    d.feed_ivf(data)
    return d, args


@cocotb.test()
async def tile_vs_model(dut):
    seeds_env = os.environ.get("TS_SEEDS", "1,2,3")
    seeds = [int(x) for x in seeds_env.split(",") if x] if "," in seeds_env else list(range(1, int(seeds_env) + 1))
    W = int(os.environ.get("TS_W", "128"))
    H = int(os.environ.get("TS_H", "96"))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("in_valid", "in_eos", "def_we", "tile_start", "tx_ack", "q_addr", "hdr", "blk_ack", "pm_plane", "pm_x", "pm_y"):
        getattr(dut, s).value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    stats = dict(tiles=0, blocks=0, txblocks=0, coefs=0, lrunits=0, palblocks=0, palpix=0)
    symlog = []
    debug = bool(os.environ.get("TS_DEBUG"))
    nblog = []
    if debug:
        cocotb.start_soon(monitor_syms(dut, symlog))
        cocotb.start_soon(monitor_nb(dut, nblog))

    def dump(dec):
        if not debug:
            return
        # normalise both logs to (kind, addr, n, value) and show the first divergence with context
        mnorm = []
        for name, where, v in dec.symlog:
            if isinstance(where, str) and where.startswith("L"):
                n = int(where[1:])
                for i in range(n - 1, -1, -1):
                    mnorm.append(("B", 0, 1, (v >> i) & 1, name))
            elif where[0] == "?":
                mnorm.append(("F", 0, 1, v, name))
            else:
                mnorm.append(("S", where[0], where[1] - 1, v, name))
        rnorm = []
        for master, addr, n, kind, sym in symlog:
            rnorm.append(("B" if kind == 2 else "F" if kind == 1 else "S", addr if kind == 0 else 0, n if kind == 0 else 1, sym, master))
        first = None
        for i in range(min(len(mnorm), len(rnorm))):
            if mnorm[i][:4] != rnorm[i][:4]:
                first = i
                break
        if first is None:
            first = min(len(mnorm), len(rnorm))
        dut._log.info(f"first divergence at symbol {first} (model {len(mnorm)} symbols, rtl {len(rnorm)})")
        for x in nblog[-3:]:
            dut._log.info(f"  NB {x}")
            r, c = x["r"], x["c"]
            S = dec.SegmentIds
            nb = dict(ul=S[r - 1][c - 1] if r > 0 and c > 0 else None, u=S[r - 1][c] if r > 0 else None, l=S[r][c - 1] if c > 0 else None)
            dut._log.info(f"     model SegmentIds at ({r},{c}): {nb}; MiSizes above/left: "
                          f"{dec.MiSizes[r - 1][c] if r > 0 else None}/{dec.MiSizes[r][c - 1] if c > 0 else None}")
        dut._log.info(f"  model blocks so far: {[(b['r'], b['c'], b['size'], b['seg'], b['skip']) for b in dec.blocks[:24]]}")
        for b in dec.blocks[:8]:
            dut._log.info(f"  MODEL BLK r={b['r']} c={b['c']} size={b['size']} skip={b['skip']} ymode={b['ymode']} tx={b['tx']}")
        for i in range(max(0, first - 20), min(first + 8, max(len(mnorm), len(rnorm)))):
            m = mnorm[i] if i < len(mnorm) else None
            r = rnorm[i] if i < len(rnorm) else None
            dut._log.info(f"  {i:5d} {'!!' if (m is None or r is None or m[:4] != r[:4]) else '  '} model {m}  rtl {r}")
    for seed in seeds:
        fmt = os.environ.get("TS_FMT") or random.Random(seed * 7).choice(["420", "444", "mono", "422"])
        d, args = gen(seed, W, H, fmt)
        for ti, (th, data) in enumerate(d.tiles):
            tag0 = f"seed {seed} {fmt} bd{args['bd']} tile {ti} ({th.MiColStart},{th.MiRowStart})"
            dec = RecDecoder(th, data)
            dec.decode_tile()
            hv = hdr_vals(th, dec)
            dut.hdr.value = pack(HDR_FIELDS, hv)
            # defaults for base_q_idx
            for i, row in enumerate(M.default_rows(th.base_q_idx)):
                dut.def_we.value = 1
                dut.def_addr.value = i
                dut.def_data.value = row
                await RisingEdge(dut.clk)
            dut.def_we.value = 0
            symlog.clear()
            nblog.clear()
            dut.tile_start.value = 1
            await RisingEdge(dut.clk)
            await Timer(1, "ns")
            dut.tile_start.value = 0
            cocotb.start_soon(feed_bytes(dut, data))
            symlog.clear()
            try:
                await collect(dut, dec, tag0, stats)
            except AssertionError:
                dump(dec)
                raise
            await ClockCycles(dut.clk, 4)
    dut._log.info(f"OK: {stats}")


async def collect(dut, dec, tag0, stats):
            blk_i = 0
            tx_i = 0
            lr_i = 0
            cycles = 0
            idle = 0
            while True:
                await RisingEdge(dut.clk)
                await ReadOnly()
                cycles += 1
                idle += 1
                if idle > 100_000:
                    state = (f"tile st={int(dut.st.value)} blk st={int(dut.u_blk.st.value)} coef st={int(dut.u_coef.st.value)} "
                             f"ctx q/w/t/r={int(dut.u_ctx.qst.value)}/{int(dut.u_ctx.wst.value)}/{int(dut.u_ctx.tst.value)}/{int(dut.u_ctx.rstt.value)} "
                             f"msac st={int(dut.u_msac.state.value)} wcnt={int(dut.u_msac.wcnt.value)} eos={int(dut.u_msac.eos.value)} "
                             f"k_busy={int(dut.k_busy.value)} c_busy={int(dut.c_busy.value)} sel_k={int(dut.sel_k.value)} tx_done={int(dut.tx_done.value)} "
                             f"sp={int(dut.sp.value)} blk_active={int(dut.blk_active.value)}")
                    raise AssertionError(f"{tag0}: hang after {blk_i} blocks / {tx_i} tx blocks (model: {len(dec.blocks)} / {len(dec.tx_recs)}); {state}")
                if int(dut.unsupported.value):
                    raise AssertionError(f"{tag0}: RTL flagged unsupported syntax")
                done = int(dut.tile_done.value)
                if int(dut.pal_hold.value):
                    # palette block held: read the onscreen colour index map(s) through pm_*, compare, release
                    idle = 0
                    got = unpack(BLK_FIELDS, int(dut.blk_rec.value))
                    assert blk_i < len(dec.blocks), f"{tag0}: RTL holds a palette block beyond the model's last block"
                    m = dec.blocks[blk_i]
                    await Timer(1, "ns")
                    for plane, size_key, cmap_key in ((0, "pal_y", "cmap_y"), (1, "pal_uv", "cmap_uv")):
                        if got[size_key] == 0:
                            continue
                        cm = m[cmap_key]
                        assert cm is not None, f"{tag0} block {blk_i}: RTL has a {cmap_key} but the model has none"
                        osw, osh = m["os"]
                        if plane:
                            osw >>= dec.h.subsampling_x
                            osh >>= dec.h.subsampling_y
                            if (T.Block_Width[m["size"]] >> dec.h.subsampling_x) < 4:
                                osw += 2
                            if (T.Block_Height[m["size"]] >> dec.h.subsampling_y) < 4:
                                osh += 2
                        bad = []
                        for y in range(osh):
                            for x in range(osw):
                                dut.pm_plane.value = plane
                                dut.pm_x.value = x
                                dut.pm_y.value = y
                                await RisingEdge(dut.clk)
                                await ReadOnly()
                                v = int(dut.pm_idx.value)
                                if v != cm[y][x]:
                                    bad.append((y, x, v, cm[y][x]))
                                await Timer(1, "ns")
                        assert not bad, f"{tag0} block {blk_i} at ({m['r']},{m['c']}) plane {plane}: {len(bad)} colour-map mismatches (y, x, rtl, model), first {bad[:8]}"
                        stats["palpix"] += osw * osh
                    stats["palblocks"] += 1
                    dut.blk_ack.value = 1
                    await RisingEdge(dut.clk)
                    await Timer(1, "ns")
                    dut.blk_ack.value = 0
                    continue
                if int(dut.blk_done.value):
                    idle = 0
                    got = unpack(BLK_FIELDS, int(dut.blk_rec.value))
                    assert blk_i < len(dec.blocks), f"{tag0}: RTL produced extra block {got}"
                    m = dec.blocks[blk_i]
                    exp = dict(r=m["r"], c=m["c"], bsize=m["size"], skip=m["skip"], seg=m["seg"], lossless=m["lossless"],
                               has_chroma=m["has_chroma"], ymode=m["ymode"], uvmode=m["uvmode"], angle_y=m["angle"][0],
                               angle_uv=m["angle"][1], cfl_u=m["cfl"][0], cfl_v=m["cfl"][1], use_fi=m["fi"], fi_mode=m["fim"] if m["fi"] else 0,
                               txsz=m["tx"], qidx=m["qidx"], dlf=m["dlf"],
                               pal_y=m["pal"][0], pal_uv=m["pal"][1], col_y=m["col_y"], col_u=m["col_u"], col_v=m["col_v"])

                    def cols(v, n):
                        return [(v >> (12 * k)) & 0xFFF for k in range(n)]
                    gsigned = dict(got, angle_y=to_signed(got["angle_y"], 3), angle_uv=to_signed(got["angle_uv"], 3),
                                   cfl_u=to_signed(got["cfl_u"], 6), cfl_v=to_signed(got["cfl_v"], 6),
                                   dlf=[to_signed((got["delta_lf"] >> (7 * i)) & 0x7F, 7) for i in range(4)],
                                   col_y=cols(got["col_y"], m["pal"][0]), col_u=cols(got["col_u"], m["pal"][1]), col_v=cols(got["col_v"], m["pal"][1]))
                    bad = {k: (gsigned[k], exp[k]) for k in exp if gsigned[k] != exp[k]}
                    assert not bad, f"{tag0} block {blk_i} at ({m['r']},{m['c']}) size {m['size']}: mismatches (rtl, model) {bad}"
                    blk_i += 1
                    stats["blocks"] += 1
                if int(dut.lr_done.value):
                    idle = 0
                    got = unpack(LR_FIELDS, int(dut.lr_rec.value))
                    assert lr_i < len(dec.lr_recs), f"{tag0}: RTL produced extra LR unit {got}"
                    m = dec.lr_recs[lr_i]
                    gw = [[to_signed((got["wiener"] >> (7 * (3 * p + j))) & 0x7F, 7) for j in range(3)] for p in range(2)]
                    gx = [to_signed((got["xqd"] >> (8 * i)) & 0xFF, 8) for i in range(2)]
                    g = dict(plane=got["plane"], unit_row=got["unit_row"], unit_col=got["unit_col"], lr_type=got["lr_type"], wiener=gw, sgr_set=got["sgr_set"], xqd=gx)
                    bad = {k: (g[k], m[k]) for k in m if g[k] != m[k]}
                    assert not bad, f"{tag0} LR unit {lr_i}: mismatches (rtl, model) {bad}"
                    lr_i += 1
                    stats["lrunits"] += 1
                if int(dut.tx_done.value):
                    idle = 0
                    got = unpack(TX_FIELDS, int(dut.tx_rec.value))
                    assert tx_i < len(dec.tx_recs), f"{tag0}: RTL produced extra tx block {got}"
                    m = dec.tx_recs[tx_i]
                    exp = dict(plane=m["plane"], x=m["x"], y=m["y"], txsz=m["txsz"], txtype=m["txtype"], eob=m["eob"], skip=m["skip"], lossless=m["lossless"])
                    bad = {k: (got[k], exp[k]) for k in exp if got[k] != exp[k]}
                    assert not bad, f"{tag0} tx block {tx_i} (block {blk_i - 1}): mismatches (rtl, model) {bad}"
                    if not m["skip"] and m["eob"] > 0:
                        q = m["quant"]
                        await Timer(1, "ns")
                        rtl = []
                        for p in range(len(q)):
                            dut.q_addr.value = p
                            await RisingEdge(dut.clk)
                            await ReadOnly()
                            rtl.append(to_signed(int(dut.q_data.value), 21))
                            await Timer(1, "ns")
                        badq = [(p, rtl[p], q[p]) for p in range(len(q)) if rtl[p] != q[p]]
                        assert not badq, f"{tag0} tx block {tx_i}: {len(badq)} Quant mismatches, first {badq[:6]}"
                        stats["coefs"] += sum(1 for v in q if v)
                    else:
                        await Timer(1, "ns")
                    if ACK_DELAY:
                        await ClockCycles(dut.clk, ACK_DELAY)
                        await Timer(1, "ns")
                    dut.tx_ack.value = 1
                    await RisingEdge(dut.clk)
                    await Timer(1, "ns")
                    dut.tx_ack.value = 0
                    tx_i += 1
                    stats["txblocks"] += 1
                if done:
                    break
            assert blk_i == len(dec.blocks), f"{tag0}: RTL emitted {blk_i} blocks, model {len(dec.blocks)}"
            assert tx_i == len(dec.tx_recs), f"{tag0}: RTL emitted {tx_i} tx blocks, model {len(dec.tx_recs)}"
            assert lr_i == len(dec.lr_recs), f"{tag0}: RTL emitted {lr_i} LR units, model {len(dec.lr_recs)}"
            stats["tiles"] += 1
            dut._log.info(f"{tag0}: OK {len(dec.blocks)} blocks, {len(dec.tx_recs)} tx blocks, {len(dec.lr_recs)} LR units, {cycles} cycles")
