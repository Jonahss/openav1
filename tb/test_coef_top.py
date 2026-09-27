"""coef_rd (+ sym_seq, cdf_store, msac) vs the Python model's coeffs( ).

For each trial ("tile"): random frame header (tb/obu_writer -> tb/obu_parser, as gen_stream does), a
TileDecoder driven by a RecordingDecoder with biased random picks, and a sequence of random transform
blocks decoded by dec.coeffs(). The recorded symbols are arithmetic-encoded into tile bytes and replayed
through the RTL block by block: phase A (all_zero), the intra_tx_type symbol through the block-level
sequencer port when the model read one, phase B; then eob / culLevel / dcCategory / every Quant entry are
compared. CDF adaptation is exercised end to end since all blocks of a tile share one store.

Env: COEF_TRIALS (default 6), COEF_BLOCKS (default 40), COEF_SEED (default 1).
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
import msac_enc as me           # noqa: E402
import tile_model as tm         # noqa: E402


class Rec(me.RecordingDecoder):
    """RecordingDecoder that also remembers each symbol's name and CDF list identity."""

    def __init__(self, pick, dis):
        super().__init__(pick, dis)
        self.meta = []

    def read_symbol(self, cdf, name=""):
        self.meta.append((name, id(cdf)))
        return super().read_symbol(cdf, name)

    def read_bool(self, name=""):
        self.meta.append((name, None))
        return super().read_bool(name)


def make_pick(rng):
    st = {"glen": 0}

    def pick(cdf, N, name):
        if name in ("coeff_base", "coeff_base_eob"):
            return min(N - 1, rng.choice([0, 0, 1, 2, 3, 3]))
        if name == "coeff_br":
            return rng.choice([0, 1, 2, 3, 3, 3])
        if name == "golomb_length_bit":
            st["glen"] += 1
            if st["glen"] >= 6 or rng.random() < 0.5:
                st["glen"] = 0
                return 1
            return 0
        if name == "all_zero":
            return rng.choice([0, 0, 0, 1])
        return rng.randint(0, N - 1)
    return pick


def id_to_addr(dec):
    m = {}
    for name in M.INTRA_TABLES:
        e = M.MAP[name]

        def walk(v, depth, idx):
            if depth == len(e["dims"]):
                m[id(v)] = (M.addr(name, *idx), e["N"])
            else:
                for i, sub in enumerate(v):
                    walk(sub, depth + 1, idx + [i])
        walk(dec.cdf[name], 0, [])
    return m


def gen_tile(rng, nblocks):
    fmt = rng.choice(["420", "444", "mono", "422"])
    bd = rng.choice([8, 10, 12])
    if fmt == "422":
        bd = rng.choice([10, 12])
    args = dict(fmt=fmt, bd=bd, w=64, h=64)
    seq, q = g.make_params(rng, args)
    d, h = g.parse_headers(seq, q)
    th = d.tile_header(h, 0, 0)
    rec = Rec(make_pick(rng), th.disable_cdf_update)
    dec = tm.TileDecoder(th, b"", None)
    dec.dec = rec
    idmap = id_to_addr(dec)
    cap = {}
    orig_az, orig_dc = dec.all_zero_ctx, dec.dc_sign_ctx

    def az(*a):
        cap["az"] = orig_az(*a)
        return cap["az"]

    def dcs(*a):
        cap["dcs"] = orig_dc(*a)
        return cap["dcs"]
    dec.all_zero_ctx, dec.dc_sign_ctx = az, dcs
    blocks = []
    for _ in range(nblocks):
        plane = rng.randrange(th.NumPlanes)
        dec.segment_id = rng.randrange(8) if th.segmentation_enabled else 0
        dec.Lossless = th.LosslessArray[dec.segment_id]
        while True:
            txSz = rng.randrange(19)
            if dec.Lossless:
                txSz = tm.TX_4X4
            if plane > 0 and T.Tx_Size_Sqr_Up[txSz] == tm.TX_64X64:
                continue
            break
        while True:
            dec.MiSize = rng.randrange(22)
            if dec.get_plane_residual_size(dec.MiSize, plane) != tm.BLOCK_INVALID:
                break
        dec.YMode = rng.randrange(13)
        dec.UVMode = rng.randrange(len(T.Mode_To_Txfm))
        dec.use_filter_intra = rng.randint(0, 1)
        dec.filter_intra_mode = rng.randrange(5)
        w4, h4 = T.Tx_Width[txSz] >> 2, T.Tx_Height[txSz] >> 2
        x4 = rng.randrange(0, max(1, th.MiCols - w4 + 1))
        y4 = rng.randrange(0, max(1, th.MiRows - h4 + 1))
        for i in range(th.MiCols):
            dec.AboveLevelContext[plane][i] = rng.choice([0, 0, rng.randrange(64)])
            dec.AboveDcContext[plane][i] = rng.randrange(3)
        for i in range(th.MiRows):
            dec.LeftLevelContext[plane][i] = rng.choice([0, 0, rng.randrange(64)])
            dec.LeftDcContext[plane][i] = rng.randrange(3)
        cap.clear()
        n0 = len(rec.events)
        eob = dec.coeffs(plane, x4 * 4, y4 * 4, txSz)
        segEob = 512 if txSz in (tm.TX_16X64, tm.TX_64X16) else min(1024, T.Tx_Width[txSz] * T.Tx_Height[txSz])
        blocks.append(dict(plane=plane, txSz=txSz, az_ctx=cap["az"], dcs_ctx=cap.get("dcs", 0),
                           tx_type=dec.PlaneTxType, ev0=n0, ev1=len(rec.events), eob=eob,
                           quant=list(dec.Quant[:segEob]),
                           cul=dec.AboveLevelContext[plane][x4], dccat=dec.AboveDcContext[plane][x4]))
    data = me.encode_events(rec.events)
    return th, rec, idmap, blocks, data


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


async def wait_pulse(dut, sig, what, limit=200000):
    for _ in range(limit):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(sig.value) == 1:
            await Timer(1, "ns")
            return
        await Timer(1, "ns")
    raise AssertionError(f"timeout waiting for {what}")


def to_signed(v, bits):
    return v - (1 << bits) if v >> (bits - 1) else v


async def monitor_syms(dut, log):
    """Log every msac request as accepted: (row addr, n, kind, first CDF entries + counter) and its symbol."""
    pend = None
    while True:
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.req_valid.value) and int(dut.req_ready.value):
            cdf = int(dut.req_cdf.value)
            n = int(dut.req_n.value)
            ents = [32768 - ((cdf >> (16 * k)) & 0xFFFF) for k in range(n)]
            pend = (int(dut.u_sq_c.l_addr.value) if not int(dut.sel_t.value) else "tb", n, int(dut.req_kind.value), ents, int(dut.req_cnt.value))
        if int(dut.resp_valid.value) and pend is not None:
            log.append(pend + (int(dut.resp_sym.value),))
            pend = None


def model_syms(rec, idmap, lo, hi):
    out = []
    for (kind, cdf, v, na), (name, cid) in zip(rec.events[lo:hi], rec.meta[lo:hi]):
        if kind == "B":
            out.append((name, "bool", v))
        else:
            out.append((name, idmap.get(cid, ("?", len(cdf) - 1)), cdf[:-2], cdf[-1], v))
    return out


@cocotb.test()
async def coef_vs_model(dut):
    symlog = []
    if os.environ.get("COEF_DEBUG"):
        cocotb.start_soon(monitor_syms(dut, symlog))
    trials = int(os.environ.get("COEF_TRIALS", "6"))
    nblocks = int(os.environ.get("COEF_BLOCKS", "40"))
    rng = random.Random(int(os.environ.get("COEF_SEED", "1")))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for s in ("in_valid", "in_eos", "init", "cdf_update_en", "def_we", "cdf_init", "start_a", "start_b", "tb_go", "q_addr"):
        getattr(dut, s).value = 0
    dut.rst.value = 1
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)
    stats = dict(blocks=0, all_zero=0, txtype=0, coefs=0, syms=0)
    for t in range(trials):
        th, rec, idmap, blocks, data = gen_tile(rng, nblocks)
        stats["syms"] += len(rec.events)
        # defaults for this frame's base_q_idx
        for i, row in enumerate(M.default_rows(th.base_q_idx)):
            dut.def_we.value = 1
            dut.def_addr.value = i
            dut.def_data.value = row
            await RisingEdge(dut.clk)
        dut.def_we.value = 0
        dut.cdf_init.value = 1
        await RisingEdge(dut.clk)
        dut.cdf_init.value = 0
        while True:
            await ReadOnly()
            b = int(dut.cdf_busy.value)
            await RisingEdge(dut.clk)
            if not b:
                break
        await Timer(1, "ns")
        dut.cdf_update_en.value = 0 if th.disable_cdf_update else 1
        dut.init.value = 1
        await RisingEdge(dut.clk)
        await Timer(1, "ns")
        dut.init.value = 0
        cocotb.start_soon(feed_bytes(dut, data))
        for bi, b in enumerate(blocks):
            tag = f"tile {t} block {bi} plane {b['plane']} txSz {b['txSz']} type {b['tx_type']}"
            ev = rec.events[b["ev0"]:b["ev1"]]
            meta = rec.meta[b["ev0"]:b["ev1"]]
            dut.cfg_tx.value = b["txSz"]
            dut.cfg_ptype.value = 1 if b["plane"] > 0 else 0
            dut.az_ctx.value = b["az_ctx"]
            dut.start_a.value = 1
            await RisingEdge(dut.clk)
            await Timer(1, "ns")
            dut.start_a.value = 0
            await wait_pulse(dut, dut.done_a, tag + " done_a")
            az = int(dut.all_zero.value)
            assert az == ev[0][2], f"{tag}: all_zero {az} vs {ev[0][2]}"
            stats["blocks"] += 1
            if az:
                stats["all_zero"] += 1
                continue
            k = 1
            if meta[k][0] == "intra_tx_type":
                addr, N = idmap[meta[k][1]]
                dut.tb_addr.value = addr
                dut.tb_n.value = N - 1
                dut.tb_kind.value = 0
                dut.tb_go.value = 1
                await RisingEdge(dut.clk)
                await Timer(1, "ns")
                dut.tb_go.value = 0
                await wait_pulse(dut, dut.tb_done, tag + " tb_done")
                got = int(dut.tb_sym.value)
                assert got == ev[k][2], f"{tag}: intra_tx_type {got} vs {ev[k][2]}"
                stats["txtype"] += 1
                k += 1
            assert meta[k][0].startswith("eob_pt"), f"{tag}: unexpected symbol order {meta[k][0]}"
            dut.tx_type.value = b["tx_type"]
            dut.dcs_ctx.value = b["dcs_ctx"]
            dut.start_b.value = 1
            await RisingEdge(dut.clk)
            await Timer(1, "ns")
            dut.start_b.value = 0
            await wait_pulse(dut, dut.done_b, tag + " done_b")
            eob = int(dut.eob_o.value)
            cul = int(dut.cul_level.value)
            dcc = int(dut.dc_category.value)
            hdr_msgs = []
            if int(dut.nonconformant.value):
                hdr_msgs.append("nonconformant flag")
            if eob != b["eob"]:
                hdr_msgs.append(f"eob {eob} vs {b['eob']}")
            if cul != b["cul"]:
                hdr_msgs.append(f"culLevel {cul} vs {b['cul']}")
            if dcc != b["dccat"]:
                hdr_msgs.append(f"dcCategory {dcc} vs {b['dccat']}")
            # read Quant
            q = b["quant"]
            got = []
            for p in range(len(q)):
                dut.q_addr.value = p
                await RisingEdge(dut.clk)
                await ReadOnly()
                got.append(to_signed(int(dut.q_data.value), 21))
                await Timer(1, "ns")
            bad = [(p, got[p], q[p]) for p in range(len(q)) if got[p] != q[p]]
            if bad or hdr_msgs:
                names = [m[0] for m in meta[k:k + 40]]
                if symlog:
                    ms = model_syms(rec, idmap, b["ev0"], b["ev1"])
                    dut._log.info("MODEL symbols (name, (addr, N) | bool, value):")
                    for i, x in enumerate(ms[:60]):
                        dut._log.info(f"  m{i}: {x}")
                    dut._log.info("RTL symbols (addr, n=N-1, kind, sym):")
                    for i, x in enumerate(symlog[-len(ms) - 4:]):
                        dut._log.info(f"  r{i}: {x}")
                raise AssertionError(f"{tag}: {'; '.join(hdr_msgs)}; {len(bad)} Quant mismatches (pos, rtl, model), first {bad[:8]}; "
                                     f"model nonzero {[(p, v) for p, v in enumerate(q) if v][:12]}; symbols {names}")
            stats["coefs"] += sum(1 for v in q if v)
        # let the byte feeder finish (eos)
        await ClockCycles(dut.clk, 4)
    dut._log.info(f"OK: {stats}")
