"""Fuzz the CDEF block parameters (rtl/cdef_str.sv, combinational) against cdef_model.Cdef.block: for random frame
strengths, damping, bit depth, subsampling, cdef_idx, luma direction and variance, the model's block() is run with
its direction search and skip test replaced by the drawn values and its filter() recording what it is called with per
plane; the module's direction, strengths, constrain() shifts and tap-weight bit must match for luma and chroma.
Env: CS_ITERS (default 20000), CS_SEED."""
import os
import random
import types
import cocotb
from cocotb.triggers import Timer

import cdef_model as cdm


class _Rec(cdm.Cdef):
    def skips(self, r, c):
        return 0

    def direction(self, r, c):
        return self.dv

    def filter(self, plane, r, c, priStr, secStr, damping, dr):
        self.calls[plane] = (priStr, secStr, damping, dr)


def flog2(v):
    return v.bit_length() - 1


@cocotb.test()
async def cdef_str_vs_model(dut):
    iters = int(os.environ.get("CS_ITERS", "20000"))
    rng = random.Random(int(os.environ.get("CS_SEED", "1")))
    stats = dict(checks=0, var0=0, pri0=0, dr_forced0=0, sec4=0)
    for it in range(iters):
        bd = rng.choice([8, 10, 12]); cs = bd - 8
        ssx, ssy = rng.choice([(1, 1), (1, 1), (1, 0), (0, 0)])
        ystr = [rng.choice([0, 0, rng.randrange(64)]) if rng.random() < 0.3 else rng.randrange(64) for _ in range(8)]
        uvstr = [rng.choice([0, rng.randrange(64)]) for _ in range(8)]
        damp = rng.randint(3, 6); idx = rng.randrange(8); yd = rng.randrange(8)
        var = rng.choice([0, 0, rng.randrange(64), rng.randrange(1 << 12), rng.randrange(1 << 20), rng.randrange(1 << 31)])
        hdr = types.SimpleNamespace(BitDepth=bd, subsampling_x=ssx, subsampling_y=ssy, NumPlanes=3, cdef_damping=damp,
                                    cdef_y_strengths=ystr, cdef_uv_strengths=uvstr)
        m = _Rec(hdr, None, None); m.dv = (yd, var); m.calls = {}
        m.block(0, 0, idx)
        dut.ch.value = (damp << 98) | (sum(v << (6 * i) for i, v in enumerate(ystr)) << 48) | sum(v << (6 * i) for i, v in enumerate(uvstr))
        dut.cs.value = cs; dut.ssx.value = ssx; dut.ssy.value = ssy; dut.idx.value = idx; dut.ydir.value = yd; dut.var_i.value = var
        for luma, plane in ((1, 0), (0, 1)):
            dut.luma.value = luma
            await Timer(1, "ns")
            pri, sec, damping, dr = m.calls[plane]
            exp = (dr, pri, sec, (max(0, damping - flog2(pri)) if pri else 0), (max(0, damping - flog2(sec)) if sec else 0), (pri >> cs) & 1)
            got = (int(dut.dr.value), int(dut.pri.value), int(dut.sec.value), int(dut.adjp.value), int(dut.adjs.value), int(dut.priodd.value))
            # with a strength of 0 the shift is never used (constrain() returns 0): compare it only where it matters
            if pri == 0: got = got[:3] + (0,) + got[4:]
            if sec == 0: got = got[:4] + (0,) + got[5:]
            assert got == exp, (f"iter {it} plane {plane} bd{bd} ss {ssx}{ssy} idx {idx} ydir {yd} var {var} y_str {ystr[idx]} uv_str {uvstr[idx]} damping {damp}: "
                                f"rtl (dr, pri, sec, adjp, adjs, priodd) {got}, model {exp}")
            stats["checks"] += 1; stats["pri0"] += pri == 0; stats["sec4"] += (sec >> cs) == 4
            if luma:
                stats["var0"] += var == 0; stats["dr_forced0"] += (ystr[idx] >> 2) == 0
    assert stats["var0"] > 500 and stats["pri0"] > 500 and stats["dr_forced0"] > 200 and stats["sec4"] > 500, stats
    dut._log.info(f"OK: {stats}")
