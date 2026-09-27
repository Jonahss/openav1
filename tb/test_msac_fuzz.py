"""Fuzz the msac RTL against a spec-literal Python model of AV1 section 8.2.

Random tile bytes, random (monotone) CDFs, random request kinds. Independent of dav1d; the
dav1d trace replay in test_msac.py is the real oracle, this guards refactors and corner cases
(tile exhaustion / padding, N from 2..16, saturated counters).
Env: MSAC_FUZZ_TILES (default 40), MSAC_FUZZ_SEED (default 1).
"""
import os
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, Timer, ClockCycles

EC_PROB_SHIFT = 6
EC_MIN_PROB = 4


class SpecDecoder:
    """Straight transcription of the spec's init_symbol / read_symbol, non-inverted CDFs."""

    def __init__(self, data, disable_cdf_update):
        self.data = data
        self.bitpos = 0
        self.disable_cdf_update = disable_cdf_update
        sz = len(data)
        num_bits = min(sz * 8, 15)
        buf = self.f(num_bits)
        padded = buf << (15 - num_bits)
        self.value = ((1 << 15) - 1) ^ padded
        self.range = 1 << 15
        self.max_bits = 8 * sz - 15

    def f(self, n):
        x = 0
        for _ in range(n):
            byte = self.data[self.bitpos >> 3]
            bit = (byte >> (7 - (self.bitpos & 7))) & 1
            x = (x << 1) | bit
            self.bitpos += 1
        return x

    def read_symbol(self, cdf):
        """cdf: list of N+1 ints, cdf[N-1] == 32768, cdf[N] = counter. Modified in place."""
        N = len(cdf) - 1
        cur = self.range
        symbol = -1
        while True:
            symbol += 1
            prev = cur
            fval = (1 << 15) - cdf[symbol]
            cur = ((self.range >> 8) * (fval >> EC_PROB_SHIFT)) >> (7 - EC_PROB_SHIFT)
            cur += EC_MIN_PROB * (N - symbol - 1)
            if not (self.value < cur):
                break
        self.range = prev - cur
        self.value -= cur
        bits = 15 - (self.range.bit_length() - 1)
        self.range <<= bits
        num_bits = min(bits, max(0, self.max_bits))
        new_data = self.f(num_bits)
        padded = new_data << (bits - num_bits)
        self.value = padded ^ (((self.value + 1) << bits) - 1)
        self.max_bits -= bits
        if not self.disable_cdf_update:
            rate = 3 + (cdf[N] > 15) + (cdf[N] > 31) + min(N.bit_length() - 1, 2)
            tmp = 0
            for i in range(N - 1):
                tmp = (1 << 15) if i == symbol else tmp
                if tmp < cdf[i]:
                    cdf[i] -= (cdf[i] - tmp) >> rate
                else:
                    cdf[i] += (tmp - cdf[i]) >> rate
            cdf[N] += 1 if cdf[N] < 32 else 0
        return symbol


def random_cdf(rng, N):
    """Random strictly-increasing CDF with N symbols; returns [cdf0..cdfN-1=32768, count]."""
    cuts = sorted(rng.sample(range(1, 32768), N - 1))
    return cuts + [32768, rng.choice([0, 0, 1, 5, 15, 16, 31, 32])]


def to_icdf(cdf):
    N = len(cdf) - 1
    return [32768 - c for c in cdf[:N - 1]]  # N-1 entries, the implicit last is 0


def pack(vals):
    v = 0
    for k, c in enumerate(vals):
        v |= (c & 0xFFFF) << (16 * k)
    return v


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


@cocotb.test()
async def fuzz_vs_spec_model(dut):
    ntiles = int(os.environ.get("MSAC_FUZZ_TILES", "40"))
    rng = random.Random(int(os.environ.get("MSAC_FUZZ_SEED", "1")))
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.init.value = 0
    dut.in_valid.value = 0
    dut.in_eos.value = 0
    dut.req_valid.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst.value = 0
    await RisingEdge(dut.clk)

    checked = 0
    for ti in range(ntiles):
        sz = rng.choice([0, 1, 2, 3, 5, 8, 16, 40, 200])
        data = bytes(rng.getrandbits(8) for _ in range(sz))
        disable = rng.random() < 0.3
        model = SpecDecoder(data, disable)
        # the model consumes at most 8*sz real bits; beyond that it pads. Ask for plenty of symbols.
        nreq = rng.randint(1, 2 * sz + 20)

        dut.cdf_update_en.value = 0 if disable else 1
        dut.init.value = 1
        await RisingEdge(dut.clk)
        await Timer(1, "ns")
        dut.init.value = 0
        feeder = cocotb.start_soon(feed_bytes(dut, data))

        for ri in range(nreq):
            kind = rng.choice([0, 0, 0, 1, 2])
            if kind == 0:
                N = rng.randint(2, 16)
                cdf = random_cdf(rng, N)
                icdf = to_icdf(cdf)
                cnt = cdf[N]
            elif kind == 1:
                N = 2
                fprob = rng.randint(1, 32767)          # inverted prob of symbol 0
                cdf = [32768 - fprob, 32768, 0]
                icdf = [fprob]
                cnt = 0
            else:
                N = 2
                cdf = [1 << 14, 1 << 15, 0]
                icdf = []
                cnt = 0
            cdf_before = list(cdf)
            exp_sym = model.read_symbol(cdf if kind == 0 else list(cdf))  # bool kinds: no update
            exp_rng = model.range
            exp_icdf = to_icdf(cdf) if kind == 0 and not disable else None
            exp_cnt = cdf[N] if kind == 0 and not disable else None

            dut.req_kind.value = kind
            dut.req_n.value = N - 1
            dut.req_cdf.value = pack(icdf)
            dut.req_cnt.value = cnt
            dut.req_valid.value = 1
            while True:
                await ReadOnly()
                ok = int(dut.req_ready.value) == 1
                await RisingEdge(dut.clk)
                if ok:
                    break
            await ReadOnly()
            where = f"tile {ti} (sz={sz}, dis={disable}) req {ri} kind={kind} N={N} cdf={cdf_before}"
            assert int(dut.resp_valid.value) == 1, where
            got_sym, got_rng = int(dut.resp_sym.value), int(dut.resp_rng.value)
            assert got_sym == exp_sym, f"{where}: sym {got_sym} != {exp_sym}"
            assert got_rng == exp_rng, f"{where}: rng {got_rng} != {exp_rng}"
            if exp_icdf is not None:
                got_cdf = int(dut.resp_cdf.value)
                got = [(got_cdf >> (16 * k)) & 0xFFFF for k in range(N - 1)]
                assert got == exp_icdf, f"{where}: icdf {got} != {exp_icdf}"
                assert int(dut.resp_cnt.value) == exp_cnt, f"{where}: cnt {int(dut.resp_cnt.value)} != {exp_cnt}"
            await Timer(1, "ns")
            dut.req_valid.value = 0
            checked += 1
        if not feeder.done():
            feeder.cancel()
        dut.in_valid.value = 0
        dut.in_eos.value = 0
        await ClockCycles(dut.clk, 2)
    dut._log.info(f"OK: {checked} symbols across {ntiles} random tiles match the spec model")
