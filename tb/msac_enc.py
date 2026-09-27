"""Arithmetic ENCODER matching the AV1 symbol decoder (spec 8.2), for generating synthetic tiles.

This is the inverse of tile_model.SymbolDecoder: feed it the same sequence of (cdf, symbol) that a
decoder would read, and it produces tile bytes the decoder turns back into exactly those symbols.
It follows libaom's od_ec_encode_q15 / od_ec_enc_done (the only public encoder for this coder),
with CDF adaptation identical to the decoder's so that both sides track the same probabilities.

Typical use (tools/gen_tile.py): run tile_model.TileDecoder with a RecordingDecoder that chooses
symbol values (random or scripted) and records every (cdf, value); then encode that list.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tile_model as tm  # noqa: E402

EC_PROB_SHIFT = 6
EC_MIN_PROB = 4
EC_WIN = 32


class Encoder:
    def __init__(self):
        self.buf = []          # output bytes (little-endian carry-propagated), built in reverse like libaom
        self.low = 0
        self.rng = 0x8000
        self.cnt = -9
        self.precarry = []     # 16-bit "pre-carry" words, libaom style

    def _encode_q15(self, fl, fh, s, nsyms):
        """fl/fh: inverse-CDF values (32768 - cdf[s-1]) and (32768 - cdf[s]), libaom convention."""
        low = self.low
        r = self.rng
        assert 32768 <= r <= 65535
        N = nsyms - 1
        if fl < 32768:
            u = ((r >> 8) * (fl >> EC_PROB_SHIFT) >> (7 - EC_PROB_SHIFT)) + EC_MIN_PROB * (N - (s - 1))
            v = ((r >> 8) * (fh >> EC_PROB_SHIFT) >> (7 - EC_PROB_SHIFT)) + EC_MIN_PROB * (N - (s + 0))
            low += r - u
            r = u - v
        else:
            r -= ((r >> 8) * (fh >> EC_PROB_SHIFT) >> (7 - EC_PROB_SHIFT)) + EC_MIN_PROB * (N - (s + 0))
        self._normalize(low, r)

    def _normalize(self, low, rng):
        d = 16 - rng.bit_length()          # leading zeros of the 16-bit range
        c = self.cnt
        s = c + d
        if s >= 0:
            c += 16
            m = (1 << c) - 1
            if s >= 8:
                self.precarry.append(low >> c)
                low &= m
                c -= 8
                m >>= 8
            self.precarry.append(low >> c)
            s = c + d - 24
            low &= m
        self.low = low << d
        self.rng = rng << d
        self.cnt = s

    def encode(self, cdf, symbol):
        """cdf: spec-form list [cdf0, ..., 32768, counter]; symbol in 0..N-1. Adapts cdf like the decoder."""
        N = len(cdf) - 1
        icdf = [32768 - c for c in cdf[:N]]     # icdf[N-1] == 0
        fl = icdf[symbol - 1] if symbol > 0 else 32768
        fh = icdf[symbol]
        self._encode_q15(fl, fh, symbol, N)
        # adaptation (spec 8.2.6) — must match the decoder exactly
        rate = 3 + (cdf[N] > 15) + (cdf[N] > 31) + min((N).bit_length() - 1, 2)
        tmp = 0
        for i in range(N - 1):
            tmp = (1 << 15) if i == symbol else tmp
            if tmp < cdf[i]:
                cdf[i] -= (cdf[i] - tmp) >> rate
            else:
                cdf[i] += (tmp - cdf[i]) >> rate
        cdf[N] += 1 if cdf[N] < 32 else 0

    def encode_noadapt(self, cdf, symbol):
        N = len(cdf) - 1
        icdf = [32768 - c for c in cdf[:N]]
        fl = icdf[symbol - 1] if symbol > 0 else 32768
        fh = icdf[symbol]
        self._encode_q15(fl, fh, symbol, N)

    def encode_bool(self, bit):
        self.encode_noadapt([1 << 14, 1 << 15, 0], bit)

    def encode_literal(self, n, value):
        for i in range(n - 1, -1, -1):
            self.encode_bool((value >> i) & 1)

    def done(self):
        """Flush (libaom od_ec_enc_done), then resolve carries. Returns the tile bytes."""
        low = self.low
        c = self.cnt
        s = 10
        m = 0x3FFF
        e = ((low + m) & ~m) | (m + 1)
        s += c
        out = list(self.precarry)
        if s > 0:
            n = (1 << (c + 16)) - 1
            while True:
                out.append(e >> (c + 16))
                e &= n
                s -= 8
                c -= 8
                n >>= 8
                if s <= 0:
                    break
        # carry propagation from the end
        data = bytearray()
        carry = 0
        res = []
        for w in reversed(out):
            w += carry
            carry = w >> 8
            res.append(w & 0xFF)
        return bytes(reversed(res))


class RecordingDecoder:
    """Drop-in for tile_model.SymbolDecoder: instead of reading bits it *chooses* values via `pick`
    (a callable (cdf_or_None, N, name) -> value) and records (kind, cdf_copy, value) for the encoder."""

    def __init__(self, pick, disable_cdf_update=0):
        self.pick = pick
        self.disable_cdf_update = disable_cdf_update
        self.events = []
        self.range = 0x8000

    def read_symbol(self, cdf, name=""):
        N = len(cdf) - 1
        v = self.pick(cdf, N, name)
        assert 0 <= v < N, (name, v, N)
        self.events.append(("S", list(cdf), v, self.disable_cdf_update))
        if not self.disable_cdf_update:
            rate = 3 + (cdf[N] > 15) + (cdf[N] > 31) + min(N.bit_length() - 1, 2)
            tmp = 0
            for i in range(N - 1):
                tmp = (1 << 15) if i == v else tmp
                if tmp < cdf[i]:
                    cdf[i] -= (cdf[i] - tmp) >> rate
                else:
                    cdf[i] += (tmp - cdf[i]) >> rate
            cdf[N] += 1 if cdf[N] < 32 else 0
        return v

    def read_bool(self, name=""):
        v = self.pick(None, 2, name)
        self.events.append(("B", None, v, 1))
        return v

    def read_literal(self, n, name=""):
        x = 0
        for _ in range(n):
            x = 2 * x + self.read_bool(name)
        return x

    def read_ns(self, n, name=""):
        w = n.bit_length()
        m = (1 << w) - n
        v = self.read_literal(w - 1, name)
        if v < m:
            return v
        extra_bit = self.read_literal(1, name)
        return (v << 1) - m + extra_bit


def encode_events(events):
    enc = Encoder()
    for kind, cdf, v, noadapt in events:
        if kind == "B":
            enc.encode_bool(v)
        elif noadapt:
            enc.encode_noadapt(cdf, v)
        else:
            enc.encode(cdf, v)
    return enc.done()


if __name__ == "__main__":
    # round-trip self test: random symbols (several adapting CDFs + raw bools) -> encoder -> decoder
    import random
    rng = random.Random(1)
    for trial in range(300):
        n_sym = rng.randint(1, 600)
        cdfs = {}
        events = []      # (kind, pre-adaptation cdf snapshot, value, noadapt)
        for _ in range(n_sym):
            if rng.random() < 0.3:
                events.append(("B", None, rng.randint(0, 1), 1))
                continue
            key = rng.randint(0, 5)
            if key not in cdfs:
                N = rng.randint(2, 16)
                cdfs[key] = sorted(rng.sample(range(1, 32768), N - 1)) + [32768, 0]
            cdf = cdfs[key]
            N = len(cdf) - 1
            v = rng.randint(0, N - 1)
            events.append(("S", list(cdf), v, 0))
            RecordingDecoder(lambda c, n, name: v).read_symbol(cdf)      # adapt the live copy
        data = encode_events([(k, (list(c) if c else None), v, na) for k, c, v, na in events])
        dec = tm.SymbolDecoder(data, 0)
        for idx, (k, c, v, na) in enumerate(events):
            got = dec.read_bool() if k == "B" else dec.read_symbol(list(c))
            if got != v:
                print(f"trial {trial}: event {idx} decoded {got}, expected {v} ({len(data)} bytes)")
                sys.exit(1)
    print("msac_enc round-trip: 300 trials OK")
