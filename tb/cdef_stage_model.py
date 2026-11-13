"""The CDEF stage of the streaming filter chain as it will see the picture (docs/filter-chain.md): deblocked lines
arrive in order (from the deblocking stage, which emits a superblock row shifted up by its context), CDEF lines leave
in order, and the stage holds only the deblocked lines its next blocks still read. The arithmetic is cdef_model's.

An 8x8 block row at luma line y reads deblocked luma y - 2 .. y + 9 and, for its chroma blocks, chroma lines
yc - 2 .. yc + (8 >> subY) + 1. A block row is filtered as soon as those lines have arrived (or the picture has ended);
its lines are then final. Deblocked lines above the next block row's reach are dropped. Any read outside the held
lines raises: the check that the stage's window is enough. `held_max` records the most deblocked lines carried from one call to the next, per plane.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cdef_model as cdm      # noqa: E402

REACH = 2                     # CDEF taps reach two samples out in every direction


class Window:
    def __init__(self):
        self.rows = {}
        self.lo = self.hi = 0

    def __getitem__(self, y):
        if not (self.lo <= y < self.hi):
            raise IndexError(f"line {y} outside the stage's window [{self.lo}, {self.hi})")
        return self.rows[y]


class CdefStage(cdm.Cdef):
    def __init__(self, hdr, state, heights):
        """heights[plane]: lines of the picture buffer per plane (may exceed the MI-aligned area)."""
        self.h = hdr
        self.s = state
        self.cur = [Window() for _ in range(hdr.NumPlanes)]
        self.out = [dict() for _ in range(hdr.NumPlanes)]     # line -> list, CdefFrame lines not yet emitted
        self.heights = heights
        self.next_r = 0                                         # next 8x8 block row (4x4 units)
        self.emitted = [0] * hdr.NumPlanes
        self.held_max = [0] * hdr.NumPlanes
        self.y_pri = [v >> 2 for v in hdr.cdef_y_strengths]
        self.y_sec = [(v & 3) + ((v & 3) == 3) for v in hdr.cdef_y_strengths]
        self.uv_pri = [v >> 2 for v in hdr.cdef_uv_strengths]
        self.uv_sec = [(v & 3) + ((v & 3) == 3) for v in hdr.cdef_uv_strengths]

    def _sub(self, plane):
        return self.h.subsampling_y if plane else 0

    def feed(self, lines, last):
        """lines[plane] = [(y, line), ...] deblocked, in order, continuing where the last call stopped.
        Returns out[plane] = [(y, line), ...] final CDEF lines in order."""
        h = self.h
        for p in range(h.NumPlanes):
            w = self.cur[p]
            for y, ln in lines[p]:
                assert y == w.hi, f"plane {p}: line {y} arrived, expected {w.hi}"
                w.rows[y] = ln; w.hi = y + 1
                self.out[p][y] = ln[:]                         # CdefFrame starts as a copy
        # block rows whose reach has arrived
        while self.next_r < h.MiRows:
            r = self.next_r
            ready = True
            for p in range(h.NumPlanes):
                sy = self._sub(p)
                need = ((r * 4) >> sy) + (8 >> sy) + REACH       # exclusive: lines below this are read
                if not (self.cur[p].hi >= need or last):
                    ready = False
            if not ready:
                break
            for c in range(0, h.MiCols, 2):
                idx = self.s.cdef_idx.get((r & ~15, c & ~15), -1)
                self.block(r, c, idx)
            self.next_r = r + 2
            for p in range(h.NumPlanes):                       # deblocked lines above the next block row's reach go
                sy = self._sub(p)
                keep = max(0, ((self.next_r * 4) >> sy) - REACH)
                w = self.cur[p]
                for y in range(w.lo, min(keep, w.hi)):
                    del w.rows[y]
                w.lo = max(w.lo, min(keep, w.hi))
        if not last:
            for p in range(h.NumPlanes):                       # what is carried to the next call
                self.held_max[p] = max(self.held_max[p], self.cur[p].hi - self.cur[p].lo)
        res = []
        for p in range(h.NumPlanes):
            sy = self._sub(p)
            done = self.heights[p] if (last and self.next_r >= h.MiRows) else min(((self.next_r * 4) >> sy), self.cur[p].hi)
            fin = [(y, self.out[p].pop(y)) for y in range(self.emitted[p], done)]
            self.emitted[p] = max(self.emitted[p], done)
            res.append(fin)
        return res
