"""The loop restoration stage of the streaming filter chain as it will see the picture (docs/filter-chain.md):
CDEF lines and deblocked lines arrive in order, restored lines leave in order. The arithmetic is lr_model's.

What the stage holds:
  - CDEF lines from three above the next 4-line block row (not above the stripe's first line) to the newest arrived;
  - of the deblocked picture only the four lines around every stripe boundary (the two above a stripe's first line and
    the stripe's first two): the stripe rule reads nothing else of it.
A block row is restored as soon as every line it reads has arrived; a read of a line that was dropped or never kept
raises (IndexError), a read of a line still to come defers the row (NotYet). `held_max` records the most CDEF lines
carried from one call to the next, per plane. Frames with super-resolution are not handled here (the upscale sits
between CDEF and this stage and works line by line).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lr_model as lrm        # noqa: E402

MI_SIZE = 4


class NotYet(Exception):
    pass


class Window:
    def __init__(self, final_height):
        self.rows = {}
        self.lo = self.hi = 0
        self.ended = False
        self.h = final_height

    def __getitem__(self, y):
        if y >= self.hi:
            if self.ended:
                raise IndexError(f"line {y} past the picture's {self.hi} lines")
            raise NotYet()
        if y < self.lo or y not in self.rows:
            raise IndexError(f"line {y} is not held (window from {self.lo}, {len(self.rows)} lines kept)")
        return self.rows[y]


class LrStage(lrm.LoopRestoration):
    def __init__(self, hdr, state, heights):
        self.h = hdr
        self.s = state
        n = hdr.NumPlanes
        self.cur = [Window(heights[p]) for p in range(n)]      # deblocked: boundary lines only
        self.cdef = [Window(heights[p]) for p in range(n)]
        self.lr = [dict() for _ in range(n)]
        self.heights = heights
        self.next_y = 0                                         # next luma block row (line)
        self.emitted = [0] * n
        self.held_max = [0] * n
        self.cur_kept_max = [0] * n
        self.on = not all(t == lrm.RESTORE_NONE for t in hdr.FrameRestorationType[:n])
        bd = hdr.BitDepth
        self.InterRound0 = 3 + (2 if bd == 12 else 0)
        self.InterRound1 = 11 - (2 if bd == 12 else 0)

    def _sy(self, p):
        return self.h.subsampling_y if p else 0

    def _boundary(self, p, y):
        """deblocked line y of plane p is one of the four around a stripe boundary"""
        sy = self._sy(p)
        n = ((y << sy) + 8) // 64
        for k in (n, n + 1):
            b = (-8 + k * 64) >> sy
            if -2 <= y - b <= 1:
                return True
        return False

    def feed(self, cur_lines, cdef_lines, last):
        h = self.h
        for p in range(h.NumPlanes):
            w = self.cur[p]
            for y, ln in cur_lines[p]:
                assert y == w.hi, f"deblocked plane {p}: line {y} arrived, expected {w.hi}"
                if self._boundary(p, y):
                    w.rows[y] = ln
                w.hi = y + 1
            w = self.cdef[p]
            for y, ln in cdef_lines[p]:
                assert y == w.hi, f"cdef plane {p}: line {y} arrived, expected {w.hi}"
                w.rows[y] = ln; w.hi = y + 1
                self.lr[p][y] = ln[:]                          # LrFrame starts as a copy of the CDEF frame
            if last:
                self.cur[p].ended = True; self.cdef[p].ended = True
        while self.on and self.next_y < h.FrameHeight:
            y = self.next_y
            try:
                for x in range(0, h.UpscaledWidth, MI_SIZE):
                    for plane in range(h.NumPlanes):
                        if h.FrameRestorationType[plane] != lrm.RESTORE_NONE:
                            self.block(plane, y >> 2, x >> 2)
            except NotYet:
                break
            self.next_y = y + MI_SIZE
            # drop what no later block row reads
            for p in range(h.NumPlanes):
                sy = self._sy(p)
                stripe = (self.next_y + 8) // 64
                start = (-8 + stripe * 64) >> sy
                keep = max(0, start, (self.next_y >> sy) - 3)
                w = self.cdef[p]
                for yy in [k for k in w.rows if k < keep]:
                    del w.rows[yy]
                w.lo = max(w.lo, min(keep, w.hi))
                wc = self.cur[p]
                for yy in [k for k in wc.rows if k < start - 2]:
                    del wc.rows[yy]
        if not last:
            for p in range(h.NumPlanes):
                self.held_max[p] = max(self.held_max[p], len(self.cdef[p].rows))
                self.cur_kept_max[p] = max(self.cur_kept_max[p], len(self.cur[p].rows))
        res = []
        for p in range(h.NumPlanes):
            sy = self._sy(p)
            if not self.on:
                done = self.cdef[p].hi
            elif last and self.next_y >= h.FrameHeight:
                done = self.heights[p]
            else:
                done = min(self.next_y >> sy, self.cdef[p].hi)
            fin = [(yy, self.lr[p].pop(yy)) for yy in range(self.emitted[p], done)]
            self.emitted[p] = max(self.emitted[p], done)
            res.append(fin)
        return res
