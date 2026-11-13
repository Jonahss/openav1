"""The deblocking stage of the streaming filter chain as it will see the picture (docs/filter-chain.md): one
superblock row of pre-filter samples in, finished lines out, and only CTX lines of each plane carried from one row
to the next. The filter arithmetic and the edge rules are lf_model's; the column order is lf_stream_model's.

For superblock row k of a plane with band lines [y0, y1):
  - the stage holds the last CTX lines of the previous band (filtered by that band's edges, not yet by this band's
    top horizontal edges) and receives the band's own lines;
  - it filters the band's vertical edges, then its horizontal edges (two columns behind): these may read and write the
    held lines, never anything above them;
  - it emits lines [y0 - CTX, y1 - CTX) as final (from line 0 for the first band, through the last line for the last
    band) and holds the band's last CTX lines.
Any access outside the held lines and the band raises: that is the check that CTX lines are enough.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lf_model as lfm            # noqa: E402
import lf_stream_model as lsm     # noqa: E402

CTX = 8


class Window:
    """Lines [lo, hi) of a plane; indexing outside is an error."""
    def __init__(self):
        self.rows = {}
        self.lo = self.hi = 0

    def __getitem__(self, y):
        if not (self.lo <= y < self.hi):
            raise IndexError(f"line {y} outside the stage's window [{self.lo}, {self.hi})")
        return self.rows[y]


class DeblockStage(lsm.StreamLoopFilter):
    def __init__(self, hdr, state):
        self.h = hdr
        self.d = state
        self.planes = [Window() for _ in range(hdr.NumPlanes)]
        self.on = not (hdr.loop_filter_level[0] == 0 and hdr.loop_filter_level[1] == 0)

    def band(self, row0, row1, lines, last, lag=2):
        """lines[plane] = the band's pre-filter lines (lists, modified in place); returns out[plane] = [(y, line), ...]
        final lines in order."""
        h = self.h
        out = []
        for plane in range(h.NumPlanes):
            sy = h.subsampling_y if plane else 0
            y0 = (row0 * lfm.MI_SIZE) >> sy
            w = self.planes[plane]
            for i, ln in enumerate(lines[plane]):
                w.rows[y0 + i] = ln
            y1 = y0 + len(lines[plane])
            w.lo = max(0, y0 - CTX); w.hi = y1
            if self.on and (plane == 0 or h.loop_filter_level[1 + plane]):
                rowStep = 1 if plane == 0 else (1 << h.subsampling_y)
                colStep = 1 if plane == 0 else (1 << h.subsampling_x)
                cols = list(range(0, h.MiCols, colStep)); rows = list(range(row0, row1, rowStep))
                for i in range(len(cols) + lag):
                    if i < len(cols):
                        for row in rows:
                            self.edge(plane, 0, row, cols[i])
                    j = i - lag
                    if 0 <= j < len(cols):
                        for row in rows:
                            self.edge(plane, 1, row, cols[j])
            end = y1 if last else y1 - CTX
            fin = [(y, w.rows.pop(y)) for y in range(w.lo, max(w.lo, end))]
            out.append(fin)
        return out


def run_frame(hdr, state, planes_pre, sb_lines=64, lag=2):
    """Feed a pre-filter frame band by band; return the filtered planes assembled from the emitted lines."""
    import copy
    st = DeblockStage(hdr, state)
    res = [[None] * len(planes_pre[p]) for p in range(hdr.NumPlanes)]
    band = sb_lines // lfm.MI_SIZE
    for row0 in range(0, hdr.MiRows, band):
        row1 = min(row0 + band, hdr.MiRows); last = row1 >= hdr.MiRows
        lines = []
        for p in range(hdr.NumPlanes):
            sy = hdr.subsampling_y if p else 0
            y0 = (row0 * lfm.MI_SIZE) >> sy
            y1 = len(planes_pre[p]) if last else (row1 * lfm.MI_SIZE) >> sy       # the last band takes the picture's remaining lines
            lines.append([list(r) for r in planes_pre[p][y0:y1]])
        for p, fin in enumerate(st.band(row0, row1, lines, last, lag)):
            for y, ln in fin:
                assert res[p][y] is None, f"plane {p} line {y} emitted twice"
                res[p][y] = ln
    for p in range(hdr.NumPlanes):
        missing = [y for y, r in enumerate(res[p]) if r is None]
        assert not missing, f"plane {p}: lines never emitted {missing[:8]}"
    return res
