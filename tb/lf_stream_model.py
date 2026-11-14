"""The deblocking filter in the order the streaming filter chain will use (docs/filter-chain.md), as a model:
superblock row by superblock row, and inside a row one column of 4x4 units at a time, left to right. For a column the
vertical edges of every unit in the row band come first; the horizontal edges follow `lag` columns behind, so that
every sample a horizontal edge reads has had all the vertical edges that can change it (an edge at column c' changes
samples of column c only for c - 1 <= c' <= c + 2 at the widest luma filter). The planes are independent.

check(): the result must equal lf_model.LoopFilter.apply() (the specification's whole-frame order) exactly; this is
the proof that the order, and therefore the context a streaming stage must keep, is sound. `touched` records, per
band, how far above the band the horizontal pass wrote and how far left of the current column anything wrote: the
context sizes in the proposal.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lf_model as lfm  # noqa: E402


class _Spy(list):
    """A picture row list that reports writes: rows are wrapped so plane[y][x] = v is seen."""


class StreamLoopFilter(lfm.LoopFilter):
    def apply_stream(self, sb_lines=64, lag=2):
        h = self.h
        self.reach_up = [0, 0, 0]          # lines above the band written by the band's horizontal edges, per plane
        self.reach_left = [0, 0, 0]        # samples left of a vertical edge's own column written, per plane
        if h.loop_filter_level[0] == 0 and h.loop_filter_level[1] == 0:
            return
        band = sb_lines // lfm.MI_SIZE     # 4x4 rows per superblock row
        for row0 in range(0, h.MiRows, band):
            row1 = min(row0 + band, h.MiRows)
            for plane in range(h.NumPlanes):
                if not (plane == 0 or h.loop_filter_level[1 + plane]):
                    continue
                rowStep = 1 if plane == 0 else (1 << h.subsampling_y)
                colStep = 1 if plane == 0 else (1 << h.subsampling_x)
                cols = list(range(0, h.MiCols, colStep))
                rows = list(range(row0, row1, rowStep))
                for i in range(len(cols) + lag):
                    if i < len(cols):
                        for row in rows:
                            self.edge(plane, 0, row, cols[i])          # vertical edges of column i
                    j = i - lag
                    if 0 <= j < len(cols):
                        for row in rows:
                            self.edge(plane, 1, row, cols[j])          # horizontal edges, `lag` columns behind


def check(hdr, state, planes_pre, sb_lines=64, lag=2):
    """Filter copies of planes_pre both ways; return (identical, first differences)."""
    import copy
    a = copy.deepcopy(planes_pre); b = copy.deepcopy(planes_pre)
    lfm.LoopFilter(hdr, state, a).apply()
    StreamLoopFilter(hdr, state, b).apply_stream(sb_lines, lag)
    bad = []
    for p in range(hdr.NumPlanes):
        for y in range(len(a[p])):
            if a[p][y] != b[p][y]:
                for x in range(len(a[p][y])):
                    if a[p][y][x] != b[p][y][x]:
                        bad.append((p, y, x, b[p][y][x], a[p][y][x]))
                        if len(bad) >= 8:
                            return False, bad, a
    changed = sum(1 for p in range(hdr.NumPlanes) for y in range(len(a[p])) for x in range(len(a[p][y])) if a[p][y][x] != planes_pre[p][y][x])
    return not bad, bad, changed
