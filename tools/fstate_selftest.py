#!/usr/bin/env python3
"""The card-state projection (seq_model._project_np, numpy, on the card's 8x8 words) against the scalar spec
implementation (SequenceState.project) on random slots and frame headers. python tools/fstate_selftest.py [seeds]"""
import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tb"))
import numpy as np
import seq_model as sm
import tile_model as tm


class H:
    pass


def make(seed):
    rnd = random.Random(seed)
    h = H()
    h.MiRows, h.MiCols = rnd.choice([16, 24, 33, 64, 67, 128]), rnd.choice([16, 30, 45, 64, 96, 160])
    h.enable_order_hint = 1
    h.OrderHintBits = rnd.choice([5, 7, 8])
    m = 1 << h.OrderHintBits
    h.order_hint = rnd.randrange(m)
    h.OrderHints = [0] + [rnd.randrange(m) for _ in range(7)]
    h.ref_frame_idx = [rnd.randrange(8) for _ in range(7)]
    h.FrameIsIntra, h.use_ref_frame_mvs = 0, 1
    R, Cc = h.MiRows, h.MiCols
    h8, w8 = R >> 1, Cc >> 1
    scalar, card = sm.SequenceState(), sm.CardSequenceState()
    for i in range(8):
        same = rnd.random() < 0.8
        rr, cc = (R, Cc) if same else (rnd.choice([16, 32]), rnd.choice([16, 32]))
        refs8 = np.zeros((rr >> 1, cc >> 1), np.int8)
        mvs8 = np.zeros((rr >> 1, cc >> 1, 2), np.int16)
        for y in range(rr >> 1):
            for x in range(cc >> 1):
                if rnd.random() < 0.6:
                    refs8[y, x] = rnd.randrange(1, 8)
                    mvs8[y, x] = (rnd.randrange(-4095, 4096), rnd.randrange(-4095, 4096))
        for st in (scalar, card):
            s = sm.Slot()
            s.mi_rows, s.mi_cols = rr, cc
            s.frame_type = rnd.choice([sm.KEY_FRAME, sm.INTER_FRAME, sm.INTER_FRAME, sm.INTRA_ONLY_FRAME])
            s.order_hint = rnd.randrange(m)
            s.saved_order_hints = [rnd.randrange(m) for _ in range(8)]
            s.ref_frames = [[sm.NONE] * cc for _ in range(rr)]
            s.mvs = [[sm.ZERO_MV] * cc for _ in range(rr)]
            for y in range(rr >> 1):
                for x in range(cc >> 1):
                    if refs8[y, x]:
                        s.ref_frames[2 * y + 1][2 * x + 1] = int(refs8[y, x])
                        s.mvs[2 * y + 1][2 * x + 1] = (int(mvs8[y, x, 0]), int(mvs8[y, x, 1]))
            s.refs8, s.mvs8 = refs8, mvs8
            st.slots[i] = s
        # the two states must hold identical slot objects' values but separate instances: share the random draws above
        card.slots[i].frame_type = scalar.slots[i].frame_type
        card.slots[i].order_hint = scalar.slots[i].order_hint
        card.slots[i].saved_order_hints = list(scalar.slots[i].saved_order_hints)
    return h, scalar, card


def main():
    seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 40
    cells = 0
    for seed in range(seeds):
        h, scalar, card = make(seed)
        a = scalar.motion_field_estimation(h)
        b = card.motion_field_estimation(h)
        for ref in range(1, 8):
            for y in range(h.MiRows >> 1):
                for x in range(h.MiCols >> 1):
                    va, vb = a[ref][y][x], b[ref][y][x]
                    assert tuple(va) == tuple(vb), f"seed {seed} ref {ref} ({y},{x}): scalar {va} numpy {vb}"
                    cells += 1
    print(f"OK: {seeds} frames, {cells} motion-field cells identical (numpy vs scalar projection)")


if __name__ == "__main__":
    main()
