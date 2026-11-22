#!/usr/bin/env python3
"""tools/mc_timeline.py: read a TD_MCTL timeline (tb/test_dec_inter.py mc_timeline: the motion-compensation pipeline and
engine 0's event queue clock by clock, run-length coded) and say what waited for what.

  mc_timeline.py <file> summary              shares of the span, the joint states, events
  mc_timeline.py <file> runs                 per run: request -> accepted -> patch complete -> start -> done, and
                                             which of run_go's conditions each start waited for last
  mc_timeline.py <file> blocks               per block: issue of its strips, idle until the next take, why
  mc_timeline.py <file> print <lo> <hi> [col ...]   the state changes between two clocks (relative to the first
                                             accepted request); default columns: the queue and both FSMs

Why it exists (2026-11-21): the card's counters said the fetch engine's ring was full half of a 4K frame; a ring four
times deeper changed nothing. Totals name a stage that waits, the timeline shows the one it waits for. First finding:
rec_fifo's next-block scan stops at a superblock start, so on a stream of one 64x64 block per superblock the block
FSM never takes a block early and idles from its last strip until the block's transform events are consumed."""
import collections
import sys
import numpy as np


def load(path):
    with open(path) as f:
        cols = f.readline().split()[1:]
        a = np.array([list(map(int, l.split())) for l in f])
    return cols, a[:, 0], a[:, 1:]


def per_clock(cols, cyc, v):
    n = cyc[-1] + 1
    idx = np.searchsorted(cyc, np.arange(n), side="right") - 1
    return {c: v[idx, i] for i, c in enumerate(cols)}


def rise(m):
    return np.flatnonzero(m[1:] & ~m[:-1]) + 1


def fall(m):
    return np.flatnonzero(~m[1:] & m[:-1]) + 1


def first_accept(X):
    return fall(X["rf_req_valid"] == 1)[0]


def summary(cols, cyc, v):
    C = {n: i for i, n in enumerate(cols)}
    g = lambda n: v[:, C[n]]
    dur = np.diff(np.append(cyc, cyc[-1] + 1))
    act = (g("bst") != 0) | (g("hst") != 0) | (g("vst") != 0) | (g("wr_busy") != 0)
    i0, i1 = np.argmax(act), len(act) - np.argmax(act[::-1])
    v, dur = v[i0:i1], dur[i0:i1]
    g = lambda n: v[:, C[n]]
    T = dur.sum()
    print(f"span {T} clocks, {len(v)} state changes")
    H, V = g("hst") != 0, g("vst") != 0
    rows = [("block FSM idle", g("bst") == 0), ("block FSM in B_MC_FETCH", g("bst") == 16), ("transform FSM busy", g("rs") != 0),
            ("H busy", H), ("V busy", V), ("H and V", H & V), ("V busy, stalled by the output FIFO", V & (g("f_stall") == 1)),
            ("a run waits between the sides (pend)", g("pend") == 1), ("two runs alive in the filter", g("alive") == 2),
            ("writer busy", g("wr_busy") == 1), ("rows ready, writer waits for its buffer", (g("wr_busy") == 0) & (g("rows_ready") > 0)),
            ("run queue full", g("pq_n") == 2), ("run queue empty", g("pq_n") == 0), ("rec_held", g("rec_held") == 1)]
    if "nb_valid" in C:
        rows += [("block FSM idle, event offered, no block record to take", (g("bst") == 0) & (g("ev_valid") == 1) & (g("nb_valid") == 0)),
                 ("syntax holds a transform block (tx_done, no ack)", (g("tx_done") == 1) & (g("tx_ack") == 0))]
    for name, m in rows:
        print(f"  {name:56s} {dur[m].sum():9d} {100 * dur[m].sum() / T:5.1f} %")
    print("  events: " + ", ".join(f"{n} {int((g(n) * dur).sum())}" for n in ("mc_start", "rf_done", "mc_done", "blk_take")))


def runs(cols, cyc, v):
    X = per_clock(cols, cyc, v)
    start, rfd, done = (np.flatnonzero(X[n] == 1) for n in ("mc_start", "rf_done", "mc_done"))
    acc, req = fall(X["rf_req_valid"] == 1), rise(X["rf_req_valid"] == 1)
    n = min(len(acc), len(rfd), len(start), len(done), len(req))
    acc, rfd, start, done, req = acc[:n], rfd[:n], start[:n], done[:n], req[:n]
    m = lambda x: f"{np.mean(x):8.1f} (median {np.median(x):7.1f})"
    print(f"{n} runs, clocks:")
    for name, d in (("request raised -> accepted", acc - req), ("accepted -> patch complete", rfd - acc), ("patch complete -> run start", start - rfd),
                    ("run start -> run done", done - start), ("start -> next start", np.diff(start))):
        print(f"  {name:32s} {m(d)}")

    def true_since(mask, at):                             # first clock of the run of trues ending just before `at`
        f0 = np.flatnonzero(~mask)
        out = np.empty(len(at), dtype=int)
        for i, x in enumerate(at):
            k = np.searchsorted(f0, x - 1, side="right") - 1
            out[i] = x if not mask[x - 1] else (f0[k] + 1 if k >= 0 else 0)
        return out
    conds = {"its patch": rfd, "the filter (h side idle, not pend, fewer than 2 alive)": true_since((X["hst"] == 0) & (X["pend"] == 0) & (X["alive"] < 2), start),
             "its run record (rec_held)": true_since(X["rec_held"] == 0, start)}
    cause, wait = collections.Counter(), collections.defaultdict(list)
    for i in range(n):
        c = {k: a[i] for k, a in conds.items()}
        k = max(c, key=c.get)
        cause[k] += 1
        wait[k].append(c[k] - sorted(c.values())[-2])
    print("  what a run's start waited for last (runs, mean clocks after the next-latest condition):")
    for k, c in cause.most_common():
        print(f"    {k:58s} {c:5d} {np.mean(wait[k]):8.1f}")


def blocks(cols, cyc, v):
    X = per_clock(cols, cyc, v)
    take = np.flatnonzero(X["blk_take"] == 1)
    idle_from = rise(X["bst"] == 0)                         # the block FSM returns to idle: its strips are issued
    done = np.flatnonzero(X["mc_done"] == 1)
    per, idl, tail, why = [], [], [], collections.Counter()
    for i in range(len(take) - 1):
        t0, t1 = take[i], take[i + 1]
        k = idle_from[(idle_from > t0) & (idle_from < t1)]
        if not len(k):
            continue
        per.append(t1 - t0); idl.append(t1 - k[-1])
        d = done[(done > t0) & (done <= t1)]
        tail.append(t1 - d[-1] if len(d) else 0)
        if "nb_valid" in X:
            seg = slice(k[-1], t1)
            kinds = X["ev_kind"][seg][(X["ev_valid"][seg] == 1) & (X["nb_valid"][seg] == 0)]
            why["event offered, no block record to take"] += len(kinds)
            why["nothing offered"] += int((X["ev_valid"][seg] == 0).sum())
    print(f"{len(per)} blocks: period {np.mean(per):.0f} clocks, block FSM idle before the next take {np.mean(idl):.0f} ({100 * np.sum(idl) / np.sum(per):.1f} %), "
          f"of which after the block's last run was done {np.mean(tail):.0f}")
    for k, c in why.items():
        print(f"  while idle: {k} {c} clocks")
    if "blk_info" in X:
        # the next block's record: how long it sat in the queue before it was taken
        info = np.flatnonzero(X["blk_info"] == 1)
        sat = [t - info[info < t][-1] for t in take[1:] if len(info[info < t])]
        print(f"  a block record sits in the queue {np.mean(sat):.0f} clocks before it is taken (median {np.median(sat):.0f})")


def show(cols, cyc, v, lo, hi, names):
    X = per_clock(cols, cyc, v)
    t0 = first_accept(X)
    names = names or [n for n in ("bst", "rs", "pq_n", "hst", "vst", "ev_valid", "ev_kind", "nb_valid", "ready0", "ready1", "q_count", "tx_inflight") if n in X]
    pulses = [n for n in ("blk_take", "mc_start", "mc_done", "rf_done", "blk_info", "tx_ack") if n in X]
    last = None
    for t in range(max(0, t0 + lo), min(len(X["bst"]), t0 + hi + 1)):
        key = tuple(int(X[n][t]) for n in names + pulses)
        if key != last:
            print(t - t0, " ".join(f"{n}={x}" for n, x in zip(names, key)), " ".join(p.upper() for p, x in zip(pulses, key[len(names):]) if x))
            last = key


if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    cols, cyc, v = load(sys.argv[1])
    cmd = sys.argv[2]
    if cmd == "summary":
        summary(cols, cyc, v)
    elif cmd == "runs":
        runs(cols, cyc, v)
    elif cmd == "blocks":
        blocks(cols, cyc, v)
    elif cmd == "print":
        show(cols, cyc, v, int(sys.argv[3]), int(sys.argv[4]), sys.argv[5:])
    else:
        sys.exit(__doc__)
