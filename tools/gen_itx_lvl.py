#!/usr/bin/env python3
"""Generate rtl/itx_lvl.sv: the level-folded 1D inverse transform engine.

The spec's DCT and ADST (tb/itx_model.py, 7.13.2) are sequences of in-place pair operations on a vector T:
B (rotation by cos128/sin128, Round2 12, optional swap) and H (Hadamard with clamp to r bits). The ops of one
transform form a DAG (each op reads and writes its two lanes); scheduled ASAP they fall into 2n-2 levels for a
2^n-point DCT, every level holding at most 32 independent ops. This engine executes ONE LEVEL PER CLOCK on
32 physical pair units (rtl/itx_pair.sv) over a 64-lane register vector: the wiring of each (program, level)
is fixed and generated here, so there is no indexed register file (the cost of the microcoded itx1d engine).

Programs pack 64/N rows of an N-point transform into the 64 lanes (row g at lanes g*N .. g*N+N-1), so the
2D wrapper loads one row per clock and the compute is amortised; DCT64 is one row. The DCT input permutation
(bit reversal) and the ADST input permutation are folded into the level-0 sources (level 0 reads in_vec, later
levels read rv); the ADST output permutation and its per-lane negation are the output map (out_vec).

  prog 0 DCT4 x16   1 DCT8 x8   2 DCT16 x4   3 DCT32 x2   4 DCT64 x1   5 ADST8 x8   6 ADST16 x4

Timing: start (while idle) runs level 0 in that same cycle from in_vec; one level per cycle after; done pulses
in the cycle after the last level's write with out_vec valid from then on (cycles = levels: 2,4,6,8,10,5,7).

Usage: tools/gen_itx_lvl.py > rtl/itx_lvl.sv      --check: simulate the generated tables in Python against
the model on random packed rows (run this after every change to the scheduler).
"""
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tb"))
import itx_model as m  # noqa: E402

OP_B, OP_H = 1, 2
NU = 32                                   # pair units
# (name, n, cls) ; rows packed = 64 >> n
PROGRAMS = [("DCT4", 2, 0), ("DCT8", 3, 0), ("DCT16", 4, 0), ("DCT32", 5, 0), ("DCT64", 6, 0),
            ("ADST8", 3, 1), ("ADST16", 4, 1)]


# ---------------------------------------------------------------- recording the spec ops
def record(n, cls):
    """Flat in-place op list [(op, a, b, angle, flip)] of one 2^n-point transform, permutations excluded."""
    ops = []

    def rb(T, a, b, angle, flip, r):
        ops.append((OP_B, a, b, angle & 255, flip))

    def rh(T, a, b, flip, r):
        ops.append((OP_H, a, b, 0, flip))

    saved = (m.B, m.H, m.inv_dct_permute, m.inv_adst_in_permute, m.inv_adst_out_permute)
    m.B, m.H = rb, rh
    m.inv_dct_permute = m.inv_adst_in_permute = m.inv_adst_out_permute = lambda T, nn: None
    try:
        T = [0] * (1 << n)
        (m.inv_dct if cls == 0 else m.inv_adst)(T, n, 24)
    finally:
        m.B, m.H, m.inv_dct_permute, m.inv_adst_in_permute, m.inv_adst_out_permute = saved
    return ops


def in_perm(n, cls):
    """lane i of the transform reads input element in_perm[i] (spec inv_dct_permute / inv_adst_in_permute)."""
    N = 1 << n
    if cls == 0:
        return [m.brev(n, i) for i in range(N)]
    return [(i - 1) if (i & 1) else (N - i - 1) for i in range(N)]


def out_perm(n, cls):
    """output element i = sign * T[idx]: list of (idx, neg) (identity for DCT, inv_adst_out_permute for ADST)."""
    N = 1 << n
    if cls == 0:
        return [(i, 0) for i in range(N)]
    res = []
    for i in range(N):
        a = (i >> 3) & 1
        b = ((i >> 2) & 1) ^ ((i >> 3) & 1)
        c = ((i >> 1) & 1) ^ ((i >> 2) & 1)
        d = (i & 1) ^ ((i >> 1) & 1)
        idx = ((d << 3) | (c << 2) | (b << 1) | a) >> (4 - n)
        res.append((idx, i & 1))
    return res


# ---------------------------------------------------------------- scheduling
def asap_levels(ops):
    last = {}
    levels = []
    for op in ops:
        _, a, b, _, _ = op
        lv = 1 + max(last.get(a, -1), last.get(b, -1))
        last[a] = last[b] = lv
        while len(levels) <= lv:
            levels.append([])
        levels[lv].append(op)
    return levels


def build_program(n, cls):
    """Packed program: levels of ops over 64 lanes, plus the level-0 source map and the output map."""
    N = 1 << n
    copies = 64 // N
    ops = record(n, cls)
    packed = [(op, a + g * N, b + g * N, ang, fl) for g in range(copies) for (op, a, b, ang, fl) in ops]
    levels = asap_levels(packed)
    ip = in_perm(n, cls)
    src0 = [g * N + ip[i] for g in range(copies) for i in range(N)]       # lane k's level-0 source in in_vec
    op_ = out_perm(n, cls)
    omap = [(g * N + op_[i][0], op_[i][1]) for g in range(copies) for i in range(N)]
    assert all(len(l) <= NU for l in levels)
    touched0 = {x for (_, a, b, _, _) in levels[0] for x in (a, b)}
    assert touched0 == set(range(64)), "level 0 must touch every lane (no separate load path)"
    return levels, src0, omap


class Assign:
    """Greedy op -> unit assignment minimising the fixed-wiring fan-in across all programs."""

    def __init__(self):
        self.fan_a = [set() for _ in range(NU)]
        self.fan_b = [set() for _ in range(NU)]
        self.wb = [set() for _ in range(64)]      # lane -> {(unit, 'a'|'b')}

    def level(self, ops, src_of):
        """ops of one level; src_of(lane) = the vector index the unit input reads. Returns [(unit, op)]."""
        res = []
        free = set(range(NU))
        for op in sorted(ops, key=lambda o: min(o[1], o[2])):
            _, a, b, _, _ = op
            sa, sb = src_of(a), src_of(b)
            home = min(a, b) // 2
            best = None
            for u in sorted(free, key=lambda u: (u - home) % NU):
                cost = (sa not in self.fan_a[u]) + (sb not in self.fan_b[u]) \
                     + ((u, 'a') not in self.wb[a]) + ((u, 'b') not in self.wb[b])
                if best is None or cost < best[0]:
                    best = (cost, u)
                if cost == 0:
                    break
            u = best[1]
            free.remove(u)
            self.fan_a[u].add(sa); self.fan_b[u].add(sb)
            self.wb[a].add((u, 'a')); self.wb[b].add((u, 'b'))
            res.append((u, op))
        return res


def schedule():
    """Returns per program: list of levels, each a list of (unit, src_a, src_b, op, angle, flip, lane_a, lane_b);
    and the output maps. Sources at level 0 index in_vec (permuted), later levels index rv."""
    asg = Assign()
    progs = []
    for name, n, cls in PROGRAMS:
        levels, src0, omap = build_program(n, cls)
        sched = []
        for lv, ops in enumerate(levels):
            src_of = (lambda k: src0[k]) if lv == 0 else (lambda k: k)
            rows = []
            for u, (op, a, b, ang, fl) in asg.level(ops, src_of):
                rows.append((u, src_of(a), src_of(b), op, ang, fl, a, b))
            sched.append(rows)
        progs.append((name, n, cls, sched, omap))
    stats = dict(fan_a=max(len(s) for s in asg.fan_a), fan_b=max(len(s) for s in asg.fan_b),
                 wb=max(len(s) for s in asg.wb))
    return progs, stats


# ---------------------------------------------------------------- python simulation of the tables
def sim_program(prog, vec, r):
    """Run the scheduled program on a 64-element input exactly as the RTL will (level by level)."""
    name, n, cls, sched, omap = prog
    rv = [0] * 64
    for lv, rows in enumerate(sched):
        sel = vec if lv == 0 else rv
        new = list(rv)
        for (u, sa, sb, op, ang, fl, a, b) in rows:
            T = {a: sel[sa], b: sel[sb]}
            if op == OP_B:
                m.B(T, a, b, ang, fl, 64)
            else:
                m.H(T, a, b, fl, r)
            new[a], new[b] = T[a], T[b]
        rv = new
    return [(-rv[idx] if neg else rv[idx]) for (idx, neg) in omap]


def check(progs, iters=300, seed=1):
    rng = random.Random(seed)
    for prog in progs:
        name, n, cls, sched, omap = prog
        N = 1 << n
        copies = 64 // N
        good = 0
        while good < iters:
            r = rng.choice([16, 18, 20])
            amp = rng.choice([50, 500, 3000, 20000])
            rows = [[rng.randint(-amp, amp) for _ in range(N)] for _ in range(copies)]
            try:
                exp = []
                for row in rows:
                    T = list(row)
                    (m.inv_dct if cls == 0 else m.inv_adst)(T, n, r)
                    exp += T
            except m.Nonconformant:
                continue
            got = sim_program(prog, [x for row in rows for x in row], r)
            assert got == exp, f"{name} mismatch\n rows={rows}\n exp={exp}\n got={got}"
            good += 1
        print(f"{name:7s} x{copies:2d}: {len(sched)} levels, {sum(len(l) for l in sched):3d} ops, "
              f"{iters} random packed vectors match the model", file=sys.stderr)


# ---------------------------------------------------------------- SystemVerilog emission
def emit(progs, stats):
    o = []
    w = o.append
    w("// Generated by tools/gen_itx_lvl.py — do not edit. Level-folded 1D inverse transform engine: one spec")
    w("// level per clock on 32 fixed pair units; wiring per (program, level) generated from tb/itx_model.py.")
    w("// Programs: " + ", ".join(f"{i} {nm} x{64 >> n}" for i, (nm, n, _) in enumerate(PROGRAMS)))
    w(f"// Fan-in: unit input a <= {stats['fan_a']}, b <= {stats['fan_b']}, write-back per lane <= {stats['wb']}.")
    w("module itx_lvl #(")
    w("    parameter int TW = 20")
    w(") (")
    w("    input  logic             clk,")
    w("    input  logic             rst,")
    w("    input  logic             start,      // pulse while idle; in_vec valid in this cycle (level 0 reads it)")
    w("    input  logic [2:0]       prog,")
    w("    input  logic [4:0]       r,          // Hadamard clamp range (bits)")
    w("    input  logic [64*TW-1:0] in_vec,")
    w("    output logic             busy,")
    w("    output logic             done,       // one-cycle pulse; out_vec valid from this cycle on")
    w("    output logic [64*TW-1:0] out_vec")
    w(");")
    w(f"    localparam int NU = {NU};")
    w("    logic [TW-1:0] rv [64];   // the register vector")
    w("    logic [2:0]    p_l;        // latched program")
    w("    logic [3:0]    lvl_l;      // next level to execute while busy")
    w("    logic [4:0]    r_l;")
    w("    logic [2:0]    p;          // program / level executing this cycle (level 0 in the start cycle)")
    w("    logic [3:0]    lvl;")
    w("    logic [4:0]    rr;")
    w("    logic          step;")
    w("    assign p    = busy ? p_l : prog;")
    w("    assign lvl  = busy ? lvl_l : 4'd0;")
    w("    assign rr   = busy ? r_l : r;")
    w("    assign step = busy | start;")
    w("    logic [3:0] nlvl;")
    w("    always_comb case (p)")
    for i, (nm, n, cls, sched, omap) in enumerate(progs):
        w(f"        3'd{i}: nlvl = 4'd{len(sched)};")
    w("        default: nlvl = 4'd1;")
    w("    endcase")
    w("")
    w("    // level source: in_vec at level 0, the register vector after")
    w("    logic [TW-1:0] sel [64];")
    w("    always_comb for (int k = 0; k < 64; k++) sel[k] = (lvl == 4'd0) ? in_vec[k*TW +: TW] : rv[k];")
    w("")
    w("    // unit inputs and constants per (program, level): fixed wiring")
    w("    logic [TW-1:0]        ia [NU], ib [NU], oa [NU], ob [NU];")
    w("    logic signed [12:0]   uc [NU], us [NU];")
    w("    logic [1:0]           um [NU];")
    w("    logic                 uf [NU];")
    w("    always_comb begin")
    w("        for (int u = 0; u < NU; u++) begin ia[u] = '0; ib[u] = '0; uc[u] = '0; us[u] = '0; um[u] = 2'd0; uf[u] = 1'b0; end")
    w("        case ({p, lvl})")
    for i, (nm, n, cls, sched, omap) in enumerate(progs):
        for lv, rows in enumerate(sched):
            w(f"            {{3'd{i}, 4'd{lv}}}: begin")
            for (u, sa, sb, op, ang, fl, a, b) in rows:
                c = m.cos128(ang) if op == OP_B else 0
                s = m.sin128(ang) if op == OP_B else 0
                w(f"                ia[{u}] = sel[{sa}]; ib[{u}] = sel[{sb}]; um[{u}] = 2'd{op}; uf[{u}] = 1'b{fl};"
                  + (f" uc[{u}] = {'-' if c < 0 else ''}13'sd{abs(c)}; us[{u}] = {'-' if s < 0 else ''}13'sd{abs(s)};" if op == OP_B else ""))
            w("            end")
    w("            default: ;")
    w("        endcase")
    w("    end")
    w("")
    w("    logic signed [TW-1:0] clip_lo, clip_hi;")
    w("    assign clip_lo = -(TW'(1) <<< (rr - 5'd1));")
    w("    assign clip_hi =  (TW'(1) <<< (rr - 5'd1)) - TW'(1);")
    w("    genvar gu;")
    w("    generate for (gu = 0; gu < NU; gu++) begin : g_unit")
    w("        itx_pair #(.TW(TW)) u_pair (.ia(ia[gu]), .ib(ib[gu]), .c(uc[gu]), .s(us[gu]), .mode(um[gu]), .flip(uf[gu]),")
    w("                                    .clip_lo(clip_lo), .clip_hi(clip_hi), .oa(oa[gu]), .ob(ob[gu]));")
    w("    end endgenerate")
    w("")
    w("    // write-back per (program, level): touched lanes take their unit's output, the rest hold")
    w("    always_ff @(posedge clk) begin")
    w("        done <= 1'b0;")
    w("        if (rst) begin")
    w("            busy <= 1'b0;")
    w("            lvl_l <= 4'd0;")
    w("        end else if (step) begin")
    w("            if (!busy) begin p_l <= prog; r_l <= r; end")
    w("            if (lvl + 4'd1 == nlvl) begin busy <= 1'b0; done <= 1'b1; lvl_l <= 4'd0; end")
    w("            else begin busy <= 1'b1; lvl_l <= lvl + 4'd1; end")
    w("            case ({p, lvl})")
    for i, (nm, n, cls, sched, omap) in enumerate(progs):
        for lv, rows in enumerate(sched):
            w(f"                {{3'd{i}, 4'd{lv}}}: begin")
            for (u, sa, sb, op, ang, fl, a, b) in rows:
                w(f"                    rv[{a}] <= oa[{u}]; rv[{b}] <= ob[{u}];")
            w("                end")
    w("                default: ;")
    w("            endcase")
    w("        end")
    w("    end")
    w("")
    w("    // output map: identity for DCT, the ADST output permutation with its per-lane negation")
    w("    always_comb begin")
    w("        for (int k = 0; k < 64; k++) out_vec[k*TW +: TW] = rv[k];")
    w("        case (p_l)")
    for i, (nm, n, cls, sched, omap) in enumerate(progs):
        if cls == 0:
            continue
        w(f"            3'd{i}: begin")
        for k, (idx, neg) in enumerate(omap):
            if idx != k or neg:
                w(f"                out_vec[{k}*TW +: TW] = {'-' if neg else ''}rv[{idx}];")
        w("            end")
    w("            default: ;")
    w("        endcase")
    w("    end")
    w("endmodule")
    return "\n".join(o) + "\n"


def main():
    progs, stats = schedule()
    if "--check" in sys.argv:
        check(progs)
        print(f"fan-in: a {stats['fan_a']}, b {stats['fan_b']}, write-back {stats['wb']}", file=sys.stderr)
        return
    sys.stdout.write(emit(progs, stats))


if __name__ == "__main__":
    main()
