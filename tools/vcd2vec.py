#!/usr/bin/env python3
"""Turn a VCD of a block-level simulation into per-cycle vectors for the gate-level replay testbench.

  tools/vcd2vec.py <dump.vcd> <scope> <clk> --in a,b,c --out x,y,z > vectors.hex

Samples every named signal just before each rising edge of <clk> (i.e. the values the DUT saw at that edge for
the inputs, and the values it was driving before the edge for the outputs). One line per cycle: the inputs
concatenated MSB-first in the order given, a space, then the outputs likewise, as hex. The widths come from the
VCD and are printed to stderr so the testbench can be checked against them. <scope> is the hierarchical scope
holding the DUT ports (e.g. TOP.ipred); signals are matched by name directly under it."""
import sys, argparse

ap = argparse.ArgumentParser()
ap.add_argument('vcd'); ap.add_argument('scope'); ap.add_argument('clk')
ap.add_argument('--in', dest='ins', required=True); ap.add_argument('--out', dest='outs', required=True)
a = ap.parse_args()
ins = a.ins.split(','); outs = a.outs.split(',')
want = set(ins + outs + [a.clk])

code = {}       # id -> name
width = {}
scope = []
per_scope = {}  # scope path -> {name: (ident, width)}
with open(a.vcd) as f:
    # header
    for line in f:
        t = line.split()
        if not t: continue
        if t[0] == '$scope': scope.append(t[2])
        elif t[0] == '$upscope': scope.pop()
        elif t[0] == '$var':
            w, ident, name = int(t[2]), t[3], t[4]
            if name in want:
                per_scope.setdefault('.'.join(scope), {}).setdefault(name, (ident, w))
        elif t[0] == '$enddefinitions': break
    # the requested scope, or else the first scope that holds every wanted signal (Verilator puts the top's
    # ports under the root scope in some versions and under the module in others)
    candidates = [a.scope] + [sc for sc in per_scope if sc != a.scope]
    chosen = next((sc for sc in candidates if sc in per_scope and want <= set(per_scope[sc])), None)
    if chosen is None:
        best = max(per_scope, key=lambda sc: len(per_scope[sc]), default=None)
        sys.exit(f"no scope holds all signals; best is {best} missing {sorted(want - set(per_scope.get(best, {})))}")
    if chosen != a.scope: print(f"(signals taken from scope {chosen})", file=sys.stderr)
    for name, (ident, w) in per_scope[chosen].items():
        code[ident] = name; width[name] = w
    for n in ins + outs: print(f"{n} {width[n]}", file=sys.stderr)
    val = {n: 0 for n in width}
    clk_id = next(i for i, n in code.items() if n == a.clk)
    prev_clk = 0
    pending = {}          # changes in the current timestep, applied together
    def flush():
        global prev_clk
        if not pending: return
        new_clk = pending.get(a.clk, val[a.clk])
        if new_clk == 1 and prev_clk == 0:
            # sample the pre-edge values (before applying this timestep's changes)
            vi = 0
            for n in ins: vi = (vi << width[n]) | val[n]
            vo = 0
            for n in outs: vo = (vo << width[n]) | val[n]
            wi = sum(width[n] for n in ins); wo = sum(width[n] for n in outs)
            print(f"{vi:0{(wi + 3) // 4}x} {vo:0{(wo + 3) // 4}x}")
        val.update(pending); prev_clk = val[a.clk]; pending.clear()
    for line in f:
        if not line.strip(): continue
        c = line[0]
        if c == '#': flush(); continue
        if c in '01xz':
            ident = line[1:].strip()
            if ident in code: pending[code[ident]] = 1 if c == '1' else 0
        elif c == 'b':
            bits, ident = line[1:].split()
            if ident in code: pending[code[ident]] = int(bits.replace('x', '0').replace('z', '0'), 2)
    flush()
