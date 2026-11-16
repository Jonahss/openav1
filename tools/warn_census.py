#!/usr/bin/env python3
"""Census of every warning a U200 image build printed, by owner and message ID, checked against the waiver list.

    tools/warn_census.py <build dir> [--waivers docs/warnings/waivers.txt] [--md out.md] [--gate]

<build dir> is ~/openav1-synth/u200-<tag>: vivado.log, proj/u200.runs/<run>/runme.log, the methodology report
(impl_methodology.rpt, written by fpga/u200/build.tcl) and the routed DRC report of impl_1.

Owner of a message = the run that printed it:
    decoder  sys_host_0_synth_1, sys_pg_0_synth_1, sys_clkgen_0_synth_1 (our RTL, synthesised out of context)
    shell    synth_1, impl_1, the top-level vivado.log, methodology, DRC (our block design, constraints, flow)
    vendor   every other sys_*_synth_1 (AMD's XDMA, DDR4, SmartConnect, reset blocks: their RTL, not ours)

A waiver is one line of the waiver file:   <owner|*>  <ID>  <regex on the message, or *>  # reason, link into the report
With --gate the exit status is 1 when a warning has no waiver, or when Vivado cut a message kind off at its limit
(then the census is not complete and proves nothing).
"""
import argparse, collections, glob, os, re, sys

OURS = {"sys_host_0_synth_1", "sys_pg_0_synth_1", "sys_clkgen_0_synth_1"}
SHELL = {"synth_1", "impl_1"}
MSG = re.compile(r"^(CRITICAL WARNING|WARNING|ERROR): \[([A-Za-z_]+ \d+-\d+)\] (.*)")
LIMIT = re.compile(r"Message '([A-Za-z_]+ \d+-\d+)' appears (\d+) times and further instances")
METH = re.compile(r"^([A-Z]+-\d+)#\d+ (Warning|Critical Warning|Advisory|Error)")
DRCL = re.compile(r"^([A-Z]+-\d+)#\d+ (Warning|Critical Warning|Advisory|Error)")


VENDOR_TEXT = re.compile(r"sys_i/(ddr|xdma|sc_\w+|jtag|rst_\w+|refbuf)\b|dbg_hub|ddr4_|xdma|smartconnect|xpm_|microblaze|bd_[0-9a-f]{4}|/opt/Xilinx|sys_(ddr|xdma|sc_\w+|jtag|refbuf|rst_\w+)_0|gtwizard|pcie|axidma|lmb_|MIG|DCIRESET")
OURS_TEXT = re.compile(r"sys_i/(host|pg|clkgen)/|\[rtl/|(^|[ /\[])rtl/\w+\.sv")


def owner_of(run, text=""):
    """The run decides; messages of the top-level runs and reports are sorted by what they talk about."""
    if run in OURS: return "decoder"
    if run in SHELL or run in ("top", "methodology", "drc"):
        if OURS_TEXT.search(text): return "decoder"
        if VENDOR_TEXT.search(text): return "vendor"
        return "shell"
    return "vendor"


def squash(msg):
    """The message with paths, numbers and instance names flattened, for showing one sample per kind."""
    msg = re.sub(r"\[/[^\]]*/(rtl|fpga|synth)/", r"[\1/", msg)
    msg = re.sub(r"/home/\S+/", "", msg)
    return msg[:200]


def read(build):
    """-> list of (run, severity, id, message), and {(run, id): limit} for the kinds Vivado cut off."""
    out, cut = [], {}
    logs = [("top", os.path.join(build, "vivado.log"))]
    for p in sorted(glob.glob(os.path.join(build, "proj", "*.runs", "*", "runme.log"))):
        logs.append((os.path.basename(os.path.dirname(p)), p))
    for run, path in logs:
        if not os.path.exists(path): continue
        for line in open(path, errors="replace"):
            m = MSG.match(line)
            if m: out.append((run, m.group(1), m.group(2), m.group(3).rstrip())); continue
            m = LIMIT.search(line)
            if m: cut[(run, m.group(1))] = int(m.group(2))
    seen = {(sev, mid, text) for run, sev, mid, text in out if run != "top"}
    out = [m for m in out if m[0] != "top" or (m[1], m[2], m[3]) not in seen]
    for run, pats, rx in (("methodology", ["impl_methodology.rpt", "proj/*.runs/impl_1/*methodology_drc_routed.rpt"], METH),
                          ("drc", ["proj/*.runs/impl_1/*_drc_routed.rpt"], DRCL)):
        paths = [p for pat in pats for p in glob.glob(os.path.join(build, pat)) if "methodology" in p or run == "drc"]
        if run == "drc": paths = [p for p in paths if "methodology" not in p]
        if not paths: out.append((run, "MISSING", "report-missing", "no %s report in the build directory" % run)); continue
        lines = open(paths[0], errors="replace").read().split("\n")
        for i, line in enumerate(lines):
            m = rx.match(line)
            if m:
                text = " ".join(l.strip() for l in lines[i + 1:i + 3])
                out.append((run, m.group(2).upper(), m.group(1), text))
    return out, cut


def load_waivers(path):
    ws = []
    if not path or not os.path.exists(path): return ws
    for n, line in enumerate(open(path), 1):
        body, _, reason = line.partition("#")
        f = body.split(None, 2)
        if len(f) < 2: continue
        # the ID is two words for Vivado messages ("Synth 8-6014"), one for report checks ("TIMING-16")
        m = re.match(r"(\S+)\s+((?:[A-Za-z_]+ \d+-\d+)|(?:[A-Z]+-\d+)|report-missing)\s*(.*)", body.strip())
        if not m: sys.exit("%s:%d: cannot read waiver line" % (path, n))
        rx = m.group(3).strip() or "*"
        if not reason.strip(): sys.exit("%s:%d: a waiver needs a reason after #" % (path, n))
        ws.append((m.group(1), m.group(2), None if rx == "*" else re.compile(rx), reason.strip(), n))
    return ws


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("build"); ap.add_argument("--waivers"); ap.add_argument("--md"); ap.add_argument("--gate", action="store_true")
    a = ap.parse_args()
    msgs, cut = read(a.build)
    waivers = load_waivers(a.waivers)
    used = collections.Counter()
    kinds = collections.OrderedDict()   # (owner, severity, id) -> [count, waived, samples, runs, reasons]
    for run, sev, mid, text in msgs:
        own = owner_of(run, squash(text))
        k = (own, sev, mid)
        e = kinds.setdefault(k, [0, 0, [], set(), set()])
        e[0] += 1; e[3].add(run)
        w = next((w for w in waivers if w[0] in ("*", own) and w[1] == mid and (w[2] is None or w[2].search(text))), None)
        if w: e[1] += 1; e[4].add(w[3]); used[w[4]] += 1
        elif len(e[2]) < 3:
            s = squash(text)
            if s not in e[2]: e[2].append(s)
    rank = {"ERROR": 0, "CRITICAL WARNING": 1, "MISSING": 1, "WARNING": 2, "ADVISORY": 3}
    order = sorted(kinds, key=lambda k: (["decoder", "shell", "vendor"].index(k[0]), rank.get(k[1], 9), -kinds[k][0]))
    tot = collections.Counter(); open_ = collections.Counter()
    lines = ["# Warning census: %s" % os.path.basename(os.path.normpath(a.build)), ""]
    cutkinds = sorted({(r, i) for (r, i) in cut})
    for own in ("decoder", "shell", "vendor"):
        rows = [k for k in order if k[0] == own]
        if not rows: continue
        lines += ["## %s" % own, "", "| ID | severity | printed | waived | open | cut off | first open sample / waiver reason |", "|---|---|---:|---:|---:|---|---|"]
        for k in rows:
            n, wv, samples, runs, reasons = kinds[k]
            tot[own] += n; open_[own] += n - wv
            c = "yes" if any((r, k[2]) in cut for r in runs) else ""
            note = samples[0] if samples else "; ".join(sorted(reasons))[:200]
            lines.append("| %s | %s | %d | %d | %d | %s | %s |" % (k[2], k[1].lower(), n, wv, n - wv, c, note.replace("|", "\\|")))
        lines.append("")
    summ = ["%s: %d printed, %d open" % (o, tot[o], open_[o]) for o in ("decoder", "shell", "vendor") if tot[o]]
    lines[1:1] = ["", "; ".join(summ) + ". Kinds Vivado cut off at its message limit (counts are lower bounds): %d." % len(cutkinds), ""]
    unused = [w for w in waivers if not used[w[4]]]
    if unused:
        lines += ["## waivers that matched nothing", ""] + ["- line %d: %s %s" % (w[4], w[0], w[1]) for w in unused] + [""]
    text = "\n".join(lines)
    if a.md: open(a.md, "w").write(text + "\n")
    else: print(text)
    nopen = sum(open_.values())
    print("WARN_CENSUS %s: %s; cut-off kinds %d; unused waivers %d" % (os.path.basename(os.path.normpath(a.build)), "; ".join(summ), len(cutkinds), len(unused)), file=sys.stderr)
    if a.gate and (nopen or cutkinds):
        print("WARN_CENSUS_FAILED: %d warnings without a waiver, %d kinds cut off" % (nopen, len(cutkinds)), file=sys.stderr); sys.exit(1)


if __name__ == "__main__":
    main()
