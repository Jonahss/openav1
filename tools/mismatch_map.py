# tools/mismatch_map.py <rtl.yuv> <model.yuv> <dump.txt> <cases.json>: mismatch map of a frame dumped by TS_DUMP (16-bit LE
# planar as test_dec_top writes it), per-block mismatch counts joined with the block features from tools/mc_cases.py,
# the rtl-model delta histogram, and a few hypothesis checks on the mismatching blocks. Found test10505 (2026-10-22):
# every block whose reference window reached row 1024 mismatched = the testbench's 16 MB reference planes aliasing.
import sys, json, collections, struct
rtl_p, mod_p, txt_p, cases_p = sys.argv[1:5]
W, H, bd, ssx, ssy, nplanes = map(int, open(txt_p).read().split()[:6])
bps = 2 if bd > 8 else 1
def load(p):
    data = open(p, "rb").read(); planes = []; off = 0
    for pl in range(nplanes):
        w = (W + ssx) >> ssx if pl else W; h = (H + ssy) >> ssy if pl else H
        n = w * h; vals = struct.unpack_from(f"<{n}{'H' if bps == 2 else 'B'}", data, off); off += n * bps
        planes.append([vals[r * w:(r + 1) * w] for r in range(h)])
    return planes
rtl, mod = load(rtl_p), load(mod_p)
cases = json.load(open(cases_p))
by_pos = {(c["block"]["c"] * 4, c["block"]["r"] * 4): c for c in cases}
# luma mismatch map
bad = [(y, x, rtl[0][y][x], mod[0][y][x]) for y in range(H) for x in range(W) if rtl[0][y][x] != mod[0][y][x]]
hist = collections.Counter(r - m for (_, _, r, m) in bad)
print(f"luma: {len(bad)} / {W * H} mismatches; rtl-model histogram {sorted(hist.items(), key=lambda kv: -kv[1])[:8]}")
if len(bad) == 0: sys.exit(0)
ys = [b[0] for b in bad]; xs = [b[1] for b in bad]
print(f"  rows {min(ys)}..{max(ys)}, cols {min(xs)}..{max(xs)}")
# per block
blk_bad = collections.Counter(); blk_px = {}
for c in cases:
    b = c["block"]; x0, y0, w, h = b["c"] * 4, b["r"] * 4, b["w"], b["h"]
    blk_px[(x0, y0)] = max(0, min(W, x0 + w) - x0) * max(0, min(H, y0 + h) - y0)
for (y, x, _, _) in bad:
    for (x0, y0), c in by_pos.items():
        if x0 <= x < x0 + c["block"]["w"] and y0 <= y < y0 + c["block"]["h"]:
            blk_bad[(x0, y0)] += 1; break
feat = collections.Counter(); feat_px = collections.Counter()
def window_class(c):
    w, h = c["block"]["w"], c["block"]["h"]
    xf = (c["startX"] >> 10) - 3; xl = ((c["startX"] + c["xs"] * (w - 1)) >> 10) + 4
    yf = (c["startY"] >> 10) - 3; yl = ((c["startY"] + c["ys"] * (h - 1)) >> 10) + 4
    lx, ly = c["refW"] - 1, c["refH"] - 1
    cx = "x-allright" if xf > lx else "x-allleft" if xl < 0 else "x-crossR" if xl > lx else "x-crossL" if xf < 0 else "x-in"
    cy = "y-allbot" if yf > ly else "y-alltop" if yl < 0 else "y-crossB" if yl > ly else "y-crossT" if yf < 0 else "y-in"
    return cx, cy
for pos, c in by_pos.items():
    cls = window_class(c); ii = "ii" if c["block"].get("interintra") else "-"
    mv = c["block"].get("mv") or [0, 0]
    key = (cls[0], cls[1], ii, "ymode%d" % c["block"]["ymode"], "gm%d" % c["block"].get("gmtype", -1), "mv8th" if (mv[0] % 2 or mv[1] % 2) else "mv4th", "%dx%d" % (c["block"]["w"], c["block"]["h"]))
    feat_px[key] += blk_px[pos]
    if blk_bad.get(pos): feat[key] += blk_bad[pos]
print("mismatching px by block feature (bad / total px of blocks with that feature):")
for key, n in sorted(feat.items(), key=lambda kv: -kv[1])[:20]:
    print(f"  {n:8d} / {feat_px[key]:8d}  {' '.join(key)}")
print("features with ZERO mismatches (largest first):")
for key, px in sorted(feat_px.items(), key=lambda kv: -kv[1]):
    if feat.get(key, 0) == 0: print(f"  {px:8d} px  {' '.join(key)}")
nb = sum(1 for p in by_pos if blk_bad.get(p)); print(f"{nb} of {len(by_pos)} blocks mismatch")
# the worst blocks
for pos, n in blk_bad.most_common(6):
    c = by_pos[pos]; print(f"  block {pos} {c['block']['w']}x{c['block']['h']} ii={c['block'].get('interintra')} ymode={c['block']['ymode']} start=({c['startX'] >> 10},{c['startY'] >> 10}) win={window_class(c)}: {n}/{blk_px[pos]} bad")
