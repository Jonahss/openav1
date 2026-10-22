# tools/mc_cases.py <obu> <frame> <out.json>: every single-ref inter luma block of frame F as a unit case for
# tb/test_mc_case.py (MC_CASE=<out.json>): reference patch (edge-replicated, from the recon-stage upscaled reference), start, steps,
# filters, bit depth, and the model's recon of the block.
import os, sys, json, re, io, contextlib
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tb")); os.environ.setdefault("TS_STAGE", "recon")
import obu_parser as op, seq_model as sm, test_dec_top as TD, av1_tables as T, recon_model as rm, superres_model as srm
path, F, out = sys.argv[1], int(sys.argv[2]), sys.argv[3]
d = op.Decoder(); d.feed_annexb(open(path, "rb").read())
seq = sm.SequenceState(); store = {}; it = iter(d.frame_headers)
clip = lambda lo, hi, v: lo if v < lo else hi if v > hi else v
for fi, tile_idx in enumerate(d.frames):
    for ph in it:
        if ph.show_existing_frame:
            seq.show_existing(ph); continue
        break
    ctx = seq.start_frame(ph); th0 = d.tiles[tile_idx[0]][0]
    frame = {"refs": store}; summaries = []; blocks = []
    rm.MC_TRACE = (fi == F); buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        for k, ti in enumerate(tile_idx):
            th, data = d.tiles[ti]
            dec = TD.RecFrame(th, data, frame, ctx=ctx); dec.decode_tile()
            summaries.append(sm.TileSummary(dec, keep_cdf=(k == ph.context_update_tile_id)))
            if fi == F: blocks.extend(dec.blocks)
    if fi == F:
        lines = {}
        for l in buf.getvalue().splitlines():
            m = re.search(r"^MODEL MC pl0 l0 at \((\d+),(\d+)\) (\d+)x(\d+) ref (\d+) warp (\d+) start (-?\d+),(-?\d+) steps (\d+),(\d+)", l)
            if m:
                mm = re.search(r"mv \[(-?\d+), (-?\d+)\]", l)
                lines[(int(m.group(1)), int(m.group(2)))] = tuple(map(int, m.groups()[4:])) + ((int(mm.group(1)), int(mm.group(2))) if mm else (None, None))
        cases = []
        for b in blocks:
            if not b.get("inter"): continue
            bw, bh = T.Block_Width[b["size"]], T.Block_Height[b["size"]]; x0, y0 = b["c"] * 4, b["r"] * 4
            if (x0, y0) not in lines: continue
            ref, warp, sx, sy, xs, ys, mvr, mvc = lines[(x0, y0)]
            st = store[ph.ref_frame_idx[ref - 1]]; refp = st["planes"][0]; refW, refH = st["upscaled_width"], st["frame_height"]
            ix, fx, iy, fy = sx >> 10, sx & 1023, sy >> 10, sy & 1023
            cols = (((bw - 1) * xs + 1023) >> 10) + 8; rows = (((bh - 1) * ys + 1023) >> 10) + 8
            patch = [[refp[clip(0, refH - 1, iy - 3 + r)][clip(0, refW - 1, ix - 3 + c)] for c in range(cols)] for r in range(rows)]
            pred = [frame["planes"][0][y0 + r][x0:x0 + bw] for r in range(bh)]
            cases.append(dict(block=dict(r=b["r"], c=b["c"], w=bw, h=bh, ymode=b["ymode"], interintra=b.get("interintra"), interp=list(b.get("interp") or (0, 0)), skip=b["skip"], mv=[mvr, mvc], ref=list(b.get("ref") or ()), gmtype=ph.GmType[ref], ii_mode=b.get("interintra_mode"), wedge_ii=b.get("wedge_interintra")),
                              bd=th0.BitDepth, ref=ref, refW=refW, refH=refH, startX=sx, startY=sy, xs=xs, ys=ys, ix=ix, fx=fx, iy=iy, fy=fy, patch=patch, pred=pred))
        json.dump(cases, open(out, "w")); print("wrote", out, len(cases), "cases; ref", refW, refH, "bd", th0.BitDepth)
        break
    seq.end_frame(ph, summaries, frame_cdfs=ctx.cdfs)
    planes = srm.upscale(th0, frame["planes"]) if ph.use_superres else frame["planes"]
    for i in range(8):
        if (ph.refresh_frame_flags >> i) & 1:
            store[i] = dict(planes=planes, upscaled_width=ph.UpscaledWidth, frame_height=ph.FrameHeight, hdr=th0)
