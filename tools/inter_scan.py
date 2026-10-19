#!/usr/bin/env python3
"""Which inter features a stream uses (per the model's syntax decode, no trace needed): to pick streams for the RTL
inter stages. Stage 2a covers single / AVERAGE / DISTANCE compound, SIMPLE motion, unscaled references, no
inter-intra, no warp. Usage: tools/inter_scan.py stream.obu [...] [--2a] (list only the stage-2a-clean ones)"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tb"))
import tile_model as tm   # noqa: E402
import seq_model as sm    # noqa: E402
import obu_parser as op   # noqa: E402


def scan(path):
    d = op.parse_file(path)
    seq = sm.SequenceState()
    hdr_iter = iter(d.frame_headers)
    feat = dict(frames=0, inter_frames=0, inter_blocks=0, compound=0, avg=0, dist=0, wedge=0, diffwtd=0, interintra=0,
                obmc=0, localwarp=0, globalwarp=0, scaled=0, intrabc=0, sub8_chroma=0, skip_mode=0, palette=0)
    for fi, tl in enumerate(d.frames):
        h = None
        for h in hdr_iter:
            if h.show_existing_frame:
                seq.show_existing(h)
                continue
            break
        if h is None:
            break
        ctx = seq.start_frame(h)
        decs = []
        feat["frames"] += 1
        if not h.FrameIsIntra:
            feat["inter_frames"] += 1
            feat["scaled"] += sum(1 for i in range(7) if h.__dict__.get("ref_scaled", [0] * 7)[i]) if hasattr(h, "ref_scaled") else 0
        for k, ti in enumerate(tl):
            th, data = d.tiles[ti]
            dec = tm.TileDecoder(th, data, None, frame=ctx)
            dec.decode_tile()
            for b in dec.blocks:
                if b.get("intrabc"):
                    feat["intrabc"] += 1
                if b["pal"] != (0, 0):
                    feat["palette"] += 1
                if not b.get("inter") or b.get("intrabc"):
                    continue
                feat["inter_blocks"] += 1
                comp = b["ref"][1] > 0
                if comp:
                    feat["compound"] += 1
                    ct = b.get("compound_type")
                    if ct == 2: feat["avg"] += 1
                    elif ct == 4: feat["dist"] += 1
                    elif ct == 0: feat["wedge"] += 1
                    elif ct == 1: feat["diffwtd"] += 1
                if b.get("interintra"):
                    feat["interintra"] += 1
                mm = b.get("motion_mode", 0)
                if mm == 1: feat["obmc"] += 1
                if mm == 2: feat["localwarp"] += 1
                if b.get("skip_mode"):
                    feat["skip_mode"] += 1
                if b.get("cmv"):
                    feat["sub8_chroma"] += 1
                # global warp: GLOBALMV family with a non-translational gm and a block >= 8x8
                ym = b["ymode"]
                if ym in (tm.GLOBALMV, tm.GLOBAL_GLOBALMV) and not th.force_integer_mv:
                    bw = tm.T.Block_Width[b["size"]]; bh = tm.T.Block_Height[b["size"]]
                    if bw >= 8 and bh >= 8:
                        for r in b["ref"][:2 if comp else 1]:
                            if r > 0 and th.GmType[r] > tm.TRANSLATION:
                                feat["globalwarp"] += 1
            decs.append(sm.TileSummary(dec, keep_cdf=(k == h.context_update_tile_id)))
            del dec
        seq.end_frame(h, decs, frame_cdfs=ctx.cdfs)
    feat["stage2a"] = int(feat["inter_blocks"] > 0 and not (feat["wedge"] or feat["diffwtd"] or feat["interintra"] or feat["obmc"]
                                                           or feat["localwarp"] or feat["globalwarp"] or feat["scaled"]))
    return feat


def main():
    only2a = "--2a" in sys.argv
    for path in [a for a in sys.argv[1:] if not a.startswith("-")]:
        try:
            f = scan(path)
        except Exception as e:  # noqa: BLE001
            print(f"{Path(path).name}: ERROR {e}")
            continue
        if only2a and not f["stage2a"]:
            continue
        used = ", ".join(f"{k} {v}" for k, v in f.items() if v and k not in ("frames", "stage2a"))
        print(f"{Path(path).name}: {'2a' if f['stage2a'] else '--'} frames {f['frames']}: {used}")


if __name__ == "__main__":
    main()
