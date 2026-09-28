"""Whole-frame intra reconstruction model: tile syntax (tile_model) + intra prediction (intra_model)
+ inverse transforms (itx_model) + the spec's edge preparation and availability rules (7.11.2.1),
palette prediction (7.11.4) and CfL (7.11.5), reconstructing into CurrFrame (pre loop filter).

Each predicted block and each reconstructed block is recorded so a harness can compare them with
dav1d's P / Q / R trace events.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import av1_tables as T          # noqa: E402
import tile_model as tm         # noqa: E402
import intra_model as im        # noqa: E402
import itx_model as xm          # noqa: E402

REF_SCALE_SHIFT, SUBPEL_BITS, SCALE_SUBPEL_BITS, SUBPEL_MASK = 14, 4, 10, 15
MI_SIZE = tm.MI_SIZE

FLIP_UD = {xm.FLIPADST_DCT, xm.FLIPADST_ADST, xm.V_FLIPADST, xm.FLIPADST_FLIPADST}
FLIP_LR = {xm.DCT_FLIPADST, xm.ADST_FLIPADST, xm.H_FLIPADST, xm.FLIPADST_FLIPADST}
SMOOTH_MODES = (im.SMOOTH_PRED, im.SMOOTH_V_PRED, im.SMOOTH_H_PRED)


class FrameRecon(tm.TileDecoder):
    def __init__(self, hdr, data, frame, hook=None):
        """frame: shared dict with 'planes' (list of 2D pixel arrays, one per plane) so that several
        tiles of one frame reconstruct into the same buffer."""
        super().__init__(hdr, data, hook)
        self.frame = frame
        h = hdr
        if "planes" not in frame:
            frame["planes"] = []
            for plane in range(h.NumPlanes):
                subX = h.subsampling_x if plane > 0 else 0
                subY = h.subsampling_y if plane > 0 else 0
                # blocks may extend past the MI-aligned frame size (odd sizes, big blocks at the edge);
                # CurrFrame is conceptually unbounded there, so allocate a margin of one 128x128 superblock
                W = ((h.MiCols * 4) >> subX) + 128
                H = ((h.MiRows * 4) >> subY) + 128
                frame["planes"].append([[0] * W for _ in range(H)])
        self.planes = frame["planes"]
        self.pred_events = []     # ("P"/"Q", plane, x4, y4, w, h, pixels)
        self.recon_events = []    # ("R", plane, x4, y4, w, h, pixels)
        self.BlockDecoded = None
        self.MaxLumaW = self.MaxLumaH = 0

    # ---- 5.11.3 clear_block_decoded_flags ------------------------------------------------------
    def clear_block_decoded_flags(self, r, c, sbSize4):
        h = self.h
        self.BlockDecoded = []
        for plane in range(h.NumPlanes):
            subX = h.subsampling_x if plane > 0 else 0
            subY = h.subsampling_y if plane > 0 else 0
            sbWidth4 = (h.MiColEnd - c) >> subX
            sbHeight4 = (h.MiRowEnd - r) >> subY
            n = (sbSize4 >> min(subX, subY)) + 3
            bd = {}
            for y in range(-1, (sbSize4 >> subY) + 1):
                for x in range(-1, (sbSize4 >> subX) + 1):
                    if y < 0 and x < sbWidth4:
                        bd[(y, x)] = 1
                    elif x < 0 and y < sbHeight4:
                        bd[(y, x)] = 1
                    else:
                        bd[(y, x)] = 0
            bd[((sbSize4 >> subY), -1)] = 0
            self.BlockDecoded.append(bd)

    # ---- prediction --------------------------------------------------------------------------------
    # ---- 7.11.1 compute_prediction: inter blocks (intra block copy) are predicted whole, before the residual
    def compute_prediction(self):
        if not self.is_inter:
            return
        h = self.h
        for plane in range(1 + self.HasChroma * 2):
            planeSz = self.get_plane_residual_size(self.MiSize, plane)
            num4x4W = T.Num_4x4_Blocks_Wide[planeSz]
            num4x4H = T.Num_4x4_Blocks_High[planeSz]
            subX = h.subsampling_x if plane > 0 else 0
            subY = h.subsampling_y if plane > 0 else 0
            baseX = (self.MiCol >> subX) * MI_SIZE
            baseY = (self.MiRow >> subY) * MI_SIZE
            candRow = (self.MiRow >> subY) << subY
            candCol = (self.MiCol >> subX) << subX
            assert self.RefFrame[1] != tm.INTRA_FRAME, "interintra is not modelled"
            predW = T.Block_Width[self.MiSize] >> subX
            predH = T.Block_Height[self.MiSize] >> subY
            someUseIntra = 0
            for r in range(num4x4H << subY):
                for c in range(num4x4W << subX):
                    rr, cc = candRow + r, candCol + c
                    if rr < h.MiRows and cc < h.MiCols and self.RefFrames[rr][cc] is not None \
                            and self.RefFrames[rr][cc][0] == tm.INTRA_FRAME:
                        someUseIntra = 1
            if someUseIntra:
                predW = num4x4W * 4
                predH = num4x4H * 4
                candRow = self.MiRow
                candCol = self.MiCol
            r = 0
            for y in range(0, num4x4H * 4, predH):
                c = 0
                for x in range(0, num4x4W * 4, predW):
                    self.predict_inter(plane, baseX + x, baseY + y, predW, predH, candRow + r, candCol + c)
                    c += 1
                r += 1

    # ---- 7.11.3 inter prediction (the intra block copy subset: one reference = the current frame, no
    # scaling, no warp, no compound, no masks, no OBMC)
    def predict_inter(self, plane, x, y, w, hh, candRow, candCol):
        h = self.h
        assert self.use_intrabc, "inter frames are not modelled"
        isCompound = self.RefFrames[candRow][candCol][1] > tm.INTRA_FRAME
        # 7.11.3.2 rounding variables
        InterRound0 = 3
        InterRound1 = 7 if isCompound else 11
        if h.BitDepth == 12:
            InterRound0 += 2
        if h.BitDepth == 12 and not isCompound:
            InterRound1 -= 2
        mv = self.Mvs[candRow][candCol][0]
        # refIdx = -1: the reference is the current (pre loop filter) frame, first with the frame's size (so the
        # scaling process has no effect), then with the size rounded up to whole 4x4 units for the clamping
        startX, startY, stepX, stepY = self.motion_vector_scaling(plane, x, y, mv, h.UpscaledWidth, h.FrameHeight)
        refUpscaledWidth = h.MiCols * MI_SIZE
        refFrameHeight = h.MiRows * MI_SIZE
        pred = self.block_inter_prediction(plane, self.planes, startX, startY, stepX, stepY, w, hh, candRow, candCol,
                                           refUpscaledWidth, refFrameHeight, InterRound0, InterRound1)
        buf = self.planes[plane]
        mx = (1 << h.BitDepth) - 1
        for i in range(hh):
            row = buf[y + i]
            for j in range(w):
                v = pred[i][j]
                row[x + j] = 0 if v < 0 else mx if v > mx else v

    def motion_vector_scaling(self, plane, x, y, mv, refUpscaledWidth, refFrameHeight):
        h = self.h
        xScale = ((refUpscaledWidth << REF_SCALE_SHIFT) + (h.FrameWidth // 2)) // h.FrameWidth
        yScale = ((refFrameHeight << REF_SCALE_SHIFT) + (h.FrameHeight // 2)) // h.FrameHeight
        subX = h.subsampling_x if plane > 0 else 0
        subY = h.subsampling_y if plane > 0 else 0
        halfSample = 1 << (SUBPEL_BITS - 1)
        origX = (x << SUBPEL_BITS) + ((2 * mv[1]) >> subX) + halfSample
        origY = (y << SUBPEL_BITS) + ((2 * mv[0]) >> subY) + halfSample
        baseX = origX * xScale - (halfSample << REF_SCALE_SHIFT)
        baseY = origY * yScale - (halfSample << REF_SCALE_SHIFT)
        off = (1 << (SCALE_SUBPEL_BITS - SUBPEL_BITS)) // 2
        startX = im.round2signed(baseX, REF_SCALE_SHIFT + SUBPEL_BITS - SCALE_SUBPEL_BITS) + off
        startY = im.round2signed(baseY, REF_SCALE_SHIFT + SUBPEL_BITS - SCALE_SUBPEL_BITS) + off
        stepX = im.round2signed(xScale, REF_SCALE_SHIFT - SCALE_SUBPEL_BITS)
        stepY = im.round2signed(yScale, REF_SCALE_SHIFT - SCALE_SUBPEL_BITS)
        return startX, startY, stepX, stepY

    def block_inter_prediction(self, plane, ref, x, y, xStep, yStep, w, hh, candRow, candCol,
                               refUpscaledWidth, refFrameHeight, InterRound0, InterRound1):
        h = self.h
        subX = h.subsampling_x if plane > 0 else 0
        subY = h.subsampling_y if plane > 0 else 0
        lastX = ((refUpscaledWidth + subX) >> subX) - 1
        lastY = ((refFrameHeight + subY) >> subY) - 1
        intermediateHeight = (((hh - 1) * yStep + (1 << SCALE_SUBPEL_BITS) - 1) >> SCALE_SUBPEL_BITS) + 8
        F = T.Subpel_Filters
        refp = ref[plane]
        interpFilter = self.InterpFilters[candRow][candCol][1]
        if w <= 4:
            if interpFilter in (tm.EIGHTTAP, tm.EIGHTTAP_SHARP):
                interpFilter = 4
            elif interpFilter == tm.EIGHTTAP_SMOOTH:
                interpFilter = 5
        intermediate = []
        for r in range(intermediateHeight):
            srow = refp[im.clip3(0, lastY, (y >> 10) + r - 3)]
            orow = []
            for c in range(w):
                p = x + xStep * c
                taps = F[interpFilter][(p >> 6) & SUBPEL_MASK]
                s = 0
                for t in range(8):
                    s += taps[t] * srow[im.clip3(0, lastX, (p >> 10) + t - 3)]
                orow.append(im.round2(s, InterRound0))
            intermediate.append(orow)
        interpFilter = self.InterpFilters[candRow][candCol][0]
        if hh <= 4:
            if interpFilter in (tm.EIGHTTAP, tm.EIGHTTAP_SHARP):
                interpFilter = 4
            elif interpFilter == tm.EIGHTTAP_SMOOTH:
                interpFilter = 5
        pred = []
        for r in range(hh):
            p = (y & 1023) + yStep * r
            taps = F[interpFilter][(p >> 6) & SUBPEL_MASK]
            base = p >> 10
            orow = []
            for c in range(w):
                s = 0
                for t in range(8):
                    s += taps[t] * intermediate[base + t][c]
                orow.append(im.round2(s, InterRound1))
            pred.append(orow)
        return pred

    def predict_block(self, plane, startX, startY, txSz, x, y, subX, subY, sbMiRow, sbMiCol, stepX, stepY):
        h = self.h
        if self.is_inter:
            return      # predicted whole by compute_prediction
        if (plane == 0 and self.PaletteSizeY) or (plane != 0 and self.PaletteSizeUV):
            self.predict_palette(plane, startX, startY, x, y, txSz)
        else:
            isCfl = plane > 0 and self.UVMode == tm.UV_CFL_PRED
            mode = self.YMode if plane == 0 else (im.DC_PRED if isCfl else self.UVMode)
            log2W = T.Tx_Width_Log2[txSz]
            log2H = T.Tx_Height_Log2[txSz]
            haveLeft = (self.AvailL if plane == 0 else self.AvailLChroma) or x > 0
            haveAbove = (self.AvailU if plane == 0 else self.AvailUChroma) or y > 0
            bd = self.BlockDecoded[plane]
            haveAboveRight = bd.get(((sbMiRow >> subY) - 1, (sbMiCol >> subX) + stepX), 0)
            haveBelowLeft = bd.get(((sbMiRow >> subY) + stepY, (sbMiCol >> subX) - 1), 0)
            pred = self.predict_intra(plane, startX, startY, bool(haveLeft), bool(haveAbove), bool(haveAboveRight),
                                      bool(haveBelowLeft), mode, log2W, log2H)
            w, hh = 1 << log2W, 1 << log2H
            buf = self.planes[plane]
            for i in range(hh):
                row = buf[startY + i]
                for j in range(w):
                    row[startX + j] = pred[i][j]
            if isCfl:
                alpha = self.CflAlphaU if plane == 1 else self.CflAlphaV
                self.predict_chroma_from_luma(plane, startX, startY, txSz)
                flat = [buf[startY + i][startX + j] for i in range(hh) for j in range(w)]
                # dav1d logs a plain DC prediction (P) when this plane's alpha is 0 (identical pixels)
                self.pred_events.append(("Q" if alpha else "P", plane, startX >> 2, startY >> 2, w, hh, flat))
            else:
                self.pred_events.append(("P", plane, startX >> 2, startY >> 2, w, hh, [v for r in pred for v in r]))
        if plane == 0:
            self.MaxLumaW = startX + stepX * 4
            self.MaxLumaH = startY + stepY * 4

    def predict_palette(self, plane, startX, startY, x, y, txSz):
        w = T.Tx_Width[txSz]
        hh = T.Tx_Height[txSz]
        palette = [self.palette_colors_y, self.palette_colors_u, self.palette_colors_v][plane]
        cmap = self.ColorMapY if plane == 0 else self.ColorMapUV
        buf = self.planes[plane]
        for i in range(hh):
            for j in range(w):
                buf[startY + i][startX + j] = palette[cmap[y * 4 + i][x * 4 + j]]

    def get_filter_type(self, plane):
        """7.11.2.8 intra filter type: 1 if the block above or to the left uses a smooth mode."""
        h = self.h
        aboveSmooth = leftSmooth = 0
        avail_u = self.AvailU if plane == 0 else self.AvailUChroma
        avail_l = self.AvailL if plane == 0 else self.AvailLChroma
        if avail_u:
            r, c = self.MiRow - 1, self.MiCol
            if plane > 0:
                if h.subsampling_x and not (self.MiCol & 1): c += 1
                if h.subsampling_y and (self.MiRow & 1): r -= 1
            aboveSmooth = self.is_smooth(r, c, plane)
        if avail_l:
            r, c = self.MiRow, self.MiCol - 1
            if plane > 0:
                if h.subsampling_x and (self.MiCol & 1): c -= 1
                if h.subsampling_y and not (self.MiRow & 1): r += 1
            leftSmooth = self.is_smooth(r, c, plane)
        return 1 if (aboveSmooth or leftSmooth) else 0

    def is_smooth(self, row, col, plane):
        mode = self.YModes[row][col] if plane == 0 else self.UVModes[row][col]
        return 1 if mode in SMOOTH_MODES else 0

    def predict_intra(self, plane, x, y, haveLeft, haveAbove, haveAboveRight, haveBelowLeft, mode, log2W, log2H):
        h = self.h
        bd = h.BitDepth
        w, hh = 1 << log2W, 1 << log2H
        buf = self.planes[plane]
        subX = h.subsampling_x if plane > 0 else 0
        subY = h.subsampling_y if plane > 0 else 0
        maxX = ((h.MiCols * 4) >> subX) - 1
        maxY = ((h.MiRows * 4) >> subY) - 1
        # 7.11.2.1 edge arrays
        if not haveAbove and haveLeft:
            above = [buf[y][x - 1]] * (w + hh)
        elif not haveAbove and not haveLeft:
            above = [(1 << (bd - 1)) - 1] * (w + hh)
        else:
            aboveLimit = min(maxX, x + (2 * w if haveAboveRight else w) - 1)
            above = [buf[y - 1][min(aboveLimit, x + i)] for i in range(w + hh)]
        if not haveLeft and haveAbove:
            left = [buf[y - 1][x]] * (w + hh)
        elif not haveLeft and not haveAbove:
            left = [(1 << (bd - 1)) + 1] * (w + hh)
        else:
            leftLimit = min(maxY, y + (2 * hh if haveBelowLeft else hh) - 1)
            left = [buf[min(leftLimit, y + i)][x - 1] for i in range(w + hh)]
        if haveAbove and haveLeft:
            tl = buf[y - 1][x - 1]
        elif haveAbove:
            tl = buf[y - 1][x]
        elif haveLeft:
            tl = buf[y][x - 1]
        else:
            tl = 1 << (bd - 1)
        use_fi = (plane == 0 and self.use_filter_intra)
        angle_delta = self.AngleDeltaY if plane == 0 else self.AngleDeltaUV
        filter_type = self.get_filter_type(plane) if im.is_directional(mode) and not use_fi else 0
        return im.predict_intra(above, left, tl, mode, log2W, log2H, bd,
                                have_left=int(haveLeft), have_above=int(haveAbove), angle_delta=angle_delta,
                                enable_intra_edge_filter=h.enable_intra_edge_filter, filter_type=filter_type,
                                use_filter_intra=use_fi, filter_intra_mode=self.filter_intra_mode,
                                above_px=min(w, maxX - x + 1), left_px=min(hh, maxY - y + 1))

    def predict_chroma_from_luma(self, plane, startX, startY, txSz):
        h = self.h
        w = T.Tx_Width[txSz]
        hh = T.Tx_Height[txSz]
        alpha = self.CflAlphaU if plane == 1 else self.CflAlphaV
        L = im.cfl_subsample_luma(self.planes[0], startX, startY, w, hh, h.subsampling_x, h.subsampling_y,
                                  self.MaxLumaW, self.MaxLumaH)
        buf = self.planes[plane]
        dc = [[buf[startY + i][startX + j] for j in range(w)] for i in range(hh)]
        out = im.predict_cfl(dc, L, alpha, T.Tx_Width_Log2[txSz], T.Tx_Height_Log2[txSz], h.BitDepth)
        for i in range(hh):
            for j in range(w):
                buf[startY + i][startX + j] = out[i][j]

    # ---- reconstruction (7.12.3 tail) ---------------------------------------------------------------
    def reconstruct_block(self, plane, startX, startY, txSz, dequant_rows):
        h = self.h
        w = T.Tx_Width[txSz]
        hh = T.Tx_Height[txSz]
        deq = [[0] * w for _ in range(hh)]
        for i, r in enumerate(dequant_rows):
            for j, v in enumerate(r):
                deq[i][j] = v
        ttype = xm.DCT_DCT if self.Lossless else self.PlaneTxType
        res = xm.inverse_transform_2d(deq, txSz, ttype, h.BitDepth, bool(self.Lossless))
        flipUD = ttype in FLIP_UD
        flipLR = ttype in FLIP_LR
        buf = self.planes[plane]
        mx = (1 << h.BitDepth) - 1
        for i in range(hh):
            for j in range(w):
                xx = w - 1 - j if flipLR else j
                yy = hh - 1 - i if flipUD else i
                v = buf[startY + yy][startX + xx] + res[i][j]
                buf[startY + yy][startX + xx] = 0 if v < 0 else mx if v > mx else v
        self.recon_events.append(("R", plane, startX >> 2, startY >> 2, w, hh,
                                  [buf[startY + i][startX + j] for i in range(hh) for j in range(w)]))

    def after_transform_block(self, plane, row, col, subX, subY, stepX, stepY, sbMiRow, sbMiCol):
        bd = self.BlockDecoded[plane]
        for i in range(stepY):
            for j in range(stepX):
                bd[((sbMiRow >> subY) + i, (sbMiCol >> subX) + j)] = 1
