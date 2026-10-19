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
FILTER_BITS = 7
# warped motion (7.11.3.5 .. 7.11.3.8)
import os as _os
MC_TRACE = bool(_os.environ.get("TD_MCTRACE"))      # print every predict_inter reference call (debugging the RTL's strips)
WARPEDMODEL_PREC_BITS = 16
WARPEDDIFF_PREC_BITS = 10
WARPEDPIXEL_PREC_SHIFTS = 1 << 6
WARP_PARAM_REDUCE_BITS = 6
DIV_LUT_BITS, DIV_LUT_PREC_BITS = 8, 14
LS_MV_MAX = 256
WARPEDMODEL_TRANS_CLAMP = 1 << 23
WARPEDMODEL_NONDIAGAFFINE_CLAMP = 1 << 13
MAX_SB_SIZE = 128
MASK_MASTER_SIZE = 64
WEDGE_TYPES = 16
WEDGE_HORIZONTAL, WEDGE_VERTICAL, WEDGE_OBLIQUE27, WEDGE_OBLIQUE63, WEDGE_OBLIQUE117, WEDGE_OBLIQUE153 = 0, 1, 2, 3, 4, 5
II_DC_PRED, II_V_PRED, II_H_PRED, II_SMOOTH_PRED = 0, 1, 2, 3
MAX_FRAME_DISTANCE = 31


def floor_log2(x):
    return x.bit_length() - 1


def _wedge_masks():
    """7.11.3.11 initialise_wedge_mask_table: WedgeMasks[bsize][flipSign][wedge] as 2D lists (None where unused)."""
    w = h = MASK_MASTER_SIZE
    master = [[[0] * w for _ in range(h)] for _ in range(6)]
    for j in range(w):
        shift = MASK_MASTER_SIZE // 4
        for i in range(0, h, 2):
            master[WEDGE_OBLIQUE63][i][j] = T.Wedge_Master_Oblique_Even[im.clip3(0, MASK_MASTER_SIZE - 1, j - shift)]
            shift -= 1
            master[WEDGE_OBLIQUE63][i + 1][j] = T.Wedge_Master_Oblique_Odd[im.clip3(0, MASK_MASTER_SIZE - 1, j - shift)]
            master[WEDGE_VERTICAL][i][j] = T.Wedge_Master_Vertical[j]
            master[WEDGE_VERTICAL][i + 1][j] = T.Wedge_Master_Vertical[j]
    for i in range(h):
        for j in range(w):
            msk = master[WEDGE_OBLIQUE63][i][j]
            master[WEDGE_OBLIQUE27][j][i] = msk
            master[WEDGE_OBLIQUE117][i][w - 1 - j] = 64 - msk
            master[WEDGE_OBLIQUE153][w - 1 - j][i] = 64 - msk
            master[WEDGE_HORIZONTAL][j][i] = master[WEDGE_VERTICAL][i][j]

    def block_shape(bsize):
        w4, h4 = T.Num_4x4_Blocks_Wide[bsize], T.Num_4x4_Blocks_High[bsize]
        return 0 if h4 > w4 else 1 if h4 < w4 else 2
    masks = {}
    for bsize in range(tm.BLOCK_8X8, len(T.Wedge_Bits)):
        if T.Wedge_Bits[bsize] > 0:
            bw, bh = T.Block_Width[bsize], T.Block_Height[bsize]
            masks[bsize] = [[None] * WEDGE_TYPES, [None] * WEDGE_TYPES]
            for wedge in range(WEDGE_TYPES):
                d, xo, yo = T.Wedge_Codebook[block_shape(bsize)][wedge]
                xoff = MASK_MASTER_SIZE // 2 - ((xo * bw) >> 3)
                yoff = MASK_MASTER_SIZE // 2 - ((yo * bh) >> 3)
                tot = sum(master[d][yoff][xoff + i] for i in range(bw)) + sum(master[d][yoff + i][xoff] for i in range(1, bh))
                avg = (tot + (bw + bh - 1) // 2) // (bw + bh - 1)
                flipSign = 1 if avg < 32 else 0
                m = [[master[d][yoff + i][xoff + j] for j in range(bw)] for i in range(bh)]
                masks[bsize][flipSign][wedge] = m
                masks[bsize][1 - flipSign][wedge] = [[64 - v for v in row] for row in m]
    return masks


WEDGE_MASKS = _wedge_masks()

FLIP_UD = {xm.FLIPADST_DCT, xm.FLIPADST_ADST, xm.V_FLIPADST, xm.FLIPADST_FLIPADST}
FLIP_LR = {xm.DCT_FLIPADST, xm.ADST_FLIPADST, xm.H_FLIPADST, xm.FLIPADST_FLIPADST}
SMOOTH_MODES = (im.SMOOTH_PRED, im.SMOOTH_V_PRED, im.SMOOTH_H_PRED)


class FrameRecon(tm.TileDecoder):
    def __init__(self, hdr, data, frame, hook=None, ctx=None):
        """frame: shared dict with 'planes' (list of 2D pixel arrays, one per plane) so that several
        tiles of one frame reconstruct into the same buffer, and 'refs' (inter frames): the reference frame store
        {slot: dict(planes, upscaled_width, frame_height)} of final pictures (7.20). ctx: the tile_model.FrameCtx
        of the frame (CDFs, segmentation map, motion field) from tb/seq_model.py."""
        super().__init__(hdr, data, hook, frame=ctx)
        self.fstore = frame                      # the frame store dict (planes, refs); self.frame stays the FrameCtx (TileSummary reads it)
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
        sbMask = 31 if h.use_128x128_superblock else 15
        subBlockMiRow = self.MiRow & sbMask
        subBlockMiCol = self.MiCol & sbMask
        self.IsInterIntra = int(self.is_inter and self.RefFrame[1] == tm.INTRA_FRAME)
        for plane in range(1 + self.HasChroma * 2):
            planeSz = self.get_plane_residual_size(self.MiSize, plane)
            num4x4W = T.Num_4x4_Blocks_Wide[planeSz]
            num4x4H = T.Num_4x4_Blocks_High[planeSz]
            log2W = tm.MI_SIZE_LOG2 + T.Mi_Width_Log2[planeSz]
            log2H = tm.MI_SIZE_LOG2 + T.Mi_Height_Log2[planeSz]
            subX = h.subsampling_x if plane > 0 else 0
            subY = h.subsampling_y if plane > 0 else 0
            baseX = (self.MiCol >> subX) * MI_SIZE
            baseY = (self.MiRow >> subY) * MI_SIZE
            candRow = (self.MiRow >> subY) << subY
            candCol = (self.MiCol >> subX) << subX
            if self.IsInterIntra:
                mode = {II_DC_PRED: im.DC_PRED, II_V_PRED: im.V_PRED, II_H_PRED: im.H_PRED}.get(self.interintra_mode, im.SMOOTH_PRED)
                bd = self.BlockDecoded[plane]
                haveAboveRight = bd.get(((subBlockMiRow >> subY) - 1, (subBlockMiCol >> subX) + num4x4W), 0)
                haveBelowLeft = bd.get(((subBlockMiRow >> subY) + num4x4H, (subBlockMiCol >> subX) - 1), 0)
                pred = self.predict_intra(plane, baseX, baseY, bool(self.AvailL if plane == 0 else self.AvailLChroma),
                                          bool(self.AvailU if plane == 0 else self.AvailUChroma), bool(haveAboveRight),
                                          bool(haveBelowLeft), mode, log2W, log2H)
                buf = self.planes[plane]
                for i in range(1 << log2H):
                    row = buf[baseY + i]
                    for j in range(1 << log2W):
                        row[baseX + j] = pred[i][j]
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

    # ---- 7.11.3 inter prediction ------------------------------------------------------------------------
    def ref_store(self, refFrame):
        """(planes, upscaled width, frame height) of reference refFrame from the frame store."""
        h = self.h
        refIdx = h.ref_frame_idx[refFrame - tm.LAST_FRAME]
        ref = self.fstore["refs"][refIdx]
        return ref["planes"], ref["upscaled_width"], ref["frame_height"]

    def predict_inter(self, plane, x, y, w, hh, candRow, candCol):
        h = self.h
        rf = self.RefFrames[candRow][candCol]
        isCompound = int(rf[1] > tm.INTRA_FRAME)
        # 7.11.3.2 rounding variables
        InterRound0 = 3
        InterRound1 = 7 if isCompound else 11
        if h.BitDepth == 12:
            InterRound0 += 2
        if h.BitDepth == 12 and not isCompound:
            InterRound1 -= 2
        InterPostRound = 2 * FILTER_BITS - (InterRound0 + InterRound1)
        if plane == 0 and self.motion_mode == tm.LOCALWARP:
            self.warp_estimation()
            if self.LocalValid:
                self.LocalValid = self.setup_shear(self.LocalWarpParams)[0]
        preds = []
        for refList in range(1 + isCompound):
            refFrame = rf[refList]
            globalValid = 0
            if self.YMode in (tm.GLOBALMV, tm.GLOBAL_GLOBALMV) and h.GmType[refFrame] > tm.TRANSLATION:
                globalValid = self.setup_shear(h.gm_params[refFrame])[0]
            if w < 8 or hh < 8 or h.force_integer_mv:
                useWarp = 0
            elif self.motion_mode == tm.LOCALWARP and self.LocalValid:
                useWarp = 1
            elif (self.YMode in (tm.GLOBALMV, tm.GLOBAL_GLOBALMV) and h.GmType[refFrame] > tm.TRANSLATION
                  and not self.is_scaled(refFrame) and globalValid):
                useWarp = 2
            else:
                useWarp = 0
            mv = self.Mvs[candRow][candCol][refList]
            if not self.use_intrabc:
                refPlanes, refW, refH = self.ref_store(refFrame)
                startX, startY, stepX, stepY = self.motion_vector_scaling(plane, x, y, mv, refW, refH)
                clampW, clampH = refW, refH
            else:
                # refIdx = -1: the current (pre loop filter) frame, first with the frame's size (so the scaling has
                # no effect), then with the size rounded up to whole 4x4 units for the clamping
                refPlanes = self.planes
                startX, startY, stepX, stepY = self.motion_vector_scaling(plane, x, y, mv, h.UpscaledWidth, h.FrameHeight)
                clampW, clampH = h.MiCols * MI_SIZE, h.MiRows * MI_SIZE
            if MC_TRACE:
                print(f"MODEL MC pl{plane} l{refList} at ({x},{y}) {w}x{hh} ref {refFrame} warp {useWarp} start {startX},{startY} "
                      f"steps {stepX},{stepY} mv {mv} clamp {clampW}x{clampH}")
            if useWarp:
                pred = [[0] * w for _ in range(hh)]
                params = self.LocalWarpParams if useWarp == 1 else h.gm_params[refFrame]
                for i8 in range(((hh - 1) >> 3) + 1):
                    for j8 in range(((w - 1) >> 3) + 1):
                        self.block_warp(params, plane, refPlanes, refW, refH, x, y, i8, j8, w, hh, pred,
                                        InterRound0, InterRound1)
            else:
                pred = self.block_inter_prediction(plane, refPlanes, startX, startY, stepX, stepY, w, hh, candRow, candCol,
                                                   clampW, clampH, InterRound0, InterRound1)
            if MC_TRACE:
                print(f"MODEL MC   pred rows 0/{hh // 2}: {pred[0][:8]} / {pred[hh // 2][:8]}")
            preds.append(pred)
        # masks (persist across the planes of the block: the luma mask serves the chroma blend)
        if self.compound_type == tm.COMPOUND_WEDGE and plane == 0:
            self.Mask = WEDGE_MASKS[self.MiSize][self.wedge_sign][self.wedge_index]
        elif self.compound_type == tm.COMPOUND_INTRA:
            self.Mask = self.intra_variant_mask(w, hh)
        elif self.compound_type == tm.COMPOUND_DIFFWTD and plane == 0:
            self.Mask = self.difference_weight_mask(preds, w, hh, InterPostRound)
        if self.compound_type == tm.COMPOUND_DISTANCE:
            FwdWeight, BckWeight = self.distance_weights(candRow, candCol)
        buf = self.planes[plane]
        mx = (1 << h.BitDepth) - 1
        clip1 = lambda v: 0 if v < 0 else mx if v > mx else v  # noqa: E731
        if not isCompound and not self.IsInterIntra:
            for i in range(hh):
                row, pr = buf[y + i], preds[0][i]
                for j in range(w):
                    row[x + j] = clip1(pr[j])
        elif self.compound_type == tm.COMPOUND_AVERAGE:
            for i in range(hh):
                row, p0, p1 = buf[y + i], preds[0][i], preds[1][i]
                for j in range(w):
                    row[x + j] = clip1(im.round2(p0[j] + p1[j], 1 + InterPostRound))
        elif self.compound_type == tm.COMPOUND_DISTANCE:
            for i in range(hh):
                row, p0, p1 = buf[y + i], preds[0][i], preds[1][i]
                for j in range(w):
                    row[x + j] = clip1(im.round2(FwdWeight * p0[j] + BckWeight * p1[j], 4 + InterPostRound))
        else:
            self.mask_blend(preds, plane, x, y, w, hh, InterPostRound)
        if self.motion_mode == tm.OBMC:
            self.overlapped_motion_compensation(plane, w, hh)

    def is_scaled(self, refFrame):
        h = self.h
        xScale = ((h.RefUpscaledWidths[refFrame] << REF_SCALE_SHIFT) + (h.FrameWidth // 2)) // h.FrameWidth
        yScale = ((h.RefFrameHeights[refFrame] << REF_SCALE_SHIFT) + (h.FrameHeight // 2)) // h.FrameHeight
        return xScale != (1 << REF_SCALE_SHIFT) or yScale != (1 << REF_SCALE_SHIFT)

    # 7.11.3.5 block warp
    def block_warp(self, warpParams, plane, ref, refW, refH, x, y, i8, j8, w, hh, pred, InterRound0, InterRound1):
        h = self.h
        subX = h.subsampling_x if plane > 0 else 0
        subY = h.subsampling_y if plane > 0 else 0
        lastX = ((refW + subX) >> subX) - 1
        lastY = ((refH + subY) >> subY) - 1
        srcX = (x + j8 * 8 + 4) << subX
        srcY = (y + i8 * 8 + 4) << subY
        dstX = warpParams[2] * srcX + warpParams[3] * srcY + warpParams[0]
        dstY = warpParams[4] * srcX + warpParams[5] * srcY + warpParams[1]
        _, alpha, beta, gamma, delta = self.setup_shear(warpParams)
        refp = ref[plane]
        x4 = dstX >> subX
        y4 = dstY >> subY
        ix4 = x4 >> WARPEDMODEL_PREC_BITS
        sx4 = x4 & ((1 << WARPEDMODEL_PREC_BITS) - 1)
        iy4 = y4 >> WARPEDMODEL_PREC_BITS
        sy4 = y4 & ((1 << WARPEDMODEL_PREC_BITS) - 1)
        WF = T.Warped_Filters
        intermediate = [[0] * 8 for _ in range(15)]
        for i1 in range(-7, 8):
            srow = refp[im.clip3(0, lastY, iy4 + i1)]
            for i2 in range(-4, 4):
                sx = sx4 + alpha * i2 + beta * i1
                offs = im.round2(sx, WARPEDDIFF_PREC_BITS) + WARPEDPIXEL_PREC_SHIFTS
                taps = WF[offs]
                s = 0
                for i3 in range(8):
                    s += taps[i3] * srow[im.clip3(0, lastX, ix4 + i2 - 3 + i3)]
                intermediate[i1 + 7][i2 + 4] = im.round2(s, InterRound0)
        for i1 in range(-4, min(4, hh - i8 * 8 - 4)):
            for i2 in range(-4, min(4, w - j8 * 8 - 4)):
                sy = sy4 + gamma * i2 + delta * i1
                offs = im.round2(sy, WARPEDDIFF_PREC_BITS) + WARPEDPIXEL_PREC_SHIFTS
                taps = WF[offs]
                s = 0
                for i3 in range(8):
                    s += taps[i3] * intermediate[i1 + i3 + 4][i2 + 4]
                pred[i8 * 8 + i1 + 4][j8 * 8 + i2 + 4] = im.round2(s, InterRound1)

    # 7.11.3.6 / 7.11.3.7 setup shear, resolve divisor
    @staticmethod
    def resolve_divisor(d):
        n = floor_log2(abs(d))
        e = abs(d) - (1 << n)
        f = im.round2(e, n - DIV_LUT_BITS) if n > DIV_LUT_BITS else e << (DIV_LUT_BITS - n)
        divShift = n + DIV_LUT_PREC_BITS
        divFactor = -T.Div_Lut[f] if d < 0 else T.Div_Lut[f]
        return divShift, divFactor

    def setup_shear(self, wp):
        alpha0 = im.clip3(-32768, 32767, wp[2] - (1 << WARPEDMODEL_PREC_BITS))
        beta0 = im.clip3(-32768, 32767, wp[3])
        divShift, divFactor = self.resolve_divisor(wp[2])
        v = wp[4] << WARPEDMODEL_PREC_BITS
        gamma0 = im.clip3(-32768, 32767, im.round2signed(v * divFactor, divShift))
        w = wp[3] * wp[4]
        delta0 = im.clip3(-32768, 32767, wp[5] - im.round2signed(w * divFactor, divShift) - (1 << WARPEDMODEL_PREC_BITS))
        rb = WARP_PARAM_REDUCE_BITS
        alpha = im.round2signed(alpha0, rb) << rb
        beta = im.round2signed(beta0, rb) << rb
        gamma = im.round2signed(gamma0, rb) << rb
        delta = im.round2signed(delta0, rb) << rb
        if 4 * abs(alpha) + 7 * abs(beta) >= (1 << WARPEDMODEL_PREC_BITS):
            warpValid = 0
        elif 4 * abs(gamma) + 4 * abs(delta) >= (1 << WARPEDMODEL_PREC_BITS):
            warpValid = 0
        else:
            warpValid = 1
        return warpValid, alpha, beta, gamma, delta

    # 7.11.3.8 warp estimation (least squares over CandList from find_warp_samples)
    def warp_estimation(self):
        A = [[0, 0], [0, 0]]
        Bx = [0, 0]
        By = [0, 0]
        w4 = T.Num_4x4_Blocks_Wide[self.MiSize]
        h4 = T.Num_4x4_Blocks_High[self.MiSize]
        midY = self.MiRow * 4 + h4 * 2 - 1
        midX = self.MiCol * 4 + w4 * 2 - 1
        suy, sux = midY * 8, midX * 8
        duy, dux = suy + self.Mv[0][0], sux + self.Mv[0][1]

        def ls_product(a, b):
            return ((a * b) >> 2) + (a + b)
        for i in range(self.NumSamples):
            cand = self.CandList[i]
            sy, sx = cand[0] - suy, cand[1] - sux
            dy, dx = cand[2] - duy, cand[3] - dux
            if abs(sx - dx) < LS_MV_MAX and abs(sy - dy) < LS_MV_MAX:
                A[0][0] += ls_product(sx, sx) + 8
                A[0][1] += ls_product(sx, sy) + 4
                A[1][1] += ls_product(sy, sy) + 8
                Bx[0] += ls_product(sx, dx) + 8
                Bx[1] += ls_product(sy, dx) + 4
                By[0] += ls_product(sx, dy) + 4
                By[1] += ls_product(sy, dy) + 8
        det = A[0][0] * A[1][1] - A[0][1] * A[0][1]
        self.LocalValid = int(det != 0)
        if det == 0:
            return
        divShift, divFactor = self.resolve_divisor(det)
        divShift -= WARPEDMODEL_PREC_BITS
        if divShift < 0:
            divFactor = divFactor << (-divShift)
            divShift = 0

        def nondiag(v):
            return im.clip3(-WARPEDMODEL_NONDIAGAFFINE_CLAMP + 1, WARPEDMODEL_NONDIAGAFFINE_CLAMP - 1,
                            im.round2signed(v * divFactor, divShift))

        def diag(v):
            return im.clip3((1 << WARPEDMODEL_PREC_BITS) - WARPEDMODEL_NONDIAGAFFINE_CLAMP + 1,
                            (1 << WARPEDMODEL_PREC_BITS) + WARPEDMODEL_NONDIAGAFFINE_CLAMP - 1,
                            im.round2signed(v * divFactor, divShift))
        p = [0] * 6
        p[2] = diag(A[1][1] * Bx[0] - A[0][1] * Bx[1])
        p[3] = nondiag(-A[0][1] * Bx[0] + A[0][0] * Bx[1])
        p[4] = nondiag(A[1][1] * By[0] - A[0][1] * By[1])
        p[5] = diag(-A[0][1] * By[0] + A[0][0] * By[1])
        mvx, mvy = self.Mv[0][1], self.Mv[0][0]
        vx = mvx * (1 << (WARPEDMODEL_PREC_BITS - 3)) - (midX * (p[2] - (1 << WARPEDMODEL_PREC_BITS)) + midY * p[3])
        vy = mvy * (1 << (WARPEDMODEL_PREC_BITS - 3)) - (midX * p[4] + midY * (p[5] - (1 << WARPEDMODEL_PREC_BITS)))
        p[0] = im.clip3(-WARPEDMODEL_TRANS_CLAMP, WARPEDMODEL_TRANS_CLAMP - 1, vx)
        p[1] = im.clip3(-WARPEDMODEL_TRANS_CLAMP, WARPEDMODEL_TRANS_CLAMP - 1, vy)
        self.LocalWarpParams = p

    # 7.11.3.9 / 7.11.3.10 overlapped motion compensation and blending
    def overlapped_motion_compensation(self, plane, w, hh):
        h = self.h
        subX = h.subsampling_x if plane > 0 else 0
        subY = h.subsampling_y if plane > 0 else 0
        buf = self.planes[plane]
        mx = (1 << h.BitDepth) - 1

        def get_obmc_mask(length):
            return {2: T.Obmc_Mask_2, 4: T.Obmc_Mask_4, 8: T.Obmc_Mask_8, 16: T.Obmc_Mask_16}.get(length, T.Obmc_Mask_32)

        def predict_overlap(candRow, candCol, x4, y4, predW, predH, mask, pass_):
            mv = self.Mvs[candRow][candCol][0]
            refFrame = self.RefFrames[candRow][candCol][0]
            refPlanes, refW, refH = self.ref_store(refFrame)
            predX = (x4 * 4) >> subX
            predY = (y4 * 4) >> subY
            startX, startY, stepX, stepY = self.motion_vector_scaling(plane, predX, predY, mv, refW, refH)
            InterRound0 = 3 + (2 if h.BitDepth == 12 else 0)
            InterRound1 = 11 - (2 if h.BitDepth == 12 else 0)
            obmcPred = self.block_inter_prediction(plane, refPlanes, startX, startY, stepX, stepY, predW, predH,
                                                   candRow, candCol, refW, refH, InterRound0, InterRound1)
            for i in range(predH):
                row = buf[predY + i]
                for j in range(predW):
                    o = obmcPred[i][j]
                    o = 0 if o < 0 else mx if o > mx else o
                    m = mask[i] if pass_ == 0 else mask[j]
                    row[predX + j] = im.round2(m * row[predX + j] + (64 - m) * o, 6)
        if self.AvailU and self.get_plane_residual_size(self.MiSize, plane) >= tm.BLOCK_8X8:
            w4 = T.Num_4x4_Blocks_Wide[self.MiSize]
            x4, y4 = self.MiCol, self.MiRow
            nCount = 0
            nLimit = min(4, T.Mi_Width_Log2[self.MiSize])
            while nCount < nLimit and x4 < min(h.MiCols, self.MiCol + w4):
                candRow = self.MiRow - 1
                candCol = x4 | 1
                candSz = self.MiSizes[candRow][candCol]
                step4 = im.clip3(2, 16, T.Num_4x4_Blocks_Wide[candSz])
                if self.RefFrames[candRow][candCol][0] > tm.INTRA_FRAME:
                    nCount += 1
                    predW = min(w, (step4 * MI_SIZE) >> subX)
                    predH = min(hh >> 1, 32 >> subY)
                    predict_overlap(candRow, candCol, x4, y4, predW, predH, get_obmc_mask(predH), 0)
                x4 += step4
        if self.AvailL:
            h4 = T.Num_4x4_Blocks_High[self.MiSize]
            x4, y4 = self.MiCol, self.MiRow
            nCount = 0
            nLimit = min(4, T.Mi_Height_Log2[self.MiSize])
            while nCount < nLimit and y4 < min(h.MiRows, self.MiRow + h4):
                candCol = self.MiCol - 1
                candRow = y4 | 1
                candSz = self.MiSizes[candRow][candCol]
                step4 = im.clip3(2, 16, T.Num_4x4_Blocks_High[candSz])
                if self.RefFrames[candRow][candCol][0] > tm.INTRA_FRAME:
                    nCount += 1
                    predW = min(w >> 1, 32 >> subX)
                    predH = min(hh, (step4 * MI_SIZE) >> subY)
                    predict_overlap(candRow, candCol, x4, y4, predW, predH, get_obmc_mask(predW), 1)
                y4 += step4

    # 7.11.3.12 / 7.11.3.13 / 7.11.3.14 / 7.11.3.15 masks, blend, distance weights
    def difference_weight_mask(self, preds, w, hh, InterPostRound):
        h = self.h
        mask = []
        for i in range(hh):
            row = []
            for j in range(w):
                diff = abs(preds[0][i][j] - preds[1][i][j])
                diff = im.round2(diff, (h.BitDepth - 8) + InterPostRound)
                m = im.clip3(0, 64, 38 + diff // 16)
                row.append(64 - m if self.mask_type else m)
            mask.append(row)
        return mask

    def intra_variant_mask(self, w, hh):
        sizeScale = MAX_SB_SIZE // max(hh, w)
        mask = []
        for i in range(hh):
            row = []
            for j in range(w):
                if self.interintra_mode == II_V_PRED:
                    row.append(T.Ii_Weights_1d[i * sizeScale])
                elif self.interintra_mode == II_H_PRED:
                    row.append(T.Ii_Weights_1d[j * sizeScale])
                elif self.interintra_mode == II_SMOOTH_PRED:
                    row.append(T.Ii_Weights_1d[min(i, j) * sizeScale])
                else:
                    row.append(32)
            mask.append(row)
        return mask

    def mask_blend(self, preds, plane, dstX, dstY, w, hh, InterPostRound):
        h = self.h
        subX = h.subsampling_x if plane > 0 else 0
        subY = h.subsampling_y if plane > 0 else 0
        buf = self.planes[plane]
        mx = (1 << h.BitDepth) - 1
        M = self.Mask
        for y in range(hh):
            row = buf[dstY + y]
            for x in range(w):
                if (not subX and not subY) or (self.interintra and not self.wedge_interintra):
                    m = M[y][x]
                elif subX and not subY:
                    m = im.round2(M[y][2 * x] + M[y][2 * x + 1], 1)
                else:
                    m = im.round2(M[2 * y][2 * x] + M[2 * y][2 * x + 1] + M[2 * y + 1][2 * x] + M[2 * y + 1][2 * x + 1], 2)
                if self.interintra:
                    p0 = im.round2(preds[0][y][x], InterPostRound)
                    p0 = 0 if p0 < 0 else mx if p0 > mx else p0
                    p1 = row[dstX + x]
                    row[dstX + x] = im.round2(m * p1 + (64 - m) * p0, 6)
                else:
                    v = im.round2(m * preds[0][y][x] + (64 - m) * preds[1][y][x], 6 + InterPostRound)
                    row[dstX + x] = 0 if v < 0 else mx if v > mx else v

    def distance_weights(self, candRow, candCol):
        h = self.h
        dist = [0, 0]
        for refList in range(2):
            oh = h.OrderHints[self.RefFrames[candRow][candCol][refList]]
            dist[refList] = im.clip3(0, MAX_FRAME_DISTANCE, abs(self.get_relative_dist(oh, h.order_hint)))
        d0, d1 = dist[1], dist[0]
        order = int(d0 <= d1)
        if d0 == 0 or d1 == 0:
            return T.Quant_Dist_Lookup[3][order], T.Quant_Dist_Lookup[3][1 - order]
        i = 0
        while i < 3:
            c0 = T.Quant_Dist_Weight[i][order]
            c1 = T.Quant_Dist_Weight[i][1 - order]
            if order:
                if d0 * c0 > d1 * c1:
                    break
            else:
                if d0 * c0 < d1 * c1:
                    break
            i += 1
        return T.Quant_Dist_Lookup[i][order], T.Quant_Dist_Lookup[i][1 - order]

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
