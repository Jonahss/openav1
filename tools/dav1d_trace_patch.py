#!/usr/bin/env python3
"""Apply the openav1 golden-trace hooks to a pristine dav1d checkout (tested on bf5a879, 2026-09).

Idempotent: refuses to run twice. Usage: tools/dav1d_trace_patch.py third_party/dav1d

Trace format v3 (one event per line, enabled by DAV1D_TRACE=<path|->, DAV1D_TRACE_MASK bits
1 sym, 2 coef, 4 pred, 8 recon; --threads 1 and an asm-free build are required):
  H <tile_row> <tile_col> <col_start> <col_end> <row_start> <row_end> <frame header fields...>  (before each T; see setup_tile hook)
  T <tile_bytes> <disable_cdf_update> <hex tile bytes>
  S A <n> <val> <rng> <cnt> <icdf[0..n-1]> [<icdf'[0..n-1]> <cnt'>]   (n = N-1; inverted CDFs)
  S B <f> <bit> <rng>        followed by  U <cnt> <f'> <cnt'>  when the bool was adaptive
  S E <bit> <rng>
  C <plane> <x4> <y4> <tx> <txtp> <eob> <w> <h> <Dequant row-major, min(w,32) x min(h,32)>
  P <plane> <x4> <y4> <w> <h> <mode> <m> <angle> <maxw> <maxh> <rawtl> <edge[-2h..2w]> <w*h pred pixels>
      (rawtl = top-left BEFORE dav1d's in-preparation Z2 corner filter; edge[0] may already be filtered)
  Q <plane> <x4> <y4> <w> <h> <m> <alpha> <edge[-2h..2w]> <w*h ac> <w*h pred pixels>   (CfL)
  R <plane> <x4> <y4> <w> <h> <tx> <txtp> <w*h pixels after inverse transform, before filters>
x4/y4 are in 4x4 units of the plane. mode = bitstream intra mode (FILTER_PRED=13 means filter
intra, then angle = filter mode); m = dav1d's DSP mode index; angle = dav1d's packed angle
(bits 0-8 absolute angle, bit 9 smooth-neighbour flag, bit 10 intra edge filter enable).
"""
import sys
from pathlib import Path

root = Path(sys.argv[1] if len(sys.argv) > 1 else "third_party/dav1d")
src = root / "src"


def patch(path, pairs, count=None):
    s = path.read_text()
    for old, new in pairs:
        n = s.count(old)
        assert n >= 1, f"{path}: anchor not found:\n{old}"
        if count is not None:
            assert n == count, f"{path}: anchor count {n} != {count}:\n{old}"
        s = s.replace(old, new)
    path.write_text(s)


if (src / "trace.h").exists():
    sys.exit("already patched")

(src / "trace.h").write_text('''/*
 * openav1 golden-trace hooks for dav1d (format v3, see tools/dav1d_trace_patch.py).
 * Enabled at runtime by DAV1D_TRACE=<path|-> (and optional DAV1D_TRACE_MASK bits).
 * Only meaningful in a build with -Denable_asm=false and --threads 1.
 */
#ifndef DAV1D_SRC_TRACE_H
#define DAV1D_SRC_TRACE_H

#include <stdio.h>

enum {
    DAV1D_TRACE_SYM   = 1 << 0,  /* S/T/U: every arithmetic-decoder symbol */
    DAV1D_TRACE_COEF  = 1 << 1,  /* C: dequantized coefficient blocks */
    DAV1D_TRACE_PRED  = 1 << 2,  /* P/Q: intra prediction (edges + output) */
    DAV1D_TRACE_RECON = 1 << 3,  /* R: pre-loop-filter reconstruction */
};

extern FILE *dav1d_trace_fp;
extern unsigned dav1d_trace_mask;
extern int dav1d_trace_corner_raw;   /* top-left before the Z2 corner filter (set by ipred_prepare) */

void dav1d_trace_init(void);

#define DAV1D_TRACE_ON(bit) (dav1d_trace_fp && (dav1d_trace_mask & (bit)))

#endif /* DAV1D_SRC_TRACE_H */
''')

(src / "trace.c").write_text('''#include "config.h"

#include <stdlib.h>
#include <string.h>

#include "src/trace.h"

FILE *dav1d_trace_fp = NULL;
unsigned dav1d_trace_mask = 0;
int dav1d_trace_corner_raw = 0;

void dav1d_trace_init(void) {
    static int done = 0;
    if (done) return;
    done = 1;
    const char *const path = getenv("DAV1D_TRACE");
    if (!path || !*path) return;
    dav1d_trace_fp = !strcmp(path, "-") ? stdout : fopen(path, "w");
    if (!dav1d_trace_fp) return;
    const char *const mask = getenv("DAV1D_TRACE_MASK");
    dav1d_trace_mask = mask ? (unsigned) strtoul(mask, NULL, 0) : 0xf;
    fprintf(dav1d_trace_fp, "# openav1 dav1d trace v3 mask=%u\\n", dav1d_trace_mask);
}
''')

patch(src / "meson.build", [("    'msac.c',\n", "    'msac.c',\n    'trace.c',\n")], 1)

patch(src / "lib.c", [
    ('#include "src/log.h"\n', '#include "src/log.h"\n#include "src/trace.h"\n'),
    ("    pthread_once(&initted, init_internal);\n\n    validate_input_or_ret(c_out != NULL, DAV1D_ERR(EINVAL));",
     "    pthread_once(&initted, init_internal);\n    dav1d_trace_init();\n\n    validate_input_or_ret(c_out != NULL, DAV1D_ERR(EINVAL));"),
], 1)

# ---- msac.c -------------------------------------------------------------------------------
patch(src / "msac.c", [
    ('#include "src/msac.h"\n', '#include "src/msac.h"\n#include "src/trace.h"\n'),
    # equiprobable bool
    ('''    v += ret * (r - 2 * v);
    ctx_norm(s, dif, v);
    return !ret;
}

/* Decode a single binary value.''',
     '''    v += ret * (r - 2 * v);
    ctx_norm(s, dif, v);
    if (DAV1D_TRACE_ON(DAV1D_TRACE_SYM))
        fprintf(dav1d_trace_fp, "S E %u %u\\n", !ret, s->rng);
    return !ret;
}

/* Decode a single binary value.'''),
    # fixed-probability bool
    ('''    v += ret * (r - 2 * v);
    ctx_norm(s, dif, v);
    return !ret;
}

/* Decodes a symbol given an inverse cumulative distribution function (CDF)''',
     '''    v += ret * (r - 2 * v);
    ctx_norm(s, dif, v);
    if (DAV1D_TRACE_ON(DAV1D_TRACE_SYM))
        fprintf(dav1d_trace_fp, "S B %u %u %u\\n", f, !ret, s->rng);
    return !ret;
}

/* Decodes a symbol given an inverse cumulative distribution function (CDF)'''),
    # adaptive multi-symbol
    ('''    ctx_norm(s, s->dif - ((ec_win)v << (EC_WIN_SIZE - 16)), u - v);

    if (s->allow_update_cdf) {
        const unsigned count = cdf[n_symbols];
        const unsigned rate = 4 + (count >> 4) + (n_symbols > 2);
        unsigned i;
        for (i = 0; i < val; i++)
            cdf[i] += (32768 - cdf[i]) >> rate;
        for (; i < n_symbols; i++)
            cdf[i] -= cdf[i] >> rate;
        cdf[n_symbols] = count + (count < 32);
    }

    return val;''',
     '''    ctx_norm(s, s->dif - ((ec_win)v << (EC_WIN_SIZE - 16)), u - v);
    const int tr = DAV1D_TRACE_ON(DAV1D_TRACE_SYM);
    if (tr) {
        fprintf(dav1d_trace_fp, "S A %u %u %u %u", (unsigned) n_symbols, val, s->rng, cdf[n_symbols]);
        for (unsigned i = 0; i < n_symbols; i++) fprintf(dav1d_trace_fp, " %u", cdf[i]);
    }

    if (s->allow_update_cdf) {
        const unsigned count = cdf[n_symbols];
        const unsigned rate = 4 + (count >> 4) + (n_symbols > 2);
        unsigned i;
        for (i = 0; i < val; i++)
            cdf[i] += (32768 - cdf[i]) >> rate;
        for (; i < n_symbols; i++)
            cdf[i] -= cdf[i] >> rate;
        cdf[n_symbols] = count + (count < 32);
        if (tr) {
            for (i = 0; i < n_symbols; i++) fprintf(dav1d_trace_fp, " %u", cdf[i]);
            fprintf(dav1d_trace_fp, " %u", cdf[n_symbols]);
        }
    }
    if (tr) fputc('\\n', dav1d_trace_fp);

    return val;'''),
    # adaptive bool update
    ('''        cdf[1] = count + (count < 32);
    }

    return bit;''',
     '''        cdf[1] = count + (count < 32);
        if (DAV1D_TRACE_ON(DAV1D_TRACE_SYM))
            fprintf(dav1d_trace_fp, "U %u %u %u\\n", count, cdf[0], cdf[1]);
    }

    return bit;'''),
    # tile init
    ('''{
    s->buf_pos = data;
    s->buf_end = data + sz;
    s->dif = 0;
    s->rng = 0x8000;''',
     '''{
    if (DAV1D_TRACE_ON(DAV1D_TRACE_SYM)) {
        fprintf(dav1d_trace_fp, "T %zu %d ", sz, disable_cdf_update_flag);
        for (size_t i = 0; i < sz; i++) fprintf(dav1d_trace_fp, "%02x", data[i]);
        fputc('\\n', dav1d_trace_fp);
    }
    s->buf_pos = data;
    s->buf_end = data + sz;
    s->dif = 0;
    s->rng = 0x8000;'''),
], 1)

# ---- recon_tmpl.c --------------------------------------------------------------------------
helpers = '''#include "src/trace.h"

/* openav1 trace helpers (pixel-templated, compiled once per bitdepth) */
static void trace_coef_block(const int plane, const int x4, const int y4, const int tx,
                             const int txtp, const int eob, const coef *const cf,
                             const int w, const int h)
{
    if (!DAV1D_TRACE_ON(DAV1D_TRACE_COEF)) return;
    const int cw = imin(w, 32), ch = imin(h, 32);
    fprintf(dav1d_trace_fp, "C %d %d %d %d %d %d %d %d", plane, x4, y4, tx, txtp, eob, w, h);
    /* dav1d stores cf column-major: cf[x * ch + y]; emit the spec's row-major Dequant[y][x] */
    for (int y = 0; y < ch; y++)
        for (int x = 0; x < cw; x++)
            fprintf(dav1d_trace_fp, " %d", (int) cf[x * ch + y]);
    fputc('\\n', dav1d_trace_fp);
}

static void trace_pixels_tail(const pixel *const dst, const ptrdiff_t stride, const int w, const int h)
{
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            fprintf(dav1d_trace_fp, " %d", (int) dst[y * PXSTRIDE(stride) + x]);
    fputc('\\n', dav1d_trace_fp);
}

static void trace_recon(const int plane, const int x4, const int y4, const int w, const int h,
                        const int tx, const int txtp, const pixel *const dst, const ptrdiff_t stride)
{
    if (!DAV1D_TRACE_ON(DAV1D_TRACE_RECON)) return;
    fprintf(dav1d_trace_fp, "R %d %d %d %d %d %d %d", plane, x4, y4, w, h, tx, txtp);
    trace_pixels_tail(dst, stride, w, h);
}

static void trace_edge(const pixel *const edge, const int w, const int h)
{
    for (int i = -2 * h; i <= 2 * w; i++) fprintf(dav1d_trace_fp, " %d", (int) edge[i]);
}

static void trace_pred(const int plane, const int x4, const int y4, const int w, const int h,
                       const int mode, const int m, const int angle, const int maxw, const int maxh,
                       const pixel *const edge, const pixel *const dst, const ptrdiff_t stride)
{
    if (!DAV1D_TRACE_ON(DAV1D_TRACE_PRED)) return;
    fprintf(dav1d_trace_fp, "P %d %d %d %d %d %d %d %d %d %d %d", plane, x4, y4, w, h, mode, m, angle, maxw, maxh,
            dav1d_trace_corner_raw);
    trace_edge(edge, w, h);
    trace_pixels_tail(dst, stride, w, h);
}

static void trace_cfl(const int plane, const int x4, const int y4, const int w, const int h,
                      const int m, const int alpha, const pixel *const edge, const int16_t *const ac,
                      const pixel *const dst, const ptrdiff_t stride)
{
    if (!DAV1D_TRACE_ON(DAV1D_TRACE_PRED)) return;
    fprintf(dav1d_trace_fp, "Q %d %d %d %d %d %d %d", plane, x4, y4, w, h, m, alpha);
    trace_edge(edge, w, h);
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            fprintf(dav1d_trace_fp, " %d", (int) ac[y * w + x]);   /* ac stride is the block width (cfl_pred: ac += width) */
    trace_pixels_tail(dst, stride, w, h);
}
'''

patch(src / "recon_tmpl.c", [
    ('#include "src/ipred_prepare.h"\n', '#include "src/ipred_prepare.h"\n' + helpers),
    # luma intra pred (intra blocks)
    ('''                    dsp->ipred.intra_pred[m](dst, f->cur.stride[0], edge,
                                             t_dim->w * 4, t_dim->h * 4,
                                             angle | intra_flags,
                                             4 * f->bw - 4 * t->bx,
                                             4 * f->bh - 4 * t->by
                                             HIGHBD_CALL_SUFFIX);
''',
     '''                    dsp->ipred.intra_pred[m](dst, f->cur.stride[0], edge,
                                             t_dim->w * 4, t_dim->h * 4,
                                             angle | intra_flags,
                                             4 * f->bw - 4 * t->bx,
                                             4 * f->bh - 4 * t->by
                                             HIGHBD_CALL_SUFFIX);
                    trace_pred(0, t->bx, t->by, t_dim->w * 4, t_dim->h * 4, b->y_mode, m,
                               angle | intra_flags, 4 * f->bw - 4 * t->bx, 4 * f->bh - 4 * t->by,
                               edge, dst, f->cur.stride[0]);
'''),
    # luma coef + itx (intra blocks)
    ('''                            dsp->itx.itxfm_add[b->tx]
                                              [txtp](dst,
                                                     f->cur.stride[0],
                                                     cf, eob HIGHBD_CALL_SUFFIX);
''',
     '''                            trace_coef_block(0, t->bx, t->by, b->tx, txtp, eob, cf, t_dim->w * 4, t_dim->h * 4);
                            dsp->itx.itxfm_add[b->tx]
                                              [txtp](dst,
                                                     f->cur.stride[0],
                                                     cf, eob HIGHBD_CALL_SUFFIX);
                            trace_recon(0, t->bx, t->by, t_dim->w * 4, t_dim->h * 4, b->tx, txtp, dst, f->cur.stride[0]);
'''),
    # CfL
    ('''                    dsp->ipred.cfl_pred[m](uv_dst[pl], stride, edge,
                                           uv_t_dim->w * 4,
                                           uv_t_dim->h * 4,
                                           ac, b->cfl_alpha[pl]
                                           HIGHBD_CALL_SUFFIX);
''',
     '''                    dsp->ipred.cfl_pred[m](uv_dst[pl], stride, edge,
                                           uv_t_dim->w * 4,
                                           uv_t_dim->h * 4,
                                           ac, b->cfl_alpha[pl]
                                           HIGHBD_CALL_SUFFIX);
                    trace_cfl(1 + pl, xpos, ypos, uv_t_dim->w * 4, uv_t_dim->h * 4, m, b->cfl_alpha[pl],
                              edge, ac, uv_dst[pl], stride);
'''),
    # chroma intra pred
    ('''                        dsp->ipred.intra_pred[m](dst, stride, edge,
                                                 uv_t_dim->w * 4,
                                                 uv_t_dim->h * 4,
                                                 angle | sm_uv_fl,
                                                 (4 * f->bw + ss_hor -
                                                  4 * (t->bx & ~ss_hor)) >> ss_hor,
                                                 (4 * f->bh + ss_ver -
                                                  4 * (t->by & ~ss_ver)) >> ss_ver
                                                 HIGHBD_CALL_SUFFIX);
''',
     '''                        dsp->ipred.intra_pred[m](dst, stride, edge,
                                                 uv_t_dim->w * 4,
                                                 uv_t_dim->h * 4,
                                                 angle | sm_uv_fl,
                                                 (4 * f->bw + ss_hor -
                                                  4 * (t->bx & ~ss_hor)) >> ss_hor,
                                                 (4 * f->bh + ss_ver -
                                                  4 * (t->by & ~ss_ver)) >> ss_ver
                                                 HIGHBD_CALL_SUFFIX);
                        trace_pred(1 + pl, xpos, ypos, uv_t_dim->w * 4, uv_t_dim->h * 4, uv_mode, m,
                                   angle | sm_uv_fl,
                                   (4 * f->bw + ss_hor - 4 * (t->bx & ~ss_hor)) >> ss_hor,
                                   (4 * f->bh + ss_ver - 4 * (t->by & ~ss_ver)) >> ss_ver,
                                   edge, dst, stride);
'''),
    # chroma coef + itx (intra blocks; the same text also appears in the inter path -> both hooked)
    ('''                                dsp->itx.itxfm_add[b->uvtx]
                                                  [txtp](dst, stride,
                                                         cf, eob HIGHBD_CALL_SUFFIX);
''',
     '''                                trace_coef_block(1 + pl, t->bx >> ss_hor, t->by >> ss_ver, b->uvtx, txtp, eob, cf, uv_t_dim->w * 4, uv_t_dim->h * 4);
                                dsp->itx.itxfm_add[b->uvtx]
                                                  [txtp](dst, stride,
                                                         cf, eob HIGHBD_CALL_SUFFIX);
                                trace_recon(1 + pl, t->bx >> ss_hor, t->by >> ss_ver, uv_t_dim->w * 4, uv_t_dim->h * 4, b->uvtx, txtp, dst, stride);
'''),
])
patch(src / "ipred_prepare_tmpl.c", [
    ('#include "src/ipred_prepare.h"\n', '#include "src/ipred_prepare.h"\n#include "src/trace.h"\n'),
    ('''        if (mode == Z2_PRED && tw + th >= 6 && filter_edge)
            *topleft_out = ((topleft_out[-1] + topleft_out[1]) * 5 +
                            topleft_out[0] * 6 + 8) >> 4;''',
     '''        dav1d_trace_corner_raw = *topleft_out;
        if (mode == Z2_PRED && tw + th >= 6 && filter_edge)
            *topleft_out = ((topleft_out[-1] + topleft_out[1]) * 5 +
                            topleft_out[0] * 6 + 8) >> 4;'''),
], 1)
patch(src / "decode.c", [
    ('#include "src/decode.h"\n', '#include "src/decode.h"\n#include "src/trace.h"\n'),
    ('''    dav1d_msac_init(&ts->msac, data, sz, f->frame_hdr->disable_cdf_update);
''',
     '''    dav1d_msac_init(&ts->msac, data, sz, f->frame_hdr->disable_cdf_update);

    if (DAV1D_TRACE_ON(DAV1D_TRACE_SYM)) {
        /* openav1: frame header fields the tile syntax depends on, plus this tile's bounds (4x4 units) */
        const Dav1dFrameHeader *const h = f->frame_hdr;
        const Dav1dSequenceHeader *const sh = f->seq_hdr;
        fprintf(dav1d_trace_fp, "H %d %d %d %d %d %d", tile_row, tile_col,
                col_sb_start << sb_shift, imin(col_sb_end << sb_shift, f->bw),
                row_sb_start << sb_shift, imin(row_sb_end << sb_shift, f->bh));
        fprintf(dav1d_trace_fp, " %d %d %d %d %d %d %d %d %d %d", f->bw, f->bh, f->cur.p.bpc, (int) f->cur.p.layout,
                sh->sb128, (int) h->frame_type, h->primary_ref_frame, (int) h->txfm_mode, h->reduced_txtp_set,
                h->disable_cdf_update);
        fprintf(dav1d_trace_fp, " %d %d %d %d %d %d %d %d %d %d", h->quant.yac, h->quant.ydc_delta, h->quant.udc_delta,
                h->quant.uac_delta, h->quant.vdc_delta, h->quant.vac_delta, h->quant.qm, h->quant.qm_y, h->quant.qm_u,
                h->quant.qm_v);
        fprintf(dav1d_trace_fp, " %d %d %d %d %d", h->delta.q.present, h->delta.q.res_log2, h->delta.lf.present,
                h->delta.lf.res_log2, h->delta.lf.multi);
        fprintf(dav1d_trace_fp, " %d %d %d %d %d %d %d %d", h->allow_screen_content_tools, h->allow_intrabc,
                sh->filter_intra, sh->intra_edge_filter, sh->cdef, h->cdef.damping, h->cdef.n_bits, h->all_lossless);
        fprintf(dav1d_trace_fp, " %d %d %d %d %d", (int) h->restoration.type[0], (int) h->restoration.type[1],
                (int) h->restoration.type[2], h->restoration.unit_size[0], h->restoration.unit_size[1]);
        fprintf(dav1d_trace_fp, " %d %d %d %d %d %d %d", h->width[0], h->width[1], h->height, h->super_res.enabled,
                h->super_res.width_scale_denominator, sh->ss_hor, sh->ss_ver);
        fprintf(dav1d_trace_fp, " %d %d %d %d %d", h->segmentation.enabled, h->segmentation.update_map,
                h->segmentation.temporal, h->segmentation.seg_data.preskip, h->segmentation.seg_data.last_active_segid);
        for (int s = 0; s < 8; s++) {
            const Dav1dSegmentationData *const d = &h->segmentation.seg_data.d[s];
            fprintf(dav1d_trace_fp, " %d %d %d %d %d %d %d %d %d %d", h->segmentation.lossless[s],
                    h->segmentation.qidx[s], d->delta_q, d->delta_lf_y_v, d->delta_lf_y_h, d->delta_lf_u,
                    d->delta_lf_v, d->ref, d->skip, d->globalmv);
        }
        /* loop filter: level_y[2] level_u level_v sharpness mode_ref_delta_enabled ref_delta[8] mode_delta[2] */
        fprintf(dav1d_trace_fp, " %d %d %d %d %d %d", h->loopfilter.level_y[0], h->loopfilter.level_y[1],
                h->loopfilter.level_u, h->loopfilter.level_v, h->loopfilter.sharpness,
                h->loopfilter.mode_ref_delta_enabled);
        for (int i = 0; i < 8; i++) fprintf(dav1d_trace_fp, " %d", h->loopfilter.mode_ref_deltas.ref_delta[i]);
        for (int i = 0; i < 2; i++) fprintf(dav1d_trace_fp, " %d", h->loopfilter.mode_ref_deltas.mode_delta[i]);
        /* cdef strengths: y_strength[8] uv_strength[8] (only the first 1 << n_bits are meaningful) */
        for (int i = 0; i < 8; i++) fprintf(dav1d_trace_fp, " %d", h->cdef.y_strength[i]);
        for (int i = 0; i < 8; i++) fprintf(dav1d_trace_fp, " %d", h->cdef.uv_strength[i]);
        fputc('\n', dav1d_trace_fp);
    }
'''),
], 1)
print("patched", root)
