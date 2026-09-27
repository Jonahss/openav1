// Shared types for the tile syntax decoder.
package syn_pkg;
    // Frame/tile-level parameters the syntax needs (software parses the headers and fills this in).
    typedef struct packed {
        logic [10:0] mi_rows;
        logic [10:0] mi_cols;
        logic [10:0] mi_row_start;
        logic [10:0] mi_row_end;
        logic [10:0] mi_col_start;
        logic [10:0] mi_col_end;
        logic        ssx;
        logic        ssy;
        logic        mono;
        logic        sb128;
        logic [3:0]  bit_depth;
        logic        seg_enabled;
        logic        seg_preskip;              // SegIdPreSkip
        logic [2:0]  last_active_segid;
        logic [7:0]  seg_skip_en;              // FeatureEnabled[s][SEG_LVL_SKIP]
        logic [7:0]  lossless;                 // LosslessArray[s]
        logic [63:0] seg_qidx;                 // get_qindex(1, s) per segment, 8 x 8 bits
        logic [7:0]  base_q_idx;
        logic [1:0]  tx_mode;                  // 0 ONLY_4X4, 1 LARGEST, 2 SELECT
        logic        reduced_tx_set;
        logic        allow_sct;                // allow_screen_content_tools (palette: not yet supported)
        logic        enable_filter_intra;
        logic        enable_cdef;
        logic [1:0]  cdef_bits;
        logic        coded_lossless;
        logic        delta_q_present;
        logic [1:0]  delta_q_res;
        logic        delta_lf_present;
        logic [1:0]  delta_lf_res;
        logic        delta_lf_multi;
        logic        disable_cdf_update;
        logic        lr_any;                   // any FrameRestorationType != NONE (read_lr: not yet supported)
    } hdr_t;

    // Per-block record emitted by the syntax decoder.
    typedef struct packed {
        logic [10:0] r;
        logic [10:0] c;
        logic [4:0]  bsize;
        logic        skip;
        logic [2:0]  seg;
        logic        lossless;
        logic        has_chroma;
        logic [3:0]  ymode;
        logic [3:0]  uvmode;
        logic signed [2:0] angle_y;
        logic signed [2:0] angle_uv;
        logic signed [4:0] cfl_u;
        logic signed [4:0] cfl_v;
        logic        use_fi;
        logic [2:0]  fi_mode;
        logic [4:0]  txsz;
        logic [7:0]  qidx;                     // CurrentQIndex after this block's delta
        logic [27:0] delta_lf;                 // 4 x signed 7 bits
        logic        cdef_valid;
        logic [2:0]  cdef_idx;
        logic [3:0]  cdef_units;               // 64x64 units of the superblock this cdef_idx applies to
    } blk_rec_t;

    // Per-transform-block record; Quant is read through coef_rd's port while tx_done is held.
    typedef struct packed {
        logic [1:0]  plane;
        logic [12:0] x;                        // start pixel (plane units)
        logic [12:0] y;
        logic [4:0]  txsz;
        logic [3:0]  txtype;
        logic [10:0] eob;
        logic        skip;                     // block skip: no coefficients
        logic        lossless;
    } tx_rec_t;
endpackage
