// decode_block( ) for intra frames (spec 5.11.5 .. 5.11.39 minus coeffs, which coef_rd owns):
// intra_frame_mode_info -> palette_mode_info (pal_syntax) -> filter_intra -> palette_tokens (pal_syntax)
// -> tx size -> residual loop; or, with use_intrabc, intra block copy: find_mv_stack (mv_stack) + read_mv,
// the inter transform-size tree (read_var_tx_size), inter transform types and the transform_tree residual
// walk (leaf list). One symbol at a time through the shared symbol sequencer; neighbour
// contexts from blk_ctx. A block with a palette is held (pal_hold) after its record is complete until
// blk_ack, so the colour index map can be read through pm_* before the next block overwrites it.
module blk_syntax
  import cdf_map_pkg::*;
  import blk_tables_pkg::*;
  import tx_tables_pkg::*;
  import syn_pkg::*;
(
    input  logic        clk,
    input  logic        rst,
    input  hdr_t        hdr,
    input  ihdr_t       ihdr,
    // control
    input  logic        sb_start,             // superblock start: ReadDeltas := delta_q_present, cdef flags := 0
    input  logic        tile_start,           // CurrentQIndex := base_q_idx, DeltaLF := 0
    input  logic        start,
    input  logic [MIW-1:0] r, c,
    input  logic [4:0]  bsize,
    output logic        busy,
    output logic        blk_info,             // blk_rec valid: pulsed before the block's transform blocks
    output logic        blk_done,
    output blk_rec_t    blk_rec,
    output logic        unsupported,          // (sticky; nothing sets it at present)
    // transform block records
    output logic        tx_done,              // held until tx_ack
    output tx_rec_t     tx_rec,
    input  logic        tx_ack,
    input  logic        blk_ack,              // releases a palette block held on pal_hold
    output logic        pal_hold,
    // palette: neighbour palettes in, this block's palette out (to blk_ctx), colour map read port
    input  logic [3:0]  a_pal_y, l_pal_y, a_pal_uv, l_pal_uv,
    input  logic [95:0] a_col_y, l_col_y, a_col_u, l_col_u,
    output logic [3:0]  w_pal_y, w_pal_uv,
    output logic [95:0] w_col_y, w_col_u,
    input  logic        pm_plane,
    input  logic [5:0]  pm_x, pm_y,
    output logic [2:0]  pm_idx,
    // symbol sequencer master
    output logic              sq_go,
    output logic [CDF_AW-1:0] sq_addr,
    output logic [3:0]        sq_n,
    output logic [1:0]        sq_kind,
    input  logic              sq_done,
    input  logic [3:0]        sq_sym,
    // blk_ctx
    output logic        nb_req,
    input  logic        nb_valid,
    input  logic        avail_u, avail_l, has_chroma,
    input  logic [3:0]  a_ymode, l_ymode,
    input  logic        a_skip, l_skip,
    input  logic [4:0]  a_txsz, l_txsz,
    input  logic [4:0]  a_misize, l_misize,
    input  logic        a_is_inter, l_is_inter,
    input  logic [16*24*2-1:0] a_recs, l_recs,
    input  logic [3:0]  seg_ul, seg_u, seg_l,
    output logic        ctx_we,
    output logic [3:0]  w_ymode,
    output logic        w_skip,
    output logic [2:0]  w_seg,
    output logic [4:0]  w_txsz,
    output logic        w_is_inter,
    output logic        w_vartx,
    output logic [159:0] w_txsz_col, w_txsz_row,
    input  logic        ctx_wbusy,
    output logic        tx_req,
    output logic [1:0]  tx_plane,
    output logic [MIW-1:0] tx_x4, tx_y4,
    output logic [4:0]  tx_sz_o,
    output logic [4:0]  tx_bsize,
    input  logic        tx_valid,
    input  logic [3:0]  az_ctx,
    input  logic [1:0]  dcs_ctx,
    output logic        tx_we,
    output logic [5:0]  w_cul,
    output logic [1:0]  w_dccat,
    output logic        rbc_we,
    input  logic        ctx_tbusy,
    // coef_rd
    output logic [4:0]  cf_tx,
    output logic        cf_ptype,
    output logic        cf_start_a,
    output logic [3:0]  cf_az_ctx,
    input  logic        cf_done_a,
    input  logic        cf_all_zero,
    output logic        cf_start_b,
    output logic [3:0]  cf_tx_type,
    output logic [1:0]  cf_dcs_ctx,
    input  logic        cf_done_b,
    input  logic [10:0] cf_eob,
    input  logic [5:0]  cf_cul,
    input  logic [1:0]  cf_dccat,
    // motion info memory (mv_mem): this block's entry out, neighbours in (for mv_stack)
    output logic        mvm_we,
    output logic [MIW-1:0] mvm_r, mvm_c,
    output logic [5:0]  mvm_bw4, mvm_bh4,
    output mv_ent_t     mvm_data,
    input  logic        mvm_busy,
    output logic [MIW-1:0] mvr_row, mvr_col,
    input  mv_ent_t     mvr_ent,
    input  logic        mvr_written,
    // motion field band read port (temporal MV candidates)
    output logic [2:0]  mf_ref,
    output logic [MIW-2:0] mf_y8, mf_x8,
    input  logic        mf_valid,
    input  logic signed [15:0] mf_row, mf_col,
    // segmentation map band read port (PrevSegmentIds, 16 ids per word; 1-cycle latency)
    output logic [MIW-1:0] sp_row,
    output logic [MIW-5:0] sp_wc,
    input  logic [47:0] sp_data
);
    localparam logic [4:0] BLOCK_4X4 = 5'd0, BLOCK_8X8 = 5'd3, BLOCK_64X64 = 5'd12;
    localparam logic [3:0] DC_PRED = 4'd0, UV_CFL_PRED = 4'd13;
    localparam logic [4:0] TX_4X4 = 5'd0, TX_16X16 = 5'd2, TX_32X32 = 5'd3, TX_64X64 = 5'd4, TX_16X32 = 5'd9, TX_32X16 = 5'd10;

    typedef enum logic [7:0] {
        S_IDLE, S_NB,
        S_SEG_A, S_SEG_A_W, S_SKIP, S_SKIP_GO, S_SKIP_W, S_SEG_B, S_SEG_B_GO, S_SEG_B_W,
        S_CDEF, S_CDEF_W,
        S_DQ, S_DQ_GO, S_DQ_W, S_DQ_REM_W, S_DQ_ABS_W, S_DQ_SIGN, S_DQ_SIGN_W,
        S_DLF, S_DLF_GO, S_DLF_W, S_DLF_REM_W, S_DLF_ABS_W, S_DLF_SIGN, S_DLF_SIGN_W, S_DLF_NEXT,
        S_YMODE, S_YMODE_W, S_ANGY, S_ANGY_GO, S_ANGY_W, S_UV, S_UV_W, S_CFLS, S_CFLS_W, S_CFLU, S_CFLU_GO, S_CFLU_W,
        S_CFLV, S_CFLV_GO, S_CFLV_W, S_ANGUV, S_ANGUV_GO, S_ANGUV_W, S_FI, S_FI_GO, S_FI_W, S_FIM, S_FIM_W,
        S_TXD, S_TXD_GO, S_TXD_W, S_RBC,
        S_RES_PLANE, S_RES_TX, S_TX_CTX_W, S_TX_A, S_TX_A_W, S_TXTYPE, S_TXTYPE_W, S_TX_B, S_TX_B_W, S_TX_UPD, S_TX_EMIT,
        S_RES_NEXT, S_END, S_END_W, S_DONE,
        S_PAL, S_PAL_W, S_PTOK, S_PTOK_W, S_HOLD, S_IBC, S_IBC_GO, S_IBC_W,
        S_MVS, S_MVS_W, S_MVJ, S_MVJ_W, S_MVC, S_MVSGN, S_MVSGN_W, S_MVCLS, S_MVCLS_W, S_MVC0B, S_MVC0B_W,
        S_MVBIT, S_MVBIT_W, S_MVMAG, S_MVEND,
        S_VTX_INIT, S_VTX_NODE, S_VTX_GO, S_VTX_W, S_VTX_LEAF, S_VTX_POP, S_RES_LEAF, S_RES_PLANE_END, S_TX_UVT,
        S_LIT, S_LIT_W,
        // inter frames (5.11.7 inter_frame_mode_info and 5.11.23 .. 5.11.33)
        S_NBA, S_NBA_W, S_NBL, S_NBL_W, S_ISEG, S_SP_RD, S_SP_W, S_SP_END, S_SEGP_GO, S_SEGP_W,
        S_SKM, S_SKM_GO, S_SKM_W, S_ISI, S_ISI_GO, S_ISI_W, S_RF, S_RFS_GO, S_RFS_W,
        S_MODE, S_CMODE_GO, S_CMODE_W, S_NEWMV_GO, S_NEWMV_W, S_ZEROMV_GO, S_ZEROMV_W, S_REFMV_GO, S_REFMV_W,
        S_DRL, S_DRL_IT, S_DRL_GO, S_DRL_W, S_AMV, S_AMV_NEXT, S_MVFR, S_MVFR_GO, S_MVFR_W, S_MVHP, S_MVHP_GO, S_MVHP_W,
        S_II, S_II_GO, S_II_W, S_IIM_GO, S_IIM_W, S_WII_GO, S_WII_W, S_WIDX_GO, S_WIDX_W,
        S_MM, S_OV, S_OV_W, S_OV_P, S_WS, S_WS_W, S_WS_P, S_MM2, S_OBMC_GO, S_OBMC_W, S_MMS_GO, S_MMS_W,
        S_CT, S_CGI_GO, S_CGI_W, S_CT2, S_CIDX_GO, S_CIDX_W, S_CTYPE_GO, S_CTYPE_W, S_CT3, S_WIDX2_GO, S_WIDX2_W, S_WSGN_W, S_MASK_W,
        S_IF, S_IF_D, S_IF_GO, S_IF_W, S_IF_END, S_NBW
    } st_t;
    st_t st, lit_ret;
    // cycle counter per state (simulation profiling; read through the hierarchy by the testbench)
    logic [31:0] perf_st [0:255];
    always_ff @(posedge clk) begin
        if (rst) perf_st <= '{default: 32'd0};
        else perf_st[st] <= perf_st[st] + 32'd1;
    end

    // ---------------------------------------------------------------- block registers
    logic [MIW-1:0] br, bc;
    logic [4:0]  bs;
    logic [5:0]  bw4, bh4;
    logic [7:0]  bwp, bhp;                     // pixels
    logic        skip, lossless, hc;
    logic [2:0]  seg;
    logic [3:0]  ymode, uvmode;
    logic signed [2:0] ang_y, ang_uv;
    logic signed [5:0] cfl_u, cfl_v;
    logic        use_fi;
    logic [2:0]  fi_mode;
    logic [4:0]  txsz;
    logic [2:0]  cfl_signs;
    logic [1:0]  sign_u, sign_v;
    // persistent across blocks
    logic [7:0]  cur_qidx;
    logic signed [6:0] dlf [4];
    logic        read_deltas;
    logic [3:0]  cdef_flags;
    logic [2:0]  cdef_val;
    logic        cdef_valid;
    logic [3:0]  cdef_units;
    // literal reader
    logic [3:0]  lit_n;
    logic [15:0] lit_val;
    // delta q / lf scratch
    logic [3:0]  rem_bits;
    logic [9:0]  d_abs;
    logic [1:0]  dlf_i, dlf_cnt;
    // residual loop
    logic        chunk_x, chunk_y, wchunks2, hchunks2;
    logic [1:0]  plane;
    logic [4:0]  p_txsz;
    logic [5:0]  step_x, step_y, n4w, n4h, rx, ry;
    logic [PXW-1:0] base_x, base_y, start_x, start_y, max_x, max_y;
    logic        sub_x, sub_y;
    logic [3:0]  p_txtype;
    logic [10:0] t_eob;
    logic        all_zero;
    logic [4:0]  chunk_bs;
    // intra block copy
    logic        use_ibc;
    logic signed [17:0] mv_row, mv_col, diff_row, diff_col;
    logic signed [17:0] pred_row, pred_col;
    logic        mvs_start, mvs_busy, mvs_done;
    logic [1:0]  mvj, mv_comp;
    logic        mv_sgn, mv_c0b;
    logic [3:0]  mv_cls, mv_bi;
    logic [9:0]  mv_d;
    // inter frames
    logic        inter_frm;                                 // ihdr.is_inter
    logic        is_inter_b;                                // this block is an inter block (not intra block copy)
    logic        inter_any;                                 // inter block or intra block copy
    logic        skip_mode, is_comp, seg_pre, seg_pred_sym, sp_mode;
    logic [3:0]  rf0, rf1;                                  // RefFrame[0..1] (15 = NONE)
    logic [4:0]  imode;                                     // YMode of an inter block (13 .. 24)
    logic [3:0]  rf_step;
    logic        mv_list;                                   // list being assigned (assign_mv)
    logic [1:0]  ref_mv_idx, drl_i, drl_end;
    logic signed [17:0] mv1_row, mv1_col, pr_r, pr_c;
    logic [1:0]  motion_mode, ii_mode;
    logic [2:0]  compound_type;
    logic        interintra, wedge_ii, wedge_sign, mask_type, comp_group_idx, compound_idx, interp_d;
    logic [3:0]  wedge_idx;
    logic [1:0]  interp [2];
    logic [1:0]  mv_fr;
    logic        mv_hp, mv_int, mv_hp_en;
    st_t         seg_ret;
    mv_ent_t     a_ent, l_ent;                              // neighbours (r - 1, c) and (r, c - 1)
    logic        a_ok, l_ok;                                // ... available and written
    logic        nb_rd;                                     // blk_syntax owns the mv_mem read port (else the stack)
    logic [MIW-1:0] nb_row, nb_col, mvs_rd_row, mvs_rd_col;
    // segment map band scan (get_segment_id: the minimum over the block)
    logic [2:0]  sp_min;
    logic [5:0]  sp_y;
    logic [MIW-5:0] sp_w, sp_w0, sp_w_end;
    logic [5:0]  sp_xmis, sp_ymis;
    // overlappable / warp-sample scans
    logic        ov_above, ws_found, do_tl, do_tr;
    logic [MIW-1:0] ov_x, ov_y;
    logic [2:0]  ws_ph;                                     // 0 above (r-1, c), 1 above loop, 2 left (r, c-1), 3 left loop, 4 TL, 5 TR
    logic [5:0]  ws_i;
    // transform-size tree (read_var_tx_size) walked with an explicit node stack, leaves listed in decode
    // order (= transform_tree order: both are z-order over the same quadtree)
    logic        vt_read;                                   // txfm_split symbols are read
    logic [5:0]  vt_ur, vt_uc, vt_uw4, vt_uh4;              // max-tx unit origin / size (block-relative 4x4)
    logic [5:0]  nd_r [3], nd_c [3];
    logic [4:0]  nd_tx [3];
    logic [1:0]  nd_ch [3];
    logic        nd_two_c [3], nd_two_r [3];                // children per row / column: 2 or 1
    logic [1:0]  vt_d;
    logic [4:0]  top_edge [32], left_edge [32];             // InterTxSizes at the walk's current bottom / right edge
    logic [5:0]  leaf_c [64], leaf_r [64];
    logic [4:0]  leaf_tx [64];
    logic [3:0]  leaf_tt [64];
    logic [6:0]  n_leaves, leaf_rd, uvt_k;
    logic        leaf_mode;

    // ---------------------------------------------------------------- helpers
    function automatic logic [2:0] neg_deinterleave(input logic [3:0] diff, input logic [3:0] rf, input logic [3:0] mx);
        if (rf == 0) neg_deinterleave = diff[2:0];
        else if (rf >= mx - 4'd1) neg_deinterleave = 3'(mx - diff - 4'd1);
        else if (5'(rf) * 2 < 5'(mx)) begin
            if (5'(diff) <= 5'(rf) * 2) neg_deinterleave = diff[0] ? 3'(rf + ((diff + 4'd1) >> 1)) : 3'(rf - (diff >> 1));
            else neg_deinterleave = diff[2:0];
        end else begin
            if (5'(diff) <= 5'(mx - rf - 4'd1) * 2) neg_deinterleave = diff[0] ? 3'(rf + ((diff + 4'd1) >> 1)) : 3'(rf - (diff >> 1));
            else neg_deinterleave = 3'(mx - (diff + 4'd1));
        end
    endfunction

    function automatic logic is_dir(input logic [3:0] m);
        is_dir = (m >= 4'd1) && (m <= 4'd8);
    endfunction

    // segment id prediction / ctx (seg_* are 4'hF when unavailable)
    logic [3:0] seg_pred;
    logic [1:0] seg_ctx;
    always_comb begin
        if (seg_u == 4'hF) seg_pred = (seg_l == 4'hF) ? 4'd0 : seg_l;
        else if (seg_l == 4'hF) seg_pred = seg_u;
        else seg_pred = (seg_ul == seg_u) ? seg_u : seg_l;
        if (seg_ul == 4'hF) seg_ctx = 2'd0;
        else if (seg_ul == seg_u && seg_ul == seg_l) seg_ctx = 2'd2;
        else if (seg_ul == seg_u || seg_ul == seg_l || seg_u == seg_l) seg_ctx = 2'd1;
        else seg_ctx = 2'd0;
    end

    // tx set / tx type helpers
    function automatic logic [1:0] tx_set(input logic [4:0] t, input logic reduced);
        if (tx_sqr_up(t) >= 3'd3) tx_set = 2'd0;          // TX_32X32 and up: DCT only
        else if (reduced) tx_set = 2'd2;
        else if (tx_sqr(t) == 3'd2) tx_set = 2'd2;        // 16x16
        else tx_set = 2'd1;
    endfunction
    function automatic logic [1:0] tx_set_inter(input logic [4:0] t, input logic reduced);
        if (tx_sqr_up(t) > 3'd3) tx_set_inter = 2'd0;           // TX_64: DCT only
        else if (reduced || tx_sqr_up(t) == 3'd3) tx_set_inter = 2'd3;
        else if (tx_sqr(t) == 3'd2) tx_set_inter = 2'd2;
        else tx_set_inter = 2'd1;
    endfunction
    logic [1:0] cur_set, cur_set_i, cur_set_eff;
    logic [7:0] cur_qidx_seg;
    logic [3:0] intra_dir;
    always_comb begin
        cur_set = tx_set(p_txsz, hdr.reduced_tx_set);
        cur_set_i = tx_set_inter(p_txsz, hdr.reduced_tx_set);
        cur_set_eff = inter_any ? cur_set_i : cur_set;
        cur_qidx_seg = hdr.seg_enabled ? hdr.seg_qidx[8*seg +: 8] : hdr.base_q_idx;
        intra_dir = use_fi ? filter_intra_mode_to_intra_dir(fi_mode) : ymode;
    end

    // uv tx size for chroma (get_tx_size)
    function automatic logic [4:0] uv_tx(input logic [4:0] b, input logic sx, input logic sy);
        logic [4:0] t;
        t = max_tx_size_rect(subsampled_size(b, sx, sy));
        if (tx_width(t) == 7'd64 || tx_height(t) == 7'd64) begin
            if (tx_width(t) == 7'd16) uv_tx = TX_16X32;
            else if (tx_height(t) == 7'd16) uv_tx = TX_32X16;
            else uv_tx = TX_32X32;
        end else uv_tx = t;
    endfunction

    // depth-dependent tx cdf
    logic [4:0] max_rect;
    logic [2:0] max_depth;
    logic [1:0] txd_ctx;
    logic [7:0] above_w, left_h;                 // 8 bits: an inter neighbour's block size can be 128
    always_comb begin
        max_rect = max_tx_size_rect(bs);
        max_depth = max_tx_depth(bs);
        // unavailable neighbours count as width/height 0 (tile_model.tx_depth_cdf, matches dav1d); inter
        // neighbours count with their block size
        above_w = avail_u ? (a_is_inter ? blk_w(a_misize) : 8'(tx_width(a_txsz))) : 8'd0;
        left_h  = avail_l ? (l_is_inter ? blk_h(l_misize) : 8'(tx_height(l_txsz))) : 8'd0;
        txd_ctx = 2'(above_w >= 8'(tx_width(max_rect))) + 2'(left_h >= 8'(tx_height(max_rect)));
    end
    logic uv_cfl_allowed;
    always_comb begin
        if (lossless) uv_cfl_allowed = (subsampled_size(bs, hdr.ssx, hdr.ssy) == BLOCK_4X4);
        else uv_cfl_allowed = (bwp <= 8'd32) && (bhp <= 8'd32);
    end
    logic [4:0] sb_bs;
    assign sb_bs = sb_size_bsize(hdr.sb128);
    logic [3:0] sbm4;                            // superblock 4x4 mask
    assign sbm4 = hdr.sb128 ? 4'd15 : 4'd15;     // (unused; cdef units are 64x64 in both cases)
    logic [1:0] cdef_unit;
    assign cdef_unit = {br[4] & hdr.sb128, bc[4] & hdr.sb128};

    // ---------------------------------------------------------------- palette sub-decoder
    logic        pal_clr, pal_start_mi, pal_start_tok, pal_done_mi, pal_done_tok;
    logic [3:0]  pal_y, pal_uv;
    logic [95:0] col_y, col_u, col_v;
    logic        pq_go; logic [CDF_AW-1:0] pq_addr; logic [3:0] pq_n; logic [1:0] pq_kind;
    logic [PXW-1:0] rem_w, rem_h;
    logic [7:0]  os_w, os_h;                     // onscreen block dims (luma pixels)
    always_comb begin
        rem_w = (PXW'(hdr.mi_cols) - PXW'(bc)) << 2;
        rem_h = (PXW'(hdr.mi_rows) - PXW'(br)) << 2;
        os_w = (rem_w < PXW'(bwp)) ? rem_w[7:0] : bwp;
        os_h = (rem_h < PXW'(bhp)) ? rem_h[7:0] : bhp;
    end
    pal_syntax u_pal (.clk, .rst, .hdr, .clr(pal_clr),
                      .start_mi(pal_start_mi), .bs, .ymode, .uvmode, .hc, .avail_u, .avail_l, .br,
                      .a_pal_y, .l_pal_y, .a_pal_uv, .l_pal_uv, .a_col_y, .l_col_y, .a_col_u, .l_col_u, .done_mi(pal_done_mi),
                      .start_tok(pal_start_tok), .os_w, .os_h, .done_tok(pal_done_tok),
                      .pal_y, .pal_uv, .col_y, .col_u, .col_v,
                      .pm_plane, .pm_x, .pm_y, .pm_idx,
                      .sq_go(pq_go), .sq_addr(pq_addr), .sq_n(pq_n), .sq_kind(pq_kind), .sq_done, .sq_sym);
    assign w_pal_y = pal_y; assign w_pal_uv = pal_uv; assign w_col_y = col_y; assign w_col_u = col_u;
    assign pal_hold = (st == S_HOLD);
    logic pal_allowed, has_pal;
    assign pal_allowed = (bs >= BLOCK_8X8) && (bwp <= 8'd64) && (bhp <= 8'd64) && hdr.allow_sct;
    assign has_pal = (pal_y != 4'd0) || (pal_uv != 4'd0);

    // ---------------------------------------------------------------- MV stack (7.10.2) for intra block copy
    // stack outputs of inter blocks (stage 1: the inter chain reads them; intra block copy uses pred_*)
    logic [3:0] mvs_n; logic signed [17:0] mvs_st0_row [4], mvs_st0_col [4], mvs_st1_row [4], mvs_st1_col [4], mvs_gmv_row [2], mvs_gmv_col [2];
    logic [2:0] mvs_new_ctx, mvs_ref_ctx; logic mvs_zero_ctx; logic [1:0] mvs_drl [3];
    mv_stack u_mvs (.clk, .rst, .hdr, .ihdr, .start(mvs_start), .r(br), .c(bc), .bs, .ref0(rf0), .ref1(rf1), .is_comp,
                    .busy(mvs_busy), .done(mvs_done), .pred_row, .pred_col, .n_found(mvs_n),
                    .st0_row(mvs_st0_row), .st0_col(mvs_st0_col), .st1_row(mvs_st1_row), .st1_col(mvs_st1_col), .gmv_row(mvs_gmv_row), .gmv_col(mvs_gmv_col),
                    .new_mv_ctx(mvs_new_ctx), .ref_mv_ctx(mvs_ref_ctx), .zero_mv_ctx(mvs_zero_ctx), .drl_ctx(mvs_drl),
                    .rd_row(mvs_rd_row), .rd_col(mvs_rd_col), .rd_ent(mvr_ent), .rd_written(mvr_written),
                    .mf_ref, .mf_y8, .mf_x8, .mf_valid, .mf_row, .mf_col);
    assign nb_rd = (st == S_NBA) || (st == S_NBA_W) || (st == S_NBL) || (st == S_NBL_W) || (st == S_OV_W) || (st == S_OV_P) || (st == S_WS_W) || (st == S_WS_P);
    always_comb begin
        if (st == S_NBA || st == S_NBA_W) begin mvr_row = br - MIW'(1); mvr_col = bc; end
        else if (st == S_NBL || st == S_NBL_W) begin mvr_row = br; mvr_col = bc - MIW'(1); end
        else if (nb_rd) begin mvr_row = nb_row; mvr_col = nb_col; end
        else begin mvr_row = mvs_rd_row; mvr_col = mvs_rd_col; end
    end
    assign inter_frm = ihdr.is_inter;
    assign inter_any = use_ibc | is_inter_b;
    assign mv_int = use_ibc | ihdr.force_integer_mv;              // intra frames read whole-sample MVs (fr = 3, hp = 1)
    assign mv_hp_en = !use_ibc && ihdr.allow_hp_mv;
    assign sp_row = br + MIW'(sp_y);
    assign sp_wc = sp_w;

    // ---------------------------------------------------------------- inter contexts (8.3.2) from the two neighbours
    localparam logic [3:0] R_INTRA = 4'd0, R_LAST = 4'd1, R_LAST2 = 4'd2, R_LAST3 = 4'd3, R_GOLDEN = 4'd4, R_BWDREF = 4'd5, R_ALTREF2 = 4'd6, R_ALTREF = 4'd7, R_NONE = 4'd15;
    localparam logic [4:0] NEARESTMV = 5'd13, NEARMV = 5'd14, GLOBALMV = 5'd15, NEWMV = 5'd16, NEAREST_NEARESTMV = 5'd17, NEAR_NEARMV = 5'd18,
                           NEAREST_NEWMV = 5'd19, NEW_NEARESTMV = 5'd20, NEAR_NEWMV = 5'd21, NEW_NEARMV = 5'd22, GLOBAL_GLOBALMV = 5'd23, NEW_NEWMV = 5'd24;
    logic [3:0] a_r0, a_r1, l_r0, l_r1;
    logic a_intra, l_intra, a_single, l_single;
    logic [2:0] cnt [8];                                    // count_refs(t) over the two neighbours' two references
    logic [1:0] ctx_comp_ref, ctx_comp_ref_p1, ctx_comp_ref_p2, ctx_comp_bwdref, ctx_comp_bwdref_p1, ctx_single_ref_p1, ctx_uni_comp_ref_p1;
    logic [2:0] ctx_comp_mode, ctx_comp_ref_type, ctx_cgi, ctx_cidx;
    logic [1:0] ctx_is_inter, ctx_skm, ctx_segp;
    logic [3:0] ctx_interp;
    logic a_comp_inter, l_comp_inter, a_uni, l_uni, samedir, l_has, a_has;
    logic [3:0] cg_g, cg_ci;
    logic [1:0] if_lt, if_at, cm_a;
    logic [2:0] cm_b;
    function automatic logic [1:0] rcc(input logic [3:0] c0, input logic [3:0] c1);   // ref_count_ctx
        rcc = (c0 < c1) ? 2'd0 : (c0 == c1) ? 2'd1 : 2'd2;
    endfunction
    function automatic logic is_bwd(input logic [3:0] rf);
        is_bwd = (rf >= R_BWDREF) && (rf <= R_ALTREF);
    endfunction
    function automatic logic [1:0] gmt2(input logic [3:0] rf);
        gmt2 = (rf >= 4'd1 && rf <= 4'd7) ? ihdr.gm_type[2 * (rf - 4'd1) +: 2] : 2'd0;
    endfunction
    function automatic logic [7:0] ohint(input logic [3:0] rf);
        ohint = ihdr.order_hints[8 * (rf - 4'd1) +: 8];
    endfunction
    function automatic logic [7:0] rel_dist_abs(input logic [7:0] a, input logic [7:0] b);  // |get_relative_dist(a, b)|
        logic [7:0] diff, m;
        logic signed [8:0] rd;
        diff = a - b;
        m = 8'd1 << (ihdr.order_hint_bits - 3'd1);
        rd = 9'(signed'({1'b0, diff & (m - 8'd1)})) - 9'(signed'({1'b0, diff & m}));
        if (!ihdr.enable_order_hint) rd = 9'sd0;
        rel_dist_abs = 8'((rd < 0) ? -rd : rd);
    endfunction
    function automatic logic has_newmv(input logic [4:0] m);
        has_newmv = (m == NEWMV) || (m == NEW_NEWMV) || (m == NEAR_NEWMV) || (m == NEW_NEARMV) || (m == NEAREST_NEWMV) || (m == NEW_NEARESTMV);
    endfunction
    function automatic logic has_nearmv(input logic [4:0] m);
        has_nearmv = (m == NEARMV) || (m == NEAR_NEARMV) || (m == NEAR_NEWMV) || (m == NEW_NEARMV);
    endfunction
    function automatic logic [4:0] get_mode(input logic [4:0] y, input logic lst);
        if (!lst) begin
            if (y < NEAREST_NEARESTMV) get_mode = y;
            else if (y == NEW_NEWMV || y == NEW_NEARESTMV || y == NEW_NEARMV) get_mode = NEWMV;
            else if (y == NEAREST_NEARESTMV || y == NEAREST_NEWMV) get_mode = NEARESTMV;
            else if (y == NEAR_NEARMV || y == NEAR_NEWMV) get_mode = NEARMV;
            else get_mode = GLOBALMV;
        end else begin
            if (y == NEW_NEWMV || y == NEAREST_NEWMV || y == NEAR_NEWMV) get_mode = NEWMV;
            else if (y == NEAREST_NEARESTMV || y == NEW_NEARESTMV) get_mode = NEARESTMV;
            else if (y == NEAR_NEARMV || y == NEW_NEARMV) get_mode = NEARMV;
            else get_mode = GLOBALMV;
        end
    endfunction
    always_comb begin
        a_r0 = avail_u ? a_ent.ref0 : R_INTRA; a_r1 = avail_u ? a_ent.ref1 : R_NONE;
        l_r0 = avail_l ? l_ent.ref0 : R_INTRA; l_r1 = avail_l ? l_ent.ref1 : R_NONE;
        a_intra = (a_r0 == R_INTRA); l_intra = (l_r0 == R_INTRA);
        a_single = (a_r1 == R_INTRA) || (a_r1 == R_NONE); l_single = (l_r1 == R_INTRA) || (l_r1 == R_NONE);
        for (int t = 0; t < 8; t++)
            cnt[t] = 3'(avail_u && a_r0 == 4'(t)) + 3'(avail_u && a_r1 == 4'(t)) + 3'(avail_l && l_r0 == 4'(t)) + 3'(avail_l && l_r1 == 4'(t));
        ctx_comp_ref = rcc(4'(cnt[1]) + 4'(cnt[2]), 4'(cnt[3]) + 4'(cnt[4]));
        ctx_comp_ref_p1 = rcc(4'(cnt[1]), 4'(cnt[2]));
        ctx_comp_ref_p2 = rcc(4'(cnt[3]), 4'(cnt[4]));
        ctx_comp_bwdref = rcc(4'(cnt[5]) + 4'(cnt[6]), 4'(cnt[7]));
        ctx_comp_bwdref_p1 = rcc(4'(cnt[5]), 4'(cnt[6]));
        ctx_single_ref_p1 = rcc(4'(cnt[1]) + 4'(cnt[2]) + 4'(cnt[3]) + 4'(cnt[4]), 4'(cnt[5]) + 4'(cnt[6]) + 4'(cnt[7]));
        ctx_uni_comp_ref_p1 = rcc(4'(cnt[2]), 4'(cnt[3]) + 4'(cnt[4]));
        // comp_mode
        if (avail_u && avail_l) begin
            if (a_single && l_single) ctx_comp_mode = 3'(is_bwd(a_r0) ^ is_bwd(l_r0));
            else if (a_single) ctx_comp_mode = 3'd2 + 3'(is_bwd(a_r0) || a_intra);
            else if (l_single) ctx_comp_mode = 3'd2 + 3'(is_bwd(l_r0) || l_intra);
            else ctx_comp_mode = 3'd4;
        end else if (avail_u) ctx_comp_mode = a_single ? 3'(is_bwd(a_r0)) : 3'd3;
        else if (avail_l) ctx_comp_mode = l_single ? 3'(is_bwd(l_r0)) : 3'd3;
        else ctx_comp_mode = 3'd1;
        // comp_ref_type
        a_comp_inter = avail_u && !a_intra && !a_single;
        l_comp_inter = avail_l && !l_intra && !l_single;
        a_uni = a_comp_inter && ((a_r0 >= R_BWDREF) == (a_r1 >= R_BWDREF));
        l_uni = l_comp_inter && ((l_r0 >= R_BWDREF) == (l_r1 >= R_BWDREF));
        samedir = ((a_r0 >= R_BWDREF) == (l_r0 >= R_BWDREF));
        if (avail_u && !a_intra && avail_l && !l_intra) begin
            if (!a_comp_inter && !l_comp_inter) ctx_comp_ref_type = 3'd1 + 3'(samedir) * 3'd2;
            else if (!a_comp_inter) ctx_comp_ref_type = !l_uni ? 3'd1 : 3'd3 + 3'(samedir);
            else if (!l_comp_inter) ctx_comp_ref_type = !a_uni ? 3'd1 : 3'd3 + 3'(samedir);
            else if (!a_uni && !l_uni) ctx_comp_ref_type = 3'd0;
            else if (!a_uni || !l_uni) ctx_comp_ref_type = 3'd2;
            else ctx_comp_ref_type = 3'd3 + 3'((a_r0 == R_BWDREF) == (l_r0 == R_BWDREF));
        end else if (avail_u && avail_l) begin
            if (a_comp_inter) ctx_comp_ref_type = 3'd1 + 3'(a_uni) * 3'd2;
            else if (l_comp_inter) ctx_comp_ref_type = 3'd1 + 3'(l_uni) * 3'd2;
            else ctx_comp_ref_type = 3'd2;
        end else if (a_comp_inter) ctx_comp_ref_type = 3'(a_uni) * 3'd4;
        else if (l_comp_inter) ctx_comp_ref_type = 3'(l_uni) * 3'd4;
        else ctx_comp_ref_type = 3'd2;
        // is_inter
        if (avail_u && avail_l) ctx_is_inter = (l_intra && a_intra) ? 2'd3 : 2'(l_intra || a_intra);
        else if (avail_u || avail_l) ctx_is_inter = 2'd2 * 2'(avail_u ? a_intra : l_intra);
        else ctx_is_inter = 2'd0;
        // skip_mode, seg_id_predicted
        ctx_skm = 2'(avail_u && a_ent.skip_mode) + 2'(avail_l && l_ent.skip_mode);
        ctx_segp = 2'(avail_u && a_ent.seg_pred) + 2'(avail_l && l_ent.seg_pred);
        // comp_group_idx / compound_idx
        cg_g = 4'd0; cg_ci = (rel_dist_abs(ohint(rf0), ihdr.order_hint) == rel_dist_abs(ohint(rf1), ihdr.order_hint)) ? 4'd3 : 4'd0;
        if (avail_u) begin
            if (!a_single) begin cg_g = cg_g + 4'(a_ent.comp_group_idx); cg_ci = cg_ci + 4'(a_ent.compound_idx); end
            else if (a_r0 == R_ALTREF) begin cg_g = cg_g + 4'd3; cg_ci = cg_ci + 4'd1; end
        end
        if (avail_l) begin
            if (!l_single) begin cg_g = cg_g + 4'(l_ent.comp_group_idx); cg_ci = cg_ci + 4'(l_ent.compound_idx); end
            else if (l_r0 == R_ALTREF) begin cg_g = cg_g + 4'd3; cg_ci = cg_ci + 4'd1; end
        end
        ctx_cgi = (cg_g > 4'd5) ? 3'd5 : 3'(cg_g);
        ctx_cidx = 3'(cg_ci);
        // interp_filter (dir = interp_d)
        l_has = avail_l && (l_ent.ref0 == rf0 || l_ent.ref1 == rf0);
        a_has = avail_u && (a_ent.ref0 == rf0 || a_ent.ref1 == rf0);
        if_lt = l_has ? (interp_d ? l_ent.interp1 : l_ent.interp0) : 2'd3;
        if_at = a_has ? (interp_d ? a_ent.interp1 : a_ent.interp0) : 2'd3;
        ctx_interp = (4'(interp_d) * 4'd2 + 4'(is_comp)) * 4'd4;
        if (if_lt == if_at) ctx_interp = ctx_interp + 4'(if_lt);
        else if (if_lt == 2'd3) ctx_interp = ctx_interp + 4'(if_at);
        else if (if_at == 2'd3) ctx_interp = ctx_interp + 4'(if_lt);
        else ctx_interp = ctx_interp + 4'd3;
    end
    // the segmentation map band word: minimum of the ids of the block's columns in it
    logic [2:0] sp_word_min;
    always_comb begin
        sp_word_min = 3'd7;
        for (int k = 0; k < 16; k++) begin
            logic [MIW-1:0] gcol;
            gcol = {sp_w, 4'(k)};
            if (gcol >= bc && gcol < bc + MIW'(sp_xmis) && sp_data[3 * k +: 3] < sp_word_min) sp_word_min = sp_data[3 * k +: 3];
        end
    end
    // needs_interp_filter (5.11.23)
    logic big_blk, needs_if, globalmv_fam, newmv_fam;
    assign big_blk = (bwp >= 8'd8) && (bhp >= 8'd8);
    assign globalmv_fam = (imode == GLOBALMV) || (imode == GLOBAL_GLOBALMV);
    assign newmv_fam = has_newmv(imode);
    always_comb begin
        if (skip_mode || motion_mode == 2'd2) needs_if = 1'b0;
        else if (big_blk && imode == GLOBALMV) needs_if = (gmt2(rf0) == 2'd1);
        else if (big_blk && imode == GLOBAL_GLOBALMV) needs_if = (gmt2(rf0) == 2'd1) || (gmt2(rf1) == 2'd1);
        else needs_if = 1'b1;
    end
    // Compound_Mode_Ctx_Map[RefMvContext >> 1][Min(NewMvContext, COMP_NEWMV_CTXS - 1)]
    logic [2:0] ctx_cmode;
    always_comb begin
        cm_a = mvs_ref_ctx[2:1]; cm_b = (mvs_new_ctx > 3'd4) ? 3'd4 : mvs_new_ctx;
        case (cm_a)
            2'd0: ctx_cmode = (cm_b == 3'd0) ? 3'd0 : 3'd1;
            2'd1: ctx_cmode = (cm_b == 3'd0) ? 3'd1 : (cm_b == 3'd1) ? 3'd2 : (cm_b == 3'd2) ? 3'd3 : 3'd4;
            default: ctx_cmode = (cm_b <= 3'd1) ? 3'd4 : (cm_b == 3'd2) ? 3'd5 : (cm_b == 3'd3) ? 3'd6 : 3'd7;
        endcase
    end
    // assign_mv of list mv_list: the mode and the predicted MV
    logic [4:0] amode;
    logic [1:0] apos;
    logic signed [17:0] amv_r, amv_c;
    always_comb begin
        amode = get_mode(imode, mv_list);
        apos = (amode == NEARESTMV) ? 2'd0 : ref_mv_idx;
        if (amode == NEWMV && mvs_n <= 4'd1) apos = 2'd0;
        if (amode == GLOBALMV) begin amv_r = mvs_gmv_row[mv_list]; amv_c = mvs_gmv_col[mv_list]; end
        else if (!mv_list) begin amv_r = mvs_st0_row[apos]; amv_c = mvs_st0_col[apos]; end
        else begin amv_r = mvs_st1_row[apos]; amv_c = mvs_st1_col[apos]; end
    end
    // the reference-frame symbol of step rf_step (read_ref_frames 5.11.25)
    logic [CDF_AW-1:0] rfs_addr;
    always_comb begin
        case (rf_step)
            4'd0: rfs_addr = CDF_AW'(CDF_COMP_MODE + int'(ctx_comp_mode));
            4'd1: rfs_addr = CDF_AW'(CDF_COMP_REF_TYPE + int'(ctx_comp_ref_type));
            4'd2: rfs_addr = CDF_AW'(CDF_UNI_COMP_REF + int'(ctx_single_ref_p1) * CDF_UNI_COMP_REF_S0 + 0);
            4'd3: rfs_addr = CDF_AW'(CDF_UNI_COMP_REF + int'(ctx_uni_comp_ref_p1) * CDF_UNI_COMP_REF_S0 + 1);
            4'd4: rfs_addr = CDF_AW'(CDF_UNI_COMP_REF + int'(ctx_comp_ref_p2) * CDF_UNI_COMP_REF_S0 + 2);
            4'd5: rfs_addr = CDF_AW'(CDF_COMP_REF + int'(ctx_comp_ref) * CDF_COMP_REF_S0 + 0);
            4'd6: rfs_addr = CDF_AW'(CDF_COMP_REF + int'(ctx_comp_ref_p1) * CDF_COMP_REF_S0 + 1);
            4'd7: rfs_addr = CDF_AW'(CDF_COMP_REF + int'(ctx_comp_ref_p2) * CDF_COMP_REF_S0 + 2);
            4'd8: rfs_addr = CDF_AW'(CDF_COMP_BWD_REF + int'(ctx_comp_bwdref) * CDF_COMP_BWD_REF_S0 + 0);
            4'd9: rfs_addr = CDF_AW'(CDF_COMP_BWD_REF + int'(ctx_comp_bwdref_p1) * CDF_COMP_BWD_REF_S0 + 1);
            4'd10: rfs_addr = CDF_AW'(CDF_SINGLE_REF + int'(ctx_single_ref_p1) * CDF_SINGLE_REF_S0 + 0);
            4'd11: rfs_addr = CDF_AW'(CDF_SINGLE_REF + int'(ctx_comp_bwdref) * CDF_SINGLE_REF_S0 + 1);
            4'd12: rfs_addr = CDF_AW'(CDF_SINGLE_REF + int'(ctx_comp_ref) * CDF_SINGLE_REF_S0 + 2);
            4'd13: rfs_addr = CDF_AW'(CDF_SINGLE_REF + int'(ctx_comp_ref_p1) * CDF_SINGLE_REF_S0 + 3);
            4'd14: rfs_addr = CDF_AW'(CDF_SINGLE_REF + int'(ctx_comp_ref_p2) * CDF_SINGLE_REF_S0 + 4);
            default: rfs_addr = CDF_AW'(CDF_SINGLE_REF + int'(ctx_comp_bwdref_p1) * CDF_SINGLE_REF_S0 + 5);
        endcase
    end
    assign mvm_r = br; assign mvm_c = bc; assign mvm_bw4 = bw4; assign mvm_bh4 = bh4;
    assign mvm_data = '{is_inter: inter_any, is_intrabc: use_ibc, ref0: is_inter_b ? rf0 : 4'd0, ref1: is_inter_b ? rf1 : 4'd15,
                        mv_row: 16'(mv_row), mv_col: 16'(mv_col), mv1_row: 16'(mv1_row), mv1_col: 16'(mv1_col),
                        newmv_fam: is_inter_b && newmv_fam, globalmv_fam: is_inter_b && globalmv_fam, bsize: bs, skip_mode: skip_mode,
                        comp_group_idx: comp_group_idx, compound_idx: compound_idx, interp0: interp[0], interp1: interp[1], seg_pred: seg_pred_sym};

    // txfm_split context of the current node (8.3.2): above / left transform widths against the node's size
    logic [4:0]  vt_tx;
    logic [5:0]  vt_r, vt_c;
    logic [23:0] vt_arec, vt_lrec;
    logic [7:0]  vt_above_w, vt_left_h;          // 8 bits: a skipped inter neighbour counts with its block size (up to 128)
    logic [2:0]  vt_max_tx;
    logic [4:0]  vt_ctx;
    always_comb begin
        vt_tx = nd_tx[vt_d]; vt_r = nd_r[vt_d]; vt_c = nd_c[vt_d];
        vt_arec = a_recs[24 * (int'(bc[3:0]) + int'(vt_c)) +: 24];
        vt_lrec = l_recs[24 * (int'(br[3:0]) + int'(vt_r)) +: 24];
        if (vt_r == 6'd0) begin
            if (!avail_u) vt_above_w = 8'd64;
            else if (vt_arec[4] && vt_arec[23]) vt_above_w = blk_w(vt_arec[9:5]);            // Skips && IsInters
            else vt_above_w = 8'(tx_width(vt_arec[14:10]));
        end else vt_above_w = 8'(tx_width(top_edge[vt_c[4:0]]));
        if (vt_c == 6'd0) begin
            if (!avail_l) vt_left_h = 8'd64;
            else if (vt_lrec[4] && vt_lrec[23]) vt_left_h = blk_h(vt_lrec[9:5]);
            else vt_left_h = 8'(tx_height(vt_lrec[14:10]));
        end else vt_left_h = 8'(tx_height(left_edge[vt_r[4:0]]));
        begin
            logic [7:0] sz;
            sz = (bwp > bhp) ? bwp : bhp;
            if (sz > 8'd64) sz = 8'd64;
            vt_max_tx = (sz == 8'd64) ? 3'd4 : (sz == 8'd32) ? 3'd3 : (sz == 8'd16) ? 3'd2 : (sz == 8'd8) ? 3'd1 : 3'd0;
        end
        vt_ctx = 5'((tx_sqr_up(vt_tx) != vt_max_tx) ? 3 : 0) + (5'd4 - 5'(vt_max_tx)) * 5'd6
                 + 5'(vt_above_w < 8'(tx_width(vt_tx))) + 5'(vt_left_h < 8'(tx_height(vt_tx)));
    end
    // chroma inter transform type: the luma leaf covering (max(MiRow, y4 << ssy), max(MiCol, x4 << ssx))
    logic [MIW-1:0] uvt_y4, uvt_x4;
    logic [5:0]  uvt_ry, uvt_rx;
    logic        uvt_hit;
    always_comb begin
        uvt_y4 = MIW'(start_y >> 2) << hdr.ssy; if (uvt_y4 < br) uvt_y4 = br;
        uvt_x4 = MIW'(start_x >> 2) << hdr.ssx; if (uvt_x4 < bc) uvt_x4 = bc;
        uvt_ry = 6'(uvt_y4 - br); uvt_rx = 6'(uvt_x4 - bc);
        uvt_hit = (uvt_k < n_leaves) && (uvt_ry >= leaf_r[uvt_k[5:0]]) && (7'(uvt_ry) < 7'(leaf_r[uvt_k[5:0]]) + 7'(tx_height(leaf_tx[uvt_k[5:0]]) >> 2))
                  && (uvt_rx >= leaf_c[uvt_k[5:0]]) && (7'(uvt_rx) < 7'(leaf_c[uvt_k[5:0]]) + 7'(tx_width(leaf_tx[uvt_k[5:0]]) >> 2));
    end

    // ---------------------------------------------------------------- symbol requests (combinational)
    always_comb begin
        sq_go = 1'b0; sq_addr = '0; sq_n = 4'd1; sq_kind = 2'd0;
        case (st)
            S_SEG_A, S_SEG_B_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_SEGMENT_ID + int'(seg_ctx)); sq_n = 4'd7; end
            S_SKIP_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_SKIP + ((avail_u && a_skip) ? 1 : 0) + ((avail_l && l_skip) ? 1 : 0)); sq_n = 4'd1; end
            S_DQ_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_DELTA_Q); sq_n = 4'd3; end
            S_IBC_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_INTRABC); sq_n = 4'd1; end
            S_DLF_GO: begin sq_go = 1'b1; sq_addr = hdr.delta_lf_multi ? CDF_AW'(CDF_DELTA_LF_MULTI + int'(dlf_i)) : CDF_AW'(CDF_DELTA_LF); sq_n = 4'd3; end
            S_YMODE: begin sq_go = 1'b1; sq_n = 4'd12;
                           if (inter_frm) sq_addr = CDF_AW'(CDF_Y_MODE + int'(size_group(bs)));
                           else sq_addr = CDF_AW'(CDF_INTRA_FRAME_Y_MODE + int'(intra_mode_ctx(avail_u ? a_ymode : DC_PRED)) * CDF_INTRA_FRAME_Y_MODE_S0
                                                  + int'(intra_mode_ctx(avail_l ? l_ymode : DC_PRED))); end
            S_ANGY_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_ANGLE_DELTA + int'(ymode) - 1); sq_n = 4'd6; end
            S_UV:    begin sq_go = 1'b1; sq_addr = uv_cfl_allowed ? CDF_AW'(CDF_UV_MODE_CFL_ALLOWED + int'(ymode)) : CDF_AW'(CDF_UV_MODE_CFL_NOT_ALLOWED + int'(ymode));
                           sq_n = uv_cfl_allowed ? 4'd13 : 4'd12; end
            S_CFLS:  begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_CFL_SIGN); sq_n = 4'd7; end
            S_CFLU_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_CFL_ALPHA + (int'(sign_u) - 1) * 3 + int'(sign_v)); sq_n = 4'd15; end
            S_CFLV_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_CFL_ALPHA + (int'(sign_v) - 1) * 3 + int'(sign_u)); sq_n = 4'd15; end
            S_ANGUV_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_ANGLE_DELTA + int'(uvmode) - 1); sq_n = 4'd6; end
            S_FI_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_FILTER_INTRA + int'(bs)); sq_n = 4'd1; end
            S_FIM:   begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_FILTER_INTRA_MODE); sq_n = 4'd4; end
            S_TXD_GO: begin
                sq_go = 1'b1;
                case (max_depth)
                    3'd4: begin sq_addr = CDF_AW'(CDF_TX_64X64 + int'(txd_ctx)); sq_n = 4'd2; end
                    3'd3: begin sq_addr = CDF_AW'(CDF_TX_32X32 + int'(txd_ctx)); sq_n = 4'd2; end
                    3'd2: begin sq_addr = CDF_AW'(CDF_TX_16X16 + int'(txd_ctx)); sq_n = 4'd2; end
                    default: begin sq_addr = CDF_AW'(CDF_TX_8X8 + int'(txd_ctx)); sq_n = 4'd1; end
                endcase
            end
            S_VTX_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_TXFM_SPLIT + int'(vt_ctx)); sq_n = 4'd1; end
            // MV coding: MvCtx = MV_INTRABC_CONTEXT (1) for intra block copy, 0 for inter blocks
            S_MVJ:   begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_MV_JOINT + (use_ibc ? 1 : 0)); sq_n = 4'd3; end
            S_MVSGN: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_MV_SIGN + (use_ibc ? CDF_MV_SIGN_S0 : 0) + int'(mv_comp)); sq_n = 4'd1; end
            S_MVCLS: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_MV_CLASS + (use_ibc ? CDF_MV_CLASS_S0 : 0) + int'(mv_comp)); sq_n = 4'd10; end
            S_MVC0B: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_MV_CLASS0_BIT + (use_ibc ? CDF_MV_CLASS0_BIT_S0 : 0) + int'(mv_comp)); sq_n = 4'd1; end
            S_MVBIT: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_MV_BIT + (use_ibc ? CDF_MV_BIT_S0 : 0) + int'(mv_comp) * CDF_MV_BIT_S1 + int'(mv_bi)); sq_n = 4'd1; end
            S_MVFR_GO: begin sq_go = 1'b1; sq_n = 4'd3;
                           if (mv_cls == 4'd0) sq_addr = CDF_AW'(CDF_MV_CLASS0_FR + (use_ibc ? CDF_MV_CLASS0_FR_S0 : 0) + int'(mv_comp) * CDF_MV_CLASS0_FR_S1 + int'(mv_c0b));
                           else sq_addr = CDF_AW'(CDF_MV_FR + (use_ibc ? CDF_MV_FR_S0 : 0) + int'(mv_comp)); end
            S_MVHP_GO: begin sq_go = 1'b1; sq_n = 4'd1;
                           if (mv_cls == 4'd0) sq_addr = CDF_AW'(CDF_MV_CLASS0_HP + (use_ibc ? CDF_MV_CLASS0_HP_S0 : 0) + int'(mv_comp));
                           else sq_addr = CDF_AW'(CDF_MV_HP + (use_ibc ? CDF_MV_HP_S0 : 0) + int'(mv_comp)); end
            // inter frames
            S_SEGP_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_SEGMENT_ID_PREDICTED + int'(ctx_segp)); sq_n = 4'd1; end
            S_SKM_GO:  begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_SKIP_MODE + int'(ctx_skm)); sq_n = 4'd1; end
            S_ISI_GO:  begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_IS_INTER + int'(ctx_is_inter)); sq_n = 4'd1; end
            S_RFS_GO:  begin sq_go = 1'b1; sq_addr = rfs_addr; sq_n = 4'd1; end
            S_CMODE_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_COMPOUND_MODE + int'(ctx_cmode)); sq_n = 4'd7; end
            S_NEWMV_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_NEW_MV + int'(mvs_new_ctx)); sq_n = 4'd1; end
            S_ZEROMV_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_ZERO_MV + int'(mvs_zero_ctx)); sq_n = 4'd1; end
            S_REFMV_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_REF_MV + int'(mvs_ref_ctx)); sq_n = 4'd1; end
            S_DRL_GO:  begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_DRL_MODE + int'(mvs_drl[drl_i])); sq_n = 4'd1; end
            S_II_GO:   begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_INTER_INTRA + int'(size_group(bs)) - 1); sq_n = 4'd1; end
            S_IIM_GO:  begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_INTER_INTRA_MODE + int'(size_group(bs)) - 1); sq_n = 4'd3; end
            S_WII_GO:  begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_WEDGE_INTER_INTRA + int'(bs)); sq_n = 4'd1; end
            S_WIDX_GO, S_WIDX2_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_WEDGE_INDEX + int'(bs)); sq_n = 4'd15; end
            S_OBMC_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_USE_OBMC + int'(bs)); sq_n = 4'd1; end
            S_MMS_GO:  begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_MOTION_MODE + int'(bs)); sq_n = 4'd2; end
            S_CGI_GO:  begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_COMP_GROUP_IDX + int'(ctx_cgi)); sq_n = 4'd1; end
            S_CIDX_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_COMPOUND_IDX + int'(ctx_cidx)); sq_n = 4'd1; end
            S_CTYPE_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_COMPOUND_TYPE + int'(bs)); sq_n = 4'd1; end
            S_IF_GO:   begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_INTERP_FILTER + int'(ctx_interp)); sq_n = 4'd2; end
            S_TXTYPE: begin
                sq_go = 1'b1;
                if (inter_any) begin
                    if (cur_set_i == 2'd1) begin sq_addr = CDF_AW'(CDF_INTER_TX_TYPE_SET1 + int'(tx_sqr(p_txsz))); sq_n = 4'd15; end
                    else if (cur_set_i == 2'd2) begin sq_addr = CDF_AW'(CDF_INTER_TX_TYPE_SET2); sq_n = 4'd11; end
                    else begin sq_addr = CDF_AW'(CDF_INTER_TX_TYPE_SET3 + int'(tx_sqr(p_txsz))); sq_n = 4'd1; end
                end else if (cur_set == 2'd1) begin sq_addr = CDF_AW'(CDF_INTRA_TX_TYPE_SET1 + int'(tx_sqr(p_txsz)) * CDF_INTRA_TX_TYPE_SET1_S0 + int'(intra_dir)); sq_n = 4'd6; end
                else begin sq_addr = CDF_AW'(CDF_INTRA_TX_TYPE_SET2 + int'(tx_sqr(p_txsz)) * CDF_INTRA_TX_TYPE_SET2_S0 + int'(intra_dir)); sq_n = 4'd4; end
            end
            S_LIT:   if (lit_n != 4'd0) begin sq_go = 1'b1; sq_kind = 2'd2; end
            S_PAL_W, S_PTOK_W: begin sq_go = pq_go; sq_addr = pq_addr; sq_n = pq_n; sq_kind = pq_kind; end
            default: ;
        endcase
    end

    // ---------------------------------------------------------------- outputs to blk_ctx / coef_rd (registered pulses)
    assign busy = (st != S_IDLE);
    assign cf_tx = p_txsz;
    assign cf_ptype = (plane != 2'd0);
    assign cf_az_ctx = az_ctx;
    assign cf_dcs_ctx = dcs_ctx;
    assign cf_tx_type = p_txtype;
    assign tx_plane = plane;
    assign tx_x4 = MIW'(start_x >> 2);
    assign tx_y4 = MIW'(start_y >> 2);
    assign tx_sz_o = p_txsz;
    assign tx_bsize = subsampled_size(bs, sub_x, sub_y);
    assign w_ymode = ymode; assign w_skip = skip; assign w_seg = seg; assign w_txsz = txsz;
    assign w_is_inter = inter_any;
    assign w_vartx = inter_any && !lossless;
    always_comb for (int k = 0; k < 32; k++) begin w_txsz_col[5 * k +: 5] = top_edge[k]; w_txsz_row[5 * k +: 5] = left_edge[k]; end
    assign w_cul = all_zero ? 6'd0 : cf_cul;
    assign w_dccat = all_zero ? 2'd0 : cf_dccat;

    // residual geometry
    always_comb begin
        sub_x = (plane != 2'd0) && hdr.ssx;
        sub_y = (plane != 2'd0) && hdr.ssy;
        max_x = PXW'(({2'b0, hdr.mi_cols} << 2) >> sub_x);
        max_y = PXW'(({2'b0, hdr.mi_rows} << 2) >> sub_y);
        if (leaf_mode) begin
            start_x = base_x + (PXW'(leaf_c[leaf_rd[5:0]]) << 2);
            start_y = base_y + (PXW'(leaf_r[leaf_rd[5:0]]) << 2);
        end else begin
            start_x = base_x + ((PXW'(rx) + (chunk_x ? (PXW'(16) >> sub_x) : PXW'(0))) << 2);
            start_y = base_y + ((PXW'(ry) + (chunk_y ? (PXW'(16) >> sub_y) : PXW'(0))) << 2);
        end
    end

    // ---------------------------------------------------------------- main FSM
    always_ff @(posedge clk) begin
        blk_done <= 1'b0; nb_req <= 1'b0; ctx_we <= 1'b0; tx_req <= 1'b0; tx_we <= 1'b0; rbc_we <= 1'b0;
        cf_start_a <= 1'b0; cf_start_b <= 1'b0; pal_clr <= 1'b0; pal_start_mi <= 1'b0; pal_start_tok <= 1'b0; blk_info <= 1'b0;
        mvs_start <= 1'b0; mvm_we <= 1'b0;
        if (rst) begin
            st <= S_IDLE; unsupported <= 1'b0; tx_done <= 1'b0; read_deltas <= 1'b0; cdef_flags <= '0;
        end else begin
            if (tile_start) begin
                cur_qidx <= hdr.base_q_idx;
                for (int i = 0; i < 4; i++) dlf[i] <= 7'sd0;
                unsupported <= 1'b0;
            end
            if (sb_start) begin
                read_deltas <= hdr.delta_q_present;
                cdef_flags <= '0;
            end
            case (st)
                S_IDLE: if (start) begin
                    br <= r; bc <= c; bs <= bsize;
                    bw4 <= num4x4w(bsize); bh4 <= num4x4h(bsize); bwp <= blk_w(bsize); bhp <= blk_h(bsize);
                    skip <= 1'b0; seg <= 3'd0; lossless <= hdr.lossless[0];
                    ymode <= DC_PRED; uvmode <= DC_PRED; ang_y <= 3'sd0; ang_uv <= 3'sd0; cfl_u <= 6'sd0; cfl_v <= 6'sd0;
                    use_fi <= 1'b0; fi_mode <= 3'd0; cdef_valid <= 1'b0; cdef_units <= 4'd0;
                    use_ibc <= 1'b0; mv_row <= 18'sd0; mv_col <= 18'sd0; leaf_mode <= 1'b0; n_leaves <= 7'd0; leaf_rd <= 7'd0;
                    is_inter_b <= 1'b0; skip_mode <= 1'b0; is_comp <= 1'b0; rf0 <= R_INTRA; rf1 <= R_NONE; imode <= 5'd0;
                    mv1_row <= 18'sd0; mv1_col <= 18'sd0; mv_list <= 1'b0; ref_mv_idx <= 2'd0; motion_mode <= 2'd0; compound_type <= 3'd2;
                    interintra <= 1'b0; ii_mode <= 2'd0; wedge_ii <= 1'b0; wedge_idx <= 4'd0; wedge_sign <= 1'b0; mask_type <= 1'b0;
                    comp_group_idx <= 1'b0; compound_idx <= 1'b1; interp[0] <= 2'd0; interp[1] <= 2'd0; seg_pred_sym <= 1'b0; seg_pre <= 1'b1;
                    pal_clr <= 1'b1;
                    st <= S_NBW;
                end
                // the previous block's motion entry must be written before this block's neighbour reads (inter contexts)
                S_NBW: if (!mvm_busy) begin nb_req <= 1'b1; st <= S_NB; end
                S_NB: if (nb_valid) begin
                    hc <= has_chroma;
                    if (inter_frm) st <= S_NBA;
                    else st <= hdr.seg_preskip ? (hdr.seg_enabled ? S_SEG_A : S_SKIP) : S_SKIP;
                end
                // ---- inter frame: the two neighbours' motion info (contexts), then inter_frame_mode_info
                S_NBA: st <= S_NBA_W;
                S_NBA_W: begin a_ent <= mvr_ent; a_ok <= avail_u && mvr_written; st <= S_NBL; end
                S_NBL: st <= S_NBL_W;
                S_NBL_W: begin l_ent <= mvr_ent; l_ok <= avail_l && mvr_written; seg_pre <= 1'b1; st <= S_ISEG; end
                // inter_segment_id(preSkip = seg_pre) (5.11.8)
                S_ISEG: begin
                    if (!hdr.seg_enabled) begin seg <= 3'd0; lossless <= hdr.lossless[0]; st <= seg_pre ? S_SKM : S_CDEF; end
                    else if (!ihdr.seg_update_map) begin sp_mode <= 1'b0; st <= S_SP_RD; end
                    else if (seg_pre && !hdr.seg_preskip) begin seg <= 3'd0; st <= S_SKM; end
                    else if (!seg_pre && skip) begin seg_pred_sym <= 1'b0; seg_ret <= S_CDEF; st <= S_SEG_B; end
                    else if (ihdr.seg_temporal_update) begin sp_mode <= 1'b1; st <= S_SP_RD; end
                    else begin seg_ret <= seg_pre ? S_SKM : S_CDEF; st <= S_SEG_B; end
                    // get_segment_id scan setup (used by the two band paths)
                    sp_min <= 3'd7; sp_y <= 6'd0;
                    sp_xmis <= (MIW'(bw4) < hdr.mi_cols - bc) ? bw4 : 6'(hdr.mi_cols - bc);
                    sp_ymis <= (MIW'(bh4) < hdr.mi_rows - br) ? bh4 : 6'(hdr.mi_rows - br);
                    sp_w0 <= bc[MIW-1:4]; sp_w <= bc[MIW-1:4];
                    sp_w_end <= (MIW-4)'((bc + ((MIW'(bw4) < hdr.mi_cols - bc) ? MIW'(bw4) : hdr.mi_cols - bc) - MIW'(1)) >> 4);
                end
                S_SP_RD: st <= S_SP_W;
                S_SP_W: begin
                    if (sp_word_min < sp_min) sp_min <= sp_word_min;
                    if (sp_w < sp_w_end) begin sp_w <= sp_w + 1'b1; st <= S_SP_RD; end
                    else if (sp_y + 6'd1 < sp_ymis) begin sp_y <= sp_y + 6'd1; sp_w <= sp_w0; st <= S_SP_RD; end
                    else st <= S_SP_END;
                end
                S_SP_END: begin
                    if (!sp_mode) begin seg <= sp_min; lossless <= hdr.lossless[sp_min]; st <= seg_pre ? S_SKM : S_CDEF; end
                    else st <= S_SEGP_GO;
                end
                S_SEGP_GO: st <= S_SEGP_W;
                S_SEGP_W: if (sq_done) begin
                    seg_pred_sym <= sq_sym[0];
                    if (sq_sym[0]) begin seg <= sp_min; lossless <= hdr.lossless[sp_min]; st <= seg_pre ? S_SKM : S_CDEF; end
                    else begin seg_ret <= seg_pre ? S_SKM : S_CDEF; st <= S_SEG_B; end
                end
                // read_skip_mode (5.11.10)
                S_SKM: begin
                    if ((hdr.seg_enabled && (hdr.seg_skip_en[seg] || ihdr.seg_ref_en[seg] || ihdr.seg_globalmv_en[seg])) ||
                        !ihdr.skip_mode_present || bwp < 8'd8 || bhp < 8'd8) begin skip_mode <= 1'b0; st <= S_SKIP; end
                    else st <= S_SKM_GO;
                end
                S_SKM_GO: st <= S_SKM_W;
                S_SKM_W: if (sq_done) begin
                    skip_mode <= sq_sym[0];
                    if (sq_sym[0]) begin
                        skip <= 1'b1;
                        if (!hdr.seg_preskip) begin seg_pre <= 1'b0; st <= S_ISEG; end else st <= S_CDEF;
                    end else st <= S_SKIP;
                end
                // ---- segment id before skip (SegIdPreSkip)
                S_SEG_A: st <= S_SEG_A_W;
                S_SEG_A_W: if (sq_done) begin
                    seg <= neg_deinterleave(sq_sym, seg_pred, 4'(hdr.last_active_segid) + 4'd1);
                    st <= S_SKIP;
                end
                // ---- skip
                S_SKIP: begin
                    lossless <= hdr.lossless[seg];
                    if (hdr.seg_preskip && hdr.seg_enabled && hdr.seg_skip_en[seg]) begin skip <= 1'b1; st <= S_CDEF; end
                    else st <= S_SKIP_GO;
                end
                S_SKIP_GO: begin st <= S_SKIP_W;
                end
                S_SKIP_W: if (sq_done) begin
                    skip <= sq_sym[0];
                    if (inter_frm) begin
                        if (!hdr.seg_preskip) begin seg_pre <= 1'b0; st <= S_ISEG; end else st <= S_CDEF;
                    end
                    else if (!hdr.seg_preskip && hdr.seg_enabled) begin seg_ret <= S_CDEF; st <= S_SEG_B; end
                    else st <= S_CDEF;
                end
                // ---- segment id after skip (read_segment_id; also the inter frames' read_segment_id, returning to seg_ret)
                S_SEG_B: begin
                    if (skip) begin seg <= seg_pred[2:0]; lossless <= hdr.lossless[seg_pred[2:0]]; st <= seg_ret; end
                    else st <= S_SEG_B_GO;
                end
                S_SEG_B_GO: begin st <= S_SEG_B_W;
                end
                S_SEG_B_W: if (sq_done) begin
                    seg <= neg_deinterleave(sq_sym, seg_pred, 4'(hdr.last_active_segid) + 4'd1);
                    lossless <= hdr.lossless[neg_deinterleave(sq_sym, seg_pred, 4'(hdr.last_active_segid) + 4'd1)];
                    st <= seg_ret;
                end
                // ---- cdef idx
                S_CDEF: begin
                    if (skip || hdr.coded_lossless || !hdr.enable_cdef || hdr.allow_intrabc || cdef_flags[cdef_unit]) st <= S_DQ;
                    else begin
                        lit_n <= 4'(hdr.cdef_bits); lit_val <= '0; lit_ret <= S_CDEF_W; st <= S_LIT;
                    end
                end
                S_CDEF_W: begin
                    cdef_val <= lit_val[2:0]; cdef_valid <= 1'b1;
                    // units covered by the block (64x64 granularity within a 128 superblock)
                    for (int i = 0; i < 2; i++)
                        for (int j = 0; j < 2; j++)
                            if (hdr.sb128 && (i >= int'(br[4]) && i < int'(br[4]) + (int'(bh4) + 15) / 16) &&
                                             (j >= int'(bc[4]) && j < int'(bc[4]) + (int'(bw4) + 15) / 16)) begin
                                cdef_flags[i * 2 + j] <= 1'b1; cdef_units[i * 2 + j] <= 1'b1;
                            end
                    if (!hdr.sb128) begin cdef_flags[0] <= 1'b1; cdef_units[0] <= 1'b1; end
                    st <= S_DQ;
                end
                // ---- delta q
                S_DQ: begin
                    if ((bs == sb_bs && skip) || !read_deltas) st <= S_IBC;
                    else st <= S_DQ_GO;
                end
                S_DQ_GO: begin st <= S_DQ_W;
                end
                S_DQ_W: if (sq_done) begin
                    d_abs <= 10'(sq_sym);
                    if (sq_sym == 4'd3) begin lit_n <= 4'd3; lit_val <= '0; lit_ret <= S_DQ_REM_W; st <= S_LIT; end
                    else st <= S_DQ_SIGN;
                end
                S_DQ_REM_W: begin
                    rem_bits <= 4'(lit_val[2:0]) + 4'd1;
                    lit_n <= 4'(lit_val[2:0]) + 4'd1; lit_val <= '0; lit_ret <= S_DQ_ABS_W; st <= S_LIT;
                end
                S_DQ_ABS_W: begin
                    d_abs <= 10'(lit_val) + (10'd1 << rem_bits) + 10'd1;
                    st <= S_DQ_SIGN;
                end
                S_DQ_SIGN: begin
                    if (d_abs == 10'd0) st <= S_DLF;
                    else begin lit_n <= 4'd1; lit_val <= '0; lit_ret <= S_DQ_SIGN_W; st <= S_LIT; end
                end
                S_DQ_SIGN_W: begin
                    // CurrentQIndex = Clip3(1, 255, CurrentQIndex + (reduced << delta_q_res))
                    begin
                        logic signed [15:0] v;
                        v = 16'(cur_qidx) + (lit_val[0] ? -(16'(d_abs) <<< hdr.delta_q_res) : (16'(d_abs) <<< hdr.delta_q_res));
                        cur_qidx <= (v < 16'sd1) ? 8'd1 : (v > 16'sd255) ? 8'd255 : 8'(v);
                    end
                    st <= S_DLF;
                end
                // ---- delta lf
                S_DLF: begin
                    dlf_i <= 2'd0;
                    dlf_cnt <= hdr.delta_lf_multi ? (hdr.mono ? 2'd1 : 2'd3) : 2'd0;     // count-1
                    if ((bs == sb_bs && skip) || !read_deltas || !hdr.delta_lf_present) st <= S_IBC;
                    else st <= S_DLF_GO;
                end
                S_DLF_GO: begin st <= S_DLF_W;
                end
                S_DLF_W: if (sq_done) begin
                    d_abs <= 10'(sq_sym);
                    if (sq_sym == 4'd3) begin lit_n <= 4'd3; lit_val <= '0; lit_ret <= S_DLF_REM_W; st <= S_LIT; end
                    else st <= S_DLF_SIGN;
                end
                S_DLF_REM_W: begin
                    rem_bits <= 4'(lit_val[2:0]) + 4'd1;
                    lit_n <= 4'(lit_val[2:0]) + 4'd1; lit_val <= '0; lit_ret <= S_DLF_ABS_W; st <= S_LIT;
                end
                S_DLF_ABS_W: begin d_abs <= 10'(lit_val) + (10'd1 << rem_bits) + 10'd1; st <= S_DLF_SIGN; end
                S_DLF_SIGN: begin
                    if (d_abs == 10'd0) st <= S_DLF_NEXT;
                    else begin lit_n <= 4'd1; lit_val <= '0; lit_ret <= S_DLF_SIGN_W; st <= S_LIT; end
                end
                S_DLF_SIGN_W: begin
                    begin
                        logic signed [15:0] v;
                        v = 16'(dlf[dlf_i]) + (lit_val[0] ? -(16'(d_abs) <<< hdr.delta_lf_res) : (16'(d_abs) <<< hdr.delta_lf_res));
                        dlf[dlf_i] <= (v < -16'sd63) ? -7'sd63 : (v > 16'sd63) ? 7'sd63 : 7'(v);
                    end
                    st <= S_DLF_NEXT;
                end
                S_DLF_NEXT: begin
                    if (dlf_i == dlf_cnt) st <= S_IBC;
                    else begin dlf_i <= dlf_i + 2'd1; st <= S_DLF_GO; end
                end
                // ---- use_intrabc (only when allow_intrabc; a set flag means an intrabc block, which is not implemented)
                S_IBC: st <= inter_frm ? S_ISI : (hdr.allow_intrabc ? S_IBC_GO : S_YMODE);
                // ---- read_is_inter (5.11.20)
                S_ISI: begin
                    read_deltas <= 1'b0;
                    if (skip_mode) begin is_inter_b <= 1'b1; st <= S_RF; end
                    else if (hdr.seg_enabled && ihdr.seg_ref_en[seg]) begin
                        is_inter_b <= (ihdr.seg_ref_data[3 * seg +: 3] != 3'd0); st <= (ihdr.seg_ref_data[3 * seg +: 3] != 3'd0) ? S_RF : S_YMODE;
                    end
                    else if (hdr.seg_enabled && ihdr.seg_globalmv_en[seg]) begin is_inter_b <= 1'b1; st <= S_RF; end
                    else st <= S_ISI_GO;
                end
                S_ISI_GO: st <= S_ISI_W;
                S_ISI_W: if (sq_done) begin is_inter_b <= sq_sym[0]; st <= sq_sym[0] ? S_RF : S_YMODE; end
                // ---- read_ref_frames (5.11.25): one symbol per step of the decision tree
                S_RF: begin
                    rf1 <= R_NONE;
                    if (skip_mode) begin rf0 <= {1'b0, ihdr.skip_mode_frame0}; rf1 <= {1'b0, ihdr.skip_mode_frame1}; is_comp <= 1'b1; st <= S_MVS; end
                    else if (hdr.seg_enabled && ihdr.seg_ref_en[seg]) begin rf0 <= {1'b0, ihdr.seg_ref_data[3 * seg +: 3]}; st <= S_MVS; end
                    else if (hdr.seg_enabled && (hdr.seg_skip_en[seg] || ihdr.seg_globalmv_en[seg])) begin rf0 <= R_LAST; st <= S_MVS; end
                    else begin rf_step <= (ihdr.reference_select && bw4 >= 6'd2 && bh4 >= 6'd2) ? 4'd0 : 4'd10; st <= S_RFS_GO; end
                end
                S_RFS_GO: st <= S_RFS_W;
                S_RFS_W: if (sq_done) begin
                    st <= S_RFS_GO;
                    case (rf_step)
                        4'd0: rf_step <= sq_sym[0] ? 4'd1 : 4'd10;                                                   // comp_mode
                        4'd1: rf_step <= sq_sym[0] ? 4'd5 : 4'd2;                                                    // comp_ref_type (0 = UNIDIR)
                        4'd2: if (sq_sym[0]) begin rf0 <= R_BWDREF; rf1 <= R_ALTREF; is_comp <= 1'b1; st <= S_MVS; end else rf_step <= 4'd3;
                        4'd3: if (sq_sym[0]) rf_step <= 4'd4; else begin rf0 <= R_LAST; rf1 <= R_LAST2; is_comp <= 1'b1; st <= S_MVS; end
                        4'd4: begin rf0 <= R_LAST; rf1 <= sq_sym[0] ? R_GOLDEN : R_LAST3; is_comp <= 1'b1; st <= S_MVS; end
                        4'd5: rf_step <= sq_sym[0] ? 4'd7 : 4'd6;                                                    // comp_ref
                        4'd6: begin rf0 <= sq_sym[0] ? R_LAST2 : R_LAST; rf_step <= 4'd8; end
                        4'd7: begin rf0 <= sq_sym[0] ? R_GOLDEN : R_LAST3; rf_step <= 4'd8; end
                        4'd8: if (sq_sym[0]) begin rf1 <= R_ALTREF; is_comp <= 1'b1; st <= S_MVS; end else rf_step <= 4'd9;    // comp_bwdref
                        4'd9: begin rf1 <= sq_sym[0] ? R_ALTREF2 : R_BWDREF; is_comp <= 1'b1; st <= S_MVS; end
                        4'd10: rf_step <= sq_sym[0] ? 4'd11 : 4'd12;                                                 // single_ref_p1
                        4'd11: if (sq_sym[0]) begin rf0 <= R_ALTREF; st <= S_MVS; end else rf_step <= 4'd15;        // single_ref_p2
                        4'd12: rf_step <= sq_sym[0] ? 4'd14 : 4'd13;                                                 // single_ref_p3
                        4'd13: begin rf0 <= sq_sym[0] ? R_LAST2 : R_LAST; st <= S_MVS; end                           // single_ref_p4
                        4'd14: begin rf0 <= sq_sym[0] ? R_GOLDEN : R_LAST3; st <= S_MVS; end                         // single_ref_p5
                        default: begin rf0 <= sq_sym[0] ? R_ALTREF2 : R_BWDREF; st <= S_MVS; end                     // single_ref_p6
                    endcase
                end
                // ---- the mode (5.11.23 after find_mv_stack)
                S_MODE: begin
                    if (skip_mode) begin imode <= NEAREST_NEARESTMV; st <= S_DRL; end
                    else if (hdr.seg_enabled && (hdr.seg_skip_en[seg] || ihdr.seg_globalmv_en[seg])) begin imode <= GLOBALMV; st <= S_DRL; end
                    else if (is_comp) st <= S_CMODE_GO;
                    else st <= S_NEWMV_GO;
                end
                S_CMODE_GO: st <= S_CMODE_W;
                S_CMODE_W: if (sq_done) begin imode <= NEAREST_NEARESTMV + 5'(sq_sym); st <= S_DRL; end
                S_NEWMV_GO: st <= S_NEWMV_W;
                S_NEWMV_W: if (sq_done) begin if (sq_sym[0] == 1'b0) begin imode <= NEWMV; st <= S_DRL; end else st <= S_ZEROMV_GO; end
                S_ZEROMV_GO: st <= S_ZEROMV_W;
                S_ZEROMV_W: if (sq_done) begin if (sq_sym[0] == 1'b0) begin imode <= GLOBALMV; st <= S_DRL; end else st <= S_REFMV_GO; end
                S_REFMV_GO: st <= S_REFMV_W;
                S_REFMV_W: if (sq_done) begin imode <= sq_sym[0] ? NEARMV : NEARESTMV; st <= S_DRL; end
                // ---- drl_mode
                S_DRL: begin
                    ref_mv_idx <= 2'd0;
                    if (imode == NEWMV || imode == NEW_NEWMV) begin drl_i <= 2'd0; drl_end <= 2'd2; st <= S_DRL_IT; end
                    else if (has_nearmv(imode)) begin ref_mv_idx <= 2'd1; drl_i <= 2'd1; drl_end <= 2'd3; st <= S_DRL_IT; end
                    else begin mv_list <= 1'b0; st <= S_AMV; end
                end
                S_DRL_IT: begin
                    if (drl_i >= drl_end) begin mv_list <= 1'b0; st <= S_AMV; end
                    else if (mvs_n > 4'(drl_i) + 4'd1) st <= S_DRL_GO;
                    else drl_i <= drl_i + 2'd1;
                end
                S_DRL_GO: st <= S_DRL_W;
                S_DRL_W: if (sq_done) begin
                    if (sq_sym[0] == 1'b0) begin ref_mv_idx <= drl_i; mv_list <= 1'b0; st <= S_AMV; end
                    else begin ref_mv_idx <= drl_i + 2'd1; drl_i <= drl_i + 2'd1; st <= S_DRL_IT; end
                end
                // ---- assign_mv (5.11.26) for list mv_list: NEWMV reads a difference on the predicted MV
                S_AMV: begin
                    pr_r <= amv_r; pr_c <= amv_c;
                    if (amode == NEWMV) st <= S_MVJ;
                    else begin
                        if (!mv_list) begin mv_row <= amv_r; mv_col <= amv_c; end else begin mv1_row <= amv_r; mv1_col <= amv_c; end
                        st <= S_AMV_NEXT;
                    end
                end
                S_AMV_NEXT: begin
                    if (is_comp && !mv_list) begin mv_list <= 1'b1; st <= S_AMV; end
                    else st <= S_II;
                end
                // ---- read_interintra_mode (5.11.28)
                S_II: begin
                    if (!skip_mode && ihdr.enable_interintra_compound && !is_comp && bs >= BLOCK_8X8 && bs <= 5'd9) st <= S_II_GO;
                    else st <= S_MM;
                end
                S_II_GO: st <= S_II_W;
                S_II_W: if (sq_done) begin interintra <= sq_sym[0]; st <= sq_sym[0] ? S_IIM_GO : S_MM; end
                S_IIM_GO: st <= S_IIM_W;
                S_IIM_W: if (sq_done) begin ii_mode <= sq_sym[1:0]; rf1 <= R_INTRA; st <= S_WII_GO; end
                S_WII_GO: st <= S_WII_W;
                S_WII_W: if (sq_done) begin wedge_ii <= sq_sym[0]; st <= sq_sym[0] ? S_WIDX_GO : S_MM; end
                S_WIDX_GO: st <= S_WIDX_W;
                S_WIDX_W: if (sq_done) begin wedge_idx <= sq_sym; wedge_sign <= 1'b0; st <= S_MM; end
                // ---- read_motion_mode (5.11.27): overlappable neighbours, warp samples, then the symbol
                S_MM: begin
                    motion_mode <= 2'd0;
                    if (skip_mode || !ihdr.motion_mode_switchable || bwp < 8'd8 || bhp < 8'd8 ||
                        (!ihdr.force_integer_mv && globalmv_fam && gmt2(rf0) > 2'd1) || is_comp || rf1 == R_INTRA) st <= S_CT;
                    else begin ov_above <= avail_u; ov_x <= bc; ov_y <= br; st <= S_OV; end
                end
                S_OV: begin         // has_overlappable_candidates (7.10.3): above row at odd columns, then left column at odd rows
                    if (ov_above) begin
                        if (ov_x < hdr.mi_cols && ov_x < bc + MIW'(bw4)) begin nb_row <= br - MIW'(1); nb_col <= ov_x | MIW'(1); st <= S_OV_W; end
                        else ov_above <= 1'b0;
                    end else begin
                        if (avail_l && ov_y < hdr.mi_rows && ov_y < br + MIW'(bh4)) begin nb_row <= ov_y | MIW'(1); nb_col <= bc - MIW'(1); st <= S_OV_W; end
                        else st <= S_CT;                                                    // none: SIMPLE
                    end
                end
                S_OV_W: st <= S_OV_P;
                S_OV_P: begin
                    if (mvr_written && mvr_ent.ref0 >= R_LAST && mvr_ent.ref0 <= R_ALTREF) begin
                        ws_found <= 1'b0; do_tl <= 1'b1; do_tr <= 1'b1; ws_i <= 6'd0;
                        ws_ph <= avail_u ? 3'd0 : (avail_l ? 3'd2 : 3'd4); st <= S_WS;
                    end
                    else begin if (ov_above) ov_x <= ov_x + MIW'(2); else ov_y <= ov_y + MIW'(2); st <= S_OV; end
                end
                // find_warp_samples (7.10.4): NumSamples == 0 iff no scanned neighbour has RefFrame[0] alone
                S_WS: begin
                    case (ws_ph)
                        3'd0: begin nb_row <= br - MIW'(1); nb_col <= bc; st <= S_WS_W; end
                        3'd1: if (ws_i < bw4 && MIW'(ws_i) < hdr.mi_cols - bc) begin nb_row <= br - MIW'(1); nb_col <= bc + MIW'(ws_i); st <= S_WS_W; end
                              else begin ws_ph <= avail_l ? 3'd2 : 3'd4; ws_i <= 6'd0; end
                        3'd2: begin nb_row <= br; nb_col <= bc - MIW'(1); st <= S_WS_W; end
                        3'd3: if (ws_i < bh4 && MIW'(ws_i) < hdr.mi_rows - br) begin nb_row <= br + MIW'(ws_i); nb_col <= bc - MIW'(1); st <= S_WS_W; end
                              else ws_ph <= 3'd4;
                        3'd4: if (do_tl) begin nb_row <= br - MIW'(1); nb_col <= bc - MIW'(1); st <= S_WS_W; end else ws_ph <= 3'd5;
                        3'd5: if (do_tr && bw4 <= 6'd16 && bh4 <= 6'd16) begin nb_row <= br - MIW'(1); nb_col <= bc + MIW'(bw4); st <= S_WS_W; end else st <= S_MM2;
                        default: st <= S_MM2;
                    endcase
                end
                S_WS_W: st <= S_WS_P;
                S_WS_P: begin
                    logic [5:0] srcw, srch;
                    logic in_tile;
                    srcw = num4x4w(mvr_ent.bsize); srch = num4x4h(mvr_ent.bsize);
                    // is_inside (a row / column above or left of 0 wraps to a large unsigned value and fails the end test)
                    in_tile = (nb_row >= hdr.mi_row_start) && (nb_row < hdr.mi_row_end) && (nb_col >= hdr.mi_col_start) && (nb_col < hdr.mi_col_end);
                    // add_sample: inside, written, RefFrames == [RefFrame[0], NONE]
                    if (in_tile && mvr_written && mvr_ent.ref0 == rf0 && mvr_ent.ref1 == R_NONE) ws_found <= 1'b1;
                    case (ws_ph)
                        3'd0: begin
                            if (bw4 <= srcw) begin
                                if ((bc & MIW'(srcw - 6'd1)) != MIW'(0)) do_tl <= 1'b0;
                                if (7'(srcw) - 7'(bc & MIW'(srcw - 6'd1)) > 7'(bw4)) do_tr <= 1'b0;
                                ws_ph <= avail_l ? 3'd2 : 3'd4; ws_i <= 6'd0;
                            end else begin ws_i <= (bw4 < srcw) ? bw4 : srcw; ws_ph <= 3'd1; end
                        end
                        3'd1: ws_i <= ws_i + ((bw4 < srcw) ? bw4 : srcw);
                        3'd2: begin
                            if (bh4 <= srch) begin
                                if ((br & MIW'(srch - 6'd1)) != MIW'(0)) do_tl <= 1'b0;
                                ws_ph <= 3'd4;
                            end else begin ws_i <= (bh4 < srch) ? bh4 : srch; ws_ph <= 3'd3; end
                        end
                        3'd3: ws_i <= ws_i + ((bh4 < srch) ? bh4 : srch);
                        3'd4: ws_ph <= 3'd5;
                        default: ws_ph <= 3'd6;
                    endcase
                    st <= S_WS;
                end
                S_MM2: begin
                    if (ihdr.force_integer_mv || !ws_found || !ihdr.allow_warped_motion || ihdr.ref_scaled[3'(rf0 - 4'd1)]) st <= S_OBMC_GO;
                    else st <= S_MMS_GO;
                end
                S_OBMC_GO: st <= S_OBMC_W;
                S_OBMC_W: if (sq_done) begin motion_mode <= sq_sym[0] ? 2'd1 : 2'd0; st <= S_CT; end
                S_MMS_GO: st <= S_MMS_W;
                S_MMS_W: if (sq_done) begin motion_mode <= sq_sym[1:0]; st <= S_CT; end
                // ---- read_compound_type (5.11.29)
                S_CT: begin
                    comp_group_idx <= 1'b0; compound_idx <= 1'b1;
                    if (skip_mode) begin compound_type <= 3'd2; st <= S_IF; end
                    else if (is_comp) st <= ihdr.enable_masked_compound ? S_CGI_GO : S_CT2;
                    else begin compound_type <= interintra ? (wedge_ii ? 3'd0 : 3'd3) : 3'd2; st <= S_IF; end
                end
                S_CGI_GO: st <= S_CGI_W;
                S_CGI_W: if (sq_done) begin comp_group_idx <= sq_sym[0]; st <= S_CT2; end
                S_CT2: begin
                    if (!comp_group_idx) begin
                        if (ihdr.enable_jnt_comp) st <= S_CIDX_GO;
                        else begin compound_type <= 3'd2; st <= S_IF; end
                    end else begin
                        if (wedge_bits(bs) == 3'd0) begin compound_type <= 3'd1; st <= S_CT3; end
                        else st <= S_CTYPE_GO;
                    end
                end
                S_CIDX_GO: st <= S_CIDX_W;
                S_CIDX_W: if (sq_done) begin compound_idx <= sq_sym[0]; compound_type <= sq_sym[0] ? 3'd2 : 3'd4; st <= S_IF; end
                S_CTYPE_GO: st <= S_CTYPE_W;
                S_CTYPE_W: if (sq_done) begin compound_type <= {2'd0, sq_sym[0]}; st <= S_CT3; end
                S_CT3: begin
                    if (compound_type == 3'd0) st <= S_WIDX2_GO;                                            // COMPOUND_WEDGE
                    else begin lit_n <= 4'd1; lit_val <= '0; lit_ret <= S_MASK_W; st <= S_LIT; end          // COMPOUND_DIFFWTD
                end
                S_WIDX2_GO: st <= S_WIDX2_W;
                S_WIDX2_W: if (sq_done) begin wedge_idx <= sq_sym; lit_n <= 4'd1; lit_val <= '0; lit_ret <= S_WSGN_W; st <= S_LIT; end
                S_WSGN_W: begin wedge_sign <= lit_val[0]; st <= S_IF; end
                S_MASK_W: begin mask_type <= lit_val[0]; st <= S_IF; end
                // ---- interp_filter (5.11.23 tail)
                S_IF: begin
                    if (ihdr.interp_filter == 3'd4) begin interp_d <= 1'b0; st <= S_IF_D; end
                    else begin interp[0] <= ihdr.interp_filter[1:0]; interp[1] <= ihdr.interp_filter[1:0]; st <= S_TXD; end
                end
                S_IF_D: begin
                    if (needs_if) st <= S_IF_GO;
                    else begin
                        interp[interp_d] <= 2'd0;
                        if (ihdr.enable_dual_filter && !interp_d) interp_d <= 1'b1; else st <= S_IF_END;
                    end
                end
                S_IF_GO: st <= S_IF_W;
                S_IF_W: if (sq_done) begin
                    interp[interp_d] <= sq_sym[1:0];
                    if (ihdr.enable_dual_filter && !interp_d) begin interp_d <= 1'b1; st <= S_IF_D; end else st <= S_IF_END;
                end
                S_IF_END: begin
                    if (!ihdr.enable_dual_filter) interp[1] <= interp[0];
                    st <= S_TXD;
                end
                S_IBC_GO: st <= S_IBC_W;
                S_IBC_W: if (sq_done) begin
                    if (sq_sym[0]) begin use_ibc <= 1'b1; read_deltas <= 1'b0; st <= S_MVS; end
                    else st <= S_YMODE;
                end
                // ---- intra block copy: find_mv_stack -> assign_mv / read_mv (MvCtx = 1; fr = 3, hp = 1 in intra frames)
                S_MVS: if (!mvm_busy && !mvs_busy) begin mvs_start <= 1'b1; st <= S_MVS_W; end
                S_MVS_W: if (mvs_done) st <= use_ibc ? S_MVJ : S_MODE;
                S_MVJ: st <= S_MVJ_W;
                S_MVJ_W: if (sq_done) begin mvj <= sq_sym[1:0]; mv_comp <= 2'd0; diff_row <= 18'sd0; diff_col <= 18'sd0; st <= S_MVC; end
                S_MVC: begin                                  // component 0 (row) if mv_joint is HZVNZ/HNZVNZ, 1 (col) if HNZVZ/HNZVNZ
                    if (mv_comp == 2'd2) st <= S_MVEND;
                    else if ((mv_comp == 2'd0 && mvj[1]) || (mv_comp == 2'd1 && mvj[0])) st <= S_MVSGN;
                    else mv_comp <= mv_comp + 2'd1;
                end
                S_MVSGN: st <= S_MVSGN_W;
                S_MVSGN_W: if (sq_done) begin mv_sgn <= sq_sym[0]; st <= S_MVCLS; end
                S_MVCLS: st <= S_MVCLS_W;
                S_MVCLS_W: if (sq_done) begin
                    mv_cls <= sq_sym; mv_d <= 10'd0; mv_bi <= 4'd0;
                    st <= (sq_sym == 4'd0) ? S_MVC0B : S_MVBIT;
                end
                S_MVC0B: st <= S_MVC0B_W;
                S_MVC0B_W: if (sq_done) begin mv_c0b <= sq_sym[0]; st <= S_MVFR; end
                S_MVBIT: st <= S_MVBIT_W;
                S_MVBIT_W: if (sq_done) begin
                    mv_d[mv_bi] <= sq_sym[0];
                    if (mv_bi + 4'd1 < mv_cls) begin mv_bi <= mv_bi + 4'd1; st <= S_MVBIT; end
                    else st <= S_MVFR;
                end
                S_MVFR: if (mv_int) begin mv_fr <= 2'd3; st <= S_MVHP; end else st <= S_MVFR_GO;
                S_MVFR_GO: st <= S_MVFR_W;
                S_MVFR_W: if (sq_done) begin mv_fr <= sq_sym[1:0]; st <= S_MVHP; end
                S_MVHP: if (!mv_hp_en) begin mv_hp <= 1'b1; st <= S_MVMAG; end else st <= S_MVHP_GO;
                S_MVHP_GO: st <= S_MVHP_W;
                S_MVHP_W: if (sq_done) begin mv_hp <= sq_sym[0]; st <= S_MVMAG; end
                S_MVMAG: begin
                    logic [17:0] mag;
                    if (mv_cls == 4'd0) mag = (18'(mv_c0b) << 3) + (18'(mv_fr) << 1) + 18'(mv_hp) + 18'd1;             // ((bit << 3) | (fr << 1) | hp) + 1
                    else mag = (18'd2 << (mv_cls + 4'd2)) + (18'(mv_d) << 3) + (18'(mv_fr) << 1) + 18'(mv_hp) + 18'd1;   // CLASS0_SIZE << (class + 2) + ...
                    if (mv_comp == 2'd0) diff_row <= mv_sgn ? -18'(signed'(mag)) : 18'(signed'(mag));
                    else diff_col <= mv_sgn ? -18'(signed'(mag)) : 18'(signed'(mag));
                    mv_comp <= mv_comp + 2'd1; st <= S_MVC;
                end
                S_MVEND: begin
                    if (use_ibc) begin
                        mv_row <= pred_row + diff_row; mv_col <= pred_col + diff_col;
                        ymode <= DC_PRED; uvmode <= DC_PRED; use_fi <= 1'b0; fi_mode <= 3'd0;
                        st <= S_TXD;
                    end else begin
                        if (!mv_list) begin mv_row <= pr_r + diff_row; mv_col <= pr_c + diff_col; end
                        else begin mv1_row <= pr_r + diff_row; mv1_col <= pr_c + diff_col; end
                        st <= S_AMV_NEXT;
                    end
                end
                // ---- modes  (ReadDeltas cleared here: after delta syntax, before anything else)
                S_YMODE: begin read_deltas <= 1'b0; st <= S_YMODE_W; end
                S_YMODE_W: if (sq_done) begin ymode <= sq_sym; st <= S_ANGY; end
                S_ANGY: begin
                    if (bs >= BLOCK_8X8 && is_dir(ymode)) st <= S_ANGY_GO;
                    else st <= hc ? S_UV : S_PAL;
                end
                S_ANGY_GO: st <= S_ANGY_W;
                S_ANGY_W: if (sq_done) begin ang_y <= 3'(sq_sym - 4'd3); st <= hc ? S_UV : S_PAL; end
                S_UV: st <= S_UV_W;
                S_UV_W: if (sq_done) begin
                    uvmode <= sq_sym;
                    st <= (sq_sym == UV_CFL_PRED) ? S_CFLS : S_ANGUV;
                end
                S_CFLS: st <= S_CFLS_W;
                S_CFLS_W: if (sq_done) begin
                    // signU = (signs+1)/3, signV = (signs+1)%3
                    sign_u <= 2'((5'(sq_sym) + 5'd1) / 5'd3);
                    sign_v <= 2'((5'(sq_sym) + 5'd1) % 5'd3);
                    st <= S_CFLU;
                end
                S_CFLU: begin
                    if (sign_u == 2'd0) begin cfl_u <= 6'sd0; st <= S_CFLV; end
                    else st <= S_CFLU_GO;
                end
                S_CFLU_GO: begin st <= S_CFLU_W;
                end
                S_CFLU_W: if (sq_done) begin
                    cfl_u <= (sign_u == 2'd1) ? -6'(6'd1 + 6'(sq_sym)) : 6'(6'd1 + 6'(sq_sym));   // CFL_SIGN_NEG = 1
                    st <= S_CFLV;
                end
                S_CFLV: begin
                    if (sign_v == 2'd0) begin cfl_v <= 6'sd0; st <= S_ANGUV; end
                    else st <= S_CFLV_GO;
                end
                S_CFLV_GO: begin st <= S_CFLV_W;
                end
                S_CFLV_W: if (sq_done) begin
                    cfl_v <= (sign_v == 2'd1) ? -6'(6'd1 + 6'(sq_sym)) : 6'(6'd1 + 6'(sq_sym));
                    st <= S_ANGUV;
                end
                S_ANGUV: begin
                    if (bs >= BLOCK_8X8 && is_dir(uvmode)) st <= S_ANGUV_GO;
                    else st <= S_PAL;
                end
                S_ANGUV_GO: st <= S_ANGUV_W;
                S_ANGUV_W: if (sq_done) begin ang_uv <= 3'(sq_sym - 4'd3); st <= S_PAL; end
                // ---- palette_mode_info (pal_syntax owns the symbols while in S_PAL_W)
                S_PAL: begin
                    if (pal_allowed) begin pal_start_mi <= 1'b1; st <= S_PAL_W; end
                    else st <= S_FI;
                end
                S_PAL_W: if (pal_done_mi) st <= S_FI;
                // ---- filter intra
                S_FI: begin
                    if (hdr.enable_filter_intra && ymode == DC_PRED && pal_y == 4'd0 && bwp <= 8'd32 && bhp <= 8'd32) st <= S_FI_GO;
                    else st <= S_PTOK;
                end
                S_FI_GO: st <= S_FI_W;
                S_FI_W: if (sq_done) begin use_fi <= sq_sym[0]; st <= sq_sym[0] ? S_FIM : S_PTOK; end
                S_FIM: st <= S_FIM_W;
                S_FIM_W: if (sq_done) begin fi_mode <= sq_sym[2:0]; st <= S_PTOK; end
                // ---- palette_tokens
                S_PTOK: begin
                    if (has_pal) begin pal_start_tok <= 1'b1; st <= S_PTOK_W; end
                    else st <= S_TXD;
                end
                S_PTOK_W: if (pal_done_tok) st <= S_TXD;
                // ---- tx size
                S_TXD: begin
                    if (lossless) begin txsz <= TX_4X4; st <= S_RBC; end
                    else begin
                        txsz <= max_rect;
                        if (inter_any) st <= S_VTX_INIT;
                        else if (bs > BLOCK_4X4 && hdr.tx_mode == 2'd2) st <= S_TXD_GO;
                        else st <= S_RBC;
                    end
                end
                // ---- read_block_tx_size for inter blocks: a walk over the max-tx units; with TX_MODE_SELECT and
                // coefficients the transform-size tree is read (read_var_tx_size), otherwise every unit is one leaf
                S_VTX_INIT: begin
                    vt_read <= (hdr.tx_mode == 2'd2) && (bs > BLOCK_4X4) && !skip;
                    vt_ur <= 6'd0; vt_uc <= 6'd0; vt_uw4 <= 6'(tx_width(max_rect) >> 2); vt_uh4 <= 6'(tx_height(max_rect) >> 2);
                    nd_r[0] <= 6'd0; nd_c[0] <= 6'd0; nd_tx[0] <= max_rect; vt_d <= 2'd0;
                    st <= S_VTX_NODE;
                end
                S_VTX_NODE: begin
                    if (MIW'(br + MIW'(vt_r)) >= hdr.mi_rows || MIW'(bc + MIW'(vt_c)) >= hdr.mi_cols) st <= S_VTX_POP;   // outside the frame
                    else if (!vt_read || vt_tx == TX_4X4 || vt_d == 2'd2) st <= S_VTX_LEAF;           // depth == MAX_VARTX_DEPTH
                    else st <= S_VTX_GO;
                end
                S_VTX_GO: st <= S_VTX_W;
                S_VTX_W: if (sq_done) begin
                    if (sq_sym[0]) begin
                        logic [4:0] sub;
                        sub = split_tx_size(vt_tx);
                        nd_two_c[vt_d] <= (tx_width(sub) < tx_width(vt_tx)); nd_two_r[vt_d] <= (tx_height(sub) < tx_height(vt_tx));
                        nd_ch[vt_d] <= 2'd0;
                        nd_r[vt_d + 2'd1] <= vt_r; nd_c[vt_d + 2'd1] <= vt_c; nd_tx[vt_d + 2'd1] <= sub;
                        vt_d <= vt_d + 2'd1; st <= S_VTX_NODE;
                    end else st <= S_VTX_LEAF;
                end
                S_VTX_LEAF: begin
                    leaf_r[n_leaves[5:0]] <= vt_r; leaf_c[n_leaves[5:0]] <= vt_c; leaf_tx[n_leaves[5:0]] <= vt_tx; leaf_tt[n_leaves[5:0]] <= 4'd0;
                    n_leaves <= n_leaves + 7'd1; txsz <= vt_tx;
                    for (int k = 0; k < 32; k++) begin
                        if (6'(k) >= vt_c && 7'(k) < 7'(vt_c) + 7'(tx_width(vt_tx) >> 2)) top_edge[k] <= vt_tx;
                        if (6'(k) >= vt_r && 7'(k) < 7'(vt_r) + 7'(tx_height(vt_tx) >> 2)) left_edge[k] <= vt_tx;
                    end
                    st <= S_VTX_POP;
                end
                S_VTX_POP: begin
                    if (vt_d == 2'd0) begin                                   // next max-tx unit, raster order
                        if (vt_uc + vt_uw4 < bw4) begin vt_uc <= vt_uc + vt_uw4; nd_c[0] <= vt_uc + vt_uw4; st <= S_VTX_NODE; end
                        else if (vt_ur + vt_uh4 < bh4) begin vt_uc <= 6'd0; nd_c[0] <= 6'd0; vt_ur <= vt_ur + vt_uh4; nd_r[0] <= vt_ur + vt_uh4; st <= S_VTX_NODE; end
                        else st <= S_RBC;
                        nd_tx[0] <= max_rect;
                    end else begin                                             // next child of the parent, raster order
                        logic [1:0] pd;
                        logic [2:0] k, nch;
                        logic [4:0] sub;
                        logic [5:0] ci, cj;
                        pd = vt_d - 2'd1;
                        k = 3'(nd_ch[pd]) + 3'd1;                                     // next child (0..3), full width
                        nch = 3'((nd_two_c[pd] ? 2 : 1) * (nd_two_r[pd] ? 2 : 1));
                        sub = split_tx_size(nd_tx[pd]);
                        if (nd_two_c[pd]) begin ci = 6'(k >> 1); cj = 6'(k & 3'd1); end
                        else begin ci = 6'(k); cj = 6'd0; end
                        if (k < nch) begin
                            nd_ch[pd] <= k[1:0];
                            nd_r[vt_d] <= nd_r[pd] + 6'(ci * 6'(tx_height(sub) >> 2));
                            nd_c[vt_d] <= nd_c[pd] + 6'(cj * 6'(tx_width(sub) >> 2));
                            nd_tx[vt_d] <= sub;
                            st <= S_VTX_NODE;
                        end else vt_d <= pd;                                  // all children done: pop (stay in S_VTX_POP)
                    end
                end
                S_TXD_GO: st <= S_TXD_W;
                S_TXD_W: if (sq_done) begin
                    txsz <= (sq_sym == 4'd0) ? max_rect : (sq_sym == 4'd1) ? split_tx_size(max_rect) : split_tx_size(split_tx_size(max_rect));
                    st <= S_RBC;
                end
                S_RBC: if (!mvm_busy) begin
                    // the block record is complete here (everything but the residual): publish it for the
                    // reconstruction stage before the transform blocks stream out; the motion info goes to mv_mem
                    mvm_we <= 1'b1;
                    blk_rec.is_inter <= inter_any; blk_rec.intrabc <= use_ibc; blk_rec.mv_row <= mv_row; blk_rec.mv_col <= mv_col;
                    blk_rec.skip_mode <= skip_mode; blk_rec.ref0 <= is_inter_b ? rf0 : R_INTRA; blk_rec.ref1 <= is_inter_b ? rf1 : R_NONE;
                    blk_rec.mv1_row <= mv1_row; blk_rec.mv1_col <= mv1_col; blk_rec.motion_mode <= motion_mode; blk_rec.compound_type <= compound_type;
                    blk_rec.interintra <= interintra; blk_rec.interintra_mode <= ii_mode; blk_rec.wedge_interintra <= wedge_ii; blk_rec.wedge_index <= wedge_idx;
                    blk_rec.wedge_sign <= wedge_sign; blk_rec.mask_type <= mask_type; blk_rec.comp_group_idx <= comp_group_idx; blk_rec.compound_idx <= compound_idx;
                    blk_rec.interp0 <= interp[0]; blk_rec.interp1 <= interp[1];
                    blk_rec.r <= br; blk_rec.c <= bc; blk_rec.bsize <= bs; blk_rec.skip <= skip; blk_rec.seg <= seg;
                    blk_rec.lossless <= lossless; blk_rec.has_chroma <= hc; blk_rec.ymode <= is_inter_b ? imode : {1'b0, ymode}; blk_rec.uvmode <= uvmode;
                    blk_rec.angle_y <= ang_y; blk_rec.angle_uv <= ang_uv; blk_rec.cfl_u <= cfl_u; blk_rec.cfl_v <= cfl_v;
                    blk_rec.use_fi <= use_fi; blk_rec.fi_mode <= fi_mode; blk_rec.txsz <= txsz; blk_rec.qidx <= cur_qidx;
                    blk_rec.delta_lf <= {dlf[3], dlf[2], dlf[1], dlf[0]};
                    blk_rec.cdef_valid <= cdef_valid; blk_rec.cdef_idx <= cdef_val; blk_rec.cdef_units <= cdef_units;
                    blk_rec.pal_y <= pal_y; blk_rec.pal_uv <= pal_uv; blk_rec.col_y <= col_y; blk_rec.col_u <= col_u; blk_rec.col_v <= col_v;
                    blk_info <= 1'b1;
                    if (skip) rbc_we <= 1'b1;
                    // residual init
                    wchunks2 <= (bwp > 8'd64); hchunks2 <= (bhp > 8'd64);
                    chunk_bs <= (bwp > 8'd64 || bhp > 8'd64) ? BLOCK_64X64 : bs;
                    chunk_x <= 1'b0; chunk_y <= 1'b0; plane <= 2'd0;
                    st <= S_RES_PLANE;
                end
                // ---- residual: per (chunk, plane) setup
                S_RES_PLANE: if (!ctx_tbusy) begin
                    p_txsz <= lossless ? TX_4X4 : (plane == 2'd0 ? txsz : uv_tx(bs, hdr.ssx, hdr.ssy));
                    begin
                        logic [4:0] psz;
                        logic [4:0] ptx;
                        ptx = lossless ? TX_4X4 : (plane == 2'd0 ? txsz : uv_tx(bs, hdr.ssx, hdr.ssy));
                        psz = subsampled_size(chunk_bs, sub_x, sub_y);
                        step_x <= 6'(tx_width(ptx) >> 2); step_y <= 6'(tx_height(ptx) >> 2);
                        n4w <= num4x4w(psz); n4h <= num4x4h(psz);
                    end
                    base_x <= (PXW'(bc) >> sub_x) << 2; base_y <= (PXW'(br) >> sub_y) << 2;
                    rx <= 6'd0; ry <= 6'd0;
                    leaf_mode <= (plane == 2'd0) && inter_any && !lossless;
                    st <= ((plane == 2'd0) && inter_any && !lossless) ? S_RES_LEAF : S_RES_TX;
                end
                // ---- inter luma: transform_tree = the leaves of this chunk in order
                S_RES_LEAF: begin
                    if (leaf_rd < n_leaves && 2'(leaf_r[leaf_rd[5:0]] >> 4) == {1'b0, chunk_y} && 2'(leaf_c[leaf_rd[5:0]] >> 4) == {1'b0, chunk_x}) begin
                        p_txsz <= leaf_tx[leaf_rd[5:0]]; st <= S_RES_TX;
                    end else st <= S_RES_PLANE_END;
                end
                // ---- one transform block
                S_RES_TX: begin
                    all_zero <= 1'b1; p_txtype <= 4'd0; t_eob <= 11'd0;
                    if (start_x >= max_x || start_y >= max_y) st <= S_RES_NEXT;
                    else if (skip) st <= S_TX_EMIT;
                    else begin tx_req <= 1'b1; st <= S_TX_CTX_W; end
                end
                S_TX_CTX_W: if (tx_valid) begin cf_start_a <= 1'b1; st <= S_TX_A_W; end
                S_TX_A_W: if (cf_done_a) begin
                    all_zero <= cf_all_zero;
                    if (cf_all_zero) st <= S_TX_UPD;
                    else if (plane == 2'd0 && cur_set_eff != 2'd0 && cur_qidx_seg != 8'd0 && !lossless) st <= S_TXTYPE;
                    else if (plane != 2'd0 && inter_any && !lossless && tx_sqr_up(p_txsz) <= 3'd3) begin uvt_k <= 7'd0; st <= S_TX_UVT; end
                    else begin
                        // compute_tx_type without a read: luma -> DCT_DCT; chroma -> Mode_To_Txfm[UVMode] if in set
                        if (lossless || tx_sqr_up(p_txsz) > 3'd3 || plane == 2'd0) p_txtype <= 4'd0;
                        else p_txtype <= tx_type_in_set_intra(cur_set, mode_to_txfm(uvmode)) ? mode_to_txfm(uvmode) : 4'd0;
                        cf_start_b <= 1'b1; st <= S_TX_B_W;
                    end
                end
                S_TXTYPE: st <= S_TXTYPE_W;
                S_TXTYPE_W: if (sq_done) begin
                    if (inter_any) p_txtype <= (cur_set_i == 2'd1) ? tx_type_inter_inv_set1(sq_sym) : (cur_set_i == 2'd2) ? tx_type_inter_inv_set2(sq_sym) : tx_type_inter_inv_set3(sq_sym[0]);
                    else p_txtype <= (cur_set == 2'd1) ? tx_type_intra_inv_set1(sq_sym[2:0]) : tx_type_intra_inv_set2(sq_sym[2:0]);
                    cf_start_b <= 1'b1; st <= S_TX_B_W;
                end
                // ---- chroma inter transform type: the co-located luma leaf's type, if in the set (compute_tx_type)
                S_TX_UVT: begin
                    if (uvt_hit) begin
                        p_txtype <= tx_type_in_set_inter(cur_set_i, leaf_tt[uvt_k[5:0]]) ? leaf_tt[uvt_k[5:0]] : 4'd0;
                        cf_start_b <= 1'b1; st <= S_TX_B_W;
                    end else if (uvt_k + 7'd1 < n_leaves) uvt_k <= uvt_k + 7'd1;
                    else begin p_txtype <= 4'd0; cf_start_b <= 1'b1; st <= S_TX_B_W; end
                end
                S_TX_B_W: if (cf_done_b) begin t_eob <= cf_eob; st <= S_TX_UPD; end
                S_TX_UPD: begin
                    tx_we <= 1'b1; st <= S_TX_EMIT;
                    if (leaf_mode) leaf_tt[leaf_rd[5:0]] <= all_zero ? 4'd0 : p_txtype;       // TxTypes of the leaf
                end
                S_TX_EMIT: begin
                    if (!tx_done) begin
                        tx_done <= 1'b1;
                        tx_rec.plane <= plane; tx_rec.x <= start_x; tx_rec.y <= start_y; tx_rec.txsz <= p_txsz;
                        tx_rec.txtype <= p_txtype; tx_rec.eob <= t_eob; tx_rec.skip <= skip; tx_rec.lossless <= lossless;
                        tx_rec.slot <= 3'd0;                                   // filled in by tile_syntax (rec_fifo slot)
                    end else if (tx_ack) begin
                        tx_done <= 1'b0; st <= S_RES_NEXT;
                    end
                end
                S_RES_NEXT: begin
                    if (leaf_mode) begin leaf_rd <= leaf_rd + 7'd1; st <= S_RES_LEAF; end
                    else if (rx + step_x < n4w) begin rx <= rx + step_x; st <= S_RES_TX; end
                    else if (ry + step_y < n4h) begin rx <= 6'd0; ry <= ry + step_y; st <= S_RES_TX; end
                    else st <= S_RES_PLANE_END;
                end
                S_RES_PLANE_END: begin
                    rx <= 6'd0; ry <= 6'd0; leaf_mode <= 1'b0;
                    if (plane < (hc ? 2'd2 : 2'd0)) begin plane <= plane + 2'd1; st <= S_RES_PLANE; end
                    else begin
                        plane <= 2'd0;
                        if (wchunks2 && !chunk_x) begin chunk_x <= 1'b1; st <= S_RES_PLANE; end
                        else if (hchunks2 && !chunk_y) begin chunk_x <= 1'b0; chunk_y <= 1'b1; st <= S_RES_PLANE; end
                        else st <= S_END;
                    end
                end
                // ---- block end
                S_END: if (!ctx_wbusy) begin ctx_we <= 1'b1; st <= S_END_W; end
                S_END_W: st <= has_pal ? S_HOLD : S_DONE;
                S_HOLD: if (blk_ack) st <= S_DONE;          // colour map readable through pm_* meanwhile
                S_DONE: begin blk_done <= 1'b1; st <= S_IDLE; end
                // ---- literal reader: lit_n equiprobable bools, MSB first
                S_LIT: begin
                    if (lit_n == 4'd0) st <= lit_ret;
                    else st <= S_LIT_W;
                end
                S_LIT_W: if (sq_done) begin
                    lit_val <= {lit_val[14:0], sq_sym[0]};
                    lit_n <= lit_n - 4'd1;
                    st <= S_LIT;
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
