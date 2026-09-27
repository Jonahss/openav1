// decode_block( ) for intra frames (spec 5.11.5 .. 5.11.39 minus coeffs, which coef_rd owns):
// intra_frame_mode_info -> palette_mode_info (pal_syntax) -> filter_intra -> palette_tokens (pal_syntax)
// -> tx size -> residual loop. One symbol at a time through the shared symbol sequencer; neighbour
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
    // control
    input  logic        sb_start,             // superblock start: ReadDeltas := delta_q_present, cdef flags := 0
    input  logic        tile_start,           // CurrentQIndex := base_q_idx, DeltaLF := 0
    input  logic        start,
    input  logic [10:0] r, c,
    input  logic [4:0]  bsize,
    output logic        busy,
    output logic        blk_info,             // blk_rec valid: pulsed before the block's transform blocks
    output logic        blk_done,
    output blk_rec_t    blk_rec,
    output logic        unsupported,          // palette / intrabc path hit (sticky)
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
    input  logic [3:0]  seg_ul, seg_u, seg_l,
    output logic        ctx_we,
    output logic [3:0]  w_ymode,
    output logic        w_skip,
    output logic [2:0]  w_seg,
    output logic [4:0]  w_txsz,
    input  logic        ctx_wbusy,
    output logic        tx_req,
    output logic [1:0]  tx_plane,
    output logic [10:0] tx_x4, tx_y4,
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
    input  logic [1:0]  cf_dccat
);
    localparam logic [4:0] BLOCK_4X4 = 5'd0, BLOCK_8X8 = 5'd3, BLOCK_64X64 = 5'd12;
    localparam logic [3:0] DC_PRED = 4'd0, UV_CFL_PRED = 4'd13;
    localparam logic [4:0] TX_4X4 = 5'd0, TX_16X16 = 5'd2, TX_32X32 = 5'd3, TX_64X64 = 5'd4, TX_16X32 = 5'd9, TX_32X16 = 5'd10;

    typedef enum logic [6:0] {
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
        S_PAL, S_PAL_W, S_PTOK, S_PTOK_W, S_HOLD,
        S_LIT, S_LIT_W
    } st_t;
    st_t st, lit_ret;

    // ---------------------------------------------------------------- block registers
    logic [10:0] br, bc;
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
    logic [12:0] base_x, base_y, start_x, start_y, max_x, max_y;
    logic        sub_x, sub_y;
    logic [3:0]  p_txtype;
    logic [10:0] t_eob;
    logic        all_zero;
    logic [4:0]  chunk_bs;

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
    logic [1:0] cur_set;
    logic [7:0] cur_qidx_seg;
    logic [3:0] intra_dir;
    always_comb begin
        cur_set = tx_set(p_txsz, hdr.reduced_tx_set);
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
    logic [6:0] above_w, left_h;
    always_comb begin
        max_rect = max_tx_size_rect(bs);
        max_depth = max_tx_depth(bs);
        // unavailable neighbours count as width/height 0 (tile_model.tx_depth_cdf, matches dav1d)
        above_w = avail_u ? tx_width(a_txsz) : 7'd0;
        left_h  = avail_l ? tx_height(l_txsz) : 7'd0;
        txd_ctx = 2'(above_w >= tx_width(max_rect)) + 2'(left_h >= tx_height(max_rect));
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
    logic [12:0] rem_w, rem_h;
    logic [7:0]  os_w, os_h;                     // onscreen block dims (luma pixels)
    always_comb begin
        rem_w = (13'(hdr.mi_cols) - 13'(bc)) << 2;
        rem_h = (13'(hdr.mi_rows) - 13'(br)) << 2;
        os_w = (rem_w < 13'(bwp)) ? rem_w[7:0] : bwp;
        os_h = (rem_h < 13'(bhp)) ? rem_h[7:0] : bhp;
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

    // ---------------------------------------------------------------- symbol requests (combinational)
    always_comb begin
        sq_go = 1'b0; sq_addr = '0; sq_n = 4'd1; sq_kind = 2'd0;
        case (st)
            S_SEG_A, S_SEG_B_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_SEGMENT_ID + int'(seg_ctx)); sq_n = 4'd7; end
            S_SKIP_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_SKIP + ((avail_u && a_skip) ? 1 : 0) + ((avail_l && l_skip) ? 1 : 0)); sq_n = 4'd1; end
            S_DQ_GO: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_DELTA_Q); sq_n = 4'd3; end
            S_DLF_GO: begin sq_go = 1'b1; sq_addr = hdr.delta_lf_multi ? CDF_AW'(CDF_DELTA_LF_MULTI + int'(dlf_i)) : CDF_AW'(CDF_DELTA_LF); sq_n = 4'd3; end
            S_YMODE: begin sq_go = 1'b1; sq_addr = CDF_AW'(CDF_INTRA_FRAME_Y_MODE + int'(intra_mode_ctx(avail_u ? a_ymode : DC_PRED)) * CDF_INTRA_FRAME_Y_MODE_S0
                                                          + int'(intra_mode_ctx(avail_l ? l_ymode : DC_PRED))); sq_n = 4'd12; end
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
            S_TXTYPE: begin
                sq_go = 1'b1;
                if (cur_set == 2'd1) begin sq_addr = CDF_AW'(CDF_INTRA_TX_TYPE_SET1 + int'(tx_sqr(p_txsz)) * CDF_INTRA_TX_TYPE_SET1_S0 + int'(intra_dir)); sq_n = 4'd6; end
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
    assign tx_x4 = 11'(start_x >> 2);
    assign tx_y4 = 11'(start_y >> 2);
    assign tx_sz_o = p_txsz;
    assign tx_bsize = subsampled_size(bs, sub_x, sub_y);
    assign w_ymode = ymode; assign w_skip = skip; assign w_seg = seg; assign w_txsz = txsz;
    assign w_cul = all_zero ? 6'd0 : cf_cul;
    assign w_dccat = all_zero ? 2'd0 : cf_dccat;

    // residual geometry
    always_comb begin
        sub_x = (plane != 2'd0) && hdr.ssx;
        sub_y = (plane != 2'd0) && hdr.ssy;
        max_x = 13'(({2'b0, hdr.mi_cols} << 2) >> sub_x);
        max_y = 13'(({2'b0, hdr.mi_rows} << 2) >> sub_y);
        start_x = base_x + ((13'(rx) + (chunk_x ? (13'd16 >> sub_x) : 13'd0)) << 2);
        start_y = base_y + ((13'(ry) + (chunk_y ? (13'd16 >> sub_y) : 13'd0)) << 2);
    end

    // ---------------------------------------------------------------- main FSM
    always_ff @(posedge clk) begin
        blk_done <= 1'b0; nb_req <= 1'b0; ctx_we <= 1'b0; tx_req <= 1'b0; tx_we <= 1'b0; rbc_we <= 1'b0;
        cf_start_a <= 1'b0; cf_start_b <= 1'b0; pal_clr <= 1'b0; pal_start_mi <= 1'b0; pal_start_tok <= 1'b0; blk_info <= 1'b0;
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
                    nb_req <= 1'b1; pal_clr <= 1'b1;
                    st <= S_NB;
                end
                S_NB: if (nb_valid) begin
                    hc <= has_chroma;
                    st <= hdr.seg_preskip ? (hdr.seg_enabled ? S_SEG_A : S_SKIP) : S_SKIP;
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
                    if (!hdr.seg_preskip && hdr.seg_enabled) st <= S_SEG_B;
                    else st <= S_CDEF;
                end
                // ---- segment id after skip
                S_SEG_B: begin
                    if (skip) begin seg <= seg_pred[2:0]; lossless <= hdr.lossless[seg_pred[2:0]]; st <= S_CDEF; end
                    else st <= S_SEG_B_GO;
                end
                S_SEG_B_GO: begin st <= S_SEG_B_W;
                end
                S_SEG_B_W: if (sq_done) begin
                    seg <= neg_deinterleave(sq_sym, seg_pred, 4'(hdr.last_active_segid) + 4'd1);
                    lossless <= hdr.lossless[neg_deinterleave(sq_sym, seg_pred, 4'(hdr.last_active_segid) + 4'd1)];
                    st <= S_CDEF;
                end
                // ---- cdef idx
                S_CDEF: begin
                    if (skip || hdr.coded_lossless || !hdr.enable_cdef || cdef_flags[cdef_unit]) st <= S_DQ;
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
                    if ((bs == sb_bs && skip) || !read_deltas) st <= S_YMODE;
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
                    if ((bs == sb_bs && skip) || !read_deltas || !hdr.delta_lf_present) st <= S_YMODE;
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
                    if (dlf_i == dlf_cnt) st <= S_YMODE;
                    else begin dlf_i <= dlf_i + 2'd1; st <= S_DLF_GO; end
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
                        if (bs > BLOCK_4X4 && hdr.tx_mode == 2'd2) st <= S_TXD_GO;
                        else st <= S_RBC;
                    end
                end
                S_TXD_GO: st <= S_TXD_W;
                S_TXD_W: if (sq_done) begin
                    txsz <= (sq_sym == 4'd0) ? max_rect : (sq_sym == 4'd1) ? split_tx_size(max_rect) : split_tx_size(split_tx_size(max_rect));
                    st <= S_RBC;
                end
                S_RBC: begin
                    // the block record is complete here (everything but the residual): publish it for the
                    // reconstruction stage before the transform blocks stream out
                    blk_rec.r <= br; blk_rec.c <= bc; blk_rec.bsize <= bs; blk_rec.skip <= skip; blk_rec.seg <= seg;
                    blk_rec.lossless <= lossless; blk_rec.has_chroma <= hc; blk_rec.ymode <= ymode; blk_rec.uvmode <= uvmode;
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
                    base_x <= (13'(bc) >> sub_x) << 2; base_y <= (13'(br) >> sub_y) << 2;
                    rx <= 6'd0; ry <= 6'd0;
                    st <= S_RES_TX;
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
                    else if (plane == 2'd0 && cur_set != 2'd0 && cur_qidx_seg != 8'd0 && !lossless) st <= S_TXTYPE;
                    else begin
                        // compute_tx_type without a read: luma -> DCT_DCT; chroma -> Mode_To_Txfm[UVMode] if in set
                        if (lossless || tx_sqr_up(p_txsz) > 3'd3 || plane == 2'd0) p_txtype <= 4'd0;
                        else p_txtype <= tx_type_in_set_intra(cur_set, mode_to_txfm(uvmode)) ? mode_to_txfm(uvmode) : 4'd0;
                        cf_start_b <= 1'b1; st <= S_TX_B_W;
                    end
                end
                S_TXTYPE: st <= S_TXTYPE_W;
                S_TXTYPE_W: if (sq_done) begin
                    p_txtype <= (cur_set == 2'd1) ? tx_type_intra_inv_set1(sq_sym[2:0]) : tx_type_intra_inv_set2(sq_sym[2:0]);
                    cf_start_b <= 1'b1; st <= S_TX_B_W;
                end
                S_TX_B_W: if (cf_done_b) begin t_eob <= cf_eob; st <= S_TX_UPD; end
                S_TX_UPD: begin tx_we <= 1'b1; st <= S_TX_EMIT; end
                S_TX_EMIT: begin
                    if (!tx_done) begin
                        tx_done <= 1'b1;
                        tx_rec.plane <= plane; tx_rec.x <= start_x; tx_rec.y <= start_y; tx_rec.txsz <= p_txsz;
                        tx_rec.txtype <= p_txtype; tx_rec.eob <= t_eob; tx_rec.skip <= skip; tx_rec.lossless <= lossless;
                    end else if (tx_ack) begin
                        tx_done <= 1'b0; st <= S_RES_NEXT;
                    end
                end
                S_RES_NEXT: begin
                    if (rx + step_x < n4w) begin rx <= rx + step_x; st <= S_RES_TX; end
                    else if (ry + step_y < n4h) begin rx <= 6'd0; ry <= ry + step_y; st <= S_RES_TX; end
                    else begin
                        rx <= 6'd0; ry <= 6'd0;
                        if (plane < (hc ? 2'd2 : 2'd0)) begin plane <= plane + 2'd1; st <= S_RES_PLANE; end
                        else begin
                            plane <= 2'd0;
                            if (wchunks2 && !chunk_x) begin chunk_x <= 1'b1; st <= S_RES_PLANE; end
                            else if (hchunks2 && !chunk_y) begin chunk_x <= 1'b0; chunk_y <= 1'b1; st <= S_RES_PLANE; end
                            else st <= S_END;
                        end
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
