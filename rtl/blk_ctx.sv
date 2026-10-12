// Block-level neighbour context storage for the tile syntax decoder (see the architecture note,
// "Neighbour context storage"). Holds, for the current tile:
//   * above[col]  : per 4x4 column, the most recent block's {ymode, skip, misize, txsize, palette sizes}
//   * left[row&31]: same per 4x4 row within the superblock row
//   * seg_strip[32][col] + seg_prev[col]: segment ids of the current superblock row and of the last row of
//     the previous one (segment_id prediction needs the above-left neighbour, which 1D arrays cannot give)
//   * above/left palette colours (indexed by col&31 / row&31; only consulted inside the same 64x64)
//   * Above/LeftLevelContext + Above/LeftDcContext per plane (coefficient contexts), packed 16 per word
// Reads are one-shot requests (blk_req / tx_req) answered a few cycles later; writes are multi-cycle
// with a busy flag. Word packing: 16 4x4 units per word so any aligned block or transform block touches at
// most 2 words per row.
// Storage (v2, mappable): the three big arrays (segment strip, above records, above level contexts) are
// block RAMs with one write port and one registered read port; every access is sequenced (a query reads
// its words one per clock, a partial-word update is read-then-write over two clocks) and the tile-start
// clear of the level contexts is a valid bit per word. The small per-superblock arrays stay registers.
module blk_ctx
  import blk_tables_pkg::*;
  import syn_pkg::*;
#(
    parameter int CAW = 6                 // log2(words of 16 columns): 6 -> 1024 columns = 4096 px
)(
    input  logic        clk,
    input  logic        rst,
    // tile parameters
    input  logic [MIW-1:0] mi_cols, mi_rows,
    input  logic [MIW-1:0] mi_col_start, mi_col_end, mi_row_start, mi_row_end,
    input  logic        ssx, ssy, mono,
    input  logic        sb128,
    input  logic        clear_above,          // tile start: coefficient contexts above := 0
    input  logic        clear_left,           // superblock row start: coefficient contexts left := 0
    input  logic        sbrow_end,            // superblock row end: seg_prev := last strip row
    // ---- block neighbour query -------------------------------------------------------------------
    input  logic        blk_req,
    input  logic [MIW-1:0] blk_r, blk_c,
    input  logic [4:0]  blk_bsize,
    output logic        nb_valid,
    output logic        avail_u, avail_l, has_chroma, avail_u_chroma, avail_l_chroma,
    output logic [3:0]  a_ymode, l_ymode,
    output logic        a_skip, l_skip,
    output logic [4:0]  a_misize, l_misize,
    output logic [4:0]  a_txsz, l_txsz,
    output logic        a_is_inter, l_is_inter,
    output logic [16*24*2-1:0] a_recs, l_recs,   // 32 above records (columns MiCol&~15 ..) / left records (rows MiRow&~15 ..)
    output logic [3:0]  a_pal_y, l_pal_y, a_pal_uv, l_pal_uv,
    output logic [3:0]  seg_ul, seg_u, seg_l,     // 4'hF = -1 (unavailable)
    output logic [95:0] a_col_y, l_col_y, a_col_u, l_col_u,
    // ---- block end write (uses the last blk_req position/size) ------------------------------------
    input  logic        blk_we,
    input  logic [3:0]  w_ymode,
    input  logic        w_skip,
    input  logic [2:0]  w_seg,
    input  logic [4:0]  w_txsz,
    input  logic        w_is_inter,
    input  logic        w_vartx,               // per-column / per-row transform sizes (InterTxSizes edges)
    input  logic [159:0] w_txsz_col, w_txsz_row,   // column j of the block: [5j +: 5]; row i: [5i +: 5]
    input  logic [3:0]  w_pal_y, w_pal_uv,
    input  logic [95:0] w_col_y, w_col_u,
    output logic        wbusy,
    // ---- transform-block coefficient contexts --------------------------------------------------------
    input  logic        tx_req,
    input  logic [1:0]  tx_plane,
    input  logic [MIW-1:0] tx_x4, tx_y4,          // plane units
    input  logic [4:0]  tx_sz,
    input  logic [4:0]  tx_bsize,              // get_plane_residual_size(MiSize, plane)
    output logic        tx_valid,
    output logic [3:0]  az_ctx,
    output logic [1:0]  dcs_ctx,
    input  logic        tx_we,                 // after coeffs(): write culLevel/dcCategory for the tx block
    input  logic [5:0]  w_cul,
    input  logic [1:0]  w_dccat,
    input  logic        rbc_we                 // reset_block_context (skip blocks): zero the block's ctx entries
);
    localparam int REC_W = 24;                 // {is_inter, pal_uv[3:0], pal_y[3:0], txsz[4:0], misize[4:0], skip, ymode[3:0]}
    localparam int WORDS = 1 << CAW;

    // ------------------------------------------------------------------ storage
    // RAMs (one write, one registered read each): the segment strip [row(5)][word(CAW)] x 16 x 3b, the above
    // records per word, the above level contexts {plane, word} x 16 x {dc[1:0], lvl[5:0]}
    logic [47:0]         seg_strip [0:32*WORDS-1];
    logic                sg_we; logic [CAW+4:0] sg_waddr, sg_raddr; logic [47:0] sg_wdata, sg_rdata;
    always_ff @(posedge clk) begin
        if (sg_we) seg_strip[sg_waddr] <= sg_wdata;
        sg_rdata <= seg_strip[sg_raddr];
    end
    logic [16*REC_W-1:0] above_rec [0:WORDS-1];
    logic                ar_we; logic [CAW-1:0] ar_waddr, ar_raddr; logic [16*REC_W-1:0] ar_wdata, ar_rdata;
    always_ff @(posedge clk) begin
        if (ar_we) above_rec[ar_waddr] <= ar_wdata;
        ar_rdata <= above_rec[ar_raddr];
    end
    logic [127:0]        a_lvl [0:3*WORDS-1];
    logic                al_we; logic [CAW+1:0] al_waddr, al_raddr; logic [127:0] al_wdata, al_rdata_m, al_rdata;
    logic [3*WORDS-1:0]  al_ok;                             // word written since the tile start (clear_above = all 0)
    logic                al_ok_q;
    always_ff @(posedge clk) begin
        if (al_we) a_lvl[al_waddr] <= al_wdata;
        al_rdata_m <= a_lvl[al_raddr];
        al_ok_q <= al_ok[al_raddr];
    end
    assign al_rdata = al_ok_q ? al_rdata_m : 128'd0;
    always_ff @(posedge clk) begin
        if (clear_above) al_ok <= '0;
        else if (al_we) al_ok[al_waddr] <= 1'b1;
    end
    // registers: left records, segment ids of the previous superblock row's last row, palettes, left levels
    logic [16*REC_W-1:0] left_rec  [0:1];
    logic [47:0]         seg_prev  [0:WORDS-1];
    logic [95:0]         a_pal [0:1][0:31];                  // plane, col & 31
    logic [95:0]         l_pal [0:1][0:31];
    logic [127:0]        l_lvl [0:2][0:1];

    // ------------------------------------------------------------------ helpers
    function automatic logic is_inside(input logic [MIW-1:0] r, input logic [MIW-1:0] c,
                                    input logic [MIW-1:0] cs, input logic [MIW-1:0] ce, input logic [MIW-1:0] rs, input logic [MIW-1:0] re);
        // r, c may be "negative" (wrapped) -> compare as signed 12-bit
        is_inside = ($signed({1'b0, c}) >= $signed({1'b0, cs})) && (c < ce) && ($signed({1'b0, r}) >= $signed({1'b0, rs})) && (r < re);
    endfunction

    // ------------------------------------------------------------------ block query (registered pipeline)
    logic [MIW-1:0] q_r, q_c;
    logic [4:0]  q_bs;
    logic [5:0]  q_bw4, q_bh4;
    logic        q_au, q_al;
    logic [4:0]  sbm;                          // row mask inside the superblock (15 or 31)
    assign sbm = sb128 ? 5'd31 : 5'd15;
    logic [MIW-1:0] rm1, cm1;
    assign rm1 = q_r - MIW'(1);
    assign cm1 = q_c - MIW'(1);

    typedef enum logic [2:0] {Q_IDLE, Q_WAIT, Q_RD, Q_RD1, Q_RD2, Q_RD3, Q_OUT} q_t;
    q_t qst;
    logic q_top;                               // the block sits on the superblock's top row: row r-1 is in seg_prev
    assign q_top = ((q_r[4:0] & sbm) == 5'd0);
    logic w_idle;                              // no block-end write in flight (declared below)
    logic [16*REC_W-1:0] a_word, l_word, a_word1, l_word1;
    logic [47:0] seg_w_u, seg_w_ul, seg_w_l;
    logic [95:0] ap_y, ap_u, lp_y, lp_u;
    always_ff @(posedge clk) begin
        nb_valid <= 1'b0;
        if (rst) qst <= Q_IDLE;
        else case (qst)
            Q_IDLE: if (blk_req) begin
                // a query must see the previous block's (multi-cycle) context write: wait for it
                q_r <= blk_r; q_c <= blk_c; q_bs <= blk_bsize;
                q_bw4 <= num4x4w(blk_bsize); q_bh4 <= num4x4h(blk_bsize);
                qst <= Q_WAIT;
            end
            Q_WAIT: if (w_idle) qst <= Q_RD;
            // RAM reads one word per clock (addresses in the port mux below), landing the clock after:
            //   Q_RD: strip(u), above(c)     Q_RD1: strip(ul), above(c+1)     Q_RD2: strip(l)     Q_RD3: last lands
            Q_RD: begin
                l_word  <= left_rec[q_r[4]];
                l_word1 <= left_rec[~q_r[4]];
                if (q_top) begin seg_w_u <= seg_prev[q_c[CAW+3:4]]; seg_w_ul <= seg_prev[cm1[CAW+3:4]]; end
                ap_y <= a_pal[0][q_c[4:0]]; ap_u <= a_pal[1][q_c[4:0]];
                lp_y <= l_pal[0][q_r[4:0]]; lp_u <= l_pal[1][q_r[4:0]];
                qst <= Q_RD1;
            end
            Q_RD1: begin if (!q_top) seg_w_u <= sg_rdata; a_word <= ar_rdata; qst <= Q_RD2; end
            Q_RD2: begin if (!q_top) seg_w_ul <= sg_rdata; a_word1 <= ar_rdata; qst <= Q_RD3; end
            Q_RD3: begin seg_w_l <= sg_rdata; qst <= Q_OUT; end
            Q_OUT: begin
                nb_valid <= 1'b1;
                qst <= Q_IDLE;
            end
            default: qst <= Q_IDLE;
        endcase
    end

    // availability (spec is_inside on the tile bounds)
    logic au, al, hc, auc, alc;
    always_comb begin
        au = is_inside(rm1, q_c, mi_col_start, mi_col_end, mi_row_start, mi_row_end);
        al = is_inside(q_r, cm1, mi_col_start, mi_col_end, mi_row_start, mi_row_end);
        if (q_bh4 == 6'd1 && ssy && q_r[0] == 1'b0) hc = 1'b0;
        else if (q_bw4 == 6'd1 && ssx && q_c[0] == 1'b0) hc = 1'b0;
        else hc = !mono;
        auc = au; alc = al;
        if (hc) begin
            if (ssy && q_bh4 == 6'd1) auc = is_inside(q_r - MIW'(2), q_c, mi_col_start, mi_col_end, mi_row_start, mi_row_end);
            if (ssx && q_bw4 == 6'd1) alc = is_inside(q_r, q_c - MIW'(2), mi_col_start, mi_col_end, mi_row_start, mi_row_end);
        end else begin
            auc = 1'b0; alc = 1'b0;
        end
    end
    assign avail_u = au; assign avail_l = al; assign has_chroma = hc;
    assign avail_u_chroma = auc; assign avail_l_chroma = alc;

    // unpack records
    logic [REC_W-1:0] a_rec, l_rec;
    assign a_rec = a_word[REC_W*q_c[3:0] +: REC_W];
    assign l_rec = l_word[REC_W*q_r[3:0] +: REC_W];
    assign a_ymode = a_rec[3:0];   assign l_ymode = l_rec[3:0];
    assign a_skip = a_rec[4];      assign l_skip = l_rec[4];
    assign a_misize = a_rec[9:5];  assign l_misize = l_rec[9:5];
    assign a_txsz = a_rec[14:10];  assign l_txsz = l_rec[14:10];
    assign a_pal_y = a_rec[18:15]; assign l_pal_y = l_rec[18:15];
    assign a_pal_uv = a_rec[22:19]; assign l_pal_uv = l_rec[22:19];
    assign a_is_inter = a_rec[23]; assign l_is_inter = l_rec[23];
    assign a_recs = {a_word1, a_word};
    assign l_recs = {l_word1, l_word};
    assign seg_u  = au ? {1'b0, seg_w_u[3*q_c[3:0] +: 3]} : 4'hF;
    assign seg_l  = al ? {1'b0, seg_w_l[3*cm1[3:0] +: 3]} : 4'hF;
    assign seg_ul = (au && al) ? {1'b0, seg_w_ul[3*cm1[3:0] +: 3]} : 4'hF;
    assign a_col_y = ap_y; assign a_col_u = ap_u; assign l_col_y = lp_y; assign l_col_u = lp_u;

    // ------------------------------------------------------------------ block end write FSM
    // rows: seg strip words for each of bh4 rows (1 or 2 words); above records (1 or 2 words); left records
    // (1 or 2 words); palette colours per column (bw4, only if any palette) and per row (bh4).
    typedef enum logic [3:0] {W_IDLE, W_SEG, W_SEG_W, W_AREC, W_AREC_W, W_LREC, W_PALA, W_PALL, W_SBROW, W_SBROW_W} w_t;
    w_t wst;
    logic [5:0]  wi;                            // row / column counter
    logic        wj;                            // word half (0/1) for 32-wide blocks
    logic [REC_W-1:0] wrec;
    logic        wvartx;
    logic [159:0] wtx_col, wtx_row;
    logic [2:0]  wseg;
    logic [95:0] wcol_y, wcol_u;
    logic        wpal;
    logic [CAW-1:0] cp_i;
    logic sbrow_pend;                          // sbrow_end arrived while a block write was in flight
    assign wbusy = (wst != W_IDLE) || blk_we || sbrow_end || sbrow_pend;
    assign w_idle = (wst == W_IDLE) && !blk_we && !sbrow_end && !sbrow_pend;

    function automatic logic [47:0] merge_seg(input logic [47:0] old, input logic [3:0] c0, input logic [5:0] n, input logic [2:0] v);
        // set entries c0 .. c0+n-1 (n<=16, c0+n<=16) to v
        merge_seg = old;
        for (int k = 0; k < 16; k++)
            if (k >= int'(c0) && k < int'(c0) + int'(n)) merge_seg[3*k +: 3] = v;
    endfunction
    function automatic logic [16*REC_W-1:0] merge_rec(input logic [16*REC_W-1:0] old, input logic [3:0] c0, input logic [5:0] n, input logic [REC_W-1:0] v,
                                                      input logic vartx, input logic [159:0] txs, input logic [4:0] j0);
        // entries c0 .. c0+n-1 := v; with vartx the tx size of entry k comes from txs[j0 + k - c0]
        merge_rec = old;
        for (int k = 0; k < 16; k++)
            if (k >= int'(c0) && k < int'(c0) + int'(n)) begin
                merge_rec[REC_W*k +: REC_W] = v;
                if (vartx) merge_rec[REC_W*k + 10 +: 5] = txs[5 * (int'(j0) + k - int'(c0)) +: 5];
            end
    endfunction

    // per-word geometry: a block is aligned to its size; for width 32 it covers two whole words
    logic [MIW-1:0] wr_r, wr_c;
    logic [5:0]  wr_bw4, wr_bh4;
    logic [5:0] n_cols_word, n_rows_word;
    assign n_cols_word = wr_bw4 > 6'd16 ? 6'd16 : wr_bw4;
    assign n_rows_word = wr_bh4 > 6'd16 ? 6'd16 : wr_bh4;
    logic [MIW-1:0] wrow;
    assign wrow = wr_r + MIW'(wi);
    logic [CAW-1:0] wcol_word;
    assign wcol_word = wr_c[CAW+3:4] + CAW'(wj);
    logic [MIW-1:0] wcol4;
    assign wcol4 = wr_c + MIW'(wi);

    always_ff @(posedge clk) begin
        if (rst) begin
            wst <= W_IDLE; sbrow_pend <= 1'b0;
        end else case (wst)
            W_IDLE: begin
                if (sbrow_end) sbrow_pend <= 1'b1;
                if (blk_we) begin
                    // latch everything the multi-cycle write needs: the block FSM moves on to the next block
                    // (and clears its palette state) while this write is still in flight
                    wrec <= {w_is_inter, w_pal_uv, w_pal_y, w_txsz, q_bs, w_skip, w_ymode};
                    wvartx <= w_vartx; wtx_col <= w_txsz_col; wtx_row <= w_txsz_row;
                    wcol_y <= w_col_y; wcol_u <= w_col_u; wpal <= (w_pal_y != 4'd0) || (w_pal_uv != 4'd0);
                    wr_r <= q_r; wr_c <= q_c; wr_bw4 <= q_bw4; wr_bh4 <= q_bh4; wseg <= w_seg;
                    wi <= 6'd0; wj <= 1'b0;
                    wst <= W_SEG;
                end else if ((sbrow_end || sbrow_pend) && (qst == Q_IDLE || qst == Q_WAIT || qst == Q_OUT)) begin
                    sbrow_pend <= 1'b0;                    // (a query's reads share the strip RAM port: let them finish)
                    cp_i <= '0; wst <= W_SBROW;
                end
            end
            W_SEG: begin                                   // the word is read this clock (address below), merged next
                if (sbrow_end) sbrow_pend <= 1'b1;
                wst <= W_SEG_W;
            end
            W_SEG_W: begin
                if (sbrow_end) sbrow_pend <= 1'b1;
                wst <= W_SEG;
                if (wr_bw4 > 6'd16 && !wj) wj <= 1'b1;
                else begin
                    wj <= 1'b0;
                    if (wi + 6'd1 >= wr_bh4) begin wi <= 6'd0; wst <= W_AREC; end
                    else wi <= wi + 6'd1;
                end
            end
            W_AREC: begin                                  // read the record word, merge and write next clock
                if (sbrow_end) sbrow_pend <= 1'b1;
                wst <= W_AREC_W;
            end
            W_AREC_W: begin
                if (sbrow_end) sbrow_pend <= 1'b1;
                if (wr_bw4 > 6'd16 && !wj) begin wj <= 1'b1; wst <= W_AREC; end
                else begin wj <= 1'b0; wst <= W_LREC; end
            end
            W_LREC: begin
                if (sbrow_end) sbrow_pend <= 1'b1;
                left_rec[wr_r[4] ^ wj] <= merge_rec(left_rec[wr_r[4] ^ wj], wj ? 4'd0 : wr_r[3:0], n_rows_word, wrec, wvartx, wtx_row, wj ? 5'd16 : 5'd0);
                if (wr_bh4 > 6'd16 && !wj) wj <= 1'b1;
                else begin wj <= 1'b0; wi <= 6'd0; wst <= wpal ? W_PALA : W_IDLE; end
            end
            W_PALA: begin
                if (sbrow_end) sbrow_pend <= 1'b1;                                  // palette blocks are <= 64 px: bw4, bh4 <= 16
                a_pal[0][wcol4[4:0]] <= wcol_y;
                a_pal[1][wcol4[4:0]] <= wcol_u;
                if (wi + 6'd1 >= wr_bw4) begin wi <= 6'd0; wst <= W_PALL; end
                else wi <= wi + 6'd1;
            end
            W_PALL: begin
                if (sbrow_end) sbrow_pend <= 1'b1;
                l_pal[0][wrow[4:0]] <= wcol_y;
                l_pal[1][wrow[4:0]] <= wcol_u;
                if (wi + 6'd1 >= wr_bh4) begin wi <= 6'd0; wst <= W_IDLE; end
                else wi <= wi + 6'd1;
            end
            W_SBROW: begin                                 // seg_prev := last row of the superblock row: read word cp_i,
                wst <= W_SBROW_W;                          // it lands next clock
            end
            W_SBROW_W: begin
                seg_prev[cp_i] <= sg_rdata;
                if (cp_i == CAW'(WORDS - 1)) wst <= W_IDLE; else wst <= W_SBROW;
                cp_i <= cp_i + 1'b1;
            end
            default: wst <= W_IDLE;
        endcase
    end

    // ---- segment strip and above record RAM ports: the write FSM owns them while busy, queries otherwise
    always_comb begin
        sg_we = 1'b0; sg_waddr = {wrow[4:0] & sbm, wcol_word}; sg_wdata = merge_seg(sg_rdata, wj ? 4'd0 : wr_c[3:0], n_cols_word, wseg);
        ar_we = 1'b0; ar_waddr = wcol_word; ar_wdata = merge_rec(ar_rdata, wj ? 4'd0 : wr_c[3:0], n_cols_word, wrec, wvartx, wtx_col, wj ? 5'd16 : 5'd0);
        sg_raddr = {wrow[4:0] & sbm, wcol_word}; ar_raddr = wcol_word;
        case (wst)
            W_SEG_W:  sg_we = (wrow < mi_rows);
            W_AREC_W: ar_we = 1'b1;
            W_SBROW, W_SBROW_W: sg_raddr = {sbm, cp_i};
            default: ;
        endcase
        case (qst)
            Q_RD:  begin sg_raddr = {rm1[4:0] & sbm, q_c[CAW+3:4]};  ar_raddr = q_c[CAW+3:4]; end
            Q_RD1: begin sg_raddr = {rm1[4:0] & sbm, cm1[CAW+3:4]};  ar_raddr = q_c[CAW+3:4] + CAW'(1); end
            Q_RD2: sg_raddr = {q_r[4:0] & sbm, cm1[CAW+3:4]};
            default: ;
        endcase
    end

    // ------------------------------------------------------------------ coefficient contexts
    // all_zero_ctx / dc_sign_ctx over the transform block's above (w4 entries) and left (h4 entries) ranges;
    // ranges are aligned so each lies within one 16-entry word.
    logic [1:0]  t_plane;
    logic [MIW-1:0] t_x4, t_y4;
    logic [5:0]  t_w4, t_h4;
    logic [4:0]  t_sz, t_bs;
    logic [127:0] t_aw, t_lw;
    typedef enum logic [2:0] {T_IDLE, T_RD, T_RD2, T_OUT, T_LW} t_t;   // T_LW: level write, word read the clock before
    t_t tst;
    logic t_pend, t_we_pend, rbc_pend;
    logic [MIW-1:0] max_x4, max_y4;
    assign max_x4 = (t_plane != 2'd0 && ssx) ? (mi_cols >> 1) : mi_cols;
    assign max_y4 = (t_plane != 2'd0 && ssy) ? (mi_rows >> 1) : mi_rows;

    // The context aggregates (max level, nonzero, dc-sign sum over the block's above and left entries) are
    // computed from the above word as it lands from the RAM (al_rdata, T_RD2) and the already-read left word,
    // and registered; T_OUT derives the contexts from the registers. (Computed in T_OUT from t_aw, the
    // 16-entry max trees plus the context compare chain were tile_syntax's worst path, 44 LUT levels.)
    logic [7:0] top_max, left_max;
    logic       above_nz, left_nz;
    logic [7:0] bw, bh;
    logic [6:0] tw, th;
    logic [7:0] top_max_r, left_max_r;
    logic       above_nz_r, left_nz_r;
    // Which entries of the above and left words belong to the block: the range masks depend only on the request,
    // so they are formed in T_RD, the cycle before the above word lands.
    logic [15:0] a_msk_c, l_msk_c, a_msk, l_msk;
    always_comb for (int k = 0; k < 16; k++) begin
        a_msk_c[k] = (k >= int'(t_x4[3:0])) && (k < int'(t_x4[3:0]) + int'(t_w4)) && ((t_x4 & ~MIW'(15)) + MIW'(k) < max_x4);
        l_msk_c[k] = (k >= int'(t_y4[3:0])) && (k < int'(t_y4[3:0]) + int'(t_h4)) && ((t_y4 & ~MIW'(15)) + MIW'(k) < max_y4);
    end
    // The aggregates as balanced trees over the masked entries. (As a loop carrying a running maximum they were a
    // 16-deep compare chain behind the RAM output, 38 LUT levels: tile_syntax's worst path.)
    function automatic logic [5:0] mx(input logic [5:0] a, input logic [5:0] b);
        mx = (a > b) ? a : b;
    endfunction
    function automatic logic [5:0] max16(input logic [95:0] v);       // sixteen 6-bit entries
        logic [5:0] m8 [8];
        logic [5:0] m4 [4];
        logic [5:0] m2 [2];
        for (int k = 0; k < 8; k++) m8[k] = mx(v[12 * k +: 6], v[12 * k + 6 +: 6]);
        for (int k = 0; k < 4; k++) m4[k] = mx(m8[2 * k], m8[2 * k + 1]);
        for (int k = 0; k < 2; k++) m2[k] = mx(m4[2 * k], m4[2 * k + 1]);
        max16 = mx(m2[0], m2[1]);
    endfunction
    logic [95:0] a_lv, l_lv;                    // the entries' levels, zero outside the block
    logic [15:0] a_nzb, l_nzb, a_plus, a_minus, l_plus, l_minus;
    logic signed [7:0] dcs_wide, dcs_wide_r;    // dc-sign sum over the block's entries (+1 code 2, -1 code 1)
    always_comb begin
        for (int k = 0; k < 16; k++) begin
            logic [7:0] ae, le;
            ae = al_rdata[8*k +: 8]; le = t_lw[8*k +: 8];
            a_lv[6*k +: 6] = a_msk[k] ? ae[5:0] : 6'd0;   l_lv[6*k +: 6] = l_msk[k] ? le[5:0] : 6'd0;
            a_nzb[k]   = a_msk[k] && (ae != 8'd0);         l_nzb[k]   = l_msk[k] && (le != 8'd0);
            a_plus[k]  = a_msk[k] && (ae[7:6] == 2'd2);    l_plus[k]  = l_msk[k] && (le[7:6] == 2'd2);
            a_minus[k] = a_msk[k] && (ae[7:6] == 2'd1);    l_minus[k] = l_msk[k] && (le[7:6] == 2'd1);
        end
        top_max = 8'(max16(a_lv)); left_max = 8'(max16(l_lv));
        above_nz = |a_nzb; left_nz = |l_nzb;
        dcs_wide = 8'($countones({a_plus, l_plus})) - 8'($countones({a_minus, l_minus}));
        bw = blk_w(t_bs); bh = blk_h(t_bs); tw = tx_width(t_sz); th = tx_height(t_sz);
    end
    logic [3:0] az_now;
    always_comb begin
        if (t_plane == 2'd0) begin
            if (bw == 8'(tw) && bh == 8'(th)) az_now = 4'd0;
            else if (top_max_r == 0 && left_max_r == 0) az_now = 4'd1;
            else if (top_max_r == 0 || left_max_r == 0) az_now = 4'd2 + 4'((top_max_r > left_max_r ? top_max_r : left_max_r) > 8'd3);
            else if ((top_max_r > left_max_r ? top_max_r : left_max_r) <= 8'd3) az_now = 4'd4;
            else if ((top_max_r < left_max_r ? top_max_r : left_max_r) <= 8'd3) az_now = 4'd5;
            else az_now = 4'd6;
        end else begin
            az_now = 4'd7 + 4'(above_nz_r) + 4'(left_nz_r) + ((16'(bw) * 16'(bh) > 16'(tw) * 16'(th)) ? 4'd3 : 4'd0);
        end
    end

    // write helpers for the level words
    function automatic logic [127:0] merge_lvl(input logic [127:0] old, input logic [3:0] c0, input logic [5:0] n, input logic [7:0] v);
        merge_lvl = old;
        for (int k = 0; k < 16; k++)
            if (k >= int'(c0) && k < int'(c0) + int'(n)) merge_lvl[8*k +: 8] = v;
    endfunction

    // reset_block_context: zero the block's above columns / left rows in every plane (multi-cycle)
    typedef enum logic [2:0] {R_IDLE, R_A, R_A_W, R_L} r_t;
    r_t rstt;
    logic [1:0] rp;
    logic       rj;
    logic [MIW-1:0] rc0, rr0;             // plane-unit start col/row
    logic [5:0]  rn_c, rn_r;           // plane-unit counts
    logic [MIW-1:0] rb_r, rb_c;
    logic [5:0]  rb_bw4, rb_bh4;
    logic        rb_hc;
    always_comb begin
        rc0 = (rp != 0 && ssx) ? (rb_c >> 1) : rb_c;
        rr0 = (rp != 0 && ssy) ? (rb_r >> 1) : rb_r;
        rn_c = (rp != 0 && ssx) ? 6'(((rb_c + MIW'(rb_bw4)) >> 1) - (rb_c >> 1)) : rb_bw4;
        rn_r = (rp != 0 && ssy) ? 6'(((rb_r + MIW'(rb_bh4)) >> 1) - (rb_r >> 1)) : rb_bh4;
    end
    logic tbusy;
    assign tbusy = (tst != T_IDLE) || (rstt != R_IDLE);
    logic [7:0] t_wd;                          // {dc category, cul level} being written in T_LW
    // ---- above level context RAM port: the reset engine, the level write, the context read
    always_comb begin
        al_we = 1'b0; al_waddr = {t_plane, t_x4[CAW+3:4]}; al_raddr = {t_plane, t_x4[CAW+3:4]};
        al_wdata = merge_lvl(al_rdata, t_x4[3:0], 6'(t_w4), t_wd);
        case (rstt)
            R_A:   al_raddr = {rp, rc0[CAW+3:4] + CAW'(rj)};
            R_A_W: begin al_we = 1'b1; al_waddr = {rp, rc0[CAW+3:4] + CAW'(rj)};
                         al_wdata = merge_lvl(al_rdata, rj ? 4'd0 : rc0[3:0], rn_c > 6'd16 ? 6'd16 : rn_c, 8'd0); end
            default: if (tst == T_LW) al_we = 1'b1;
        endcase
    end

    always_ff @(posedge clk) begin
        tx_valid <= 1'b0;
        if (rst) begin
            tst <= T_IDLE; rstt <= R_IDLE; t_pend <= 1'b0; t_we_pend <= 1'b0; rbc_pend <= 1'b0;
        end else begin
            if (clear_left)  for (int p = 0; p < 3; p++) begin l_lvl[p][0] <= '0; l_lvl[p][1] <= '0; end
            case (tst)
                T_IDLE: if (tx_req || t_pend) begin
                    if (tx_req) begin
                        t_plane <= tx_plane; t_x4 <= tx_x4; t_y4 <= tx_y4; t_sz <= tx_sz; t_bs <= tx_bsize;
                        t_w4 <= 6'(tx_width(tx_sz) >> 2); t_h4 <= 6'(tx_height(tx_sz) >> 2);
                    end
                    if (rstt == R_IDLE && !rbc_we) begin t_pend <= 1'b0; tst <= T_RD; end
                    else t_pend <= 1'b1;
                end else if (tx_we || t_we_pend) begin     // the above word is read this clock (address below), merged and
                    if (tx_we) begin
                        l_lvl[t_plane][t_y4[4]] <= merge_lvl(l_lvl[t_plane][t_y4[4]], t_y4[3:0], 6'(t_h4), {w_dccat, w_cul});
                        t_wd <= {w_dccat, w_cul};          // written in T_LW
                    end
                    if (rstt == R_IDLE && !rbc_we) begin t_we_pend <= 1'b0; tst <= T_LW; end
                    else t_we_pend <= 1'b1;                // the reset engine owns the level RAM: write afterwards
                end
                T_LW: tst <= T_IDLE;
                T_RD: begin                                // above word read this clock, lands in T_RD2
                    t_lw <= l_lvl[t_plane][t_y4[4]];
                    a_msk <= a_msk_c; l_msk <= l_msk_c;
                    tst <= T_RD2;
                end
                T_RD2: begin
                    t_aw <= al_rdata; tst <= T_OUT;
                    top_max_r <= top_max; left_max_r <= left_max; above_nz_r <= above_nz; left_nz_r <= left_nz;
                    dcs_wide_r <= dcs_wide;
                end
                T_OUT: begin
                    az_ctx <= az_now;
                    dcs_ctx <= (dcs_wide_r < 0) ? 2'd1 : (dcs_wide_r > 0) ? 2'd2 : 2'd0;
                    tx_valid <= 1'b1;
                    tst <= T_IDLE;
                end
                default: tst <= T_IDLE;
            endcase
            // reset_block_context
            case (rstt)
                R_IDLE: begin
                    if (rbc_we) begin rb_r <= q_r; rb_c <= q_c; rb_bw4 <= q_bw4; rb_bh4 <= q_bh4; rb_hc <= hc; end
                    if ((rbc_we || rbc_pend) && tst != T_LW && !t_we_pend) begin rbc_pend <= 1'b0; rp <= 2'd0; rj <= 1'b0; rstt <= R_A; end
                    else if (rbc_we) rbc_pend <= 1'b1;     // a level write is using the RAM: start afterwards
                end
                R_A: rstt <= R_A_W;                        // above: columns rc0 .. rc0+rn_c-1 (aligned; up to 2 words):
                R_A_W: begin                               // word read in R_A, zeroed entries written here
                    if (rn_c > 6'd16 && !rj) begin rj <= 1'b1; rstt <= R_A; end
                    else begin rj <= 1'b0; rstt <= R_L; end
                end
                R_L: begin
                    l_lvl[rp][rr0[4] ^ rj] <= merge_lvl(l_lvl[rp][rr0[4] ^ rj], rj ? 4'd0 : rr0[3:0], rn_r > 6'd16 ? 6'd16 : rn_r, 8'd0);
                    if (rn_r > 6'd16 && !rj) rj <= 1'b1;
                    else begin
                        rj <= 1'b0;
                        if (rp == 2'd2 || (rp == 2'd0 && !rb_hc)) rstt <= R_IDLE;
                        else begin rp <= rp + 2'd1; rstt <= R_A; end
                    end
                end
                default: rstt <= R_IDLE;
            endcase
        end
    end
endmodule
