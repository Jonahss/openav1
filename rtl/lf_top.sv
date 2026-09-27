// Deblocking loop filter (spec 7.14), frame-level, in the spec's order: for each plane, pass 0 (vertical
// edges) over the whole frame, then pass 1 (horizontal edges); per 4x4 edge position the edge parameters
// (7.14.2), filter size (7.14.3) and strength (7.14.4/5) come from mi_store, then the four samples along the
// edge are filtered one at a time (7.14.6: masks, narrow filter, 6/8/14-tap wide filters) read-modify-write
// through the single-port frame buffer. Intra frames only (all references INTRA_FRAME, modeType 0).
// Slow-and-correct: ~25 cycles per sample position. A real design filters 4-8 samples per clock out of
// line buffers and interleaves the passes per superblock.
module lf_top
  import blk_tables_pkg::*;
  import tx_tables_pkg::*;
  import syn_pkg::*;
#(
    parameter int FBX = 10,
    parameter int FBY = 9
) (
    input  logic        clk,
    input  logic        rst,
    input  hdr_t        hdr,
    input  lf_hdr_t     lh,
    input  logic        start,
    output logic        busy,
    output logic        done,
    // mi_store reads (registered, 1 cycle)
    output logic [10:0] rd_row, rd_col,
    input  mi_lf_t      rd_info,
    output logic [1:0]  txr_plane,
    output logic [10:0] txr_row, txr_col,
    input  logic [4:0]  txr_sz,
    // frame buffer
    output logic        fb_re,
    output logic        fb_we,
    output logic [1:0]  fb_plane,
    output logic [FBX-1:0] fb_x,
    output logic [FBY-1:0] fb_y,
    output logic [11:0] fb_wdata,
    input  logic [11:0] fb_rdata
);
    localparam int MAX_LOOP_FILTER = 63;

    typedef enum logic [3:0] {L_IDLE, L_EDGE, L_RD1, L_RD2, L_CALC, L_S_RD, L_S_CALC, L_S_WR, L_NEXT, L_DONE} st_t;
    st_t st;

    // ---------------------------------------------------------------- loop state
    logic [1:0]  plane;
    logic        pss;                                   // 0: vertical edges (dx=1), 1: horizontal edges (dy=1)
    logic [10:0] row, col;
    logic        sub_x, sub_y;
    logic [10:0] rowa, cola, prow, pcol;                // adjusted (row | subY, col | subX) and previous positions
    logic [12:0] x, y, xp, yp;
    mi_lf_t      cur, prv;
    logic [4:0]  cur_tx, prv_tx;
    logic [1:0]  i4;                                    // sample index along the edge (0..3)
    logic [4:0]  filter_size;                           // 4, 8 or 16
    logic [5:0]  lvl;
    logic [7:0]  limit, blimit, thresh;
    logic [11:0] pix [0:13];                            // pix[7 + k] = pixel k (k = -7..6): p_j = pix[6-j], q_j = pix[7+j]
    logic [3:0]  kk, n_rd;                              // read counter / count
    logic        rd_pend;
    logic [3:0]  rd_slot;
    logic signed [4:0] wk;                              // write position k
    logic signed [4:0] w_lo, w_hi;
    logic [12:0] sx, sy;                                // sample position

    always_comb begin
        sub_x = (plane != 2'd0) && hdr.ssx;
        sub_y = (plane != 2'd0) && hdr.ssy;
        x = 13'(col) << 2; y = 13'(row) << 2;
        rowa = row | 11'(sub_y); cola = col | 11'(sub_x);
        xp = x >> sub_x; yp = y >> sub_y;
        prow = pss ? rowa - (11'd1 << sub_y) : rowa;
        pcol = pss ? cola : cola - (11'd1 << sub_x);
        sx = pss ? xp + 13'(i4) : xp;
        sy = pss ? yp : yp + 13'(i4);
    end

    // ---------------------------------------------------------------- 7.14.4 / 7.14.5 strength
    function automatic logic [5:0] clip63(input logic signed [9:0] v);
        clip63 = (v < 0) ? 6'd0 : (v > 10'sd63) ? 6'd63 : 6'(v);
    endfunction
    function automatic logic [29:0] strength(input mi_lf_t info, input logic [1:0] pl, input logic ps);
        // returns {lvl[5:0], limit[7:0], blimit[7:0], thresh[7:0]}
        logic [1:0] i;
        logic signed [6:0] dlf;
        logic [5:0] base, lvl_seg, lv;
        logic [4:0] seg_i;
        logic [1:0] sh;
        logic [5:0] lim;
        i = (pl == 2'd0) ? 2'(ps) : 2'(pl + 2'd1);
        dlf = lh.delta_lf_multi ? 7'(info.delta_lf[7 * i +: 7]) : 7'(info.delta_lf[6:0]);
        base = clip63(10'(dlf) + 10'(signed'({4'b0, lh.level[6 * i +: 6]})));
        lvl_seg = base;
        seg_i = {i, info.seg};
        if (hdr.seg_enabled && lh.seg_lf_en[seg_i])
            lvl_seg = clip63(10'(signed'(lh.seg_lf_data[7 * seg_i +: 7])) + 10'(signed'({4'b0, lvl_seg})));
        if (lh.delta_enabled)
            lvl_seg = clip63(10'(signed'({4'b0, lvl_seg})) + (10'(lh.ref_delta_intra) <<< (lvl_seg >> 5)));
        lv = lvl_seg;
        sh = (lh.sharpness > 3'd4) ? 2'd2 : (lh.sharpness > 3'd0) ? 2'd1 : 2'd0;
        if (lh.sharpness > 3'd0) begin
            lim = lv >> sh;
            if (lim < 6'd1) lim = 6'd1;
            if (lim > 6'(4'd9 - 4'(lh.sharpness))) lim = 6'(4'd9 - 4'(lh.sharpness));
        end else begin
            lim = lv >> sh;
            if (lim < 6'd1) lim = 6'd1;
        end
        begin
            logic [8:0] bl;
            bl = (({3'b0, lv} + 9'd2) << 1) + {3'b0, lim};          // blimit = 2 * (lvl + 2) + limit
            strength = {lv, 8'(lim), 8'(bl), 8'(lv >> 4)};
        end
    endfunction
    logic [29:0] str_cur, str_prv;
    always_comb begin
        str_cur = strength(cur, plane, pss);
        str_prv = strength(prv, plane, pss);
    end

    // ---------------------------------------------------------------- 7.14.2 / 7.14.3 edge decision (combinational from cur/prv)
    logic [4:0]  plane_size;
    logic        is_blk_edge, is_tx_edge, apply_filter;
    logic [6:0]  base_size;
    logic [4:0]  fsize_c;
    always_comb begin
        plane_size = subsampled_size(cur.bsize, sub_x, sub_y);
        if (!pss) begin
            is_blk_edge = (xp & (13'(blk_w(plane_size)) - 13'd1)) == 13'd0;
            is_tx_edge  = (xp & (13'(tx_width(cur_tx)) - 13'd1)) == 13'd0;
            base_size = (tx_width(prv_tx) < tx_width(cur_tx)) ? tx_width(prv_tx) : tx_width(cur_tx);
        end else begin
            is_blk_edge = (yp & (13'(blk_h(plane_size)) - 13'd1)) == 13'd0;
            is_tx_edge  = (yp & (13'(tx_height(cur_tx)) - 13'd1)) == 13'd0;
            base_size = (tx_height(prv_tx) < tx_height(cur_tx)) ? tx_height(prv_tx) : tx_height(cur_tx);
        end
        apply_filter = is_tx_edge && (is_blk_edge || !cur.skip || 1'b1);     // isIntra: always true for intra frames
        fsize_c = (plane == 2'd0) ? ((base_size > 7'd16) ? 5'd16 : 5'(base_size)) : ((base_size > 7'd8) ? 5'd8 : 5'(base_size));
    end

    // ---------------------------------------------------------------- 7.14.6 sample filter datapath (from pix[])
    logic [3:0]  bd8;
    assign bd8 = hdr.bit_depth - 4'd8;
    logic [12:0] p [0:6];
    logic [12:0] q [0:6];
    always_comb for (int j = 0; j < 7; j++) begin p[j] = 13'(pix[6 - j]); q[j] = 13'(pix[7 + j]); end
    function automatic logic [12:0] adiff(input logic [12:0] a, input logic [12:0] b);
        adiff = (a > b) ? a - b : b - a;
    endfunction
    logic [4:0]  filter_len;
    logic        hev, filter_mask, flat, flat2;
    logic [15:0] thresh_bd, limit_bd, blimit_bd, one_bd;
    always_comb begin
        filter_len = (filter_size == 5'd4) ? 5'd4 : (plane != 2'd0) ? 5'd6 : (filter_size == 5'd8) ? 5'd8 : 5'd16;
        thresh_bd = 16'(thresh) << bd8; limit_bd = 16'(limit) << bd8; blimit_bd = 16'(blimit) << bd8; one_bd = 16'd1 << bd8;
        hev = (16'(adiff(p[1], p[0])) > thresh_bd) || (16'(adiff(q[1], q[0])) > thresh_bd);
        begin
            logic m;
            m = (16'(adiff(p[1], p[0])) > limit_bd) || (16'(adiff(q[1], q[0])) > limit_bd) ||
                ((16'(adiff(p[0], q[0])) * 2 + (16'(adiff(p[1], q[1])) >> 1)) > blimit_bd);
            if (filter_len >= 5'd6) m = m || (16'(adiff(p[2], p[1])) > limit_bd) || (16'(adiff(q[2], q[1])) > limit_bd);
            if (filter_len >= 5'd8) m = m || (16'(adiff(p[3], p[2])) > limit_bd) || (16'(adiff(q[3], q[2])) > limit_bd);
            filter_mask = !m;
        end
        begin
            logic m;
            m = (16'(adiff(p[1], p[0])) > one_bd) || (16'(adiff(q[1], q[0])) > one_bd) ||
                (16'(adiff(p[2], p[0])) > one_bd) || (16'(adiff(q[2], q[0])) > one_bd);
            if (filter_len >= 5'd8) m = m || (16'(adiff(p[3], p[0])) > one_bd) || (16'(adiff(q[3], q[0])) > one_bd);
            flat = (filter_size >= 5'd8) && !m;
        end
        begin
            logic m;
            m = (16'(adiff(p[6], p[0])) > one_bd) || (16'(adiff(q[6], q[0])) > one_bd) ||
                (16'(adiff(p[5], p[0])) > one_bd) || (16'(adiff(q[5], q[0])) > one_bd) ||
                (16'(adiff(p[4], p[0])) > one_bd) || (16'(adiff(q[4], q[0])) > one_bd);
            flat2 = (filter_size >= 5'd16) && !m;
        end
    end
    // narrow filter (7.14.6.3)
    logic signed [13:0] ps1, ps0, qs0, qs1, filt, filter1, filter2, filt_r;
    logic [11:0] nq0, np0, nq1, np1;
    logic signed [13:0] off_s, lo_s, hi_s;
    function automatic logic signed [13:0] c4(input logic signed [15:0] v, input logic signed [13:0] lo, input logic signed [13:0] hi);
        c4 = (v < 16'(lo)) ? lo : (v > 16'(hi)) ? hi : 14'(v);
    endfunction
    always_comb begin
        off_s = 14'(13'd128 << bd8);
        lo_s = -(14'sd1 <<< (hdr.bit_depth - 4'd1));
        hi_s = (14'sd1 <<< (hdr.bit_depth - 4'd1)) - 14'sd1;
        ps1 = 14'(p[1]) - off_s; ps0 = 14'(p[0]) - off_s; qs0 = 14'(q[0]) - off_s; qs1 = 14'(q[1]) - off_s;
        filt = hev ? c4(16'(ps1) - 16'(qs1), lo_s, hi_s) : 14'sd0;
        filt = c4(16'(filt) + 16'sd3 * (16'(qs0) - 16'(ps0)), lo_s, hi_s);
        filter1 = c4(16'(filt) + 16'sd4, lo_s, hi_s) >>> 3;
        filter2 = c4(16'(filt) + 16'sd3, lo_s, hi_s) >>> 3;
        nq0 = 12'(c4(16'(qs0) - 16'(filter1), lo_s, hi_s) + off_s);
        np0 = 12'(c4(16'(ps0) + 16'(filter2), lo_s, hi_s) + off_s);
        filt_r = (filter1 + 14'sd1) >>> 1;                                   // Round2(filter1, 1)
        nq1 = 12'(c4(16'(qs1) - 16'(filt_r), lo_s, hi_s) + off_s);
        np1 = 12'(c4(16'(ps1) + 16'(filt_r), lo_s, hi_s) + off_s);
    end
    // wide filters (7.14.6.4): F[i] = Round2(sum_j pix[clip3(-(n+1), n, i+j)] * tap(j), log2Size), i in -n..n-1
    function automatic logic [11:0] wide_val(input int i, input int n, input int n2, input int log2sz);
        logic [17:0] t;
        int pidx;
        t = 18'd0;
        for (int j = -6; j <= 6; j++) begin
            if (j >= -n && j <= n) begin
                pidx = i + j;
                if (pidx < -(n + 1)) pidx = -(n + 1);
                if (pidx > n) pidx = n;
                t = t + (((j < 0 ? -j : j) <= n2) ? 18'(pix[7 + pidx]) * 18'd2 : 18'(pix[7 + pidx]));
            end
        end
        wide_val = 12'((t + (18'd1 << (log2sz - 1))) >> log2sz);
    endfunction
    logic [11:0] w16 [0:11];                            // luma 16: n=6, n2=1, log2 4: i = -6..5
    logic [11:0] w8  [0:5];                             // luma 8 : n=3, n2=0, log2 3: i = -3..2
    logic [11:0] w6  [0:3];                             // chroma : n=2, n2=1, log2 3: i = -2..1
    always_comb begin
        for (int i = -6; i <= 5; i++) w16[i + 6] = wide_val(i, 6, 1, 4);
        for (int i = -3; i <= 2; i++) w8[i + 3] = wide_val(i, 3, 0, 3);
        for (int i = -2; i <= 1; i++) w6[i + 2] = wide_val(i, 2, 1, 3);
    end
    // decision + output pixel for write position k
    typedef enum logic [1:0] {F_NONE, F_NARROW, F_WIDE8, F_WIDE16} fmode_t;
    fmode_t fmode;
    always_comb begin
        if (!filter_mask) fmode = F_NONE;
        else if (filter_size == 5'd4 || !flat) fmode = F_NARROW;
        else if (filter_size == 5'd8 || !flat2) fmode = F_WIDE8;
        else fmode = F_WIDE16;
    end
    logic [11:0] out_pix;
    logic        out_valid;                             // this k is modified
    always_comb begin
        out_pix = 12'd0; out_valid = 1'b0;
        case (fmode)
            F_NARROW: begin
                case (wk)
                    5'sd0:  begin out_pix = nq0; out_valid = 1'b1; end
                    -5'sd1: begin out_pix = np0; out_valid = 1'b1; end
                    5'sd1:  begin out_pix = nq1; out_valid = !hev; end
                    -5'sd2: begin out_pix = np1; out_valid = !hev; end
                    default: ;
                endcase
            end
            F_WIDE8: begin
                if (plane == 2'd0) begin out_valid = (wk >= -5'sd3) && (wk <= 5'sd2); out_pix = w8[int'(wk) + 3]; end
                else begin out_valid = (wk >= -5'sd2) && (wk <= 5'sd1); out_pix = w6[int'(wk) + 2]; end
            end
            F_WIDE16: begin out_valid = (wk >= -5'sd6) && (wk <= 5'sd5); out_pix = w16[int'(wk) + 6]; end
            default: ;
        endcase
    end

    // ---------------------------------------------------------------- memory / frame buffer addressing
    logic signed [4:0] rk;                              // pixel offset being read
    assign rk = 5'(signed'({1'b0, kk})) - 5'(signed'({1'b0, n_rd >> 1}));
    always_comb begin
        rd_row = (st == L_EDGE) ? rowa : prow;
        rd_col = (st == L_EDGE) ? cola : pcol;
        txr_plane = plane;
        txr_row = ((st == L_EDGE) ? rowa : prow) >> sub_y;
        txr_col = ((st == L_EDGE) ? cola : pcol) >> sub_x;
        fb_re = 1'b0; fb_we = 1'b0; fb_plane = plane; fb_x = '0; fb_y = '0; fb_wdata = out_pix;
        if (st == L_S_RD && kk < n_rd) begin
            fb_re = 1'b1;
            fb_x = FBX'(pss ? sx : sx + 13'(rk)); fb_y = FBY'(pss ? sy + 13'(rk) : sy);
        end
        if (st == L_S_WR && out_valid) begin
            fb_we = 1'b1;
            fb_x = FBX'(pss ? sx : sx + 13'(wk)); fb_y = FBY'(pss ? sy + 13'(wk) : sy);
        end
    end
    assign busy = (st != L_IDLE);

    // ---------------------------------------------------------------- FSM
    always_ff @(posedge clk) begin
        done <= 1'b0;
        if (rd_pend) pix[rd_slot] <= fb_rdata;         // read issued last cycle lands now
        rd_pend <= 1'b0;
        if (rst) st <= L_IDLE;
        else case (st)
            L_IDLE: if (start) begin
                plane <= 2'd0; pss <= 1'b0; row <= 11'd0; col <= 11'd0;
                st <= (lh.level[5:0] == 6'd0 && lh.level[11:6] == 6'd0) ? L_DONE : L_EDGE;
            end
            L_EDGE: begin
                if (x >= lh.frame_width || y >= lh.frame_height || (!pss && x == 13'd0) || (pss && y == 13'd0)) st <= L_NEXT;
                else st <= L_RD1;                        // cur read issued (addresses are combinational on the state)
            end
            L_RD1: begin cur <= rd_info; cur_tx <= txr_sz; st <= L_RD2; end     // prev read issued
            L_RD2: begin prv <= rd_info; prv_tx <= txr_sz; st <= L_CALC; end
            L_CALC: begin
                filter_size <= fsize_c;
                begin
                    logic [29:0] s;
                    s = (str_cur[29:24] != 6'd0) ? str_cur : str_prv;
                    lvl <= s[29:24]; limit <= s[23:16]; blimit <= s[15:8]; thresh <= s[7:0];
                    if (!apply_filter || s[29:24] == 6'd0) st <= L_NEXT;
                    else begin
                        i4 <= 2'd0; kk <= 4'd0;
                        n_rd <= (fsize_c == 5'd16) ? 4'd14 : (fsize_c == 5'd8) ? ((plane == 2'd0) ? 4'd8 : 4'd6) : 4'd4;
                        st <= L_S_RD;
                    end
                end
            end
            // ---- one sample: read n_rd pixels (k = -n_rd/2 .. n_rd/2-1), compute, write back the modified ones
            L_S_RD: begin
                if (kk < n_rd) begin
                    rd_pend <= 1'b1; rd_slot <= 4'(7 + int'(rk)); kk <= kk + 4'd1;
                end else if (!rd_pend) begin
                    wk <= -5'sd7; st <= L_S_CALC;
                end
            end
            L_S_CALC: st <= L_S_WR;                      // one cycle for pix[] to settle after the last landing read
            L_S_WR: begin
                if (wk == 5'sd6) begin
                    kk <= 4'd0;
                    if (i4 == 2'd3) st <= L_NEXT;
                    else begin i4 <= i4 + 2'd1; st <= L_S_RD; end
                end else wk <= wk + 5'sd1;
            end
            // ---- next edge position: col, row, pass, plane
            L_NEXT: begin
                if (col + (11'd1 << sub_x) < hdr.mi_cols) begin col <= col + (11'd1 << sub_x); st <= L_EDGE; end
                else begin
                    col <= 11'd0;
                    if (row + (11'd1 << sub_y) < hdr.mi_rows) begin row <= row + (11'd1 << sub_y); st <= L_EDGE; end
                    else begin
                        row <= 11'd0;
                        if (!pss) begin pss <= 1'b1; st <= L_EDGE; end
                        else begin
                            pss <= 1'b0;
                            // next plane with a non-zero level (plane 0 always runs when we got here)
                            if (plane == 2'd0 && !hdr.mono && lh.level[17:12] != 6'd0) begin plane <= 2'd1; st <= L_EDGE; end
                            else if (plane <= 2'd1 && !hdr.mono && lh.level[23:18] != 6'd0) begin plane <= 2'd2; st <= L_EDGE; end
                            else st <= L_DONE;
                        end
                    end
                end
            end
            L_DONE: begin done <= 1'b1; st <= L_IDLE; end
            default: st <= L_IDLE;
        endcase
    end
endmodule
