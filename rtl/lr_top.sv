// Loop restoration (spec 7.17), frame-level: for every 4x4 luma position and every plane, the unit's filter
// (Wiener 7.17.4/5 or self-guided 7.17.2/3) is applied to the (subsampled) block, reading a 10x10 window
// through the 7.17.6 source rule (rows outside the 64-row stripe, offset -8, come from the deblocked frame
// clamped to stripe +/- 2, everything else from CdefFrame), writing LrFrame. Blocks with RESTORE_NONE (frame
// or unit) are copied from CdefFrame. No super-resolution (UpscaledWidth == FrameWidth).
// Slow-and-correct: one 7-tap MAC per cycle for Wiener; one window row (<= 5 squares) per cycle for the
// self-guided box sums; ~180-330 cycles per 4x4 luma block.
module lr_top
  import syn_pkg::*;
  import lr_pkg::*;
#(
    parameter int FBX = 10,
    parameter int FBY = 9
) (
    input  logic        clk,
    input  logic        rst,
    input  hdr_t        hdr,
    input  logic        start,
    output logic        busy,
    output logic        done,
    // mi_store: unit records
    output logic [1:0]  lrr_plane,
    output logic [5:0]  lrr_row, lrr_col,
    input  lr_rec_t     lrr_rec,
    // deblocked frame (s0) and CdefFrame (s1) reads, LrFrame write
    output logic        s0_re, s1_re,
    output logic [1:0]  s_plane,
    output logic [FBX-1:0] s_x,
    output logic [FBY-1:0] s_y,
    input  logic [11:0] s0_rdata, s1_rdata,
    output logic        d_we,
    output logic [1:0]  d_plane,
    output logic [FBX-1:0] d_x,
    output logic [FBY-1:0] d_y,
    output logic [11:0] d_wdata
);
    localparam logic [1:0] RESTORE_NONE = 2'd0, RESTORE_WIENER = 2'd1, RESTORE_SGRPROJ = 2'd2;

    typedef enum logic [4:0] {
        L_IDLE, L_BLK, L_REC, L_DECIDE,
        L_WIN, L_WIEN_H, L_WIEN_V, L_WIEN_WR,
        L_AB, L_F, L_PASS_NEXT, L_BLEND,
        L_COPY_RD, L_COPY_WR, L_NEXT, L_DONE
    } st_t;
    st_t st;

    // ---------------------------------------------------------------- block loop (luma 4x4 positions, all planes)
    logic [12:0] ly, lx;                                // luma y / x (step 4)
    logic [1:0]  plane;
    logic        sub_x, sub_y;
    logic [3:0]  bd;
    logic signed [13:0] stripe_start, stripe_end;
    logic [12:0] plane_end_x, plane_end_y;
    logic [12:0] x, y;
    logic [2:0]  w, hh;
    logic [1:0]  ftype;                                 // frame restoration type of the plane
    logic [3:0]  unit_log2;                             // log2(unitSize)
    logic [12:0] unit_rows, unit_cols, ur, uc;
    lr_rec_t     rec;
    always_comb begin
        sub_x = (plane != 2'd0) && hdr.ssx; sub_y = (plane != 2'd0) && hdr.ssy;
        bd = hdr.bit_depth;
        ftype = hdr.lr_type[2 * plane +: 2];
        unit_log2 = 4'd6 + 4'(hdr.lr_size[2 * plane +: 2]);
        begin
            logic [12:0] stripe_num;
            stripe_num = (ly + 13'd8) >> 6;
            stripe_start = (14'(signed'({1'b0, stripe_num})) * 14'sd64 - 14'sd8) >>> sub_y;
            stripe_end = stripe_start + (14'sd64 >>> sub_y) - 14'sd1;
        end
        begin
            logic [12:0] fh_p, fw_p;
            fh_p = (hdr.frame_height + 13'(sub_y)) >> sub_y;        // Round2(FrameHeight, subY)
            fw_p = (hdr.upscaled_width + 13'(sub_x)) >> sub_x;
            unit_rows = (fh_p + (13'd1 << (unit_log2 - 4'd1))) >> unit_log2; if (unit_rows == 13'd0) unit_rows = 13'd1;
            unit_cols = (fw_p + (13'd1 << (unit_log2 - 4'd1))) >> unit_log2; if (unit_cols == 13'd0) unit_cols = 13'd1;
            plane_end_x = fw_p - 13'd1; plane_end_y = fh_p - 13'd1;
        end
        ur = ((ly + 13'd8) >> sub_y) >> unit_log2; if (ur > unit_rows - 13'd1) ur = unit_rows - 13'd1;
        uc = (lx >> sub_x) >> unit_log2;           if (uc > unit_cols - 13'd1) uc = unit_cols - 13'd1;
        x = lx >> sub_x; y = ly >> sub_y;
        begin
            logic [12:0] wmax, hmax;
            wmax = plane_end_x - x + 13'd1; hmax = plane_end_y - y + 13'd1;
            w = (13'(3'd4 >> sub_x) < wmax) ? 3'(3'd4 >> sub_x) : 3'(wmax);
            hh = (13'(3'd4 >> sub_y) < hmax) ? 3'(3'd4 >> sub_y) : 3'(hmax);
        end
        lrr_plane = plane; lrr_row = ur[5:0]; lrr_col = uc[5:0];
    end

    // ---------------------------------------------------------------- 7.17.6 source selection for a window read
    logic [3:0]  wr, wc;                                // window position 0..9 (origin x-3, y-3)
    logic signed [14:0] rx, ry, rxc, ryc;
    logic        use_s0;
    always_comb begin
        rx = 15'(signed'({2'b0, x})) - 15'sd3 + 15'(signed'({11'b0, wc}));
        ry = 15'(signed'({2'b0, y})) - 15'sd3 + 15'(signed'({11'b0, wr}));
        rxc = (rx < 0) ? 15'sd0 : (rx > 15'(signed'({2'b0, plane_end_x}))) ? 15'(signed'({2'b0, plane_end_x})) : rx;
        ryc = (ry < 0) ? 15'sd0 : (ry > 15'(signed'({2'b0, plane_end_y}))) ? 15'(signed'({2'b0, plane_end_y})) : ry;
        use_s0 = 1'b0;
        if (ryc < 15'(stripe_start)) begin use_s0 = 1'b1; if (ryc < 15'(stripe_start) - 15'sd2) ryc = 15'(stripe_start) - 15'sd2; end
        else if (ryc > 15'(stripe_end)) begin use_s0 = 1'b1; if (ryc > 15'(stripe_end) + 15'sd2) ryc = 15'(stripe_end) + 15'sd2; end
    end
    logic [11:0] win [0:9][0:9];
    logic        wpend; logic [3:0] wpr, wpc; logic wp_s0;

    // ---------------------------------------------------------------- Wiener
    logic signed [9:0] hf [0:6];
    logic signed [9:0] vf [0:6];
    function automatic logic signed [9:0] wcoef(input logic [20:0] c3, input logic [2:0] t);
        // wiener_coeffs: f[i] = f[6-i] = c[i] for i < 3, f[3] = 128 - 2 * (c0 + c1 + c2)
        logic signed [6:0] c0, c1, c2;
        c0 = c3[6:0]; c1 = c3[13:7]; c2 = c3[20:14];
        case (t)
            3'd0, 3'd6: wcoef = 10'(c0);
            3'd1, 3'd5: wcoef = 10'(c1);
            3'd2, 3'd4: wcoef = 10'(c2);
            default: wcoef = 10'sd128 - 10'sd2 * (10'(c0) + 10'(c1) + 10'(c2));
        endcase
    endfunction
    always_comb for (int t = 0; t < 7; t++) begin
        vf[t] = wcoef(rec.wiener[0 +: 21], 3'(t));       // pass 0 = vertical
        hf[t] = wcoef(rec.wiener[21 +: 21], 3'(t));      // pass 1 = horizontal
    end
    logic [3:0]  round0, round1;
    logic signed [20:0] w_offset, w_limit;
    always_comb begin
        round0 = (bd == 4'd12) ? 4'd5 : 4'd3;
        round1 = (bd == 4'd12) ? 4'd9 : 4'd11;
        w_offset = 21'sd1 <<< (bd + 4'd7 - round0 - 4'd1);
        w_limit = (21'sd1 <<< (bd + 4'd8 - round0)) - 21'sd1;
    end
    logic signed [17:0] inter [0:9][0:3];
    logic [3:0]  ir; logic [1:0] ic;                    // inter row 0..hh+5 / col
    logic signed [30:0] mac;
    always_comb begin
        mac = 31'sd0;
        for (int t = 0; t < 7; t++)
            if (st == L_WIEN_H) mac = mac + 31'(hf[t]) * 31'(signed'({19'b0, win[ir][4'(ic) + 4'(t)]}));
            else mac = mac + 31'(vf[t]) * 31'(inter[ir + 4'(t)][ic]);
    end
    logic signed [20:0] inter_v;
    logic [11:0] wien_out;
    always_comb begin
        begin
            logic signed [30:0] v;
            v = (mac + (31'sd1 <<< (round0 - 4'd1))) >>> round0;
            inter_v = (v < -31'(w_offset)) ? -w_offset : (v > 31'(w_limit) - 31'(w_offset)) ? 21'(w_limit - w_offset) : 21'(v);
        end
        begin
            logic signed [30:0] v;
            v = (mac + (31'sd1 <<< (round1 - 4'd1))) >>> round1;
            wien_out = (v < 0) ? 12'd0 : (v > 31'(signed'({19'b0, 12'((13'd1 << bd) - 13'd1)}))) ? 12'((13'd1 << bd) - 13'd1) : 12'(v);
        end
    end

    // ---------------------------------------------------------------- self-guided
    logic        pss;
    logic [1:0]  r_p; logic [6:0] e_p;
    logic [15:0] sp;
    logic [11:0] s_val;
    logic [8:0]  one_over_n;
    logic [5:0]  n_p;
    always_comb begin
        sp = sgr_params(rec.sgr_set);
        r_p = pss ? sp[6:5] : sp[15:14];
        e_p = pss ? 7'(sp[4:0]) : sp[13:7];
        s_val = sgr_s(rec.sgr_set, pss);
        one_over_n = sgr_one_over_n(r_p);
        n_p = (r_p == 2'd1) ? 6'd9 : 6'd25;
    end
    // box sums: position (ai, aj) in -1..hh / -1..w (stored at +1), one window row per cycle (row counter ab_k)
    logic [2:0]  ai, aj, ab_k;                          // ai/aj = position + 1 (0..5); ab_k = row within the box (0..2r)
    logic [29:0] acc_a; logic [17:0] acc_b;
    logic [29:0] row_a; logic [17:0] row_b;
    always_comb begin
        row_a = 30'd0; row_b = 18'd0;
        for (int dx = -2; dx <= 2; dx++) begin
            logic [11:0] c;
            // window row = 3 + (ai-1) + (ab_k - r) ; col = 3 + (aj-1) + dx
            c = win[4'(int'(3) + int'(ai) - 1 + int'(ab_k) - int'(r_p))][4'(int'(3) + int'(aj) - 1 + dx)];
            if (dx >= -int'(r_p) && dx <= int'(r_p)) begin
                row_a = row_a + 30'(c) * 30'(c);
                row_b = row_b + 18'(c);
            end
        end
    end
    logic [8:0]  A [0:5][0:5];
    logic [21:0] B [0:5][0:5];
    logic [8:0]  a2_v; logic [21:0] b2_v;
    always_comb begin
        logic [29:0] a_r; logic [17:0] d_r; logic [29:0] an; logic [35:0] dd; logic [36:0] p; logic [48:0] pz; logic [28:0] z;
        logic [8:0] a2; logic [42:0] b2;
        a_r = (acc_a + (30'd1 << (2 * (bd - 4'd8))) / 2) >> (2 * (bd - 4'd8));
        if (bd == 4'd8) a_r = acc_a;
        d_r = (bd == 4'd8) ? acc_b : ((acc_b + (18'd1 << (bd - 4'd9))) >> (bd - 4'd8));
        an = a_r * 30'(n_p);
        dd = 36'(d_r) * 36'(d_r);
        p = (36'(an) > dd) ? 37'(36'(an) - dd) : 37'd0;
        pz = 49'(p) * 49'(s_val);
        z = 29'((pz + (49'd1 << 19)) >> 20);
        if (z >= 29'd255) a2 = 9'd256;
        else if (z == 29'd0) a2 = 9'd1;
        else a2 = sgr_a2(8'(z));
        b2 = 43'(9'd256 - a2) * 43'(acc_b) * 43'(one_over_n);
        a2_v = a2;
        b2_v = 22'((b2 + (43'd1 << 11)) >> 12);
    end
    // F for pixel (fi, fj): 3x3 weighted A/B
    logic [2:0]  fi, fj;
    logic [23:0] flt [0:1][0:3][0:3];
    logic [23:0] f_val;
    always_comb begin
        logic [15:0] fa; logic [28:0] fb; logic [3:0] wgt; logic [3:0] shift; logic [40:0] v;
        fa = 16'd0; fb = 29'd0;
        for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++) begin
                if (!pss) wgt = (((int'(fi) + dy) & 1) != 0) ? ((dx == 0) ? 4'd6 : 4'd5) : 4'd0;
                else wgt = (dx == 0 || dy == 0) ? 4'd4 : 4'd3;
                fa = fa + 16'(wgt) * 16'(A[3'(int'(fi) + 1 + dy)][3'(int'(fj) + 1 + dx)]);
                fb = fb + 29'(wgt) * 29'(B[3'(int'(fi) + 1 + dy)][3'(int'(fj) + 1 + dx)]);
            end
        shift = (!pss && fi[0]) ? 4'd4 : 4'd5;
        v = 41'(fa) * 41'(win[4'd3 + 4'(fi)][4'd3 + 4'(fj)]) + 41'(fb);
        f_val = 24'((v + (41'd1 << (4'd8 + shift - 4'd4 - 4'd1))) >> (4'd8 + shift - 4'd4));
    end
    logic [11:0] sgr_out;
    always_comb begin
        logic signed [8:0] w0, w1; logic signed [9:0] w2; logic signed [35:0] v; logic [16:0] u; logic signed [35:0] sv;
        w0 = 9'(signed'(rec.xqd[7:0])); w1 = 9'(signed'(rec.xqd[15:8])); w2 = 10'sd128 - 10'(w0) - 10'(w1);
        u = 17'(win[4'd3 + 4'(fi)][4'd3 + 4'(fj)]) << 4;
        v = 36'(w1) * 36'(signed'({19'b0, u}));
        v = v + 36'(w0) * ((sp[15:14] != 2'd0) ? 36'(signed'({12'b0, flt[0][fi[1:0]][fj[1:0]]})) : 36'(signed'({19'b0, u})));
        v = v + 36'(w2) * ((sp[6:5] != 2'd0) ? 36'(signed'({12'b0, flt[1][fi[1:0]][fj[1:0]]})) : 36'(signed'({19'b0, u})));
        sv = (v + 36'sd1024) >>> 11;
        sgr_out = (sv < 0) ? 12'd0 : (sv > 36'(signed'({24'b0, 12'((13'd1 << bd) - 13'd1)}))) ? 12'((13'd1 << bd) - 13'd1) : 12'(sv);
    end

    // ---------------------------------------------------------------- copy path
    logic cp_pend;

    // ---------------------------------------------------------------- memory ports
    always_comb begin
        s0_re = 1'b0; s1_re = 1'b0; s_plane = plane; s_x = '0; s_y = '0;
        d_we = 1'b0; d_plane = plane; d_x = '0; d_y = '0; d_wdata = 12'd0;
        case (st)
            L_WIN: if (wr < 4'd10) begin
                s0_re = use_s0; s1_re = !use_s0; s_x = FBX'(rxc); s_y = FBY'(ryc);
            end
            L_WIEN_WR: begin d_we = 1'b1; d_x = FBX'(x + 13'(fj)); d_y = FBY'(y + 13'(fi)); d_wdata = wien_out; end
            L_BLEND: begin d_we = 1'b1; d_x = FBX'(x + 13'(fj)); d_y = FBY'(y + 13'(fi)); d_wdata = sgr_out; end
            L_COPY_RD: begin s1_re = 1'b1; s_x = FBX'(x + 13'(fj)); s_y = FBY'(y + 13'(fi)); end
            L_COPY_WR: begin d_we = 1'b1; d_x = FBX'(x + 13'(fj)); d_y = FBY'(y + 13'(fi)); d_wdata = s1_rdata; end
            default: ;
        endcase
    end
    assign busy = (st != L_IDLE);

    // ---------------------------------------------------------------- FSM
    always_ff @(posedge clk) begin
        done <= 1'b0;
        if (wpend) win[wpr][wpc] <= wp_s0 ? s0_rdata : s1_rdata;
        wpend <= 1'b0;
        if (rst) st <= L_IDLE;
        else case (st)
            L_IDLE: if (start) begin ly <= 13'd0; lx <= 13'd0; plane <= 2'd0; st <= L_BLK; end
            L_BLK: st <= L_REC;                          // unit record read issued (lrr_* combinational)
            L_REC: begin rec <= lrr_rec; st <= L_DECIDE; end
            L_DECIDE: begin
                fi <= 3'd0; fj <= 3'd0; wr <= 4'd0; wc <= 4'd0; pss <= 1'b0;
                if (w == 3'd0 || hh == 3'd0) st <= L_NEXT;
                else if (ftype == RESTORE_NONE || rec.lr_type == RESTORE_NONE) st <= L_COPY_RD;
                else st <= L_WIN;
            end
            // ---- 10x10 window through the source rule (one read per cycle, lands next cycle)
            L_WIN: begin
                if (wr < 4'd10) begin
                    wpend <= 1'b1; wpr <= wr; wpc <= wc; wp_s0 <= use_s0;
                    if (wc == 4'd9) begin wc <= 4'd0; wr <= wr + 4'd1; end else wc <= wc + 4'd1;
                end else if (!wpend) begin
                    wr <= 4'd0; wc <= 4'd0;
                    if (rec.lr_type == RESTORE_WIENER) begin ir <= 4'd0; ic <= 2'd0; st <= L_WIEN_H; end
                    else begin
                        // self-guided: pass 0 unless r0 == 0, else pass 1 unless r1 == 0
                        if (sp[15:14] != 2'd0) begin pss <= 1'b0; ai <= 3'd0; aj <= 3'd0; ab_k <= 3'd0; acc_a <= 30'd0; acc_b <= 18'd0; st <= L_AB; end
                        else if (sp[6:5] != 2'd0) begin pss <= 1'b1; ai <= 3'd0; aj <= 3'd0; ab_k <= 3'd0; acc_a <= 30'd0; acc_b <= 18'd0; st <= L_AB; end
                        else begin fi <= 3'd0; fj <= 3'd0; st <= L_BLEND; end
                    end
                end
            end
            // ---- Wiener: horizontal pass into inter (hh+6 rows x w cols), then vertical pass with writes
            L_WIEN_H: begin
                inter[ir][ic] <= 18'(inter_v);
                if (3'(ic) + 3'd1 < w) ic <= ic + 2'd1;
                else begin
                    ic <= 2'd0;
                    if (ir + 4'd1 < 4'(hh) + 4'd6) ir <= ir + 4'd1;
                    else begin ir <= 4'd0; st <= L_WIEN_V; end
                end
            end
            L_WIEN_V: begin fi <= 3'(ir); fj <= 3'(ic); st <= L_WIEN_WR; end          // mac (vertical) valid: write next cycle
            L_WIEN_WR: begin
                if (3'(ic) + 3'd1 < w) begin ic <= ic + 2'd1; st <= L_WIEN_V; end
                else begin
                    ic <= 2'd0;
                    if (ir + 4'd1 < 4'(hh)) begin ir <= ir + 4'd1; st <= L_WIEN_V; end
                    else st <= L_NEXT;
                end
            end
            // ---- self-guided: A/B over (hh+2) x (w+2) positions, one window row of the box per cycle
            L_AB: begin
                if (ab_k == 3'd7) begin
                    // box sums complete: a2 / b2 for this position, advance
                    A[ai][aj] <= a2_v; B[ai][aj] <= b2_v;
                    acc_a <= 30'd0; acc_b <= 18'd0; ab_k <= 3'd0;
                    if (aj + 3'd1 < 3'(w) + 3'd2) aj <= aj + 3'd1;
                    else begin
                        aj <= 3'd0;
                        if (ai + 3'd1 < 3'(hh) + 3'd2) ai <= ai + 3'd1;
                        else begin ai <= 3'd0; fi <= 3'd0; fj <= 3'd0; st <= L_F; end
                    end
                end else begin
                    // fold one row of the (2r+1)^2 box per cycle
                    acc_a <= acc_a + row_a; acc_b <= acc_b + row_b;
                    ab_k <= (ab_k == 3'(2 * r_p)) ? 3'd7 : ab_k + 3'd1;
                end
            end
            L_F: begin
                flt[pss][fi[1:0]][fj[1:0]] <= f_val;
                if (fj + 3'd1 < 3'(w)) fj <= fj + 3'd1;
                else begin
                    fj <= 3'd0;
                    if (fi + 3'd1 < 3'(hh)) fi <= fi + 3'd1;
                    else begin fi <= 3'd0; st <= L_PASS_NEXT; end
                end
            end
            L_PASS_NEXT: begin
                if (!pss && sp[6:5] != 2'd0) begin pss <= 1'b1; ai <= 3'd0; aj <= 3'd0; ab_k <= 3'd0; acc_a <= 30'd0; acc_b <= 18'd0; st <= L_AB; end
                else begin fi <= 3'd0; fj <= 3'd0; st <= L_BLEND; end
            end
            L_BLEND: begin
                if (fj + 3'd1 < 3'(w)) fj <= fj + 3'd1;
                else begin
                    fj <= 3'd0;
                    if (fi + 3'd1 < 3'(hh)) fi <= fi + 3'd1;
                    else st <= L_NEXT;
                end
            end
            // ---- copy from CdefFrame
            L_COPY_RD: st <= L_COPY_WR;
            L_COPY_WR: begin
                if (fj + 3'd1 < 3'(w)) begin fj <= fj + 3'd1; st <= L_COPY_RD; end
                else begin
                    fj <= 3'd0;
                    if (fi + 3'd1 < 3'(hh)) begin fi <= fi + 3'd1; st <= L_COPY_RD; end
                    else st <= L_NEXT;
                end
            end
            // ---- next: plane, then x, then y (luma 4x4 raster)
            L_NEXT: begin
                if (plane + 2'd1 < (hdr.mono ? 2'd1 : 2'd3)) begin plane <= plane + 2'd1; st <= L_BLK; end
                else begin
                    plane <= 2'd0;
                    if (lx + 13'd4 < hdr.upscaled_width) begin lx <= lx + 13'd4; st <= L_BLK; end
                    else begin
                        lx <= 13'd0;
                        if (ly + 13'd4 < hdr.frame_height) begin ly <= ly + 13'd4; st <= L_BLK; end
                        else st <= L_DONE;
                    end
                end
            end
            L_DONE: begin done <= 1'b1; st <= L_IDLE; end
            default: st <= L_IDLE;
        endcase
    end
endmodule
