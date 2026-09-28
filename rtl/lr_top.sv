// Loop restoration (spec 7.17), frame-level, row-streaming version.
// The spec loops over 4x4 luma blocks; the results of a block depend only on its unit record, its 64-row
// stripe (7.17.6 source rule) and the source pixels around it. This engine processes one band of luma rows
// (4 rows, 4 >> subY in a subsampled plane) per plane at a time: the band's window rows (band rows plus 3
// above and 3 below, through the source rule) are kept in a ring of 16 line buffers tagged with (source, row)
// so a row is read from the frame buffers only when it is not already present; then every unit column
// segment of the band is filtered from the line buffers with one output sample per clock (Wiener: seven
// horizontal 7-tap filters and the vertical 7-tap filter in one cycle; copy: one write per clock) or, for
// the self-guided filter, one box-sum position (a2 / b2) per clock per pass followed by one output per clock.
// Same ports as the block-at-a-time version it replaces; the Python model (tb/lr_model.py) is the oracle.
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
    input  logic [12:0] ly_first, ly_last,    // luma rows to restore (inclusive; multiples of 4): a stripe, or the whole frame
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
    localparam int RING = 16;
    localparam int W = 1 << FBX;

    typedef enum logic [4:0] {
        L_IDLE, L_BAND, L_ROWS, L_LOAD, L_LOAD_LAST, L_UNIT, L_REC, L_DECIDE,
        L_COPY, L_WIEN, L_AB, L_PASS_NEXT, L_BLEND, L_UNIT_NEXT, L_BAND_NEXT, L_DONE
    } st_t;
    st_t st;

    // ---------------------------------------------------------------- band geometry (luma row ly, plane)
    logic [12:0] ly;
    logic [1:0]  plane;
    logic        sub_x, sub_y;
    logic [3:0]  bd;
    logic signed [13:0] stripe_start, stripe_end;
    logic [12:0] plane_end_x, plane_end_y, y0;
    logic [2:0]  hh;                                    // rows in the band (1..4)
    logic [1:0]  ftype;
    logic [3:0]  unit_log2;
    logic [12:0] unit_rows, unit_cols, ur;
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
            fh_p = (hdr.frame_height + 13'(sub_y)) >> sub_y;
            fw_p = (hdr.upscaled_width + 13'(sub_x)) >> sub_x;
            unit_rows = (fh_p + (13'd1 << (unit_log2 - 4'd1))) >> unit_log2; if (unit_rows == 13'd0) unit_rows = 13'd1;
            unit_cols = (fw_p + (13'd1 << (unit_log2 - 4'd1))) >> unit_log2; if (unit_cols == 13'd0) unit_cols = 13'd1;
            plane_end_x = fw_p - 13'd1; plane_end_y = fh_p - 13'd1;
        end
        ur = ((ly + 13'd8) >> sub_y) >> unit_log2; if (ur > unit_rows - 13'd1) ur = unit_rows - 13'd1;
        y0 = ly >> sub_y;
        begin
            logic [12:0] hmax;
            hmax = plane_end_y - y0 + 13'd1;
            hh = (13'(3'd4 >> sub_y) < hmax) ? 3'(3'd4 >> sub_y) : 3'(hmax);
        end
    end

    // ---------------------------------------------------------------- window rows through the source rule
    // window row k (0 .. hh+5) is plane row y0 - 3 + k before the rule; after it: (source, row)
    function automatic logic [14:0] row_rule(input logic [3:0] k);         // {src, row[13:0]}
        logic signed [14:0] ry;
        logic src;
        ry = 15'(signed'({2'b0, y0})) - 15'sd3 + 15'(signed'({11'b0, k}));
        if (ry < 0) ry = 15'sd0;
        if (ry > 15'(signed'({2'b0, plane_end_y}))) ry = 15'(signed'({2'b0, plane_end_y}));
        src = 1'b0;
        if (ry < 15'(stripe_start)) begin src = 1'b1; if (ry < 15'(stripe_start) - 15'sd2) ry = 15'(stripe_start) - 15'sd2; end
        else if (ry > 15'(stripe_end)) begin src = 1'b1; if (ry > 15'(stripe_end) + 15'sd2) ry = 15'(stripe_end) + 15'sd2; end
        row_rule = {src, 14'(ry)};
    endfunction
    logic [11:0] ring [0:RING-1][0:W-1];
    logic [13:0] ring_row [0:RING-1];
    logic        ring_src [0:RING-1];
    logic        ring_ok  [0:RING-1];
    logic [3:0]  k;                                     // window row being checked / loaded
    logic [14:0] k_rule;
    logic [3:0]  k_slot;
    logic        k_hit;
    assign k_rule = row_rule(k);
    assign k_slot = k_rule[3:0];
    assign k_hit = ring_ok[k_slot] && (ring_row[k_slot] == k_rule[13:0]) && (ring_src[k_slot] == k_rule[14]);
    // slot of window row k for the compute phases (all rows present)
    function automatic logic [3:0] slot_of(input logic [3:0] kk);
        logic [14:0] r;
        r = row_rule(kk);
        slot_of = r[3:0];
    endfunction
    function automatic logic [12:0] clampx(input logic signed [14:0] xx);
        clampx = (xx < 0) ? 13'd0 : (xx > 15'(signed'({2'b0, plane_end_x}))) ? plane_end_x : 13'(xx);
    endfunction
    logic [12:0] lx;                                    // load column

    // ---------------------------------------------------------------- unit segment
    logic [12:0] uc, seg_x0, seg_x1, x;
    logic [2:0]  i;                                     // output row within the band
    lr_rec_t     rec;
    always_comb begin
        seg_x0 = uc << unit_log2;
        seg_x1 = (uc + 13'd1 >= unit_cols) ? plane_end_x : 13'(((uc + 13'd1) << unit_log2) - 13'd1);
        lrr_plane = plane; lrr_row = ur[5:0]; lrr_col = uc[5:0];
    end

    // ---------------------------------------------------------------- Wiener: 7 horizontal filters + vertical, one sample per clock
    logic signed [9:0] hf [0:6];
    logic signed [9:0] vf [0:6];
    function automatic logic signed [9:0] wcoef(input logic [20:0] c3, input logic [2:0] t);
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
        vf[t] = wcoef(rec.wiener[0 +: 21], 3'(t));
        hf[t] = wcoef(rec.wiener[21 +: 21], 3'(t));
    end
    logic [3:0]  round0, round1;
    logic signed [20:0] w_offset, w_limit;
    always_comb begin
        round0 = (bd == 4'd12) ? 4'd5 : 4'd3;
        round1 = (bd == 4'd12) ? 4'd9 : 4'd11;
        w_offset = 21'sd1 <<< (bd + 4'd7 - round0 - 4'd1);
        w_limit = (21'sd1 <<< (bd + 4'd8 - round0)) - 21'sd1;
    end
    logic [11:0] wien_out;
    always_comb begin
        logic signed [30:0] vmac;
        vmac = 31'sd0;
        for (int r = 0; r < 7; r++) begin
            logic signed [30:0] hmac, v;
            logic signed [20:0] hv;
            logic [3:0] sl;
            hmac = 31'sd0;
            sl = slot_of(4'(int'(i) + r));
            for (int t = 0; t < 7; t++)
                hmac = hmac + 31'(hf[t]) * 31'(signed'({19'b0, ring[sl][FBX'(clampx(15'(signed'({2'b0, x})) - 15'sd3 + 15'(t)))]}));
            v = (hmac + (31'sd1 <<< (round0 - 4'd1))) >>> round0;
            hv = (v < -31'(w_offset)) ? -w_offset : (v > 31'(w_limit) - 31'(w_offset)) ? 21'(w_limit - w_offset) : 21'(v);
            vmac = vmac + 31'(vf[r]) * 31'(hv);
        end
        begin
            logic signed [30:0] v;
            v = (vmac + (31'sd1 <<< (round1 - 4'd1))) >>> round1;
            wien_out = (v < 0) ? 12'd0 : (v > 31'(signed'({19'b0, 12'((13'd1 << bd) - 13'd1)}))) ? 12'((13'd1 << bd) - 13'd1) : 12'(v);
        end
    end

    // ---------------------------------------------------------------- self-guided: a2 / b2 per box position, then F + blend
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
    // box position: A/B row ai (0 .. hh+1 = block row -1 .. hh), column ax (seg_x0-1 .. seg_x1+1)
    logic [2:0]  ai;
    logic signed [14:0] ax;
    logic [29:0] box_a; logic [17:0] box_b;
    always_comb begin
        box_a = 30'd0; box_b = 18'd0;
        for (int dy = -2; dy <= 2; dy++)
            for (int dx = -2; dx <= 2; dx++) begin
                logic [11:0] c;
                // block row (ai - 1) + dy -> window row k = (ai - 1) + dy + 3
                c = ring[slot_of(4'(int'(ai) + 2 + dy))][FBX'(clampx(ax + 15'(dx)))];
                if (dy >= -int'(r_p) && dy <= int'(r_p) && dx >= -int'(r_p) && dx <= int'(r_p)) begin
                    box_a = box_a + 30'(c) * 30'(c);
                    box_b = box_b + 18'(c);
                end
            end
    end
    logic [8:0]  a2_v; logic [21:0] b2_v;
    always_comb begin
        logic [29:0] a_r; logic [17:0] d_r; logic [29:0] an; logic [35:0] dd; logic [36:0] p; logic [48:0] pz; logic [28:0] z;
        logic [8:0] a2; logic [42:0] b2;
        a_r = (box_a + (30'd1 << (2 * (bd - 4'd8))) / 2) >> (2 * (bd - 4'd8));
        if (bd == 4'd8) a_r = box_a;
        d_r = (bd == 4'd8) ? box_b : ((box_b + (18'd1 << (bd - 4'd9))) >> (bd - 4'd8));
        an = a_r * 30'(n_p);
        dd = 36'(d_r) * 36'(d_r);
        p = (36'(an) > dd) ? 37'(36'(an) - dd) : 37'd0;
        pz = 49'(p) * 49'(s_val);
        z = 29'((pz + (49'd1 << 19)) >> 20);
        if (z >= 29'd255) a2 = 9'd256;
        else if (z == 29'd0) a2 = 9'd1;
        else a2 = sgr_a2(8'(z));
        b2 = 43'(9'd256 - a2) * 43'(box_b) * 43'(one_over_n);
        a2_v = a2;
        b2_v = 22'((b2 + (43'd1 << 11)) >> 12);
    end
    // A/B of both passes over the band: rows 0..5 (block row + 1), columns indexed by plane x + 1
    logic [8:0]  A [0:1][0:5][0:W+1];
    logic [21:0] B [0:1][0:5][0:W+1];
    // F for output (i, x) of pass p, then the blend
    function automatic logic [23:0] f_of(input logic p, input logic [2:0] fi, input logic [12:0] fx, input logic [11:0] pix);
        logic [15:0] fa; logic [28:0] fb; logic [3:0] wgt; logic [3:0] shift; logic [40:0] v;
        fa = 16'd0; fb = 29'd0;
        for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++) begin
                if (!p) wgt = (((int'(fi) + dy) & 1) != 0) ? ((dx == 0) ? 4'd6 : 4'd5) : 4'd0;
                else wgt = (dx == 0 || dy == 0) ? 4'd4 : 4'd3;
                fa = fa + 16'(wgt) * 16'(A[p][3'(int'(fi) + 1 + dy)][int'(fx) + 1 + dx]);
                fb = fb + 29'(wgt) * 29'(B[p][3'(int'(fi) + 1 + dy)][int'(fx) + 1 + dx]);
            end
        shift = (!p && fi[0]) ? 4'd4 : 4'd5;
        v = 41'(fa) * 41'(pix) + 41'(fb);
        f_of = 24'((v + (41'd1 << (4'd8 + shift - 4'd4 - 4'd1))) >> (4'd8 + shift - 4'd4));
    endfunction
    logic [11:0] cur_pix;                               // source sample of output (i, x): window row i + 3
    assign cur_pix = ring[slot_of(4'(int'(i) + 3))][x[FBX-1:0]];
    logic [11:0] sgr_out;
    always_comb begin
        logic signed [8:0] w0, w1; logic signed [9:0] w2; logic signed [35:0] v; logic [16:0] u; logic signed [35:0] sv;
        w0 = 9'(signed'(rec.xqd[7:0])); w1 = 9'(signed'(rec.xqd[15:8])); w2 = 10'sd128 - 10'(w0) - 10'(w1);
        u = 17'(cur_pix) << 4;
        v = 36'(w1) * 36'(signed'({19'b0, u}));
        v = v + 36'(w0) * ((sp[15:14] != 2'd0) ? 36'(signed'({12'b0, f_of(1'b0, i, x, cur_pix)})) : 36'(signed'({19'b0, u})));
        v = v + 36'(w2) * ((sp[6:5] != 2'd0) ? 36'(signed'({12'b0, f_of(1'b1, i, x, cur_pix)})) : 36'(signed'({19'b0, u})));
        sv = (v + 36'sd1024) >>> 11;
        sgr_out = (sv < 0) ? 12'd0 : (sv > 36'(signed'({24'b0, 12'((13'd1 << bd) - 13'd1)}))) ? 12'((13'd1 << bd) - 13'd1) : 12'(sv);
    end

    // ---------------------------------------------------------------- memory ports
    logic ld_pend;                                      // a load read is in flight (lands this cycle)
    always_comb begin
        s0_re = 1'b0; s1_re = 1'b0; s_plane = plane; s_x = FBX'(lx); s_y = FBY'(k_rule[13:0]);
        d_we = 1'b0; d_plane = plane; d_x = FBX'(x); d_y = FBY'(y0 + 13'(i)); d_wdata = 12'd0;
        case (st)
            L_LOAD: begin s0_re = k_rule[14]; s1_re = !k_rule[14]; end
            L_COPY: begin d_we = 1'b1; d_wdata = cur_pix; end
            L_WIEN: begin d_we = 1'b1; d_wdata = wien_out; end
            L_BLEND: begin d_we = 1'b1; d_wdata = sgr_out; end
            default: ;
        endcase
    end
    assign busy = (st != L_IDLE);

    // ---------------------------------------------------------------- FSM
    always_ff @(posedge clk) begin
        done <= 1'b0;
        if (ld_pend) ring[k_slot][lx[FBX-1:0] - FBX'(1)] <= k_rule[14] ? s0_rdata : s1_rdata;
        ld_pend <= 1'b0;
        if (rst) begin
            st <= L_IDLE;
            for (int s = 0; s < RING; s++) ring_ok[s] <= 1'b0;
        end else case (st)
            L_IDLE: if (start) begin
                plane <= 2'd0; ly <= ly_first;
                for (int s = 0; s < RING; s++) ring_ok[s] <= 1'b0;           // a new start: nothing cached (tags are rows, not planes)
                st <= L_BAND;
            end
            // ---- a band: make every window row present in the ring
            L_BAND: begin k <= 4'd0; st <= L_ROWS; end
            L_ROWS: begin
                if (k > 4'(hh) + 4'd5) begin uc <= 13'd0; st <= L_UNIT; end
                else if (k_hit) k <= k + 4'd1;
                else begin lx <= 13'd0; st <= L_LOAD; end
            end
            L_LOAD: begin                                                     // one read per clock, lands next cycle
                ld_pend <= (lx < plane_end_x);                                // (the last sample is stored by L_LOAD_LAST)
                if (lx < plane_end_x) lx <= lx + 13'd1;
                else st <= L_LOAD_LAST;
            end
            L_LOAD_LAST: begin
                ring[k_slot][plane_end_x[FBX-1:0]] <= k_rule[14] ? s0_rdata : s1_rdata;
                ring_row[k_slot] <= k_rule[13:0]; ring_src[k_slot] <= k_rule[14]; ring_ok[k_slot] <= 1'b1;
                k <= k + 4'd1; st <= L_ROWS;
            end
            // ---- unit column segment
            L_UNIT: st <= L_REC;                                              // record read issued (lrr_* combinational)
            L_REC: begin rec <= lrr_rec; st <= L_DECIDE; end
            L_DECIDE: begin
                i <= 3'd0; x <= seg_x0; pss <= 1'b0;
                if (hh == 3'd0) st <= L_UNIT_NEXT;
                else if (ftype == RESTORE_NONE || lrr_rec.lr_type == RESTORE_NONE) st <= L_COPY;
                else if (lrr_rec.lr_type == RESTORE_WIENER) st <= L_WIEN;
                else begin
                    if (sp[15:14] != 2'd0) begin pss <= 1'b0; ai <= 3'd0; ax <= 15'(signed'({2'b0, seg_x0})) - 15'sd1; st <= L_AB; end
                    else if (sp[6:5] != 2'd0) begin pss <= 1'b1; ai <= 3'd0; ax <= 15'(signed'({2'b0, seg_x0})) - 15'sd1; st <= L_AB; end
                    else st <= L_BLEND;
                end
            end
            L_COPY, L_WIEN, L_BLEND: begin                                    // one output sample per clock
                if (x < seg_x1) x <= x + 13'd1;
                else begin
                    x <= seg_x0;
                    if (i + 3'd1 < hh) i <= i + 3'd1;
                    else st <= L_UNIT_NEXT;
                end
            end
            L_AB: begin                                                       // one box position per clock
                A[pss][ai][(FBX + 1)'(ax + 15'sd1)] <= a2_v; B[pss][ai][(FBX + 1)'(ax + 15'sd1)] <= b2_v;
                if (ax < 15'(signed'({2'b0, seg_x1})) + 15'sd1) ax <= ax + 15'sd1;
                else begin
                    ax <= 15'(signed'({2'b0, seg_x0})) - 15'sd1;
                    if (ai + 3'd1 < 3'(hh) + 3'd2) ai <= ai + 3'd1;
                    else st <= L_PASS_NEXT;
                end
            end
            L_PASS_NEXT: begin
                if (!pss && sp[6:5] != 2'd0) begin pss <= 1'b1; ai <= 3'd0; ax <= 15'(signed'({2'b0, seg_x0})) - 15'sd1; st <= L_AB; end
                else begin i <= 3'd0; x <= seg_x0; st <= L_BLEND; end
            end
            L_UNIT_NEXT: begin
                if (uc + 13'd1 < unit_cols) begin uc <= uc + 13'd1; st <= L_UNIT; end
                else st <= L_BAND_NEXT;
            end
            // ---- next band (luma rows +4), then next plane
            L_BAND_NEXT: begin
                if (ly + 13'd4 <= ly_last && ly + 13'd4 < hdr.frame_height) begin ly <= ly + 13'd4; st <= L_BAND; end
                else if (plane + 2'd1 < (hdr.mono ? 2'd1 : 2'd3)) begin
                    plane <= plane + 2'd1; ly <= ly_first;
                    for (int s = 0; s < RING; s++) ring_ok[s] <= 1'b0;       // rows of another plane
                    st <= L_BAND;
                end else st <= L_DONE;
            end
            L_DONE: begin done <= 1'b1; st <= L_IDLE; end
            default: st <= L_IDLE;
        endcase
    end
endmodule
