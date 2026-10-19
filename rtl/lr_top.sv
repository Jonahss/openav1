// Loop restoration (spec 7.17), frame-level, row-streaming version.
// The spec loops over 4x4 luma blocks; the results of a block depend only on its unit record, its 64-row
// stripe (7.17.6 source rule) and the source pixels around it. This engine processes one band of luma rows
// (4 rows, 4 >> subY in a subsampled plane) per plane at a time: the band's window rows (band rows plus 3
// above and 3 below, through the source rule) are kept in a ring of 16 line buffers tagged with (source, row)
// so a row is read from the frame buffers only when it is not already present; then every unit column
// segment of the band is filtered with four output samples per clock (Wiener: 4 x 7 horizontal 7-tap
// filters and the vertical 7-tap filter in one cycle; copy: one 4-wide write per clock) or, for the
// self-guided filter, one box-sum position (a2 / b2) per clock per pass followed by four outputs per clock.
// Same ports as the block-at-a-time version it replaces; the Python model (tb/lr_model.py) is the oracle.
//
// Storage (v3, the mappable version): the line buffers are block RAMs of 4-sample words, read one word per
// line per clock. The filters never touch them directly: a sliding window of 12 columns (three words) x 10
// window rows in registers is filled from the line buffers as the column position advances, with the
// picture's edge samples replicated into the columns outside the plane, so every tap is a fixed window
// position and the filter datapath is pure arithmetic. (The previous version indexed a 16 x W register
// array at some 200 random columns per clock and synthesised to 622k LUTs.) The self-guided A/B tables
// (two passes x six rows x W+2 positions) are stored the same way: 12 line RAMs of 4-position words, written
// as the box phase completes each word, read into a second window that moves with the output column.
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
    input  logic [PXW-1:0] ly_first, ly_last,    // luma rows to restore (inclusive; multiples of 4): a stripe, or the whole frame
    output logic        busy,
    output logic        done,
    // mi_store: unit records
    output logic [1:0]  lrr_plane,
    output logic [SBW-1:0] lrr_row, lrr_col,
    input  lr_rec_t     lrr_rec,
    // deblocked frame (s0) and CdefFrame (s1) reads, LrFrame write
    output logic        s0_re, s1_re,
    output logic [1:0]  s_plane,
    output logic [FBX-1:0] s_x,
    output logic [FBY-1:0] s_y,
    input  logic [11:0] s0_rdata, s1_rdata,
    input  logic [47:0] s0_rdata4, s1_rdata4,           // aligned groups of 4 containing s_x
    output logic        d_we,
    output logic [1:0]  d_plane,
    output logic [FBX-1:0] d_x,
    output logic [FBY-1:0] d_y,
    output logic [11:0] d_wdata,
    output logic        d4_we,                          // 4-wide write of (d_x .. d_x+3, d_y) with lane enables d4_be
    output logic [3:0]  d4_be,
    output logic [47:0] d4_wdata
);
    localparam logic [1:0] RESTORE_NONE = 2'd0, RESTORE_WIENER = 2'd1, RESTORE_SGRPROJ = 2'd2;
    localparam int RING = 16;
    localparam int W = 1 << FBX;

    typedef enum logic [4:0] {
        L_IDLE, L_BAND, L_ROWS, L_LOAD, L_LOAD_LAST, L_UNIT, L_REC, L_DECIDE,
        L_COPY, L_WIEN, L_AB, L_AB_DRAIN, L_PASS_NEXT, L_BLEND, L_UNIT_NEXT, L_BAND_NEXT, L_DONE,
        L_WPRO                                                 // window prologue before a compute row
    } st_t;
    st_t st, st_after;                                         // compute state the prologue leads to

    // ---------------------------------------------------------------- band geometry (luma row ly, plane)
    logic [PXW-1:0] ly;
    logic [1:0]  plane;
    logic        sub_x, sub_y;
    logic [3:0]  bd;
    logic signed [PXS-1:0] stripe_start, stripe_end;
    logic [PXW-1:0] plane_end_x, plane_end_y, y0;
    logic [2:0]  hh;                                    // rows in the band (1..4)
    logic [1:0]  ftype;
    logic [3:0]  unit_log2;
    logic [PXW-1:0] unit_rows, unit_cols, ur;
    always_comb begin
        sub_x = (plane != 2'd0) && hdr.ssx; sub_y = (plane != 2'd0) && hdr.ssy;
        bd = hdr.bit_depth;
        ftype = hdr.lr_type[2 * plane +: 2];
        unit_log2 = 4'd5 + 4'(hdr.lr_size[2 * plane +: 2]);
        begin
            logic [PXW-1:0] stripe_num;
            stripe_num = (ly + PXW'(8)) >> 6;
            stripe_start = (PXS'(signed'({1'b0, stripe_num})) * PXS'(64) - PXS'(8)) >>> sub_y;
            stripe_end = stripe_start + (PXS'(64) >>> sub_y) - PXS'(1);
        end
        begin
            logic [PXW-1:0] fh_p, fw_p;
            fh_p = (hdr.frame_height + PXW'(sub_y)) >> sub_y;
            fw_p = (hdr.upscaled_width + PXW'(sub_x)) >> sub_x;
            unit_rows = (fh_p + (PXW'(1) << (unit_log2 - 4'd1))) >> unit_log2; if (unit_rows == PXW'(0)) unit_rows = PXW'(1);
            unit_cols = (fw_p + (PXW'(1) << (unit_log2 - 4'd1))) >> unit_log2; if (unit_cols == PXW'(0)) unit_cols = PXW'(1);
            plane_end_x = fw_p - PXW'(1); plane_end_y = fh_p - PXW'(1);
        end
        ur = ((ly + PXW'(8)) >> sub_y) >> unit_log2; if (ur > unit_rows - PXW'(1)) ur = unit_rows - PXW'(1);
        y0 = ly >> sub_y;
        begin
            logic [PXW-1:0] hmax;
            hmax = plane_end_y - y0 + PXW'(1);
            hh = (PXW'(3'd4 >> sub_y) < hmax) ? 3'(3'd4 >> sub_y) : 3'(hmax);
        end
    end

    // ---------------------------------------------------------------- window rows through the source rule
    // window row k (0 .. hh+5) is plane row y0 - 3 + k before the rule; after it: (source, row)
    function automatic logic [PXW:0] row_rule(input logic [3:0] k);         // {src, row[PXW-1:0]}
        logic signed [PXS-1:0] ry;
        logic src;
        ry = PXS'(signed'({2'b0, y0})) - PXS'(3) + PXS'(signed'({11'b0, k}));
        if (ry < 0) ry = PXS'(0);
        if (ry > PXS'(signed'({2'b0, plane_end_y}))) ry = PXS'(signed'({2'b0, plane_end_y}));
        src = 1'b0;
        if (ry < stripe_start) begin src = 1'b1; if (ry < stripe_start - PXS'(2)) ry = stripe_start - PXS'(2); end
        else if (ry > stripe_end) begin src = 1'b1; if (ry > stripe_end + PXS'(2)) ry = stripe_end + PXS'(2); end
        row_rule = {src, PXW'(ry)};
    endfunction
    // ---- line buffers: RING block RAMs of W/4 words x 4 samples, written by the loader (one aligned group of 4
    // per clock), read every clock at one common word address (the window fill picks the rows it needs)
    localparam int GW = FBX - 2;                           // word (group) address width
    logic           lb_we;
    logic [3:0]     lb_wslot;
    logic [GW-1:0]  lb_waddr, lb_raddr;
    logic [47:0]    lb_wdata;
    logic [47:0]    lb_rdata [0:RING-1];
    genvar gs;
    generate
        for (gs = 0; gs < RING; gs++) begin : g_line
            logic [47:0] mem [0:(1 << GW) - 1];
            always_ff @(posedge clk) begin
                if (lb_we && lb_wslot == 4'(gs)) mem[lb_waddr] <= lb_wdata;
                lb_rdata[gs] <= mem[lb_raddr];
            end
        end
    endgenerate
    logic [PXW-1:0] ring_row [0:RING-1];
    logic        ring_src [0:RING-1];
    logic        ring_ok  [0:RING-1];
    logic [3:0]  k;                                     // window row being checked / loaded
    logic [PXW:0] k_rule;
    logic [3:0]  k_slot;
    logic        k_hit;
    assign k_rule = row_rule(k);
    assign k_slot = k_rule[3:0];
    assign k_hit = ring_ok[k_slot] && (ring_row[k_slot] == k_rule[PXW-1:0]) && (ring_src[k_slot] == k_rule[PXW]);
    // slot of window row k for the compute phases (all rows present)
    function automatic logic [3:0] slot_of(input logic [3:0] kk);
        logic [PXW:0] r;
        r = row_rule(kk);
        slot_of = r[3:0];
    endfunction
    logic [PXW-1:0] lx;                                    // load column

    // ---------------------------------------------------------------- sliding window
    // win[k][c]: window row k (0 .. hh+5), column c of 12 = plane column 4 (wg - 1) + c, where wg is the group
    // (4 columns) the compute phase is at. Columns outside the plane hold the edge sample (replicated at fill
    // time), so a tap at a fixed (k, c) is already the clamped sample the spec asks for.
    //   Wiener lane l tap t (column x + l - 3 + t):  c = l + t + 1
    //   box position ax = 4 wg + a, offset dx:       c = 4 + a + dx
    //   source sample of lane l:                     c = 4 + l
    // The line RAMs are read every clock at group wg + 2 (virtual: clamped to the plane, fixed up per lane),
    // so a shift (wg -> wg + 1) can take the landing word as the new right-hand group. A prologue of four
    // clocks (L_WPRO) fills the three groups wg - 1, wg, wg + 1 at the start of each compute row.
    logic [11:0] win [0:9][0:11];
    logic win_shift;                                       // compute phase moves to the next group this clock
    logic signed [PXS-1:0] wg;                             // current group (may be -1 for the box phase)
    logic signed [PXS-1:0] rq_g, ld_g;                     // group requested this clock / landing this clock
    logic [1:0]  pro;                                      // prologue step
    logic [GW:0] last_g;                                   // last group of the plane
    logic [1:0]  last_lane;                                // lane of plane_end_x in it
    assign last_g = (GW + 1)'(plane_end_x >> 2);
    assign last_lane = plane_end_x[1:0];
    always_comb begin
        // the word to land next clock: the prologue's three groups, then the one the next shift will need (two
        // ahead of the group current at that shift: three ahead when shifting every clock, two in the box phase)
        rq_g = (st == L_WPRO) ? wg - PXS'(1) + PXS'(signed'({1'b0, pro})) : wg + PXS'(2) + PXS'(signed'({1'b0, win_shift}));
        lb_raddr = (rq_g < 0) ? GW'(0) : (rq_g > PXS'(signed'({1'b0, last_g}))) ? GW'(last_g) : GW'(rq_g);
    end
    // the landing word of window row k, with the edge replication for a virtual group ld_g
    function automatic logic [47:0] land(input logic [47:0] w, input logic signed [PXS-1:0] g);
        logic [47:0] r;
        for (int l = 0; l < 4; l++) begin
            if (g < 0) r[l * 12 +: 12] = w[0 +: 12];                                           // left of the plane: column 0
            else if (g > PXS'(signed'({1'b0, last_g})) || (g == PXS'(signed'({1'b0, last_g})) && 2'(l) > last_lane))
                r[l * 12 +: 12] = w[last_lane * 12 +: 12];                                     // right of the plane: plane_end_x
            else r[l * 12 +: 12] = w[l * 12 +: 12];
        end
        return r;
    endfunction
    logic [47:0] land_k [0:9];
    always_comb for (int kk = 0; kk < 10; kk++) land_k[kk] = land(lb_rdata[slot_of(4'(kk))], ld_g);
    always_ff @(posedge clk) begin
        ld_g <= rq_g;
        if (st == L_WPRO) begin
            // pro = 1, 2, 3: groups wg - 1, wg, wg + 1 land into window thirds 0, 1, 2
            if (pro != 2'd0)
                for (int kk = 0; kk < 10; kk++)
                    for (int l = 0; l < 4; l++) win[kk][4 * (int'(pro) - 1) + l] <= land_k[kk][l * 12 +: 12];
        end else if (win_shift) begin
            for (int kk = 0; kk < 10; kk++) begin
                for (int c = 0; c < 8; c++) win[kk][c] <= win[kk][c + 4];
                for (int l = 0; l < 4; l++) win[kk][8 + l] <= land_k[kk][l * 12 +: 12];
            end
        end
    end

    // ---------------------------------------------------------------- unit segment
    logic [PXW-1:0] uc, seg_x0, seg_x1, x;
    logic [2:0]  i;                                     // output row within the band
    lr_rec_t     rec;
    always_comb begin
        seg_x0 = uc << unit_log2;
        seg_x1 = (uc + PXW'(1) >= unit_cols) ? plane_end_x : PXW'(((uc + PXW'(1)) << unit_log2) - PXW'(1));
        lrr_plane = plane; lrr_row = SBW'(ur); lrr_col = SBW'(uc);
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
    // Wiener output of lane (column x + lane) of output row i (7.17.4), from the window
    function automatic logic [11:0] wien_at(input logic [1:0] lane);
        logic [11:0] wien_out;

        logic signed [30:0] vmac;
        vmac = 31'sd0;
        for (int r = 0; r < 7; r++) begin
            logic signed [30:0] hmac, v;
            logic signed [20:0] hv;
            hmac = 31'sd0;
            for (int t = 0; t < 7; t++)
                hmac = hmac + 31'(hf[t]) * 31'(signed'({19'b0, win[int'(i) + r][int'(lane) + t + 1]}));
            v = (hmac + (31'sd1 <<< (round0 - 4'd1))) >>> round0;
            hv = (v < -31'(w_offset)) ? -w_offset : (v > 31'(w_limit) - 31'(w_offset)) ? 21'(w_limit - w_offset) : 21'(v);
            vmac = vmac + 31'(vf[r]) * 31'(hv);
        end
        begin
            logic signed [30:0] v;
            v = (vmac + (31'sd1 <<< (round1 - 4'd1))) >>> round1;
            wien_out = (v < 0) ? 12'd0 : (v > 31'(signed'({19'b0, 12'((13'd1 << bd) - 13'd1)}))) ? 12'((13'd1 << bd) - 13'd1) : 12'(v);
        end
            return wien_out;
    endfunction
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
    logic signed [PXS-1:0] ax;
    // box sums as five per-row partial sums (the window reads, the squares and four adds per row); the pipeline's
    // first stage registers them and its second adds the rows, so no stage holds the whole 25-term tree
    logic [27:0] row_a [0:4]; logic [15:0] row_b [0:4];
    always_comb begin
        for (int dy = -2; dy <= 2; dy++) begin
            row_a[dy + 2] = 28'd0; row_b[dy + 2] = 16'd0;
            for (int dx = -2; dx <= 2; dx++) begin
                logic [11:0] c;
                // block row (ai - 1) + dy -> window row k = (ai - 1) + dy + 3; column ax + dx = 4 wg + a + dx
                c = win[int'(ai) + 2 + dy][4 + int'(ax[1:0]) + dx];
                if (dy >= -int'(r_p) && dy <= int'(r_p) && dx >= -int'(r_p) && dx <= int'(r_p)) begin
                    row_a[dy + 2] = row_a[dy + 2] + 28'(c) * 28'(c);
                    row_b[dy + 2] = row_b[dy + 2] + 16'(c);
                end
            end
        end
    end
    // A/B pipeline: six registered stages between the window and the A/B line RAM. As one combinational cone
    // (25 squares, a_r*n - d_r^2, p*s, the a2 ROM and two more multiplies) it was the decoder's worst path, 30 ns.
    // Every stage carries its position (idx = ax + 1), row, pass and "last position of the row" flag, so the
    // writes below land exactly as before, six clocks later; the sequencer drains the pipe before the blend.
    localparam int ABP = 6;
    logic        pv [1:ABP]; logic plast [1:ABP]; logic ppss [1:ABP];
    logic [2:0]  pai [1:ABP]; logic [PXS-1:0] pidx [1:ABP];
    logic [27:0] p0_row_a [0:4]; logic [15:0] p0_row_b [0:4];
    logic [29:0] p1_box_a; logic [17:0] p1_box_b;
    logic [36:0] p2_p;     logic [17:0] p2_box_b;
    logic [28:0] p3_z;     logic [17:0] p3_box_b;
    logic [8:0]  p4_a2;    logic [26:0] p4_m1;             // (256 - a2) * box_b
    logic [8:0]  p5_a2;    logic [21:0] p5_b2;
    logic        abp_busy;
    assign abp_busy = pv[1] || pv[2] || pv[3] || pv[4] || pv[5] || pv[6];
    // per-pass constants recomputed from the carried pass bit (pss flips while the pipe still drains a pass)
    function automatic logic [11:0] s_of(input logic ps);  return sgr_s(rec.sgr_set, ps); endfunction
    function automatic logic [1:0]  r_of(input logic ps);  return ps ? sp[6:5] : sp[15:14]; endfunction
    always_ff @(posedge clk) begin
        if (rst) for (int q = 1; q <= ABP; q++) pv[q] <= 1'b0;
        else begin
            pv[1] <= (st == L_AB);
            for (int q = 2; q <= ABP; q++) pv[q] <= pv[q - 1];
        end
        // stage 1: the five row partial sums of position (ai, ax)
        plast[1] <= (ax >= PXS'(signed'({2'b0, seg_x1})) + PXS'(1)); ppss[1] <= pss; pai[1] <= ai; pidx[1] <= ax + PXS'(1);
        for (int r = 0; r < 5; r++) begin p0_row_a[r] <= row_a[r]; p0_row_b[r] <= row_b[r]; end
        // stage 2: the box sums
        p1_box_a <= 30'(p0_row_a[0]) + 30'(p0_row_a[1]) + 30'(p0_row_a[2]) + 30'(p0_row_a[3]) + 30'(p0_row_a[4]);
        p1_box_b <= 18'(p0_row_b[0]) + 18'(p0_row_b[1]) + 18'(p0_row_b[2]) + 18'(p0_row_b[3]) + 18'(p0_row_b[4]);
        plast[2] <= plast[1]; ppss[2] <= ppss[1]; pai[2] <= pai[1]; pidx[2] <= pidx[1];
        // stage 3: p = max(0, a_r * n - d_r^2)
        begin
            logic [29:0] a_r; logic [17:0] d_r; logic [29:0] an; logic [35:0] dd; logic [5:0] n_1;
            a_r = (p1_box_a + (30'd1 << (2 * (bd - 4'd8))) / 2) >> (2 * (bd - 4'd8));
            if (bd == 4'd8) a_r = p1_box_a;
            d_r = (bd == 4'd8) ? p1_box_b : ((p1_box_b + (18'd1 << (bd - 4'd9))) >> (bd - 4'd8));
            n_1 = (r_of(ppss[2]) == 2'd1) ? 6'd9 : 6'd25;
            an = a_r * 30'(n_1);
            dd = 36'(d_r) * 36'(d_r);
            p2_p <= (36'(an) > dd) ? 37'(36'(an) - dd) : 37'd0;
        end
        plast[3] <= plast[2]; ppss[3] <= ppss[2]; pai[3] <= pai[2]; pidx[3] <= pidx[2]; p2_box_b <= p1_box_b;
        // stage 4: z = (p * s + 2^19) >> 20
        begin
            logic [48:0] pz;
            pz = 49'(p2_p) * 49'(s_of(ppss[3]));
            p3_z <= 29'((pz + (49'd1 << 19)) >> 20);
        end
        plast[4] <= plast[3]; ppss[4] <= ppss[3]; pai[4] <= pai[3]; pidx[4] <= pidx[3]; p3_box_b <= p2_box_b;
        // stage 5: a2 from the ROM, m1 = (256 - a2) * box_b
        begin
            logic [8:0] a2;
            if (p3_z >= 29'd255) a2 = 9'd256;
            else if (p3_z == 29'd0) a2 = 9'd1;
            else a2 = sgr_a2(8'(p3_z));
            p4_a2 <= a2;
            p4_m1 <= 27'(9'd256 - a2) * 27'(p3_box_b);
        end
        plast[5] <= plast[4]; ppss[5] <= ppss[4]; pai[5] <= pai[4]; pidx[5] <= pidx[4];
        // stage 6: b2 = (m1 * one_over_n + 2^11) >> 12
        begin
            logic [42:0] b2;
            b2 = 43'(p4_m1) * 43'(sgr_one_over_n(r_of(ppss[5])));
            p5_b2 <= 22'((b2 + (43'd1 << 11)) >> 12);
        end
        p5_a2 <= p4_a2;
        plast[6] <= plast[5]; ppss[6] <= ppss[5]; pai[6] <= pai[5]; pidx[6] <= pidx[5];
    end
    // A/B of both passes over the band: rows 0..5 (block row + 1), positions indexed by plane x + 1.
    // 12 line RAMs (pass, row) of 4-position words {a2, b2} x 4; the box phase accumulates a word and writes it
    // at its last position (or the row's end); the blend reads them through a window like the pixel one.
    localparam int ABW = 31;                               // {a2[8:0], b2[21:0]}
    logic             ab_we;
    logic [3:0]       ab_wline;                            // pass * 6 + row
    logic [GW:0]      ab_waddr, ab_raddr;                  // positions reach W + 1: one word more than the pixels
    logic [4*ABW-1:0] ab_wdata;
    logic [4*ABW-1:0] ab_rdata [0:11];
    genvar ga;
    generate
        for (ga = 0; ga < 12; ga++) begin : g_ab
            logic [4*ABW-1:0] mem [0:(1 << GW)];
            always_ff @(posedge clk) begin
                if (ab_we && ab_wline == 4'(ga)) mem[ab_waddr] <= ab_wdata;
                ab_rdata[ga] <= mem[ab_raddr];
            end
        end
    endgenerate
    logic [4*ABW-1:0] ab_acc;                              // the word being built, one position per clock
    logic [PXS-1:0]   ab_idx;                              // position index of the box position leaving the pipe
    assign ab_idx = pidx[ABP];
    always_comb begin
        ab_we = pv[ABP] && (ab_idx[1:0] == 2'd3 || plast[ABP]);
        ab_wline = 4'(ppss[ABP]) * 4'd6 + 4'(pai[ABP]);
        ab_waddr = (GW + 1)'(ab_idx >>> 2);
        ab_wdata = ab_acc;                                 // earlier positions of the word, plus the newest at its lane
        ab_wdata[ABW * int'(ab_idx[1:0]) +: ABW] = {p5_a2, p5_b2};
        ab_raddr = (rq_g < 0) ? (GW + 1)'(0) : (rq_g > PXS'(1 << GW)) ? (GW + 1)'(1 << GW) : (GW + 1)'(rq_g);
    end
    always_ff @(posedge clk)
        if (pv[ABP]) ab_acc[ABW * int'(ab_idx[1:0]) +: ABW] <= {p5_a2, p5_b2};
    // the A/B window: [pass][row][12 columns] = positions 4 (wg - 1) + c; f_of for lane l, offset dx reads
    // position x + l + 1 + dx = column 4 + l + 1 + dx
    logic [8:0]  awin [0:1][0:5][0:11];
    logic [21:0] bwin [0:1][0:5][0:11];
    always_ff @(posedge clk) begin
        if (st == L_WPRO) begin
            if (pro != 2'd0)
                for (int pp = 0; pp < 2; pp++) for (int rr = 0; rr < 6; rr++) for (int l = 0; l < 4; l++) begin
                    awin[pp][rr][4 * (int'(pro) - 1) + l] <= ab_rdata[pp * 6 + rr][l * ABW + 22 +: 9];
                    bwin[pp][rr][4 * (int'(pro) - 1) + l] <= ab_rdata[pp * 6 + rr][l * ABW +: 22];
                end
        end else if (win_shift) begin
            for (int pp = 0; pp < 2; pp++) for (int rr = 0; rr < 6; rr++) begin
                for (int c = 0; c < 8; c++) begin awin[pp][rr][c] <= awin[pp][rr][c + 4]; bwin[pp][rr][c] <= bwin[pp][rr][c + 4]; end
                for (int l = 0; l < 4; l++) begin
                    awin[pp][rr][8 + l] <= ab_rdata[pp * 6 + rr][l * ABW + 22 +: 9];
                    bwin[pp][rr][8 + l] <= ab_rdata[pp * 6 + rr][l * ABW +: 22];
                end
            end
        end
    end
    // F for output (i, x + lane) of pass p, then the blend
    function automatic logic [23:0] f_of(input logic p, input logic [2:0] fi, input logic [1:0] lane, input logic [11:0] pix);
        logic [15:0] fa; logic [28:0] fb; logic [3:0] wgt; logic [3:0] shift; logic [40:0] v;
        fa = 16'd0; fb = 29'd0;
        for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++) begin
                if (!p) wgt = (((int'(fi) + dy) & 1) != 0) ? ((dx == 0) ? 4'd6 : 4'd5) : 4'd0;
                else wgt = (dx == 0 || dy == 0) ? 4'd4 : 4'd3;
                fa = fa + 16'(wgt) * 16'(awin[p][int'(fi) + 1 + dy][5 + int'(lane) + dx]);
                fb = fb + 29'(wgt) * 29'(bwin[p][int'(fi) + 1 + dy][5 + int'(lane) + dx]);
            end
        shift = (!p && fi[0]) ? 4'd4 : 4'd5;
        v = 41'(fa) * 41'(pix) + 41'(fb);
        f_of = 24'((v + (41'd1 << (4'd8 + shift - 4'd4 - 4'd1))) >> (4'd8 + shift - 4'd4));
    endfunction
    function automatic logic [11:0] src_at(input logic [1:0] lane);   // source sample of output (i, x + lane): window row i + 3
        return win[int'(i) + 3][4 + int'(lane)];
    endfunction
    function automatic logic [11:0] sgr_at(input logic [1:0] lane);   // self-guided blend of lane (column x + lane) (7.17.3)
        logic [11:0] cur_pix, sgr_out;
        logic signed [8:0] w0, w1; logic signed [9:0] w2; logic signed [35:0] v; logic [16:0] u; logic signed [35:0] sv;
        cur_pix = src_at(lane);
        w0 = 9'(signed'(rec.xqd[7:0])); w1 = 9'(signed'(rec.xqd[15:8])); w2 = 10'sd128 - 10'(w0) - 10'(w1);
        u = 17'(cur_pix) << 4;
        v = 36'(w1) * 36'(signed'({19'b0, u}));
        v = v + 36'(w0) * ((sp[15:14] != 2'd0) ? 36'(signed'({12'b0, f_of(1'b0, i, lane, cur_pix)})) : 36'(signed'({19'b0, u})));
        v = v + 36'(w2) * ((sp[6:5] != 2'd0) ? 36'(signed'({12'b0, f_of(1'b1, i, lane, cur_pix)})) : 36'(signed'({19'b0, u})));
        sv = (v + 36'sd1024) >>> 11;
        sgr_out = (sv < 0) ? 12'd0 : (sv > 36'(signed'({24'b0, 12'((13'd1 << bd) - 13'd1)}))) ? 12'((13'd1 << bd) - 13'd1) : 12'(sv);
        return sgr_out;
    endfunction
    // the 4 output samples x .. x+3 of row i (lanes past the segment end are not written)
    logic [47:0] out4; logic [3:0] out_be;
    always_comb begin
        for (int l = 0; l < 4; l++) begin
            logic [PXW-1:0] xx;
            xx = x + PXW'(l);
            out_be[l] = (xx <= seg_x1);
            out4[l * 12 +: 12] = (st == L_COPY) ? src_at(2'(l)) : (st == L_WIEN) ? wien_at(2'(l)) : sgr_at(2'(l));
        end
    end

    // ---------------------------------------------------------------- memory ports
    logic ld_pend;                                      // a load read is in flight (lands this cycle)
    logic [PXW-1:0] ld_x;                                  // its x (aligned group of 4)
    always_comb begin
        lb_we = ld_pend; lb_wslot = k_slot; lb_waddr = GW'(ld_x >> 2);
        for (int l = 0; l < 4; l++) lb_wdata[l * 12 +: 12] = k_rule[PXW] ? s0_rdata4[l * 12 +: 12] : s1_rdata4[l * 12 +: 12];
        // the compute phases move to the next group: output states every clock, the box phase at a group's last column
        win_shift = (st == L_COPY || st == L_WIEN || st == L_BLEND) || (st == L_AB && ax[1:0] == 2'd3);
        s0_re = 1'b0; s1_re = 1'b0; s_plane = plane; s_x = FBX'(lx); s_y = FBY'(k_rule[PXW-1:0]);
        d_we = 1'b0; d_plane = plane; d_x = FBX'(x); d_y = FBY'(y0 + PXW'(i)); d_wdata = 12'd0;
        d4_we = 1'b0; d4_be = out_be; d4_wdata = out4;
        case (st)
            L_LOAD: begin s0_re = k_rule[PXW]; s1_re = !k_rule[PXW]; end
            L_COPY, L_WIEN, L_BLEND: d4_we = 1'b1;
            default: ;
        endcase
    end
    assign busy = (st != L_IDLE);

    // ---------------------------------------------------------------- FSM
    // a compute row starts with the window prologue: wg = the group of its first column, then the state after
    task automatic start_row(input logic signed [PXS-1:0] first_col, input st_t next);
        wg <= first_col >>> 2; pro <= 2'd0; st_after <= next; st <= L_WPRO;
    endtask
    always_ff @(posedge clk) begin
        done <= 1'b0;
        ld_pend <= 1'b0;                                                  // (the landing group is written by lb_we)
        if (win_shift) wg <= wg + PXS'(1);
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
                if (k > 4'(hh) + 4'd5) begin uc <= PXW'(0); st <= L_UNIT; end
                else if (k_hit) k <= k + 4'd1;
                else begin lx <= PXW'(0); st <= L_LOAD; end
            end
            L_LOAD: begin                                                     // one aligned group of 4 per clock, lands next cycle
                ld_pend <= 1'b1; ld_x <= lx;
                if (lx + PXW'(4) <= plane_end_x) lx <= lx + PXW'(4);
                else st <= L_LOAD_LAST;
            end
            L_LOAD_LAST: begin                                                // the last group lands this cycle
                ring_row[k_slot] <= k_rule[PXW-1:0]; ring_src[k_slot] <= k_rule[PXW]; ring_ok[k_slot] <= 1'b1;
                k <= k + 4'd1; st <= L_ROWS;
            end
            // ---- unit column segment
            L_UNIT: st <= L_REC;                                              // record read issued (lrr_* combinational)
            L_REC: begin rec <= lrr_rec; st <= L_DECIDE; end
            L_DECIDE: begin
                i <= 3'd0; x <= seg_x0; pss <= 1'b0;
                if (hh == 3'd0) st <= L_UNIT_NEXT;
                else if (ftype == RESTORE_NONE || lrr_rec.lr_type == RESTORE_NONE) start_row(PXS'(signed'({2'b0, seg_x0})), L_COPY);
                else if (lrr_rec.lr_type == RESTORE_WIENER) start_row(PXS'(signed'({2'b0, seg_x0})), L_WIEN);
                else begin
                    if (sp[15:14] != 2'd0) begin pss <= 1'b0; ai <= 3'd0; ax <= PXS'(signed'({2'b0, seg_x0})) - PXS'(1); start_row(PXS'(signed'({2'b0, seg_x0})) - PXS'(1), L_AB); end
                    else if (sp[6:5] != 2'd0) begin pss <= 1'b1; ai <= 3'd0; ax <= PXS'(signed'({2'b0, seg_x0})) - PXS'(1); start_row(PXS'(signed'({2'b0, seg_x0})) - PXS'(1), L_AB); end
                    else start_row(PXS'(signed'({2'b0, seg_x0})), L_BLEND);
                end
            end
            L_WPRO: begin                                                     // four clocks: request wg-1, wg, wg+1; the last lands
                if (pro == 2'd3) st <= st_after;
                pro <= pro + 2'd1;
            end
            L_COPY, L_WIEN, L_BLEND: begin                                    // four output samples per clock
                if (x + PXW'(4) <= seg_x1) x <= x + PXW'(4);
                else begin
                    x <= seg_x0;
                    if (i + 3'd1 < hh) begin i <= i + 3'd1; start_row(PXS'(signed'({2'b0, seg_x0})), st); end
                    else st <= L_UNIT_NEXT;
                end
            end
            L_AB: begin                                                       // one box position per clock
                if (ax < PXS'(signed'({2'b0, seg_x1})) + PXS'(1)) ax <= ax + PXS'(1);
                else begin
                    ax <= PXS'(signed'({2'b0, seg_x0})) - PXS'(1);
                    if (ai + 3'd1 < 3'(hh) + 3'd2) begin ai <= ai + 3'd1; start_row(PXS'(signed'({2'b0, seg_x0})) - PXS'(1), L_AB); end
                    else st <= L_AB_DRAIN;
                end
            end
            L_AB_DRAIN: if (!abp_busy) st <= L_PASS_NEXT;                 // the last positions land in the A/B RAM
            L_PASS_NEXT: begin
                if (!pss && sp[6:5] != 2'd0) begin pss <= 1'b1; ai <= 3'd0; ax <= PXS'(signed'({2'b0, seg_x0})) - PXS'(1); start_row(PXS'(signed'({2'b0, seg_x0})) - PXS'(1), L_AB); end
                else begin i <= 3'd0; x <= seg_x0; start_row(PXS'(signed'({2'b0, seg_x0})), L_BLEND); end
            end
            L_UNIT_NEXT: begin
                if (uc + PXW'(1) < unit_cols) begin uc <= uc + PXW'(1); st <= L_UNIT; end
                else st <= L_BAND_NEXT;
            end
            // ---- next band (luma rows +4), then next plane
            L_BAND_NEXT: begin
                if (ly + PXW'(4) <= ly_last && ly + PXW'(4) < hdr.frame_height) begin ly <= ly + PXW'(4); st <= L_BAND; end
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
