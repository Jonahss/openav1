// openav1 — intra predictor (AV1 spec 7.11.2), four pixels per clock.
//
// The caller loads the prepared edges (AboveRow[0..w+h-1], LeftCol[0..w+h-1], top-left) through
// the edge write port, sets the parameters and pulses start. The block then
//   1. runs the spec's edge preparation passes in place for directional modes
//      (filter corner, intra edge filter per side, 2x upsampling per side),
//   2. for DC, sums the available edges eight per clock and divides in one cycle,
//   3. emits pred[i][j] in raster order, four pixels per clock (an aligned group of columns), with (x, y) of
//      the group's first pixel alongside.
//
// Edge storage: A[] = AboveRow, L[] = LeftCol, indexed -2 .. 257 (index -1 holds the top-left
// sample of that side, exactly like the spec's two arrays; upsampling may rewrite it).
// Filter-intra keeps its own copy of the emitted pixels (max 32x32) for the recursion.
module ipred #(
    parameter int PW = 12                    // pixel width (covers 8/10/12-bit)
) (
    input  logic          clk,
    input  logic          rst,

    // ---- edge write port (while idle). side 0 = AboveRow[idx], 1 = LeftCol[idx], 2 = top-left ----
    input  logic          edge_we,
    input  logic [1:0]    edge_side,
    input  logic [7:0]    edge_idx,
    input  logic [PW-1:0] edge_data,
    input  logic          edge_we4,          // AboveRow[edge_idx .. edge_idx+3] <= edge_data4 lanes (edge_idx a multiple of 4)
    input  logic [4*PW-1:0] edge_data4,

    // ---- parameters (sampled at start) ----
    input  logic          start,
    input  logic [3:0]    mode,             // DC_PRED=0 .. PAETH_PRED=12 (spec order)
    input  logic          use_filter_intra,
    input  logic [2:0]    filter_intra_mode,
    input  logic signed [2:0] angle_delta,  // -3..3
    input  logic [2:0]    log2w,
    input  logic [2:0]    log2h,
    input  logic [3:0]    bit_depth,
    input  logic          have_left,
    input  logic          have_above,
    input  logic          filter_type,      // smooth neighbour (intra filter type process)
    input  logic          edge_filter_en,   // enable_intra_edge_filter
    input  logic [6:0]    above_px,         // Min(w, maxX - x + 1)
    input  logic [6:0]    left_px,          // Min(h, maxY - y + 1)
    output logic          busy,
    output logic          done,

    // ---- prediction output stream ----
    output logic          out_valid,
    output logic [5:0]    out_x,
    output logic [5:0]    out_y,
    output logic [4*PW-1:0] out_pix4       // pixels (out_x .. out_x+3, out_y), lane l at [l*PW +: PW]
);
    localparam int EOFF = 2;                 // edge index k is stored at k + EOFF
    localparam int ESZ  = 260;               // -2 .. 257

    localparam logic [3:0] DC_PRED = 0, SMOOTH_PRED = 9, SMOOTH_V_PRED = 10, SMOOTH_H_PRED = 11, PAETH_PRED = 12;

    logic [PW-1:0] A [ESZ];
    logic [PW-1:0] L [ESZ];
    logic [PW-1:0] TL_in;                    // top-left as written through the port

    // ------------------------------------------------------------------ latched parameters
    logic [3:0]  l_mode;
    logic        l_fi;
    logic [2:0]  l_fimode;
    logic [2:0]  l_log2w, l_log2h;
    logic [6:0]  w, h;
    logic [7:0]  wh;
    logic [3:0]  l_bd;
    logic        l_hl, l_ha, l_ft, l_efe;
    logic [6:0]  l_apx, l_lpx;
    logic [8:0]  p_angle;
    logic        up_a, up_l;
    logic [10:0] dx, dy;
    logic [PW-1:0] pix_max;
    assign pix_max = PW'((1 << l_bd) - 1);
    wire is_dir = !l_fi && (l_mode >= 4'd1) && (l_mode <= 4'd8);

    // ------------------------------------------------------------------ tables (spec)
    function automatic logic [8:0] mode_to_angle(input logic [3:0] m);
        case (m)
            4'd1: mode_to_angle = 90;  4'd2: mode_to_angle = 180; 4'd3: mode_to_angle = 45;  4'd4: mode_to_angle = 135;
            4'd5: mode_to_angle = 113; 4'd6: mode_to_angle = 157; 4'd7: mode_to_angle = 203; 4'd8: mode_to_angle = 67;
            default: mode_to_angle = 0;
        endcase
    endfunction

    function automatic logic [10:0] dr_intra_derivative(input logic [8:0] a);
        case (a)
            9'd3: dr_intra_derivative = 1023; 9'd6: dr_intra_derivative = 547; 9'd9: dr_intra_derivative = 372;
            9'd14: dr_intra_derivative = 273; 9'd17: dr_intra_derivative = 215; 9'd20: dr_intra_derivative = 178;
            9'd23: dr_intra_derivative = 151; 9'd26: dr_intra_derivative = 132; 9'd29: dr_intra_derivative = 116;
            9'd32: dr_intra_derivative = 102; 9'd36: dr_intra_derivative = 90;  9'd39: dr_intra_derivative = 80;
            9'd42: dr_intra_derivative = 71;  9'd45: dr_intra_derivative = 64;  9'd48: dr_intra_derivative = 57;
            9'd51: dr_intra_derivative = 51;  9'd54: dr_intra_derivative = 45;  9'd58: dr_intra_derivative = 40;
            9'd61: dr_intra_derivative = 35;  9'd64: dr_intra_derivative = 31;  9'd67: dr_intra_derivative = 27;
            9'd70: dr_intra_derivative = 23;  9'd73: dr_intra_derivative = 19;  9'd76: dr_intra_derivative = 15;
            9'd81: dr_intra_derivative = 11;  9'd84: dr_intra_derivative = 7;   9'd87: dr_intra_derivative = 3;
            default: dr_intra_derivative = 0;
        endcase
    endfunction

    function automatic logic [1:0] edge_strength(input logic [7:0] blk, input logic ft, input logic [8:0] d);
        logic [1:0] s;
        s = 0;
        if (!ft) begin
            if (blk <= 8)       begin if (d >= 56) s = 1; end
            else if (blk <= 12) begin if (d >= 40) s = 1; end
            else if (blk <= 16) begin if (d >= 40) s = 1; end
            else if (blk <= 24) begin if (d >= 8) s = 1; if (d >= 16) s = 2; if (d >= 32) s = 3; end
            else if (blk <= 32) begin s = 1; if (d >= 4) s = 2; if (d >= 32) s = 3; end
            else s = 3;
        end else begin
            if (blk <= 8)       begin if (d >= 40) s = 1; if (d >= 64) s = 2; end
            else if (blk <= 16) begin if (d >= 20) s = 1; if (d >= 48) s = 2; end
            else if (blk <= 24) begin if (d >= 4) s = 3; end
            else s = 3;
        end
        edge_strength = s;
    endfunction

    function automatic logic use_upsample(input logic [7:0] blk, input logic ft, input logic [8:0] d);
        if (d == 0 || d >= 40) use_upsample = 0;
        else if (!ft)          use_upsample = (blk <= 16);
        else                   use_upsample = (blk <= 8);
    endfunction

    function automatic logic [8:0] abs_diff(input logic [8:0] a, input logic [8:0] b);
        abs_diff = (a >= b) ? a - b : b - a;
    endfunction

    function automatic logic [7:0] sm_weight(input logic [2:0] lg, input logic [5:0] i);
        logic [7:0] w4 [4]; logic [7:0] w8 [8]; logic [7:0] w16 [16]; logic [7:0] w32 [32]; logic [7:0] w64 [64];
        w4  = '{255, 149, 85, 64};
        w8  = '{255, 197, 146, 105, 73, 50, 37, 32};
        w16 = '{255, 225, 196, 170, 145, 123, 102, 84, 68, 54, 43, 33, 26, 20, 17, 16};
        w32 = '{255, 240, 225, 210, 196, 182, 169, 157, 145, 133, 122, 111, 101, 92, 83, 74,
                66, 59, 52, 45, 39, 34, 29, 25, 21, 17, 14, 12, 10, 9, 8, 8};
        w64 = '{255, 248, 240, 233, 225, 218, 210, 203, 196, 189, 182, 176, 169, 163, 156,
                150, 144, 138, 133, 127, 121, 116, 111, 106, 101, 96, 91, 86, 82, 77, 73, 69,
                65, 61, 57, 54, 50, 47, 44, 41, 38, 35, 32, 29, 27, 25, 22, 20, 18, 16, 15,
                13, 12, 10, 9, 8, 7, 6, 6, 5, 5, 4, 4, 4};
        case (lg)
            3'd2:    sm_weight = w4[i[1:0]];
            3'd3:    sm_weight = w8[i[2:0]];
            3'd4:    sm_weight = w16[i[3:0]];
            3'd5:    sm_weight = w32[i[4:0]];
            default: sm_weight = w64[i];
        endcase
    endfunction

    function automatic logic signed [5:0] fi_tap(input logic [2:0] m, input logic [2:0] pos, input logic [2:0] t);
        logic signed [5:0] tab [5][8][7];
        tab = '{
            '{'{-6,10,0,0,0,12,0}, '{-5,2,10,0,0,9,0}, '{-3,1,1,10,0,7,0}, '{-3,1,1,2,10,5,0},
              '{-4,6,0,0,0,2,12}, '{-3,2,6,0,0,2,9}, '{-3,2,2,6,0,2,7}, '{-3,1,2,2,6,3,5}},
            '{'{-10,16,0,0,0,10,0}, '{-6,0,16,0,0,6,0}, '{-4,0,0,16,0,4,0}, '{-2,0,0,0,16,2,0},
              '{-10,16,0,0,0,0,10}, '{-6,0,16,0,0,0,6}, '{-4,0,0,16,0,0,4}, '{-2,0,0,0,16,0,2}},
            '{'{-8,8,0,0,0,16,0}, '{-8,0,8,0,0,16,0}, '{-8,0,0,8,0,16,0}, '{-8,0,0,0,8,16,0},
              '{-4,4,0,0,0,0,16}, '{-4,0,4,0,0,0,16}, '{-4,0,0,4,0,0,16}, '{-4,0,0,0,4,0,16}},
            '{'{-2,8,0,0,0,10,0}, '{-1,3,8,0,0,6,0}, '{-1,2,3,8,0,4,0}, '{0,1,2,3,8,2,0},
              '{-1,4,0,0,0,3,10}, '{-1,3,4,0,0,4,6}, '{-1,2,3,4,0,4,4}, '{-1,2,2,3,4,3,3}},
            '{'{-12,14,0,0,0,14,0}, '{-10,0,14,0,0,12,0}, '{-9,0,0,14,0,11,0}, '{-8,0,0,0,14,10,0},
              '{-10,12,0,0,0,0,14}, '{-9,1,12,0,0,0,12}, '{-8,0,0,12,0,1,11}, '{-7,0,0,1,12,1,9}}
        };
        fi_tap = (m < 3'd5 && t < 3'd7) ? tab[m][pos][t] : 6'sd0;
    endfunction

    function automatic int ai(input int idx); ai = idx + EOFF; endfunction

    function automatic logic [PW-1:0] clip1i(input int v, input logic [PW-1:0] mx);
        if (v < 0) clip1i = '0;
        else if (v > int'(mx)) clip1i = mx;
        else clip1i = PW'(v);
    endfunction

    // ------------------------------------------------------------------ control state
    typedef enum logic [3:0] {
        S_IDLE, S_SETUP, S_CORNER, S_EF, S_UP_A, S_UP_L, S_DC_SUM, S_PIX, S_DONE
    } state_t;
    state_t state;

    logic [8:0]    k;                 // pass counter
    logic          ef_side;           // 0 above, 1 left (edge filter pass)
    logic [1:0]    ef_str;
    logic [8:0]    ef_sz;             // edge filter sz / upsample numPx
    logic [PW-1:0] o0, o1;            // originals of edge[k-2], edge[k-1] during the edge filter
    logic [PW-1:0] up_save1;          // original buf[1] across the descending upsample pass

    logic [19:0]   dc_sum, dc_part;   // running sum and the sum of the next (up to) 8 edge samples
    logic [8:0]    dc_n;              // number of edge samples summed
    logic [PW-1:0] dc_val;

    logic [5:0]    px, py;

    // filter-intra: emitted pixels, one 4x2 patch (8 pixels) per entry, entry = (row>>1)*8 + (col>>2)
    logic [8*PW-1:0] fi_buf [128];
    logic [8*PW-1:0] fi_next_packed;
    function automatic logic [PW-1:0] fi_rd(input int r, input int c);
        fi_rd = fi_buf[((r >> 1) & 15) * 8 + ((c >> 2) & 7)][((r & 1) * 4 + (c & 3)) * PW +: PW];
    endfunction

    // ------------------------------------------------------------------ pixel datapath: pred[i][j]
    function automatic logic [PW-1:0] pred_pix(input int i, input int j);
        int tl, base, pl, pt, ptl, wy, wx, s, idx, shift, maxb, idx2, base2, shift2;
        logic [PW-1:0] pix_val;
        tl = 0; base = 0; pl = 0; pt = 0; ptl = 0; wy = 0; wx = 0; s = 0; idx = 0; shift = 0; maxb = 0;
        idx2 = 0; base2 = 0; shift2 = 0;
        pix_val = '0;
        if (l_fi) begin
            pix_val = fi_rd(i, j);
        end else if (l_mode == DC_PRED) begin
            pix_val = dc_val;
        end else if (l_mode == PAETH_PRED) begin
            tl   = int'(A[ai(-1)]);
            base = int'(A[ai(j)]) + int'(L[ai(i)]) - tl;
            pl  = base - int'(L[ai(i)]);  if (pl < 0) pl = -pl;
            pt  = base - int'(A[ai(j)]);  if (pt < 0) pt = -pt;
            ptl = base - tl;              if (ptl < 0) ptl = -ptl;
            if (pl <= pt && pl <= ptl) pix_val = L[ai(i)];
            else if (pt <= ptl)        pix_val = A[ai(j)];
            else                       pix_val = PW'(tl);
        end else if (l_mode == SMOOTH_PRED) begin
            wy = int'(sm_weight(l_log2h, 6'(i)));
            wx = int'(sm_weight(l_log2w, 6'(j)));
            s = wy * int'(A[ai(j)]) + (256 - wy) * int'(L[ai(int'(h) - 1)])
              + wx * int'(L[ai(i)]) + (256 - wx) * int'(A[ai(int'(w) - 1)]);
            pix_val = PW'((s + 256) >> 9);
        end else if (l_mode == SMOOTH_V_PRED) begin
            wy = int'(sm_weight(l_log2h, 6'(i)));
            s = wy * int'(A[ai(j)]) + (256 - wy) * int'(L[ai(int'(h) - 1)]);
            pix_val = PW'((s + 128) >> 8);
        end else if (l_mode == SMOOTH_H_PRED) begin
            wx = int'(sm_weight(l_log2w, 6'(j)));
            s = wx * int'(L[ai(i)]) + (256 - wx) * int'(A[ai(int'(w) - 1)]);
            pix_val = PW'((s + 128) >> 8);
        end else if (p_angle == 9'd90) begin
            pix_val = A[ai(j)];
        end else if (p_angle == 9'd180) begin
            pix_val = L[ai(i)];
        end else if (p_angle < 9'd90) begin                                   // Z1
            idx   = (i + 1) * int'(dx);
            base  = (idx >> (6 - int'(up_a))) + (j << int'(up_a));
            shift = ((idx << int'(up_a)) >> 1) & 31;
            maxb  = (int'(w) + int'(h) - 1) << int'(up_a);
            if (base < maxb)
                pix_val = PW'((int'(A[ai(base)]) * (32 - shift) + int'(A[ai(base + 1)]) * shift + 16) >> 5);
            else
                pix_val = A[ai(maxb)];
        end else if (p_angle < 9'd180) begin                                  // Z2
            idx  = (j << 6) - (i + 1) * int'(dx);
            base = idx >>> (6 - int'(up_a));
            if (base >= -(1 << int'(up_a))) begin
                shift = ((idx << int'(up_a)) >> 1) & 31;
                pix_val = PW'((int'(A[ai(base)]) * (32 - shift) + int'(A[ai(base + 1)]) * shift + 16) >> 5);
            end else begin
                idx2   = (i << 6) - (j + 1) * int'(dy);
                base2  = idx2 >>> (6 - int'(up_l));
                shift2 = ((idx2 << int'(up_l)) >> 1) & 31;
                pix_val = PW'((int'(L[ai(base2)]) * (32 - shift2) + int'(L[ai(base2 + 1)]) * shift2 + 16) >> 5);
            end
        end else begin                                                        // Z3
            idx   = (j + 1) * int'(dy);
            base  = (idx >> (6 - int'(up_l))) + (i << int'(up_l));
            shift = ((idx << int'(up_l)) >> 1) & 31;
            pix_val = PW'((int'(L[ai(base)]) * (32 - shift) + int'(L[ai(base + 1)]) * shift + 16) >> 5);
        end
        return pix_val;
    endfunction
    logic [PW-1:0] pix4 [4];
    always_comb for (int l = 0; l < 4; l++) pix4[l] = pred_pix(int'(py), int'(px) + l);

    // filter-intra: the 4x2 patch at (i2 = py>>1, j4 = px>>2), from edges and already-predicted pixels
    logic [PW-1:0] fi_next [2][4];
    always_comb begin
        int p [7];
        int i2, j4, pr;
        i2 = int'(py) >> 1; j4 = int'(px) >> 2; pr = 0;
        for (int t = 0; t < 7; t++) begin
            p[t] = 0;
            if (t < 5) begin
                if (i2 == 0)                p[t] = int'(A[ai((j4 << 2) + t - 1)]);          // t=0 -> A[-1] = top-left
                else if (j4 == 0 && t == 0) p[t] = int'(L[ai((i2 << 1) - 1)]);
                else                        p[t] = int'(fi_rd((i2 << 1) - 1, (j4 << 2) + t - 1));
            end else begin
                if (j4 == 0) p[t] = int'(L[ai((i2 << 1) + t - 5)]);
                else         p[t] = int'(fi_rd((i2 << 1) + t - 5, (j4 << 2) - 1));
            end
        end
        for (int i1 = 0; i1 < 2; i1++)
            for (int j1 = 0; j1 < 4; j1++) begin
                pr = 0;
                for (int t = 0; t < 7; t++) pr += int'(fi_tap(l_fimode, 3'((i1 << 2) + j1), 3'(t))) * p[t];
                if (pr >= 0) pr = (pr + 8) >> 4; else pr = -((-pr + 8) >> 4);   // Round2Signed(pr, 4)
                fi_next[i1][j1] = clip1i(pr, pix_max);
            end
        for (int i1 = 0; i1 < 2; i1++)
            for (int j1 = 0; j1 < 4; j1++)
                fi_next_packed[(i1 * 4 + j1) * PW +: PW] = fi_next[i1][j1];
    end

    // ------------------------------------------------------------------ edge filter step
    // edge[m] = side[m-1] for m = 0..sz-1 (edge[0] = top-left at index -1). Output side[k-1] for k = 1..sz-1
    // from ORIGINAL edge values at clip(k-2..k+2): k-2, k-1 come from o0/o1 (already overwritten), k.. from the array.
    function automatic logic [PW-1:0] ef_calc(input logic [PW-1:0] e0, e1, e2, e3, e4, input logic [1:0] str);
        int s;
        case (str)
            2'd1:    s = 4 * int'(e1) + 8 * int'(e2) + 4 * int'(e3);
            2'd2:    s = 5 * int'(e1) + 6 * int'(e2) + 5 * int'(e3);
            default: s = 2 * int'(e0) + 4 * int'(e1) + 4 * int'(e2) + 4 * int'(e3) + 2 * int'(e4);
        endcase
        ef_calc = PW'((s + 8) >> 4);
    endfunction

    logic [PW-1:0] ef_out;
    function automatic logic [PW-1:0] ef_src(input int m, input int kk);
        // original value of edge[m] at step kk: positions 1..kk-1 were overwritten (history o0/o1),
        // position 0 (top-left) and positions >= kk are untouched in the array
        if (m >= 1 && m == kk - 1)      ef_src = o1;
        else if (m >= 1 && m == kk - 2) ef_src = o0;
        else                            ef_src = ef_side ? L[ai(m - 1)] : A[ai(m - 1)];
    endfunction
    always_comb begin
        int m0, m1, m3, m4, kk, szi;
        kk = int'(k); szi = int'(ef_sz);
        m0 = kk - 2; m1 = kk - 1; m3 = kk + 1; m4 = kk + 2;
        if (m0 < 0) m0 = 0; if (m1 < 0) m1 = 0;
        if (m3 > szi - 1) m3 = szi - 1; if (m4 > szi - 1) m4 = szi - 1;
        ef_out = ef_calc(ef_src(m0, kk), ef_src(m1, kk), ef_src(kk, kk), ef_src(m3, kk), ef_src(m4, kk), ef_str);
    end

    // ------------------------------------------------------------------ upsample step (descending i)
    // dup[i]=buf[i-2] (i==0 -> buf[-1]), dup[i+1]=buf[i-1], dup[i+2]=buf[i], dup[i+3]=buf[i+1] (clamped to numPx-1).
    // Writes buf[2i-1] and buf[2i]. Processing i descending keeps every read original except buf[1] (saved).
    logic [PW-1:0] up_s, up_d2;
    always_comb begin
        int i, n, s;
        logic [PW-1:0] d0, d1, d2, d3;
        logic side;
        s = 0; d0 = '0; d1 = '0; d2 = '0; d3 = '0;
        side = (state == S_UP_L);
        i = int'(k) - 1; n = int'(ef_sz);
        d0 = side ? L[ai((i - 2 < -1) ? -1 : i - 2)] : A[ai((i - 2 < -1) ? -1 : i - 2)];
        d1 = side ? L[ai(i - 1)] : A[ai(i - 1)];
        d2 = side ? L[ai(i)] : A[ai(i)];
        d3 = side ? L[ai((i + 1 > n - 1) ? n - 1 : i + 1)] : A[ai((i + 1 > n - 1) ? n - 1 : i + 1)];
        if (i - 2 == 1) d0 = up_save1;
        if (i - 1 == 1) d1 = up_save1;
        if (i == 1)     d2 = up_save1;
        if (i + 1 == 1 && n - 1 >= 1) d3 = up_save1;
        s = -int'(d0) + 9 * int'(d1) + 9 * int'(d2) - int'(d3);
        s = (s + 8) >>> 4;                 // arithmetic: s may be negative (then Clip1 -> 0)
        up_s  = clip1i(s, pix_max);
        up_d2 = d2;
    end

    // DC: the edge samples in order LeftCol[0..h-1] (if have_left) then AboveRow[0..w-1] (if have_above);
    // dc_part sums the 8 of them at k .. k+7 (masked past the end)
    always_comb begin
        dc_n = (l_hl ? 9'(h) : 9'd0) + (l_ha ? 9'(w) : 9'd0);
        dc_part = 20'd0;
        for (int l = 0; l < 8; l++) begin
            int kk;
            kk = int'(k) + l;
            if (kk < int'(dc_n)) begin
                if (l_hl && kk < int'(h)) dc_part += 20'(L[ai(kk)]);
                else                      dc_part += 20'(A[ai(kk - (l_hl ? int'(h) : 0))]);
            end
        end
    end
    logic first_of_patch;           // filter-intra: an even row emits the freshly computed 4x2 patch's top row
    assign first_of_patch = l_fi && (py[0] == 1'b0);

    // ------------------------------------------------------------------ sequencer
    always_ff @(posedge clk) begin
        done      <= 1'b0;
        out_valid <= 1'b0;
        if (rst) begin
            state <= S_IDLE;
            busy  <= 1'b0;
        end else begin
            case (state)
            S_IDLE: begin
                if (edge_we) begin
                    case (edge_side)
                        2'd0: A[EOFF + (int'(edge_idx))] <= edge_data;
                        2'd1: L[EOFF + (int'(edge_idx))] <= edge_data;
                        default: TL_in <= edge_data;
                    endcase
                end
                if (edge_we4) for (int l = 0; l < 4; l++) A[EOFF + (int'(edge_idx)) + l] <= edge_data4[l * PW +: PW];
                if (start) begin
                    l_mode <= mode; l_fi <= use_filter_intra; l_fimode <= filter_intra_mode;
                    l_log2w <= log2w; l_log2h <= log2h;
                    w <= 7'd1 << log2w; h <= 7'd1 << log2h;
                    wh <= 8'(7'd1 << log2w) + 8'(7'd1 << log2h);
                    l_bd <= bit_depth;
                    l_hl <= have_left; l_ha <= have_above; l_ft <= filter_type; l_efe <= edge_filter_en;
                    l_apx <= above_px; l_lpx <= left_px;
                    p_angle <= mode_to_angle(mode) + 9'(signed'({{6{angle_delta[2]}}, angle_delta}) * 3);
                    up_a <= 1'b0; up_l <= 1'b0;
                    A[EOFF + (-1)] <= TL_in; L[ai(-1)] <= TL_in;
                    busy  <= 1'b1;
                    state <= S_SETUP;
                end
            end
            S_SETUP: begin
                px <= 0; py <= 0; dc_sum <= 0; k <= 0;
                if (is_dir) begin
                    dx <= (p_angle < 90) ? dr_intra_derivative(p_angle) :
                          (p_angle < 180) ? dr_intra_derivative(9'd180 - p_angle) : 11'd0;
                    dy <= (p_angle > 90 && p_angle < 180) ? dr_intra_derivative(p_angle - 9'd90) :
                          (p_angle > 180) ? dr_intra_derivative(9'd270 - p_angle) : 11'd0;
                    state <= (l_efe && p_angle != 90 && p_angle != 180) ? S_CORNER : S_PIX;
                end else if (!l_fi && l_mode == DC_PRED) begin
                    if (!l_hl && !l_ha) begin dc_val <= PW'(1 << (l_bd - 1)); state <= S_PIX; end
                    else state <= S_DC_SUM;
                end else state <= S_PIX;
            end
            S_CORNER: begin
                if (p_angle > 90 && p_angle < 180 && wh >= 24) begin
                    A[EOFF + (-1)] <= PW'((int'(L[ai(0)]) * 5 + int'(A[ai(-1)]) * 6 + int'(A[ai(0)]) * 5 + 8) >> 4);
                    L[EOFF + (-1)] <= PW'((int'(L[ai(0)]) * 5 + int'(A[ai(-1)]) * 6 + int'(A[ai(0)]) * 5 + 8) >> 4);
                end
                // set up the above-edge filter pass (or the left one if there is no above)
                if (l_ha) begin
                    ef_side <= 1'b0;
                    ef_str  <= edge_strength(wh, l_ft, abs_diff(p_angle, 9'd90));
                    ef_sz   <= 9'(l_apx) + (p_angle < 90 ? 9'(h) : 9'd0) + 9'd1;
                end else begin
                    ef_side <= 1'b1;
                    ef_str  <= edge_strength(wh, l_ft, abs_diff(p_angle, 9'd180));
                    ef_sz   <= 9'(l_lpx) + (p_angle > 180 ? 9'(w) : 9'd0) + 9'd1;
                end
                k <= (l_ha || l_hl) ? 9'd1 : 9'd0;      // S_UP_A expects k == 0 on entry
                state <= (l_ha || l_hl) ? S_EF : S_UP_A;
            end
            S_EF: begin
                if (ef_str == 0 || k >= ef_sz) begin
                    if (!ef_side && l_hl) begin
                        ef_side <= 1'b1;
                        ef_str  <= edge_strength(wh, l_ft, abs_diff(p_angle, 9'd180));
                        ef_sz   <= 9'(l_lpx) + (p_angle > 180 ? 9'(w) : 9'd0) + 9'd1;
                        k <= 9'd1;
                    end else begin
                        state <= S_UP_A; k <= 0;
                    end
                end else begin
                    // side[k-1] (= edge[k]) <- filtered; remember its original for the next two steps
                    if (ef_side) L[EOFF + (int'(k) - 1)] <= ef_out;
                    else         A[EOFF + (int'(k) - 1)] <= ef_out;
                    o0 <= o1;
                    o1 <= ef_side ? L[ai(int'(k) - 1)] : A[ai(int'(k) - 1)];
                    k  <= k + 9'd1;
                end
            end
            S_UP_A: begin
                if (k == 0) begin
                    if (is_dir && l_efe && use_upsample(wh, l_ft, abs_diff(p_angle, 9'd90))) begin
                        up_a  <= 1'b1;
                        ef_sz <= 9'(w) + (p_angle < 90 ? 9'(h) : 9'd0);
                        k     <= 9'(w) + (p_angle < 90 ? 9'(h) : 9'd0);
                        up_save1 <= A[ai(1)];
                    end else begin
                        state <= S_UP_L;
                    end
                end else begin
                    int i;
                    i = int'(k) - 1;
                    A[EOFF + (2 * i - 1)] <= up_s;
                    A[EOFF + (2 * i)]     <= up_d2;
                    if (i == 0) begin
                        A[EOFF + (-2)] <= A[ai(-1)];
                        state <= S_UP_L; k <= 0;
                    end else k <= k - 9'd1;
                end
            end
            S_UP_L: begin
                if (k == 0) begin
                    if (is_dir && l_efe && use_upsample(wh, l_ft, abs_diff(p_angle, 9'd180))) begin
                        up_l  <= 1'b1;
                        ef_sz <= 9'(h) + (p_angle > 180 ? 9'(w) : 9'd0);
                        k     <= 9'(h) + (p_angle > 180 ? 9'(w) : 9'd0);
                        up_save1 <= L[ai(1)];
                    end else begin
                        state <= S_PIX;
                    end
                end else begin
                    int i;
                    i = int'(k) - 1;
                    L[EOFF + (2 * i - 1)] <= up_s;
                    L[EOFF + (2 * i)]     <= up_d2;
                    if (i == 0) begin
                        L[EOFF + (-2)] <= L[ai(-1)];
                        state <= S_PIX;
                    end else k <= k - 9'd1;
                end
            end
            S_DC_SUM: begin
                if (k < dc_n) begin dc_sum <= dc_sum + dc_part; k <= k + 9'd8; end
                else begin
                    // both edges: (sum + (w+h)/2) / (w+h), an exact integer division by one of 12 possible sizes
                    if (l_hl && l_ha) dc_val <= PW'((dc_sum + 20'(wh >> 1)) / 20'(wh));
                    else if (l_hl)    dc_val <= PW'((dc_sum + 20'(h >> 1)) >> l_log2h);
                    else              dc_val <= PW'((dc_sum + 20'(w >> 1)) >> l_log2w);
                    state <= S_PIX;
                end
            end
            S_PIX: begin
                if (first_of_patch)
                    fi_buf[(int'(py) >> 1) * 8 + (int'(px) >> 2)] <= fi_next_packed;
                for (int l = 0; l < 4; l++) out_pix4[l*PW +: PW] <= first_of_patch ? fi_next[0][l] : pix4[l];
                out_valid <= 1'b1;
                out_x <= px; out_y <= py;
                if (px + 7'd4 >= w) begin
                    px <= 0;
                    if (py + 7'd1 == h) state <= S_DONE;
                    else py <= py + 6'd1;
                end else px <= px + 6'd4;
            end
            S_DONE: begin
                done  <= 1'b1;
                busy  <= 1'b0;
                state <= S_IDLE;
            end
            default: state <= S_IDLE;
            endcase
        end
    end
endmodule
