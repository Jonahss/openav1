// CDEF (spec 7.15), frame-level, window version: for every 8x8 luma block (raster order) read the deblocked
// frame (src), write CdefFrame (dst). Blocks whose 64x64 unit has no cdef_idx or whose four 4x4s are all
// skipped are copied unfiltered. Otherwise, per plane, the block plus a 2-sample border is loaded once into
// a 12x12 window register file (one aligned group of 4 samples per clock; samples outside the MI-aligned frame are marked absent:
// is_inside_filter_region), the direction search (7.15.2: 8 x 15 partial sums, the 64 cost terms with
// Div_Table, argmax, variance) is evaluated from the luma window in one cycle, and the constrained
// directional filter (7.15.3) produces one output sample per clock from the window. Chroma uses the chroma
// direction map and damping - 1. ~4 cycles per sample against ~16 for the sample-at-a-time version.
module cdef_top
  import syn_pkg::*;
  import cdef_pkg::*;
#(
    parameter int FBX = 10,
    parameter int FBY = 9
) (
    input  logic        clk,
    input  logic        rst,
    input  hdr_t        hdr,
    input  cdef_hdr_t   ch,
    input  logic        start,
    input  logic [MIW-1:0] row_first, row_last,  // mi rows of the 8x8 blocks to filter (inclusive; multiples of 2)
    output logic        busy,
    output logic        done,
    // mi_store: skips (lf_info) and cdef_idx
    output logic [MIW-1:0] rd_row, rd_col,
    input  mi_lf_t      rd_info,
    output logic [SBW-1:0]  cdr_row64, cdr_col64,
    input  logic [3:0]  cdr_val,
    // deblocked frame (read) and CdefFrame (write)
    output logic        src_re,
    output logic [1:0]  src_plane,
    output logic [FBX-1:0] src_x,
    output logic [FBY-1:0] src_y,
    input  logic [11:0] src_rdata,
    input  logic [47:0] src_rdata4,      // aligned group of 4 samples containing src_x (window loads, block copies)
    output logic        dst_we,
    output logic [1:0]  dst_plane,
    output logic [FBX-1:0] dst_x,
    output logic [FBY-1:0] dst_y,
    output logic [11:0] dst_wdata,
    output logic        dst4_we,         // 4-wide write (block copies), aligned at dst4_x
    output logic [1:0]  dst4_plane,
    output logic [FBX-1:0] dst4_x,
    output logic [FBY-1:0] dst4_y,
    output logic [47:0] dst4_wdata
);
    typedef enum logic [4:0] {
        C_IDLE, C_BLK, C_RD_IDX, C_RD_S0, C_RD_S1, C_RD_S2, C_RD_S3, C_DECIDE,
        C_PLANE, C_WIN, C_WIN_LAST, C_DIR_A, C_DIR_B, C_DIR_B2, C_DIR_C, C_DIR, C_PIX,
        C_COPY_RD, C_COPY_WR, C_NEXT
    } st_t;
    st_t st;
    logic done_pend;

    // ---------------------------------------------------------------- block loop
    logic [MIW-1:0] r, c;                                  // 8x8 block position in 4x4 units (step 2)
    logic [3:0]  cidx;                                  // {valid, idx}
    logic [3:0]  skips;
    logic [MIW-1:0] r1, c1;
    logic [3:0]  cs;                                    // coeffShift = BitDepth - 8
    always_comb begin
        r1 = (r + MIW'(1) < hdr.mi_rows) ? r + MIW'(1) : hdr.mi_rows - MIW'(1);
        c1 = (c + MIW'(1) < hdr.mi_cols) ? c + MIW'(1) : hdr.mi_cols - MIW'(1);
        cs = hdr.bit_depth - 4'd8;
        cdr_row64 = SBW'(r >> 4); cdr_col64 = SBW'(c >> 4);
        case (st)
            C_RD_S0: begin rd_row = r;  rd_col = c;  end
            C_RD_S1: begin rd_row = r1; rd_col = c;  end
            C_RD_S2: begin rd_row = r;  rd_col = c1; end
            default: begin rd_row = r1; rd_col = c1; end
        endcase
    end

    // ---------------------------------------------------------------- plane geometry
    logic [1:0]  fplane;
    logic        sub_x, sub_y;
    logic [3:0]  pw, ph;                                // block dims in the plane (8 or 4)
    logic [PXW-1:0] px0, py0, fw, fh;
    always_comb begin
        sub_x = (fplane != 2'd0) && hdr.ssx; sub_y = (fplane != 2'd0) && hdr.ssy;
        pw = 4'd8 >> sub_x; ph = 4'd8 >> sub_y;
        px0 = (PXW'(c) << 2) >> sub_x; py0 = (PXW'(r) << 2) >> sub_y;
        fw = (PXW'(hdr.mi_cols) << 2) >> sub_x; fh = (PXW'(hdr.mi_rows) << 2) >> sub_y;
    end

    // ---------------------------------------------------------------- window: block + 2-sample border, [row][col] at +2
    logic [11:0] win [0:11][0:11];
    logic        win_ok [0:11][0:11];
    logic [3:0]  wr;                                    // load row 0 .. ph+3
    logic [1:0]  wg, wg_last;                           // aligned group of 4 within the row: x = px0 - 4 + 4 wg
    logic        wpend; logic [3:0] wpr; logic [1:0] wpg; logic wp_rowok; logic signed [PXS-1:0] wpx;
    logic signed [PXS-1:0] wy, wx;
    logic        w_row_in;
    always_comb begin
        wy = PXS'(signed'({2'b0, py0})) + PXS'(signed'({11'b0, wr})) - PXS'(2);
        wx = PXS'(signed'({2'b0, px0})) + PXS'(signed'({13'b0, wg}) << 2) - PXS'(4);
        w_row_in = (wy >= 0) && (wy < PXS'(signed'({2'b0, fh})));
        wg_last = (pw == 4'd8) ? 2'd3 : 2'd2;           // 12 window columns span 4 groups, 8 span 3
    end

    // ---------------------------------------------------------------- 7.15.2 direction search from the luma window
    logic signed [15:0] part [0:7][0:14];
    always_comb begin
        for (int d = 0; d < 8; d++) for (int k = 0; k < 15; k++) part[d][k] = 16'sd0;
        for (int i = 0; i < 8; i++)
            for (int j = 0; j < 8; j++) begin
                logic signed [15:0] s;
                s = 16'(signed'({4'b0, win[i + 2][j + 2] >> cs})) - 16'sd128;
                part[0][i + j] = part[0][i + j] + s;
                part[1][i + (j >> 1)] = part[1][i + (j >> 1)] + s;
                part[2][i] = part[2][i] + s;
                part[3][3 + i - (j >> 1)] = part[3][3 + i - (j >> 1)] + s;
                part[4][7 + i - j] = part[4][7 + i - j] + s;
                part[5][3 - (i >> 1) + j] = part[5][3 - (i >> 1) + j] + s;
                part[6][j] = part[6][j] + s;
                part[7][(i >> 1) + j] = part[7][(i >> 1) + j] + s;
            end
    end
    // The search is pipelined over four extra luma states (C_DIR_A, C_DIR_B, C_DIR_B2, C_DIR_C): the partial sums are
    // registered from the complete window, their squares from the registered sums (120 16x16 products, kept in DSPs:
    // as LUT multipliers they were ~14k LUTs), the costs from the registered squares, and C_DIR picks the direction
    // from the registered costs. As one cone (window -> sums -> squares -> costs -> best -> strength) it was 21 ns.
    logic signed [15:0] part_r [0:7][0:14];
    always_ff @(posedge clk) for (int d = 0; d < 8; d++) for (int k = 0; k < 15; k++) part_r[d][k] <= part[d][k];
    (* use_dsp = "yes" *) logic signed [31:0] sq_r [0:7][0:14];     // |part| < 2^15, so the square fits 31 bits
    // only the partial sums the costs use are squared: d = 0, 4 all 15; d = 2, 6 the first 8; odd d the first 11
    always_ff @(posedge clk) for (int d = 0; d < 8; d++) for (int k = 0; k < 15; k++)
        if (d % 4 == 0 || (d % 2 == 0 && k < 8) || (d % 2 == 1 && k < 11)) sq_r[d][k] <= part_r[d][k] * part_r[d][k];
    function automatic logic signed [42:0] sq(input logic signed [31:0] q);
        sq = 43'(q);
    endfunction
    logic signed [42:0] cost [0:7];
    // The squares sharing the weight 105 are summed first and multiplied once (34 constant multiplies instead of
    // 120: the Div_Table shift-add trees on 31-bit squares were ~12k LUTs); integer arithmetic, so bit-identical.
    always_comb begin
        logic signed [42:0] s105 [0:7];
        for (int d = 0; d < 8; d++) begin cost[d] = 43'sd0; s105[d] = 43'sd0; end
        for (int i = 0; i < 8; i++) begin
            s105[2] = s105[2] + sq(sq_r[2][i]);
            s105[6] = s105[6] + sq(sq_r[6][i]);
        end
        for (int i = 0; i < 7; i++) begin
            cost[0] = cost[0] + (sq(sq_r[0][i]) + sq(sq_r[0][14 - i])) * 43'(signed'({33'b0, div_table(4'(i + 1))}));
            cost[4] = cost[4] + (sq(sq_r[4][i]) + sq(sq_r[4][14 - i])) * 43'(signed'({33'b0, div_table(4'(i + 1))}));
        end
        s105[0] = sq(sq_r[0][7]);
        s105[4] = sq(sq_r[4][7]);
        for (int o = 0; o < 4; o++) begin
            int d;
            d = 2 * o + 1;
            for (int j = 0; j < 5; j++) s105[d] = s105[d] + sq(sq_r[d][3 + j]);
            for (int j = 0; j < 3; j++) cost[d] = cost[d] + (sq(sq_r[d][j]) + sq(sq_r[d][10 - j])) * 43'(signed'({33'b0, div_table(4'(2 * j + 2))}));
        end
        for (int d = 0; d < 8; d++) cost[d] = cost[d] + s105[d] * 43'sd105;
    end
    logic signed [42:0] cost_r [0:7];
    always_ff @(posedge clk) for (int d = 0; d < 8; d++) cost_r[d] <= cost[d];
    logic [2:0]  best_dir_w, best_dir_c;
    logic signed [42:0] best_cost_w;
    logic [31:0] var_w, var_c;
    always_comb begin
        best_cost_w = 43'sd0; best_dir_w = 3'd0;
        for (int d = 0; d < 8; d++) if (cost_r[d] > best_cost_w) begin best_cost_w = cost_r[d]; best_dir_w = 3'(d); end
        var_w = 32'((best_cost_w - cost_r[(best_dir_w + 3'd4) & 3'd7]) >>> 10);
    end
    always_ff @(posedge clk) begin best_dir_c <= best_dir_w; var_c <= var_w; end   // third stage (C_DIR_C)
    logic [2:0]  ydir;
    logic [31:0] var_;

    // ---------------------------------------------------------------- filter (7.15.3): four samples per clock from the window
    // Pipelined (2026-10-25, for 100 MHz on the U200: the one-cycle function was 60+ logic levels, -6.4 ns):
    //   stage A  the group's neighbourhood (window rows pi .. pi+4, columns pj .. pj+8) and the block's filter parameters,
    //            registered in the FSM's C_PIX cycle (the window may be reloaded for the next plane right after);
    //   stage B  the 12 taps of each lane selected by direction (8-way muxes);
    //   stage C  constrain() of each tap's difference to the centre (the damping adjustment per threshold is a block
    //            constant from stage A) times its tap weight: 12 signed terms per lane; the min / max over the taps;
    //   stage D  three partial sums of four terms;
    //   stage E  the sum, rounding and clamp straight into the registered write port (dst4_*).
    // A write issued in C_PIX at cycle t lands at the port in t + 5; done / busy follow (dsr).
    logic [11:0] pri_str, sec_str;
    logic [4:0]  damping;
    logic [2:0]  dr;
    logic [2:0]  pi, pj;                                // sample in block
    function automatic logic [3:0] floor_log2_12(input logic [11:0] v);
        floor_log2_12 = 4'd0;
        for (int i = 11; i >= 0; i--) if (floor_log2_12 == 4'd0 && v[i]) floor_log2_12 = 4'(i);
    endfunction
    // the shift of constrain(): damping - FloorLog2(threshold), floored at 0; a constant per block and threshold
    function automatic logic [4:0] constrain_adj(input logic [11:0] thr, input logic [4:0] damp);
        constrain_adj = (damp > 5'(floor_log2_12(thr))) ? damp - 5'(floor_log2_12(thr)) : 5'd0;
    endfunction
    // constrain() with the shift precomputed (7.15.3)
    function automatic logic signed [12:0] constrain_s(input logic signed [12:0] diff, input logic [11:0] thr, input logic [4:0] adj);
        logic [12:0] ad;
        logic signed [13:0] v;
        if (thr == 12'd0) constrain_s = 13'sd0;
        else begin
            ad = (diff < 0) ? 13'(-diff) : 13'(diff);
            v = 14'(signed'({2'b0, thr})) - 14'(signed'({1'b0, ad >> adj}));
            if (v < 0) v = 14'sd0;
            if (v > 14'(signed'({1'b0, ad}))) v = 14'(signed'({1'b0, ad}));
            constrain_s = (diff < 0) ? -13'(v) : 13'(v);
        end
    endfunction
    logic pri_odd;
    assign pri_odd = pri_str[cs];                       // (priStr >> coeffShift) & 1

    // ---- stage A: neighbourhood of the four output samples (pi, pj .. pj+3): window rows pi .. pi+4, columns pj .. pj+8
    // (sample (pi, pj+l) sits at nbA[2][l+2]; taps reach +-2), with the block's parameters and the group's address.
    logic        vA;
    logic [11:0] nbA    [0:4][0:8];
    logic        nbokA  [0:4][0:8];
    logic [2:0]  drA; logic [11:0] priA, secA; logic [4:0] adjpA, adjsA; logic prioddA;
    logic [FBX-1:0] xA; logic [FBY-1:0] yA; logic [1:0] plA;
    always_ff @(posedge clk) begin
        vA <= (st == C_PIX) && !rst;
        for (int rr = 0; rr < 5; rr++)
            for (int cc = 0; cc < 9; cc++) begin
                nbA[rr][cc]   <= win[int'(pi) + rr][int'(pj) + cc];
                nbokA[rr][cc] <= win_ok[int'(pi) + rr][int'(pj) + cc];
            end
        drA <= dr; priA <= pri_str; secA <= sec_str; prioddA <= pri_odd;
        adjpA <= constrain_adj(pri_str, damping); adjsA <= constrain_adj(sec_str, damping);
        xA <= FBX'(px0 + PXW'(pj)); yA <= FBY'(py0 + PXW'(pi)); plA <= fplane;
    end

    // tap (kind, k, sign) of lane l for direction d, read from the neighbourhood: row 2 + dy, column l + 2 + dx.
    // kind 0 primary direction, 1 dir - 2, 2 dir + 2; k 0..1; sg 0 -, 1 +.
    function automatic int tap_dy(input int d, input int kind, input int k, input int sg);
        logic [2:0] tdir;
        tdir = (kind == 0) ? 3'(d) : (kind == 1) ? 3'(d - 2) : 3'(d + 2);
        tap_dy = (sg == 1) ? int'(cdef_dy(tdir, 1'(k))) : -int'(cdef_dy(tdir, 1'(k)));
    endfunction
    function automatic int tap_dx(input int d, input int kind, input int k, input int sg);
        logic [2:0] tdir;
        tdir = (kind == 0) ? 3'(d) : (kind == 1) ? 3'(d - 2) : 3'(d + 2);
        tap_dx = (sg == 1) ? int'(cdef_dx(tdir, 1'(k))) : -int'(cdef_dx(tdir, 1'(k)));
    endfunction
    function automatic logic [11:0] tap_at(input int d, input int kind, input int k, input int sg, input int l);
        tap_at = nbA[2 + tap_dy(d, kind, k, sg)][l + 2 + tap_dx(d, kind, k, sg)];
    endfunction
    function automatic logic tap_ok(input int d, input int kind, input int k, input int sg, input int l);
        tap_ok = nbokA[2 + tap_dy(d, kind, k, sg)][l + 2 + tap_dx(d, kind, k, sg)];
    endfunction
    // tap index: kind * 4 + k * 2 + sg
    function automatic int tap_kind(input int ti); tap_kind = ti / 4; endfunction
    function automatic int tap_k(input int ti); tap_k = (ti / 2) % 2; endfunction
    function automatic int tap_sg(input int ti); tap_sg = ti % 2; endfunction

    // ---- stage B: the 12 taps of each lane selected by direction (8-way muxes), with their presence
    logic        vB;
    logic [11:0] tB  [0:3][0:11];
    logic        okB [0:3][0:11];
    logic [11:0] tmaxB [0:3][0:11], tminB [0:3][0:11];   // the tap for the max / min trees: an absent tap does not count
    logic [11:0] ctrB [0:3];
    logic [11:0] priB, secB; logic [4:0] adjpB, adjsB; logic prioddB;
    logic [FBX-1:0] xB; logic [FBY-1:0] yB; logic [1:0] plB;
    logic [11:0] tB_c  [0:3][0:11];
    logic        okB_c [0:3][0:11];
    always_comb
        for (int l = 0; l < 4; l++)
            for (int ti = 0; ti < 12; ti++) begin
                logic [11:0] t; logic ok;
                t = '0; ok = 1'b0;
                case (drA)                                  // constant offsets in every arm: an 8-way mux per tap
                    3'd0: begin t = tap_at(0, tap_kind(ti), tap_k(ti), tap_sg(ti), l); ok = tap_ok(0, tap_kind(ti), tap_k(ti), tap_sg(ti), l); end
                    3'd1: begin t = tap_at(1, tap_kind(ti), tap_k(ti), tap_sg(ti), l); ok = tap_ok(1, tap_kind(ti), tap_k(ti), tap_sg(ti), l); end
                    3'd2: begin t = tap_at(2, tap_kind(ti), tap_k(ti), tap_sg(ti), l); ok = tap_ok(2, tap_kind(ti), tap_k(ti), tap_sg(ti), l); end
                    3'd3: begin t = tap_at(3, tap_kind(ti), tap_k(ti), tap_sg(ti), l); ok = tap_ok(3, tap_kind(ti), tap_k(ti), tap_sg(ti), l); end
                    3'd4: begin t = tap_at(4, tap_kind(ti), tap_k(ti), tap_sg(ti), l); ok = tap_ok(4, tap_kind(ti), tap_k(ti), tap_sg(ti), l); end
                    3'd5: begin t = tap_at(5, tap_kind(ti), tap_k(ti), tap_sg(ti), l); ok = tap_ok(5, tap_kind(ti), tap_k(ti), tap_sg(ti), l); end
                    3'd6: begin t = tap_at(6, tap_kind(ti), tap_k(ti), tap_sg(ti), l); ok = tap_ok(6, tap_kind(ti), tap_k(ti), tap_sg(ti), l); end
                    default: begin t = tap_at(7, tap_kind(ti), tap_k(ti), tap_sg(ti), l); ok = tap_ok(7, tap_kind(ti), tap_k(ti), tap_sg(ti), l); end
                endcase
                tB_c[l][ti] = t; okB_c[l][ti] = ok;
            end
    always_ff @(posedge clk) begin
        vB <= vA && !rst;
        for (int l = 0; l < 4; l++) begin
            ctrB[l] <= nbA[2][l + 2];
            for (int ti = 0; ti < 12; ti++) begin
                tB[l][ti] <= tB_c[l][ti]; okB[l][ti] <= okB_c[l][ti];
                tmaxB[l][ti] <= okB_c[l][ti] ? tB_c[l][ti] : 12'd0;
                tminB[l][ti] <= okB_c[l][ti] ? tB_c[l][ti] : 12'hFFF;
            end
        end
        priB <= priA; secB <= secA; adjpB <= adjpA; adjsB <= adjsA; prioddB <= prioddA;
        xB <= xA; yB <= yA; plB <= plA;
    end

    // ---- stage C: constrain() of every present tap's difference to the centre, times its weight (Cdef_Pri_Taps /
    // Cdef_Sec_Taps: 1 .. 4); the min / max over the present taps and the centre
    logic        vC;
    logic signed [15:0] termC [0:3][0:11];
    logic [11:0] ctrC [0:3], mnC [0:3], mxC [0:3];
    logic [FBX-1:0] xC; logic [FBY-1:0] yC; logic [1:0] plC;
    logic signed [15:0] termC_c [0:3][0:11];
    logic [11:0] mnC_c [0:3], mxC_c [0:3];
    // max / min of the 12 taps and the centre as balanced trees (a sequential if-chain synthesised 12 deep: -4.6 ns)
    function automatic logic [11:0] max13(input logic [11:0] v [0:11], input logic [11:0] ctr);
        logic [11:0] l1 [0:5]; logic [11:0] l2 [0:2]; logic [11:0] a, b;
        for (int k = 0; k < 6; k++) l1[k] = (v[2 * k] > v[2 * k + 1]) ? v[2 * k] : v[2 * k + 1];
        for (int k = 0; k < 3; k++) l2[k] = (l1[2 * k] > l1[2 * k + 1]) ? l1[2 * k] : l1[2 * k + 1];
        a = (l2[0] > l2[1]) ? l2[0] : l2[1];
        b = (l2[2] > ctr) ? l2[2] : ctr;
        max13 = (a > b) ? a : b;
    endfunction
    function automatic logic [11:0] min13(input logic [11:0] v [0:11], input logic [11:0] ctr);
        logic [11:0] l1 [0:5]; logic [11:0] l2 [0:2]; logic [11:0] a, b;
        for (int k = 0; k < 6; k++) l1[k] = (v[2 * k] < v[2 * k + 1]) ? v[2 * k] : v[2 * k + 1];
        for (int k = 0; k < 3; k++) l2[k] = (l1[2 * k] < l1[2 * k + 1]) ? l1[2 * k] : l1[2 * k + 1];
        a = (l2[0] < l2[1]) ? l2[0] : l2[1];
        b = (l2[2] < ctr) ? l2[2] : ctr;
        min13 = (a < b) ? a : b;
    endfunction
    always_comb
        for (int l = 0; l < 4; l++) begin
            for (int ti = 0; ti < 12; ti++) begin
                logic signed [12:0] d, cv; logic [2:0] w;
                d = 13'(signed'({1'b0, tB[l][ti]})) - 13'(signed'({1'b0, ctrB[l]}));
                cv = okB[l][ti] ? ((tap_kind(ti) == 0) ? constrain_s(d, priB, adjpB) : constrain_s(d, secB, adjsB)) : 13'sd0;
                w = (tap_kind(ti) == 0) ? pri_tap(prioddB, 1'(tap_k(ti))) : sec_tap(prioddB, 1'(tap_k(ti)));
                case (w)
                    3'd1: termC_c[l][ti] = 16'(cv);
                    3'd2: termC_c[l][ti] = 16'(cv) <<< 1;
                    3'd3: termC_c[l][ti] = (16'(cv) <<< 1) + 16'(cv);
                    default: termC_c[l][ti] = 16'(cv) <<< 2;
                endcase
            end
            mxC_c[l] = max13(tmaxB[l], ctrB[l]); mnC_c[l] = min13(tminB[l], ctrB[l]);
        end
    always_ff @(posedge clk) begin
        vC <= vB && !rst;
        for (int l = 0; l < 4; l++) begin
            ctrC[l] <= ctrB[l]; mnC[l] <= mnC_c[l]; mxC[l] <= mxC_c[l];
            for (int ti = 0; ti < 12; ti++) termC[l][ti] <= termC_c[l][ti];
        end
        xC <= xB; yC <= yB; plC <= plB;
    end

    // ---- stage D: three partial sums of four terms each per lane
    logic        vD;
    logic signed [17:0] partD [0:3][0:2];
    logic [11:0] ctrD [0:3], mnD [0:3], mxD [0:3];
    logic [FBX-1:0] xD; logic [FBY-1:0] yD; logic [1:0] plD;
    always_ff @(posedge clk) begin
        vD <= vC && !rst;
        for (int l = 0; l < 4; l++) begin
            for (int g = 0; g < 3; g++)
                partD[l][g] <= 18'(termC[l][4 * g]) + 18'(termC[l][4 * g + 1]) + 18'(termC[l][4 * g + 2]) + 18'(termC[l][4 * g + 3]);
            ctrD[l] <= ctrC[l]; mnD[l] <= mnC[l]; mxD[l] <= mxC[l];
        end
        xD <= xC; yD <= yC; plD <= plC;
    end

    // ---- stage E: the sum, rounding and the clamp to the taps' range (into the port register below)
    function automatic logic [11:0] cdef_out(input int l);
        logic signed [17:0] total;
        logic signed [18:0] v;
        total = partD[l][0] + partD[l][1] + partD[l][2];
        v = 19'(signed'({7'b0, ctrD[l]})) + ((19'sd8 + 19'(total) - (total < 0 ? 19'sd1 : 19'sd0)) >>> 4);
        if (v < 19'(signed'({7'b0, mnD[l]}))) cdef_out = mnD[l];
        else if (v > 19'(signed'({7'b0, mxD[l]}))) cdef_out = mxD[l];
        else cdef_out = 12'(v);
    endfunction
    logic [47:0] out4;                                 // samples (pi, pj .. pj+3) of the group in stage D
    always_comb for (int l = 0; l < 4; l++) out4[l * 12 +: 12] = cdef_out(l);
    // copy path
    logic [5:0] cpi;

    // ---------------------------------------------------------------- memory ports
    // The write port is registered. A filtered group issued in C_PIX at cycle t reaches the port at t + 5 (stages A .. D,
    // then this register); the copy path (unfiltered blocks) writes from C_COPY_WR directly. The two never overlap: a block
    // is either filtered or copied, and the pipeline has drained (4 cycles) long before the next block's first copy write
    // (C_NEXT, C_BLK, the cdef index and skip reads: 7 cycles). done and busy wait for the last write to land.
    logic        dst_we_c, dst4_we_c; logic [1:0] dst_plane_c, dst4_plane_c; logic [FBX-1:0] dst_x_c, dst4_x_c; logic [FBY-1:0] dst_y_c, dst4_y_c;
    logic [11:0] dst_wdata_c; logic [47:0] dst4_wdata_c; logic done_i, busy_q; logic [3:0] dsr;
    always_comb begin
        src_re = 1'b0; src_plane = fplane; src_x = FBX'(wx); src_y = FBY'(wy);
        dst_we_c = 1'b0; dst_plane_c = fplane; dst_x_c = FBX'(px0 + PXW'(pj)); dst_y_c = FBY'(py0 + PXW'(pi)); dst_wdata_c = 12'd0;
        dst4_we_c = 1'b0; dst4_plane_c = fplane; dst4_x_c = FBX'(px0 + PXW'(cpi[2:0])); dst4_y_c = FBY'(py0 + PXW'(cpi[5:3])); dst4_wdata_c = src_rdata4;
        case (st)
            C_WIN: src_re = w_row_in;                    // absent columns of a present row are masked at landing
            C_COPY_RD: begin src_re = 1'b1; src_x = FBX'(px0 + PXW'(cpi[2:0])); src_y = FBY'(py0 + PXW'(cpi[5:3])); end
            C_COPY_WR: dst4_we_c = 1'b1;
            default: ;
        endcase
        if (vD) begin dst4_we_c = 1'b1; dst4_plane_c = plD; dst4_x_c = xD; dst4_y_c = yD; dst4_wdata_c = out4; end
    end
    always_ff @(posedge clk) begin
        if (rst) begin dst_we <= 1'b0; dst4_we <= 1'b0; done <= 1'b0; busy_q <= 1'b0; dsr <= 4'd0; end
        else begin
            dst_we <= dst_we_c; dst4_we <= dst4_we_c;
            dsr <= {dsr[2:0], done_i};                   // done_i is set in C_NEXT, one cycle after the last C_PIX: the last
            done <= dsr[3];                              // write lands 4 cycles after that; done follows it
            busy_q <= (st != C_IDLE) || vA || vB || vC || vD;
        end
        dst_plane <= dst_plane_c; dst_x <= dst_x_c; dst_y <= dst_y_c; dst_wdata <= dst_wdata_c;
        dst4_plane <= dst4_plane_c; dst4_x <= dst4_x_c; dst4_y <= dst4_y_c; dst4_wdata <= dst4_wdata_c;
    end
    assign busy = (st != C_IDLE) || vA || vB || vC || vD || busy_q || (dsr != 4'd0) || done;

    // ---------------------------------------------------------------- FSM
    always_ff @(posedge clk) begin
        done_i <= 1'b0;
        if (wpend)                                       // the group issued last cycle lands: lane l -> column 4 wpg - 2 + l
            for (int l = 0; l < 4; l++) begin
                int col;
                col = 4 * int'(wpg) - 2 + l;
                if (col >= 0 && col < int'(pw) + 4) begin
                    win[wpr][col] <= src_rdata4[l * 12 +: 12];
                    win_ok[wpr][col] <= wp_rowok && (wpx + PXS'(l) >= 0) && (wpx + PXS'(l) < PXS'(signed'({2'b0, fw})));
                end
            end
        wpend <= 1'b0;
        if (rst) st <= C_IDLE;
        else case (st)
            C_IDLE: if (start) begin r <= row_first; c <= MIW'(0); st <= C_BLK; end
            C_BLK: st <= C_RD_IDX;                       // cdr_* addressed; value lands next cycle
            C_RD_IDX: begin cidx <= cdr_val; st <= C_RD_S0; end
            C_RD_S0: st <= C_RD_S1;
            C_RD_S1: begin skips[0] <= rd_info.skip; st <= C_RD_S2; end
            C_RD_S2: begin skips[1] <= rd_info.skip; st <= C_RD_S3; end
            C_RD_S3: begin skips[2] <= rd_info.skip; st <= C_DECIDE; end
            C_DECIDE: begin
                skips[3] <= rd_info.skip;
                fplane <= 2'd0;
                if (!cidx[3] || (skips[0] && skips[1] && skips[2] && rd_info.skip)) begin cpi <= 6'd0; st <= C_COPY_RD; end
                else st <= C_PLANE;
            end
            // ---- per plane: load the window (one read per clock, lands next cycle)
            C_PLANE: begin wr <= 4'd0; wg <= 2'd0; st <= C_WIN; end
            C_WIN: begin
                wpend <= 1'b1; wpr <= wr; wpg <= wg; wp_rowok <= w_row_in; wpx <= wx;
                if (wg != wg_last) wg <= wg + 2'd1;
                else begin
                    wg <= 2'd0;
                    if (wr + 4'd1 < ph + 4'd4) wr <= wr + 4'd1;
                    else st <= C_WIN_LAST;
                end
            end
            C_WIN_LAST: st <= (fplane == 2'd0) ? C_DIR_A : C_DIR;   // the last read lands here; luma searches
            C_DIR_A: st <= C_DIR_B;                      // partial sums registered from the complete window
            C_DIR_B: st <= C_DIR_B2;                     // squares registered
            C_DIR_B2: st <= C_DIR_C;                     // costs registered
            C_DIR_C: st <= C_DIR;                        // best direction and variance registered
            // ---- direction (luma) and strengths, one cycle
            C_DIR: begin
                logic [5:0] sv; logic [3:0] pri, sec; logic [11:0] ps; logic [3:0] var_str; logic [31:0] v6;
                logic [2:0] yd; logic [31:0] vr;
                yd = (fplane == 2'd0) ? best_dir_c : ydir;
                vr = (fplane == 2'd0) ? var_c : var_;
                if (fplane == 2'd0) begin ydir <= best_dir_c; var_ <= var_c; end
                sv = (fplane == 2'd0) ? ch.y_str[6 * cidx[2:0] +: 6] : ch.uv_str[6 * cidx[2:0] +: 6];
                pri = 4'(sv >> 2); sec = 4'(sv[1:0]) + ((sv[1:0] == 2'd3) ? 4'd1 : 4'd0);
                ps = 12'(pri) << cs;
                if (fplane == 2'd0) begin
                    v6 = vr >> 6;
                    var_str = 4'd0;
                    if (v6 != 32'd0) begin
                        for (int b = 31; b >= 0; b--) if (var_str == 4'd0 && v6[b]) var_str = (b > 12) ? 4'd12 : 4'(b);
                    end
                    dr <= (ps == 12'd0) ? 3'd0 : yd;
                    pri_str <= (vr != 32'd0) ? 12'((16'(ps) * (16'd4 + 16'(var_str)) + 16'd8) >> 4) : 12'd0;
                    damping <= 5'(ch.damping) + 5'(cs);
                end else begin
                    dr <= (ps == 12'd0) ? 3'd0 : cdef_uv_dir(hdr.ssx, hdr.ssy, yd);
                    pri_str <= ps;
                    damping <= 5'(ch.damping) + 5'(cs) - 5'd1;
                end
                sec_str <= 12'(sec) << cs;
                pi <= 3'd0; pj <= 3'd0;
                st <= C_PIX;
            end
            // ---- one filtered sample per clock
            C_PIX: begin                                 // four filtered samples per clock (an aligned group of the row)
                if (4'(pj) + 4'd4 < pw) pj <= pj + 3'd4;
                else begin
                    pj <= 3'd0;
                    if (4'(pi) + 4'd1 < ph) pi <= pi + 3'd1;
                    else begin
                        if (fplane == 2'd0 && !hdr.mono) begin fplane <= 2'd1; st <= C_PLANE; end
                        else if (fplane == 2'd1) begin fplane <= 2'd2; st <= C_PLANE; end
                        else st <= C_NEXT;
                    end
                end
            end
            // ---- unfiltered block: copy src -> dst for all planes
            C_COPY_RD: st <= C_COPY_WR;
            C_COPY_WR: begin
                if (4'(cpi[2:0]) + 4'd4 < pw) begin cpi <= cpi + 6'd4; st <= C_COPY_RD; end
                else if (4'(cpi[5:3]) + 4'd1 < ph) begin cpi <= {cpi[5:3] + 3'd1, 3'd0}; st <= C_COPY_RD; end
                else begin
                    cpi <= 6'd0;
                    if (fplane == 2'd0 && !hdr.mono) begin fplane <= 2'd1; st <= C_COPY_RD; end
                    else if (fplane == 2'd1) begin fplane <= 2'd2; st <= C_COPY_RD; end
                    else st <= C_NEXT;
                end
            end
            C_NEXT: begin
                if (c + MIW'(2) < hdr.mi_cols) begin c <= c + MIW'(2); st <= C_BLK; end
                else begin
                    c <= MIW'(0);
                    if (r + MIW'(2) <= row_last && r + MIW'(2) < hdr.mi_rows) begin r <= r + MIW'(2); st <= C_BLK; end
                    else begin done_i <= 1'b1; st <= C_IDLE; end
                end
            end
            default: st <= C_IDLE;
        endcase
    end
endmodule
