// Intra reconstruction stage: consumes the syntax decoder's block records (blk_info / blk_rec), transform-block
// records (tx_done / tx_rec + Quant through q_addr/q_data, acknowledged with tx_ack) and palette colour maps
// (pm_*), and produces the pre-loop-filter picture in the frame buffer (fb_*), following spec 7.11 / 7.12:
//   per transform block, in order:  prediction  ->  dequant -> inverse transform -> add  ->  BlockDecoded update
//     prediction = palette (7.11.4) | edge preparation (7.11.2.1, reads the frame buffer) + ipred (7.11.2)
//                  [+ CfL (7.11.5): load the reconstructed luma region and the DC prediction, run cfl]
// Slow-and-correct first version: one pixel (or one coefficient) per cycle or two, no overlap between stages.
// Events arrive through rec_fifo in decode order (superblock start, block record, transform blocks); a
// transform block is popped only once it is fully written, so its coefficient slot stays valid until then.
//
// State kept here: BlockDecoded flags per plane for the current superblock (cleared on sb_start, 5.11.3),
// MaxLumaW/H, and a per-4x4 map of {uvmode, ymode} for the tile (get_filter_type needs the neighbours'
// modes with the chroma position adjustments of 7.11.2.8; a frame-wide map is the simple, obviously right
// choice — the loop filter will need per-4x4 frame state anyway).
module recon_top
  import blk_tables_pkg::*;
  import tx_tables_pkg::*;
  import q_tables_pkg::*;
  import syn_pkg::*;
#(
    parameter int FBX = 9,                   // frame buffer x address bits
    parameter int FBY = 9,
    parameter int ML2R = 8,                  // mode map capacity: log2(MI rows), log2(MI cols)
    parameter int ML2C = 8
) (
    input  logic        clk,
    input  logic        rst,
    input  hdr_t        hdr,
    input  rec_hdr_t    rh,
    // from the event queue (rec_fifo): head event, popped when consumed
    input  logic        ev_valid,
    input  logic [1:0]  ev_kind,              // 0 transform block, 1 block record, 2 superblock start
    input  logic [10:0] ev_sb_r, ev_sb_c,
    input  blk_rec_t    ev_blk,
    input  tx_rec_t     ev_tx,
    output logic        ev_pop,
    output logic [2:0]  q_slot,
    output logic [9:0]  q_addr,
    input  logic signed [20:0] q_data,
    output logic        pm_plane,
    output logic [5:0]  pm_x, pm_y,
    input  logic [2:0]  pm_idx,
    output logic        busy,
    // per-4x4 state for the in-loop filters (mi_store write ports)
    output logic        mi_blk_we,
    output logic [10:0] mi_blk_r, mi_blk_c,
    output logic [5:0]  mi_blk_bw4, mi_blk_bh4,
    output mi_lf_t      mi_blk_data,
    input  logic        mi_blk_busy,
    output logic        mi_tx_we,
    output logic [1:0]  mi_tx_plane,
    output logic [10:0] mi_tx_row, mi_tx_col,
    output logic [4:0]  mi_tx_w4, mi_tx_h4,
    output logic [4:0]  mi_tx_sz,
    input  logic        mi_tx_busy,
    output logic        mi_cd_clr,             // superblock start: clear the superblock's cdef_idx unit(s)
    output logic        mi_cd_we,              // block with a coded cdef_idx: set the covered unit(s)
    output logic [6:0]  mi_cd_row64, mi_cd_col64,
    output logic [3:0]  mi_cd_mask,
    output logic [2:0]  mi_cd_idx,
    // frame buffer (single port, registered read)
    output logic        fb_re,
    output logic        fb_we,
    output logic [1:0]  fb_plane,
    output logic [FBX-1:0] fb_x,
    output logic [FBY-1:0] fb_y,
    output logic [11:0] fb_wdata,
    input  logic [11:0] fb_rdata,
    input  logic [47:0] fb_rdata4,            // aligned group of 4 samples containing fb_x (with fb_re)
    // frame buffer write port (all of this stage's writes; fb_we stays 0)
    output logic        fb2_we,
    output logic [1:0]  fb2_plane,
    output logic [FBX-1:0] fb2_x,
    output logic [FBY-1:0] fb2_y,
    output logic [11:0] fb2_wdata,
    // frame buffer 4-wide write port (prediction and residual-add output, aligned groups of 4)
    output logic        fb4_we,
    output logic [1:0]  fb4_plane,
    output logic [FBX-1:0] fb4_x,
    output logic [FBY-1:0] fb4_y,
    output logic [47:0] fb4_wdata
);
    localparam logic [3:0] DC_PRED = 4'd0, UV_CFL_PRED = 4'd13, SMOOTH_PRED = 4'd9, SMOOTH_H_PRED = 4'd11;
    localparam logic [3:0] IDTX = 4'd9;
    localparam int TW = 22;

    function automatic logic is_inside(input logic [10:0] r, input logic [10:0] c);
        is_inside = ($signed({1'b0, c}) >= $signed({1'b0, hdr.mi_col_start})) && (c < hdr.mi_col_end) &&
                    ($signed({1'b0, r}) >= $signed({1'b0, hdr.mi_row_start})) && (r < hdr.mi_row_end);
    endfunction
    function automatic logic is_smooth(input logic [3:0] m);
        is_smooth = (m >= SMOOTH_PRED) && (m <= SMOOTH_H_PRED);
    endfunction
    function automatic logic is_dir(input logic [3:0] m);
        is_dir = (m >= 4'd1) && (m <= 4'd8);
    endfunction
    function automatic logic [7:0] clip255(input logic signed [10:0] v);
        clip255 = (v < 0) ? 8'd0 : (v > 11'sd255) ? 8'd255 : 8'(v);
    endfunction

    // ================================================================ block state (latched on blk_info)
    blk_rec_t    b;
    logic        blk_ready;
    logic [5:0]  bw4, bh4;
    logic        av_u, av_l, av_uc, av_lc;
    logic        ft_y, ft_uv;                              // intra filter type per plane class
    always_comb begin
        bw4 = num4x4w(b.bsize); bh4 = num4x4h(b.bsize);
        av_u = is_inside(b.r - 11'd1, b.c);
        av_l = is_inside(b.r, b.c - 11'd1);
        av_uc = 1'b0; av_lc = 1'b0;
        if (b.has_chroma) begin
            av_uc = av_u; av_lc = av_l;
            if (hdr.ssy && bh4 == 6'd1) av_uc = is_inside(b.r - 11'd2, b.c);
            if (hdr.ssx && bw4 == 6'd1) av_lc = is_inside(b.r, b.c - 11'd2);
        end
    end

    // ---------------------------------------------------------------- mode map (per 4x4: {uvmode, ymode})
    localparam int MWORDS = (1 << ML2R) * (1 << (ML2C - 4));
    logic [127:0] mi_map [0:MWORDS-1];
    logic [ML2R+ML2C-5:0] mm_addr, mm_waddr;             // read address (combinational per state), write address
    logic [127:0] mm_rdata;                                // = mi_map[mm_addr of the previous cycle]
    logic         mm_we;
    logic [127:0] mm_wdata;
    always_ff @(posedge clk) begin
        if (mm_we) mi_map[mm_waddr] <= mm_wdata;
        mm_rdata <= mi_map[mm_addr];
    end
    function automatic logic [ML2R+ML2C-5:0] mm_index(input logic [10:0] row, input logic [10:0] col);
        mm_index = {row[ML2R-1:0], col[ML2C-1:4]};
    endfunction
    function automatic logic [127:0] mm_merge(input logic [127:0] w, input logic [3:0] start, input logic [5:0] n, input logic [7:0] e);
        mm_merge = w;
        for (int k = 0; k < 16; k++)
            if (6'(k) >= 6'(start) && 6'(k) < 6'(start) + n) mm_merge[8 * k +: 8] = e;
    endfunction

    typedef enum logic [3:0] {B_IDLE, B_FT0, B_FT1, B_FT2, B_FT3, B_FT4, B_WR, B_WR_W,
                              B_IBC_INIT, B_IBC_PL, B_IBC_R0, B_IBC_R1, B_IBC_R2, B_IBC_R3, B_IBC_WR} bst_t;
    bst_t bst;
    // event queue consumption: a superblock start is applied when both FSMs are idle, a block record is taken by
    // the block FSM when idle, a transform block is taken by the transform FSM (and popped when acknowledged)
    // ---- intra block copy (7.11.3 with refIdx = -1): the block is predicted whole from the pre-filter current
    // frame before its transform blocks add residuals. someUseIntra is always 1 for an intrabc block (its own
    // RefFrames[0] is INTRA_FRAME), so each plane is one prediction of the plane residual size with this
    // block's vector. Luma vectors are whole samples; chroma may be half-sample -> bilinear (Subpel_Filters
    // BILINEAR: taps 64/64) through the spec's rounding chain (InterRound0 / InterRound1).
    logic [1:0]  ib_pl;
    logic        ibc_sx, ibc_sy;
    logic [7:0]  ib_w, ib_h, ib_i, ib_j;
    logic [12:0] ib_bx, ib_by, ib_lastx, ib_lasty;
    logic [3:0]  ib_fx, ib_fy;
    logic signed [14:0] ib_x0, ib_y0;
    logic [11:0] ib_s0, ib_s1, ib_s2;
    logic [4:0]  ib_r0, ib_r1;
    assign ibc_sx = (ib_pl != 2'd0) && hdr.ssx;
    assign ibc_sy = (ib_pl != 2'd0) && hdr.ssy;
    assign ib_r0 = (hdr.bit_depth == 4'd12) ? 5'd5 : 5'd3;
    assign ib_r1 = (hdr.bit_depth == 4'd12) ? 5'd9 : 5'd11;
    function automatic logic [12:0] ib_clip(input logic signed [14:0] v, input logic [12:0] last);
        if (v < 15'sd0) ib_clip = 13'd0;
        else if (v > 15'(signed'({2'b0, last}))) ib_clip = last;
        else ib_clip = 13'(v);
    endfunction
    logic [12:0] ib_rx0, ib_rx1, ib_ry0, ib_ry1;          // clamped source coordinates of the 2x2 neighbourhood
    assign ib_rx0 = ib_clip(ib_x0 + 15'(signed'({7'b0, ib_j})), ib_lastx);
    assign ib_rx1 = ib_clip(ib_x0 + 15'(signed'({7'b0, ib_j})) + 15'sd1, ib_lastx);
    assign ib_ry0 = ib_clip(ib_y0 + 15'(signed'({7'b0, ib_i})), ib_lasty);
    assign ib_ry1 = ib_clip(ib_y0 + 15'(signed'({7'b0, ib_i})) + 15'sd1, ib_lasty);
    logic [11:0] ib_out;
    always_comb begin
        logic [19:0] a0, a1;
        logic [16:0] h0, h1;
        logic [24:0] v;
        logic [13:0] o;
        a0 = ib_fx[3] ? 20'd64 * 20'(ib_s0) + 20'd64 * 20'(ib_s1) : 20'd128 * 20'(ib_s0);
        a1 = ib_fx[3] ? 20'd64 * 20'(ib_s2) + 20'd64 * 20'(fb_rdata) : 20'd128 * 20'(ib_s2);
        h0 = 17'((a0 + (20'd1 << (ib_r0 - 5'd1))) >> ib_r0);
        h1 = 17'((a1 + (20'd1 << (ib_r0 - 5'd1))) >> ib_r0);
        v = ib_fy[3] ? 25'd64 * 25'(h0) + 25'd64 * 25'(h1) : 25'd128 * 25'(h0);
        o = 14'((v + (25'd1 << (ib_r1 - 5'd1))) >> ib_r1);
        ib_out = (o > 14'(pix_max)) ? pix_max : 12'(o);
    end
    logic [7:0]  nb_ay, nb_ly, nb_auv, nb_luv;
    logic [5:0]  wr_i;                                     // row counter for the map write
    logic        wr_j;                                     // word half
    logic [10:0] a_r_uv, a_c_uv, l_r_uv, l_c_uv;           // 7.11.2.8 chroma neighbour positions
    always_comb begin
        a_r_uv = b.r - 11'd1; a_c_uv = b.c;
        if (hdr.ssx && !b.c[0]) a_c_uv = b.c + 11'd1;
        if (hdr.ssy && b.r[0]) a_r_uv = b.r - 11'd2;
        l_r_uv = b.r; l_c_uv = b.c - 11'd1;
        if (hdr.ssx && b.c[0]) l_c_uv = b.c - 11'd2;
        if (hdr.ssy && !b.r[0]) l_r_uv = b.r + 11'd1;
    end
    logic [5:0] wr_n;
    assign wr_n = (bw4 > 6'd16) ? 6'd16 : bw4;
    // read address by state: each state's read lands in mm_rdata during the next state
    always_comb begin
        case (bst)
            B_FT0: mm_addr = mm_index(b.r - 11'd1, b.c);
            B_FT1: mm_addr = mm_index(b.r, b.c - 11'd1);
            B_FT2: mm_addr = mm_index(a_r_uv, a_c_uv);
            B_FT3: mm_addr = mm_index(l_r_uv, l_c_uv);
            B_FT4: mm_addr = mm_index(b.r, b.c);
            B_WR_W: mm_addr = (bw4 > 6'd16 && !wr_j) ? mm_index(b.r + 11'(wr_i), b.c + 11'd16) : mm_index(b.r + 11'(wr_i) + 11'd1, b.c);
            default: mm_addr = mm_index(b.r + 11'(wr_i), b.c + (wr_j ? 11'd16 : 11'd0));   // B_WR: the word being merged
        endcase
    end

    assign mi_blk_r = b.r; assign mi_blk_c = b.c; assign mi_blk_bw4 = bw4; assign mi_blk_bh4 = bh4;
    // cdef_idx bookkeeping: clear on sb_start (uses sb_r/sb_c), set with the block record (superblock-relative units)
    logic [10:0] sb_mask;
    assign sb_mask = hdr.sb128 ? ~11'd31 : ~11'd15;
    assign mi_cd_clr = sb_start;
    assign mi_cd_row64 = sb_start ? 7'(sb_r >> 4) : 7'((b.r & sb_mask) >> 4);
    assign mi_cd_col64 = sb_start ? 7'(sb_c >> 4) : 7'((b.c & sb_mask) >> 4);
    assign mi_cd_mask = hdr.sb128 ? b.cdef_units : 4'b0001;
    assign mi_cd_idx = b.cdef_idx;
    assign mi_blk_data = '{bsize: b.bsize, skip: b.skip, seg: b.seg, delta_lf: b.delta_lf};
    always_ff @(posedge clk) begin
        mm_we <= 1'b0; mi_blk_we <= 1'b0; mi_cd_we <= 1'b0;
        if (rst) begin
            bst <= B_IDLE; blk_ready <= 1'b0;
        end else case (bst)
            B_IDLE: if (blk_take) begin
                b <= ev_blk; blk_ready <= 1'b0; bst <= B_FT0;
            end
            // four neighbour-mode reads (issued by the combinational mm_addr; captured in the following state):
            // above Y, left Y, above UV, left UV
            B_FT0: if (!mi_blk_busy && !mi_blk_we) begin mi_blk_we <= 1'b1; mi_cd_we <= b.cdef_valid; bst <= B_FT1; end   // per-4x4 LF state + cdef_idx
            B_FT1: begin nb_ay <= mm_rdata[8 * b.c[3:0] +: 8]; bst <= B_FT2; end
            B_FT2: begin nb_ly <= mm_rdata[8 * 4'(b.c - 11'd1) +: 8]; bst <= B_FT3; end
            B_FT3: begin nb_auv <= mm_rdata[8 * a_c_uv[3:0] +: 8]; bst <= B_FT4; end
            B_FT4: begin
                nb_luv <= mm_rdata[8 * l_c_uv[3:0] +: 8];
                ft_y <= (av_u && is_smooth(nb_ay[3:0])) || (av_l && is_smooth(nb_ly[3:0]));
                wr_i <= 6'd0; wr_j <= 1'b0;
                bst <= B_WR;                             // mm_addr = first word to merge into; read lands in B_WR
            end
            B_WR: begin
                if (wr_i == 6'd0 && !wr_j) ft_uv <= (av_uc && is_smooth(nb_auv[7:4])) || (av_lc && is_smooth(nb_luv[7:4]));
                mm_we <= 1'b1; mm_waddr <= mm_addr;
                mm_wdata <= mm_merge(mm_rdata, wr_j ? 4'd0 : b.c[3:0], wr_n, {b.uvmode, b.ymode});
                bst <= B_WR_W;
            end
            B_WR_W: begin
                // advance: second word of a >16-wide block, next row, or done (the next word's read is in flight)
                if (bw4 > 6'd16 && !wr_j) begin
                    wr_j <= 1'b1; bst <= B_WR;
                end else if (wr_i + 6'd1 < bh4) begin
                    wr_j <= 1'b0; wr_i <= wr_i + 6'd1; bst <= B_WR;
                end else if (b.is_inter) begin
                    bst <= B_IBC_INIT;
                end else begin
                    blk_ready <= 1'b1; bst <= B_IDLE;
                end
            end
            // ---- intra block copy prediction, one plane at a time, one output sample per 4-5 cycles
            B_IBC_INIT: if (rs == R_IDLE) begin ib_pl <= 2'd0; bst <= B_IBC_PL; end
            B_IBC_PL: begin
                logic [4:0] psz;
                logic signed [19:0] mvx2, mvy2;
                psz = subsampled_size(b.bsize, ibc_sx, ibc_sy);
                ib_w <= 8'(num4x4w(psz)) << 2; ib_h <= 8'(num4x4h(psz)) << 2;
                ib_bx <= 13'((13'(b.c) >> ibc_sx) << 2); ib_by <= 13'((13'(b.r) >> ibc_sy) << 2);
                mvx2 = (20'(b.mv_col) * 20'sd2) >>> ibc_sx;                   // (2 * mv[1]) >> subX
                mvy2 = (20'(b.mv_row) * 20'sd2) >>> ibc_sy;
                ib_fx <= mvx2[3:0]; ib_fy <= mvy2[3:0];                       // (p >> 6) & SUBPEL_MASK
                ib_x0 <= 15'(signed'({2'b0, 13'((13'(b.c) >> ibc_sx) << 2)})) + 15'(mvx2 >>> 4);
                ib_y0 <= 15'(signed'({2'b0, 13'((13'(b.r) >> ibc_sy) << 2)})) + 15'(mvy2 >>> 4);
                ib_lastx <= 13'((((13'(hdr.mi_cols) << 2) + 13'(ibc_sx)) >> ibc_sx) - 13'd1);   // RefUpscaledWidth[-1] = MiCols * MI_SIZE
                ib_lasty <= 13'((((13'(hdr.mi_rows) << 2) + 13'(ibc_sy)) >> ibc_sy) - 13'd1);
                ib_i <= 8'd0; ib_j <= 8'd0;
                bst <= B_IBC_R0;
            end
            B_IBC_R0: bst <= B_IBC_R1;
            B_IBC_R1: begin ib_s0 <= fb_rdata; bst <= B_IBC_R2; end
            B_IBC_R2: begin ib_s1 <= fb_rdata; bst <= B_IBC_R3; end
            B_IBC_R3: begin ib_s2 <= fb_rdata; bst <= B_IBC_WR; end
            B_IBC_WR: begin
                if (ib_j + 8'd1 < ib_w) begin ib_j <= ib_j + 8'd1; bst <= B_IBC_R0; end
                else if (ib_i + 8'd1 < ib_h) begin ib_j <= 8'd0; ib_i <= ib_i + 8'd1; bst <= B_IBC_R0; end
                else if (ib_pl < (b.has_chroma ? 2'd2 : 2'd0)) begin ib_pl <= ib_pl + 2'd1; bst <= B_IBC_PL; end
                else begin blk_ready <= 1'b1; bst <= B_IDLE; end
            end
            default: bst <= B_IDLE;
        endcase
    end

    // ================================================================ BlockDecoded flags + MaxLuma
    logic bdf [0:2][0:33][0:33];                           // [plane][y+1][x+1], y,x in -1..32
    logic [5:0] sb_size4;
    assign sb_size4 = hdr.sb128 ? 6'd32 : 6'd16;
    logic [12:0] max_luma_w, max_luma_h;

    // ================================================================ transform-block state
    tx_rec_t     t;
    logic        sub_x, sub_y;
    logic [6:0]  tw_px, th_px;                             // transform width / height in pixels
    logic [2:0]  l2w, l2h;
    logic [5:0]  tw_c, th_c;                               // coefficient dims: min(32, w/h)
    logic [4:0]  step_x, step_y;                           // in 4x4 units
    logic [10:0] mi_row, mi_col;                           // luma MI position of the tx block
    logic [5:0]  sb_row_p, sb_col_p;                       // (row & sbMask) >> sub, plane units
    logic [12:0] max_x, max_y;                             // last valid pixel index in the plane
    logic [12:0] base_x, base_y;                           // block origin in plane pixels
    logic        is_pal, is_cfl;
    logic [3:0]  mode;
    logic        have_l, have_a, have_ar, have_bl;
    logic [12:0] above_lim, left_lim;
    logic [6:0]  above_px, left_px;
    logic [95:0] pal_cols;
    always_comb begin
        sub_x = (t.plane != 2'd0) && hdr.ssx;
        sub_y = (t.plane != 2'd0) && hdr.ssy;
        tw_px = tx_width(t.txsz); th_px = tx_height(t.txsz);
        l2w = tx_w_log2(t.txsz); l2h = tx_h_log2(t.txsz);
        tw_c = (tw_px > 7'd32) ? 6'd32 : 6'(tw_px);
        th_c = (th_px > 7'd32) ? 6'd32 : 6'(th_px);
        step_x = 5'(tw_px >> 2); step_y = 5'(th_px >> 2);
        mi_row = 11'((t.y << sub_y) >> 2); mi_col = 11'((t.x << sub_x) >> 2);
        sb_row_p = 6'((mi_row & 11'(hdr.sb128 ? 31 : 15)) >> sub_y);
        sb_col_p = 6'((mi_col & 11'(hdr.sb128 ? 31 : 15)) >> sub_x);
        max_x = 13'(((13'(hdr.mi_cols) << 2) >> sub_x) - 13'd1);
        max_y = 13'(((13'(hdr.mi_rows) << 2) >> sub_y) - 13'd1);
        base_x = 13'((13'(b.c) >> sub_x) << 2); base_y = 13'((13'(b.r) >> sub_y) << 2);
        is_pal = (t.plane == 2'd0) ? (b.pal_y != 4'd0) : (b.pal_uv != 4'd0);
        is_cfl = (t.plane != 2'd0) && (b.uvmode == UV_CFL_PRED);
        mode = (t.plane == 2'd0) ? b.ymode : (is_cfl ? DC_PRED : b.uvmode);
        have_l = ((t.plane == 2'd0) ? av_l : av_lc) || (t.x > base_x);
        have_a = ((t.plane == 2'd0) ? av_u : av_uc) || (t.y > base_y);
        have_ar = bdf[t.plane][sb_row_p - 6'd1 + 6'd1][sb_col_p + 6'(step_x) + 6'd1];
        have_bl = bdf[t.plane][sb_row_p + 6'(step_y) + 6'd1][sb_col_p - 6'd1 + 6'd1];
        above_lim = t.x + (have_ar ? 13'(tw_px) * 2 : 13'(tw_px)) - 13'd1;
        if (above_lim > max_x) above_lim = max_x;
        left_lim = t.y + (have_bl ? 13'(th_px) * 2 : 13'(th_px)) - 13'd1;
        if (left_lim > max_y) left_lim = max_y;
        above_px = (13'(tw_px) < max_x - t.x + 13'd1) ? tw_px : 7'(max_x - t.x + 13'd1);
        left_px  = (13'(th_px) < max_y - t.y + 13'd1) ? th_px : 7'(max_y - t.y + 13'd1);
        pal_cols = (t.plane == 2'd0) ? b.col_y : (t.plane == 2'd1) ? b.col_u : b.col_v;
    end

    // ---------------------------------------------------------------- dequant parameters (per tx block)
    logic [7:0]  qindex, dc_qi, ac_qi;
    logic [14:0] dc_q, ac_q;
    logic [3:0]  qm_lvl;
    logic        use_qm;
    logic [1:0]  bd_idx;
    logic [1:0]  dq_shift;
    logic signed [10:0] q_base;
    logic signed [6:0] d_dc, d_ac;
    always_comb begin
        q_base = 11'(signed'({3'b0, hdr.delta_q_present ? b.qidx : hdr.base_q_idx}));
        if (hdr.seg_enabled && rh.seg_altq_en[b.seg]) qindex = clip255(q_base + 11'(signed'(rh.seg_altq[9 * b.seg +: 9])));
        else qindex = 8'(q_base);
        case (t.plane)
            2'd0: begin d_dc = rh.dq_ydc; d_ac = 7'sd0; end
            2'd1: begin d_dc = rh.dq_udc; d_ac = rh.dq_uac; end
            default: begin d_dc = rh.dq_vdc; d_ac = rh.dq_vac; end
        endcase
        dc_qi = clip255(11'(signed'({3'b0, qindex})) + 11'(d_dc));
        ac_qi = clip255(11'(signed'({3'b0, qindex})) + 11'(d_ac));
        bd_idx = 2'((hdr.bit_depth - 4'd8) >> 1);
        dc_q = dc_qlookup(bd_idx, dc_qi);
        ac_q = ac_qlookup(bd_idx, ac_qi);
        qm_lvl = rh.qm_level[(int'(t.plane) * 8 + int'(b.seg)) * 4 +: 4];
        use_qm = rh.using_qmatrix && (t.txtype < IDTX) && (qm_lvl < 4'd15);
        case (t.txsz)
            5'd3, 5'd9, 5'd10, 5'd17, 5'd18: dq_shift = 2'd1;    // TX_32X32, 16X32, 32X16, 16X64, 64X16
            5'd4, 5'd11, 5'd12: dq_shift = 2'd2;                 // TX_64X64, 32X64, 64X32
            default: dq_shift = 2'd0;
        endcase
    end
    logic flip_ud, flip_lr;
    always_comb begin
        flip_ud = (t.txtype == 4'd4) || (t.txtype == 4'd6) || (t.txtype == 4'd8) || (t.txtype == 4'd14);   // FLIPADST_DCT, FLIPADST_FLIPADST, FLIPADST_ADST, V_FLIPADST
        flip_lr = (t.txtype == 4'd5) || (t.txtype == 4'd6) || (t.txtype == 4'd7) || (t.txtype == 4'd15);   // DCT_FLIPADST, FLIPADST_FLIPADST, ADST_FLIPADST, H_FLIPADST
    end

    // ================================================================ sub-blocks
    // itx2d
    logic itx_we, itx_clr, itx_start, itx_busy, itx_done; logic [9:0] itx_addr; logic [TW-1:0] itx_data; logic [11:0] res_addr; logic [TW-1:0] res_data; logic [4*TW-1:0] res_data4;
    itx2d #(.TW(TW)) u_itx (.clk, .rst, .coef_clr(itx_clr), .coef_we(itx_we), .coef_addr(itx_addr), .coef_data(itx_data),
                            .start(itx_start), .tx_sz(t.txsz), .tx_type(t.lossless ? 4'd0 : t.txtype), .bit_depth(hdr.bit_depth), .lossless(t.lossless),
                            .busy(itx_busy), .done(itx_done), .res_addr, .res_data, .res_data4);
    // ipred
    logic edge_we, ip_start, ip_busy, ip_done, ip_ov; logic [1:0] edge_side; logic [7:0] edge_idx; logic [11:0] edge_data; logic [47:0] ip_pix4; logic [5:0] ip_ox, ip_oy;
    logic ft_sel;
    assign ft_sel = (is_dir(mode) && !((t.plane == 2'd0) && b.use_fi)) ? ((t.plane == 2'd0) ? ft_y : ft_uv) : 1'b0;
    ipred #(.PW(12)) u_ip (.clk, .rst, .edge_we, .edge_side, .edge_idx, .edge_data,
                           .start(ip_start), .mode, .use_filter_intra((t.plane == 2'd0) && b.use_fi), .filter_intra_mode(b.fi_mode),
                           .angle_delta((t.plane == 2'd0) ? b.angle_y : b.angle_uv), .log2w(l2w), .log2h(l2h), .bit_depth(hdr.bit_depth),
                           .have_left(have_l), .have_above(have_a), .filter_type(ft_sel), .edge_filter_en(rh.enable_intra_edge_filter),
                           .above_px, .left_px, .busy(ip_busy), .done(ip_done),
                           .out_valid(ip_ov), .out_x(ip_ox), .out_y(ip_oy), .out_pix4(ip_pix4));
    // cfl
    logic cl_lwe, cl_dwe, cl_start, cl_busy, cl_done, cl_ov; logic [9:0] cl_laddr, cl_daddr; logic [11:0] cl_ldata, cl_ddata, cl_pix; logic [4:0] cl_ox, cl_oy;
    logic [12:0] lx0, ly0;
    assign lx0 = 13'(t.x << sub_x); assign ly0 = 13'(t.y << sub_y);
    logic [6:0] l_av_w, l_av_h;
    assign l_av_w = 7'(max_luma_w - lx0); assign l_av_h = 7'(max_luma_h - ly0);
    cfl #(.PW(12)) u_cfl (.clk, .rst, .luma_we(cl_lwe), .luma_addr(cl_laddr), .luma_data(cl_ldata), .dc_we(cl_dwe), .dc_addr(cl_daddr), .dc_data(cl_ddata),
                          .start(cl_start), .log2w(l2w), .log2h(l2h), .sub_x, .sub_y, .alpha((t.plane == 2'd1) ? b.cfl_u : b.cfl_v),
                          .bit_depth(hdr.bit_depth), .luma_avail_w(l_av_w), .luma_avail_h(l_av_h), .busy(cl_busy), .done(cl_done),
                          .out_valid(cl_ov), .out_x(cl_ox), .out_y(cl_oy), .out_pix(cl_pix));
    // quantizer matrix ROM
    logic [16:0] qm_addr; logic [7:0] qm_data;
    qm_rom u_qm (.clk, .addr(qm_addr), .data(qm_data));

    // ================================================================ per-transform-block FSM
    typedef enum logic [4:0] {
        R_IDLE, R_SETUP,
        R_PAL, R_PAL_LAST,
        R_EDGE, R_IP_START, R_IP_W,
        R_CFL_L, R_CFL_D, R_CFL_START, R_CFL_W,
        R_RESID, R_DQ, R_DQ_LAST, R_ITX_START, R_ITX_W, R_ADD, R_ADD_LAST,
        R_FIN, R_MIW
    } rst_t;
    rst_t rs;
    logic tx_ack;                                          // registered: the transform block was fully written (R_MIW)
    // cycle counters per transform-FSM state and per block-FSM state (simulation profiling; read through the
    // hierarchy by the testbench, never reset except by rst)
    logic [31:0] perf_rs [0:31];
    logic [31:0] perf_bst [0:15];
    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < 32; i++) perf_rs[i] <= 32'd0;
            for (int i = 0; i < 16; i++) perf_bst[i] <= 32'd0;
        end else begin
            perf_rs[rs] <= perf_rs[rs] + 32'd1;
            perf_bst[bst] <= perf_bst[bst] + 32'd1;
        end
    end
    logic sb_start, blk_take, tx_take, tx_pop;
    logic [10:0] sb_r, sb_c;
    assign sb_start = ev_valid && (ev_kind == 2'd2) && (rs == R_IDLE) && (bst == B_IDLE);
    assign sb_r = ev_sb_r; assign sb_c = ev_sb_c;
    assign blk_take = ev_valid && (ev_kind == 2'd1) && (bst == B_IDLE);
    assign tx_take = ev_valid && (ev_kind == 2'd0) && (rs == R_IDLE) && blk_ready && !tx_ack;
    assign tx_pop = (rs == R_MIW) && !mi_tx_busy && !mi_tx_we;
    assign ev_pop = sb_start || blk_take || tx_pop;
    assign q_slot = t.slot;
    logic [6:0]  ci, cj;                                   // generic pixel / coefficient counters (row, col)
    logic [8:0]  ek, n_edges;                              // edge entry counter: 0..2(w+h)
    logic        ep_valid, ep_const;                       // edge pipeline register
    logic [1:0]  ep_side; logic [7:0] ep_idx; logic [11:0] ep_val;
    logic        px_valid;                                 // generic 1-deep read pipeline
    logic [6:0]  px_i, px_j;
    logic        p_valid;                                  // dequant / add pipelines: previous position issued
    logic [6:0]  pci, pcj;
    // dequant walks the coded coefficients in scan order (k = 0 .. eob-1) instead of every position of the
    // block: scan ROM (registered) -> position -> Quant read (registered) -> dequant -> itx coefficient write
    logic [10:0] sk;                                       // next scan index to issue
    logic        sd_valid;                                 // scan index issued last cycle (its position lands now)
    logic [10:0] sd_k;
    logic [2:0]  s_bwl, s_hlog;                            // adjusted transform size (spec Adjusted_Tx_Size)
    logic [1:0]  s_cls;                                    // 0 2D (default scan), 1 Mrow (V_*), 2 Mcol (H_*)
    logic [9:0]  s_pos;                                    // position of sd_k in the adjusted grid
    logic [6:0]  sc_ci, sc_cj;                             // its coefficient row / column
    logic [11:0] scan_addr; logic [9:0] scan_data;
    scan_rom u_scan (.clk(clk), .addr(scan_addr), .data(scan_data));
    always_comb begin
        s_bwl  = tx_w_log2(tx_adj(t.txsz));
        s_hlog = tx_h_log2(tx_adj(t.txsz));
        s_cls  = (t.txtype == 4'd10 || t.txtype == 4'd12 || t.txtype == 4'd14) ? 2'd1 :
                 (t.txtype == 4'd11 || t.txtype == 4'd13 || t.txtype == 4'd15) ? 2'd2 : 2'd0;
        scan_addr = scan_base(t.txsz) + 12'(sk);
        case (t.lossless ? 2'd0 : s_cls)
            2'd1:    s_pos = sd_k[9:0];                                                                      // Mrow: row-major
            2'd2:    s_pos = ((sd_k[9:0] & ((10'd1 << s_hlog) - 10'd1)) << s_bwl) | (sd_k[9:0] >> s_hlog);   // Mcol
            default: s_pos = scan_data;
        endcase
        sc_ci = 7'(s_pos >> s_bwl);
        sc_cj = 7'(s_pos & ((10'd1 << s_bwl) - 10'd1));
    end
    logic        pal_first;
    logic [12:0] cur_x, cur_y;
    logic [11:0] mid_m1, mid_p1, mid;
    assign mid = 12'(13'd1 << (hdr.bit_depth - 4'd1));
    assign mid_m1 = mid - 12'd1;
    assign mid_p1 = mid + 12'd1;
    logic [11:0] pix_max;
    assign pix_max = 12'((13'd1 << hdr.bit_depth) - 13'd1);

    // edge entry k -> (side, idx, is_const, const value, read coordinates)
    logic        e_const;
    logic [1:0]  e_side;
    logic [7:0]  e_idx;
    logic [11:0] e_val;
    logic [12:0] e_rx, e_ry;
    always_comb begin
        e_const = 1'b0; e_side = 2'd0; e_idx = 8'd0; e_val = 12'd0; e_rx = t.x; e_ry = t.y;
        if (ek < 9'(tw_px) + 9'(th_px)) begin
            e_side = 2'd0; e_idx = 8'(ek);
            if (have_a) begin
                e_rx = (t.x + 13'(ek) > above_lim) ? above_lim : t.x + 13'(ek); e_ry = t.y - 13'd1;
            end else if (have_l) begin
                e_rx = t.x - 13'd1; e_ry = t.y;
            end else begin e_const = 1'b1; e_val = mid_m1; end
        end else if (ek < 2 * (9'(tw_px) + 9'(th_px))) begin
            e_side = 2'd1; e_idx = 8'(ek - 9'(tw_px) - 9'(th_px));
            if (have_l) begin
                e_rx = t.x - 13'd1; e_ry = (t.y + 13'(e_idx) > left_lim) ? left_lim : t.y + 13'(e_idx);
            end else if (have_a) begin
                e_rx = t.x; e_ry = t.y - 13'd1;
            end else begin e_const = 1'b1; e_val = mid_p1; end
        end else begin
            e_side = 2'd2; e_idx = 8'd0;
            if (have_a && have_l) begin e_rx = t.x - 13'd1; e_ry = t.y - 13'd1; end
            else if (have_a) begin e_rx = t.x; e_ry = t.y - 13'd1; end
            else if (have_l) begin e_rx = t.x - 13'd1; e_ry = t.y; end
            else begin e_const = 1'b1; e_val = mid; end
        end
    end

    // dequant datapath (R_DQ_B): q_data / qm_data are valid for the position issued in R_DQ_A
    logic [17:0] q_eff;
    logic signed [39:0] dq_full;
    logic [23:0] dq_abs;
    logic [23:0] dq_sh;
    logic signed [TW-1:0] dq_out;
    logic signed [20:0] lim_pos;
    always_comb begin
        begin
            logic [14:0] qv;
            logic [22:0] prod;
            qv = (pci == 7'd0 && pcj == 7'd0) ? dc_q : ac_q;      // dequant of the position issued last cycle
            prod = 23'(qv) * 23'(qm_data);
            q_eff = use_qm ? 18'((prod + 23'd16) >> 5) : 18'(qv);
        end
        dq_full = 40'(q_data) * 40'(signed'({1'b0, q_eff}));
        dq_abs = (dq_full < 0) ? 24'(-dq_full) : 24'(dq_full);           // & 0xFFFFFF
        dq_sh = dq_abs >> dq_shift;
        lim_pos = 21'sd1 <<< (5'(hdr.bit_depth) + 5'd7);          // 1 << (7 + BitDepth), up to 2^19
        begin
            logic signed [24:0] v;
            v = (dq_full < 0) ? -$signed({1'b0, dq_sh}) : $signed({1'b0, dq_sh});
            if (v < -25'(lim_pos)) dq_out = TW'(-25'(lim_pos));
            else if (v > 25'(lim_pos) - 25'sd1) dq_out = TW'(25'(lim_pos) - 25'sd1);
            else dq_out = TW'(v);
        end
    end
    // recon add datapath: residual columns cj .. cj+3 of row ci map to the aligned picture group starting at
    // add_xx (reversed when flip_lr); (ci, cj) is the group being read, (pci, pcj) the one being added and written
    logic [6:0]  add_xx, add_yy, padd_xx, padd_yy;
    assign padd_xx = flip_lr ? 7'(tw_px - 7'd4 - pcj) : pcj;
    assign padd_yy = flip_ud ? 7'(th_px - 7'd1 - pci) : pci;
    assign add_xx = flip_lr ? 7'(tw_px - 7'd4 - cj) : cj;
    assign add_yy = flip_ud ? 7'(th_px - 7'd1 - ci) : ci;
    logic [47:0] add_out4;
    always_comb begin
        for (int m = 0; m < 4; m++) begin
            logic signed [TW:0] sum;
            logic [TW-1:0] r;
            r = res_data4[(flip_lr ? 3 - m : m) * TW +: TW];
            sum = $signed({1'b0, (TW - 12)'(0), fb_rdata4[m*12 +: 12]}) + $signed({r[TW-1], r});
            add_out4[m*12 +: 12] = (sum < 0) ? 12'd0 : (sum > $signed({1'b0, (TW - 12)'(0), pix_max})) ? pix_max : 12'(sum);
        end
    end

    // ---------------------------------------------------------------- combinational outputs
    always_comb begin
        fb_re = 1'b0; fb_we = 1'b0; fb_plane = t.plane; fb_x = '0; fb_y = '0; fb_wdata = 12'd0;
        fb2_we = 1'b0; fb2_plane = t.plane; fb2_x = '0; fb2_y = '0; fb2_wdata = 12'd0;
        fb4_we = 1'b0; fb4_plane = t.plane; fb4_x = '0; fb4_y = '0; fb4_wdata = 48'd0;
        edge_we = 1'b0; edge_side = ep_side; edge_idx = ep_idx; edge_data = ep_const ? ep_val : fb_rdata;
        itx_we = 1'b0; itx_addr = 10'd0; itx_data = '0;
        cl_lwe = 1'b0; cl_dwe = 1'b0; cl_laddr = 10'd0; cl_ldata = fb_rdata; cl_daddr = 10'd0; cl_ddata = fb_rdata;
        res_addr = 12'({pci[5:0], pcj[5:0]});                 // residual of the position read last cycle
        pm_plane = (t.plane != 2'd0); pm_x = 6'(t.x - base_x + 13'(cj)); pm_y = 6'(t.y - base_y + 13'(ci));
        q_addr = 10'(sc_ci) * 10'(tw_c) + 10'(sc_cj);    // registered read in coef_rd: data lands the next cycle
        qm_addr = 17'(int'(qm_lvl) * QM_LEVEL_STRIDE + ((t.plane != 2'd0) ? QM_PLANE_STRIDE : 0)) + 17'(qm_offset(t.txsz)) + 17'(sc_ci) * 17'(tw_c) + 17'(sc_cj);
        itx_clr = (rs == R_RESID);
        case (rs)
            R_PAL, R_PAL_LAST: begin
                // pm_idx holds the index for (px_i, px_j) issued last cycle
                if (px_valid) begin
                    fb2_we = 1'b1; fb2_x = FBX'(t.x + 13'(px_j)); fb2_y = FBY'(t.y + 13'(px_i));
                    fb2_wdata = pal_cols[12 * pm_idx +: 12];
                end
            end
            R_EDGE: begin
                if (ek <= n_edges && !e_const) begin fb_re = 1'b1; fb_x = FBX'(e_rx); fb_y = FBY'(e_ry); end
                if (ep_valid) edge_we = 1'b1;
            end
            R_IP_W: if (ip_ov) begin fb4_we = 1'b1; fb4_x = FBX'(t.x + 13'(ip_ox)); fb4_y = FBY'(t.y + 13'(ip_oy)); fb4_wdata = ip_pix4; end
            R_CFL_L: begin
                // read luma (plane 0) at (lx0 + cj, ly0 + ci); write the previously read pixel into cfl
                fb_plane = 2'd0;
                if (ci < 7'(th_px << sub_y)) begin fb_re = 1'b1; fb_x = FBX'(lx0 + 13'(cj)); fb_y = FBY'(ly0 + 13'(ci)); end
                if (px_valid) begin cl_lwe = 1'b1; cl_laddr = {px_i[4:0], px_j[4:0]}; end
            end
            R_CFL_D: begin
                if (ci < 7'(th_px)) begin fb_re = 1'b1; fb_x = FBX'(t.x + 13'(cj)); fb_y = FBY'(t.y + 13'(ci)); end
                if (px_valid) begin cl_dwe = 1'b1; cl_daddr = {px_i[4:0], px_j[4:0]}; end
            end
            R_CFL_W: if (cl_ov) begin fb2_we = 1'b1; fb2_x = FBX'(t.x + 13'(cl_ox)); fb2_y = FBY'(t.y + 13'(cl_oy)); fb2_wdata = cl_pix; end
            // dequant: one coefficient per clock (address (ci, cj) out, the previous position's data in)
            R_DQ, R_DQ_LAST: if (p_valid) begin itx_we = 1'b1; itx_addr = {pci[4:0], pcj[4:0]}; itx_data = dq_out; end
            // residual add: read the group at (ci, cj) through port 1, write the previous group through port 4
            R_ADD, R_ADD_LAST: begin
                if (rs == R_ADD) begin fb_re = 1'b1; fb_x = FBX'(t.x + 13'(add_xx)); fb_y = FBY'(t.y + 13'(add_yy)); end
                if (p_valid) begin fb4_we = 1'b1; fb4_x = FBX'(t.x + 13'(padd_xx)); fb4_y = FBY'(t.y + 13'(padd_yy)); fb4_wdata = add_out4; end
            end
            default: ;
        endcase
        // intra block copy: the block FSM owns the frame-buffer ports while the transform FSM is idle
        case (bst)
            B_IBC_R0: begin fb_re = 1'b1; fb_plane = ib_pl; fb_x = FBX'(ib_rx0); fb_y = FBY'(ib_ry0); end
            B_IBC_R1: begin fb_re = ib_fx[3]; fb_plane = ib_pl; fb_x = FBX'(ib_rx1); fb_y = FBY'(ib_ry0); end
            B_IBC_R2: begin fb_re = ib_fy[3]; fb_plane = ib_pl; fb_x = FBX'(ib_rx0); fb_y = FBY'(ib_ry1); end
            B_IBC_R3: begin fb_re = ib_fx[3] && ib_fy[3]; fb_plane = ib_pl; fb_x = FBX'(ib_rx1); fb_y = FBY'(ib_ry1); end
            B_IBC_WR: begin fb2_we = 1'b1; fb2_plane = ib_pl; fb2_x = FBX'(ib_bx + 13'(ib_j)); fb2_y = FBY'(ib_by + 13'(ib_i)); fb2_wdata = ib_out; end
            default: ;
        endcase
    end
    assign busy = (rs != R_IDLE) || (bst != B_IDLE) || !blk_ready || mi_blk_busy || mi_tx_busy;
    assign mi_tx_plane = t.plane; assign mi_tx_sz = t.txsz;
    assign mi_tx_row = 11'(t.y >> 2); assign mi_tx_col = 11'(t.x >> 2);          // plane 4x4 units
    assign mi_tx_w4 = step_x; assign mi_tx_h4 = step_y;

    // ---------------------------------------------------------------- sequential
    always_ff @(posedge clk) begin
        tx_ack <= 1'b0; itx_start <= 1'b0; ip_start <= 1'b0; cl_start <= 1'b0; mi_tx_we <= 1'b0;
        if (rst) begin
            rs <= R_IDLE; max_luma_w <= 13'd0; max_luma_h <= 13'd0;
            for (int p = 0; p < 3; p++) for (int yy = 0; yy < 34; yy++) for (int xx = 0; xx < 34; xx++) bdf[p][yy][xx] <= 1'b0;
        end else begin
            // 5.11.3 clear_block_decoded_flags at each superblock start
            if (sb_start) begin
                for (int p = 0; p < 3; p++) begin
                    logic sx, sy;
                    logic [10:0] sbw4, sbh4;
                    logic [5:0] lim_x, lim_y;
                    sx = (p != 0) && hdr.ssx; sy = (p != 0) && hdr.ssy;
                    sbw4 = (hdr.mi_col_end - sb_c) >> sx;
                    sbh4 = (hdr.mi_row_end - sb_r) >> sy;
                    lim_x = sb_size4 >> sx; lim_y = sb_size4 >> sy;
                    for (int yy = -1; yy <= 32; yy++)
                        for (int xx = -1; xx <= 32; xx++) begin
                            logic v;
                            v = 1'b0;
                            if (yy <= int'(lim_y) && xx <= int'(lim_x)) begin
                                if (yy < 0 && xx < int'(sbw4)) v = 1'b1;
                                else if (xx < 0 && yy < int'(sbh4)) v = 1'b1;
                            end
                            bdf[p][yy + 1][xx + 1] <= v;
                        end
                    bdf[p][int'(lim_y) + 1][0] <= 1'b0;
                end
            end
            case (rs)
                R_IDLE: if (tx_take) begin
                    t <= ev_tx; rs <= R_SETUP;
                end
                R_SETUP: begin
                    ci <= 7'd0; cj <= 7'd0; px_valid <= 1'b0; ek <= 9'd0; ep_valid <= 1'b0;
                    n_edges <= 2 * (9'(tw_px) + 9'(th_px));
                    if (b.is_inter) rs <= R_RESID;                       // predicted whole by the block copy
                    else if (is_pal) rs <= R_PAL;
                    else rs <= R_EDGE;
                end
                // ---- palette prediction: issue map read for (ci, cj), write the previous pixel
                R_PAL: begin
                    px_valid <= 1'b1; px_i <= ci; px_j <= cj;
                    if (cj + 7'd1 < tw_px) cj <= cj + 7'd1;
                    else begin
                        cj <= 7'd0;
                        if (ci + 7'd1 < th_px) ci <= ci + 7'd1;
                        else rs <= R_PAL_LAST;
                    end
                end
                R_PAL_LAST: begin px_valid <= 1'b0; ci <= 7'd0; cj <= 7'd0; rs <= R_RESID; end
                // ---- edge preparation (7.11.2.1), pipelined: issue entry ek, write entry ek-1
                R_EDGE: begin
                    if (ek <= n_edges) begin
                        ep_valid <= 1'b1; ep_side <= e_side; ep_idx <= e_idx; ep_const <= e_const; ep_val <= e_val;
                        ek <= ek + 9'd1;
                    end else begin
                        ep_valid <= 1'b0;
                        if (!ep_valid) begin ip_start <= 1'b1; rs <= R_IP_W; end
                    end
                end
                R_IP_W: if (ip_done) begin
                    ci <= 7'd0; cj <= 7'd0; px_valid <= 1'b0;
                    rs <= is_cfl ? R_CFL_L : R_RESID;
                end
                // ---- CfL: load the luma region, then the DC prediction, run, write back
                R_CFL_L: begin
                    px_valid <= (ci < 7'(th_px << sub_y)); px_i <= ci; px_j <= cj;
                    if (ci < 7'(th_px << sub_y)) begin
                        if (cj + 7'd1 < 7'(tw_px << sub_x)) cj <= cj + 7'd1;
                        else begin cj <= 7'd0; ci <= ci + 7'd1; end
                    end else begin
                        ci <= 7'd0; cj <= 7'd0; rs <= R_CFL_D;
                    end
                end
                R_CFL_D: begin
                    px_valid <= (ci < 7'(th_px)); px_i <= ci; px_j <= cj;
                    if (ci < 7'(th_px)) begin
                        if (cj + 7'd1 < tw_px) cj <= cj + 7'd1;
                        else begin cj <= 7'd0; ci <= ci + 7'd1; end
                    end else begin
                        cl_start <= 1'b1; rs <= R_CFL_W;
                    end
                end
                R_CFL_W: if (cl_done) begin ci <= 7'd0; cj <= 7'd0; rs <= R_RESID; end
                // ---- residual
                R_RESID: begin                           // itx coefficient array cleared this cycle (itx_clr)
                    ci <= 7'd0; cj <= 7'd0;
                    p_valid <= 1'b0; sd_valid <= 1'b0; sk <= 11'd0;
                    if (!t.skip && t.eob != 11'd0) rs <= R_DQ;
                    else rs <= R_FIN;
                end
                R_DQ: begin                              // issue scan index sk; sd_k's position reads Quant; the previous Quant is dequantized
                    sd_valid <= (sk < t.eob); sd_k <= sk;
                    if (sk < t.eob) sk <= sk + 11'd1;
                    p_valid <= sd_valid; pci <= sc_ci; pcj <= sc_cj;
                    if (sk >= t.eob && !sd_valid) rs <= R_DQ_LAST;
                end
                R_DQ_LAST: begin p_valid <= 1'b0; ci <= 7'd0; cj <= 7'd0; itx_start <= 1'b1; rs <= R_ITX_W; end
                R_ITX_W: if (itx_done) begin p_valid <= 1'b0; rs <= R_ADD; end
                R_ADD: begin                             // read group (ci, cj..cj+3); the previous group is added and written this cycle
                    p_valid <= 1'b1; pci <= ci; pcj <= cj;
                    if (cj + 7'd4 < tw_px) cj <= cj + 7'd4;
                    else if (ci + 7'd1 < th_px) begin cj <= 7'd0; ci <= ci + 7'd1; end
                    else rs <= R_ADD_LAST;
                end
                R_ADD_LAST: begin p_valid <= 1'b0; rs <= R_FIN; end
                // ---- bookkeeping + ack
                R_FIN: begin
                    for (int yy = 0; yy < 16; yy++)
                        for (int xx = 0; xx < 16; xx++)
                            if (yy < int'(step_y) && xx < int'(step_x))
                                bdf[t.plane][int'(sb_row_p) + yy + 1][int'(sb_col_p) + xx + 1] <= 1'b1;
                    if (t.plane == 2'd0) begin max_luma_w <= t.x + 13'(tw_px); max_luma_h <= t.y + 13'(th_px); end
                    rs <= R_MIW;
                end
                R_MIW: if (!mi_tx_busy && !mi_tx_we) begin       // LoopfilterTxSizes over the transform block's area
                    mi_tx_we <= 1'b1; tx_ack <= 1'b1; rs <= R_IDLE;
                end
                default: rs <= R_IDLE;
            endcase
        end
    end
endmodule
