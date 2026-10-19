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
  import mc_tables_pkg::*;
#(
    parameter int FBX = 9,                   // frame buffer x address bits
    parameter int ITX_LANES = 4,             // 1-D transform engines (rows, then columns, at once): 4 for throughput, 2 fits the K26
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
    input  logic [MIW-1:0] ev_sb_r, ev_sb_c,
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
    output logic [MIW-1:0] mi_blk_r, mi_blk_c,
    output logic [5:0]  mi_blk_bw4, mi_blk_bh4,
    output mi_lf_t      mi_blk_data,
    input  logic        mi_blk_busy,
    output logic        mi_tx_we,
    output logic [1:0]  mi_tx_plane,
    output logic [MIW-1:0] mi_tx_row, mi_tx_col,
    output logic [4:0]  mi_tx_w4, mi_tx_h4,
    output logic [4:0]  mi_tx_sz,
    input  logic        mi_tx_busy,
    output logic        mi_cd_clr,             // superblock start: clear the superblock's cdef_idx unit(s)
    output logic        mi_cd_we,              // block with a coded cdef_idx: set the covered unit(s)
    output logic [SBW-1:0]  mi_cd_row64, mi_cd_col64,
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
    output logic [47:0] fb4_wdata,
    output logic [3:0]  fb4_be,               // lane enables of the 4-wide write (inter prediction units 2 samples wide)
    // ---- inter prediction (stage 2a, docs/inter-syntax.md / the design note §8): the inter header, the reference
    //      slot table (software: REF_SLOT, the slots' plane bases / strides / sizes) and the AXI read master of ref_fetch
    input  ihdr_t       ihdr,
    input  logic [20:0] ref_slot,             // slot of LAST_FRAME + i at [3i +: 3]
    input  logic [1727:0] ref_planes,         // slot s, plane p: {stride[31:0], base[39:0]} at [72 (3s + p) +: 72]
    input  logic [255:0] ref_dims,            // slot s: {frame height[15:0], upscaled width[15:0]} at [32s +: 32]
    input  logic        ref_fmt16,            // reference samples are 16-bit LE (else one byte each)
    output logic        mc_unsup,             // sticky: an inter block used a feature beyond stage 2a (masks, OBMC, warp, scaled references)
    output logic [39:0] m_axi_araddr,
    output logic [7:0]  m_axi_arlen,
    output logic [2:0]  m_axi_arsize,
    output logic [1:0]  m_axi_arburst,
    output logic [3:0]  m_axi_arcache,
    output logic [2:0]  m_axi_arprot,
    output logic        m_axi_arvalid,
    input  logic        m_axi_arready,
    input  logic [63:0] m_axi_rdata,
    input  logic [1:0]  m_axi_rresp,
    input  logic        m_axi_rlast,
    input  logic        m_axi_rvalid,
    output logic        m_axi_rready
);
    localparam logic [3:0] DC_PRED = 4'd0, UV_CFL_PRED = 4'd13, SMOOTH_PRED = 4'd9, SMOOTH_H_PRED = 4'd11;
    localparam logic [3:0] IDTX = 4'd9;
    localparam int TW = 22;

    function automatic logic is_inside(input logic [MIW-1:0] r, input logic [MIW-1:0] c);
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
        av_u = is_inside(b.r - MIW'(1), b.c);
        av_l = is_inside(b.r, b.c - MIW'(1));
        av_uc = 1'b0; av_lc = 1'b0;
        if (b.has_chroma) begin
            av_uc = av_u; av_lc = av_l;
            if (hdr.ssy && bh4 == 6'd1) av_uc = is_inside(b.r - MIW'(2), b.c);
            if (hdr.ssx && bw4 == 6'd1) av_lc = is_inside(b.r, b.c - MIW'(2));
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
    function automatic logic [ML2R+ML2C-5:0] mm_index(input logic [MIW-1:0] row, input logic [MIW-1:0] col);
        mm_index = {ML2R'(row), (ML2C - 4)'(col >> 4)};
    endfunction
    function automatic logic [127:0] mm_merge(input logic [127:0] w, input logic [3:0] start, input logic [5:0] n, input logic [7:0] e);
        mm_merge = w;
        for (int k = 0; k < 16; k++)
            if (6'(k) >= 6'(start) && 6'(k) < 6'(start) + n) mm_merge[8 * k +: 8] = e;
    endfunction

    typedef enum logic [4:0] {B_IDLE, B_FT0, B_FT1, B_FT2, B_FT3, B_FT4, B_WR, B_WR_W,
                              B_IBC_INIT, B_IBC_PL, B_IBC_R0, B_IBC_R1, B_IBC_R2, B_IBC_R3, B_IBC_WR,
                              B_MC_INIT, B_MC_PL, B_MC_UNIT, B_MC_STRIP, B_MC_FETCH, B_MC_FILT, B_MC_PLEND, B_MC_OB, B_MC_MASK, B_MC_II} bst_t;
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
    logic [PXW-1:0] ib_bx, ib_by, ib_lastx, ib_lasty;
    logic [3:0]  ib_fx, ib_fy;
    logic signed [PXS-1:0] ib_x0, ib_y0;
    logic [11:0] ib_s0, ib_s1, ib_s2;
    logic [4:0]  ib_r0, ib_r1;
    assign ibc_sx = (ib_pl != 2'd0) && hdr.ssx;
    assign ibc_sy = (ib_pl != 2'd0) && hdr.ssy;
    assign ib_r0 = (hdr.bit_depth == 4'd12) ? 5'd5 : 5'd3;
    assign ib_r1 = (hdr.bit_depth == 4'd12) ? 5'd9 : 5'd11;
    function automatic logic [PXW-1:0] ib_clip(input logic signed [PXS-1:0] v, input logic [PXW-1:0] last);
        if (v < 0) ib_clip = PXW'(0);
        else if (v > PXS'(signed'({2'b0, last}))) ib_clip = last;
        else ib_clip = PXW'(v);
    endfunction
    logic [PXW-1:0] ib_rx0, ib_rx1, ib_ry0, ib_ry1;          // clamped source coordinates of the 2x2 neighbourhood
    assign ib_rx0 = ib_clip(ib_x0 + PXS'(signed'({7'b0, ib_j})), ib_lastx);
    assign ib_rx1 = ib_clip(ib_x0 + PXS'(signed'({7'b0, ib_j})) + PXS'(1), ib_lastx);
    assign ib_ry0 = ib_clip(ib_y0 + PXS'(signed'({7'b0, ib_i})), ib_lasty);
    assign ib_ry1 = ib_clip(ib_y0 + PXS'(signed'({7'b0, ib_i})) + PXS'(1), ib_lasty);
    // ---- inter prediction (7.11.3, stage 2a: translational, unscaled; single / AVERAGE / DISTANCE): the block is
    // predicted whole, plane by plane, unit by unit (one unit per plane, or the per-luma-4x4 chroma units of a sub-8x8
    // block from b.cmv), in strips of at most 64 x 64, list by list: ref_fetch brings the clamped reference region into
    // mc_filt's patch, mc_filt filters (and blends list 1 with the parked list 0) and its output goes to the frame
    // memory through the fb4 port; the transform flow then adds residuals (R_SETUP -> R_RESID, like intrabc).
    logic [1:0]  mc_pl;
    logic        mc_sx, mc_sy;
    logic [7:0]  mc_pw, mc_ph;                               // plane block size
    logic [PXW-1:0] mc_bx, mc_by;                            // plane block origin
    logic [7:0]  mc_uw, mc_uh;                               // unit size
    logic [1:0]  mc_ur, mc_uc, mc_ui, mc_uj;                 // unit grid and index
    logic        mc_cmv;                                     // units take their motion from b.cmv
    logic [PXW-1:0] mc_ux, mc_uy;                            // unit origin
    logic [1:0]  mc_nsr, mc_nsc, mc_sr, mc_sc;               // 64 x 64 strips of the unit
    logic [PXW-1:0] mc_px, mc_py;                            // strip origin
    logic [6:0]  mc_sw, mc_sh;                               // strip size
    logic        mc_l, mc_comp;
    logic [1:0]  mc_bmode;                                   // list-1 blend mode: 2 average, 3 distance
    logic [4:0]  mc_wt0, mc_wt1;
    logic [3:0]  u_ref0, u_ref1;                             // the unit's motion
    logic signed [15:0] u_mv0r, u_mv0c, u_mv1r, u_mv1c;
    logic [1:0]  u_i0, u_i1;
    // ref_fetch request / mc_filt run
    logic        rf_req_valid, rf_req_ready, rf_done, rf_err, rf_lst;
    logic [39:0] rf_base; logic [31:0] rf_stride; logic [15:0] rf_lastx, rf_lasty; logic signed [16:0] rf_x0, rf_y0;
    logic        mc_start, mc_busy, mc_done, mc_out_valid;
    logic        mc_fin;                                     // mc_done latched (the OBMC pipeline may still be draining at the pulse)
    logic [3:0]  mc_fx, mc_fy;
    logic [2:0]  mc_mode;
    logic [127:0] mc_out_pair;
    logic [6:0]  mc_out_x, mc_out_y; logic [47:0] mc_out_pix4; logic [3:0] mc_out_mask;
    logic [PXW-1:0] mc_ox; logic [1:0] mc_osh;              // output sample x, and its lane shift within the 4-group
    // OBMC (7.11.3.10, stage 2b.1): after a plane's own prediction, each overlap neighbour of b.obmc predicts a
    // (predW x predH) region with its own motion (single, through the same fetch / filter) and the frame memory is
    // read-modify-written: fb = Round2(m fb + (64 - m) pred, 6) with Obmc_Mask[predH][row] (above) / [predW][col] (left)
    logic        ob_active, ob_done, ob_pass;               // in an overlap prediction; both passes done for this plane; 0 above, 1 left
    logic [2:0]  ob_k;                                       // entry index within the pass
    logic [4:0]  mc_psz;                                     // the plane's block size (the above pass needs >= BLOCK_8X8)
    logic [5:0]  ob_mlen;                                    // mask length (predH above, predW left)
    logic        ob_rd_v, ob_rd2_v, ob_wr_v;                 // read-modify-write pipeline: A (read issued), A2 (data lands), B (write)
    logic [47:0] ob_a2_pix; logic [PXW-1:0] ob_a2_x, ob_a2_y; logic [3:0] ob_a2_be; logic [6:0] ob_a2_m [4];
    logic [47:0] ob_a_pix, ob_b_pix;
    logic [PXW-1:0] ob_a_x, ob_a_y, ob_b_x, ob_b_y;
    logic [3:0]  ob_a_be, ob_b_be;
    logic [6:0]  ob_a_m [4];                                 // per-lane mask values
    // Masked compound (7.11.3.12 - 7.11.3.14, stage 2b.2): the block's luma mask in 8 banks (row parity, x & 3) of 7 bits,
    // entry (y >> 1) * 32 + (x >> 2): luma reads one value per lane, 4:2:0 chroma averages the four luma values of its
    // 2x2 (two reads per bank), 4:2:2 the two of its row. WEDGE is filled from the master masks before the prediction,
    // DIFFWTD from the luma pair output as it is blended; list 1 of a masked block runs mc_filt in mode 4 (pair) and the
    // blend fb = clip(Round2(m p0 + (64 - m) p1, 6 + post)) happens here.
    logic [6:0]  mask_bank [8][0:2047];
    logic        mk_we [8];
    logic [10:0] mk_waddr [8];
    logic [6:0]  mk_wdata [8];
    logic [10:0] mk_ra0 [8], mk_ra1 [8];                    // two read ports per bank
    logic [6:0]  mk_q0 [8], mk_q1 [8];
    logic        mb_active;                                  // the block is a masked compound (WEDGE / DIFFWTD)
    logic        mb_diff;                                    // DIFFWTD (else WEDGE)
    logic [6:0]  mk_x, mk_y;                                 // wedge fill position (luma, block-relative)
    logic [2:0]  mk_d;                                       // wedge master direction
    logic [5:0]  mk_xoff, mk_yoff;
    logic        mk_inv;
    logic        mb_a_v, mb_b_v;                             // masked blend pipeline: A (the mask data lands, blend), B (write)
    logic [127:0] mb_a_pair, mb_a2_pair;
    logic [PXW-1:0] mb_a_x, mb_a_y, mb_a2_x, mb_a2_y, mb_b_x, mb_b_y;
    logic [3:0]  mb_a_be, mb_a2_be, mb_b_be;
    logic [6:0]  mb_a2_m [4];                                // DIFFWTD luma: the mask computed in A, carried to A2
    logic [47:0] mb_b_pix;
    logic [3:0]  mc_post;                                    // InterPostRound of the block's compound prediction
    always_ff @(posedge clk) begin
        for (int k = 0; k < 8; k++) begin
            if (mk_we[k]) mask_bank[k][mk_waddr[k]] <= mk_wdata[k];
            mk_q0[k] <= mask_bank[k][mk_ra0[k]];
            mk_q1[k] <= mask_bank[k][mk_ra1[k]];
        end
    end
    // Inter-intra (7.11.3.1 / 7.11.3.13, stage 2b.3): the plane's intra prediction (ipred through the transform flow with
    // a virtual transform record of the block size, written into the frame memory first), then the single inter
    // prediction blended in through the OBMC read-modify-write path: fb = Round2(m intra + (64 - m) inter, 6), m from
    // Ii_Weights_1d (sizeScale = 128 / max(w, h) of the plane block) or the wedge buffer (wedge_interintra)
    logic        ii_active, ii_wedge, ii_req, ii_run, ii_done;
    logic [1:0]  ii_pl;
    logic [PXW-1:0] ii_x, ii_y;
    logic [4:0]  ii_txsz;
    logic [3:0]  ii_mode;                                    // ipred mode: DC 0, V 1, H 2, SMOOTH 9
    logic [2:0]  ii_lsh;                                     // log2(sizeScale)
    function automatic logic [2:0] ii_scale_shift(input logic [7:0] w, input logic [7:0] h);   // log2(128 / max(w, h))
        logic [7:0] m;
        m = (w > h) ? w : h;
        if (m >= 8'd128) return 3'd0;
        if (m >= 8'd64) return 3'd1;
        if (m >= 8'd32) return 3'd2;
        if (m >= 8'd16) return 3'd3;
        if (m >= 8'd8) return 3'd4;
        if (m >= 8'd4) return 3'd5;
        return 3'd6;
    endfunction
    function automatic logic [6:0] diff_mask(input logic signed [15:0] p0, input logic signed [15:0] p1, input logic [3:0] bd,
                                             input logic [3:0] post, input logic mtype);
        logic signed [16:0] d;
        logic [16:0] a;
        logic [6:0] m;
        d = 17'(p0) - 17'(p1);
        a = (d < 0) ? 17'(-d) : 17'(d);
        a = (a + (17'd1 << (bd - 4'd8 + post - 4'd1))) >> (bd - 4'd8 + post);         // Round2(diff, bd - 8 + post)
        m = (a[16:4] + 13'd38 > 13'd64) ? 7'd64 : 7'(a[16:4] + 13'd38);                // clip3(0, 64, 38 + diff / 16)
        return mtype ? 7'd64 - m : m;
    endfunction
    function automatic logic [6:0] obmc_mask(input logic [5:0] len, input logic [5:0] i);
        case (len)
            6'd2:  return (i == 0) ? 7'd45 : 7'd64;
            6'd4:  case (i) 6'd0: return 7'd39; 6'd1: return 7'd50; 6'd2: return 7'd59; default: return 7'd64; endcase
            6'd8:  case (i) 6'd0: return 7'd36; 6'd1: return 7'd42; 6'd2: return 7'd48; 6'd3: return 7'd53; 6'd4: return 7'd57; 6'd5: return 7'd61; default: return 7'd64; endcase
            6'd16: case (i) 6'd0: return 7'd34; 6'd1: return 7'd37; 6'd2: return 7'd40; 6'd3: return 7'd43; 6'd4: return 7'd46; 6'd5: return 7'd49;
                            6'd6: return 7'd52; 6'd7: return 7'd54; 6'd8: return 7'd56; 6'd9: return 7'd58; 6'd10: return 7'd60; 6'd11: return 7'd61; default: return 7'd64; endcase
            default: case (i) 6'd0: return 7'd33; 6'd1: return 7'd35; 6'd2: return 7'd36; 6'd3: return 7'd38; 6'd4: return 7'd40; 6'd5: return 7'd41; 6'd6: return 7'd43;
                              6'd7: return 7'd44; 6'd8: return 7'd45; 6'd9: return 7'd47; 6'd10: return 7'd48; 6'd11: return 7'd50; 6'd12: return 7'd51; 6'd13: return 7'd52;
                              6'd14: return 7'd53; 6'd15: return 7'd55; 6'd16: return 7'd56; 6'd17: return 7'd57; 6'd18: return 7'd58; 6'd19: return 7'd59;
                              6'd20: return 7'd60; 6'd21: return 7'd60; 6'd22: return 7'd61; 6'd23: return 7'd62; default: return 7'd64; endcase
        endcase
    endfunction
    logic        pw_we, pw_list; logic [6:0] pw_row; logic [4:0] pw_w4; logic [47:0] pw_d4;

    ref_fetch #(.PW(12)) u_rf (.clk, .rst,
        .req_valid(rf_req_valid), .req_ready(rf_req_ready), .req_lst(rf_lst), .req_fmt16(ref_fmt16), .req_base(rf_base),
        .req_stride(rf_stride), .req_lastx(rf_lastx), .req_lasty(rf_lasty), .req_x0(rf_x0), .req_y0(rf_y0),
        .req_w(mc_sw), .req_h(mc_sh), .done(rf_done), .err(rf_err),
        .m_axi_araddr, .m_axi_arlen, .m_axi_arsize, .m_axi_arburst, .m_axi_arcache, .m_axi_arprot, .m_axi_arvalid, .m_axi_arready,
        .m_axi_rdata, .m_axi_rresp, .m_axi_rlast, .m_axi_rvalid, .m_axi_rready,
        .patch_we(pw_we), .patch_list(pw_list), .patch_row(pw_row), .patch_w4(pw_w4), .patch_d4(pw_d4));
    mc_filt #(.PW(12)) u_mc (.clk, .rst,
        .patch_we(pw_we), .patch_list(pw_list), .patch_row(pw_row), .patch_w4(pw_w4), .patch_d4(pw_d4),
        .start(mc_start), .lst(mc_l), .w(mc_sw), .h(mc_sh), .interp_x(u_i1), .interp_y(u_i0), .fx(mc_fx), .fy(mc_fy),
        .bit_depth(hdr.bit_depth), .mode(mc_mode), .wt0(mc_wt0), .wt1(mc_wt1), .busy(mc_busy), .done(mc_done),
        .out_valid(mc_out_valid), .out_x(mc_out_x), .out_y(mc_out_y), .out_pix4(mc_out_pix4), .out_mask(mc_out_mask), .out_pair(mc_out_pair));

    // 7.11.3.15 distance weights: {BckWeight (list 1), FwdWeight (list 0)} from the order-hint distances
    function automatic int rel_dist(input ihdr_t ih, input logic [7:0] oa, input logic [7:0] ob);
        int diff, m;
        if (!ih.enable_order_hint) return 0;
        diff = int'(oa) - int'(ob);
        m = 1 << (int'(ih.order_hint_bits) - 1);
        return (diff & (m - 1)) - (diff & m);
    endfunction
    function automatic logic [9:0] dist_weights(input ihdr_t ih, input logic [3:0] r0, input logic [3:0] r1);
        int dist0, dist1, d0, d1, i, c0, c1;
        logic order, found;
        logic [4:0] qdw [4][2];
        logic [4:0] qdl [4][2];
        qdw = '{'{5'd2, 5'd3}, '{5'd2, 5'd5}, '{5'd2, 5'd7}, '{5'd1, 5'd31}};
        qdl = '{'{5'd9, 5'd7}, '{5'd11, 5'd5}, '{5'd12, 5'd4}, '{5'd13, 5'd3}};
        dist0 = rel_dist(ih, ih.order_hints[8 * (int'(r0) - 1) +: 8], ih.order_hint);
        dist1 = rel_dist(ih, ih.order_hints[8 * (int'(r1) - 1) +: 8], ih.order_hint);
        if (dist0 < 0) dist0 = -dist0;
        if (dist1 < 0) dist1 = -dist1;
        if (dist0 > 31) dist0 = 31;
        if (dist1 > 31) dist1 = 31;
        d0 = dist1; d1 = dist0;
        order = (d0 <= d1);
        i = 3; found = 1'b0;
        if (d0 != 0 && d1 != 0)
            for (int k = 0; k < 3; k++)
                if (!found) begin
                    c0 = int'(qdw[k][order]); c1 = int'(qdw[k][!order]);
                    if (order ? (d0 * c0 > d1 * c1) : (d0 * c0 < d1 * c1)) begin i = k; found = 1'b1; end
                end
        return {qdl[i][!order], qdl[i][order]};
    endfunction
    function automatic logic [1:0] gm_type_of(input ihdr_t ih, input logic [3:0] r);
        return ih.gm_type[2 * (int'(r) - 1) +: 2];
    endfunction

    // declared ahead of their first use: Vivado makes an implicit 1-bit net of an identifier used before its declaration
    typedef enum logic [4:0] {
        R_IDLE, R_SETUP,
        R_PAL, R_PAL_LAST,
        R_EDGE, R_IP_START, R_IP_W,
        R_CFL_L, R_CFL_D, R_CFL_START, R_CFL_W,
        R_RESID, R_DQ, R_DQ_LAST, R_ITX_START, R_ITX_W, R_ADD, R_ADD_LAST, R_ADD_LAST2,
        R_FIN, R_MIW
    } rst_t;
    rst_t rs;
    logic sb_start, blk_take;
    logic [MIW-1:0] sb_r, sb_c;
    logic [5:0] sb_size4;
    logic [11:0] pix_max;
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
    logic [MIW-1:0] a_r_uv, a_c_uv, l_r_uv, l_c_uv;           // 7.11.2.8 chroma neighbour positions
    always_comb begin
        a_r_uv = b.r - MIW'(1); a_c_uv = b.c;
        if (hdr.ssx && !b.c[0]) a_c_uv = b.c + MIW'(1);
        if (hdr.ssy && b.r[0]) a_r_uv = b.r - MIW'(2);
        l_r_uv = b.r; l_c_uv = b.c - MIW'(1);
        if (hdr.ssx && b.c[0]) l_c_uv = b.c - MIW'(2);
        if (hdr.ssy && !b.r[0]) l_r_uv = b.r + MIW'(1);
    end
    logic [5:0] wr_n;
    assign wr_n = (bw4 > 6'd16) ? 6'd16 : bw4;
    // read address by state: each state's read lands in mm_rdata during the next state
    always_comb begin
        case (bst)
            B_FT0: mm_addr = mm_index(b.r - MIW'(1), b.c);
            B_FT1: mm_addr = mm_index(b.r, b.c - MIW'(1));
            B_FT2: mm_addr = mm_index(a_r_uv, a_c_uv);
            B_FT3: mm_addr = mm_index(l_r_uv, l_c_uv);
            B_FT4: mm_addr = mm_index(b.r, b.c);
            B_WR_W: mm_addr = (bw4 > 6'd16 && !wr_j) ? mm_index(b.r + MIW'(wr_i), b.c + MIW'(16)) : mm_index(b.r + MIW'(wr_i) + MIW'(1), b.c);
            default: mm_addr = mm_index(b.r + MIW'(wr_i), b.c + (wr_j ? MIW'(16) : MIW'(0)));   // B_WR: the word being merged
        endcase
    end

    assign mi_blk_r = b.r; assign mi_blk_c = b.c; assign mi_blk_bw4 = bw4; assign mi_blk_bh4 = bh4;
    // cdef_idx bookkeeping: clear on sb_start (uses sb_r/sb_c), set with the block record (superblock-relative units)
    logic [MIW-1:0] sb_mask;
    assign sb_mask = hdr.sb128 ? ~MIW'(31) : ~MIW'(15);
    assign mi_cd_clr = sb_start;
    assign mi_cd_row64 = sb_start ? SBW'(sb_r >> 4) : SBW'((b.r & sb_mask) >> 4);
    assign mi_cd_col64 = sb_start ? SBW'(sb_c >> 4) : SBW'((b.c & sb_mask) >> 4);
    assign mi_cd_mask = hdr.sb128 ? b.cdef_units : 4'b0001;
    assign mi_cd_idx = b.cdef_idx;
    assign mi_blk_data = '{bsize: b.bsize, skip: b.skip, seg: b.seg, delta_lf: b.delta_lf};
    always_ff @(posedge clk) begin
        mm_we <= 1'b0; mi_blk_we <= 1'b0; mi_cd_we <= 1'b0; mc_start <= 1'b0; ii_req <= 1'b0;
        if (mc_done) mc_fin <= 1'b1;
        if (rst) begin
            bst <= B_IDLE; blk_ready <= 1'b0; mc_unsup <= 1'b0; rf_req_valid <= 1'b0; mc_fin <= 1'b0;
        end else case (bst)
            B_IDLE: if (blk_take) begin
                b <= ev_blk; blk_ready <= 1'b0; bst <= B_FT0;
            end
            // four neighbour-mode reads (issued by the combinational mm_addr; captured in the following state):
            // above Y, left Y, above UV, left UV
            B_FT0: if (!mi_blk_busy && !mi_blk_we) begin mi_blk_we <= 1'b1; mi_cd_we <= b.cdef_valid; bst <= B_FT1; end   // per-4x4 LF state + cdef_idx
            B_FT1: begin nb_ay <= mm_rdata[8 * b.c[3:0] +: 8]; bst <= B_FT2; end
            B_FT2: begin nb_ly <= mm_rdata[8 * 4'(b.c - MIW'(1)) +: 8]; bst <= B_FT3; end
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
                mm_wdata <= mm_merge(mm_rdata, wr_j ? 4'd0 : b.c[3:0], wr_n, {b.uvmode, b.ymode[3:0]});
                bst <= B_WR_W;
            end
            B_WR_W: begin
                // advance: second word of a >16-wide block, next row, or done (the next word's read is in flight)
                if (bw4 > 6'd16 && !wr_j) begin
                    wr_j <= 1'b1; bst <= B_WR;
                end else if (wr_i + 6'd1 < bh4) begin
                    wr_j <= 1'b0; wr_i <= wr_i + 6'd1; bst <= B_WR;
                end else if (b.intrabc) begin
                    bst <= B_IBC_INIT;
                end else if (b.is_inter) begin
                    bst <= B_MC_INIT;
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
                ib_bx <= PXW'((PXW'(b.c) >> ibc_sx) << 2); ib_by <= PXW'((PXW'(b.r) >> ibc_sy) << 2);
                mvx2 = (20'(b.mv_col) * 20'sd2) >>> ibc_sx;                   // (2 * mv[1]) >> subX
                mvy2 = (20'(b.mv_row) * 20'sd2) >>> ibc_sy;
                ib_fx <= mvx2[3:0]; ib_fy <= mvy2[3:0];                       // (p >> 6) & SUBPEL_MASK
                ib_x0 <= PXS'(signed'({2'b0, PXW'((PXW'(b.c) >> ibc_sx) << 2)})) + PXS'(mvx2 >>> 4);
                ib_y0 <= PXS'(signed'({2'b0, PXW'((PXW'(b.r) >> ibc_sy) << 2)})) + PXS'(mvy2 >>> 4);
                ib_lastx <= PXW'((((PXW'(hdr.mi_cols) << 2) + PXW'(ibc_sx)) >> ibc_sx) - PXW'(1));   // RefUpscaledWidth[-1] = MiCols * MI_SIZE
                ib_lasty <= PXW'((((PXW'(hdr.mi_rows) << 2) + PXW'(ibc_sy)) >> ibc_sy) - PXW'(1));
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
            // ---- inter prediction (stage 2a): see the declarations above
            B_MC_INIT: if (rs == R_IDLE) begin
                logic comp;
                logic [9:0] wts;
                comp = (b.ref1 >= 4'd1) && (b.ref1 <= 4'd7);
                mc_comp <= comp;
                mc_bmode <= (b.compound_type == 3'd4) ? 2'd3 : 2'd2;
                wts = dist_weights(ihdr, b.ref0, b.ref1);
                mc_wt0 <= wts[4:0]; mc_wt1 <= wts[9:5];
                if (b.motion_mode == 2'd2 || (comp && b.compound_type == 3'd3)
                    || ihdr.ref_scaled[int'(b.ref0) - 1] || (comp && ihdr.ref_scaled[int'(b.ref1) - 1])
                    || ((b.ymode == 5'd15 || b.ymode == 5'd23) && !ihdr.force_integer_mv && bw4 >= 6'd2 && bh4 >= 6'd2
                        && (gm_type_of(ihdr, b.ref0) > 2'd1 || (comp && gm_type_of(ihdr, b.ref1) > 2'd1))))
                    mc_unsup <= 1'b1;
                mc_pl <= 2'd0; ob_active <= 1'b0; ob_done <= 1'b0; mb_active <= 1'b0;
                ii_active <= b.interintra; ii_wedge <= b.interintra && b.wedge_interintra;
                ii_mode <= (b.interintra_mode == 2'd0) ? 4'd0 : (b.interintra_mode == 2'd1) ? 4'd1 : (b.interintra_mode == 2'd2) ? 4'd2 : 4'd9;
                mc_post <= 4'd14 - ((hdr.bit_depth == 4'd12) ? 4'd5 : 4'd3) - 4'd7;        // compound: InterRound1 = 7
                if ((comp && (b.compound_type == 3'd0 || b.compound_type == 3'd1)) || (b.interintra && b.wedge_interintra)) begin
                    logic [10:0] cb;
                    logic [1:0] shape;
                    logic [7:0] bw, bh;
                    mb_active <= comp; mb_diff <= comp && (b.compound_type == 3'd1);
                    bw = 8'(num4x4w(b.bsize)) << 2; bh = 8'(num4x4h(b.bsize)) << 2;
                    shape = (bh4 > bw4) ? 2'd0 : (bh4 < bw4) ? 2'd1 : 2'd2;
                    cb = wedge_codebook(shape, b.wedge_index);
                    mk_d <= cb[10:8];
                    mk_xoff <= 6'(8'd32 - ((8'(cb[7:4]) * bw) >> 3)); mk_yoff <= 6'(8'd32 - ((8'(cb[3:0]) * bh) >> 3));
                    mk_inv <= (b.wedge_sign != wedge_flip(b.bsize, b.wedge_index));
                    mk_x <= 7'd0; mk_y <= 7'd0;
                    bst <= ((comp && b.compound_type == 3'd0) || (b.interintra && b.wedge_interintra)) ? B_MC_MASK : B_MC_PL;
                end else bst <= B_MC_PL;
            end
            B_MC_MASK: begin
                // the wedge mask of the block, four luma samples a cycle (wedge blocks are at most 32 x 32)
                if (mk_x + 7'd4 < (8'(num4x4w(b.bsize)) << 2)) mk_x <= mk_x + 7'd4;
                else begin mk_x <= 7'd0; if (mk_y + 7'd1 < (8'(num4x4h(b.bsize)) << 2)) mk_y <= mk_y + 7'd1; else bst <= B_MC_PL; end
            end
            B_MC_PL: begin
                logic sx, sy;
                logic [4:0] psz;
                logic cmv_units;
                sx = (mc_pl != 2'd0) && hdr.ssx; sy = (mc_pl != 2'd0) && hdr.ssy;
                psz = subsampled_size(b.bsize, sx, sy);
                mc_sx <= sx; mc_sy <= sy; mc_psz <= psz;
                mc_pw <= 8'(num4x4w(psz)) << 2; mc_ph <= 8'(num4x4h(psz)) << 2;
                mc_bx <= PXW'((PXW'(b.c) >> sx) << 2); mc_by <= PXW'((PXW'(b.r) >> sy) << 2);
                // chroma of a sub-8x8 luma block: per-luma-4x4 units from b.cmv unless someUseIntra
                cmv_units = (mc_pl != 2'd0) && ((bw4 == 6'd1 && hdr.ssx) || (bh4 == 6'd1 && hdr.ssy)) && !b.cmv_intra;
                mc_cmv <= cmv_units;
                if (cmv_units) begin
                    mc_uw <= (8'(num4x4w(b.bsize)) << 2) >> sx; mc_uh <= (8'(num4x4h(b.bsize)) << 2) >> sy;
                    mc_uc <= (bw4 == 6'd1 && hdr.ssx) ? 2'd2 : 2'd1; mc_ur <= (bh4 == 6'd1 && hdr.ssy) ? 2'd2 : 2'd1;
                end else begin
                    mc_uw <= 8'(num4x4w(psz)) << 2; mc_uh <= 8'(num4x4h(psz)) << 2;
                    mc_uc <= 2'd1; mc_ur <= 2'd1;
                end
                mc_ui <= 2'd0; mc_uj <= 2'd0;
                if (b.interintra) begin
                    ii_pl <= mc_pl; ii_x <= PXW'((PXW'(b.c) >> sx) << 2); ii_y <= PXW'((PXW'(b.r) >> sy) << 2);
                    ii_txsz <= max_tx_size_rect(psz);
                    ii_lsh <= ii_scale_shift(8'(num4x4w(psz)) << 2, 8'(num4x4h(psz)) << 2);
                    ii_req <= 1'b1; bst <= B_MC_II;
                end else bst <= B_MC_UNIT;
            end
            B_MC_II: if (ii_done) bst <= B_MC_UNIT;                              // the intra prediction is in the frame memory
            B_MC_UNIT: begin
                int k;
                logic [75:0] u;
                k = int'(mc_ui) * int'(mc_uc) + int'(mc_uj);
                u = b.cmv[76 * k +: 76];
                if (mc_cmv) begin
                    u_ref0 <= u[3:0]; u_ref1 <= u[7:4]; u_mv0r <= u[23:8]; u_mv0c <= u[39:24]; u_mv1r <= u[55:40]; u_mv1c <= u[71:56];
                    u_i0 <= u[73:72]; u_i1 <= u[75:74];
                end else begin
                    u_ref0 <= b.ref0; u_ref1 <= b.ref1; u_mv0r <= 16'(b.mv_row); u_mv0c <= 16'(b.mv_col);
                    u_mv1r <= 16'(b.mv1_row); u_mv1c <= 16'(b.mv1_col); u_i0 <= b.interp0; u_i1 <= b.interp1;
                end
                mc_ux <= mc_bx + PXW'(mc_uj) * PXW'(mc_uw); mc_uy <= mc_by + PXW'(mc_ui) * PXW'(mc_uh);
                mc_nsc <= 2'((mc_uw + 8'd63) >> 6); mc_nsr <= 2'((mc_uh + 8'd63) >> 6);
                mc_sr <= 2'd0; mc_sc <= 2'd0; mc_l <= 1'b0;
                bst <= B_MC_STRIP;
            end
            B_MC_STRIP: begin
                // the strip's geometry and this list's reference position (7.11.3.3 for an unscaled reference:
                // integer part x + ((2 mv) >> subX) >> 4, phase its low 4 bits)
                logic [7:0] rem_w, rem_h;
                logic signed [19:0] mvx2, mvy2;
                logic [3:0] rref;
                logic [2:0] slot;
                logic [15:0] dw, dh;
                logic [PXW-1:0] px, py;
                rem_w = mc_uw - (8'(mc_sc) << 6); rem_h = mc_uh - (8'(mc_sr) << 6);
                mc_sw <= (rem_w > 8'd64) ? 7'd64 : 7'(rem_w); mc_sh <= (rem_h > 8'd64) ? 7'd64 : 7'(rem_h);
                px = mc_ux + (PXW'(mc_sc) << 6); py = mc_uy + (PXW'(mc_sr) << 6);
                mc_px <= px; mc_py <= py;
                mvx2 = (20'(mc_l ? u_mv1c : u_mv0c) * 20'sd2) >>> mc_sx;
                mvy2 = (20'(mc_l ? u_mv1r : u_mv0r) * 20'sd2) >>> mc_sy;
                mc_fx <= mvx2[3:0]; mc_fy <= mvy2[3:0];
                rf_x0 <= 17'(signed'({2'b0, px})) + 17'(mvx2 >>> 4);
                rf_y0 <= 17'(signed'({2'b0, py})) + 17'(mvy2 >>> 4);
                rref = mc_l ? u_ref1 : u_ref0;
                slot = ref_slot[3 * (int'(rref) - 1) +: 3];
                rf_base <= ref_planes[72 * (3 * int'(slot) + int'(mc_pl)) +: 40];
                rf_stride <= ref_planes[72 * (3 * int'(slot) + int'(mc_pl)) + 40 +: 32];
                dw = ref_dims[32 * int'(slot) +: 16]; dh = ref_dims[32 * int'(slot) + 16 +: 16];
                rf_lastx <= ((dw + 16'(mc_sx)) >> mc_sx) - 16'd1; rf_lasty <= ((dh + 16'(mc_sy)) >> mc_sy) - 16'd1;
                rf_lst <= mc_l;
                mc_mode <= ob_active ? 3'd0 : mc_l ? (mb_active ? 3'd4 : 3'(mc_bmode)) : (mc_comp ? 3'd1 : 3'd0);
                rf_req_valid <= 1'b1;
                bst <= B_MC_FETCH;
            end
            B_MC_FETCH: begin
                if (rf_req_valid && rf_req_ready) rf_req_valid <= 1'b0;       // accepted
                if (rf_done) begin mc_start <= 1'b1; bst <= B_MC_FILT; end
            end
            B_MC_FILT: if ((mc_done || mc_fin) && !ob_rd_v && !ob_rd2_v && !ob_wr_v && !mb_a_v && !mb_b_v) begin   // the run finished and the blend pipelines drained
                mc_fin <= 1'b0;
                if (ob_active) begin ob_k <= ob_k + 3'd1; bst <= B_MC_OB; end
                else if (mc_comp && !mc_l) begin mc_l <= 1'b1; bst <= B_MC_STRIP; end
                else if (mc_sc + 2'd1 < mc_nsc) begin mc_sc <= mc_sc + 2'd1; mc_l <= 1'b0; bst <= B_MC_STRIP; end
                else if (mc_sr + 2'd1 < mc_nsr) begin mc_sc <= 2'd0; mc_sr <= mc_sr + 2'd1; mc_l <= 1'b0; bst <= B_MC_STRIP; end
                else if (mc_uj + 2'd1 < mc_uc) begin mc_uj <= mc_uj + 2'd1; bst <= B_MC_UNIT; end
                else if (mc_ui + 2'd1 < mc_ur) begin mc_uj <= 2'd0; mc_ui <= mc_ui + 2'd1; bst <= B_MC_UNIT; end
                else bst <= B_MC_PLEND;
            end
            B_MC_PLEND: begin
                if (b.motion_mode == 2'd1 && !ob_done) begin ob_done <= 1'b1; ob_pass <= 1'b0; ob_k <= 3'd0; bst <= B_MC_OB; end
                else begin
                    ob_done <= 1'b0; ob_active <= 1'b0;
                    if (mc_pl < (b.has_chroma ? 2'd2 : 2'd0)) begin mc_pl <= mc_pl + 2'd1; bst <= B_MC_PL; end
                    else begin blk_ready <= 1'b1; bst <= B_IDLE; end
                end
            end
            B_MC_OB: begin
                // the next overlap neighbour of this pass (above: entries 0 .. 3, needs the plane block >= BLOCK_8X8; left: 4 .. 7)
                logic [50:0] e;
                logic [7:0] pw, ph, sw4;
                e = b.obmc[51 * (int'(ob_pass) * 4 + int'(ob_k)) +: 51];
                ob_active <= 1'b1;
                if (ob_k == 3'd4 || !e[0] || (!ob_pass && mc_psz < 5'd3)) begin
                    if (!ob_pass) begin ob_pass <= 1'b1; ob_k <= 3'd0; end
                    else begin ob_active <= 1'b0; bst <= B_MC_PLEND; end
                end else begin
                    sw4 = 8'(e[10:6]) << 2;                                         // step4 * MI_SIZE
                    if (!ob_pass) begin
                        pw = (sw4 >> mc_sx); if (pw > mc_pw) pw = mc_pw;             // min(w, (step4 * 4) >> subX)
                        ph = (8'd32 >> mc_sy); if (ph > (mc_ph >> 1)) ph = mc_ph >> 1; // min(h >> 1, 32 >> subY)
                        mc_ux <= mc_bx + PXW'((PXW'(e[5:1]) << 2) >> mc_sx); mc_uy <= mc_by;
                        ob_mlen <= 6'(ph);
                    end else begin
                        pw = (8'd32 >> mc_sx); if (pw > (mc_pw >> 1)) pw = mc_pw >> 1;
                        ph = (sw4 >> mc_sy); if (ph > mc_ph) ph = mc_ph;
                        mc_ux <= mc_bx; mc_uy <= mc_by + PXW'((PXW'(e[5:1]) << 2) >> mc_sy);
                        ob_mlen <= 6'(pw);
                    end
                    mc_uw <= pw; mc_uh <= ph;
                    u_ref0 <= e[14:11]; u_mv0r <= e[30:15]; u_mv0c <= e[46:31]; u_i0 <= e[48:47]; u_i1 <= e[50:49];
                    mc_nsc <= 2'd1; mc_nsr <= 2'd1; mc_sr <= 2'd0; mc_sc <= 2'd0; mc_l <= 1'b0;
                    bst <= B_MC_STRIP;
                end
            end
            default: bst <= B_IDLE;
        endcase
    end

    // OBMC read-modify-write pipeline: A = the prediction group and its mask values (the frame memory read is issued
    // from these registers), B = the blended group to write. One group per cycle.
    always_ff @(posedge clk) begin
        ob_rd_v <= 1'b0; ob_rd2_v <= 1'b0; ob_wr_v <= 1'b0;
        if (!rst) begin
            if (mc_out_valid && (ob_active || ii_active)) begin
                ob_rd_v <= 1'b1;
                ob_a_pix <= mc_out_pix4 << (12 * mc_osh);
                ob_a_be <= mc_out_mask << mc_osh;
                ob_a_x <= {mc_ox[PXW-1:2], 2'b00}; ob_a_y <= mc_py + PXW'(mc_out_y);
                for (int l = 0; l < 4; l++) begin
                    logic [6:0] i, j;
                    i = 7'(mb_by); j = 7'(mb_bx) + 7'(l) - 7'(mc_osh);                     // block-relative position (plane samples)
                    if (ob_active) ob_a_m[l] <= ob_pass ? obmc_mask(ob_mlen, 6'(mc_out_x + 7'(l) - 7'(mc_osh))) : obmc_mask(ob_mlen, 6'(mc_out_y));
                    else if (ii_wedge) ob_a_m[l] <= 7'd0;                                  // from the wedge buffer, in the A cycle
                    else case (ii_mode)
                        4'd1: ob_a_m[l] <= ii_weight(i << ii_lsh);                          // II_V_PRED
                        4'd2: ob_a_m[l] <= ii_weight(j << ii_lsh);                          // II_H_PRED
                        4'd9: ob_a_m[l] <= ii_weight(((i < j) ? i : j) << ii_lsh);          // II_SMOOTH_PRED
                        default: ob_a_m[l] <= 7'd32;                                        // II_DC_PRED
                    endcase
                end
            end
            // the frame memory read issued from the A registers lands one cycle later (registered read): A2 holds the
            // group meanwhile, and the blend uses fb_rdata4 in the A2 cycle
            ob_rd2_v <= ob_rd_v; ob_a2_pix <= ob_a_pix; ob_a2_x <= ob_a_x; ob_a2_y <= ob_a_y; ob_a2_be <= ob_a_be;
            if (ii_active && ii_wedge) ob_a2_m <= mb_wm;                                   // the wedge buffer's values for the A group
            else ob_a2_m <= ob_a_m;
            if (ob_rd2_v) begin
                ob_wr_v <= 1'b1; ob_b_x <= ob_a2_x; ob_b_y <= ob_a2_y; ob_b_be <= ob_a2_be;
                for (int l = 0; l < 4; l++) begin
                    logic [19:0] v;
                    v = 20'(ob_a2_m[l]) * 20'(fb_rdata4[l*12 +: 12]) + (20'd64 - 20'(ob_a2_m[l])) * 20'(ob_a2_pix[l*12 +: 12]);
                    ob_b_pix[l*12 +: 12] <= 12'((v + 20'd32) >> 6);
                end
            end
        end
    end

    // mask bank ports: the wedge fill (B_MC_MASK) writes four luma samples a cycle; a masked block's pair output reads
    // the mask of its group in the same cycle (luma: one value per lane; 4:2:0 chroma: the four luma values of each
    // lane's 2x2 -> both row-parity banks, two column words per x bank; 4:2:2: the two of its row) and, for DIFFWTD
    // luma, writes the mask it computes from the pair. The data lands with the A stage, which blends; B writes.
    logic [PXW-1:0] mb_bx, mb_by;                            // block-relative position of the output group (plane samples)
    assign mb_bx = mc_px - mc_bx + PXW'(mc_out_x);
    assign mb_by = mc_py - mc_by + PXW'(mc_out_y);
    logic [6:0] mb_dm [4];                                   // DIFFWTD luma: the mask of the current group
    logic [6:0] wv [4];                                      // the wedge fill's four master values
    always_comb begin
        logic [10:0] rowbase, colw;
        rowbase = 11'd0; colw = 11'd0;
        for (int k = 0; k < 8; k++) begin mk_we[k] = 1'b0; mk_waddr[k] = 11'd0; mk_wdata[k] = 7'd0; mk_ra0[k] = 11'd0; mk_ra1[k] = 11'd0; end
        for (int l = 0; l < 4; l++) mb_dm[l] = 7'd0;
        for (int l = 0; l < 4; l++) wv[l] = wedge_master(mk_d, 6'(mk_yoff + 6'(mk_y)), 6'(mk_xoff + 6'(mk_x) + 6'(l)));
        if (bst == B_MC_MASK) begin
            for (int l = 0; l < 4; l++) begin
                mk_we[(mk_y[0] ? 4 : 0) + l] = 1'b1;
                mk_waddr[(mk_y[0] ? 4 : 0) + l] = 11'(mk_y[6:1]) * 11'd32 + 11'(mk_x[6:2]);
                mk_wdata[(mk_y[0] ? 4 : 0) + l] = mk_inv ? 7'd64 - wv[l] : wv[l];
            end
        end
        if (mc_out_valid && (mb_active || (ii_active && ii_wedge))) begin
            if (mc_pl == 2'd0 || !(mc_sx || mc_sy)) begin
                // luma (or 4:4:4 chroma): the row's parity banks, one lane per x bank
                rowbase = 11'(mb_by[PXW-1:1]) * 11'd32 + 11'(mb_bx[PXW-1:2]);
                for (int l = 0; l < 4; l++) begin
                    mk_ra0[(mb_by[0] ? 4 : 0) + l] = rowbase;
                    if (mc_pl == 2'd0 && mb_active && mb_diff) begin
                        mb_dm[l] = diff_mask(signed'(mc_out_pair[16*l +: 16]), signed'(mc_out_pair[64 + 16*l +: 16]), hdr.bit_depth, mc_post, b.mask_type);
                        mk_we[(mb_by[0] ? 4 : 0) + l] = 1'b1; mk_waddr[(mb_by[0] ? 4 : 0) + l] = rowbase; mk_wdata[(mb_by[0] ? 4 : 0) + l] = mb_dm[l];
                    end
                end
            end else begin
                // subsampled chroma: luma columns 2x .. 2x + 7 = column words 2k and 2k + 1 of every x bank (k = x >> 2)
                colw = 11'(mb_bx[PXW-1:2]) << 1;
                if (mc_sy) begin                                   // 4:2:0: luma rows 2y and 2y + 1 = row word y, both parities
                    rowbase = 11'(mb_by) * 11'd32;
                    for (int k = 0; k < 8; k++) begin mk_ra0[k] = rowbase + colw; mk_ra1[k] = rowbase + colw + 11'd1; end
                end else begin                                     // 4:2:2: luma row y, its parity's banks
                    rowbase = 11'(mb_by[PXW-1:1]) * 11'd32;
                    for (int k = 0; k < 4; k++) begin mk_ra0[(mb_by[0] ? 4 : 0) + k] = rowbase + colw; mk_ra1[(mb_by[0] ? 4 : 0) + k] = rowbase + colw + 11'd1; end
                end
            end
        end
    end
    // the masked blend pipeline: A (the mask data of the group lands, blend), B (write)
    logic        mb_a_sub, mb_a_sy, mb_a_p;                  // the A group's plane kind: subsampled, 4:2:0, row parity
    logic [6:0]  mb_wm [4];                                  // the mask values of the A group's lanes, from the banks
    logic [8:0] mb_sum4 [4], mb_sum2 [4];                    // the 4:2:0 / 4:2:2 luma sums of each lane's group
    always_comb begin
        int hi;
        hi = mb_a_p ? 4 : 0;
        for (int l = 0; l < 4; l++) begin
            int lo;
            lo = ((l & 1) != 0) ? 2 : 0;                          // lanes 0, 2 use x banks 0 / 1; lanes 1, 3 use 2 / 3
            mb_sum4[l] = (l < 2) ? 9'(mk_q0[lo]) + 9'(mk_q0[lo + 1]) + 9'(mk_q0[4 + lo]) + 9'(mk_q0[5 + lo])
                                 : 9'(mk_q1[lo]) + 9'(mk_q1[lo + 1]) + 9'(mk_q1[4 + lo]) + 9'(mk_q1[5 + lo]);
            mb_sum2[l] = (l < 2) ? 9'(mk_q0[hi + lo]) + 9'(mk_q0[hi + lo + 1]) : 9'(mk_q1[hi + lo]) + 9'(mk_q1[hi + lo + 1]);
            if (!mb_a_sub) mb_wm[l] = mk_q0[hi + l];
            else if (mb_a_sy) mb_wm[l] = 7'((mb_sum4[l] + 9'd2) >> 2);   // 4:2:0: four values, column word by lane >> 1
            else mb_wm[l] = 7'((mb_sum2[l] + 9'd1) >> 1);                 // 4:2:2: two values of the row's parity
        end
    end
    logic [6:0]  mb_a_dm [4];
    always_ff @(posedge clk) begin
        mb_a_v <= 1'b0; mb_b_v <= 1'b0;
        if (!rst) begin
            if (mc_out_valid && (ii_active && ii_wedge)) begin                  // flags only: the RMW path blends
                mb_a_sub <= (mc_pl != 2'd0) && (mc_sx || mc_sy); mb_a_sy <= mc_sy; mb_a_p <= mb_by[0];
            end
            if (mc_out_valid && mb_active) begin
                mb_a_v <= 1'b1; mb_a_pair <= mc_out_pair;
                mb_a_x <= {mc_ox[PXW-1:2], 2'b00}; mb_a_y <= mc_py + PXW'(mc_out_y); mb_a_be <= mc_out_mask << mc_osh;
                mb_a_sub <= (mc_pl != 2'd0) && (mc_sx || mc_sy); mb_a_sy <= mc_sy; mb_a_p <= mb_by[0];
                mb_a_dm <= mb_dm;
            end
            if (mb_a_v) begin
                mb_b_v <= 1'b1; mb_b_x <= mb_a_x; mb_b_y <= mb_a_y; mb_b_be <= mb_a_be;
                for (int l = 0; l < 4; l++) begin
                    logic [6:0] m;
                    logic signed [23:0] v;
                    logic signed [15:0] p0, p1;
                    m = (mc_pl == 2'd0 && mb_diff) ? mb_a_dm[l] : mb_wm[l];
                    p0 = signed'(mb_a_pair[16*l +: 16]); p1 = signed'(mb_a_pair[64 + 16*l +: 16]);
                    v = (24'(signed'({1'b0, m})) * 24'(p0) + 24'(signed'({1'b0, 7'd64 - m})) * 24'(p1) + (24'sd1 <<< (mc_post + 4'd5))) >>> (mc_post + 4'd6);
                    mb_b_pix[l*12 +: 12] <= (v < 24'sd0) ? 12'd0 : (v > 24'(pix_max)) ? pix_max : 12'(v);
                end
            end
        end
    end

    // ================================================================ BlockDecoded flags + MaxLuma
    // [plane][y+1] is a 34-bit row, bit x+1, for y,x in -1..32. Rows, not bits, are the unit of update: a
    // transform block marks its area with up to 16 row ORs of a shifted mask. (Per-bit writes with computed
    // indices were 256 decoders over 3,468 cells and synthesis never finished.)
    // [plane][y+1] is a 34-bit row, bit x+1, for y,x in -1..32: 102 rows (plane*34 + y+1) in a dual-read
    // distributed RAM. A row not written since the superblock start reads as its 5.11.3 initial value
    // (bdf_wr tracks that), so sb_start clears 102 bits, not 102 rows. A transform block marks its area in
    // R_FIN with one read-modify-write per row (step_y clocks). Was 3,468 flip-flops of OR/mux logic (5k LUTs,
    // 2026-10-13); before that, per-bit writes with computed indices never finished synthesis.
    (* ram_style = "distributed" *) logic [33:0] bdf_m [0:127];
    logic [101:0] bdf_wr;
    logic [6:0]   bdf_a_rd, bdf_a_bl, bdf_a_fin;      // row addresses: read port A (above-right / FIN rmw), below-left, FIN write
    logic [5:0]   bdf_y_rd;
    logic [33:0]  bdf_row_a, bdf_row_bl;
    logic [4:0]   fy;                                  // FIN row counter
    logic [MIW-1:0] l_sb_r, l_sb_c;                    // the superblock's position, latched at sb_start (sb_r/sb_c are the event head's)
    assign sb_size4 = hdr.sb128 ? 6'd32 : 6'd16;
    // 5.11.3 clear_block_decoded_flags: the row a superblock starts with (above row decoded up to the frame
    // edge; left column available where the frame extends, the corner below the superblock's last row cleared)
    function automatic logic [33:0] bdf_init_row(input logic [1:0] p, input logic [5:0] yy1);   // yy1 = y+1
        logic sx, sy;
        logic [MIW-1:0] sbw4, sbh4;
        logic [5:0] lim_x, lim_y;
        logic [33:0] v;
        int yy;
        sx = (p != 0) && hdr.ssx; sy = (p != 0) && hdr.ssy;
        sbw4 = (hdr.mi_col_end - l_sb_c) >> sx;
        sbh4 = (hdr.mi_row_end - l_sb_r) >> sy;
        lim_x = sb_size4 >> sx; lim_y = sb_size4 >> sy;
        yy = int'(yy1) - 1;
        v = '0;
        if (yy <= int'(lim_y)) begin
            if (yy < 0) begin
                for (int xx = -1; xx <= 32; xx++) if (xx <= int'(lim_x) && (xx < 0 || xx < int'(sbw4))) v[xx + 1] = 1'b1;
            end else if (yy < int'(sbh4) && yy != int'(lim_y)) v[0] = 1'b1;
        end
        return v;
    endfunction
    logic [PXW-1:0] max_luma_w, max_luma_h;

    // ================================================================ transform-block state
    tx_rec_t     t;
    logic        sub_x, sub_y;
    logic [6:0]  tw_px, th_px;                             // transform width / height in pixels
    logic [2:0]  l2w, l2h;
    logic [5:0]  tw_c, th_c;                               // coefficient dims: min(32, w/h)
    logic [4:0]  step_x, step_y;                           // in 4x4 units
    logic [MIW-1:0] mi_row, mi_col;                           // luma MI position of the tx block
    logic [5:0]  sb_row_p, sb_col_p;                       // (row & sbMask) >> sub, plane units
    logic [33:0] fin_mask;                                 // the transform block's columns as bdf bits (x+1): one shifter
    assign fin_mask = ((34'd1 << step_x) - 34'd1) << (sb_col_p + 6'd1);
    logic [PXW-1:0] max_x, max_y;                             // last valid pixel index in the plane
    logic [PXW-1:0] base_x, base_y;                           // block origin in plane pixels
    logic        is_pal, is_cfl;
    logic [3:0]  mode;
    logic        have_l, have_a, have_ar, have_bl;
    logic [PXW-1:0] above_lim, left_lim;
    // The limits are constant over a transform block (geometry, the flag rows, the picture edge) and are only used
    // by the edge gather: registered in R_SETUP, so the per-cycle read coordinate is an add and a compare on
    // registers. Computed in front of the frame memory's address every cycle, the chain transform-size register ->
    // availability flag -> limit -> clamp -> window line -> UltraRAM address was the 80 MHz window build's worst
    // path (-1.03 ns, 2026-10-17).
    logic [PXW-1:0] above_lim_r, left_lim_r;
    always_comb begin
        bdf_y_rd  = (rs == R_FIN) ? 6'(sb_row_p + 6'd1 + 6'(fy)) : sb_row_p;
        bdf_a_rd  = 7'(t.plane) * 7'd34 + 7'(bdf_y_rd);
        bdf_a_bl  = 7'(t.plane) * 7'd34 + 7'(sb_row_p) + 7'(step_y) + 7'd1;
        bdf_a_fin = bdf_a_rd;
        bdf_row_a  = bdf_wr[bdf_a_rd] ? bdf_m[bdf_a_rd] : bdf_init_row(t.plane, bdf_y_rd);
        bdf_row_bl = bdf_wr[bdf_a_bl] ? bdf_m[bdf_a_bl] : bdf_init_row(t.plane, 6'(sb_row_p + 6'(step_y) + 6'd1));
    end
    always_ff @(posedge clk) if (rs == R_FIN) bdf_m[bdf_a_fin] <= bdf_row_a | fin_mask;
    // availability beyond the block (above-right, below-left) from the flags, and the sample limits
    // (its own block: geometry -> flag rows -> availability, no scheduling cycle)
    always_comb begin
        have_ar = bdf_row_a[sb_col_p + 6'(step_x) + 6'd1];     // read port A shows row sb_row_p outside R_FIN
        have_bl = bdf_row_bl[sb_col_p];
        above_lim = t.x + (have_ar ? PXW'(tw_px) * 2 : PXW'(tw_px)) - PXW'(1);
        if (above_lim > max_x) above_lim = max_x;
        left_lim = t.y + (have_bl ? PXW'(th_px) * 2 : PXW'(th_px)) - PXW'(1);
        if (left_lim > max_y) left_lim = max_y;
    end
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
        mi_row = MIW'((t.y << sub_y) >> 2); mi_col = MIW'((t.x << sub_x) >> 2);
        sb_row_p = 6'((mi_row & MIW'(hdr.sb128 ? 31 : 15)) >> sub_y);
        sb_col_p = 6'((mi_col & MIW'(hdr.sb128 ? 31 : 15)) >> sub_x);
        max_x = PXW'(((PXW'(hdr.mi_cols) << 2) >> sub_x) - PXW'(1));
        max_y = PXW'(((PXW'(hdr.mi_rows) << 2) >> sub_y) - PXW'(1));
        base_x = PXW'((PXW'(b.c) >> sub_x) << 2); base_y = PXW'((PXW'(b.r) >> sub_y) << 2);
        is_pal = !ii_run && ((t.plane == 2'd0) ? (b.pal_y != 4'd0) : (b.pal_uv != 4'd0));
        is_cfl = !ii_run && (t.plane != 2'd0) && (b.uvmode == UV_CFL_PRED);
        mode = ii_run ? ii_mode : (t.plane == 2'd0) ? b.ymode[3:0] : (is_cfl ? DC_PRED : b.uvmode);
        have_l = ((t.plane == 2'd0) ? av_l : av_lc) || (t.x > base_x);
        have_a = ((t.plane == 2'd0) ? av_u : av_uc) || (t.y > base_y);
        above_px = (PXW'(tw_px) < max_x - t.x + PXW'(1)) ? tw_px : 7'(max_x - t.x + PXW'(1));
        left_px  = (PXW'(th_px) < max_y - t.y + PXW'(1)) ? th_px : 7'(max_y - t.y + PXW'(1));
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
    itx2d #(.TW(TW), .NLANES(ITX_LANES)) u_itx (.clk, .rst, .coef_clr(itx_clr), .coef_we(itx_we), .coef_addr(itx_addr), .coef_data(itx_data),
                            .start(itx_start), .tx_sz(t.txsz), .tx_type(t.lossless ? 4'd0 : t.txtype), .bit_depth(hdr.bit_depth), .lossless(t.lossless),
                            .busy(itx_busy), .done(itx_done), .res_addr, .res_data, .res_data4);
    // ipred
    logic edge_we, edge_we4, ip_start, ip_busy, ip_done, ip_ov; logic [1:0] edge_side; logic [7:0] edge_idx; logic [11:0] edge_data; logic [47:0] edge_data4; logic [47:0] ip_pix4; logic [5:0] ip_ox, ip_oy;
    logic ft_sel;
    assign ft_sel = (is_dir(mode) && !((t.plane == 2'd0) && b.use_fi)) ? ((t.plane == 2'd0) ? ft_y : ft_uv) : 1'b0;
    ipred #(.PW(12)) u_ip (.clk, .rst, .edge_we, .edge_side, .edge_idx, .edge_data, .edge_we4, .edge_data4,
                           .start(ip_start), .mode, .use_filter_intra(!ii_run && (t.plane == 2'd0) && b.use_fi), .filter_intra_mode(b.fi_mode),
                           .angle_delta(ii_run ? 3'sd0 : (t.plane == 2'd0) ? b.angle_y : b.angle_uv), .log2w(l2w), .log2h(l2h), .bit_depth(hdr.bit_depth),
                           .have_left(have_l), .have_above(have_a), .filter_type(ft_sel), .edge_filter_en(rh.enable_intra_edge_filter),
                           .above_px, .left_px, .busy(ip_busy), .done(ip_done),
                           .out_valid(ip_ov), .out_x(ip_ox), .out_y(ip_oy), .out_pix4(ip_pix4));
    // cfl
    logic cl_lwe, cl_dwe, cl_start, cl_busy, cl_done, cl_ov; logic [9:0] cl_laddr, cl_daddr; logic [11:0] cl_ldata, cl_ddata, cl_pix; logic [4:0] cl_ox, cl_oy;
    logic [PXW-1:0] lx0, ly0;
    assign lx0 = PXW'(t.x << sub_x); assign ly0 = PXW'(t.y << sub_y);
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
    logic tx_ack;                                          // registered: the transform block was fully written (R_MIW)
    // cycle counters per transform-FSM state and per block-FSM state (simulation profiling; read through the
    // hierarchy by the testbench, never reset except by rst)
    logic [31:0] perf_rs [0:31];
    logic [31:0] perf_bst [0:31];
    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < 32; i++) perf_rs[i] <= 32'd0;
            for (int i = 0; i < 32; i++) perf_bst[i] <= 32'd0;
        end else begin
            perf_rs[rs] <= perf_rs[rs] + 32'd1;
            perf_bst[bst] <= perf_bst[bst] + 32'd1;
        end
    end
    logic tx_take, tx_pop;
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
    // above-row groups: entries 0 .. w+h-1 are gathered 4 per read (aligned at t.x); entries past AboveRow's last
    // available sample (above_lim) replicate it, which may lie in an earlier group (last_above)
    logic        ep_grp, ep_rep;                           // landing is a group; group replicates the left neighbour (no above)
    logic [PXW-1:0] ep_xb;                                    // x of the group's first sample
    logic [11:0] last_above;
    logic [8:0]  n_above;
    assign n_above = n_edges >> 1;                         // w + h: n_edges is registered in R_SETUP
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
    logic [PXW-1:0] cur_x, cur_y;
    logic [11:0] mid_m1, mid_p1, mid;
    assign mid = 12'(13'd1 << (hdr.bit_depth - 4'd1));
    assign mid_m1 = mid - 12'd1;
    assign mid_p1 = mid + 12'd1;
    assign pix_max = 12'((13'd1 << hdr.bit_depth) - 13'd1);

    // edge entry k -> (side, idx, is_const, const value, read coordinates)
    logic        e_const;
    logic [1:0]  e_side;
    logic [7:0]  e_idx;
    logic [11:0] e_val;
    logic [PXW-1:0] e_rx, e_ry;
    always_comb begin
        e_const = 1'b0; e_side = 2'd0; e_idx = 8'd0; e_val = 12'd0; e_rx = t.x; e_ry = t.y;
        if (ek < n_above) begin
            e_side = 2'd0; e_idx = 8'(ek);
            if (have_a) begin
                e_rx = (t.x + PXW'(ek) > above_lim_r) ? above_lim_r : t.x + PXW'(ek); e_ry = t.y - PXW'(1);
            end else if (have_l) begin
                e_rx = t.x - PXW'(1); e_ry = t.y;
            end else begin e_const = 1'b1; e_val = mid_m1; end
        end else if (ek < n_edges) begin
            e_side = 2'd1; e_idx = 8'(ek - n_above);
            if (have_l) begin
                e_rx = t.x - PXW'(1); e_ry = (t.y + PXW'(e_idx) > left_lim_r) ? left_lim_r : t.y + PXW'(e_idx);
            end else if (have_a) begin
                e_rx = t.x; e_ry = t.y - PXW'(1);
            end else begin e_const = 1'b1; e_val = mid_p1; end
        end else begin
            e_side = 2'd2; e_idx = 8'd0;
            if (have_a && have_l) begin e_rx = t.x - PXW'(1); e_ry = t.y - PXW'(1); end
            else if (have_a) begin e_rx = t.x; e_ry = t.y - PXW'(1); end
            else if (have_l) begin e_rx = t.x - PXW'(1); e_ry = t.y; end
            else begin e_const = 1'b1; e_val = mid; end
        end
    end

    // dequant datapath, two stages behind the Quant/qm reads: q_data / qm_data are valid for the position issued
    // the cycle before; stage 1 forms the effective quantiser (qm multiply) and registers it with the coefficient
    // and position, stage 2 multiplies, shifts, clamps and writes the itx bank. (In one cycle the two chained
    // multiplies into the bank RAM were the decoder's worst path, -3.8 ns at 100 MHz.)
    logic [17:0] q_eff, q_eff_r;
    logic signed [20:0] q_data_r;
    logic        dq_v2;                                     // stage 2 holds a coefficient to write
    logic [6:0]  dq_ci2, dq_cj2;
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
        dq_full = 40'(q_data_r) * 40'(signed'({1'b0, q_eff_r}));
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
    always_ff @(posedge clk) begin                            // add stage: sum of the group read last cycle, written next cycle
        a_valid <= p_valid && (rs == R_ADD || rs == R_ADD_LAST) && !rst;
        add_r <= add_out4; aadd_x_r <= padd_x_r; aadd_y_r <= padd_y_r;
    end
    always_ff @(posedge clk) begin                            // dequant stage 1 -> 2
        dq_v2 <= p_valid && (rs == R_DQ || rs == R_DQ_LAST) && !rst;
        dq_ci2 <= pci; dq_cj2 <= pcj; q_eff_r <= q_eff; q_data_r <= q_data;
    end
    // recon add datapath: residual columns cj .. cj+3 of row ci map to the aligned picture group starting at
    // add_xx (reversed when flip_lr); (ci, cj) is the group being read, (pci, pcj) the one being added and written
    logic [6:0]  add_xx, add_yy, padd_xx, padd_yy;
    assign padd_xx = flip_lr ? 7'(tw_px - 7'd4 - pcj) : pcj;
    assign padd_yy = flip_ud ? 7'(th_px - 7'd1 - pci) : pci;
    // The written group's picture position, registered with (pci, pcj): the flip arithmetic and the block offset
    // then sit a cycle ahead of the frame memory's write address (31 logic levels from the transform-size register
    // to the UltraRAM address pin was the core build's worst clock-crossing path at 100 MHz).
    logic [FBX-1:0] padd_x_r;
    logic [FBY-1:0] padd_y_r;
    // The sum is registered too and written the cycle after: the residual read out of the transform engine's
    // banks, the add, the clamp and the frame memory's write-port mux and capture were one path (0.00 ns at 80 MHz,
    // 2026-10-18). Groups of a block are each written once and read (for the prediction) before their own add, so
    // the later landing changes nothing; R_ADD_LAST2 drains the last write before R_FIN.
    logic           a_valid;
    logic [47:0]    add_r;
    logic [FBX-1:0] aadd_x_r;
    logic [FBY-1:0] aadd_y_r;
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
        fb4_we = 1'b0; fb4_plane = t.plane; fb4_x = '0; fb4_y = '0; fb4_wdata = 48'd0; fb4_be = 4'hF;
        edge_we = 1'b0; edge_side = ep_side; edge_idx = ep_idx; edge_data = ep_const ? ep_val : fb_rdata;
        edge_we4 = 1'b0;
        for (int l = 0; l < 4; l++) begin                                    // group lanes with the clamp to above_lim
            logic [PXW-1:0] gx;
            gx = ep_xb + PXW'(l);
            if (ep_const) edge_data4[l * 12 +: 12] = ep_val;
            else if (ep_rep) edge_data4[l * 12 +: 12] = fb_rdata;
            else if (gx <= above_lim_r) edge_data4[l * 12 +: 12] = fb_rdata4[l * 12 +: 12];
            else if (above_lim_r >= ep_xb) edge_data4[l * 12 +: 12] = fb_rdata4[(above_lim_r - ep_xb) * 12 +: 12];
            else edge_data4[l * 12 +: 12] = last_above;
        end
        itx_we = 1'b0; itx_addr = 10'd0; itx_data = '0;
        cl_lwe = 1'b0; cl_dwe = 1'b0; cl_laddr = 10'd0; cl_ldata = fb_rdata; cl_daddr = 10'd0; cl_ddata = fb_rdata;
        res_addr = 12'({pci[5:0], pcj[5:0]});                 // residual of the position read last cycle
        pm_plane = (t.plane != 2'd0); pm_x = 6'(t.x - base_x + PXW'(cj)); pm_y = 6'(t.y - base_y + PXW'(ci));
        q_addr = 10'(sc_ci) * 10'(tw_c) + 10'(sc_cj);    // registered read in coef_rd: data lands the next cycle
        qm_addr = 17'(int'(qm_lvl) * QM_LEVEL_STRIDE + ((t.plane != 2'd0) ? QM_PLANE_STRIDE : 0)) + 17'(qm_offset(t.txsz)) + 17'(sc_ci) * 17'(tw_c) + 17'(sc_cj);
        itx_clr = (rs == R_RESID);
        case (rs)
            R_PAL, R_PAL_LAST: begin
                // pm_idx holds the index for (px_i, px_j) issued last cycle
                if (px_valid) begin
                    fb2_we = 1'b1; fb2_x = FBX'(t.x + PXW'(px_j)); fb2_y = FBY'(t.y + PXW'(px_i));
                    fb2_wdata = pal_cols[12 * pm_idx +: 12];
                end
            end
            R_EDGE: begin
                if (ek < n_above) begin                                      // above row: one aligned group of 4 per read
                    if (have_a) begin fb_re = 1'b1; fb_x = FBX'(t.x + PXW'(ek)); fb_y = FBY'(t.y - PXW'(1)); end
                    else if (have_l) begin fb_re = 1'b1; fb_x = FBX'(t.x - PXW'(1)); fb_y = FBY'(t.y); end
                end else if (ek <= n_edges && !e_const) begin fb_re = 1'b1; fb_x = FBX'(e_rx); fb_y = FBY'(e_ry); end
                if (ep_valid && !ep_grp) edge_we = 1'b1;
                if (ep_valid && ep_grp) edge_we4 = 1'b1;
            end
            R_IP_W: if (ip_ov) begin fb4_we = 1'b1; fb4_x = FBX'(t.x + PXW'(ip_ox)); fb4_y = FBY'(t.y + PXW'(ip_oy)); fb4_wdata = ip_pix4; end
            R_CFL_L: begin
                // read luma (plane 0) at (lx0 + cj, ly0 + ci); write the previously read pixel into cfl
                fb_plane = 2'd0;
                if (ci < 7'(th_px << sub_y)) begin fb_re = 1'b1; fb_x = FBX'(lx0 + PXW'(cj)); fb_y = FBY'(ly0 + PXW'(ci)); end
                if (px_valid) begin cl_lwe = 1'b1; cl_laddr = {px_i[4:0], px_j[4:0]}; end
            end
            R_CFL_D: begin
                if (ci < 7'(th_px)) begin fb_re = 1'b1; fb_x = FBX'(t.x + PXW'(cj)); fb_y = FBY'(t.y + PXW'(ci)); end
                if (px_valid) begin cl_dwe = 1'b1; cl_daddr = {px_i[4:0], px_j[4:0]}; end
            end
            R_CFL_W: if (cl_ov) begin fb2_we = 1'b1; fb2_x = FBX'(t.x + PXW'(cl_ox)); fb2_y = FBY'(t.y + PXW'(cl_oy)); fb2_wdata = cl_pix; end
            // dequant: one coefficient per clock (address (ci, cj) out, the previous position's data in)
            R_DQ, R_DQ_LAST, R_ITX_START: if (dq_v2) begin itx_we = 1'b1; itx_addr = {dq_ci2[4:0], dq_cj2[4:0]}; itx_data = dq_out; end
            // residual add: read the group at (ci, cj) through port 1, write the previous group through port 4
            R_ADD, R_ADD_LAST, R_ADD_LAST2: begin
                if (rs == R_ADD) begin fb_re = 1'b1; fb_x = FBX'(t.x + PXW'(add_xx)); fb_y = FBY'(t.y + PXW'(add_yy)); end
                if (a_valid) begin fb4_we = 1'b1; fb4_x = aadd_x_r; fb4_y = aadd_y_r; fb4_wdata = add_r; end
            end
            default: ;
        endcase
        // intra block copy: the block FSM owns the frame-buffer ports while the transform FSM is idle
        case (bst)
            B_IBC_R0: begin fb_re = 1'b1; fb_plane = ib_pl; fb_x = FBX'(ib_rx0); fb_y = FBY'(ib_ry0); end
            B_IBC_R1: begin fb_re = ib_fx[3]; fb_plane = ib_pl; fb_x = FBX'(ib_rx1); fb_y = FBY'(ib_ry0); end
            B_IBC_R2: begin fb_re = ib_fy[3]; fb_plane = ib_pl; fb_x = FBX'(ib_rx0); fb_y = FBY'(ib_ry1); end
            B_IBC_R3: begin fb_re = ib_fx[3] && ib_fy[3]; fb_plane = ib_pl; fb_x = FBX'(ib_rx1); fb_y = FBY'(ib_ry1); end
            B_IBC_WR: begin fb2_we = 1'b1; fb2_plane = ib_pl; fb2_x = FBX'(ib_bx + PXW'(ib_j)); fb2_y = FBY'(ib_by + PXW'(ib_i)); fb2_wdata = ib_out; end
            default: ;
        endcase
        // motion compensation output: 4 samples at (strip origin + out_x, + out_y); a 2-wide unit at an x offset of 2
        // within its 4-group is lane-shifted (the frame memory's 4-wide port is aligned)
        mc_ox = mc_px + PXW'(mc_out_x);
        mc_osh = mc_ox[1:0];
        if (mc_out_valid && !ob_active && !ii_active && !(mb_active && mc_l)) begin
            fb4_we = 1'b1; fb4_plane = mc_pl;
            fb4_x = FBX'({mc_ox[PXW-1:2], 2'b00}); fb4_y = FBY'(mc_py + PXW'(mc_out_y));
            fb4_wdata = mc_out_pix4 << (12 * mc_osh); fb4_be = mc_out_mask << mc_osh;
        end
        // OBMC: read the group to blend (stage A), write the blended group (stage B)
        if (ob_rd_v) begin fb_re = 1'b1; fb_plane = mc_pl; fb_x = FBX'(ob_a_x); fb_y = FBY'(ob_a_y); end
        if (ob_wr_v) begin fb4_we = 1'b1; fb4_plane = mc_pl; fb4_x = FBX'(ob_b_x); fb4_y = FBY'(ob_b_y); fb4_wdata = ob_b_pix; fb4_be = ob_b_be; end
        if (mb_b_v) begin fb4_we = 1'b1; fb4_plane = mc_pl; fb4_x = FBX'(mb_b_x); fb4_y = FBY'(mb_b_y); fb4_wdata = mb_b_pix; fb4_be = mb_b_be; end
    end
    assign busy = (rs != R_IDLE) || (bst != B_IDLE) || !blk_ready || mi_blk_busy || mi_tx_busy;
    assign mi_tx_plane = t.plane; assign mi_tx_sz = t.txsz;
    assign mi_tx_row = MIW'(t.y >> 2); assign mi_tx_col = MIW'(t.x >> 2);          // plane 4x4 units
    assign mi_tx_w4 = step_x; assign mi_tx_h4 = step_y;

    // ---------------------------------------------------------------- sequential
    always_ff @(posedge clk) begin
        tx_ack <= 1'b0; itx_start <= 1'b0; ip_start <= 1'b0; cl_start <= 1'b0; mi_tx_we <= 1'b0; ii_done <= 1'b0;
        if (rst) begin
            rs <= R_IDLE; max_luma_w <= PXW'(0); max_luma_h <= PXW'(0); ii_run <= 1'b0;
            bdf_wr <= '0; fy <= '0; l_sb_r <= '0; l_sb_c <= '0;
        end else begin
            // 5.11.3 clear_block_decoded_flags at each superblock start
            if (sb_start) begin bdf_wr <= '0; l_sb_r <= sb_r; l_sb_c <= sb_c; end
            case (rs)
                R_IDLE: if (tx_take) begin
                    t <= ev_tx; rs <= R_SETUP;
                end else if (ii_req) begin
                    // inter-intra: the intra prediction of the plane block through the edge / ipred flow
                    t <= '{plane: ii_pl, x: ii_x, y: ii_y, txsz: ii_txsz, txtype: 4'd0, eob: 11'd0, skip: 1'b1, lossless: 1'b0, slot: 3'd0};
                    ii_run <= 1'b1; rs <= R_SETUP;
                end
                R_SETUP: begin
                    ci <= 7'd0; cj <= 7'd0; px_valid <= 1'b0; ek <= 9'd0; ep_valid <= 1'b0;
                    n_edges <= 2 * (9'(tw_px) + 9'(th_px));
                    above_lim_r <= above_lim; left_lim_r <= left_lim;
                    if (ii_run) rs <= R_EDGE;                           // inter-intra: the intra prediction of the block
                    else if (b.intrabc || b.is_inter) rs <= R_RESID;    // predicted whole by the block copy / motion compensation
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
                    if (ep_valid && ep_grp && !ep_const && !ep_rep && above_lim_r >= ep_xb && above_lim_r <= ep_xb + PXW'(3))
                        last_above <= fb_rdata4[(above_lim_r - ep_xb) * 12 +: 12];
                    if (ek < n_above) begin                                  // above row, groups of 4
                        ep_valid <= 1'b1; ep_grp <= 1'b1; ep_side <= 2'd0; ep_idx <= 8'(ek); ep_xb <= t.x + PXW'(ek);
                        ep_const <= !have_a && !have_l; ep_val <= mid_m1; ep_rep <= !have_a && have_l;
                        ek <= ek + 9'd4;
                    end else if (ek <= n_edges) begin
                        ep_valid <= 1'b1; ep_grp <= 1'b0; ep_side <= e_side; ep_idx <= e_idx; ep_const <= e_const; ep_val <= e_val;
                        ek <= ek + 9'd1;
                    end else begin
                        ep_valid <= 1'b0;
                        if (!ep_valid) begin ip_start <= 1'b1; rs <= R_IP_W; end
                    end
                end
                R_IP_W: if (ip_done) begin
                    ci <= 7'd0; cj <= 7'd0; px_valid <= 1'b0;
                    if (ii_run) begin ii_run <= 1'b0; ii_done <= 1'b1; rs <= R_IDLE; end
                    else rs <= is_cfl ? R_CFL_L : R_RESID;
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
                R_DQ_LAST: begin p_valid <= 1'b0; ci <= 7'd0; cj <= 7'd0; rs <= R_ITX_START; end
                R_ITX_START: begin itx_start <= 1'b1; rs <= R_ITX_W; end   // the last coefficient lands this cycle
                R_ITX_W: if (itx_done) begin p_valid <= 1'b0; rs <= R_ADD; end
                R_ADD: begin                             // read group (ci, cj..cj+3); the previous group is added and written this cycle
                    p_valid <= 1'b1; pci <= ci; pcj <= cj;
                    padd_x_r <= FBX'(t.x + PXW'(flip_lr ? 7'(tw_px - 7'd4 - cj) : cj));
                    padd_y_r <= FBY'(t.y + PXW'(flip_ud ? 7'(th_px - 7'd1 - ci) : ci));
                    if (cj + 7'd4 < tw_px) cj <= cj + 7'd4;
                    else if (ci + 7'd1 < th_px) begin cj <= 7'd0; ci <= ci + 7'd1; end
                    else rs <= R_ADD_LAST;
                end
                R_ADD_LAST: begin p_valid <= 1'b0; rs <= R_ADD_LAST2; end
                R_ADD_LAST2: begin rs <= R_FIN; end
                // ---- bookkeeping + ack
                R_FIN: begin
                    // mark the block's area: one row per clock, rows sb_row_p+1 .. sb_row_p+step_y (the RAM write
                    // and the row read on port A follow fy above)
                    bdf_wr[bdf_a_fin] <= 1'b1;
                    if (t.plane == 2'd0) begin max_luma_w <= t.x + PXW'(tw_px); max_luma_h <= t.y + PXW'(th_px); end
                    if (fy + 5'd1 >= step_y) begin fy <= '0; rs <= R_MIW; end
                    else fy <= fy + 5'd1;
                end
                R_MIW: if (!mi_tx_busy && !mi_tx_we) begin       // LoopfilterTxSizes over the transform block's area
                    mi_tx_we <= 1'b1; tx_ack <= 1'b1; rs <= R_IDLE;
                end
                default: rs <= R_IDLE;
            endcase
        end
    end
endmodule
