// Deblocking edge decision (spec 7.14.2 - 7.14.5) for one 4x4 position and one pass, combinational: whether the edge
// is filtered, the filter size and the strength, from the mode info of the position and of its neighbour across the
// edge. The logic is lf_top's (strength(), the edge tests, the level fallback to the neighbour), lifted out so that the
// streaming deblock stage (rtl/lf_stream.sv, docs/filter-chain.md) can take a block's two edges a clock.
//   par = {apply, size[4:0], limit[7:0], blimit[7:0], thresh[7:0]}: apply = filtered at all (a transform edge that
//         7.14.2 lets through, in_pic the frame, not the picture's left / top border, level > 0).
module lf_edge
  import blk_tables_pkg::*;
  import tx_tables_pkg::*;
  import syn_pkg::*;
(
    input  hdr_t        hdr,
    input  lf_hdr_t     lh,
    input  logic [1:0]  plane,
    input  logic        pss,               // 0: the vertical edge on the position's left, 1: the horizontal edge on its top
    input  logic [MIW-1:0] row, col,       // the position in 4x4 luma units (a chroma position: the luma unit it starts at)
    input  mi_lf_t      cur, prv,          // mode info at (row | subY, col | subX) and across the edge
    input  logic [4:0]  cur_tx, prv_tx,    // loop-filter transform sizes of the plane at the two positions
    output logic [29:0] par
);
    logic        sub_x, sub_y;
    logic [PXW-1:0] x, y, xp, yp;
    always_comb begin
        sub_x = (plane != 2'd0) && hdr.ssx;
        sub_y = (plane != 2'd0) && hdr.ssy;
        x = PXW'(col) << 2; y = PXW'(row) << 2;
        xp = x >> sub_x; yp = y >> sub_y;
    end
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
        if (lh.delta_enabled) begin
            // 7.14.4: the reference frame's delta, plus the mode delta for inter blocks, both scaled by lvlSeg >> 5
            logic signed [9:0] dl;
            dl = 10'(signed'(lh.ref_deltas[7 * info.ref0 +: 7]));
            if (info.ref0 != 3'd0) dl = dl + 10'(signed'(lh.mode_deltas[7 * info.mode_type +: 7]));
            lvl_seg = clip63(10'(signed'({4'b0, lvl_seg})) + (dl <<< (lvl_seg >> 5)));
        end
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
    logic [29:0] str_cur, str_prv, s;
    logic [4:0]  plane_size;
    logic        is_blk_edge, is_tx_edge, apply_filter, in_pic;
    logic [6:0]  base_size;
    logic [4:0]  fsize_c;
    always_comb begin
        str_cur = strength(cur, plane, pss);
        str_prv = strength(prv, plane, pss);
        plane_size = subsampled_size(cur.bsize, sub_x, sub_y);
        if (!pss) begin
            is_blk_edge = (xp & (PXW'(blk_w(plane_size)) - PXW'(1))) == PXW'(0);
            is_tx_edge  = (xp & (PXW'(tx_width(cur_tx)) - PXW'(1))) == PXW'(0);
            base_size = (tx_width(prv_tx) < tx_width(cur_tx)) ? tx_width(prv_tx) : tx_width(cur_tx);
        end else begin
            is_blk_edge = (yp & (PXW'(blk_h(plane_size)) - PXW'(1))) == PXW'(0);
            is_tx_edge  = (yp & (PXW'(tx_height(cur_tx)) - PXW'(1))) == PXW'(0);
            base_size = (tx_height(prv_tx) < tx_height(cur_tx)) ? tx_height(prv_tx) : tx_height(cur_tx);
        end
        apply_filter = is_tx_edge && (is_blk_edge || !cur.skip || cur.ref0 == 3'd0);   // 7.14.2: tx edges in_pic a skipped INTER block are not filtered
        fsize_c = (plane == 2'd0) ? ((base_size > 7'd16) ? 5'd16 : 5'(base_size)) : ((base_size > 7'd8) ? 5'd8 : 5'(base_size));
        in_pic = (x < lh.frame_width) && (y < lh.frame_height) && !(!pss && x == PXW'(0)) && !(pss && y == PXW'(0));
        s = (str_cur[29:24] != 6'd0) ? str_cur : str_prv;       // level 0 at the position: the neighbour's strength
        par = {in_pic && apply_filter && (s[29:24] != 6'd0), fsize_c, s[23:0]};
    end
endmodule
