// One plane of the streaming filter chain (docs/filter-chain.md): the deblock stage (lf_feed + lf_stream), the CDEF
// stage (cdef_stream) and the restoration stage (lr_stream with its boundary line store lr_bstore), one superblock
// row (band) at a time. In: the band's pre-filter 4x4 blocks from the row store and the frame's mode info; out: the
// plane's final line segments, shifted up by the stages' context (16 lines for CDEF, three more for restoration).
// All three stages always run. A filter that the frame or the plane has switched off passes its samples through:
//   lf_on = 0   : no edge is applied (7.14.1: both luma levels 0, or the chroma plane's level 0)
//   cdef_on = 0 : every block has "no cdef_idx" (enable_cdef = 0, CodedLossless or allow_intrabc)
//   restoration: off when the plane's FrameRestorationType is NONE (hdr.lr_type)
// Between CDEF and restoration sits a FIFO of lines: restoration takes a self-guided line in about 30 clocks, and
// CDEF's output cannot wait line by line, so CDEF starts a block column only when the FIFO has room for all of it.
// Super-resolution (7.16) is two sr_stream in front of restoration (CDEF's lines, the boundary lines); what leaves
// the plane is then in the upscaled picture's columns.
// Not here: the stores shared between planes (mode info, luma's CDEF record for chroma), the row store itself.
module fc_plane
  import syn_pkg::*;
  import lr_pkg::*;
#(
    parameter int LUMA = 1,                  // 0: a chroma-only deblock stage
    parameter int NBMAX = 32,
    parameter int CW = 10,                   // log2 of the most 4x4 columns
    parameter int OUT_SLACK = 16             // lines that may still come out after out_hold rises (the tests measure it)
) (
    input  logic            clk,
    input  logic            rst,
    input  hdr_t            hdr,
    input  lf_hdr_t         lh,
    input  cdef_hdr_t       ch,
    input  logic [1:0]      plane,
    input  logic            lf_on,
    input  logic            cdef_on,
    input  logic            band_start,      // pulse while idle
    input  logic [MIW-1:0]  row0,            // the band's first 4x4 luma row
    input  logic [5:0]      n_blk,           // the plane's 4x4 block rows in the band
    input  logic [CW:0]     n_col,           // the plane's 4x4 block columns
    input  logic            first_band,
    input  logic            last_band,
    output logic            busy,
    output logic            band_done,
    output logic            err_ovf,         // a CDEF line found the FIFO full (never, if col_hold's arithmetic holds); stays set
    // row store: block (row, col) of the band, data the next clock
    output logic            bk_re,
    output logic [5:0]      bk_row,
    output logic [CW-1:0]   bk_col,
    input  logic [191:0]    bk_data,
    // mode info for the deblock stage (mi_store, registered reads)
    output logic [MIW-1:0]  rd_row, rd_col,
    input  mi_lf_t          rd_info,
    output logic [1:0]      txr_plane,
    output logic [MIW-1:0]  txr_row, txr_col,
    input  logic [4:0]      txr_sz,
    // mode info for the CDEF stage: a 4x4 unit's skip flag, a 64x64 unit's cdef_idx
    output logic [MIW-1:0]  rd2_row, rd2_col,
    input  logic            rd2_skip,
    output logic [SBW-1:0]  cdr_row64, cdr_col64,
    input  logic [3:0]      cdr_val,
    // luma's CDEF decisions per 8x8 block {filtered, cdef_idx, direction}: written by the luma plane, read by chroma
    output logic [MIW-1:0]  yd_row, yd_col,
    input  logic [6:0]      yd,
    output logic            dw_we,
    output logic [MIW-1:0]  dw_row, dw_col,
    output logic [6:0]      dw_info,
    // the CDEF stage's progress in the band and a hold from outside (a chroma plane waits for luma's directions)
    output logic [CW:0]     cd_col,          // first 4x4 column of the block column CDEF filters next
    input  logic            cd_hold,
    // restoration unit records
    output logic [1:0]      lrr_plane,
    output logic [SBW-1:0]  lrr_row, lrr_col,
    input  lr_rec_t         lrr_rec,
    // the taker's hold: restoration pushes no further line into its filters while it is high. The lines already in
    // them still come out (at most OUT_SLACK), so the taker raises it with that much room left
    input  logic            out_hold,
    output logic            out_valid,
    output logic [95:0]     out_line,        // out_n4 groups of four samples from bit 0
    output logic [1:0]      out_n4,
    output logic [CW-1:0]   out_col,         // the segment's first 4x4 column
    output logic signed [8:0] out_y          // the line, counted from the band's first line
);
    // ---------------------------------------------------------------- the plane's geometry
    logic            ssx, ssy;
    logic [MIW-1:0]  prow0, pic_rows;
    logic [3:0]      u_log2;
    logic [PXW-1:0]  fh_p, fw_p, ur, uc;
    always_comb begin
        ssx = (plane != 2'd0) && hdr.ssx; ssy = (plane != 2'd0) && hdr.ssy;
        prow0 = row0 >> ssy;
        pic_rows = hdr.mi_rows >> ssy;
        u_log2 = 4'd5 + 4'(hdr.lr_size[2 * plane +: 2]);
        fh_p = (hdr.frame_height + PXW'(ssy)) >> ssy;
        fw_p = (hdr.upscaled_width + PXW'(ssx)) >> ssx;
        ur = (fh_p + (PXW'(1) << (u_log2 - 4'd1))) >> u_log2; if (ur == '0) ur = PXW'(1);
        uc = (fw_p + (PXW'(1) << (u_log2 - 4'd1))) >> u_log2; if (uc == '0) uc = PXW'(1);
    end

    // ---------------------------------------------------------------- deblock
    logic         f_busy, s_busy, c_busy, r_busy;
    logic         fi_valid, fi_ready;
    logic [191:0] fi_blk;
    logic [29:0]  fi_vpar, fi_hpar;
    logic         lo_valid, lo_ready;
    logic         cd_ready, bc_ready;        // a deblocked block goes to CDEF and past the boundary-line converter in the same clock
    assign lo_ready = cd_ready && bc_ready;
    logic [191:0] lo_blk;
    logic [CW-1:0] lo_col;
    logic signed [6:0] lo_row;
    lf_feed #(.NBMAX(NBMAX), .CW(CW)) u_feed (.clk, .rst, .hdr, .lh, .plane, .band_start, .row0, .n_blk, .n_col, .busy(f_busy),
        .rd_row, .rd_col, .rd_info, .txr_plane, .txr_row, .txr_col, .txr_sz,
        .bk_re, .bk_row, .bk_col, .bk_data, .in_valid(fi_valid), .in_ready(fi_ready), .in_blk(fi_blk), .in_vpar(fi_vpar), .in_hpar(fi_hpar));
    lf_stream #(.LUMA(LUMA), .NBMAX(NBMAX), .CW(CW)) u_lf (.clk, .rst, .bit_depth(hdr.bit_depth), .chroma(plane != 2'd0),
        .band_start, .n_blk, .n_col, .first_band, .last_band, .busy(s_busy), .band_done(),
        .in_valid(fi_valid), .in_ready(fi_ready), .in_blk(fi_blk),
        .in_vpar({fi_vpar[29] && lf_on, fi_vpar[28:0]}), .in_hpar({fi_hpar[29] && lf_on, fi_hpar[28:0]}),
        .out_valid(lo_valid), .out_ready(lo_ready), .out_blk(lo_blk), .out_col(lo_col), .out_row(lo_row));

    // ---------------------------------------------------------------- CDEF
    localparam int FAW = 8;
    logic         co_valid;
    logic [95:0]  co_line;
    logic signed [8:0] co_y;
    logic [FAW:0] ff_free;
    logic         ff_in_ready;
    cdef_stream #(.NBMAX(NBMAX), .CW(CW)) u_cdef (.clk, .rst, .bit_depth(hdr.bit_depth), .ch, .luma(plane == 2'd0), .ssx, .ssy,
        .mi_rows(hdr.mi_rows), .mi_cols(hdr.mi_cols), .band_start, .row0(prow0), .pic_rows, .n_blk, .n_col, .first_band, .last_band,
        .busy(c_busy), .band_done(), .in_valid(lo_valid && bc_ready), .in_ready(cd_ready), .in_blk(lo_blk),
        .rd_row(rd2_row), .rd_col(rd2_col), .rd_skip(rd2_skip), .cdr_row64, .cdr_col64, .cdr_val(cdef_on ? cdr_val : 4'd0),
        .yd_row, .yd_col, .yd, .dw_we, .dw_row, .dw_col, .dw_info,
        .col_hold(cd_hold || ff_free < (FAW+1)'(160)),   // a column's lines: at most 4 (32 + 4), and four still on their way
        .col_next(cd_col),
        .out_valid(co_valid), .out_line(co_line), .out_n4(), .out_col(), .out_y(co_y));

    // ---------------------------------------------------------------- restoration
    logic         ri_valid, ri_ready;
    logic [95:0]  ri_line;
    logic signed [8:0] ri_y;
    logic [SBW:0]   bl_k;
    logic [CW-1:0]  bl_col;
    logic [383:0]   bl_rd;
    logic           r_done;
    fc_lfifo #(.W(105), .AW(FAW)) u_ff (.clk, .rst, .in_valid(co_valid), .in_ready(ff_in_ready), .in_data({co_y, co_line}),
        .out_valid(ri_valid), .out_ready(ri_ready), .out_data({ri_y, ri_line}), .free(ff_free));
    // super-resolution (7.16) sits in front of restoration: on CDEF's lines and on the deblocked boundary lines.
    // Behind it everything is in the upscaled picture's columns. Without super-resolution both upscalers are wires.
    logic [PXW-1:0] dw_p, max_x_p;
    logic [CW:0]    n_seg, n_seg_up, n_seg_up_b, n_col_lr;
    logic signed [8:0] y_last;               // a column's last line from CDEF, counted from the band's first line
    logic           su_valid, su_ready, su_last, sa_busy, sb_busy;
    logic [95:0]    su_line;
    logic [8:0]     su_tag;
    logic           bc_v, bc_rdy, bc_last, bu_v, bu_last;
    logic [95:0]    bc_line, bu_line;
    logic [8:0]     bc_tag, bu_tag;
    always_comb begin
        dw_p = (lh.frame_width + PXW'(ssx)) >> ssx;
        max_x_p = PXW'({(hdr.mi_cols >> ssx), 2'b00}) - PXW'(1);
        n_seg = ssx ? n_col : (CW+1)'((n_col + (CW+1)'(1)) >> 1);
        n_col_lr = ssx ? n_seg_up : (CW+1)'(n_seg_up << 1);
        y_last = last_band ? 9'({n_blk, 2'b00}) - 9'sd1 : 9'({n_blk, 2'b00}) - (ssy ? 9'sd13 : 9'sd17);
    end
    sr_stream #(.CW(CW)) u_sra (.clk, .rst, .bit_depth(hdr.bit_depth), .use_sr(hdr.use_superres), .ssx, .down_w(dw_p), .up_w(fw_p), .max_x(max_x_p),
        .band_start, .n_seg_in(n_seg), .n_seg_out(n_seg_up), .busy(sa_busy),
        .in_valid(ri_valid), .in_ready(ri_ready), .in_line(ri_line), .in_tag(ri_y), .in_last(ri_y == y_last),
        .out_valid(su_valid), .out_ready(su_ready), .out_line(su_line), .out_tag(su_tag), .out_last(su_last));
    lr_bconv #(.CW(CW)) u_bc (.clk, .rst, .ssx, .ssy, .band_start, .row0(prow0), .n_blk, .last_band,
        .in_valid(lo_valid && cd_ready), .in_ready(bc_ready), .in_blk(lo_blk), .in_col(lo_col), .in_row(lo_row),
        .out_valid(bc_v), .out_ready(bc_rdy), .out_line(bc_line), .out_tag(bc_tag), .out_last(bc_last));
    sr_stream #(.CW(CW)) u_srb (.clk, .rst, .bit_depth(hdr.bit_depth), .use_sr(hdr.use_superres), .ssx, .down_w(dw_p), .up_w(fw_p), .max_x(max_x_p),
        .band_start, .n_seg_in(n_seg), .n_seg_out(n_seg_up_b), .busy(sb_busy),
        .in_valid(bc_v), .in_ready(bc_rdy), .in_line(bc_line), .in_tag(bc_tag), .in_last(bc_last),
        .out_valid(bu_v), .out_ready(1'b1), .out_line(bu_line), .out_tag(bu_tag), .out_last(bu_last));
    lr_bstore #(.CW(CW)) u_bs (.clk, .rst, .ssx, .band_start, .wr_valid(bu_v), .wr_line(bu_line), .wr_tag(bu_tag), .wr_last(bu_last),
        .bl_k, .bl_col, .bl_rd);
    lr_stream #(.CW(CW)) u_lr (.clk, .rst, .bit_depth(hdr.bit_depth), .en(hdr.lr_type[2 * plane +: 2] != 2'd0), .plane, .ssx, .ssy,
        .pl_w((MIW+2)'(fw_p)), .pl_h((MIW+2)'(fh_p)), .u_log2, .u_rows(SBW'(ur)), .u_cols(SBW'(uc)),
        .band_start, .row0(prow0), .pic_rows, .n_blk, .n_col(n_col_lr), .first_band, .last_band, .busy(r_busy), .band_done(r_done),
        .in_valid(su_valid), .in_ready(su_ready), .in_line(su_line), .in_y(su_tag), .out_hold,
        .lrr_plane, .lrr_row, .lrr_col, .lrr_rec, .bl_k, .bl_col, .bl_rd,
        .out_valid, .out_line, .out_n4, .out_col, .out_y);

    // ---------------------------------------------------------------- the band is done when restoration is and the stages before it are idle
    logic done_l, act;
    assign busy = act;
    always_ff @(posedge clk) begin
        band_done <= 1'b0;
        if (rst) begin done_l <= 1'b0; act <= 1'b0; err_ovf <= 1'b0; end
        else begin
            if (band_start && !act) begin act <= 1'b1; done_l <= 1'b0; end
            if (r_done) done_l <= 1'b1;
            if (co_valid && !ff_in_ready) err_ovf <= 1'b1;
            if (act && done_l && !f_busy && !s_busy && !c_busy && !r_busy && !sa_busy && !sb_busy && !bc_v) begin act <= 1'b0; done_l <= 1'b0; band_done <= 1'b1; end
        end
    end
endmodule
