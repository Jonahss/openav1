// Test wrapper: one plane of the streaming filter chain as it will sit in the decoder, without the decoder: mi_store
// (mode info, transform sizes, cdef_idx and restoration units, loaded by the test) + a block memory standing in for
// the row store (loaded by the test, one band at a time) + fc_plane. The luma directions for a chroma plane's CDEF
// are answered by the test. tb/test_fc_plane.py.
module fc_plane_tb_top
  import syn_pkg::*;
  import lr_pkg::*;
#(
    parameter int LUMA = 1,
    parameter int NBMAX = 32,
    parameter int CW = 8
) (
    input  logic        clk,
    input  logic        rst,
    input  hdr_t        hdr,
    input  lf_hdr_t     lh,
    input  cdef_hdr_t   ch,
    input  logic [1:0]  plane,
    input  logic        lf_on,
    input  logic        cdef_on,
    input  logic        band_start,
    input  logic [MIW-1:0] row0,
    input  logic [5:0]  n_blk,
    input  logic [CW:0] n_col,
    input  logic        first_band,
    input  logic        last_band,
    output logic        busy,
    output logic        band_done,
    output logic        err_ovf,
    output logic [MIW-1:0] yd_row, yd_col,
    input  logic [2:0]  yd,
    output logic        dw_we,
    output logic [MIW-1:0] dw_row, dw_col,
    output logic [2:0]  dw_dir,
    output logic        out_valid,
    output logic [95:0] out_line,
    output logic [1:0]  out_n4,
    output logic [CW-1:0] out_col,
    output logic signed [8:0] out_y,
    // the row store's stand-in: block (row, col) of the band
    input  logic        bm_we,
    input  logic [5:0]  bm_row,
    input  logic [CW-1:0] bm_col,
    input  logic [191:0] bm_data,
    // mode info writes
    input  logic        blk_we,
    input  logic [MIW-1:0] blk_r, blk_c,
    input  logic [5:0]  blk_bw4, blk_bh4,
    input  mi_lf_t      blk_data,
    output logic        blk_busy,
    input  logic        tx_we,
    input  logic [1:0]  tx_plane,
    input  logic [MIW-1:0] tx_row, tx_col,
    input  logic [4:0]  tx_w4, tx_h4,
    input  logic [4:0]  tx_sz,
    output logic        tx_busy,
    input  logic        cd_clr,
    input  logic        cd_we,
    input  logic [SBW-1:0] cd_row64, cd_col64,
    input  logic        cd_sb128,
    input  logic [3:0]  cd_mask,
    input  logic [2:0]  cd_idx,
    input  logic        lr_we,
    input  lr_rec_t     lr_rec
);
    logic [MIW-1:0] rd_row, rd_col, rd2_row, rd2_col, txr_row, txr_col;
    mi_lf_t         rd_info, rd2_info;
    logic [1:0]     txr_plane, lrr_plane;
    logic [4:0]     txr_sz;
    logic [SBW-1:0] cdr_row64, cdr_col64, lrr_row, lrr_col;
    logic [3:0]     cdr_val;
    lr_rec_t        lrr_rec;
    mi_store u_mi (.clk, .rst, .blk_we, .blk_r, .blk_c, .blk_bw4, .blk_bh4, .blk_data, .blk_busy,
                   .tx_we, .tx_plane, .tx_row, .tx_col, .tx_w4, .tx_h4, .tx_sz, .tx_busy,
                   .rd_row, .rd_col, .rd_info, .rd2_row, .rd2_col, .rd2_info, .txr_plane, .txr_row, .txr_col, .txr_sz,
                   .cd_clr, .cd_we, .cd_row64, .cd_col64, .cd_sb128, .cd_mask, .cd_idx, .cdr_row64, .cdr_col64, .cdr_val,
                   .lr_we, .lr_rec, .lrr_plane, .lrr_row, .lrr_col, .lrr_rec);
    localparam int RW = $clog2(NBMAX);
    logic [191:0] bmem [1 << (CW + RW)];
    logic         bk_re; logic [5:0] bk_row; logic [CW-1:0] bk_col; logic [191:0] bk_data;
    always_ff @(posedge clk) begin
        if (bm_we) bmem[{bm_col, bm_row[RW-1:0]}] <= bm_data;
        if (bk_re) bk_data <= bmem[{bk_col, bk_row[RW-1:0]}];
    end
    fc_plane #(.LUMA(LUMA), .NBMAX(NBMAX), .CW(CW)) u_fc (.clk, .rst, .hdr, .lh, .ch, .plane, .lf_on, .cdef_on,
        .band_start, .row0, .n_blk, .n_col, .first_band, .last_band, .busy, .band_done, .err_ovf,
        .bk_re, .bk_row, .bk_col, .bk_data, .rd_row, .rd_col, .rd_info, .txr_plane, .txr_row, .txr_col, .txr_sz,
        .rd2_row, .rd2_col, .rd2_skip(rd2_info.skip), .cdr_row64, .cdr_col64, .cdr_val,
        .yd_row, .yd_col, .yd, .dw_we, .dw_row, .dw_col, .dw_dir, .cd_col(), .cd_hold(1'b0), .lrr_plane, .lrr_row, .lrr_col, .lrr_rec,
        .out_valid, .out_line, .out_n4, .out_col, .out_y);
endmodule
