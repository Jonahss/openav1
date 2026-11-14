// Test wrapper: the streaming deblock stage of one plane as it will sit in the decoder, without the decoder:
// mi_store (mode info and transform sizes, loaded by the test) + a block memory standing in for the row store
// (loaded by the test, one band at a time) + lf_feed + lf_stream. tb/test_lf_stage.py.
module lf_stage_tb_top
  import syn_pkg::*;
#(
    parameter int NBMAX = 32,
    parameter int CW = 8
) (
    input  logic        clk,
    input  logic        rst,
    input  hdr_t        hdr,
    input  lf_hdr_t     lh,
    input  logic [1:0]  plane,
    input  logic        band_start,
    input  logic [MIW-1:0] row0,
    input  logic [5:0]  n_blk,
    input  logic [CW:0] n_col,
    input  logic        first_band,
    input  logic        last_band,
    output logic        busy,
    output logic        band_done,
    output logic        out_valid,
    input  logic        out_ready,
    output logic [191:0] out_blk,
    output logic [CW-1:0] out_col,
    output logic signed [6:0] out_row,
    // the row store's stand-in: block (row, col) of the band
    input  logic        bm_we,
    input  logic [5:0]  bm_row,
    input  logic [CW-1:0] bm_col,
    input  logic [191:0] bm_data,
    // mode info writes (as rtl/lf_tb_top.sv)
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
    output logic        tx_busy
);
    logic [MIW-1:0] rd_row, rd_col, txr_row, txr_col; mi_lf_t rd_info; logic [1:0] txr_plane; logic [4:0] txr_sz;
    mi_store u_mi (.clk, .rst, .rd2_row(MIW'(0)), .rd2_col(MIW'(0)), .rd2_info(), .blk_we, .blk_r, .blk_c, .blk_bw4, .blk_bh4, .blk_data, .blk_busy,
                   .tx_we, .tx_plane, .tx_row, .tx_col, .tx_w4, .tx_h4, .tx_sz, .tx_busy,
                   .rd_row, .rd_col, .rd_info, .txr_plane, .txr_row, .txr_col, .txr_sz,
                   .cd_clr(1'b0), .cd_we(1'b0), .cd_row64(SBW'(0)), .cd_col64(SBW'(0)), .cd_sb128(1'b0), .cd_mask(4'd0), .cd_idx(3'd0),
                   .cdr_row64(SBW'(0)), .cdr_col64(SBW'(0)), .cdr_val(),
                   .lr_we(1'b0), .lr_rec('0), .lrr_plane(2'd0), .lrr_row(SBW'(0)), .lrr_col(SBW'(0)), .lrr_rec());
    localparam int RW = $clog2(NBMAX);
    logic [191:0] bmem [1 << (CW + RW)];
    logic         bk_re; logic [5:0] bk_row; logic [CW-1:0] bk_col; logic [191:0] bk_data;
    always_ff @(posedge clk) begin
        if (bm_we) bmem[{bm_col, bm_row[RW-1:0]}] <= bm_data;
        if (bk_re) bk_data <= bmem[{bk_col, bk_row[RW-1:0]}];
    end
    logic         in_valid, in_ready, f_busy, s_busy; logic [191:0] in_blk; logic [29:0] in_vpar, in_hpar;
    lf_feed #(.NBMAX(NBMAX), .CW(CW)) u_feed (.clk, .rst, .hdr, .lh, .plane, .band_start, .row0, .n_blk, .n_col, .busy(f_busy),
        .rd_row, .rd_col, .rd_info, .txr_plane, .txr_row, .txr_col, .txr_sz,
        .bk_re, .bk_row, .bk_col, .bk_data, .in_valid, .in_ready, .in_blk, .in_vpar, .in_hpar);
    lf_stream #(.NBMAX(NBMAX), .CW(CW)) u_lf (.clk, .rst, .bit_depth(hdr.bit_depth), .chroma(plane != 2'd0),
        .band_start, .n_blk, .n_col, .first_band, .last_band, .busy(s_busy), .band_done,
        .in_valid, .in_ready, .in_blk, .in_vpar, .in_hpar, .out_valid, .out_ready, .out_blk, .out_col, .out_row);
    assign busy = f_busy || s_busy;
endmodule
