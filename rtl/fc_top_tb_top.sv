// Test wrapper: the streaming filter chain for all planes as it will sit in the decoder, without the decoder: fc_top
// with, per plane, a mi_store (all three written alike by the test: the planes read mode info at the same time) and a
// block memory standing in for the row store (loaded by the test, one band at a time). tb/test_fc_top.py.
module fc_top_tb_top
  import syn_pkg::*;
  import lr_pkg::*;
#(
    parameter int NBMAX = 32,
    parameter int CW = 8
) (
    input  logic        clk,
    input  logic        rst,
    input  hdr_t        hdr,
    input  lf_hdr_t     lh,
    input  cdef_hdr_t   ch,
    input  logic [2:0]  lf_on,
    input  logic        cdef_on,
    input  logic        band_start,
    input  logic [MIW-1:0] row0,
    input  logic [5:0]  n_blk,
    input  logic        first_band,
    input  logic        last_band,
    output logic        busy,
    output logic        band_done,
    output logic        err_ovf,
    output logic [2:0]            out_valid,
    output logic [2:0][95:0]      out_line,
    output logic [2:0][1:0]       out_n4,
    output logic [2:0][CW-1:0]    out_col,
    output logic signed [2:0][8:0] out_y,
    // the row store's stand-ins: block (row, col) of the band in plane bm_plane
    input  logic        bm_we,
    input  logic [1:0]  bm_plane,
    input  logic [5:0]  bm_row,
    input  logic [CW-1:0] bm_col,
    input  logic [191:0] bm_data,
    // mode info writes (to all three stores)
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
    logic [2:0][MIW-1:0] rd_row, rd_col, rd2_row, rd2_col, txr_row, txr_col;
    mi_lf_t [2:0]        rd_info, rd2_info;
    logic [2:0][1:0]     txr_plane, lrr_plane;
    logic [2:0][4:0]     txr_sz;
    logic [2:0][SBW-1:0] cdr_row64, cdr_col64, lrr_row, lrr_col;
    logic [2:0][3:0]     cdr_val;
    lr_rec_t [2:0]       lrr_rec;
    logic [2:0]          bk_re, rd2_skip, b_busy, t_busy;
    logic [2:0][5:0]     bk_row;
    logic [2:0][CW-1:0]  bk_col;
    logic [2:0][191:0]   bk_data;
    localparam int RW = $clog2(NBMAX);
    assign blk_busy = b_busy[0]; assign tx_busy = t_busy[0];
    generate for (genvar p = 0; p < 3; p++) begin : g_p
        mi_store u_mi (.clk, .rst, .blk_we, .blk_r, .blk_c, .blk_bw4, .blk_bh4, .blk_data, .blk_busy(b_busy[p]),
                       .tx_we, .tx_plane, .tx_row, .tx_col, .tx_w4, .tx_h4, .tx_sz, .tx_busy(t_busy[p]),
                       .rd_row(rd_row[p]), .rd_col(rd_col[p]), .rd_info(rd_info[p]), .rd2_row(rd2_row[p]), .rd2_col(rd2_col[p]), .rd2_info(rd2_info[p]),
                       .txr_plane(txr_plane[p]), .txr_row(txr_row[p]), .txr_col(txr_col[p]), .txr_sz(txr_sz[p]),
                       .cd_clr, .cd_we, .cd_row64, .cd_col64, .cd_sb128, .cd_mask, .cd_idx,
                       .cdr_row64(cdr_row64[p]), .cdr_col64(cdr_col64[p]), .cdr_val(cdr_val[p]),
                       .lr_we, .lr_rec, .lrr_plane(lrr_plane[p]), .lrr_row(lrr_row[p]), .lrr_col(lrr_col[p]), .lrr_rec(lrr_rec[p]));
        assign rd2_skip[p] = rd2_info[p].skip;
        logic [191:0] bmem [1 << (CW + RW)];
        always_ff @(posedge clk) begin
            if (bm_we && bm_plane == 2'(p)) bmem[{bm_col, bm_row[RW-1:0]}] <= bm_data;
            if (bk_re[p]) bk_data[p] <= bmem[{bk_col[p], bk_row[p][RW-1:0]}];
        end
    end endgenerate
    fc_top #(.NBMAX(NBMAX), .CW(CW)) u_fc (.*);
endmodule
