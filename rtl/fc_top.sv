// The streaming filter chain for a picture's planes (docs/filter-chain.md): one fc_plane each (deblock, CDEF,
// restoration), all working on the same superblock row (band), and what they share:
//   - luma's CDEF directions for the chroma planes: a small memory written as luma's stage finds them, read by
//     chroma's. A chroma plane may not start a block column before luma has finished the columns above the same
//     picture area (chroma has a quarter or half of luma's samples and would otherwise run ahead of it);
//   - the band's start and end.
// Each plane has its own read ports into the row store and into the mode info (arrays over the plane below): the
// planes run at the same time, so the stores behind those ports must answer three readers.
// Monochrome: planes 1 and 2 stay idle (hdr.mono).
module fc_top
  import syn_pkg::*;
  import lr_pkg::*;
#(
    parameter int NBMAX = 32,
    parameter int CW = 10                    // log2 of the most 4x4 columns of a plane
) (
    input  logic            clk,
    input  logic            rst,
    input  hdr_t            hdr,
    input  lf_hdr_t         lh,
    input  cdef_hdr_t       ch,
    input  logic [2:0]      lf_on,           // per plane: the plane is deblocked
    input  logic            cdef_on,
    input  logic            band_start,      // pulse while idle
    input  logic [MIW-1:0]  row0,            // the band's first 4x4 luma row
    input  logic [5:0]      n_blk,           // luma 4x4 block rows in the band
    input  logic            first_band,
    input  logic            last_band,
    output logic            busy,
    output logic            band_done,
    output logic            err_ovf,
    // per plane p: row store
    output logic [2:0]            bk_re,
    output logic [2:0][5:0]       bk_row,
    output logic [2:0][CW-1:0]    bk_col,
    input  logic [2:0][191:0]     bk_data,
    // per plane: mode info for the deblock stage
    output logic [2:0][MIW-1:0]   rd_row, rd_col,
    input  mi_lf_t [2:0]          rd_info,
    output logic [2:0][1:0]       txr_plane,
    output logic [2:0][MIW-1:0]   txr_row, txr_col,
    input  logic [2:0][4:0]       txr_sz,
    // per plane: mode info for the CDEF stage
    output logic [2:0][MIW-1:0]   rd2_row, rd2_col,
    input  logic [2:0]            rd2_skip,
    output logic [2:0][SBW-1:0]   cdr_row64, cdr_col64,
    input  logic [2:0][3:0]       cdr_val,
    // per plane: restoration unit records
    output logic [2:0][1:0]       lrr_plane,
    output logic [2:0][SBW-1:0]   lrr_row, lrr_col,
    input  lr_rec_t [2:0]         lrr_rec,
    // per plane: the final line segments
    output logic [2:0]            out_valid,
    output logic [2:0][95:0]      out_line,
    output logic [2:0][1:0]       out_n4,
    output logic [2:0][CW-1:0]    out_col,
    output logic signed [2:0][8:0] out_y
);
    logic [2:0]            p_busy, p_done, p_ovf, p_act, done_l;
    logic [2:0][MIW-1:0]   yd_row, yd_col, dw_row, dw_col;
    logic [2:0][2:0]       yd, dw_dir;
    logic [2:0]            dw_we, cd_hold;
    logic [2:0][CW:0]      cd_col;
    logic [CW:0]           n_col_y, n_col_c;
    logic [5:0]            n_blk_c;
    always_comb begin
        n_col_y = (CW+1)'(hdr.mi_cols);
        n_col_c = (CW+1)'((hdr.mi_cols + MIW'(hdr.ssx)) >> hdr.ssx);
        n_blk_c = 6'((n_blk + 6'(hdr.ssy)) >> hdr.ssy);
        p_act = hdr.mono ? 3'b001 : 3'b111;
        // luma's directions exist for the block columns before cd_col[0]; a chroma block column at 4x4 column c lies
        // under luma's 4x4 columns from c << ssx on, and luma's block column there is two 4x4 columns wide
        cd_hold[0] = 1'b0;
        for (int p = 1; p < 3; p++)
            cd_hold[p] = (CW+2)'(cd_col[0]) < (((CW+2)'(cd_col[p]) << hdr.ssx) + (CW+2)'(2)) && (cd_col[0] < n_col_y);
    end
    generate for (genvar p = 0; p < 3; p++) begin : g_p
        fc_plane #(.LUMA(p == 0 ? 1 : 0), .NBMAX(NBMAX), .CW(CW)) u_pl (.clk, .rst, .hdr, .lh, .ch, .plane(2'(p)), .lf_on(lf_on[p]), .cdef_on,
            .band_start(band_start && p_act[p]), .row0, .n_blk(p == 0 ? n_blk : n_blk_c), .n_col(p == 0 ? n_col_y : n_col_c),
            .first_band, .last_band, .busy(p_busy[p]), .band_done(p_done[p]), .err_ovf(p_ovf[p]),
            .bk_re(bk_re[p]), .bk_row(bk_row[p]), .bk_col(bk_col[p]), .bk_data(bk_data[p]),
            .rd_row(rd_row[p]), .rd_col(rd_col[p]), .rd_info(rd_info[p]), .txr_plane(txr_plane[p]), .txr_row(txr_row[p]), .txr_col(txr_col[p]),
            .txr_sz(txr_sz[p]), .rd2_row(rd2_row[p]), .rd2_col(rd2_col[p]), .rd2_skip(rd2_skip[p]),
            .cdr_row64(cdr_row64[p]), .cdr_col64(cdr_col64[p]), .cdr_val(cdr_val[p]),
            .yd_row(yd_row[p]), .yd_col(yd_col[p]), .yd(yd[p]), .dw_we(dw_we[p]), .dw_row(dw_row[p]), .dw_col(dw_col[p]), .dw_dir(dw_dir[p]),
            .cd_col(cd_col[p]), .cd_hold(cd_hold[p]),
            .lrr_plane(lrr_plane[p]), .lrr_row(lrr_row[p]), .lrr_col(lrr_col[p]), .lrr_rec(lrr_rec[p]),
            .out_valid(out_valid[p]), .out_line(out_line[p]), .out_n4(out_n4[p]), .out_col(out_col[p]), .out_y(out_y[p]));
    end endgenerate

    // luma's directions per 8x8 block: 64 block rows (a band of 128 lines is 16, and chroma reads back into the band
    // above) by the picture's block columns; one copy for each chroma plane's reads
    localparam int DAW = 6 + CW - 1;
    logic [2:0] dmem1 [1 << DAW];
    logic [2:0] dmem2 [1 << DAW];
    logic [DAW-1:0] d_wa, d_ra1, d_ra2;
    logic [2:0]     d_q1, d_q2;
    always_comb begin
        d_wa  = {dw_row[0][6:1], dw_col[0][CW-1:1]};
        d_ra1 = {yd_row[1][6:1], yd_col[1][CW-1:1]};
        d_ra2 = {yd_row[2][6:1], yd_col[2][CW-1:1]};
    end
    always_ff @(posedge clk) begin
        if (dw_we[0]) begin dmem1[d_wa] <= dw_dir[0]; dmem2[d_wa] <= dw_dir[0]; end
        d_q1 <= dmem1[d_ra1]; d_q2 <= dmem2[d_ra2];
    end
    assign yd[0] = 3'd0; assign yd[1] = d_q1; assign yd[2] = d_q2;

    // the band is done when every active plane is (a plane's done pulse comes as its busy falls)
    logic act;
    assign busy = act;
    always_ff @(posedge clk) begin
        band_done <= 1'b0;
        if (rst) begin done_l <= '0; err_ovf <= 1'b0; act <= 1'b0; end
        else begin
            if (band_start && !act) begin act <= 1'b1; done_l <= ~p_act; end
            else if (act) begin
                done_l <= done_l | p_done;
                if (&(done_l | p_done)) begin band_done <= 1'b1; act <= 1'b0; done_l <= '0; end
            end
            if (|p_ovf) err_ovf <= 1'b1;
        end
    end
endmodule
