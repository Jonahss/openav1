// Feeds the streaming deblock stage (rtl/lf_stream.sv): walks a band's 4x4 blocks column by column, reads each
// block's samples from the row store and its mode info and transform size from mi_store (all one clock after the
// request), and gives lf_stream the block with the strengths of its left and top edges (two lf_edge).
// The neighbour across the left edge is the same row of the column before: kept in a register file over the band's
// rows. The neighbour across the top edge is the row before in the column, or, for a column's first block, the last
// block of that column in the band above: kept per column from band to band.
// One block a clock when lf_stream takes them: requests run ahead into a two-entry queue, and the block leaves
// through a register with its strengths.
module lf_feed
  import blk_tables_pkg::*;
  import tx_tables_pkg::*;
  import syn_pkg::*;
#(
    parameter int NBMAX = 32,
    parameter int CW = 10
) (
    input  logic            clk,
    input  logic            rst,
    input  hdr_t            hdr,
    input  lf_hdr_t         lh,
    input  logic [1:0]      plane,
    input  logic            band_start,      // pulse while idle (with lf_stream's)
    input  logic [MIW-1:0]  row0,            // the band's first 4x4 luma row
    input  logic [5:0]      n_blk,           // block rows of the plane in the band
    input  logic [CW:0]     n_col,           // block columns of the plane
    output logic            busy,
    // mode info (mi_store, registered reads)
    output logic [MIW-1:0]  rd_row, rd_col,
    input  mi_lf_t          rd_info,
    output logic [1:0]      txr_plane,
    output logic [MIW-1:0]  txr_row, txr_col,
    input  logic [4:0]      txr_sz,
    // row store: block (row, col) of the band, data the next clock
    output logic            bk_re,
    output logic [5:0]      bk_row,
    output logic [CW-1:0]   bk_col,
    input  logic [191:0]    bk_data,
    // to lf_stream
    output logic            in_valid,
    input  logic            in_ready,
    output logic [191:0]    in_blk,
    output logic [29:0]     in_vpar,
    output logic [29:0]     in_hpar
);
    localparam int IW = $bits(mi_lf_t);
    localparam int EW = IW + 5;                     // {mode info, transform size}
    localparam int RW = $clog2(NBMAX);
    logic        sub_x, sub_y;
    assign sub_x = (plane != 2'd0) && hdr.ssx;
    assign sub_y = (plane != 2'd0) && hdr.ssy;
    // ---------------------------------------------------------------- requests
    logic            i_act;
    logic [5:0]      i_row;
    logic [CW:0]     i_col;
    logic            rq_v;                          // a request went out last clock
    logic [5:0]      rq_row;
    logic [CW:0]     rq_col;
    logic [1:0]      q_n;                           // queue entries
    logic            issue, pop;
    assign issue = i_act && (2'(q_n) + 2'(rq_v) < 2'd2);
    assign bk_re = issue; assign bk_row = i_row; assign bk_col = CW'(i_col);
    assign rd_row = (row0 + (MIW'(i_row) << sub_y)) | MIW'(sub_y);
    assign rd_col = (MIW'(i_col) << sub_x) | MIW'(sub_x);
    assign txr_plane = plane;
    assign txr_row = (row0 >> sub_y) + MIW'(i_row);
    assign txr_col = MIW'(i_col);
    // ---------------------------------------------------------------- queue of blocks with their mode info
    logic [EW-1:0]   q_e [2];
    logic [191:0]    q_b [2];
    logic [5:0]      q_r [2];
    logic [CW:0]     q_c [2];
    logic            q_h;                           // head index
    logic            q_t;                           // where a landing request goes: head + entries (a full queue only
    assign q_t = q_h ^ q_n[0];                      //   receives in the clock its head is taken, into the slot that frees)
    logic [EW-1:0]   h_e;
    logic [5:0]      h_r;
    logic [CW:0]     h_c;
    assign h_e = q_e[q_h]; assign h_r = q_r[q_h]; assign h_c = q_c[q_h];
    // ---------------------------------------------------------------- neighbours
    logic [EW-1:0]   left_e [NBMAX];                // the column before, by row
    logic [EW-1:0]   above_e;                       // the row before in this column
    logic [EW-1:0]   top_e [1 << CW];               // the band above: last row of each column
    logic [EW-1:0]   pv_e, ph_e;
    assign pv_e = (h_c != '0) ? left_e[h_r[RW-1:0]] : '0;
    assign ph_e = (h_r != 6'd0) ? above_e : top_e[h_c[CW-1:0]];
    mi_lf_t h_info, pv_info, ph_info;
    assign h_info = mi_lf_t'(h_e[EW-1:5]); assign pv_info = mi_lf_t'(pv_e[EW-1:5]); assign ph_info = mi_lf_t'(ph_e[EW-1:5]);
    logic [MIW-1:0] e_row, e_col;
    assign e_row = row0 + (MIW'(h_r) << sub_y);
    assign e_col = MIW'(h_c) << sub_x;
    logic [29:0] e_vpar, e_hpar;
    lf_edge u_ev (.hdr, .lh, .plane, .pss(1'b0), .row(e_row), .col(e_col), .cur(h_info), .prv(pv_info),
                  .cur_tx(h_e[4:0]), .prv_tx(pv_e[4:0]), .par(e_vpar));
    lf_edge u_eh (.hdr, .lh, .plane, .pss(1'b1), .row(e_row), .col(e_col), .cur(h_info), .prv(ph_info),
                  .cur_tx(h_e[4:0]), .prv_tx(ph_e[4:0]), .par(e_hpar));
    // output register: the block with its two strengths (the edge decision ends here, not inside lf_stream)
    logic o_v;
    assign in_valid = o_v;
    assign pop = (q_n != 2'd0) && (!o_v || in_ready);
    assign busy = i_act || rq_v || (q_n != 2'd0) || o_v;
    always_ff @(posedge clk) begin
        if (rst) begin
            i_act <= 1'b0; i_row <= '0; i_col <= '0; rq_v <= 1'b0; rq_row <= '0; rq_col <= '0; q_n <= 2'd0; q_h <= 1'b0;
            above_e <= '0; o_v <= 1'b0; in_blk <= '0; in_vpar <= '0; in_hpar <= '0;
        end else begin
            if (band_start && !busy) begin i_act <= 1'b1; i_row <= '0; i_col <= '0; end
            rq_v <= issue; rq_row <= i_row; rq_col <= i_col;
            if (issue) begin
                if (i_row == n_blk - 6'd1) begin
                    i_row <= '0; i_col <= i_col + 1'b1;
                    if (i_col == n_col - 1'b1) i_act <= 1'b0;
                end else i_row <= i_row + 6'd1;
            end
            // the request of the last clock lands at the queue's tail
            if (rq_v) begin
                q_e[q_t] <= {rd_info, txr_sz};
                q_b[q_t] <= bk_data;
                q_r[q_t] <= rq_row;
                q_c[q_t] <= rq_col;
            end
            if (in_ready) o_v <= 1'b0;
            if (pop) begin
                o_v <= 1'b1; in_blk <= q_b[q_h]; in_vpar <= e_vpar; in_hpar <= e_hpar;
                q_h <= !q_h;
                left_e[h_r[RW-1:0]] <= h_e;
                above_e <= h_e;
                if (h_r == n_blk - 6'd1) top_e[h_c[CW-1:0]] <= h_e;
            end
            q_n <= q_n + 2'(rq_v) - 2'(pop);
        end
    end
endmodule
