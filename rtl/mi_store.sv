// Per-4x4 frame state for the in-loop filters (spec arrays MiSizes, Skips, SegmentIds, DeltaLFs and
// LoopfilterTxSizes[plane]). Two independent memories:
//   lf_info[mi_row][mi_col]         : mi_lf_t, written per block (blk_we iterates the block's bh4 x bw4 area)
//   lf_tx[plane][row_p][col_p]      : transform size, written per transform block (tx_we iterates h4 x w4)
// Both writers run as small FSMs with busy flags; reads are registered (1-cycle latency).
// Capacity: (1 << ML2R) x (1 << ML2C) 4x4 units per plane array.
module mi_store
  import syn_pkg::*;
#(
    parameter int ML2R = 8,
    parameter int ML2C = 8
) (
    input  logic        clk,
    input  logic        rst,
    // block write: the block at (r, c), bw4 x bh4 units
    input  logic        blk_we,
    input  logic [10:0] blk_r, blk_c,
    input  logic [5:0]  blk_bw4, blk_bh4,
    input  mi_lf_t      blk_data,
    output logic        blk_busy,
    // transform-block write: plane, position and size in 4x4 units of the plane
    input  logic        tx_we,
    input  logic [1:0]  tx_plane,
    input  logic [10:0] tx_row, tx_col,
    input  logic [4:0]  tx_w4, tx_h4,
    input  logic [4:0]  tx_sz,
    output logic        tx_busy,
    // reads (registered)
    input  logic [10:0] rd_row, rd_col,
    output mi_lf_t      rd_info,
    input  logic [1:0]  txr_plane,
    input  logic [10:0] txr_row, txr_col,
    output logic [4:0]  txr_sz
);
    localparam int N = 1 << (ML2R + ML2C);
    mi_lf_t     lf_info [0:N-1];
    logic [4:0] lf_tx0 [0:N-1];
    logic [4:0] lf_tx1 [0:N-1];
    logic [4:0] lf_tx2 [0:N-1];

    function automatic logic [ML2R+ML2C-1:0] idx(input logic [10:0] row, input logic [10:0] col);
        idx = {row[ML2R-1:0], col[ML2C-1:0]};
    endfunction

    // ---- block writer
    logic [5:0]  bi, bj;
    logic [10:0] b_r, b_c;
    logic [5:0]  b_w, b_h;
    mi_lf_t      b_d;
    always_ff @(posedge clk) begin
        if (rst) blk_busy <= 1'b0;
        else if (!blk_busy) begin
            if (blk_we) begin
                blk_busy <= 1'b1; b_r <= blk_r; b_c <= blk_c; b_w <= blk_bw4; b_h <= blk_bh4; b_d <= blk_data; bi <= 6'd0; bj <= 6'd0;
            end
        end else begin
            logic [ML2R+ML2C-1:0] a;
            a = idx(b_r + 11'(bi), b_c + 11'(bj));                 // (Verilator: no function call in an NBA LHS index)
            lf_info[a] <= b_d;
            if (bj + 6'd1 < b_w) bj <= bj + 6'd1;
            else begin
                bj <= 6'd0;
                if (bi + 6'd1 < b_h) bi <= bi + 6'd1;
                else blk_busy <= 1'b0;
            end
        end
    end

    // ---- transform-block writer
    logic [4:0]  ti, tj;
    logic [1:0]  t_p;
    logic [10:0] t_r, t_c;
    logic [4:0]  t_w, t_h, t_sz;
    always_ff @(posedge clk) begin
        if (rst) tx_busy <= 1'b0;
        else if (!tx_busy) begin
            if (tx_we) begin
                tx_busy <= 1'b1; t_p <= tx_plane; t_r <= tx_row; t_c <= tx_col; t_w <= tx_w4; t_h <= tx_h4; t_sz <= tx_sz; ti <= 5'd0; tj <= 5'd0;
            end
        end else begin
            logic [ML2R+ML2C-1:0] a;
            a = idx(t_r + 11'(ti), t_c + 11'(tj));
            case (t_p)
                2'd0: lf_tx0[a] <= t_sz;
                2'd1: lf_tx1[a] <= t_sz;
                default: lf_tx2[a] <= t_sz;
            endcase
            if (tj + 5'd1 < t_w) tj <= tj + 5'd1;
            else begin
                tj <= 5'd0;
                if (ti + 5'd1 < t_h) ti <= ti + 5'd1;
                else tx_busy <= 1'b0;
            end
        end
    end

    // ---- reads
    always_ff @(posedge clk) begin
        rd_info <= lf_info[idx(rd_row, rd_col)];
        case (txr_plane)
            2'd0: txr_sz <= lf_tx0[idx(txr_row, txr_col)];
            2'd1: txr_sz <= lf_tx1[idx(txr_row, txr_col)];
            default: txr_sz <= lf_tx2[idx(txr_row, txr_col)];
        endcase
    end
endmodule
