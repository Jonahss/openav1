// Event queue between the tile syntax decoder and the reconstruction stage, so the syntax side runs ahead
// instead of stalling on every transform block. Events, in decode order: superblock start (7.11 BlockDecoded
// clear, cdef_idx clear), block record, transform block record. A transform block's dequantised coefficients
// stay in coef_rd's multi-slot buffer (slot = tx push count mod NSLOT); the queue accepts a transform block
// only while a free slot is guaranteed (fewer than NSLOT - 1 in flight: the syntax side starts decoding the
// next block's coefficients into the following slot as soon as this one is accepted).
module rec_fifo
  import syn_pkg::*;
#(
    parameter int DEPTH = 32,
    parameter int NSLOT = 8
) (
    input  logic        clk,
    input  logic        rst,
    input  logic        flush,                // tile start
    // producers (tile_syntax): at most one push per cycle
    input  logic        sb_start,
    input  logic [10:0] sb_r, sb_c,
    input  logic        blk_info,
    input  blk_rec_t    blk_rec,
    input  logic        tx_done,
    input  tx_rec_t     tx_rec,
    output logic        tx_ack,
    output logic [2:0]  w_slot,               // coefficient slot the syntax side decodes into next
    // consumer (recon_top): head of the queue
    output logic        ev_valid,
    output logic [1:0]  ev_kind,              // 0 transform block, 1 block record, 2 superblock start
    output logic [10:0] ev_sb_r, ev_sb_c,
    output blk_rec_t    ev_blk,
    output tx_rec_t     ev_tx,
    input  logic        ev_pop,
    output logic [3:0]  tx_inflight
);
    localparam int AW = $clog2(DEPTH);
    logic [1:0]  kind_q [0:DEPTH-1];
    blk_rec_t    blk_q  [0:DEPTH-1];
    tx_rec_t     tx_q   [0:DEPTH-1];
    logic [10:0] sbr_q  [0:DEPTH-1], sbc_q [0:DEPTH-1];
    logic [AW-1:0] wr, rd;
    logic [AW:0]   count;
    logic [2:0]    tx_pushes;
    logic          push;
    logic [1:0]    push_kind;
    assign tx_ack = tx_done && (tx_inflight < 4'(NSLOT - 1)) && (count < (AW + 1)'(DEPTH - 2));
    assign push = sb_start || blk_info || tx_ack;
    assign push_kind = sb_start ? 2'd2 : blk_info ? 2'd1 : 2'd0;
    assign w_slot = tx_pushes;
    assign ev_valid = (count != '0);
    assign ev_kind = kind_q[rd];
    assign ev_blk = blk_q[rd];
    assign ev_tx = tx_q[rd];
    assign ev_sb_r = sbr_q[rd];
    assign ev_sb_c = sbc_q[rd];
    always_ff @(posedge clk) begin
        if (rst || flush) begin
            wr <= '0; rd <= '0; count <= '0; tx_pushes <= 3'd0; tx_inflight <= 4'd0;
        end else begin
            if (push) begin
                kind_q[wr] <= push_kind;
                if (push_kind == 2'd1) blk_q[wr] <= blk_rec;
                if (push_kind == 2'd0) tx_q[wr] <= tx_rec;
                if (push_kind == 2'd2) begin sbr_q[wr] <= sb_r; sbc_q[wr] <= sb_c; end
                wr <= wr + 1'b1;
                if (push_kind == 2'd0) tx_pushes <= tx_pushes + 3'd1;
            end
            if (ev_pop) rd <= rd + 1'b1;
            count <= count + (push ? (AW + 1)'(1) : (AW + 1)'(0)) - (ev_pop ? (AW + 1)'(1) : (AW + 1)'(0));
            tx_inflight <= tx_inflight + ((push && push_kind == 2'd0) ? 4'd1 : 4'd0) - ((ev_pop && ev_kind == 2'd0) ? 4'd1 : 4'd0);
        end
    end
endmodule
