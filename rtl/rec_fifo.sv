// Event queue between the tile syntax decoder and the reconstruction stage, so the syntax side runs ahead
// instead of stalling on every transform block. Events, in decode order: superblock start (7.11 BlockDecoded
// clear, cdef_idx clear), block record, transform block record. A transform block's dequantised coefficients
// stay in coef_rd's multi-slot buffer (slot = tx push count mod NSLOT); the queue accepts a transform block
// only while a free slot is guaranteed (fewer than NSLOT - 1 in flight: the syntax side starts decoding the
// next block's coefficients into the following slot as soon as this one is accepted).
//
// Early block take (2026-11-09): reconstruction's block FSM may take the next block record before the transform
// events ahead of it are consumed (they wait for that block's prediction to land, which kept the next block behind
// them). Every record carries its block's parity (ppar, toggled per block record pushed; a transform record takes its
// block's); nb_* is the next untaken block record at or after the head, found by a scan that stops at a superblock
// event (registered pointer, validated live); nb_pop marks it taken, and a taken record is popped as a no-op when it
// reaches the head (ev_taken), so the pop order seen by the frame-state tap is unchanged.
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
    input  logic [MIW-1:0] sb_r, sb_c,
    input  logic        blk_info,
    input  blk_rec_t    blk_rec,
    input  logic        tx_done,
    input  tx_rec_t     tx_rec,
    output logic        tx_ack,
    output logic [2:0]  w_slot,               // coefficient slot the syntax side decodes into next
    // consumer (recon_top): head of the queue
    output logic        ev_valid,
    output logic [1:0]  ev_kind,              // 0 transform block, 1 block record, 2 superblock start
    output logic [MIW-1:0] ev_sb_r, ev_sb_c,
    output blk_rec_t    ev_blk,
    output tx_rec_t     ev_tx,
    input  logic        ev_pop,
    output logic        ev_par,               // the head record's block parity (a transform record: its block's)
    output logic        ev_taken,             // the head is a block record already taken early (pop it as a no-op)
    output logic        nb_valid,             // the next untaken block record at or after the head (no superblock event before it)
    output blk_rec_t    nb_blk,
    output logic        nb_par,
    output logic        nb_at_head,           // that record is the head itself
    input  logic        nb_pop,               // taken early (not at the head)
    input  logic        tx_rel,               // reconstruction finished a transform block: its coefficient slot is free
    output logic [3:0]  tx_inflight
);
    localparam int AW = $clog2(DEPTH);
    logic [1:0]  kind_q [0:DEPTH-1];
    // record queues as flat vectors in distributed RAM (struct-typed arrays became 14k flops + read muxes)
    localparam int BW = $bits(blk_rec_t), TXW = $bits(tx_rec_t);
    (* ram_style = "distributed" *) logic [BW-1:0]  blk_q [0:DEPTH-1];
    (* ram_style = "distributed" *) logic [TXW-1:0] tx_q  [0:DEPTH-1];
    logic [MIW-1:0] sbr_q  [0:DEPTH-1], sbc_q [0:DEPTH-1];
    logic        par_q   [0:DEPTH-1];                      // the record's block parity
    logic        taken_q [0:DEPTH-1];                      // block record taken early
    logic        ppar;                                     // parity of the block being pushed
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
    assign ev_blk = blk_rec_t'(blk_q[rd]);
    assign ev_tx = tx_rec_t'(tx_q[rd]);
    assign ev_par = par_q[rd];
    assign ev_taken = (kind_q[rd] == 2'd1) && taken_q[rd];
    // the next untaken block record: scanned from the head, stopping at a superblock event; the pointer is registered
    // (the scan is a long priority chain) and validated against the live flags, so a stale pointer never reads as valid
    logic [AW-1:0] nb_scan, nb_idx;
    logic          nb_found;
    always_comb begin
        logic stop;
        nb_found = 1'b0; nb_scan = rd; stop = 1'b0;
        for (int k = 0; k < DEPTH; k++) begin
            logic [AW-1:0] i;
            i = rd + AW'(k);
            if (!nb_found && !stop && (AW + 1)'(k) < count) begin
                // stop at a superblock event, and at a block record already taken early: the transform events before it
                // belong to the block whose parity the next record would reuse (bq[par]), so that record may be taken
                // only once the head has passed them (one block ahead of event consumption at most)
                if (kind_q[i] == 2'd2 || (kind_q[i] == 2'd1 && taken_q[i])) stop = 1'b1;
                else if (kind_q[i] == 2'd1) begin nb_found = 1'b1; nb_scan = i; end
            end
        end
    end
    logic nb_q_valid;
    always_ff @(posedge clk) begin
        if (rst || flush) begin nb_q_valid <= 1'b0; nb_idx <= '0; end
        else begin nb_q_valid <= nb_found; nb_idx <= nb_scan; end
    end
    // live validation: still an untaken block record inside the queue (a pop never passes an untaken block record,
    // and a superblock event never appears between the head and a record already behind one)
    assign nb_valid = nb_q_valid && (kind_q[nb_idx] == 2'd1) && !taken_q[nb_idx] && ((AW + 1)'(nb_idx - rd) < count);
    assign nb_blk = blk_rec_t'(blk_q[nb_idx]);
    assign nb_par = par_q[nb_idx];
    assign nb_at_head = nb_valid && (nb_idx == rd);
    // the queue memories in their own processes (one write port each, no reset: the RAM template)
    always_ff @(posedge clk) if (push && push_kind == 2'd1 && !(rst || flush)) blk_q[wr] <= BW'(blk_rec);
    always_ff @(posedge clk) if (push && push_kind == 2'd0 && !(rst || flush)) tx_q[wr] <= TXW'(tx_rec);
    assign ev_sb_r = sbr_q[rd];
    assign ev_sb_c = sbc_q[rd];
    always_ff @(posedge clk) begin
        if (rst || flush) begin
            wr <= '0; rd <= '0; count <= '0; tx_pushes <= 3'd0; tx_inflight <= 4'd0; ppar <= 1'b0;
        end else begin
            if (push) begin
                kind_q[wr] <= push_kind;
                if (push_kind == 2'd2) begin sbr_q[wr] <= sb_r; sbc_q[wr] <= sb_c; end
                if (push_kind == 2'd1) begin ppar <= !ppar; par_q[wr] <= !ppar; taken_q[wr] <= 1'b0; end
                else par_q[wr] <= ppar;
                wr <= wr + 1'b1;
                if (push_kind == 2'd0) tx_pushes <= tx_pushes + 3'd1;
            end
            if (nb_pop) taken_q[nb_idx] <= 1'b1;
            if (ev_pop) rd <= rd + 1'b1;
            count <= count + (push ? (AW + 1)'(1) : (AW + 1)'(0)) - (ev_pop ? (AW + 1)'(1) : (AW + 1)'(0));
            // in flight = pushed and not yet released by reconstruction (since 2026-11-08 the event is popped when taken, so the
            // pop no longer means the slot is free; before, the pop came at the end of the transform block)
            tx_inflight <= tx_inflight + ((push && push_kind == 2'd0) ? 4'd1 : 4'd0) - (tx_rel ? 4'd1 : 4'd0);
        end
    end
endmodule
