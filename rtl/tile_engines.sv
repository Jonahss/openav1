// openav1 — TE tile syntax engines behind one reconstruction stage (notes: openav1-tile-columns.md, phase 1).
//
// AV1 tiles are independent for entropy decoding, intra prediction and motion-vector prediction (availability
// stops at the tile's MiColStart / MiColEnd); only the loop filters cross tile edges. So the tile columns of a
// tile row can be decoded by TE copies of tile_syntax at once, each with its own byte feed, CDF store, event
// queue and motion-info ring, while one recon_top consumes their events in raster order per superblock row:
// column 0's events until its superblock start for the next row shows at the head of its queue, then column 1's,
// and so on; a column whose tile has ended is skipped. The window accounting in dec_top (row_cnt per tile column,
// the hold on reconstruction) is unchanged, since the pops it counts arrive in row order exactly as with one engine
// and several tile rows. TE = 1 is the old single engine (eng_mask ignored, the header's own column range).
//
// Per-tile-row protocol (dec_host): the frame header carries the tile row's mi_row_start / end; eng_mask selects
// the engines that take part and col_start / col_end give each its MiColStart / MiColEnd; tile_start pulses them
// all; the host pushes every tile's bytes into that engine's FIFO (round-robin in chunks: an engine starved of
// bytes stalls the row, since reconstruction waits for its row before moving on); tile_done when every engine has
// finished and every queue has drained. The frame's initial CDF rows are the same for every tile, so the default
// loads (def_*) and the band writes (mf_*, sp_*) are broadcast; the CDF read-back selects an engine (rb_sel).
module tile_engines
  import syn_pkg::*;
  import cdf_map_pkg::*;
#(
    parameter int TE = 1,                     // tile engines
    parameter int INTER = 1,
    parameter int ML2R = 9,
    parameter int ML2C = 9,
    parameter int MFL2R = 8,
    parameter int MFL2C = 8,
    parameter int SPL2R = 9,
    parameter int SPL2W = 5
) (
    input  logic              clk,
    input  logic              rst,
    input  hdr_t              hdr,            // the frame header with the tile ROW's mi_row_start / end
    input  ihdr_t             ihdr,
    // per-engine tile geometry and byte feed
    input  logic [TE-1:0]     eng_mask,       // engines taking part in this tile row (0: engine 0 with hdr's own columns)
    input  logic [TE*MIW-1:0] col_start,      // MiColStart per engine
    input  logic [TE*MIW-1:0] col_end,        // MiColEnd per engine
    input  logic [TE*8-1:0]   in_data,
    input  logic [TE-1:0]     in_valid,
    output logic [TE-1:0]     in_ready,
    input  logic [TE-1:0]     in_eos,
    // CDF defaults (broadcast) and read-back (one engine)
    input  logic              def_we,
    input  logic [CDF_AW-1:0] def_addr,
    input  logic [245:0]      def_data,
    input  logic              rb_en,
    input  logic [CDF_AW-1:0] rb_addr,
    input  logic [$clog2(TE > 1 ? TE : 2)-1:0] rb_sel,
    output logic [245:0]      rb_data,
    // the previous frame's bands (broadcast)
    input  logic              mf_we,
    input  logic [2:0]        mf_wref,
    input  logic [MIW-2:0]    mf_wy8, mf_wx8,
    input  logic              mf_wvalid,
    input  logic signed [15:0] mf_wrow, mf_wcol,
    input  logic              sp_we,
    input  logic [MIW-1:0]    sp_wrow,
    input  logic [MIW-5:0]    sp_wwc,
    input  logic [47:0]       sp_wdata,
    // tile row control
    input  logic              tile_start,
    output logic              tile_busy,
    output logic              tile_done,      // one pulse: every engine finished and every queue drained
    output logic              unsupported,
    // the event stream to reconstruction (one rec_fifo head at a time)
    output logic              ev_valid,
    output logic [1:0]        ev_kind,
    output logic [MIW-1:0]    ev_sb_r, ev_sb_c,
    output blk_rec_t          ev_blk,
    output tx_rec_t           ev_tx,
    input  logic              ev_pop,
    output logic [3:0]        tx_inflight,
    input  logic              rc_busy,        // reconstruction busy: engines are switched only between blocks
    // reconstruction's reads into the current engine
    input  logic [2:0]        q_slot_r,
    input  logic [9:0]        q_addr,
    output logic signed [20:0] q_data,
    output logic              pal_hold,
    input  logic              blk_ack,
    input  logic              pm_plane,
    input  logic [5:0]        pm_x, pm_y,
    output logic [2:0]        pm_idx,
    // loop-restoration unit records (merged, one per cycle)
    output logic              lr_done,
    output lr_rec_t           lr_rec,
    // observability (the current engine's)
    output logic              blk_done,
    output blk_rec_t          blk_rec,
    output logic              tx_done,
    output tx_rec_t           tx_rec,
    output logic              tx_ack
);
    localparam int EW = $clog2(TE > 1 ? TE : 2);
    // ---------------------------------------------------------------- the engines and their queues
    hdr_t        hdr_e [TE];
    logic [TE-1:0] e_busy, e_done, e_unsup, e_in_ready, e_blk_done, e_tx_done, e_tx_ack, e_pal_hold, e_blk_ack, e_lr_done;
    logic [TE-1:0] e_sb_start, e_blk_info;
    logic [MIW-1:0] e_sb_r [TE], e_sb_c [TE];
    blk_rec_t    e_blk_rec [TE];
    tx_rec_t     e_tx_rec [TE];
    lr_rec_t     e_lr_rec [TE];
    logic [2:0]  e_slot_w [TE];
    logic signed [20:0] e_q_data [TE];
    logic [2:0]  e_pm_idx [TE];
    logic [245:0] e_rb_data [TE];
    logic        e_start [TE];
    // queue heads
    logic [TE-1:0] h_valid;
    logic [1:0]  h_kind [TE];
    logic [MIW-1:0] h_sb_r [TE], h_sb_c [TE];
    blk_rec_t    h_blk [TE];
    tx_rec_t     h_tx [TE];
    logic [3:0]  h_infl [TE];
    logic [TE-1:0] h_pop;
    logic [TE-1:0] mask;                                      // the engines of this tile row (eng_mask, or engine 0)
    assign mask = (TE == 1 || eng_mask == '0) ? TE'(1) : eng_mask;
    for (genvar e = 0; e < TE; e++) begin : g_e
        always_comb begin
            hdr_e[e] = hdr;
            if (TE > 1 && eng_mask != '0) begin
                hdr_e[e].mi_col_start = col_start[e * MIW +: MIW];
                hdr_e[e].mi_col_end   = col_end[e * MIW +: MIW];
            end
        end
        assign e_start[e] = tile_start && mask[e];
        tile_syntax #(.INTER(INTER), .ML2R(ML2R), .ML2C(ML2C), .MFL2R(MFL2R), .MFL2C(MFL2C), .SPL2R(SPL2R), .SPL2W(SPL2W)) u_ts (
            .clk, .rst, .hdr(hdr_e[e]), .ihdr,
            .in_data(in_data[e * 8 +: 8]), .in_valid(in_valid[e]), .in_ready(e_in_ready[e]), .in_eos(in_eos[e]),
            .def_we, .def_addr, .def_data, .rb_en(rb_en && (EW'(e) == rb_sel)), .rb_addr, .rb_data(e_rb_data[e]),
            .mf_we, .mf_wref, .mf_wy8, .mf_wx8, .mf_wvalid, .mf_wrow, .mf_wcol, .sp_we, .sp_wrow, .sp_wwc, .sp_wdata,
            .tile_start(e_start[e]), .tile_busy(e_busy[e]), .tile_done(e_done[e]), .unsupported(e_unsup[e]),
            .sb_start_o(e_sb_start[e]), .sb_r_o(e_sb_r[e]), .sb_c_o(e_sb_c[e]), .blk_info(e_blk_info[e]), .blk_done(e_blk_done[e]), .blk_rec(e_blk_rec[e]),
            .tx_done(e_tx_done[e]), .tx_rec(e_tx_rec[e]), .tx_ack(e_tx_ack[e]), .lr_done(e_lr_done[e]), .lr_rec(e_lr_rec[e]),
            .q_slot_w(e_slot_w[e]), .q_slot_r, .q_addr, .q_data(e_q_data[e]),
            .pal_hold(e_pal_hold[e]), .blk_ack(e_blk_ack[e]), .pm_plane, .pm_x, .pm_y, .pm_idx(e_pm_idx[e]));
        rec_fifo u_q (.clk, .rst, .flush(e_start[e]), .sb_start(e_sb_start[e]), .sb_r(e_sb_r[e]), .sb_c(e_sb_c[e]),
                      .blk_info(e_blk_info[e]), .blk_rec(e_blk_rec[e]), .tx_done(e_tx_done[e]), .tx_rec(e_tx_rec[e]), .tx_ack(e_tx_ack[e]),
                      .w_slot(e_slot_w[e]),
                      .ev_valid(h_valid[e]), .ev_kind(h_kind[e]), .ev_sb_r(h_sb_r[e]), .ev_sb_c(h_sb_c[e]), .ev_blk(h_blk[e]), .ev_tx(h_tx[e]),
                      .ev_pop(h_pop[e]), .tx_inflight(h_infl[e]));
    end
    assign in_ready = e_in_ready;
    assign rb_data  = e_rb_data[rb_sel];

    // ---------------------------------------------------------------- the arbiter: raster order per superblock row
    logic [EW-1:0]  cur;                                      // the engine whose queue feeds reconstruction
    logic [TE-1:0]  done_lvl;                                 // engine finished its tile (its queue may still hold events)
    logic [TE-1:0]  exhausted;                                // finished and drained
    logic           row_valid;
    logic [MIW-1:0] cur_row;                                  // superblock row (in MI rows, superblock aligned) being reconstructed
    logic [4:0]     sb_shift;
    logic           new_row, advance, all_done;
    logic [EW-1:0]  nxt;
    logic           wrap;
    assign sb_shift = hdr.sb128 ? 5'd5 : 5'd4;
    assign exhausted = done_lvl & ~h_valid;
    assign all_done = &(exhausted | ~mask);
    // the head of the current queue: a superblock start for a later row ends this engine's share of the row
    assign new_row = h_valid[cur] && (h_kind[cur] == 2'd2) && row_valid && ((h_sb_r[cur] >> sb_shift) != (cur_row >> sb_shift));
    // the next engine of the mask after cur that still has work (wrapping past the last engine = a new row begins;
    // the only engine left finds itself again, with wrap set)
    logic found;
    always_comb begin
        nxt = cur; wrap = 1'b0; found = 1'b0;
        for (int k = 1; k <= TE; k++) begin
            int c;
            c = (int'(cur) + k) % TE;
            if (!found && mask[c] && !exhausted[c]) begin found = 1'b1; nxt = EW'(c); wrap = (c <= int'(cur)); end
        end
    end
    assign advance = (TE > 1) && (new_row || (exhausted[cur] && !all_done)) && !rc_busy && (h_infl[cur] == 4'd0) && found;
    always_ff @(posedge clk) begin
        if (rst) begin cur <= '0; done_lvl <= '0; row_valid <= 1'b0; cur_row <= '0; end
        else begin
            if (tile_start) begin
                done_lvl <= '0; row_valid <= 1'b0;
                cur <= '0;
                for (int c = TE - 1; c >= 0; c--) if (mask[c]) cur <= EW'(c);      // the lowest engine of the mask
            end else begin
                done_lvl <= done_lvl | e_done;
                if (advance) begin
                    cur <= nxt;
                    if (wrap) row_valid <= 1'b0;                                 // the next head's superblock start sets the row
                end else if (ev_valid && ev_pop && (ev_kind == 2'd2) && !row_valid) begin
                    row_valid <= 1'b1; cur_row <= ev_sb_r;
                end
            end
        end
    end
    // the head presented to reconstruction: never the next row's superblock start of an engine still waiting its turn
    assign ev_valid = h_valid[cur] && !(TE > 1 && (new_row || exhausted[cur]));
    assign ev_kind  = h_kind[cur];
    assign ev_sb_r  = h_sb_r[cur];
    assign ev_sb_c  = h_sb_c[cur];
    assign ev_blk   = h_blk[cur];
    assign ev_tx    = h_tx[cur];
    assign tx_inflight = h_infl[cur];
    always_comb begin
        h_pop = '0; h_pop[cur] = ev_pop && ev_valid;
        e_blk_ack = '0; e_blk_ack[cur] = blk_ack;
    end
    assign q_data   = e_q_data[cur];
    assign pal_hold = e_pal_hold[cur];
    assign pm_idx   = e_pm_idx[cur];
    assign blk_done = e_blk_done[cur];
    assign blk_rec  = e_blk_rec[cur];
    assign tx_done  = e_tx_done[cur];
    assign tx_rec   = e_tx_rec[cur];
    assign tx_ack   = e_tx_ack[cur];
    assign tile_busy = |(e_busy & mask);
    assign unsupported = |(e_unsup & mask);
    // tile_done: one pulse when the last engine finishes and every queue has drained (dec_top then waits for
    // reconstruction to drain as before)
    logic done_sent;
    always_ff @(posedge clk) begin
        tile_done <= 1'b0;
        if (rst || tile_start) done_sent <= 1'b0;
        else if (!done_sent && all_done && !tile_busy && (|done_lvl)) begin tile_done <= 1'b1; done_sent <= 1'b1; end
    end

    // ---------------------------------------------------------------- loop-restoration records: per-engine queue, one out per cycle
    // (an engine emits at most one record per cycle; several engines may in the same cycle)
    if (TE == 1) begin : g_lr1
        assign lr_done = e_lr_done[0];
        assign lr_rec  = e_lr_rec[0];
    end else begin : g_lrn
        localparam int LQ = 8;
        lr_rec_t   lq [TE][LQ];
        logic [$clog2(LQ):0] lq_w [TE], lq_r [TE];
        logic [TE-1:0] lq_nonempty;
        for (genvar e = 0; e < TE; e++) begin : g_lq
            assign lq_nonempty[e] = (lq_w[e] != lq_r[e]);
        end
        logic [EW-1:0] lsel; logic lany;
        always_comb begin
            lsel = '0; lany = 1'b0;
            for (int e = TE - 1; e >= 0; e--) if (lq_nonempty[e]) begin lsel = EW'(e); lany = 1'b1; end
        end
        always_ff @(posedge clk) begin
            lr_done <= 1'b0;
            if (rst) begin
                for (int e = 0; e < TE; e++) begin lq_w[e] <= '0; lq_r[e] <= '0; end
            end else begin
                for (int e = 0; e < TE; e++) if (e_lr_done[e]) begin lq[e][lq_w[e][$clog2(LQ)-1:0]] <= e_lr_rec[e]; lq_w[e] <= lq_w[e] + 1'b1; end
                if (lany) begin lr_done <= 1'b1; lr_rec <= lq[lsel][lq_r[lsel][$clog2(LQ)-1:0]]; lq_r[lsel] <= lq_r[lsel] + 1'b1; end
            end
        end
    end
endmodule
