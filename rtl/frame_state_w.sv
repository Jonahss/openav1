// Frame-state sideband writer (docs/frame-state.md, stage A): what the next frames need from this one besides its
// pixels, captured from the block events in decode order and written to DDR per superblock row by dec_host's
// sideband DMA, so the host (stage A) or the card's loaders (stage B) can give the next frame its context without a
// model decoding alongside.
//   MV sideband (spec 7.19 motion field motion vector storage): one 32-bit word per 8x8 luma unit, taken from the
//     block that covers the unit's odd 4x4 (2*y8+1, 2*x8+1): {3'd0, mv_row[12:0], mv_col[12:0], ref[2:0]} with
//     ref = 0 (none) or LAST_FRAME..ALTREF_FRAME = 1..7: the first of the block's two references whose relative
//     distance to this frame is negative (ihdr.ref_dist_neg, one bit per reference, from the host) and whose MV
//     is within REFMVS_LIMIT (|mv| <= 4095). Two units per 64-bit DDR beat, the even unit in the low word.
//   Segment-id sideband (spec SegmentIds, saved as the slot's segmentation map): one byte per 4x4 unit, 8 per beat.
//     (A frame with segmentation_enabled and segmentation_update_map = 0 keeps the PREVIOUS map verbatim: the host
//     reuses the previous slot's area and ignores this one; 7.20 / test10147.)
// Rows are double-buffered: the fill of superblock row n+2 waits (fs_hold stalls the event pop) until the DMA has
// read row n. A row is complete when a block of a later superblock row is popped, or at tile_done for the last row:
// the completion rides the block queue, so it fires only after that row's last block was written.
module frame_state_w
  import syn_pkg::*;
  import blk_tables_pkg::*;
#(
    parameter int FBX = 11,                   // frame width log2 (samples): 1 << (FBX - 3) 8x8 units per row at most
    parameter int SB128 = 0                   // 1: 128x128 superblocks possible (16 8x8 / 32 4x4 lines per row)
) (
    input  logic          clk,
    input  logic          rst,
    input  hdr_t          hdr,
    input  ihdr_t         ihdr,
    input  logic          frame_start,
    input  logic          tile_done,          // the frame's last tile row finished (every block popped before this)
    // the block events reconstruction pops (dec_top: ev_valid && ev_pop)
    input  logic          ev_pop,             // a pop this cycle
    input  logic [1:0]    ev_kind,            // 1 = block record
    input  blk_rec_t      ev_blk,
    output logic          fs_hold,            // stall the pop: the block queue is full
    // a finished superblock row for the DMA: pulse + row index; the DMA answers fs_done when it has read it
    output logic          fs_row_ready,
    output logic [SBW-1:0] fs_row_idx,
    input  logic          fs_done,
    // read port for the DMA (registered, 1 cycle): beat index within the row's buffer; sel 0 = MV, 1 = segment ids
    input  logic          fs_rd_sel,
    input  logic [12:0]   fs_rd_addr,         // MV: {line[3:0], beat}; seg: {line[4:0], beat}
    input  logic          fs_rd_buf,
    output logic [63:0]   fs_rd_data
);
    localparam int W8L2 = FBX - 3;            // log2 of 8x8 units per row (max)
    localparam int MVB  = W8L2 - 1;           // log2 of 64-bit MV words per 8x8 line
    localparam int SEGW = FBX - 6;            // log2 of 128-bit seg words (16 units) per 4x4 line
    localparam int MVL  = 4;                  // 8x8 lines per superblock row buffer (16: sb128)
    localparam int SEGL = 5;                  // 4x4 lines per superblock row buffer (32: sb128)
    localparam int MVAW = 1 + MVL + MVB;      // {buf, line, word}
    localparam int SGAW = 1 + SEGL + SEGW;

    // ---------------------------------------------------------------- the memories (byte-enabled block RAM shape)
    (* ram_style = "block" *) logic [63:0]  mv_mem  [0:(1 << MVAW) - 1];
    (* ram_style = "block" *) logic [127:0] seg_mem [0:(1 << SGAW) - 1];
    logic             mv_we;  logic [MVAW-1:0] mv_wa, mv_ra;  logic [7:0]  mv_be;  logic [63:0]  mv_wd, mv_rd;
    logic             sg_we;  logic [SGAW-1:0] sg_wa, sg_ra;  logic [15:0] sg_be;  logic [127:0] sg_wd, sg_rd;
    always_ff @(posedge clk) begin
        if (mv_we) for (int b = 0; b < 8; b++) if (mv_be[b]) mv_mem[mv_wa][8*b +: 8] <= mv_wd[8*b +: 8];
        mv_rd <= mv_mem[mv_ra];
    end
    always_ff @(posedge clk) begin
        if (sg_we) for (int b = 0; b < 16; b++) if (sg_be[b]) seg_mem[sg_wa][8*b +: 8] <= sg_wd[8*b +: 8];
        sg_rd <= seg_mem[sg_ra];
    end
    // DMA reads: MV beat = one 64-bit word; seg beat = half of a 128-bit word
    logic rd_sel_q, rd_half_q;
    assign mv_ra = MVAW'({fs_rd_buf, fs_rd_addr[MVL+MVB-1:0]});
    assign sg_ra = SGAW'({fs_rd_buf, fs_rd_addr[SEGL+SEGW:1]});
    always_ff @(posedge clk) begin rd_sel_q <= fs_rd_sel; rd_half_q <= fs_rd_addr[0]; end
    assign fs_rd_data = rd_sel_q ? (rd_half_q ? sg_rd[127:64] : sg_rd[63:0]) : mv_rd;

    // ---------------------------------------------------------------- the block queue (row-end markers ride along)
    typedef struct packed {
        logic           row_end;               // marker: the row `row` is complete (no block fields)
        logic [SBW-1:0] row;                   // superblock row of the block / the completed row
        logic [MIW-1:0] r, c;
        logic [5:0]     bw4, bh4;
        logic [2:0]     seg;
        logic [28:0]    mv;                    // the MV sideband word (bits 28:0)
    } qent_t;
    localparam int QD = 8;
    qent_t          q [QD];
    logic [3:0]     q_w, q_r;
    logic           q_empty, q_full;
    assign q_empty = (q_w == q_r);
    assign q_full  = (q_w[2:0] == q_r[2:0]) && (q_w[3] != q_r[3]);
    assign fs_hold = q_full || (q_w[2:0] + 3'd1 == q_r[2:0] && q_w[3] != q_r[3]);   // one entry spare for the marker

    logic [4:0]     sb_shift;
    assign sb_shift = (SB128 != 0 && hdr.sb128) ? 5'd5 : 5'd4;
    logic           cur_valid;                 // a row is being filled
    logic [SBW-1:0] cur_row;
    logic [SBW-1:0] blk_row;
    assign blk_row = SBW'(ev_blk.r >> sb_shift);

    // the MV sideband word of the popped block (7.19): first list with a negative-distance reference and a small MV
    function automatic logic [28:0] mv_word(input blk_rec_t b, input logic [6:0] dneg);
        logic [28:0] w; logic ok0, ok1;
        ok0 = b.is_inter && !b.intrabc && (b.ref0 >= 4'd1) && (b.ref0 <= 4'd7) && dneg[3'(b.ref0 - 4'd1)]
              && (b.mv_row <= 18'sd4095) && (b.mv_row >= -18'sd4095) && (b.mv_col <= 18'sd4095) && (b.mv_col >= -18'sd4095);
        ok1 = b.is_inter && !b.intrabc && (b.ref1 >= 4'd1) && (b.ref1 <= 4'd7) && dneg[3'(b.ref1 - 4'd1)]
              && (b.mv1_row <= 18'sd4095) && (b.mv1_row >= -18'sd4095) && (b.mv1_col <= 18'sd4095) && (b.mv1_col >= -18'sd4095);
        if (ok0)      w = {b.mv_row[12:0], b.mv_col[12:0], b.ref0[2:0]};
        else if (ok1) w = {b.mv1_row[12:0], b.mv1_col[12:0], b.ref1[2:0]};
        else          w = '0;
        return w;
    endfunction

    logic push_blk, push_end;
    assign push_blk = ev_pop && (ev_kind == 2'd1);
    assign push_end = (push_blk && cur_valid && blk_row != cur_row) || (tile_done && cur_valid);
    always_ff @(posedge clk) begin
        if (rst || frame_start) begin
            q_w <= '0; cur_valid <= 1'b0; cur_row <= '0;
        end else begin
            // a block of a later row (or the frame's end) completes the current row: the marker goes in ahead of the block
            if (push_end) begin
                q[q_w[2:0]] <= '{row_end: 1'b1, row: cur_row, r: '0, c: '0, bw4: '0, bh4: '0, seg: '0, mv: '0};
                q_w <= q_w + 4'd1;
                cur_valid <= 1'b0;
            end
            if (push_blk) begin
                q[3'(push_end ? q_w + 4'd1 : q_w)] <= '{row_end: 1'b0, row: blk_row, r: ev_blk.r, c: ev_blk.c,
                                                                  bw4: num4x4w(ev_blk.bsize), bh4: num4x4h(ev_blk.bsize),
                                                                  seg: ev_blk.seg, mv: mv_word(ev_blk, ihdr.ref_dist_neg)};
                q_w <= (push_end ? q_w + 4'd1 : q_w) + 4'd1;
                cur_valid <= 1'b1; cur_row <= blk_row;
            end
        end
    end
    // (the marker and a block in one cycle: the index arithmetic above keeps the marker first; q_w wraps mod 16 with
    // 8 entries, the usual full/empty trick, so `& 7` is the slot of q_w + 1)

    // ---------------------------------------------------------------- the fill: segment bytes, then MV words
    typedef enum logic [2:0] {F_IDLE, F_SEG, F_MV, F_END, F_WAIT} fstate_t;
    fstate_t        fs;
    qent_t          cur;
    logic [MIW-1:0] f_r, f_c, r_end, c_end;    // the block's clipped area [r, r_end) x [c, c_end)
    logic [MIW-1:0] s_row;  logic [SEGW-1:0] s_wc;          // seg: current 4x4 line and 16-wide word column
    logic [MIW-1:0] m_row, m_col;                            // mv: current odd 4x4 position
    logic           pend [2];                                // buffer b holds a finished row the DMA has not read yet
    logic [SBW-1:0] pend_row [2];
    logic           fs_buf;                                  // the buffer of the row being filled (row parity)
    assign fs_buf = cur.row[0];
    logic           dma_buf;                                 // the buffer the DMA is reading
    logic [MIW-1:0] c_lo, c_hi;                              // seg word's covered column range
    always_comb begin
        c_lo = (MIW'({s_wc, 4'd0}) <= f_c) ? f_c : MIW'({s_wc, 4'd0});
        c_hi = (MIW'({s_wc, 4'd0}) + MIW'(16) < c_end) ? MIW'({s_wc, 4'd0}) + MIW'(16) : c_end;
        sg_be = '0;
        for (int k = 0; k < 16; k++) sg_be[k] = (MIW'({s_wc, 4'd0}) + MIW'(k) >= c_lo) && (MIW'({s_wc, 4'd0}) + MIW'(k) < c_hi);
        sg_wd = {16{5'd0, cur.seg}};
        sg_wa = SGAW'({fs_buf, s_row[SEGL-1:0], s_wc});
        sg_we = (fs == F_SEG);
        mv_wa = MVAW'({fs_buf, m_row[MVL:1], m_col[MVB+1:2]});   // 8x8 unit (m_row >> 1, m_col >> 1); two units per word
        mv_be = m_col[1] ? 8'hF0 : 8'h0F;
        mv_wd = {2{3'd0, cur.mv}};
        mv_we = (fs == F_MV);
    end
    logic [MIW-1:0] mv_r0, mv_c0;                            // the block's first odd row / column
    assign mv_r0 = cur.r | MIW'(1);
    assign mv_c0 = cur.c | MIW'(1);
    always_ff @(posedge clk) begin
        fs_row_ready <= 1'b0;
        if (rst || frame_start) begin
            fs <= F_IDLE; q_r <= '0; pend[0] <= 1'b0; pend[1] <= 1'b0; pend_row[0] <= '0; pend_row[1] <= '0; fs_row_idx <= '0;
            s_row <= '0; s_wc <= '0; m_row <= '0; m_col <= '0; f_c <= '0; r_end <= '0; c_end <= '0; cur <= '0;
        end else begin
            if (fs_done) pend[dma_buf] <= 1'b0;
            case (fs)
                F_IDLE: if (!q_empty) begin
                    cur <= q[q_r[2:0]]; q_r <= q_r + 4'd1;
                    fs <= q[q_r[2:0]].row_end ? F_END : F_WAIT;
                end
                F_WAIT: begin                                   // the row's buffer must be free of the DMA
                    f_c <= cur.c;
                    r_end <= (MIW'(cur.r) + MIW'(cur.bh4) < hdr.mi_rows) ? MIW'(cur.r) + MIW'(cur.bh4) : hdr.mi_rows;
                    c_end <= (MIW'(cur.c) + MIW'(cur.bw4) < hdr.mi_cols) ? MIW'(cur.c) + MIW'(cur.bw4) : hdr.mi_cols;
                    s_row <= cur.r; s_wc <= cur.c[SEGW+3:4];
                    m_row <= mv_r0; m_col <= mv_c0;
                    if (!pend[fs_buf]) fs <= F_SEG;
                end
                F_SEG: begin                                    // one 16-unit word of one 4x4 line per cycle
                    if (MIW'({s_wc, 4'd0}) + MIW'(16) < c_end) s_wc <= s_wc + 1'b1;
                    else begin
                        s_wc <= f_c[SEGW+3:4];
                        if (s_row + MIW'(1) < r_end) s_row <= s_row + MIW'(1);
                        else fs <= (mv_r0 < r_end && mv_c0 < c_end) ? F_MV : F_IDLE;
                    end
                end
                F_MV: begin                                     // one 8x8 unit per cycle over the block's odd positions
                    if (m_col + MIW'(2) < c_end) m_col <= m_col + MIW'(2);
                    else begin
                        m_col <= mv_c0;
                        if (m_row + MIW'(2) < r_end) m_row <= m_row + MIW'(2);
                        else fs <= F_IDLE;
                    end
                end
                F_END: begin                                    // the row is complete: hand it to the DMA
                    pend[cur.row[0]] <= 1'b1; pend_row[cur.row[0]] <= cur.row;
                    fs <= F_IDLE;
                end
                default: fs <= F_IDLE;
            endcase
            // the DMA takes finished rows in order (row parity alternates)
            if (!fs_row_ready && !dma_busy_q && (pend[dma_buf])) begin fs_row_ready <= 1'b1; fs_row_idx <= pend_row[dma_buf]; end
        end
    end
    // the DMA's busy, tracked here from ready / done
    logic dma_busy_q;
    always_ff @(posedge clk) begin
        if (rst || frame_start) begin dma_busy_q <= 1'b0; dma_buf <= 1'b0; end
        else if (fs_row_ready) dma_busy_q <= 1'b1;
        else if (fs_done) begin dma_busy_q <= 1'b0; dma_buf <= ~dma_buf; end
    end
endmodule
