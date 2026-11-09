// Reference fetch (stage 2a): reads the clamped reference region of one prediction unit from a reference picture in
// DDR (the DMA flush layout: one byte per sample, or 16-bit little-endian samples) through an AXI4 read master, and
// writes it into mc_filt's patch buffer.
//
// Request: list, plane geometry (base, stride, last column / row index of the plane), the unit's integer position
// (x0, y0: may lie outside the picture) and size (w, h <= 64). The region is rows [y0 - 3, y0 + h + 4) and columns
// [x0 - 3, x0 - 3 + 4 (ngrp + 2)), ngrp = ceil(w / 4) (mc_filt reads whole 4-sample words up to word ngrp + 1), with
// 7.11.3.4's clamping = edge replication: a row outside the picture is the nearest picture row, a column outside the
// nearest column. Per row: the clamped column range [cx_lo, cx_hi] is read as 8-byte INCR bursts (<= 16 beats, never
// across 4 KB) into a line buffer, then the patch words are written with each lane's column clamped into that range.
// Rows that clamp to the same picture row are read again (simple; bandwidth is not the limit in stage 2a).
//
// Pipelined rows (2026-11-03, docs/reference-prefetch.md step A). Three parts share a ring of RING line buffers:
//   - the address side walks the rows (clamp, row address, burst split) and issues a row's bursts as soon as a line
//     buffer is free, so up to RING rows are in flight ahead of the writer;
//   - the data side takes the read responses in order (one AXI ID, so the interconnect answers in order) into the
//     line buffer of the row they belong to;
//   - the writer produces the patch words of each completed row.
// A patch then costs about L + rows x max(beats, words) cycles instead of rows x (L + beats + words), L being the
// read latency the memory path shows: ~48 cycles through the U200's XDMA interconnect and DDR4 controller (measured
// 2026-11-02), which made the serial engine the inter frames' wall. Request, clamping, burst rules, the patch write
// port and `done` are unchanged; the simulation twin's memory answers in a few cycles, so the gain shows on the
// card and under tb/axi_mem.py's TS_AXI_LAT / TS_AXI_OUT.
//
// Handover between requests (2026-11-09, docs/reference-prefetch.md section 8, step C1). A request is accepted as soon
// as the address side is idle: the ring may still hold the previous request's rows and the writer may still be on
// them, so each row carries its request's geometry in its ring slot (s_*), set when the row takes the slot, and the
// writer / data side read it from the slot of the row they work on. The request names the patch buffer its rows go to
// (req_buf -> patch_buf with every write) and the writer takes a row only while that buffer is not being read
// (buf_busy, mc_filt's horizontal side): rows of the next-but-one strip wait in the ring until the run that read their
// buffer has finished. done pulses once per request, in order, at the last word of its last row; an empty request
// (no rows) is accepted only when nothing is outstanding, so its done cannot overtake another's.
module ref_fetch #(
    parameter int PW = 12,
    parameter int RING = 16                               // line buffers = rows in flight ahead of the writer
) (
    input  logic          clk,
    input  logic          rst,
    // ---- request (accepted when req_ready) ----
    input  logic          req_valid,
    output logic          req_ready,
    input  logic          req_lst,
    input  logic          req_fmt16,            // 16-bit samples in memory (else one byte each)
    input  logic [39:0]   req_base,             // plane base (8-aligned)
    input  logic [31:0]   req_stride,           // plane stride in bytes (multiple of 8)
    input  logic [15:0]   req_lastx,            // last sample column of the plane (((RefUpscaledWidth + subX) >> subX) - 1)
    input  logic [15:0]   req_lasty,            // last sample row
    input  logic signed [16:0] req_x0,          // unit position, integer samples (unscaled: startX >> 10)
    input  logic signed [16:0] req_y0,
    input  logic [6:0]    req_w,                // 2 .. 64
    input  logic [6:0]    req_h,
    input  logic [6:0]    req_r_lo,             // patch rows [r_lo, r_hi) to produce; r_hi = 0: all h + 7 rows
    input  logic [6:0]    req_r_hi,
    input  logic          req_buf,              // the patch buffer the rows go to (named with every patch write)
    input  logic [1:0]    buf_busy,             // per patch buffer: being read (mc_filt h_buf_busy); the writer waits for it
    output logic          done,                 // 1-cycle pulse: the patch is complete (one per request, in order)
    output logic          err,                  // sticky: a read response other than OKAY
    // ---- AXI4 read master ----
    output logic [39:0]   m_axi_araddr,
    output logic [7:0]    m_axi_arlen,
    output logic [2:0]    m_axi_arsize,
    output logic [1:0]    m_axi_arburst,
    output logic [3:0]    m_axi_arcache,
    output logic [2:0]    m_axi_arprot,
    output logic          m_axi_arvalid,
    input  logic          m_axi_arready,
    input  logic [63:0]   m_axi_rdata,
    input  logic [1:0]    m_axi_rresp,
    input  logic          m_axi_rlast,
    input  logic          m_axi_rvalid,
    output logic          m_axi_rready,
    // ---- patch write (mc_filt) ----
    output logic          patch_we,
    output logic          patch_buf,
    output logic          patch_list,
    output logic [6:0]    patch_row,
    output logic [4:0]    patch_w4,
    output logic [4*PW-1:0] patch_d4
);
    localparam int LINE = 80;                             // line buffer samples (a row reads at most 72)
    localparam int SW = (RING > 1) ? $clog2(RING) : 1;    // ring slot index
    localparam int CW = $clog2(RING + 1);                 // ring occupancy counters, 0 .. RING

    // request copy (per patch)
    logic        q_lst, q_fmt16, q_buf;
    logic [39:0] q_base;
    logic [31:0] q_stride;
    logic [15:0] q_lastx, q_lasty;
    logic signed [16:0] q_x0, q_y0;
    logic [4:0]  q_nw;                                    // patch words per row = ngrp + 2
    logic [6:0]  q_nrow;                                  // last row + 1
    logic [15:0] cx_lo, cx_hi;                            // clamped column range of the region
    logic [15:0] nsamp;                                   // cx_hi - cx_lo + 1: samples per row

    // ---- address side ----
    typedef enum logic [2:0] {A_IDLE, A_PREP, A_ROW, A_MUL, A_ADDR, A_AR, A_NEXT} ast_t;
    ast_t ast;
    logic [6:0]  r_ar;                                    // row whose bursts are being issued
    logic [15:0] src_y;
    logic [47:0] mul;                                     // src_y * stride
    logic [39:0] cur_addr;
    logic [15:0] beats_left;                              // beats of the row still to issue
    logic [SW-1:0] slot_ar;                               // line buffer the row being issued was given

    // ---- ring ----
    logic [PW-1:0] line [0:RING-1][0:LINE-1];
    logic [2:0]  row_lead  [0:RING-1];                    // per slot: bytes before the first wanted byte in the row's first beat
    logic [15:0] row_beats [0:RING-1];                    // per slot: beats of the row
    logic [CW-1:0] rows_alloc;                            // rows issued and not yet written: ring occupancy
    logic [CW-1:0] rows_ready;                            // rows fully received and not yet taken by the writer
    // per slot: the row's request geometry (rows of consecutive requests share the ring)
    logic        s_fmt16 [0:RING-1];
    logic        s_lst   [0:RING-1];
    logic        s_buf   [0:RING-1];
    logic        s_last  [0:RING-1];                      // the request's last row: its last word pulses done
    logic [15:0] s_nsamp [0:RING-1];
    logic [15:0] s_cxlo  [0:RING-1];
    logic [15:0] s_lastx [0:RING-1];
    logic signed [16:0] s_x0 [0:RING-1];
    logic [4:0]  s_nw    [0:RING-1];
    logic [6:0]  s_row   [0:RING-1];

    // ---- data side ----
    logic [SW-1:0] slot_rx;                               // line buffer of the row being received
    logic [15:0] beats_rx;                                // beats received of that row
    logic [6:0]  wptr;                                    // next line sample to write
    logic        first_beat;

    // ---- writer ----
    logic        wr_busy;
    logic [SW-1:0] slot_wr;
    logic [4:0]  w4;                                      // patch word being written
    logic        w_lst, w_buf, w_last;                    // the row's geometry, copied from its slot when the writer takes it
    logic [15:0] w_cxlo, w_lastx;
    logic signed [16:0] w_x0;
    logic [4:0]  w_nw;
    logic [6:0]  w_row;                                   // patch row being written

    function automatic logic [15:0] clip_col(input logic signed [17:0] c, input logic [15:0] last);
        if (c < 18'sd0) return 16'd0;
        if (c > 18'(signed'({2'b0, last}))) return last;
        return 16'(c);
    endfunction
    function automatic logic [SW-1:0] slot_next(input logic [SW-1:0] s);
        return (int'(s) == RING - 1) ? SW'(0) : SW'(s + 1'b1);
    endfunction

    // observability: recon_top's B_MC_FETCH heartbeat and tb/test_dec_top.py print `st` as a number
    logic [7:0] st;
    assign st = {wr_busy, 4'(rows_alloc), 3'(ast)};

    // events
    logic accept, alloc, rx_beat, rx_last, wr_take, wr_last;
    assign accept  = (ast == A_IDLE) && req_valid && req_ready;
    assign alloc   = (ast == A_ADDR);                                              // the row takes its line buffer
    assign rx_beat = m_axi_rvalid && m_axi_rready;
    assign rx_last = rx_beat && (beats_rx == row_beats[slot_rx] - 16'd1);           // the row's last beat
    assign wr_take = !wr_busy && (rows_ready != CW'(0)) && !buf_busy[s_buf[slot_wr]];   // the head row's buffer is free to write
    assign wr_last = wr_busy && (w4 == w_nw - 5'd1);                               // the row's last word: its slot is free

    // a request with rows is accepted whenever the address side is idle; an empty one only when nothing is outstanding
    logic req_empty;
    assign req_empty = !(req_r_lo < ((req_r_hi == 7'd0) ? req_h + 7'd7 : req_r_hi));
    assign req_ready = (ast == A_IDLE) && (!req_empty || (!wr_busy && (rows_alloc == CW'(0))));
    assign m_axi_arsize  = 3'b011;                        // 8 bytes
    assign m_axi_arburst = 2'b01;                         // INCR
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;
    assign m_axi_araddr  = cur_addr;
    assign m_axi_arvalid = (ast == A_AR);
    assign m_axi_rready  = 1'b1;                          // every response belongs to a row that owns a line buffer

    // burst length: <= 16 beats, <= beats_left, never across a 4 KB boundary
    logic [4:0] burst_len;
    logic [9:0] to4k;
    assign to4k = 10'((13'd4096 - 13'(cur_addr[11:0])) >> 3);
    always_comb begin
        burst_len = 5'd16;
        if (beats_left < 16'(burst_len)) burst_len = 5'(beats_left);
        if (to4k < 10'(burst_len)) burst_len = 5'(to4k);
    end
    assign m_axi_arlen = 8'(burst_len - 5'd1);

    // patch word lanes: column of lane l clamped into the region's range, read from the row's line buffer
    logic [4*PW-1:0] word_d4;
    always_comb begin
        for (int l = 0; l < 4; l++) begin
            logic signed [17:0] col;
            logic [15:0] cc;
            col = 18'(w_x0) - 18'sd3 + 18'(signed'({13'd0, w4})) * 18'sd4 + 18'(l);
            cc = clip_col(col, w_lastx) - w_cxlo;
            word_d4[l*PW +: PW] = line[slot_wr][cc[6:0]];
        end
    end
    assign patch_d4 = word_d4;
    assign patch_buf = w_buf;
    assign patch_list = w_lst;
    assign patch_row = w_row;
    assign patch_w4 = w4;
    assign patch_we = wr_busy;

    // the samples of a beat (8 bytes): 8 one-byte samples or 4 16-bit samples
    function automatic logic [PW-1:0] beat_sample(input logic [63:0] d, input logic fmt16, input int k);
        // 16-bit samples occupy lanes 0..3 only (the beat loop stops at nk); the masked index keeps the
        // select in range for the unrolled k = 4..7 copies, which Vivado elaborates even though they never run
        if (fmt16) return PW'(d[(k & 3)*16 +: 16]);
        return PW'(d[k*8 +: 8]);
    endfunction

    // done: an empty request (no rows to produce) or the last word of the last row; err: sticky on a bad response
    logic empty_req;
    assign empty_req = accept && req_empty;
    always_ff @(posedge clk) done <= !rst && (empty_req || (wr_last && w_last));
    always_ff @(posedge clk) if (rst) err <= 1'b0; else if (rx_beat && m_axi_rresp != 2'b00) err <= 1'b1;

    // ---- address side ----
    always_ff @(posedge clk) begin
        if (rst) begin
            ast <= A_IDLE; slot_ar <= SW'(0);
        end else case (ast)
            A_IDLE: if (req_valid && req_ready) begin
                logic [4:0] nw;
                logic [6:0] nrow;
                nw = 5'((req_w + 7'd3) >> 2) + 5'd2;
                nrow = (req_r_hi == 7'd0) ? req_h + 7'd7 : req_r_hi;
                q_lst <= req_lst; q_fmt16 <= req_fmt16; q_buf <= req_buf; q_base <= req_base; q_stride <= req_stride;
                q_lastx <= req_lastx; q_lasty <= req_lasty; q_x0 <= req_x0; q_y0 <= req_y0;
                q_nw <= nw; q_nrow <= nrow;
                cx_lo <= clip_col(18'(req_x0) - 18'sd3, req_lastx);
                cx_hi <= clip_col(18'(req_x0) - 18'sd3 + 18'(signed'({13'd0, nw})) * 18'sd4 - 18'sd1, req_lastx);
                r_ar <= req_r_lo;
                if (req_r_lo < nrow) ast <= A_PREP;                           // else: nothing to produce, done pulses
            end
            A_PREP: begin
                nsamp <= cx_hi - cx_lo + 16'd1;
                ast <= A_ROW;
            end
            A_ROW: if (rows_alloc < CW'(RING)) begin                          // a line buffer is free for this row
                logic signed [17:0] yy;
                yy = 18'(q_y0) - 18'sd3 + 18'(signed'({11'd0, r_ar}));
                src_y <= clip_col(yy, q_lasty);
                ast <= A_MUL;
            end
            A_MUL: begin
                mul <= 48'(src_y) * 48'(q_stride);
                ast <= A_ADDR;
            end
            A_ADDR: begin
                logic [39:0] a_lo, a_hi_excl;
                logic [15:0] nbytes;
                a_lo = q_base + 40'(mul) + (q_fmt16 ? {23'd0, cx_lo, 1'b0} : {24'd0, cx_lo});
                nbytes = q_fmt16 ? {nsamp[14:0], 1'b0} : nsamp;
                a_hi_excl = a_lo + 40'(nbytes);
                cur_addr <= {a_lo[39:3], 3'b000};
                beats_left <= 16'((a_hi_excl + 40'd7 - {a_lo[39:3], 3'b000}) >> 3);
                row_lead[slot_ar] <= a_lo[2:0];
                row_beats[slot_ar] <= 16'((a_hi_excl + 40'd7 - {a_lo[39:3], 3'b000}) >> 3);
                s_fmt16[slot_ar] <= q_fmt16; s_nsamp[slot_ar] <= nsamp; s_cxlo[slot_ar] <= cx_lo; s_x0[slot_ar] <= q_x0;
                s_lastx[slot_ar] <= q_lastx; s_nw[slot_ar] <= q_nw; s_lst[slot_ar] <= q_lst; s_buf[slot_ar] <= q_buf;
                s_row[slot_ar] <= r_ar; s_last[slot_ar] <= (r_ar == q_nrow - 7'd1);
                ast <= A_AR;
            end
            A_AR: if (m_axi_arready) begin
                cur_addr <= cur_addr + 40'(burst_len) * 40'd8;
                beats_left <= beats_left - 16'(burst_len);
                if (beats_left == 16'(burst_len)) ast <= A_NEXT;              // the row's last burst
            end
            A_NEXT: begin
                slot_ar <= slot_next(slot_ar);
                if (r_ar == q_nrow - 7'd1) ast <= A_IDLE;                     // all rows issued; the writer finishes the patch
                else begin r_ar <= r_ar + 7'd1; ast <= A_ROW; end
            end
            default: ast <= A_IDLE;
        endcase
    end

    // ---- data side: responses in order into the line buffer of the row they belong to ----
    always_ff @(posedge clk) begin
        if (rst) begin
            slot_rx <= SW'(0); beats_rx <= 16'd0; wptr <= 7'd0; first_beat <= 1'b1;
        end else if (rx_beat) begin
            logic [3:0] k0;                               // first wanted sample of the beat
            logic [3:0] nk;                               // samples in the beat
            logic [2:0] lead;
            lead = row_lead[slot_rx];
            nk = s_fmt16[slot_rx] ? 4'd4 : 4'd8;
            k0 = first_beat ? (s_fmt16[slot_rx] ? 4'(lead >> 1) : 4'(lead)) : 4'd0;
            for (int k = 0; k < 8; k++)
                if (4'(k) >= k0 && 4'(k) < nk && (wptr + 7'(k) - 7'(k0)) < 7'(s_nsamp[slot_rx]))
                    line[slot_rx][wptr + 7'(k) - 7'(k0)] <= beat_sample(m_axi_rdata, s_fmt16[slot_rx], k);
            wptr <= wptr + 7'(nk) - 7'(k0);
            first_beat <= 1'b0;
            beats_rx <= beats_rx + 16'd1;
            if (rx_last) begin
                slot_rx <= slot_next(slot_rx); beats_rx <= 16'd0; wptr <= 7'd0; first_beat <= 1'b1;
            end
        end
    end

    // ---- writer: the patch words of each completed row, in row order ----
    always_ff @(posedge clk) begin
        if (rst) begin
            wr_busy <= 1'b0; slot_wr <= SW'(0); w4 <= 5'd0; w_row <= 7'd0;
        end else begin
            if (wr_take) begin
                wr_busy <= 1'b1; w4 <= 5'd0;
                w_lst <= s_lst[slot_wr]; w_buf <= s_buf[slot_wr]; w_last <= s_last[slot_wr]; w_row <= s_row[slot_wr];
                w_x0 <= s_x0[slot_wr]; w_lastx <= s_lastx[slot_wr]; w_cxlo <= s_cxlo[slot_wr]; w_nw <= s_nw[slot_wr];
            end else if (wr_busy) begin
                if (wr_last) begin
                    wr_busy <= 1'b0; slot_wr <= slot_next(slot_wr);
                end else w4 <= w4 + 5'd1;
            end
        end
    end

    // ---- ring accounting ----
    always_ff @(posedge clk) begin
        if (rst) begin
            rows_alloc <= CW'(0); rows_ready <= CW'(0);
        end else begin
            rows_alloc <= rows_alloc + CW'(alloc) - CW'(wr_last);
            rows_ready <= rows_ready + CW'(rx_last) - CW'(wr_take);
        end
    end
endmodule
