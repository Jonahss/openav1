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
module ref_fetch #(
    parameter int PW = 12
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
    output logic          done,                 // 1-cycle pulse: the patch is complete
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
    output logic          patch_list,
    output logic [6:0]    patch_row,
    output logic [4:0]    patch_w4,
    output logic [4*PW-1:0] patch_d4
);
    localparam int LINE = 80;                             // line buffer samples (a row reads at most 72)

    typedef enum logic [3:0] {F_IDLE, F_ROW, F_MUL, F_ADDR, F_AR, F_R, F_WR, F_NEXT} st_t;
    st_t st;

    // request copy
    logic        q_lst, q_fmt16;
    logic [39:0] q_base;
    logic [31:0] q_stride;
    logic [15:0] q_lastx, q_lasty;
    logic signed [16:0] q_x0, q_y0;
    logic [6:0]  q_h;
    logic [4:0]  q_nw;                                    // patch words per row = ngrp + 2
    logic [6:0]  q_nrow;                                  // h + 7
    logic [15:0] cx_lo, cx_hi;                            // clamped column range of the region
    logic [15:0] nsamp;                                   // cx_hi - cx_lo + 1

    // per row
    logic [6:0]  r;                                       // patch row
    logic [15:0] src_y;
    logic [47:0] mul;                                     // src_y * stride
    logic [39:0] row_addr, addr_lo, cur_addr;
    logic [2:0]  lead;                                    // bytes before the first wanted byte in the first beat
    logic [15:0] beats_left;
    logic [4:0]  len;                                     // beats of the burst being issued / received
    logic        first_beat;
    logic [6:0]  wptr;                                    // next line sample to write
    logic [PW-1:0] line [0:LINE-1];
    logic [4:0]  w4;                                      // patch word being written

    function automatic logic [15:0] clip_col(input logic signed [17:0] c, input logic [15:0] last);
        if (c < 18'sd0) return 16'd0;
        if (c > 18'(signed'({2'b0, last}))) return last;
        return 16'(c);
    endfunction

    assign req_ready = (st == F_IDLE);
    assign m_axi_arsize  = 3'b011;                        // 8 bytes
    assign m_axi_arburst = 2'b01;                         // INCR
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;
    assign m_axi_araddr  = cur_addr;
    assign m_axi_arlen   = 8'(burst_len - 5'd1);             // the length of the burst being offered (len latches it at the handshake)
    assign m_axi_arvalid = (st == F_AR);
    assign m_axi_rready  = (st == F_R);

    // burst length: <= 16 beats, <= beats_left, never across a 4 KB boundary
    logic [9:0] to4k;
    assign to4k = 10'((13'd4096 - 13'(cur_addr[11:0])) >> 3);
    logic [4:0] burst_len;
    always_comb begin
        burst_len = 5'd16;
        if (beats_left < 16'(burst_len)) burst_len = 5'(beats_left);
        if (to4k < 10'(burst_len)) burst_len = 5'(to4k);
    end

    // patch word lanes: column of lane l clamped into the region's range, read from the line buffer
    logic [4*PW-1:0] word_d4;
    always_comb begin
        for (int l = 0; l < 4; l++) begin
            logic signed [17:0] col;
            logic [15:0] cc;
            col = 18'(q_x0) - 18'sd3 + 18'(signed'({13'd0, w4})) * 18'sd4 + 18'(l);
            cc = clip_col(col, q_lastx) - cx_lo;
            word_d4[l*PW +: PW] = line[cc[6:0]];
        end
    end
    assign patch_d4 = word_d4;
    assign patch_list = q_lst;
    assign patch_row = r;
    assign patch_w4 = w4;
    assign patch_we = (st == F_WR);

    // the samples of a beat (8 bytes): 8 one-byte samples or 4 16-bit samples
    function automatic logic [PW-1:0] beat_sample(input logic [63:0] d, input logic fmt16, input int k);
        // 16-bit samples occupy lanes 0..3 only (the beat loop stops at nk); the masked index keeps the
        // select in range for the unrolled k = 4..7 copies, which Vivado elaborates even though they never run
        if (fmt16) return PW'(d[(k & 3)*16 +: 16]);
        return PW'(d[k*8 +: 8]);
    endfunction

    always_ff @(posedge clk) begin
        done <= 1'b0;
        if (rst) begin
            st <= F_IDLE; err <= 1'b0;
        end else case (st)
            F_IDLE: if (req_valid) begin
                logic [4:0] nw;
                nw = 5'((req_w + 7'd3) >> 2) + 5'd2;
                q_lst <= req_lst; q_fmt16 <= req_fmt16; q_base <= req_base; q_stride <= req_stride;
                q_lastx <= req_lastx; q_lasty <= req_lasty; q_x0 <= req_x0; q_y0 <= req_y0; q_h <= req_h;
                q_nw <= nw; q_nrow <= req_h + 7'd7;
                cx_lo <= clip_col(18'(req_x0) - 18'sd3, req_lastx);
                cx_hi <= clip_col(18'(req_x0) - 18'sd3 + 18'(signed'({13'd0, nw})) * 18'sd4 - 18'sd1, req_lastx);
                r <= 7'd0;
                st <= F_ROW;
            end
            F_ROW: begin
                logic signed [17:0] yy;
                yy = 18'(q_y0) - 18'sd3 + 18'(signed'({11'd0, r}));
                src_y <= clip_col(yy, q_lasty);
                nsamp <= cx_hi - cx_lo + 16'd1;
                st <= F_MUL;
            end
            F_MUL: begin
                mul <= 48'(src_y) * 48'(q_stride);
                st <= F_ADDR;
            end
            F_ADDR: begin
                logic [39:0] a_lo, a_hi_excl;
                logic [15:0] nbytes;
                a_lo = q_base + 40'(mul) + (q_fmt16 ? {23'd0, cx_lo, 1'b0} : {24'd0, cx_lo});
                nbytes = q_fmt16 ? {nsamp[14:0], 1'b0} : nsamp;
                a_hi_excl = a_lo + 40'(nbytes);
                lead <= a_lo[2:0];
                cur_addr <= {a_lo[39:3], 3'b000};
                beats_left <= 16'((a_hi_excl + 40'd7 - {a_lo[39:3], 3'b000}) >> 3);
                wptr <= 7'd0; first_beat <= 1'b1;
                st <= F_AR;
            end
            F_AR: if (m_axi_arready) begin len <= burst_len; st <= F_R; end
            F_R: if (m_axi_rvalid) begin
                logic [3:0] k0;                           // first wanted sample of the beat
                logic [3:0] nk;                           // samples in the beat
                if (m_axi_rresp != 2'b00) err <= 1'b1;
                nk = q_fmt16 ? 4'd4 : 4'd8;
                k0 = first_beat ? (q_fmt16 ? 4'(lead >> 1) : 4'(lead)) : 4'd0;
                for (int k = 0; k < 8; k++)
                    if (4'(k) >= k0 && 4'(k) < nk && (wptr + 7'(k) - 7'(k0)) < 7'(nsamp))
                        line[wptr + 7'(k) - 7'(k0)] <= beat_sample(m_axi_rdata, q_fmt16, k);
                wptr <= wptr + 7'(nk) - 7'(k0);
                first_beat <= 1'b0;
                cur_addr <= cur_addr + 40'd8;
                beats_left <= beats_left - 16'd1;
                if (m_axi_rlast) begin
                    if (beats_left == 16'd1) begin w4 <= 5'd0; st <= F_WR; end   // this was the row's last beat
                    else st <= F_AR;
                end
            end
            F_WR: begin
                if (w4 == q_nw - 5'd1) st <= F_NEXT;
                else w4 <= w4 + 5'd1;
            end
            F_NEXT: begin
                if (r == q_nrow - 7'd1) begin done <= 1'b1; st <= F_IDLE; end
                else begin r <= r + 7'd1; st <= F_ROW; end
            end
            default: st <= F_IDLE;
        endcase
    end
endmodule
