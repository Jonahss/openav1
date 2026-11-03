// Motion field projection engine (spec 7.9 motion_field_estimation; docs/frame-state.md stage B, the last block):
// at a frame start the card fills the motion-field band itself from the reference slots' saved MV words
// (rtl/frame_state_w.sv's sideband, one {ref, mv} per 8x8 unit) instead of the host writing every entry.
// The host supplies what the spec derives from the order hints alone: which sources project and in which order
// (LAST, BWDREF, ALTREF2, ALTREF, LAST2 with the refStamp rule), each source's dstSign and refToCur, its refOffset for
// each saved reference 1..7, and refToDst for each destination 1..7. The engine does 7.9.2 per cell:
//   posValid: refOffset > 0 and |refToCur|, |refOffset| <= MAX_FRAME_DISTANCE (31);
//   projMv = mv_projection(mv, refToCur * dstSign, refOffset); (PosY8, PosX8) by block_position (rows stay in the
//   64-pixel band, columns within 8 units); then for dst = 1..7: mf[dst][PosY8][PosX8] = mv_projection(mv, refToDst, refOffset)
// mv_projection(mv, num, den): round2signed(mv * clip3(-31, 31, num) * Div_Mult[min(31, den)], 14) clipped to +-16383.
// Cells go in raster order per source, sources in the host's order, so a later write wins at a shared target, as the
// model does. The field is cleared (every entry invalid) first.
//
// Throughput (2026-11-03): the clear writes all seven references of a position in one cycle (w8 x h8 cycles; it starts
// with mf_start, which the loader group raises together with the segment-map / CDF loader, so the clear overlaps that
// loader's reads: src_go says when the read channel is free); the reads run up to OUT bursts ahead into a beat FIFO
// whose room is reserved at issue (rready never drops); the cells flow through a pipeline at one cell a cycle: the
// source's constants and posValid (S1), the per-destination multipliers K = clip(num) * Div_Mult[den] (S2), the
// sixteen products mv * K (S3), round2signed and the clip (S4), block_position (S5), the seven-reference write (S6).
// The first version took 11 cycles a cell and 7 x w8 x h8 cycles for the clear: 1080p ran past 460k cycles a frame.
module mf_project
  import syn_pkg::*;
#(
    parameter int OUT = 8                                    // bursts in flight (a power of two)
) (
    input  logic          clk,
    input  logic          rst,
    input  hdr_t          hdr,
    input  logic          mf_start,
    input  logic          src_go,              // the read channel is free (the frame-state loader's reads are over)
    input  logic [2:0]    mf_nsrc,             // sources to project, 0..5, in order
    input  logic [199:0]  src_base,            // source s base at [40s +: 40]
    input  logic [79:0]   src_rowb,            // bytes per 8x8 line at [16s +: 16]
    input  logic [4:0]    src_sign,            // 1: dstSign = +1, 0: -1
    input  logic [39:0]   src_tocur,           // refToCur, signed 8 at [8s +: 8]
    input  logic [279:0]  src_off,             // refOffset of saved reference r (1..7) of source s: signed 8 at [8 (7s + r - 1) +: 8]
    input  logic [55:0]   dst_dist,            // refToDst for destination d (1..7): signed 8 at [8 (d - 1) +: 8]
    output logic          mf_busy,
    output logic          mf_done,
    output logic          mf_err,
    // AXI4 read master
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
    // the band (mf_band: ref 0 = LAST_FRAME; one write enable and one vector per reference)
    output logic [6:0]    mf_we,
    output logic [MIW-2:0] mf_wy8, mf_wx8,
    output logic          mf_wvalid,
    output logic [111:0]  mf_wrow, mf_wcol
);
    // Div_Mult[1..31] (av1_tables); index 1 = 16384 is the only one past 14 bits, so the ROM is 15 bits wide
    function automatic logic [14:0] divm(input logic [4:0] d);
        case (d)
            5'd1: divm = 15'd16384; 5'd2: divm = 15'd8192; 5'd3: divm = 15'd5461; 5'd4: divm = 15'd4096; 5'd5: divm = 15'd3276;
            5'd6: divm = 15'd2730; 5'd7: divm = 15'd2340; 5'd8: divm = 15'd2048; 5'd9: divm = 15'd1820; 5'd10: divm = 15'd1638;
            5'd11: divm = 15'd1489; 5'd12: divm = 15'd1365; 5'd13: divm = 15'd1260; 5'd14: divm = 15'd1170; 5'd15: divm = 15'd1092;
            5'd16: divm = 15'd1024; 5'd17: divm = 15'd963; 5'd18: divm = 15'd910; 5'd19: divm = 15'd862; 5'd20: divm = 15'd819;
            5'd21: divm = 15'd780; 5'd22: divm = 15'd744; 5'd23: divm = 15'd712; 5'd24: divm = 15'd682; 5'd25: divm = 15'd655;
            5'd26: divm = 15'd630; 5'd27: divm = 15'd606; 5'd28: divm = 15'd585; 5'd29: divm = 15'd564; 5'd30: divm = 15'd546;
            5'd31: divm = 15'd528; default: divm = 15'd0;
        endcase
    endfunction
    // clip3(-31, 31, num) as a 6-bit signed value
    function automatic logic signed [5:0] clip31(input logic signed [7:0] num);
        clip31 = (num > 8'sd31) ? 6'sd31 : (num < -8'sd31) ? -6'sd31 : 6'(num);
    endfunction
    // round2signed(x, 14) clipped to +-16383: the tail of mv_projection, x = mv * clippedNum * Div_Mult (35 bits)
    function automatic logic signed [15:0] rnd14(input logic signed [34:0] x);
        logic signed [34:0] r;
        r = (x >= 0) ? ((x + 35'sd8192) >>> 14) : -((-x + 35'sd8192) >>> 14);
        rnd14 = (r > 35'sd16383) ? 16'sd16383 : (r < -35'sd16383) ? -16'sd16383 : 16'(r);
    endfunction

    logic [MIW-2:0] w8, h8;
    assign w8 = (MIW-1)'(hdr.mi_cols >> 1);
    assign h8 = (MIW-1)'(hdr.mi_rows >> 1);
    logic signed [5:0] n_dst [7];                            // clip3(-31, 31, refToDst[d]) for d = 1..7
    for (genvar d = 0; d < 7; d++) begin : g_nd
        assign n_dst[d] = clip31(signed'(dst_dist[8 * d +: 8]));
    end

    // ---------------------------------------------------------------- the address side
    localparam int OW = $clog2(OUT);
    localparam int BD = 32;                                  // beat FIFO depth
    localparam int BW = $clog2(BD);
    typedef enum logic [3:0] {A_IDLE, A_CLEAR, A_WAIT, A_SRC, A_LINE, A_LEN, A_AR, A_DRAIN, A_DONE} astate_t;
    astate_t        as;
    logic [2:0]     s;                                       // the source being read
    logic [39:0]    addr;
    logic [15:0]    beats;                                   // beats left in the line
    logic [4:0]     len;
    logic [9:0]     to4k;
    assign to4k = 10'((13'd4096 - 13'(addr[11:0])) >> 3);
    logic [MIW-2:0] line, bx8;                               // the line, the x8 of the next burst's first cell
    logic [MIW-2:0] c_y, c_x;                                // the clear pass
    logic [OW:0]    inflight;                                // bursts issued whose last beat has not arrived
    logic [BW:0]    resv;                                    // beats reserved in the beat FIFO
    assign m_axi_arsize = 3'b011; assign m_axi_arburst = 2'b01; assign m_axi_arcache = 4'b0011; assign m_axi_arprot = 3'b000;
    assign m_axi_araddr = addr; assign m_axi_arlen = 8'(len - 5'd1); assign m_axi_arvalid = (as == A_AR);
    assign m_axi_rready = 1'b1;                              // room is reserved at issue
    assign mf_busy = (as != A_IDLE);
    logic [4:0] burst_len;
    always_comb begin
        burst_len = 5'd16;
        if (beats < 16'd16) burst_len = 5'(beats);
        if (to4k < 10'(burst_len)) burst_len = 5'(to4k);
    end
    // issued bursts, in order: what the data side needs to tag the beats
    typedef struct packed { logic [2:0] src; logic [MIW-2:0] line, x8; } desc_t;
    desc_t          dq [OUT];
    logic [OW:0]    dq_w, dq_r;
    logic           issue, retire, pop_beat;
    assign issue  = (as == A_AR) && m_axi_arready;
    assign retire = m_axi_rvalid && m_axi_rlast;
    logic           can_issue;
    assign can_issue = (inflight < (OW+1)'(OUT)) && ((resv + (BW+1)'(burst_len)) <= (BW+1)'(BD));
    logic           pipe_v;
    logic [BW:0]    bq_n;
    always_ff @(posedge clk) begin
        mf_done <= 1'b0;
        if (rst) begin
            as <= A_IDLE; s <= '0; addr <= '0; beats <= '0; len <= '0; line <= '0; bx8 <= '0; c_y <= '0; c_x <= '0;
            inflight <= '0; resv <= '0; dq_w <= '0;
        end else begin
            inflight <= inflight + (OW+1)'(issue) - (OW+1)'(retire);
            resv <= resv + (issue ? (BW+1)'(len) : '0) - (BW+1)'(pop_beat);
            if (issue) begin dq[dq_w[OW-1:0]] <= '{src: s, line: line, x8: bx8}; dq_w <= dq_w + 1'b1; end
            case (as)
                A_IDLE: if (mf_start) begin c_y <= '0; c_x <= '0; inflight <= '0; resv <= '0; dq_w <= '0; as <= A_CLEAR; end
                A_CLEAR: begin                               // every entry of the field invalid: one position (all references) a cycle
                    if (c_x + 1 < w8) c_x <= c_x + 1;
                    else begin
                        c_x <= '0;
                        if (c_y + 1 < h8) c_y <= c_y + 1;
                        else begin c_y <= '0; as <= A_WAIT; end
                    end
                end
                A_WAIT: if (src_go) begin s <= '0; as <= A_SRC; end
                A_SRC: begin
                    if (s >= mf_nsrc) as <= A_DRAIN;
                    else begin addr <= src_base[40 * s +: 40]; line <= '0; as <= A_LINE; end
                end
                A_LINE: begin
                    beats <= src_rowb[16 * s +: 16] >> 3; bx8 <= '0;
                    if (line < h8 && src_rowb[16 * s +: 16] != 16'd0) as <= A_LEN;
                    else begin s <= s + 3'd1; as <= A_SRC; end
                end
                A_LEN: begin len <= burst_len; if (can_issue) as <= A_AR; end   // arlen settled a cycle before arvalid (the sideband DMA's lesson); a burst goes only with room reserved
                A_AR: if (m_axi_arready) begin
                    addr <= addr + 40'({len, 3'b000}); beats <= beats - 16'(len); bx8 <= bx8 + (MIW-1)'({len, 1'b0});
                    if (beats == 16'(len)) begin line <= line + 1; as <= A_LINE; end
                    else as <= A_LEN;
                end
                A_DRAIN: if (inflight == '0 && bq_n == '0 && !pipe_v) as <= A_DONE;   // the last cells written
                A_DONE: begin mf_done <= 1'b1; as <= A_IDLE; end
                default: as <= A_IDLE;
            endcase
        end
    end

    // ---------------------------------------------------------------- the data side: beats into the FIFO
    typedef struct packed { logic [63:0] w; logic [2:0] src; logic [MIW-2:0] line, x8; } beat_t;
    beat_t          bq [BD];
    logic [BW:0]    bq_w, bq_r;
    logic [3:0]     bcnt;                                    // beat index within the burst
    assign bq_n = bq_w - bq_r;
    always_ff @(posedge clk) begin
        if (rst) begin bq_w <= '0; dq_r <= '0; bcnt <= '0; mf_err <= 1'b0; end
        else begin
            if (mf_start) begin bq_w <= '0; dq_r <= '0; bcnt <= '0; mf_err <= 1'b0; end
            if (m_axi_rvalid) begin
                if (m_axi_rresp[1]) mf_err <= 1'b1;
                bq[bq_w[BW-1:0]] <= '{w: m_axi_rdata, src: dq[dq_r[OW-1:0]].src, line: dq[dq_r[OW-1:0]].line,
                                      x8: dq[dq_r[OW-1:0]].x8 + (MIW-1)'({bcnt, 1'b0})};
                bq_w <= bq_w + 1'b1;
                bcnt <= m_axi_rlast ? 4'd0 : bcnt + 4'd1;
                if (m_axi_rlast) dq_r <= dq_r + 1'b1;
            end
        end
    end

    // ---------------------------------------------------------------- the cell pipeline, one cell a cycle
    // S0: take a cell (half a beat)
    logic           half;
    logic           v0;
    logic [31:0]    c0_w; logic [2:0] c0_src; logic [MIW-2:0] c0_y, c0_x;
    assign pop_beat = (bq_n != '0) && half;
    always_ff @(posedge clk) begin
        if (rst) begin bq_r <= '0; half <= 1'b0; v0 <= 1'b0; end
        else begin
            if (mf_start) begin bq_r <= '0; half <= 1'b0; end
            v0 <= (bq_n != '0);
            if (bq_n != '0) begin
                c0_w   <= half ? bq[bq_r[BW-1:0]].w[63:32] : bq[bq_r[BW-1:0]].w[31:0];
                c0_src <= bq[bq_r[BW-1:0]].src;
                c0_y   <= bq[bq_r[BW-1:0]].line;
                c0_x   <= bq[bq_r[BW-1:0]].x8 + (MIW-1)'(half);
                half   <= ~half;
                if (half) bq_r <= bq_r + 1'b1;
            end
        end
    end
    // S1: the source's constants, posValid
    logic           v1;
    logic signed [13:0] c1_mvr, c1_mvc; logic [MIW-2:0] c1_y, c1_x;
    logic signed [5:0]  c1_ncur; logic [14:0] c1_dm; logic c1_sign;
    logic [2:0]     c0_ref; logic signed [7:0] c0_off, c0_tocur; logic c0_sign;
    assign c0_ref   = c0_w[2:0];
    logic [5:0]     c0_oi;                                   // 7 src + ref - 1 (0 for ref 0: the value is unused)
    assign c0_oi    = (c0_ref == 3'd0) ? 6'd0 : 6'(7 * 32'(c0_src) + 32'(c0_ref) - 1);
    assign c0_off   = signed'(src_off[8 * c0_oi +: 8]);
    assign c0_tocur = signed'(src_tocur[8 * c0_src +: 8]);
    assign c0_sign  = src_sign[c0_src];
    always_ff @(posedge clk) begin
        if (rst) v1 <= 1'b0;
        else v1 <= v0 && c0_ref != 3'd0 && c0_x < w8 && c0_off > 8'sd0 && c0_off <= 8'sd31 && c0_tocur <= 8'sd31 && c0_tocur >= -8'sd31;
        c1_mvc  <= 14'(signed'(c0_w[15:3]));
        c1_mvr  <= 14'(signed'(c0_w[28:16]));
        c1_y    <= c0_y; c1_x <= c0_x;
        c1_ncur <= c0_sign ? 6'(c0_tocur) : -6'(c0_tocur);
        c1_dm   <= divm(5'(c0_off));
        c1_sign <= c0_sign;
    end
    // S2: K = clip(num) * Div_Mult[den] toward the current frame and each destination (signed 21 bits)
    logic           v2;
    logic signed [13:0] c2_mvr, c2_mvc; logic [MIW-2:0] c2_y, c2_x; logic c2_sign;
    logic signed [20:0] k2_cur, k2_dst [7];
    always_ff @(posedge clk) begin
        if (rst) v2 <= 1'b0; else v2 <= v1;
        c2_mvr <= c1_mvr; c2_mvc <= c1_mvc; c2_y <= c1_y; c2_x <= c1_x; c2_sign <= c1_sign;
        k2_cur <= 21'(c1_ncur) * 21'(signed'({6'd0, c1_dm}));
        for (int d = 0; d < 7; d++) k2_dst[d] <= 21'(n_dst[d]) * 21'(signed'({6'd0, c1_dm}));
    end
    // S3: the sixteen products mv * K (signed 35 bits)
    logic           v3;
    logic [MIW-2:0] c3_y, c3_x; logic c3_sign;
    logic signed [34:0] x3_r, x3_c, x3_dr [7], x3_dc [7];
    always_ff @(posedge clk) begin
        if (rst) v3 <= 1'b0; else v3 <= v2;
        c3_y <= c2_y; c3_x <= c2_x; c3_sign <= c2_sign;
        x3_r <= 35'(c2_mvr) * 35'(k2_cur);
        x3_c <= 35'(c2_mvc) * 35'(k2_cur);
        for (int d = 0; d < 7; d++) begin
            x3_dr[d] <= 35'(c2_mvr) * 35'(k2_dst[d]);
            x3_dc[d] <= 35'(c2_mvc) * 35'(k2_dst[d]);
        end
    end
    // S4: round2signed(., 14) and the clip
    logic           v4;
    logic [MIW-2:0] c4_y, c4_x; logic c4_sign;
    logic signed [15:0] p4_r, p4_c, p4_dr [7], p4_dc [7];
    always_ff @(posedge clk) begin
        if (rst) v4 <= 1'b0; else v4 <= v3;
        c4_y <= c3_y; c4_x <= c3_x; c4_sign <= c3_sign;
        p4_r <= rnd14(x3_r); p4_c <= rnd14(x3_c);
        for (int d = 0; d < 7; d++) begin p4_dr[d] <= rnd14(x3_dr[d]); p4_dc[d] <= rnd14(x3_dc[d]); end
    end
    // S5: block_position for one axis: v8 + dstSign * offset8, within the 8-unit block (+- margin) and the frame
    function automatic logic [MIW:0] bpos(input logic [MIW-2:0] v8, input logic signed [15:0] delta, input logic sign, input logic [MIW-2:0] max8, input int maxoff8);
        logic signed [MIW:0] v, base8, off8;
        off8 = (delta >= 0) ? (MIW+1)'(delta >>> 6) : -(MIW+1)'((-delta) >>> 6);
        base8 = (MIW+1)'({v8[MIW-2:3], 3'd0});
        v = (MIW+1)'(v8) + (sign ? off8 : -off8);
        bpos = {1'b0, v[MIW-1:0]};
        if (v < 0 || v >= (MIW+1)'(max8) || v < base8 - (MIW+1)'(maxoff8) || v >= base8 + (MIW+1)'(8 + maxoff8)) bpos[MIW] = 1'b1;   // invalid
        return bpos;
    endfunction
    logic [MIW:0] by, bx;
    always_comb begin
        by = bpos(c4_y, p4_r, c4_sign, h8, 0);
        bx = bpos(c4_x, p4_c, c4_sign, w8, 8);
    end
    logic           v5;
    logic [MIW-2:0] p5_y, p5_x;
    logic signed [15:0] p5_dr [7], p5_dc [7];
    always_ff @(posedge clk) begin
        if (rst) v5 <= 1'b0; else v5 <= v4 && !by[MIW] && !bx[MIW];
        p5_y <= by[MIW-2:0]; p5_x <= bx[MIW-2:0];
        for (int d = 0; d < 7; d++) begin p5_dr[d] <= p4_dr[d]; p5_dc[d] <= p4_dc[d]; end
    end
    assign pipe_v = v0 | v1 | v2 | v3 | v4 | v5;
    // S6: the write (the clear pass uses the same register from the address side's counters)
    always_ff @(posedge clk) begin
        mf_we <= 7'd0;
        if (rst) begin mf_wy8 <= '0; mf_wx8 <= '0; mf_wvalid <= 1'b0; mf_wrow <= '0; mf_wcol <= '0; end
        else if (as == A_CLEAR) begin mf_we <= 7'h7f; mf_wy8 <= c_y; mf_wx8 <= c_x; mf_wvalid <= 1'b0; mf_wrow <= '0; mf_wcol <= '0; end
        else if (v5) begin
            mf_we <= 7'h7f; mf_wy8 <= p5_y; mf_wx8 <= p5_x; mf_wvalid <= 1'b1;
            for (int d = 0; d < 7; d++) begin mf_wrow[16 * d +: 16] <= p5_dr[d]; mf_wcol[16 * d +: 16] <= p5_dc[d]; end
        end
    end
endmodule
