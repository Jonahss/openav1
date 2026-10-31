// Motion field projection engine (spec 7.9 motion_field_estimation; docs/frame-state.md stage B, the last block):
// at a frame start, after the loader, the card fills the motion-field band itself from the reference slots' saved MV
// words (rtl/frame_state_w.sv's sideband, one {ref, mv} per 8x8 unit) instead of the host writing every entry.
// The host supplies what the spec derives from the order hints alone: which sources project and in which order
// (LAST, BWDREF, ALTREF2, ALTREF, LAST2 with the refStamp rule), each source's dstSign and refToCur, its refOffset for
// each saved reference 1..7, and refToDst for each destination 1..7. The engine does 7.9.2 per cell:
//   posValid: refOffset > 0 and |refToCur|, |refOffset| <= MAX_FRAME_DISTANCE (31);
//   projMv = mv_projection(mv, refToCur * dstSign, refOffset); (PosY8, PosX8) by block_position (rows stay in the
//   64-pixel band, columns within 8 units); then for dst = 1..7: mf[dst][PosY8][PosX8] = mv_projection(mv, refToDst, refOffset)
// mv_projection(mv, num, den): round2signed(mv * clip3(-31, 31, num) * Div_Mult[min(31, den)], 14) clipped to +-16383.
// Cells go in raster order per source, sources in the host's order, so a later write wins at a shared target, as the
// model does. The field is cleared (every entry invalid) first. One AXI read master; bursts of up to 16 beats.
module mf_project
  import syn_pkg::*;
(
    input  logic          clk,
    input  logic          rst,
    input  hdr_t          hdr,
    input  logic          mf_start,
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
    // the band (mf_band: ref 0 = LAST_FRAME)
    output logic          mf_we,
    output logic [2:0]    mf_wref,
    output logic [MIW-2:0] mf_wy8, mf_wx8,
    output logic          mf_wvalid,
    output logic signed [15:0] mf_wrow, mf_wcol
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
    // mv_projection for one component: mv (signed 14: |mv| <= 4095 from the sideband), num signed 8 (clipped to +-31 here),
    // den 5 bits (1..31): round2signed(mv * num * Div_Mult[den], 14) clipped to +-16383
    function automatic logic signed [15:0] mvproj(input logic signed [13:0] mv, input logic signed [7:0] num, input logic [4:0] den);
        logic signed [7:0] n; logic signed [35:0] x, r;
        n = (num > 8'sd31) ? 8'sd31 : (num < -8'sd31) ? -8'sd31 : num;
        x = 36'(mv) * 36'(n) * 36'(signed'({21'd0, divm(den)}));
        r = (x >= 0) ? ((x + 36'sd8192) >>> 14) : -((-x + 36'sd8192) >>> 14);
        mvproj = (r > 36'sd16383) ? 16'sd16383 : (r < -36'sd16383) ? -16'sd16383 : 16'(r);
    endfunction

    logic [MIW-2:0] w8, h8;
    assign w8 = (MIW-1)'(hdr.mi_cols >> 1);
    assign h8 = (MIW-1)'(hdr.mi_rows >> 1);

    // ---------------------------------------------------------------- the read side: cells into a small queue
    typedef enum logic [3:0] {P_IDLE, P_CLEAR, P_SRC, P_LINE, P_LEN, P_AR, P_R, P_NEXT, P_DRAIN, P_DONE} pstate_t;
    pstate_t        ps;
    logic [2:0]     s;                                       // the source being read
    logic [39:0]    addr;
    logic [31:0]    beats;
    logic [4:0]     len, cnt;
    logic [9:0]     to4k;
    assign to4k = 10'((13'd4096 - 13'(addr[11:0])) >> 3);
    logic [MIW-2:0] line, beat;
    logic [2:0]     c_dst; logic [MIW-2:0] c_y, c_x;        // the clear pass
    assign m_axi_arsize = 3'b011; assign m_axi_arburst = 2'b01; assign m_axi_arcache = 4'b0011; assign m_axi_arprot = 3'b000;
    assign m_axi_araddr = addr; assign m_axi_arlen = 8'(len - 5'd1); assign m_axi_arvalid = (ps == P_AR);
    assign mf_busy = (ps != P_IDLE);
    logic [4:0] burst_len;
    always_comb begin
        burst_len = 5'd16;
        if (beats < 32'd16) burst_len = 5'(beats);
        if (to4k < 10'(burst_len)) burst_len = 5'(to4k);
    end
    // the cell queue: {word[31:0], y8, x8, src} x 4; a beat pushes two cells
    typedef struct packed { logic [31:0] w; logic [MIW-2:0] y8, x8; logic [2:0] src; } cell_t;
    cell_t          q [4];
    logic [2:0]     q_w, q_r;
    logic [2:0]     q_n;
    assign q_n = q_w - q_r;
    assign m_axi_rready = (ps == P_R) && (q_n <= 3'd2);
    logic           pop;
    always_ff @(posedge clk) begin
        if (rst) begin q_w <= '0; end
        else begin
            if (mf_start) q_w <= '0;
            if (ps == P_R && m_axi_rvalid && m_axi_rready) begin
                q[q_w[1:0]] <= '{w: m_axi_rdata[31:0], y8: line, x8: (MIW-1)'({beat[MIW-3:0], 1'b0}), src: s};
                q[2'(q_w + 3'd1)] <= '{w: m_axi_rdata[63:32], y8: line, x8: (MIW-1)'({beat[MIW-3:0], 1'b1}), src: s};
                q_w <= q_w + 3'd2;
            end
        end
    end
    always_ff @(posedge clk) begin
        mf_done <= 1'b0;
        if (rst) begin
            ps <= P_IDLE; s <= '0; addr <= '0; beats <= '0; len <= '0; cnt <= '0; line <= '0; beat <= '0; mf_err <= 1'b0;
            c_dst <= '0; c_y <= '0; c_x <= '0;
        end else begin
            case (ps)
                P_IDLE: if (mf_start) begin mf_err <= 1'b0; c_dst <= '0; c_y <= '0; c_x <= '0; ps <= P_CLEAR; end
                P_CLEAR: begin                               // every entry of the field invalid, one a cycle
                    if (c_x + 1 < w8) c_x <= c_x + 1;
                    else begin
                        c_x <= '0;
                        if (c_y + 1 < h8) c_y <= c_y + 1;
                        else begin c_y <= '0; if (c_dst != 3'd6) c_dst <= c_dst + 3'd1; else begin s <= '0; ps <= P_SRC; end end
                    end
                end
                P_SRC: begin
                    if (s >= mf_nsrc) ps <= P_DRAIN;
                    else begin addr <= src_base[40 * s +: 40]; line <= '0; ps <= P_LINE; end
                end
                P_LINE: begin
                    beats <= 32'(src_rowb[16 * s +: 16] >> 3); beat <= '0;
                    if (line < h8 && src_rowb[16 * s +: 16] != 16'd0) ps <= P_LEN;
                    else begin s <= s + 3'd1; ps <= P_SRC; end
                end
                P_LEN: begin len <= burst_len; cnt <= '0; ps <= P_AR; end
                P_AR: if (m_axi_arready) ps <= P_R;
                P_R: if (m_axi_rvalid && m_axi_rready) begin
                    if (m_axi_rresp[1]) mf_err <= 1'b1;
                    addr <= addr + 40'd8; beats <= beats - 32'd1; cnt <= cnt + 5'd1; beat <= beat + 1;
                    if (m_axi_rlast) ps <= P_NEXT;
                end
                P_NEXT: begin
                    if (beats != 32'd0) ps <= P_LEN;
                    else begin line <= line + 1; ps <= P_LINE; end
                end
                P_DRAIN: if (q_n == 3'd0 && cs == C_IDLE) ps <= P_DONE;   // the last cells written
                P_DONE: begin mf_done <= 1'b1; ps <= P_IDLE; end
                default: ps <= P_IDLE;
            endcase
        end
    end

    // ---------------------------------------------------------------- the cell side: project and write
    typedef enum logic [2:0] {C_IDLE, C_POS, C_DST} cstate_t;
    cstate_t        cs;
    cell_t          c;
    logic [2:0]     c_ref;
    logic signed [13:0] c_mvr, c_mvc;
    logic signed [7:0]  c_off, c_tocur;
    logic           c_sign;
    logic signed [15:0] pr, pc;                              // projMv
    logic [MIW-2:0] py, px;
    logic [2:0]     d;
    assign c_ref = c.w[2:0];
    assign c_mvc = 14'(signed'(c.w[15:3]));
    assign c_mvr = 14'(signed'(c.w[28:16]));
    assign pop = (cs == C_IDLE) && (q_n != 3'd0) && (ps != P_CLEAR);
    // block_position for one axis: v8 + dstSign * offset8, within the 8-unit block (+- margin) and the frame
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
        by = bpos(c.y8, pr, c_sign, h8, 0);
        bx = bpos(c.x8, pc, c_sign, w8, 8);
    end
    always_ff @(posedge clk) begin
        mf_we <= 1'b0;
        if (rst) begin cs <= C_IDLE; q_r <= '0; c <= '0; d <= '0; pr <= '0; pc <= '0; py <= '0; px <= '0; c_off <= '0; c_tocur <= '0; c_sign <= 1'b0;
                       mf_wref <= '0; mf_wy8 <= '0; mf_wx8 <= '0; mf_wvalid <= 1'b0; mf_wrow <= '0; mf_wcol <= '0; end
        else begin
            if (mf_start) q_r <= '0;
            // the clear pass writes from the read-side FSM's counters
            if (ps == P_CLEAR) begin mf_we <= 1'b1; mf_wref <= c_dst; mf_wy8 <= c_y; mf_wx8 <= c_x; mf_wvalid <= 1'b0; mf_wrow <= '0; mf_wcol <= '0; end
            case (cs)
                C_IDLE: if (pop) begin
                    c <= q[q_r[1:0]]; q_r <= q_r + 3'd1;
                    cs <= C_POS;
                end
                C_POS: begin                                 // posValid, projMv, then the position
                    c_off   <= signed'(src_off[8 * (7 * c.src + c_ref - 1) +: 8]);
                    c_tocur <= signed'(src_tocur[8 * c.src +: 8]);
                    c_sign  <= src_sign[c.src];
                    if (c_ref == 3'd0 || c.x8 >= w8) cs <= C_IDLE;                  // no saved reference / the padding cell
                    else begin
                        // computed from the registered inputs next state: hold one cycle via d as a phase counter
                        d <= 3'd0; cs <= C_DST;
                    end
                end
                C_DST: begin
                    if (d == 3'd0) begin                     // phase 0: the test and the projection toward the current frame
                        if (c_off <= 8'sd0 || c_off > 8'sd31 || c_tocur > 8'sd31 || c_tocur < -8'sd31) cs <= C_IDLE;
                        else begin
                            pr <= mvproj(c_mvr, c_sign ? c_tocur : -c_tocur, 5'(c_off));
                            pc <= mvproj(c_mvc, c_sign ? c_tocur : -c_tocur, 5'(c_off));
                            d <= 3'd1;
                        end
                    end else if (d == 3'd1) begin            // phase 1: block_position (by / bx from pr / pc, combinational)
                        if (by[MIW] || bx[MIW]) cs <= C_IDLE;
                        else begin py <= by[MIW-2:0]; px <= bx[MIW-2:0]; d <= 3'd2; end
                    end else begin                           // phases 2..8: the seven destinations
                        mf_we <= 1'b1; mf_wref <= d - 3'd2; mf_wy8 <= py; mf_wx8 <= px; mf_wvalid <= 1'b1;
                        mf_wrow <= mvproj(c_mvr, signed'(dst_dist[8 * (d - 2) +: 8]), 5'(c_off));
                        mf_wcol <= mvproj(c_mvc, signed'(dst_dist[8 * (d - 2) +: 8]), 5'(c_off));
                        if (d == 3'd7 + 3'd1) cs <= C_IDLE; else d <= d + 3'd1;
                    end
                end
                default: cs <= C_IDLE;
            endcase
        end
    end
endmodule
