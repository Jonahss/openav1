// Super-resolution upscaling (7.16): each row of each plane is resampled horizontally from FrameWidth to
// UpscaledWidth with the 8-tap Upscale_Filter, in place: the row is first copied into a line buffer, then
// the upscaled row is written back over it (the buffer row is at least UpscaledWidth wide). Runs once per
// input picture of loop restoration (the deblocked frame and the CDEF frame); the caller selects the buffer.
// Spec-literal arithmetic: stepX / initialSubpelX use truncating divisions (one 32-bit divider, evaluated
// once per plane; fine for simulation, to be replaced by a sequential divider for synthesis).
// The output sample is a 4-stage pipeline (2026-10-26): source position -> the 8 clamped line-buffer indices and
// the subpel; the 8 samples and taps; the 8 products; the sum, Round2 and clamp into the write port. Combinational
// from src_x into the frame memory's write capture it was the decoder's worst path at 100 MHz routed (-1.43 ns, 33
// logic levels: the clamps, the line-buffer reads, the tap ROM, 8 DSPs, the adder tree). Writes follow S_OUT by 4
// cycles; S_DRAIN empties the pipeline before done. The line buffer is read in stage 1 only (the cycle after
// S_OUT at the latest), before the next row's S_RD overwrites it.
module sr_top
  import syn_pkg::*;
  import blk_tables_pkg::*;
#(
    parameter int FBX = 10,
    parameter int FBY = 9
) (
    input  logic        clk,
    input  logic        rst,
    input  hdr_t        hdr,
    input  lf_hdr_t     lh,                   // frame_width (FrameWidth before upscaling)
    input  logic        start,
    input  logic [PXW-1:0] ly_first, ly_last,    // luma rows to upscale (inclusive); chroma rows follow by subsampling
    output logic        busy,
    output logic        done,
    // the buffer being upscaled (same plane for reads and writes)
    output logic        s_re,
    output logic [1:0]  plane,
    output logic [FBX-1:0] s_x,
    output logic [FBY-1:0] s_y,
    input  logic [11:0] s_rdata,
    output logic        d_we,
    output logic [1:0]  d_plane,              // the write's plane: the pipeline's last samples of a plane's last row leave
                                              // after `plane` has moved on (2026-10-26: they landed in the next plane)
    output logic [FBX-1:0] d_x,
    output logic [FBY-1:0] d_y,
    output logic [11:0] d_wdata
);
    localparam int SUPERRES_SCALE_BITS = 14;
    localparam int SUPERRES_EXTRA_BITS = 8;
    localparam int FILTER_BITS = 7;

    typedef enum logic [3:0] {S_IDLE, S_PLANE, S_DIV, S_INIT1, S_INIT2, S_RD, S_RD_LAST, S_OUT, S_NEXT, S_DRAIN, S_DONE} st_t;
    st_t st;
    logic        sub_x, sub_y;
    logic [PXW-1:0] up_w, plane_h, max_x;
    logic [31:0] step_x;
    logic signed [31:0] init_subpel, src_x;
    logic [PXW-1:0] x, y;                                       // read column / row
    logic [PXW-1:0] xo;                                         // output column
    logic [11:0] line [0:(1 << FBX) - 1];
    logic [11:0] pix_max;
    assign pix_max = 12'((13'd1 << hdr.bit_depth) - 13'd1);

    // per-plane geometry (7.16)
    logic [PXW-1:0] g_down, g_up, g_h, g_maxx;
    logic signed [31:0] g_num;
    always_comb begin
        sub_x = (plane != 2'd0) && hdr.ssx;
        sub_y = (plane != 2'd0) && hdr.ssy;
        g_down = (lh.frame_width + PXW'(sub_x)) >> sub_x;          // Round2(FrameWidth, subX)
        g_up = (hdr.upscaled_width + PXW'(sub_x)) >> sub_x;
        g_h = (lh.frame_height + PXW'(sub_y)) >> sub_y;
        g_maxx = PXW'(((PXW'(hdr.mi_cols) >> sub_x) << 2) - PXW'(1));
        g_num = -$signed((32'(g_up) - 32'(g_down)) << (SUPERRES_SCALE_BITS - 1)) + $signed(32'(g_up >> 1));
    end
    // stepX = (down << 14 + up/2) / up and the truncating q = num / up (num possibly negative) are two 32-bit
    // divisions, once per plane: a shared restoring divider, one quotient bit per cycle (S_DIV, 32 cycles each).
    // Combinational, the two dividers into init_subpel were the decoder's worst path (-19.5 ns at 100 MHz).
    logic [PXW-1:0] dv_down, dv_up;
    logic [31:0] dv_num, dv_quo, dv_num2;
    logic [32:0] dv_rem;
    logic [5:0]  dv_cnt;
    logic        dv_phase, dv_neg2;          // phase 0: stepX; phase 1: |num| / up, sign dv_neg2
    logic [31:0] dv_q2;
    logic [32:0] dv_rem_sh;
    logic        dv_ge;
    always_comb begin
        dv_rem_sh = {dv_rem[31:0], dv_num[31]};
        dv_ge = dv_rem_sh >= 33'(dv_up);
    end
    logic signed [31:0] sr_err;              // up * stepX - (down << 14)

    // output sample: 8 taps around srcXPx from the line buffer, as a 4-stage pipeline behind S_OUT
    logic signed [31:0] src_px;
    logic [5:0]  subpel;
    logic [FBX-1:0] sxc [8];
    always_comb begin
        src_px = src_x >>> SUPERRES_SCALE_BITS;
        subpel = 6'((src_x & 32'sd16383) >> SUPERRES_EXTRA_BITS);
        for (int k = 0; k < 8; k++) begin
            logic signed [31:0] sx_;
            logic [PXW-1:0] sxp;
            sx_ = src_px + 32'(k) - 32'sd3;
            sxp = (sx_ < 0) ? PXW'(0) : (sx_ > 32'(max_x)) ? max_x : PXW'(sx_);
            sxc[k] = sxp[FBX-1:0];
        end
    end
    // stage 1: indices + subpel   2: samples + taps   3: products   4: sum / round / clamp
    logic           p1_v, p2_v, p3_v, p4_v;
    logic [1:0]     p1_pl, p2_pl, p3_pl, p4_pl;
    logic [FBX-1:0] p1_x, p2_x, p3_x, p4_x;
    logic [FBY-1:0] p1_y, p2_y, p3_y, p4_y;
    logic [FBX-1:0] p1_idx [8];
    logic [5:0]     p1_subpel;
    logic [11:0]    p2_s [8];
    logic signed [8:0] p2_tap [8];
    logic signed [21:0] p3_prod [8];
    logic [11:0]    p4_pix;
    logic signed [21:0] acc;
    logic signed [14:0] o_val;
    logic [11:0] o_pix;
    always_comb begin
        acc = 22'sd0;
        for (int k = 0; k < 8; k++) acc = acc + p3_prod[k];
        o_val = 15'((acc + 22'sd64) >>> FILTER_BITS);           // Round2(sum, FILTER_BITS)
        o_pix = (o_val < 0) ? 12'd0 : (o_val > 15'(signed'({3'b0, pix_max}))) ? pix_max : 12'(o_val);
    end
    always_ff @(posedge clk) begin
        p1_v <= (st == S_OUT) && !rst; p1_x <= FBX'(xo); p1_y <= FBY'(y); p1_pl <= plane; p1_idx <= sxc; p1_subpel <= subpel;
        p2_v <= p1_v && !rst; p2_x <= p1_x; p2_y <= p1_y; p2_pl <= p1_pl;
        for (int k = 0; k < 8; k++) begin p2_s[k] <= line[p1_idx[k]]; p2_tap[k] <= upscale_filter(p1_subpel, 3'(k)); end
        p3_v <= p2_v && !rst; p3_x <= p2_x; p3_y <= p2_y; p3_pl <= p2_pl;
        for (int k = 0; k < 8; k++) p3_prod[k] <= 22'(signed'({1'b0, p2_s[k]})) * 22'(p2_tap[k]);
        p4_v <= p3_v && !rst; p4_x <= p3_x; p4_y <= p3_y; p4_pl <= p3_pl; p4_pix <= o_pix;
    end

    assign busy = (st != S_IDLE);
    assign s_re = (st == S_RD);
    assign s_x = FBX'(x); assign s_y = FBY'(y);
    assign d_we = p4_v;
    assign d_plane = p4_pl; assign d_x = p4_x; assign d_y = p4_y; assign d_wdata = p4_pix;

    always_ff @(posedge clk) begin
        done <= 1'b0;
        if (rst) st <= S_IDLE;
        else case (st)
            S_IDLE: if (start) begin plane <= 2'd0; st <= S_PLANE; end
            S_PLANE: begin
                up_w <= g_up; plane_h <= g_h; max_x <= g_maxx;
                y <= ly_first >> sub_y; x <= PXW'(0);
                dv_down <= g_down; dv_up <= g_up;
                dv_num <= (32'(g_down) << SUPERRES_SCALE_BITS) + 32'(g_up >> 1);
                dv_num2 <= (g_num < 0) ? 32'(-g_num) : 32'(g_num); dv_neg2 <= (g_num < 0);
                dv_rem <= '0; dv_quo <= '0; dv_cnt <= 6'd0; dv_phase <= 1'b0;
                st <= S_DIV;
            end
            S_DIV: begin                                        // one quotient bit per cycle, MSB first
                dv_rem <= dv_ge ? dv_rem_sh - 33'(dv_up) : dv_rem_sh;
                dv_num <= {dv_num[30:0], 1'b0};
                dv_quo <= {dv_quo[30:0], dv_ge};
                if (dv_cnt != 6'd31) dv_cnt <= dv_cnt + 6'd1;
                else begin
                    dv_cnt <= 6'd0; dv_rem <= '0; dv_quo <= '0;
                    if (!dv_phase) begin step_x <= {dv_quo[30:0], dv_ge}; dv_num <= dv_num2; dv_phase <= 1'b1; end
                    else begin dv_q2 <= {dv_quo[30:0], dv_ge}; st <= S_INIT1; end
                end
            end
            S_INIT1: begin                                      // err = up * stepX - (down << 14)
                sr_err <= $signed(32'(dv_up) * step_x) - $signed(32'(dv_down) << SUPERRES_SCALE_BITS);
                st <= S_INIT2;
            end
            S_INIT2: begin                                      // initialSubpelX = (q + 128 - err/2) & 16383, C truncation
                init_subpel <= ((dv_neg2 ? -$signed(dv_q2) : $signed(dv_q2)) + 32'sd128
                                - ((sr_err < 0) ? -$signed(32'(-sr_err) >> 1) : $signed(32'(sr_err) >> 1))) & 32'sd16383;
                st <= S_RD;
            end
            // copy row y into the line buffer (the read lands one cycle after its address): all MiCols * 4
            // decoded samples, not just FrameWidth of them -- the filter clamps to maxX = miW * MI_SIZE - 1 and
            // reads the decoded margin beyond the picture edge
            S_RD: begin
                if (x != PXW'(0)) line[x[FBX-1:0] - FBX'(1)] <= s_rdata;
                if (x < max_x) x <= x + PXW'(1);
                else st <= S_RD_LAST;
            end
            S_RD_LAST: begin
                line[x[FBX-1:0]] <= s_rdata;
                xo <= PXW'(0); src_x <= -(32'sd1 <<< SUPERRES_SCALE_BITS) + init_subpel;
                st <= S_OUT;
            end
            // one upscaled sample per cycle: srcX = -(1 << 14) + initialSubpelX + x * stepX
            S_OUT: begin
                src_x <= src_x + $signed(step_x);
                if (xo + PXW'(1) < up_w) xo <= xo + PXW'(1);
                else st <= S_NEXT;
            end
            S_NEXT: begin
                x <= PXW'(0);
                if (y + PXW'(1) < plane_h && y + PXW'(1) <= (ly_last >> sub_y)) begin y <= y + PXW'(1); st <= S_RD; end
                else if (plane + 2'd1 < (hdr.mono ? 2'd1 : 2'd3)) begin plane <= plane + 2'd1; st <= S_PLANE; end
                else st <= S_DRAIN;
            end
            S_DRAIN: if (!p1_v && !p2_v && !p3_v && !p4_v) st <= S_DONE;   // the last writes are out
            S_DONE: begin done <= 1'b1; st <= S_IDLE; end
            default: st <= S_IDLE;
        endcase
    end
endmodule
