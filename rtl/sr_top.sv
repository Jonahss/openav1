// Super-resolution upscaling (7.16): each row of each plane is resampled horizontally from FrameWidth to
// UpscaledWidth with the 8-tap Upscale_Filter, in place: the row is first copied into a line buffer, then
// the upscaled row is written back over it (the buffer row is at least UpscaledWidth wide). Runs once per
// input picture of loop restoration (the deblocked frame and the CDEF frame); the caller selects the buffer.
// Spec-literal arithmetic: stepX / initialSubpelX use truncating divisions (one 32-bit divider, evaluated
// once per plane; fine for simulation, to be replaced by a sequential divider for synthesis).
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
    output logic        busy,
    output logic        done,
    // the buffer being upscaled (same plane for reads and writes)
    output logic        s_re,
    output logic [1:0]  plane,
    output logic [FBX-1:0] s_x,
    output logic [FBY-1:0] s_y,
    input  logic [11:0] s_rdata,
    output logic        d_we,
    output logic [FBX-1:0] d_x,
    output logic [FBY-1:0] d_y,
    output logic [11:0] d_wdata
);
    localparam int SUPERRES_SCALE_BITS = 14;
    localparam int SUPERRES_EXTRA_BITS = 8;
    localparam int FILTER_BITS = 7;

    typedef enum logic [2:0] {S_IDLE, S_PLANE, S_RD, S_RD_LAST, S_OUT, S_NEXT, S_DONE} st_t;
    st_t st;
    logic        sub_x, sub_y;
    logic [12:0] up_w, plane_h, max_x;
    logic [31:0] step_x;
    logic signed [31:0] init_subpel, src_x;
    logic [12:0] x, y;                                       // read column / row
    logic [12:0] xo;                                         // output column
    logic [11:0] line [0:(1 << FBX) - 1];
    logic [11:0] pix_max;
    assign pix_max = 12'((13'd1 << hdr.bit_depth) - 13'd1);

    // per-plane geometry (7.16)
    logic [12:0] g_down, g_up, g_h, g_maxx;
    logic [31:0] g_step;
    logic signed [31:0] g_err, g_num, g_q, g_init;
    always_comb begin
        sub_x = (plane != 2'd0) && hdr.ssx;
        sub_y = (plane != 2'd0) && hdr.ssy;
        g_down = (lh.frame_width + 13'(sub_x)) >> sub_x;          // Round2(FrameWidth, subX)
        g_up = (hdr.upscaled_width + 13'(sub_x)) >> sub_x;
        g_h = (lh.frame_height + 13'(sub_y)) >> sub_y;
        g_maxx = 13'(((13'(hdr.mi_cols) >> sub_x) << 2) - 13'd1);
        g_step = ((32'(g_down) << SUPERRES_SCALE_BITS) + 32'(g_up >> 1)) / 32'(g_up);
        g_err = $signed(32'(g_up) * g_step) - $signed(32'(g_down) << SUPERRES_SCALE_BITS);
        g_num = -$signed((32'(g_up) - 32'(g_down)) << (SUPERRES_SCALE_BITS - 1)) + $signed(32'(g_up >> 1));
        // truncating (C) divisions of possibly negative numerators
        g_q = (g_num < 0) ? -$signed((32'(-g_num)) / 32'(g_up)) : $signed(32'(g_num) / 32'(g_up));
        g_init = g_q + 32'sd128 - ((g_err < 0) ? -$signed(32'(-g_err) >> 1) : $signed(32'(g_err) >> 1));
    end

    // output sample: 8 taps around srcXPx from the line buffer
    logic signed [31:0] src_px;
    logic [5:0]  subpel;
    logic signed [21:0] acc;
    logic signed [14:0] o_val;
    logic [11:0] o_pix;
    always_comb begin
        src_px = src_x >>> SUPERRES_SCALE_BITS;
        subpel = 6'((src_x & 32'sd16383) >> SUPERRES_EXTRA_BITS);
        acc = 22'sd0;
        for (int k = 0; k < 8; k++) begin
            logic signed [31:0] sx_;
            logic [12:0] sxc;
            sx_ = src_px + 32'(k) - 32'sd3;
            sxc = (sx_ < 0) ? 13'd0 : (sx_ > 32'(max_x)) ? max_x : 13'(sx_);
            acc = acc + 22'(signed'({1'b0, line[sxc[FBX-1:0]]})) * 22'(upscale_filter(subpel, 3'(k)));
        end
        o_val = 15'((acc + 22'sd64) >>> FILTER_BITS);           // Round2(sum, FILTER_BITS)
        o_pix = (o_val < 0) ? 12'd0 : (o_val > 15'(signed'({3'b0, pix_max}))) ? pix_max : 12'(o_val);
    end

    assign busy = (st != S_IDLE);
    assign s_re = (st == S_RD);
    assign s_x = FBX'(x); assign s_y = FBY'(y);
    assign d_we = (st == S_OUT);
    assign d_x = FBX'(xo); assign d_y = FBY'(y); assign d_wdata = o_pix;

    always_ff @(posedge clk) begin
        done <= 1'b0;
        if (rst) st <= S_IDLE;
        else case (st)
            S_IDLE: if (start) begin plane <= 2'd0; st <= S_PLANE; end
            S_PLANE: begin
                up_w <= g_up; plane_h <= g_h; max_x <= g_maxx; step_x <= g_step;
                init_subpel <= g_init & 32'sd16383;
                y <= 13'd0; x <= 13'd0;
                st <= S_RD;
            end
            // copy row y into the line buffer (the read lands one cycle after its address): all MiCols * 4
            // decoded samples, not just FrameWidth of them -- the filter clamps to maxX = miW * MI_SIZE - 1 and
            // reads the decoded margin beyond the picture edge
            S_RD: begin
                if (x != 13'd0) line[x[FBX-1:0] - FBX'(1)] <= s_rdata;
                if (x < max_x) x <= x + 13'd1;
                else st <= S_RD_LAST;
            end
            S_RD_LAST: begin
                line[x[FBX-1:0]] <= s_rdata;
                xo <= 13'd0; src_x <= -(32'sd1 <<< SUPERRES_SCALE_BITS) + init_subpel;
                st <= S_OUT;
            end
            // one upscaled sample per cycle: srcX = -(1 << 14) + initialSubpelX + x * stepX
            S_OUT: begin
                src_x <= src_x + $signed(step_x);
                if (xo + 13'd1 < up_w) xo <= xo + 13'd1;
                else st <= S_NEXT;
            end
            S_NEXT: begin
                x <= 13'd0;
                if (y + 13'd1 < plane_h) begin y <= y + 13'd1; st <= S_RD; end
                else if (plane + 2'd1 < (hdr.mono ? 2'd1 : 2'd3)) begin plane <= plane + 2'd1; st <= S_PLANE; end
                else st <= S_DONE;
            end
            S_DONE: begin done <= 1'b1; st <= S_IDLE; end
            default: st <= S_IDLE;
        endcase
    end
endmodule
