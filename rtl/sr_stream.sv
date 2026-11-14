// Super-resolution (7.16) for the streaming filter chain (docs/filter-chain.md), one plane: the horizontal 8-tap
// upscale of line segments on their way from CDEF (or from the deblocked boundary lines) to loop restoration.
// In: line segments as cdef_stream sends them: segment column by segment column (8 samples, or 4 in a horizontally
// subsampled plane), each column's lines top to bottom; in_last marks a column's last line (every column of a band
// has the same number of lines, at most 256), in_tag travels with the line (the line's number, or whatever the
// sender needs back) and must be the same for a given line in every column.
// Out: the same lines for the upscaled picture's segment columns, in the same order; n_seg_out columns.
// With use_sr = 0 the stage is wires.
//
// A sample x of the upscaled line is an 8-tap filter around srcX = -(1 << 14) + initialSubpelX + x * stepX (in 1/16384
// samples), its taps clamped to 0 .. max_x. stepX <= 1 sample, so the 8 (4) samples of an output column read at most
// 15 (11) neighbouring input samples: a window of three input columns of 8, or four of 4.
//   - arriving input columns go into six rotating column buffers;
//   - an output column is set up lane by lane (one clock a lane): its source position, the filter phase and the 8
//     clamped tap positions, relative to the window's first column;
//   - then its lines run one a clock: the window columns' line is read, each lane's 8 samples are picked and
//     multiplied by the phase's taps, summed, rounded by 7 bits and clamped to the bit depth.
// stepX and initialSubpelX are two divisions (sr_top's sequential divider, 64 clocks), redone at every band_start.
// Lines below the picture are filtered like the others (nothing reads them).
module sr_stream
  import syn_pkg::*;
  import blk_tables_pkg::*;
#(
    parameter int CW = 10                    // log2 of the most segment columns
) (
    input  logic            clk,
    input  logic            rst,
    input  logic [3:0]      bit_depth,
    input  logic            use_sr,          // the frame uses super-resolution
    input  logic            ssx,             // segments of 4 samples (a horizontally subsampled plane), else 8
    input  logic [PXW-1:0]  down_w, up_w,    // the plane's width before and after the upscale
    input  logic [PXW-1:0]  max_x,           // the last decoded sample column before the upscale
    input  logic            band_start,      // pulse while idle
    input  logic [CW:0]     n_seg_in,        // input segment columns
    output logic [CW:0]     n_seg_out,
    output logic            busy,
    input  logic            in_valid,
    output logic            in_ready,
    input  logic [95:0]     in_line,
    input  logic [8:0]      in_tag,
    input  logic            in_last,
    output logic            out_valid,
    input  logic            out_ready,
    output logic [95:0]     out_line,
    output logic [8:0]      out_tag,
    output logic            out_last
);
    localparam int SSB = 14;                 // SUPERRES_SCALE_BITS
    logic [3:0]  segw;
    logic [1:0]  sgl;                        // log2 of the segment width
    assign segw = ssx ? 4'd4 : 4'd8;
    assign sgl = ssx ? 2'd2 : 2'd3;
    logic [CW:0] n_seg_o;
    assign n_seg_o = (CW+1)'((up_w + PXW'(segw) - PXW'(1)) >> sgl);
    assign n_seg_out = use_sr ? n_seg_o : n_seg_in;

    // ---------------------------------------------------------------- stepX, initialSubpelX (7.16), once a band
    typedef enum logic [2:0] {G_OFF, G_DIV, G_INIT1, G_INIT2, G_RDY} gst_t;
    gst_t        gst;
    logic [31:0] dv_num, dv_quo, dv_num2, dv_q2, step_x;
    logic [32:0] dv_rem, dv_rem_sh;
    logic [5:0]  dv_cnt;
    logic        dv_phase, dv_neg2, dv_ge;
    logic signed [31:0] g_num, sr_err, init_subpel;
    always_comb begin
        g_num = -$signed((32'(up_w) - 32'(down_w)) << (SSB - 1)) + $signed(32'(up_w >> 1));
        dv_rem_sh = {dv_rem[31:0], dv_num[31]};
        dv_ge = dv_rem_sh >= 33'(up_w);
    end

    // ---------------------------------------------------------------- arrival: six rotating column buffers
    logic [95:0] cb [6][256];
    logic [8:0]  tg [256];
    logic        act;
    logic [CW:0] a_col;                      // input column arriving (n_seg_in: all in)
    logic [7:0]  a_ln;
    logic [2:0]  a_slot;
    logic [8:0]  nl;                         // lines of a column (known when the band's first column is complete)
    logic [CW:0] w_lo, w_hi;                 // the window's first input column; the last one the output column needs
    logic [2:0]  w_slot;                     // w_lo's buffer
    logic        a_step, in_rdy_i;
    assign in_rdy_i = act && (a_col < n_seg_in) && (a_col < w_lo + (CW+1)'(6));
    assign a_step = use_sr && in_valid && in_rdy_i;

    // ---------------------------------------------------------------- an output column
    typedef enum logic [2:0] {P_IDLE, P_SET, P_WAIT, P_RUN, P_DRAIN} pst_t;
    pst_t        pst;
    logic [CW:0] o_col;                      // output column (n_seg_o: all done)
    logic signed [31:0] s_pos;               // source position of the lane being set up
    logic [3:0]  s_l;
    logic [4:0]  idx [8][8];                 // lane, tap: the tap's sample in the window
    logic [5:0]  sub [8];                    // lane: the filter phase
    logic [8:0]  r_j;                        // line to read next
    // the lane's taps (combinational from s_pos)
    logic signed [31:0] s_px;
    logic [PXW-1:0] pos [8];
    logic [CW:0] lo_c;                       // the window's first column, from lane 0's first tap
    logic [PXW-1:0] w_x0;                    // the window's first sample column
    always_comb begin
        s_px = s_pos >>> SSB;
        for (int k = 0; k < 8; k++) begin
            logic signed [31:0] t;
            t = s_px + 32'(k) - 32'sd3;
            pos[k] = (t < 0) ? PXW'(0) : (t > 32'(max_x)) ? max_x : PXW'(t);
        end
        lo_c = (s_l == 4'd0) ? (CW+1)'(pos[0] >> sgl) : w_lo;
        w_x0 = PXW'(lo_c) << sgl;
    end

    // ---------------------------------------------------------------- the line pipeline (advances when the output can take a line)
    logic        adv;
    logic        r1_v, r2_v, r3_v, r4_v;     // the buffers' data; the window; the picked samples and taps; the products
    logic        r1_l, r2_l, r3_l, r4_l;
    logic [8:0]  r1_t, r2_t, r3_t, r4_t;
    logic [95:0] q [6];
    logic [11:0] win [24];
    logic [11:0] smp [8][8];
    logic signed [8:0]  tap [8][8];
    logic signed [21:0] prod [8][8];
    logic [95:0] o_line;
    logic        o_v, o_l;
    logic [8:0]  o_t;
    assign adv = !o_v || out_ready;
    logic        rd;                         // a line is read this clock
    assign rd = (pst == P_RUN) && adv;
    function automatic logic [2:0] slot_add(input logic [2:0] b, input logic [2:0] k);
        logic [3:0] t;
        t = 4'(b) + 4'(k);
        slot_add = (t >= 4'd6) ? 3'(t - 4'd6) : 3'(t);
    endfunction
    logic [11:0] pix_max;
    assign pix_max = 12'((13'd1 << bit_depth) - 13'd1);

    always_ff @(posedge clk) begin
        // arrival
        if (a_step) begin cb[a_slot][a_ln] <= in_line; tg[a_ln] <= in_tag; end
        // the pipeline
        if (adv) begin
            for (int b = 0; b < 6; b++) q[b] <= cb[b][r_j[7:0]];
            r1_v <= rd; r1_l <= (r_j + 9'd1 == nl); r1_t <= tg[r_j[7:0]];
            // the window: three columns of 8, or four of 4
            for (int i = 0; i < 4; i++)
                for (int m = 0; m < 8; m++)
                    if (ssx) begin if (m < 4) win[4 * i + m] <= q[slot_add(w_slot, 3'(i))][12 * m +: 12]; end
                    else if (i < 3) win[8 * i + m] <= q[slot_add(w_slot, 3'(i))][12 * m +: 12];
            r2_v <= r1_v; r2_l <= r1_l; r2_t <= r1_t;
            for (int l = 0; l < 8; l++)
                for (int k = 0; k < 8; k++) begin
                    smp[l][k] <= win[idx[l][k]];
                    tap[l][k] <= upscale_filter(sub[l], 3'(k));
                end
            r3_v <= r2_v; r3_l <= r2_l; r3_t <= r2_t;
            for (int l = 0; l < 8; l++)
                for (int k = 0; k < 8; k++) prod[l][k] <= 22'(signed'({1'b0, smp[l][k]})) * 22'(tap[l][k]);
            r4_v <= r3_v; r4_l <= r3_l; r4_t <= r3_t;
            for (int l = 0; l < 8; l++) begin
                logic signed [21:0] acc; logic signed [14:0] v;
                acc = 22'sd0;
                for (int k = 0; k < 8; k++) acc = acc + prod[l][k];
                v = 15'((acc + 22'sd64) >>> 7);                 // Round2(sum, FILTER_BITS)
                o_line[12 * l +: 12] <= (ssx && l >= 4) ? 12'd0 : (v < 0) ? 12'd0 : (v > 15'(signed'({3'b0, pix_max}))) ? pix_max : 12'(v);
            end
            o_v <= r4_v; o_l <= r4_l; o_t <= r4_t;
        end
        if (rst) begin r1_v <= 1'b0; r2_v <= 1'b0; r3_v <= 1'b0; r4_v <= 1'b0; o_v <= 1'b0; end
    end

    // ---------------------------------------------------------------- sequencing
    always_ff @(posedge clk) begin
        if (rst) begin
            gst <= G_OFF; act <= 1'b0; pst <= P_IDLE; a_col <= '0; a_ln <= '0; a_slot <= '0; nl <= '0; w_lo <= '0; w_hi <= '0; w_slot <= '0;
            o_col <= '0; r_j <= '0; s_l <= '0;
        end else begin
            if (band_start && use_sr) begin
                act <= 1'b1; a_col <= '0; a_ln <= '0; a_slot <= 3'd0; w_lo <= '0; w_slot <= 3'd0; o_col <= '0; pst <= P_IDLE; nl <= '0;
                dv_num <= (32'(down_w) << SSB) + 32'(up_w >> 1);
                dv_num2 <= (g_num < 0) ? 32'(-g_num) : 32'(g_num); dv_neg2 <= (g_num < 0);
                dv_rem <= '0; dv_quo <= '0; dv_cnt <= 6'd0; dv_phase <= 1'b0;
                gst <= G_DIV;
            end else begin
                // ---- the two divisions, then the first source position
                case (gst)
                    G_DIV: begin
                        dv_rem <= dv_ge ? dv_rem_sh - 33'(up_w) : dv_rem_sh;
                        dv_num <= {dv_num[30:0], 1'b0};
                        dv_quo <= {dv_quo[30:0], dv_ge};
                        if (dv_cnt != 6'd31) dv_cnt <= dv_cnt + 6'd1;
                        else begin
                            dv_cnt <= 6'd0; dv_rem <= '0; dv_quo <= '0;
                            if (!dv_phase) begin step_x <= {dv_quo[30:0], dv_ge}; dv_num <= dv_num2; dv_phase <= 1'b1; end
                            else begin dv_q2 <= {dv_quo[30:0], dv_ge}; gst <= G_INIT1; end
                        end
                    end
                    G_INIT1: begin sr_err <= $signed(32'(up_w) * step_x) - $signed(32'(down_w) << SSB); gst <= G_INIT2; end
                    G_INIT2: begin
                        init_subpel <= ((dv_neg2 ? -$signed(dv_q2) : $signed(dv_q2)) + 32'sd128
                                        - ((sr_err < 0) ? -$signed(32'(-sr_err) >> 1) : $signed(32'(sr_err) >> 1))) & 32'sd16383;
                        gst <= G_RDY;
                    end
                    default: ;
                endcase
                // ---- arrival
                if (a_step) begin
                    if (in_last) begin
                        nl <= 9'(a_ln) + 9'd1; a_ln <= '0; a_col <= a_col + 1'b1; a_slot <= slot_add(a_slot, 3'd1);
                    end else a_ln <= a_ln + 8'd1;
                end
                // ---- output columns
                case (pst)
                    P_IDLE: if (act && gst == G_RDY && o_col < n_seg_o) begin
                                if (o_col == '0) s_pos <= -(32'sd1 <<< SSB) + init_subpel;
                                s_l <= 4'd0; pst <= P_SET;
                            end else if (act && gst == G_RDY && o_col >= n_seg_o && a_col >= n_seg_in) act <= 1'b0;
                    P_SET: begin                                 // one lane a clock; s_pos runs on into the next column
                        for (int k = 0; k < 8; k++) idx[s_l[2:0]][k] <= 5'(pos[k] - w_x0);
                        sub[s_l[2:0]] <= 6'((s_pos & 32'sd16383) >> 8);
                        if (s_l == 4'd0) begin w_slot <= slot_add(w_slot, 3'(lo_c - w_lo)); w_lo <= lo_c; end
                        s_pos <= s_pos + $signed(step_x);
                        s_l <= s_l + 4'd1;
                        if (s_l == segw - 4'd1) begin w_hi <= (CW+1)'(pos[7] >> sgl); pst <= P_WAIT; end
                    end
                    P_WAIT: if ((a_col > w_hi) || (a_col >= n_seg_in)) begin r_j <= 9'd0; pst <= P_RUN; end
                    P_RUN: if (adv) begin
                               r_j <= r_j + 9'd1;
                               if (r_j + 9'd1 == nl) pst <= P_DRAIN;
                           end
                    P_DRAIN: if (!r1_v && !r2_v && !r3_v && !r4_v) begin o_col <= o_col + 1'b1; pst <= P_IDLE; end   // the lanes' settings are free again
                    default: pst <= P_IDLE;
                endcase
            end
        end
    end
    assign busy = use_sr && (act || o_v || r1_v || r2_v || r3_v || r4_v);
    assign in_ready  = use_sr ? in_rdy_i : out_ready;
    assign out_valid = use_sr ? o_v : in_valid;
    assign out_line  = use_sr ? o_line : in_line;
    assign out_tag   = use_sr ? o_t : in_tag;
    assign out_last  = use_sr ? o_l : in_last;
endmodule
