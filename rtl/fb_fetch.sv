// Ring fetch (intra block copy, 7.11.3 with refIdx = -1, on the window build): the part of a prediction unit's clamped
// source region that is still in the frame memory (fb0: the current picture's recent superblock rows, or the whole
// picture without a window), read through the 4-wide frame buffer port and written into mc_filt's patch buffer with
// ref_fetch's geometry and clamping (edge replication), so the two loaders fill one patch: this one the patch rows
// [r_lo, r_hi) of the h + 7, ref_fetch the rows the window has flushed to DDR. Rows [y0 - 3, y0 + h + 4), columns
// [x0 - 3, x0 - 3 + 4 (ngrp + 2)) as in ref_fetch. Per row: the aligned groups of 4 covering the clamped column range
// are read one a cycle (data the next cycle) into a line buffer, then the patch words are written with each lane's
// column clamped into the range.
module fb_fetch #(
    parameter int PW = 12,
    parameter int FBX = 9,
    parameter int FBY = 9
) (
    input  logic          clk,
    input  logic          rst,
    // ---- request (accepted when req_ready) ----
    input  logic          req_valid,
    output logic          req_ready,
    input  logic          req_lst,
    input  logic [1:0]    req_plane,
    input  logic [15:0]   req_lastx,            // last sample column of the plane (((MiCols * 4 + subX) >> subX) - 1)
    input  logic [15:0]   req_lasty,            // last sample row
    input  logic signed [16:0] req_x0,          // unit position, integer samples
    input  logic signed [16:0] req_y0,
    input  logic [6:0]    req_w,                // 2 .. 64
    input  logic [6:0]    req_r_lo,             // patch rows [r_lo, r_hi) to produce, within 0 .. h + 7
    input  logic [6:0]    req_r_hi,
    output logic          done,                 // 1-cycle pulse: the rows are in the patch
    // ---- frame buffer read port: the aligned group of 4 samples containing fb_x, data the next cycle ----
    output logic          fb_re,
    output logic [1:0]    fb_plane,
    output logic [FBX-1:0] fb_x,
    output logic [FBY-1:0] fb_y,
    input  logic [4*PW-1:0] fb_rdata4,
    // ---- patch writes (mc_filt) ----
    output logic          patch_we,
    output logic          patch_list,
    output logic [6:0]    patch_row,
    output logic [4:0]    patch_w4,
    output logic [4*PW-1:0] patch_d4
);
    localparam int LINE = 80;                             // line buffer samples (a row covers at most 19 groups = 76)

    typedef enum logic [2:0] {F_IDLE, F_ROW, F_RD, F_RDW, F_WR, F_NEXT} st_t;
    st_t st;

    logic        q_lst;
    logic [1:0]  q_plane;
    logic [15:0] q_lastx, q_lasty;
    logic signed [16:0] q_x0, q_y0;
    logic [4:0]  q_nw;                                    // patch words per row = ngrp + 2
    logic [6:0]  q_r_hi;
    logic [15:0] cx_lo, cx_hi;                            // clamped column range of the region
    logic [15:0] cx_al;                                   // cx_lo aligned down to a group of 4: the line buffer's column 0
    logic [6:0]  r;                                       // patch row
    logic [15:0] src_y;
    logic [4:0]  g, ng;                                   // group being read / groups in the row
    logic        rd_v;                                    // a read issued last cycle lands now
    logic [4:0]  rd_g;
    logic [PW-1:0] line [0:LINE-1];
    logic [4:0]  w4;                                      // patch word being written

    function automatic logic [15:0] clip_col(input logic signed [17:0] c, input logic [15:0] last);
        if (c < 18'sd0) return 16'd0;
        if (c > 18'(signed'({2'b0, last}))) return last;
        return 16'(c);
    endfunction

    assign req_ready = (st == F_IDLE);
    assign fb_re    = (st == F_RD);
    assign fb_plane = q_plane;
    assign fb_x     = FBX'(cx_al + {11'd0, g, 2'b00});
    assign fb_y     = FBY'(src_y);

    // the read data: group rd_g of the row lands the cycle after its read
    always_ff @(posedge clk) begin
        rd_v <= !rst && (st == F_RD);
        rd_g <= g;
        if (rd_v)
            for (int l = 0; l < 4; l++)
                if (({2'd0, rd_g} << 2) + 7'(l) < 7'(LINE)) line[({2'd0, rd_g} << 2) + 7'(l)] <= fb_rdata4[l*PW +: PW];
    end

    // patch word lanes: column of lane l clamped into the region's range, read from the line buffer
    logic [4*PW-1:0] word_d4;
    always_comb begin
        for (int l = 0; l < 4; l++) begin
            logic signed [17:0] col;
            logic [15:0] cc;
            col = 18'(q_x0) - 18'sd3 + 18'(signed'({13'd0, w4})) * 18'sd4 + 18'(l);
            cc = clip_col(col, q_lastx) - cx_al;
            word_d4[l*PW +: PW] = line[cc[6:0]];
        end
    end
    assign patch_d4   = word_d4;
    assign patch_list = q_lst;
    assign patch_row  = r;
    assign patch_w4   = w4;
    assign patch_we   = (st == F_WR);

    always_ff @(posedge clk) begin
        done <= 1'b0;
        if (rst) begin
            st <= F_IDLE;
        end else case (st)
            F_IDLE: if (req_valid) begin
                logic [4:0] nw;
                nw = 5'((req_w + 7'd3) >> 2) + 5'd2;
                q_lst <= req_lst; q_plane <= req_plane; q_lastx <= req_lastx; q_lasty <= req_lasty;
                q_x0 <= req_x0; q_y0 <= req_y0; q_nw <= nw; q_r_hi <= req_r_hi;
                cx_lo <= clip_col(18'(req_x0) - 18'sd3, req_lastx);
                cx_hi <= clip_col(18'(req_x0) - 18'sd3 + 18'(signed'({13'd0, nw})) * 18'sd4 - 18'sd1, req_lastx);
                r <= req_r_lo;
                if (req_r_lo < req_r_hi) st <= F_ROW;
                else done <= 1'b1;                                            // nothing to produce
            end
            F_ROW: begin
                logic signed [17:0] yy;
                yy = 18'(q_y0) - 18'sd3 + 18'(signed'({11'd0, r}));
                src_y <= clip_col(yy, q_lasty);
                cx_al <= {cx_lo[15:2], 2'b00};
                ng <= 5'(((cx_hi - {cx_lo[15:2], 2'b00}) >> 2) + 16'd1);
                g <= 5'd0;
                st <= F_RD;
            end
            F_RD: begin                                                       // one group a cycle
                if (g == ng - 5'd1) st <= F_RDW;
                else g <= g + 5'd1;
            end
            F_RDW: begin w4 <= 5'd0; st <= F_WR; end                           // the last group lands during this cycle
            F_WR: begin
                if (w4 == q_nw - 5'd1) st <= F_NEXT;
                else w4 <= w4 + 5'd1;
            end
            F_NEXT: begin
                if (r + 7'd1 >= q_r_hi) begin done <= 1'b1; st <= F_IDLE; end
                else begin r <= r + 7'd1; st <= F_ROW; end
            end
            default: st <= F_IDLE;
        endcase
    end
endmodule
