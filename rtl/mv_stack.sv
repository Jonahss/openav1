// find_mv_stack (7.10.2) for intra block copy: single reference (the current frame) inside an intra frame.
// Builds RefStackMv / WeightStack from the intrabc neighbours' motion vectors with the spec's scans
// (row -1, col -1, top-right point, top-left point, rows -3 / -5, cols -3 / -5), the two sorts, the extra
// search (which in an intra frame only fills global (0, 0) entries), the clamping, and delivers PredMv as
// assign_mv (5.11.26) selects it: stack[0], else stack[1], else the fixed fallback vector.
// Not modelled (inter frames): global motion candidates, temporal candidates, compound stacks and the
// NewMv / RefMv / DrlCtx contexts (intrabc reads none of them).
// Neighbour data comes from the mv_info memory (mi_store): one registered read per scanned 4x4 position.
module mv_stack
  import syn_pkg::*;
  import blk_tables_pkg::*;
(
    input  logic        clk,
    input  logic        rst,
    input  hdr_t        hdr,
    input  logic        parity,               // frame parity of entries written this frame
    input  logic        start,
    input  logic [10:0] r, c,                 // MiRow, MiCol
    input  logic [4:0]  bs,                   // MiSize
    output logic        busy,
    output logic        done,                 // 1-cycle pulse; pred_* valid from then on
    output logic signed [17:0] pred_row, pred_col,
    // mv_info read port (entry valid the cycle after the address)
    output logic [10:0] rd_row, rd_col,
    input  mv_ent_t     rd_ent
);
    localparam int REF_CAT_LEVEL = 640;
    localparam int MV_BORDER = 128;
    localparam int INTRABC_DELAY_PIXELS = 256;

    // ---------------------------------------------------------------- block geometry
    logic [5:0] bw4, bh4;
    assign bw4 = num4x4w(bs);
    assign bh4 = num4x4h(bs);
    logic [10:0] mi_rows, mi_cols;
    assign mi_rows = hdr.mi_rows;
    assign mi_cols = hdr.mi_cols;

    function automatic logic is_inside(input logic signed [12:0] rr, input logic signed [12:0] cc);
        is_inside = (cc >= 13'(signed'({2'b0, hdr.mi_col_start}))) && (cc < 13'(signed'({2'b0, hdr.mi_col_end}))) &&
                    (rr >= 13'(signed'({2'b0, hdr.mi_row_start}))) && (rr < 13'(signed'({2'b0, hdr.mi_row_end})));
    endfunction

    // lower_mv_precision (7.10.2.10) with force_integer_mv = 1 (every intra frame): round to whole samples
    function automatic logic signed [17:0] lower_int(input logic signed [17:0] v);
        logic [17:0] a, a_int;
        a = (v < 0) ? 18'(-v) : 18'(v);
        a_int = (a + 18'd3) >> 3;
        lower_int = (v > 0) ? 18'(signed'(a_int << 3)) : -18'(signed'(a_int << 3));
    endfunction

    // ---------------------------------------------------------------- the stack
    logic signed [17:0] st_row [8], st_col [8];
    logic [15:0] weight [8];
    logic [3:0]  n_found, n_nearest;

    // ---------------------------------------------------------------- scan control
    typedef enum logic [3:0] {P_ROW1, P_COL1, P_TR, P_TL, P_ROW3, P_COL3, P_ROW5, P_COL5, P_END} phase_t;
    phase_t ph;
    typedef enum logic [3:0] {M_IDLE, M_PH, M_ISSUE, M_WAIT, M_PROC, M_NEAR, M_SORT, M_SORT_END, M_FILL, M_CLAMP, M_SEL, M_DONE} st_t;
    st_t st;
    logic        is_row, is_point;
    logic signed [12:0] d_row, d_col;         // scan offsets (after the odd-position adjustment)
    logic [5:0]  end4, i;
    logic        step16, far;                 // far: |delta| > 1
    logic signed [12:0] mv_r, mv_c;           // position being scanned
    logic [5:0]  cand_len;
    logic [15:0] cand_w;
    logic signed [17:0] cand_row, cand_col;
    logic        cand_ok;
    logic [7:0]  hit;                         // stack match (idx < n_found)
    // sorting
    logic [3:0]  s_start, s_end, s_idx, s_new;
    logic        s_second;

    always_comb begin
        // position of the current scan step
        if (is_point) begin mv_r = 13'(signed'({2'b0, r})) + d_row; mv_c = 13'(signed'({2'b0, c})) + d_col; end
        else if (is_row) begin mv_r = 13'(signed'({2'b0, r})) + d_row; mv_c = 13'(signed'({2'b0, c})) + d_col + 13'(signed'({7'b0, i})); end
        else begin mv_r = 13'(signed'({2'b0, r})) + d_row + 13'(signed'({7'b0, i})); mv_c = 13'(signed'({2'b0, c})) + d_col; end
        rd_row = 11'(mv_r); rd_col = 11'(mv_c);
        // candidate from the entry just read
        if (is_point) cand_len = 6'd0;
        else if (is_row) cand_len = (bw4 < num4x4w(rd_ent.bsize)) ? bw4 : num4x4w(rd_ent.bsize);
        else cand_len = (bh4 < num4x4h(rd_ent.bsize)) ? bh4 : num4x4h(rd_ent.bsize);
        if (!is_point && far && cand_len < 6'd2) cand_len = 6'd2;
        if (!is_point && step16 && cand_len < 6'd4) cand_len = 6'd4;
        cand_w = is_point ? 16'd4 : 16'(cand_len) * 16'd2;
        cand_ok = rd_ent.is_intrabc && (rd_ent.parity == parity);     // IsInters && RefFrames[0] == INTRA_FRAME, written this frame
        cand_row = lower_int(rd_ent.mv_row);
        cand_col = lower_int(rd_ent.mv_col);
        for (int k = 0; k < 8; k++)
            hit[k] = (4'(k) < n_found) && (st_row[k] == cand_row) && (st_col[k] == cand_col);
    end

    // clamp_mv_row / clamp_mv_col (border = MV_BORDER + bh * 8 / bw * 8)
    logic signed [19:0] top_lo, top_hi, left_lo, left_hi;
    always_comb begin
        logic signed [19:0] border_r, border_c;
        border_r = 20'(MV_BORDER) + 20'(signed'({14'b0, bh4})) * 20'sd32;       // bh * 8 = bh4 * 4 * 8
        border_c = 20'(MV_BORDER) + 20'(signed'({14'b0, bw4})) * 20'sd32;
        top_lo  = -(20'(signed'({9'b0, r})) * 20'sd32) - border_r;             // mbToTopEdge - border
        top_hi  = (20'(signed'({9'b0, mi_rows})) - 20'(signed'({14'b0, bh4})) - 20'(signed'({9'b0, r}))) * 20'sd32 + border_r;
        left_lo = -(20'(signed'({9'b0, c})) * 20'sd32) - border_c;
        left_hi = (20'(signed'({9'b0, mi_cols})) - 20'(signed'({14'b0, bw4})) - 20'(signed'({9'b0, c}))) * 20'sd32 + border_c;
    end
    function automatic logic signed [17:0] clamp18(input logic signed [17:0] v, input logic signed [19:0] lo, input logic signed [19:0] hi);
        logic signed [19:0] x;
        x = 20'(v);
        if (x < lo) x = lo;
        else if (x > hi) x = hi;
        clamp18 = 18'(x);
    endfunction

    // fallback vector (assign_mv when both stack entries are zero)
    logic [5:0] sb_size4;
    assign sb_size4 = hdr.sb128 ? 6'd32 : 6'd16;
    logic top_of_tile;
    assign top_of_tile = (13'(signed'({2'b0, r})) - 13'(signed'({7'b0, sb_size4}))) < 13'(signed'({2'b0, hdr.mi_row_start}));

    assign busy = (st != M_IDLE);

    always_ff @(posedge clk) begin
        done <= 1'b0;
        if (rst) begin
            st <= M_IDLE;
        end else case (st)
            M_IDLE: if (start) begin
                n_found <= 4'd0; n_nearest <= 4'd0;
                for (int k = 0; k < 8; k++) begin weight[k] <= 16'd0; st_row[k] <= 18'sd0; st_col[k] <= 18'sd0; end
                ph <= P_ROW1; st <= M_PH;
            end
            // ---- phase setup (scan row / scan col / scan point parameters)
            M_PH: begin
                i <= 6'd0; is_point <= 1'b0; far <= 1'b0; is_row <= 1'b0; step16 <= 1'b0; d_row <= 13'sd0; d_col <= 13'sd0;
                case (ph)
                    P_ROW1, P_ROW3, P_ROW5: begin
                        logic signed [12:0] dr;
                        dr = (ph == P_ROW1) ? -13'sd1 : (ph == P_ROW3) ? -13'sd3 : -13'sd5;
                        is_row <= 1'b1;
                        begin   // end4 = Min(Min(bw4, MiCols - MiCol), 16), compared at full width
                            logic [10:0] rem, e;
                            rem = mi_cols - c;
                            e = (11'(bw4) < rem) ? 11'(bw4) : rem;
                            end4 <= (e > 11'd16) ? 6'd16 : 6'(e);
                        end
                        step16 <= (bw4 >= 6'd16);
                        if (ph != P_ROW1) begin far <= 1'b1; d_row <= dr + 13'(signed'({12'b0, r[0]})); d_col <= 13'sd1 - 13'(signed'({12'b0, c[0]})); end
                        else d_row <= dr;
                        // rows -5 only for bh4 > 1
                        if (ph == P_ROW5 && bh4 == 6'd1) begin ph <= P_COL5; st <= M_PH; end
                        else st <= M_ISSUE;
                    end
                    P_COL1, P_COL3, P_COL5: begin
                        logic signed [12:0] dc;
                        dc = (ph == P_COL1) ? -13'sd1 : (ph == P_COL3) ? -13'sd3 : -13'sd5;
                        begin
                            logic [10:0] rem, e;
                            rem = mi_rows - r;
                            e = (11'(bh4) < rem) ? 11'(bh4) : rem;
                            end4 <= (e > 11'd16) ? 6'd16 : 6'(e);
                        end
                        step16 <= (bh4 >= 6'd16);
                        if (ph != P_COL1) begin far <= 1'b1; d_row <= 13'sd1 - 13'(signed'({12'b0, r[0]})); d_col <= dc + 13'(signed'({12'b0, c[0]})); end
                        else d_col <= dc;
                        if (ph == P_COL5 && bw4 == 6'd1) begin ph <= P_END; st <= M_PH; end
                        else st <= M_ISSUE;
                    end
                    P_TR: begin
                        is_point <= 1'b1; d_row <= -13'sd1; d_col <= 13'(signed'({7'b0, bw4}));
                        if ((bw4 > bh4 ? bw4 : bh4) <= 6'd16) st <= M_ISSUE;
                        else st <= M_NEAR;                          // no top-right scan for large blocks
                    end
                    P_TL: begin is_point <= 1'b1; d_row <= -13'sd1; d_col <= -13'sd1; st <= M_ISSUE; end
                    default: begin                                  // P_END: sorts
                        s_start <= 4'd0; s_end <= n_nearest; s_second <= 1'b0; st <= M_SORT_END;
                    end
                endcase
            end
            // ---- one scanned position: address out, entry in, stack update
            M_ISSUE: begin
                if (!is_inside(mv_r, mv_c)) begin
                    // scan row / col: stop the whole scan; point: nothing to add
                    st <= (ph == P_TR) ? M_NEAR : M_PH;
                    if (ph != P_TR) ph <= phase_t'(ph + 4'd1);
                end else st <= M_WAIT;
            end
            M_WAIT: st <= M_PROC;
            M_PROC: begin
                if (cand_ok) begin
                    if (|hit) begin
                        for (int k = 0; k < 8; k++) if (hit[k]) weight[k] <= weight[k] + cand_w;
                    end else if (n_found < 4'd8) begin
                        st_row[n_found[2:0]] <= cand_row; st_col[n_found[2:0]] <= cand_col; weight[n_found[2:0]] <= cand_w;
                        n_found <= n_found + 4'd1;
                    end
                end
                if (is_point || (i + cand_len >= end4)) begin
                    st <= (ph == P_TR) ? M_NEAR : M_PH;
                    if (ph != P_TR) ph <= phase_t'(ph + 4'd1);
                end else begin
                    i <= i + cand_len; st <= M_ISSUE;
                end
            end
            // ---- after the three nearest scans: numNearest, REF_CAT_LEVEL bonus
            M_NEAR: begin
                n_nearest <= n_found;
                for (int k = 0; k < 8; k++) if (4'(k) < n_found) weight[k] <= weight[k] + 16'(REF_CAT_LEVEL);
                ph <= P_TL; st <= M_PH;
            end
            // ---- sorting process (7.10.2.11): the spec's loop, one compare/swap per cycle
            M_SORT_END: begin
                if (s_end > s_start) begin s_new <= s_start; s_idx <= s_start + 4'd1; st <= M_SORT; end
                else if (!s_second) begin s_second <= 1'b1; s_start <= n_nearest; s_end <= n_found; end
                else st <= M_FILL;
            end
            M_SORT: begin
                if (s_idx < s_end) begin
                    if (weight[s_idx[2:0] - 3'd1] < weight[s_idx[2:0]]) begin
                        weight[s_idx[2:0] - 3'd1] <= weight[s_idx[2:0]]; weight[s_idx[2:0]] <= weight[s_idx[2:0] - 3'd1];
                        st_row[s_idx[2:0] - 3'd1] <= st_row[s_idx[2:0]]; st_row[s_idx[2:0]] <= st_row[s_idx[2:0] - 3'd1];
                        st_col[s_idx[2:0] - 3'd1] <= st_col[s_idx[2:0]]; st_col[s_idx[2:0]] <= st_col[s_idx[2:0] - 3'd1];
                        s_new <= s_idx;
                    end
                    s_idx <= s_idx + 4'd1;
                end else begin
                    s_end <= s_new; st <= M_SORT_END;
                end
            end
            // ---- extra search (7.10.2.12): in an intra frame no neighbour has a reference frame > INTRA_FRAME,
            // so it only writes the global (0, 0) vectors into entries NumMvFound..1 (NumMvFound unchanged)
            M_FILL: begin
                for (int k = 0; k < 2; k++) if (4'(k) >= n_found) begin st_row[k] <= 18'sd0; st_col[k] <= 18'sd0; end
                st <= M_CLAMP;
            end
            // ---- context and clamping process: clamp the found entries
            M_CLAMP: begin
                for (int k = 0; k < 8; k++) if (4'(k) < n_found) begin
                    st_row[k] <= clamp18(st_row[k], top_lo, top_hi);
                    st_col[k] <= clamp18(st_col[k], left_lo, left_hi);
                end
                st <= M_SEL;
            end
            // ---- assign_mv: PredMv = stack[0] | stack[1] | fallback
            M_SEL: begin
                if (st_row[0] != 18'sd0 || st_col[0] != 18'sd0) begin pred_row <= st_row[0]; pred_col <= st_col[0]; end
                else if (st_row[1] != 18'sd0 || st_col[1] != 18'sd0) begin pred_row <= st_row[1]; pred_col <= st_col[1]; end
                else if (top_of_tile) begin pred_row <= 18'sd0; pred_col <= -18'(signed'((18'(sb_size4) * 18'd4 + 18'(INTRABC_DELAY_PIXELS)) * 18'd8)); end
                else begin pred_row <= -18'(signed'(18'(sb_size4) * 18'd32)); pred_col <= 18'sd0; end
                st <= M_DONE;
            end
            M_DONE: begin done <= 1'b1; st <= M_IDLE; end
            default: st <= M_IDLE;
        endcase
    end
endmodule
