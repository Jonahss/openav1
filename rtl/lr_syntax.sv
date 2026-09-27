// read_lr( ) (spec 5.11.57-58): loop-restoration unit parameters signalled at superblock start, for every
// LR unit whose top-left lies in this superblock, per plane with FrameRestorationType != NONE.
// Uses the shared symbol sequencer; keeps the RefLrWiener / RefSgrXqd reference state per tile.
module lr_syntax
  import cdf_map_pkg::*;
  import syn_pkg::*;
(
    input  logic        clk,
    input  logic        rst,
    input  hdr_t        hdr,
    input  logic        tile_start,
    input  logic        start,                 // superblock start
    input  logic [10:0] r, c,                  // superblock MiRow / MiCol
    input  logic        sb128,
    output logic        busy,
    output logic        done,
    output logic        lr_done,               // one pulse per unit
    output lr_rec_t     lr_rec,
    // symbol sequencer master
    output logic              sq_go,
    output logic [CDF_AW-1:0] sq_addr,
    output logic [3:0]        sq_n,
    output logic [1:0]        sq_kind,
    input  logic              sq_done,
    input  logic [3:0]        sq_sym
);
    localparam logic [1:0] R_NONE = 2'd0, R_WIENER = 2'd1, R_SGRPROJ = 2'd2, R_SWITCHABLE = 2'd3;

    typedef enum logic [4:0] {
        S_IDLE, S_PLANE, S_UNIT, S_TYPE_W, S_WIEN, S_WIEN_W, S_SGR_SET_W, S_SGR, S_SGR_W, S_EMIT, S_NEXT,
        SE_LOOP, SE_NS_W, SE_NS2_W, SE_MORE_W, SE_BITS_W, SE_DONE,
        S_LIT, S_LIT_W
    } st_t;
    st_t st, lit_ret, se_ret;

    // reference state
    logic signed [6:0] ref_w [3][2][3];
    logic signed [7:0] ref_x [3][2];

    logic [1:0]  plane;
    logic [1:0]  frt;                          // FrameRestorationType[plane]
    logic [3:0]  usz_log2;                     // 6..8
    logic        sub_x, sub_y;
    logic [7:0]  unit_rows, unit_cols, ur_start, ur_end, uc_start, uc_end, ur, uc;
    logic [1:0]  rtype;
    logic signed [6:0] coef [2][3];
    logic        pss;
    logic [1:0]  tap;
    logic [3:0]  sgr_set;
    logic signed [7:0] xqd [2];
    logic        xi;
    // subexp decoder
    logic [7:0]  se_num;                       // numSyms (mx)
    logic [3:0]  se_k;
    logic [7:0]  se_r;                         // r - low
    logic signed [8:0] se_low;
    logic [3:0]  se_i, se_b2;
    logic [7:0]  se_mk;
    logic [7:0]  se_v;
    logic signed [8:0] se_result;
    logic [3:0]  ns_w;
    logic [8:0]  ns_m;
    // literal reader
    logic [3:0]  lit_n;
    logic [15:0] lit_val;

    // geometry helpers -------------------------------------------------------------------------------
    function automatic logic [7:0] count_units(input logic [3:0] lg, input logic [13:0] frame_size);
        logic [13:0] n;
        n = (frame_size + (14'd1 << (lg - 1))) >> lg;
        count_units = (n == 0) ? 8'd1 : 8'(n);
    endfunction
    function automatic logic [13:0] round2(input logic [12:0] x, input logic n);
        round2 = n ? (14'(x) + 14'd1) >> 1 : 14'(x);
    endfunction
    function automatic logic [7:0] ceil_div_pow2(input logic [15:0] x, input logic [3:0] lg);
        ceil_div_pow2 = 8'((x + (16'd1 << lg) - 16'd1) >> lg);
    endfunction

    logic [5:0] sb4;
    assign sb4 = sb128 ? 6'd32 : 6'd16;

    // Wiener / Sgrproj constants
    function automatic logic signed [6:0] wt_min(input logic [1:0] j); wt_min = (j == 0) ? -7'sd5 : (j == 1) ? -7'sd23 : -7'sd17; endfunction
    function automatic logic signed [6:0] wt_max(input logic [1:0] j); wt_max = (j == 0) ? 7'sd10 : (j == 1) ? 7'sd8 : 7'sd46; endfunction
    function automatic logic [3:0] wt_k(input logic [1:0] j); wt_k = (j == 0) ? 4'd1 : (j == 1) ? 4'd2 : 4'd3; endfunction
    function automatic logic signed [7:0] xq_min(input logic i); xq_min = i ? -8'sd32 : -8'sd96; endfunction
    function automatic logic signed [7:0] xq_max(input logic i); xq_max = i ? 8'sd95 : 8'sd31; endfunction
    // Sgr_Params[set][0] (r0) is 0 for sets 10..13; Sgr_Params[set][2] (r1) is 0 for sets 14, 15
    function automatic logic sgr_radius_nz(input logic [3:0] set, input logic i);
        sgr_radius_nz = i ? (set < 4'd14) : !(set >= 4'd10 && set <= 4'd13);
    endfunction

    // inverse_recenter / unsigned-with-ref (combinational on se_v)
    function automatic logic [7:0] inv_recenter(input logic [7:0] rr, input logic [7:0] v);
        if (9'(v) > 9'(rr) * 2) inv_recenter = v;
        else if (v[0]) inv_recenter = rr - ((v + 8'd1) >> 1);
        else inv_recenter = rr + (v >> 1);
    endfunction
    logic [7:0] se_unsigned;
    always_comb begin
        if (9'(se_r) * 2 <= 9'(se_num)) se_unsigned = inv_recenter(se_r, se_v);
        else se_unsigned = se_num - 8'd1 - inv_recenter(se_num - 8'd1 - se_r, se_v);
    end

    // symbol requests -------------------------------------------------------------------------------------
    always_comb begin
        sq_go = 1'b0; sq_addr = '0; sq_n = 4'd1; sq_kind = 2'd0;
        case (st)
            S_UNIT: begin
                sq_go = 1'b1;
                case (frt)
                    R_WIENER:  begin sq_addr = CDF_AW'(CDF_USE_WIENER); sq_n = 4'd1; end
                    R_SGRPROJ: begin sq_addr = CDF_AW'(CDF_USE_SGRPROJ); sq_n = 4'd1; end
                    default:   begin sq_addr = CDF_AW'(CDF_RESTORATION_TYPE); sq_n = 4'd2; end
                endcase
            end
            S_LIT: if (lit_n != 4'd0) begin sq_go = 1'b1; sq_kind = 2'd2; end
            default: ;
        endcase
    end

    assign busy = (st != S_IDLE);

    always_ff @(posedge clk) begin
        done <= 1'b0; lr_done <= 1'b0;
        if (rst) begin
            st <= S_IDLE;
        end else begin
            if (tile_start) begin
                for (int p = 0; p < 3; p++) begin
                    for (int q = 0; q < 2; q++) begin
                        ref_w[p][q][0] <= 7'sd3; ref_w[p][q][1] <= -7'sd7; ref_w[p][q][2] <= 7'sd15;   // Wiener_Taps_Mid
                    end
                    ref_x[p][0] <= -8'sd32; ref_x[p][1] <= 8'sd31;                                   // Sgrproj_Xqd_Mid
                end
            end
            case (st)
                S_IDLE: if (start) begin plane <= 2'd0; st <= S_PLANE; end
                // ---- per plane: unit range covered by this superblock
                S_PLANE: begin
                    if (plane >= (hdr.mono ? 2'd1 : 2'd3)) begin done <= 1'b1; st <= S_IDLE; end
                    else if (hdr.lr_type[2*plane +: 2] == R_NONE) plane <= plane + 2'd1;
                    else begin
                        logic [3:0] lg;
                        logic sx, sy;
                        logic [7:0] urows, ucols, urs, ure, ucs, uce;
                        sx = (plane != 0) && hdr.ssx; sy = (plane != 0) && hdr.ssy;
                        lg = 4'd6 + 4'(hdr.lr_size[2*plane +: 2]);
                        urows = count_units(lg, round2(hdr.frame_height, sy));
                        ucols = count_units(lg, round2(hdr.upscaled_width, sx));
                        urs = ceil_div_pow2(16'(r) * (16'd4 >> sy), lg);
                        ure = ceil_div_pow2((16'(r) + 16'(sb4)) * (16'd4 >> sy), lg);
                        ucs = ceil_div_pow2(16'(c) * (16'd4 >> sx), lg);
                        uce = ceil_div_pow2((16'(c) + 16'(sb4)) * (16'd4 >> sx), lg);
                        frt <= hdr.lr_type[2*plane +: 2]; usz_log2 <= lg; sub_x <= sx; sub_y <= sy;
                        unit_rows <= urows; unit_cols <= ucols;
                        ur_start <= urs; ur_end <= (ure < urows) ? ure : urows;
                        uc_start <= ucs; uc_end <= (uce < ucols) ? uce : ucols;
                        ur <= urs; uc <= ucs;
                        if (urs >= ((ure < urows) ? ure : urows) || ucs >= ((uce < ucols) ? uce : ucols)) plane <= plane + 2'd1;
                        else st <= S_UNIT;
                    end
                end
                // ---- one unit
                S_UNIT: begin
                    for (int q = 0; q < 2; q++) for (int j = 0; j < 3; j++) coef[q][j] <= 7'sd0;
                    sgr_set <= 4'd0; xqd[0] <= 8'sd0; xqd[1] <= 8'sd0;
                    st <= S_TYPE_W;
                end
                S_TYPE_W: if (sq_done) begin
                    case (frt)
                        R_WIENER:  rtype <= sq_sym[0] ? R_WIENER : R_NONE;
                        R_SGRPROJ: rtype <= sq_sym[0] ? R_SGRPROJ : R_NONE;
                        default:   rtype <= sq_sym[1:0];
                    endcase
                    pss <= 1'b0; tap <= (plane != 0) ? 2'd1 : 2'd0;
                    case (frt == R_SWITCHABLE ? sq_sym[1:0] : (sq_sym[0] ? frt : R_NONE))
                        R_WIENER:  st <= S_WIEN;
                        R_SGRPROJ: begin lit_n <= 4'd4; lit_val <= '0; lit_ret <= S_SGR_SET_W; st <= S_LIT; end
                        default:   st <= S_EMIT;
                    endcase
                end
                // ---- Wiener taps: decode_signed_subexp_with_ref_bool(min, max+1, k, ref)
                S_WIEN: begin
                    se_low <= 9'(wt_min(tap));
                    se_num <= 8'(9'(wt_max(tap)) + 9'sd1 - 9'(wt_min(tap)));
                    se_k <= wt_k(tap);
                    se_r <= 8'(9'(ref_w[plane][pss][tap]) - 9'(wt_min(tap)));
                    se_i <= 4'd0; se_mk <= 8'd0; se_ret <= S_WIEN_W; st <= SE_LOOP;
                end
                S_WIEN_W: begin
                    coef[pss][tap] <= 7'(se_result);
                    ref_w[plane][pss][tap] <= 7'(se_result);
                    if (tap != 2'd2) begin tap <= tap + 2'd1; st <= S_WIEN; end
                    else if (!pss) begin pss <= 1'b1; tap <= (plane != 0) ? 2'd1 : 2'd0; st <= S_WIEN; end
                    else st <= S_EMIT;
                end
                // ---- Sgrproj
                S_SGR_SET_W: begin sgr_set <= lit_val[3:0]; xi <= 1'b0; st <= S_SGR; end
                S_SGR: begin
                    if (sgr_radius_nz(sgr_set, xi)) begin
                        se_low <= 9'(xq_min(xi));
                        se_num <= 8'(9'(xq_max(xi)) + 9'sd1 - 9'(xq_min(xi)));
                        se_k <= 4'd4;
                        se_r <= 8'(9'(ref_x[plane][xi]) - 9'(xq_min(xi)));
                        se_i <= 4'd0; se_mk <= 8'd0; se_ret <= S_SGR_W; st <= SE_LOOP;
                    end else begin
                        logic signed [8:0] v;
                        v = xi ? (9'sd128 - 9'(ref_x[plane][0])) : 9'sd0;
                        if (xi) v = (v < 9'(xq_min(1'b1))) ? 9'(xq_min(1'b1)) : (v > 9'(xq_max(1'b1))) ? 9'(xq_max(1'b1)) : v;
                        xqd[xi] <= 8'(v); ref_x[plane][xi] <= 8'(v);
                        if (xi) st <= S_EMIT; else xi <= 1'b1;
                    end
                end
                S_SGR_W: begin
                    xqd[xi] <= 8'(se_result); ref_x[plane][xi] <= 8'(se_result);
                    if (xi) st <= S_EMIT; else begin xi <= 1'b1; st <= S_SGR; end
                end
                // ---- emit the unit record, advance
                S_EMIT: begin
                    lr_rec.plane <= plane; lr_rec.unit_row <= ur; lr_rec.unit_col <= uc; lr_rec.lr_type <= rtype;
                    lr_rec.wiener <= {coef[1][2], coef[1][1], coef[1][0], coef[0][2], coef[0][1], coef[0][0]};
                    lr_rec.sgr_set <= sgr_set; lr_rec.xqd <= {xqd[1], xqd[0]};
                    lr_done <= 1'b1;
                    st <= S_NEXT;
                end
                S_NEXT: begin
                    if (uc + 8'd1 < uc_end) begin uc <= uc + 8'd1; st <= S_UNIT; end
                    else if (ur + 8'd1 < ur_end) begin uc <= uc_start; ur <= ur + 8'd1; st <= S_UNIT; end
                    else begin plane <= plane + 2'd1; st <= S_PLANE; end
                end
                // ---- decode_subexp_bool(numSyms, k) -> se_v, then unsigned-with-ref + low -> se_result
                SE_LOOP: begin
                    logic [3:0] b2;
                    logic [7:0] a;
                    b2 = (se_i != 0) ? se_k + se_i - 4'd1 : se_k;
                    a = 8'd1 << b2;
                    se_b2 <= b2;
                    if (9'(se_num) <= 9'(se_mk) + 9'(a) * 3) begin
                        // read_ns(numSyms - mk): w = bit length of n, m = (1<<w) - n
                        logic [7:0] n;
                        logic [3:0] w;
                        n = se_num - se_mk;
                        w = (n >= 128) ? 4'd8 : (n >= 64) ? 4'd7 : (n >= 32) ? 4'd6 : (n >= 16) ? 4'd5 : (n >= 8) ? 4'd4 : (n >= 4) ? 4'd3 : (n >= 2) ? 4'd2 : 4'd1;
                        ns_w <= w; ns_m <= (9'd1 << w) - 9'(n);
                        lit_n <= w - 4'd1; lit_val <= '0; lit_ret <= SE_NS_W; st <= S_LIT;
                    end else begin
                        lit_n <= 4'd1; lit_val <= '0; lit_ret <= SE_MORE_W; st <= S_LIT;
                    end
                end
                SE_NS_W: begin
                    if (9'(lit_val[7:0]) < ns_m) begin se_v <= lit_val[7:0] + se_mk; st <= SE_DONE; end
                    else begin lit_n <= 4'd1; lit_ret <= SE_NS2_W; st <= S_LIT; end     // lit_val keeps v; one more bit
                end
                SE_NS2_W: begin
                    // (v << 1) - m + extra_bit, with lit_val now = (v << 1) | bit
                    se_v <= 8'(9'(lit_val[8:0]) - ns_m) + se_mk;
                    st <= SE_DONE;
                end
                SE_MORE_W: begin
                    if (lit_val[0]) begin se_i <= se_i + 4'd1; se_mk <= se_mk + (8'd1 << se_b2); st <= SE_LOOP; end
                    else begin lit_n <= se_b2; lit_val <= '0; lit_ret <= SE_BITS_W; st <= S_LIT; end
                end
                SE_BITS_W: begin se_v <= lit_val[7:0] + se_mk; st <= SE_DONE; end
                SE_DONE: begin se_result <= 9'(se_unsigned) + se_low; st <= se_ret; end
                // ---- literal reader
                S_LIT: begin
                    if (lit_n == 4'd0) st <= lit_ret;
                    else st <= S_LIT_W;
                end
                S_LIT_W: if (sq_done) begin
                    lit_val <= {lit_val[14:0], sq_sym[0]};
                    lit_n <= lit_n - 4'd1;
                    st <= S_LIT;
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
