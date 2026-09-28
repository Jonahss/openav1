// openav1 — 2D inverse transform block (AV1 spec 7.13.3) around the itx1d engine.
//
// Load the dequantized coefficients through the write port (only the top-left 32x32 can be
// non-zero, per spec), pulse start with the transform size / type / bit depth / lossless flag,
// wait for done, then read the h x w Residual through the read port.
//
// Sequence, exactly as the spec orders it:
//   rows i = 0..h-1 : T[j] = Dequant[i][j] (0 for i or j >= 32)
//                     if |log2W - log2H| == 1: T[j] = Round2(T[j] * 2896, 12)
//                     1D row transform (WHT shift 2 if lossless), r = BitDepth + 8
//                     Residual[i][j] = Round2(T[j], rowShift)   then clamp to colClampRange bits
//   cols j = 0..w-1 : T[i] = Residual[i][j]; 1D column transform (WHT shift 0), r = colClampRange
//                     Residual[i][j] = Round2(T[i], colShift)   (colShift = 4, or 0 if lossless)
// Rows i >= 32 are all-zero and every 1D transform maps zero to zero, so they are skipped.
//
// Dequant and Residual live in register arrays for simulation; a real implementation puts them in a
// transpose memory. Flips (FLIPADST) are applied by the reconstruction stage, not here.
//
// v2 (throughput): NLANES 1D engines run NLANES rows (then NLANES columns) at once. A group of rows is
// loaded into the lanes in one cycle, transformed (DCT4 5 cycles .. DCT64 65), and stored in one cycle;
// the next group starts the cycle after. Row groups at i >= 32 are all-zero and written without a
// transform. Cost per block ~ ceil(h/NLANES) x (rowprog + 1) + ceil(w/NLANES) x (colprog + 1) + 2:
// a 4x4 DCT block is 14 cycles with 4 lanes (was ~52 with one engine visited row by row).
module itx2d #(
    parameter int TW = 20,
    parameter int NLANES = 4              // power of two, 1..32 (row groups must not straddle row 32)
) (
    input  logic          clk,
    input  logic          rst,

    // coefficient write port (valid while idle): addr = i*32 + j, i,j < 32; coef_clr zeroes the whole array
    input  logic          coef_clr,
    input  logic          coef_we,
    input  logic [9:0]    coef_addr,
    input  logic [TW-1:0] coef_data,

    // control
    input  logic          start,
    input  logic [4:0]    tx_sz,          // TX_4X4 .. TX_64X16 (spec order)
    input  logic [3:0]    tx_type,        // DCT_DCT .. H_FLIPADST (spec order)
    input  logic [3:0]    bit_depth,      // 8, 10 or 12
    input  logic          lossless,
    output logic          busy,
    output logic          done,

    // residual read port (valid after done): addr = i*64 + j
    input  logic [11:0]   res_addr,
    output logic [TW-1:0] res_data,
    output logic [4*TW-1:0] res_data4       // the aligned group of 4 columns containing res_addr
);
    // ---------------------------------------------------------------- tables (spec)
    logic [2:0] log2w, log2h;
    logic [1:0] row_shift_tab;
    always_comb begin
        case (tx_sz)
            5'd0:  begin log2w = 2; log2h = 2; row_shift_tab = 0; end   // 4x4
            5'd1:  begin log2w = 3; log2h = 3; row_shift_tab = 1; end   // 8x8
            5'd2:  begin log2w = 4; log2h = 4; row_shift_tab = 2; end   // 16x16
            5'd3:  begin log2w = 5; log2h = 5; row_shift_tab = 2; end   // 32x32
            5'd4:  begin log2w = 6; log2h = 6; row_shift_tab = 2; end   // 64x64
            5'd5:  begin log2w = 2; log2h = 3; row_shift_tab = 0; end   // 4x8
            5'd6:  begin log2w = 3; log2h = 2; row_shift_tab = 0; end   // 8x4
            5'd7:  begin log2w = 3; log2h = 4; row_shift_tab = 1; end   // 8x16
            5'd8:  begin log2w = 4; log2h = 3; row_shift_tab = 1; end   // 16x8
            5'd9:  begin log2w = 4; log2h = 5; row_shift_tab = 1; end   // 16x32
            5'd10: begin log2w = 5; log2h = 4; row_shift_tab = 1; end   // 32x16
            5'd11: begin log2w = 5; log2h = 6; row_shift_tab = 1; end   // 32x64
            5'd12: begin log2w = 6; log2h = 5; row_shift_tab = 1; end   // 64x32
            5'd13: begin log2w = 2; log2h = 4; row_shift_tab = 1; end   // 4x16
            5'd14: begin log2w = 4; log2h = 2; row_shift_tab = 1; end   // 16x4
            5'd15: begin log2w = 3; log2h = 5; row_shift_tab = 2; end   // 8x32
            5'd16: begin log2w = 5; log2h = 3; row_shift_tab = 2; end   // 32x8
            5'd17: begin log2w = 4; log2h = 6; row_shift_tab = 2; end   // 16x64
            default: begin log2w = 6; log2h = 4; row_shift_tab = 2; end // 64x16
        endcase
    end

    // 1D class per direction: 0 DCT, 1 ADST, 2 identity  (spec 7.13.3 lists)
    logic [1:0] row_cls, col_cls;
    always_comb begin
        case (tx_type)
            4'd0, 4'd1, 4'd4, 4'd11:                              row_cls = 2'd0; // DCT_DCT ADST_DCT FLIPADST_DCT H_DCT
            4'd2, 4'd3, 4'd5, 4'd6, 4'd7, 4'd8, 4'd13, 4'd15:     row_cls = 2'd1; // *ADST rows
            default:                                              row_cls = 2'd2; // IDTX V_DCT V_ADST V_FLIPADST
        endcase
        case (tx_type)
            4'd0, 4'd2, 4'd5, 4'd10:                              col_cls = 2'd0; // DCT_DCT DCT_ADST DCT_FLIPADST V_DCT
            4'd1, 4'd3, 4'd4, 4'd6, 4'd7, 4'd8, 4'd12, 4'd14:     col_cls = 2'd1; // *ADST cols
            default:                                              col_cls = 2'd2; // IDTX H_DCT H_ADST H_FLIPADST
        endcase
    end

    function automatic logic [3:0] prog_id(input logic [1:0] cls, input logic [2:0] nn, input logic ll);
        if (ll) return 4'd12;
        case (cls)
            2'd0:    return 4'(nn - 3'd2);
            2'd1:    return 4'd5 + 4'(nn - 3'd2);
            default: return 4'd8 + 4'(nn - 3'd2);
        endcase
    endfunction

    // ---------------------------------------------------------------- buffers
    logic signed [TW-1:0] coef [32][32];
    logic signed [TW-1:0] res  [64][64];

    always_ff @(posedge clk) begin
        if (coef_clr && !busy) for (int i = 0; i < 32; i++) for (int j = 0; j < 32; j++) coef[i][j] <= '0;
        else if (coef_we && !busy) coef[coef_addr[9:5]][coef_addr[4:0]] <= coef_data;
    end
    assign res_data = res[res_addr[11:6]][res_addr[5:0]];
    always_comb for (int l = 0; l < 4; l++) res_data4[l*TW +: TW] = res[res_addr[11:6]][{res_addr[5:2], 2'(l)}];

    // ---------------------------------------------------------------- latched parameters
    logic [3:0] l_bd;
    logic       l_lossless;
    logic [2:0] l_log2w, l_log2h;
    logic [6:0] w, h;
    logic       rect;                  // |log2W - log2H| == 1
    logic [1:0] l_rowshift;
    logic [4:0] row_clamp, col_clamp;
    assign w = 7'd1 << l_log2w;
    assign h = 7'd1 << l_log2h;
    assign row_clamp = 5'(l_bd) + 5'd8;
    assign col_clamp = (5'(l_bd) + 5'd6 > 5'd16) ? 5'(l_bd) + 5'd6 : 5'd16;

    // ---------------------------------------------------------------- engines (one per lane)
    localparam int NL = NLANES;
    logic             go;                  // registered start pulse for the lanes
    logic [3:0]       e_prog;
    logic [2:0]       e_n;
    logic [4:0]       e_r;
    logic [1:0]       e_wht;
    logic [64*TW-1:0] e_in  [NL];
    logic [64*TW-1:0] e_out [NL];
    logic             e_done [NL];
    logic             e_busy [NL];
    genvar gl;
    generate
        for (gl = 0; gl < NL; gl++) begin : g_lane
            itx1d #(.TW(TW)) u_1d (
                .clk(clk), .rst(rst), .start(go), .prog(e_prog), .n(e_n), .r(e_r), .wht_shift(e_wht),
                .in_vec(e_in[gl]), .busy(e_busy[gl]), .done(e_done[gl]), .out_vec(e_out[gl])
            );
        end
    endgenerate

    typedef enum logic [1:0] { S_IDLE, S_ROW, S_COL, S_DONE } state_t;
    state_t state;
    logic [6:0] idx;                    // first row i (S_ROW) or first column j (S_COL) of the current group
    logic [1:0] row_cls_l, col_cls_l;

    // Lane l carries row idx+l (with the rectangular pre-scale) or column idx+l.
    logic signed [TW-1:0] rowvec [NL][64];
    logic signed [TW-1:0] colvec [NL][64];
    always_comb begin
        for (int l = 0; l < NL; l++) begin
            logic [6:0] i, jj;
            i  = idx + 7'(l);
            jj = idx + 7'(l);
            for (int j = 0; j < 64; j++) begin
                logic signed [TW-1:0]  c;
                logic signed [TW+12:0] p;
                c = (i < 7'd32 && i < h && j < 32 && j < 32'(w)) ? coef[i[4:0]][j[4:0]] : '0;
                p = c * 13'sd2896;
                rowvec[l][j] = rect ? TW'((p + (TW+13)'(2048)) >>> 12) : c;
            end
            for (int k = 0; k < 64; k++) colvec[l][k] = (k < 32'(h) && jj < w) ? res[k[5:0]][jj[5:0]] : '0;
        end
    end

    // Round2 helper with a variable shift (0..4)
    function automatic logic signed [TW-1:0] round2v(input logic signed [TW-1:0] x, input logic [2:0] sh);
        logic signed [TW:0] t;
        if (sh == 3'd0) return x;
        t = (TW+1)'(x) + (TW+1)'(1 <<< (sh - 3'd1));
        return TW'(t >>> sh);
    endfunction

    logic signed [TW-1:0] clip_lo, clip_hi;
    assign clip_lo = -(TW'(1) <<< (col_clamp - 5'd1));
    assign clip_hi =  (TW'(1) <<< (col_clamp - 5'd1)) - TW'(1);

    always_comb begin
        e_prog = 4'd0; e_n = 3'd2; e_r = 5'd16; e_wht = 2'd0;
        for (int l = 0; l < NL; l++) e_in[l] = '0;
        case (state)
            S_ROW: begin
                e_prog = prog_id(row_cls_l, l_log2w, l_lossless);
                e_n    = l_log2w;
                e_r    = row_clamp;
                e_wht  = 2'd2;
                for (int l = 0; l < NL; l++)
                    for (int j = 0; j < 64; j++) e_in[l][j*TW +: TW] = rowvec[l][j];
            end
            S_COL: begin
                e_prog = prog_id(col_cls_l, l_log2h, l_lossless);
                e_n    = l_log2h;
                e_r    = col_clamp;
                e_wht  = 2'd0;
                for (int l = 0; l < NL; l++)
                    for (int i = 0; i < 64; i++) e_in[l][i*TW +: TW] = colvec[l][i];
            end
            default: ;
        endcase
    end

    always_ff @(posedge clk) begin
        done <= 1'b0;
        go   <= 1'b0;
        if (rst) begin
            state <= S_IDLE;
            busy  <= 1'b0;
            idx   <= '0;
        end else begin
            case (state)
                S_IDLE: if (start) begin
                    l_bd       <= bit_depth;
                    l_lossless <= lossless;
                    l_log2w    <= log2w;
                    l_log2h    <= log2h;
                    rect       <= (log2w > log2h) ? (log2w - log2h == 3'd1) : (log2h - log2w == 3'd1);
                    l_rowshift <= lossless ? 2'd0 : row_shift_tab;
                    row_cls_l  <= row_cls;
                    col_cls_l  <= col_cls;
                    idx        <= '0;
                    busy       <= 1'b1;
                    go         <= 1'b1;
                    state      <= S_ROW;
                end
                // ---- rows: a group of NL rows per pass; groups at i >= 32 are zero and skip the engines
                S_ROW: begin
                    if (e_done[0]) begin
                        for (int l = 0; l < NL; l++) begin
                            logic [6:0] i;
                            i = idx + 7'(l);
                            if (i < h)
                                for (int j = 0; j < 64; j++) begin
                                    logic signed [TW-1:0] v;
                                    v = round2v(e_out[l][j*TW +: TW], 3'(l_rowshift));
                                    v = (v < clip_lo) ? clip_lo : (v > clip_hi) ? clip_hi : v;
                                    if (j < 32'(w)) res[i[5:0]][j] <= v;
                                end
                        end
                        if (idx + 7'(NL) >= h) begin idx <= '0; go <= 1'b1; state <= S_COL; end
                        else begin idx <= idx + 7'(NL); go <= (idx + 7'(NL) < 7'd32); end
                    end else if (!go && !e_busy[0] && idx >= 7'd32) begin
                        for (int l = 0; l < NL; l++) begin
                            logic [6:0] i;
                            i = idx + 7'(l);
                            if (i < h) for (int j = 0; j < 64; j++) res[i[5:0]][j] <= '0;
                        end
                        if (idx + 7'(NL) >= h) begin idx <= '0; go <= 1'b1; state <= S_COL; end
                        else idx <= idx + 7'(NL);
                    end
                end
                // ---- columns: a group of NL columns per pass
                S_COL: if (e_done[0]) begin
                    for (int l = 0; l < NL; l++) begin
                        logic [6:0] jj;
                        jj = idx + 7'(l);
                        if (jj < w)
                            for (int i = 0; i < 64; i++)
                                if (i < 32'(h)) res[i[5:0]][jj[5:0]] <= round2v(e_out[l][i*TW +: TW], l_lossless ? 3'd0 : 3'd4);
                    end
                    if (idx + 7'(NL) >= w) state <= S_DONE;
                    else begin idx <= idx + 7'(NL); go <= 1'b1; end
                end
                S_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
