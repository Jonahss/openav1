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
// v1 keeps Dequant and Residual in register arrays for simulation; a real implementation puts them
// in a transpose memory. Flips (FLIPADST) are applied by the reconstruction stage, not here.
module itx2d #(
    parameter int TW = 20
) (
    input  logic          clk,
    input  logic          rst,

    // coefficient write port (valid while idle): addr = i*32 + j, i,j < 32
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
    output logic [TW-1:0] res_data
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

    always_ff @(posedge clk) if (coef_we && !busy) coef[coef_addr[9:5]][coef_addr[4:0]] <= coef_data;
    assign res_data = res[res_addr[11:6]][res_addr[5:0]];

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

    // ---------------------------------------------------------------- engine
    logic             e_start, e_done;
    /* verilator lint_off UNUSEDSIGNAL */
    logic             e_busy;
    /* verilator lint_on UNUSEDSIGNAL */
    logic [3:0]       e_prog;
    logic [2:0]       e_n;
    logic [4:0]       e_r;
    logic [1:0]       e_wht;
    logic [64*TW-1:0] e_in, e_out;
    itx1d #(.TW(TW)) u_1d (
        .clk(clk), .rst(rst), .start(e_start), .prog(e_prog), .n(e_n), .r(e_r), .wht_shift(e_wht),
        .in_vec(e_in), .busy(e_busy), .done(e_done), .out_vec(e_out)
    );

    typedef enum logic [2:0] { S_IDLE, S_ROW_START, S_ROW_WAIT, S_COL_START, S_COL_WAIT, S_DONE } state_t;
    state_t state;
    logic [6:0] idx;                    // current row i or column j

    // Row vector assembly (with the rectangular pre-scale) and column vector assembly.
    logic signed [TW-1:0] rowvec [64];
    logic signed [TW-1:0] colvec [64];
    always_comb begin
        for (int j = 0; j < 64; j++) begin
            logic signed [TW-1:0]  c;
            logic signed [TW+12:0] p;
            c = (idx < 7'd32 && j < 32 && j < 32'(w)) ? coef[idx[4:0]][j[4:0]] : '0;
            p = c * 13'sd2896;
            rowvec[j] = rect ? TW'((p + (TW+13)'(2048)) >>> 12) : c;
        end
        for (int i = 0; i < 64; i++) colvec[i] = (i < 32'(h)) ? res[i[5:0]][idx[5:0]] : '0;
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
        e_start = 1'b0;
        e_prog  = 4'd0;
        e_n     = 3'd2;
        e_r     = 5'd16;
        e_wht   = 2'd0;
        e_in    = '0;
        case (state)
            S_ROW_START: begin
                e_start = (idx < 7'd32);            // rows >= 32 are all-zero and skipped
                e_prog  = prog_id(row_cls_l, l_log2w, l_lossless);
                e_n     = l_log2w;
                e_r     = row_clamp;
                e_wht   = 2'd2;
                for (int j = 0; j < 64; j++) e_in[j*TW +: TW] = rowvec[j];
            end
            S_COL_START: begin
                e_start = 1'b1;
                e_prog  = prog_id(col_cls_l, l_log2h, l_lossless);
                e_n     = l_log2h;
                e_r     = col_clamp;
                e_wht   = 2'd0;
                for (int i = 0; i < 64; i++) e_in[i*TW +: TW] = colvec[i];
            end
            default: ;
        endcase
    end

    logic [1:0] row_cls_l, col_cls_l;

    always_ff @(posedge clk) begin
        done <= 1'b0;
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
                    state      <= S_ROW_START;
                end
                S_ROW_START: begin
                    if (idx >= 7'd32) begin
                        // all-zero row: transform of zero is zero
                        for (int j = 0; j < 64; j++) res[idx[5:0]][j] <= '0;
                        if (idx + 7'd1 == h) begin idx <= '0; state <= S_COL_START; end
                        else idx <= idx + 7'd1;
                    end else
                        state <= S_ROW_WAIT;
                end
                S_ROW_WAIT: if (e_done) begin
                    for (int j = 0; j < 64; j++) begin
                        logic signed [TW-1:0] v;
                        v = round2v(e_out[j*TW +: TW], 3'(l_rowshift));
                        v = (v < clip_lo) ? clip_lo : (v > clip_hi) ? clip_hi : v;
                        if (j < 32'(w)) res[idx[5:0]][j] <= v;
                    end
                    if (idx + 7'd1 == h) begin idx <= '0; state <= S_COL_START; end
                    else begin idx <= idx + 7'd1; state <= S_ROW_START; end
                end
                S_COL_START: state <= S_COL_WAIT;
                S_COL_WAIT: if (e_done) begin
                    for (int i = 0; i < 64; i++)
                        if (i < 32'(h)) res[i[5:0]][idx[5:0]] <= round2v(e_out[i*TW +: TW], l_lossless ? 3'd0 : 3'd4);
                    if (idx + 7'd1 == w) state <= S_DONE;
                    else begin idx <= idx + 7'd1; state <= S_COL_START; end
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
