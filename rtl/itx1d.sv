// openav1 — 1D inverse transform engine (AV1 spec 7.13.2).
//
// A microcoded butterfly machine. The transform vector T lives in 64 registers; each clock it
// executes one row of the ROM in rtl/itx_ucode.sv, which tools/gen_itx_ucode.py derives from the
// spec model (tb/itx_model.py). A row is either:
//   * up to ITX_NSLOTS independent pair operations from one spec step-line: B (rotation by
//     cos128/sin128, Round2 12, optional swap) or H (Hadamard with clamp to r bits),
//   * a whole-vector permutation (DCT bit-reversal, ADST input / output permutations),
//   * one of the dedicated single-cycle transforms (ADST4, identity, WHT),
//   * end.
// Start latches the input vector and the program; done pulses with the result in out_vec.
// Cycle count = ROM rows for the program (+1): DCT4 5, DCT8 7, DCT16 14, DCT32 29, DCT64 65,
// ADST8 9, ADST16 17, ADST4/identity/WHT 3 — at NSLOTS=4. Widen NSLOTS to trade area for speed.
module itx1d #(
    parameter int TW = 20                   // element width (bits): BitDepth + 10 covers every intermediate (20 for 8/10-bit, 22 for 12-bit)
) (
    input  logic                 clk,
    input  logic                 rst,
    input  logic                 start,     // pulse; ignored while busy
    input  logic [3:0]           prog,      // program id: DCT n=2..6 -> 0..4, ADST n=2..4 -> 5..7, IDTX n=2..5 -> 8..11, WHT -> 12
    input  logic [2:0]           n,         // log2 vector length
    input  logic [4:0]           r,         // intermediate clamp range (bits)
    input  logic [1:0]           wht_shift, // WHT pre-shift (2 for rows, 0 for columns)
    input  logic [64*TW-1:0]     in_vec,    // element i at [i*TW +: TW], two's complement; zero beyond 1<<n
    output logic                 busy,
    output logic                 done,      // one-cycle pulse; out_vec valid from this cycle on
    output logic [64*TW-1:0]     out_vec
);
    localparam int NS = itx_ucode_pkg::ITX_NSLOTS;

    // ------------------------------------------------------------------ state
    logic signed [TW-1:0] T [64];
    logic [7:0]           pc;
    logic [2:0]           l_n;        // parameters latched at start
    logic [4:0]           l_r;
    logic [1:0]           l_wht;

    logic [itx_ucode_pkg::ITX_ROW_W-1:0] row;
    logic [7:0]           prog_addr;
    itx_ucode_rom  u_rom   (.addr(pc), .row(row));
    itx_prog_start u_start (.pid(prog), .addr(prog_addr));
    wire [3:0] kind = row[3:0];
    localparam logic [3:0] K_OPS = 4'd1, K_PERM_DCT = 4'd2, K_PERM_ADST_IN = 4'd3, K_PERM_ADST_OUT = 4'd4,
                           K_ADST4 = 4'd5, K_IDENT = 4'd6, K_WHT = 4'd7, K_END = 4'd8;

    // ------------------------------------------------------------------ slot decode
    logic [1:0] s_op   [NS];
    logic [5:0] s_a    [NS];
    logic [5:0] s_b    [NS];
    logic [7:0] s_ang  [NS];
    logic       s_flip [NS];
    always_comb begin
        for (int s = 0; s < NS; s++) begin
            s_op[s]   = row[4 + s*23 + 21 +: 2];
            s_a[s]    = row[4 + s*23 + 15 +: 6];
            s_b[s]    = row[4 + s*23 + 9  +: 6];
            s_ang[s]  = row[4 + s*23 + 1  +: 8];
            s_flip[s] = row[4 + s*23];
        end
    end

    // ------------------------------------------------------------------ butterfly units
    logic signed [12:0] s_cos [NS];
    logic signed [12:0] s_sin [NS];
    logic signed [TW-1:0] wv_a [NS];      // value written to index a (after flip handling)
    logic signed [TW-1:0] wv_b [NS];
    logic signed [TW-1:0] clip_lo, clip_hi;
    assign clip_lo = -(TW'(1) <<< (l_r - 5'd1));
    assign clip_hi =  (TW'(1) <<< (l_r - 5'd1)) - TW'(1);

    genvar gs;
    generate
        for (gs = 0; gs < NS; gs++) begin : g_slot
            cos128_lut u_cos (.angle(s_ang[gs]),          .val(s_cos[gs]));
            cos128_lut u_sin (.angle(s_ang[gs] - 8'd64),  .val(s_sin[gs]));
        end
    endgenerate

    /* verilator lint_off UNUSEDSIGNAL */
    always_comb begin
        for (int s = 0; s < NS; s++) begin
            logic signed [TW-1:0]    ta, tb;
            logic signed [TW+13:0]   x, y;          // TW x 13-bit products, summed
            logic signed [TW+13:0]   rx, ry;
            logic signed [TW:0]      sum, dif;
            logic signed [TW-1:0]    csum, cdif;
            ta = T[s_a[s]];
            tb = T[s_b[s]];
            // B: rotation
            x  = ta * s_cos[s] - tb * s_sin[s];
            y  = ta * s_sin[s] + tb * s_cos[s];
            rx = (x + (TW+14)'(2048)) >>> 12;
            ry = (y + (TW+14)'(2048)) >>> 12;
            // H: Hadamard with clamp (flip swaps the roles of a and b)
            begin
                logic signed [TW:0] ha, hb;
                ha  = (TW+1)'(s_flip[s] ? tb : ta);
                hb  = (TW+1)'(s_flip[s] ? ta : tb);
                sum = ha + hb;
                dif = ha - hb;
            end
            csum = (sum < (TW+1)'(clip_lo)) ? clip_lo : (sum > (TW+1)'(clip_hi)) ? clip_hi : TW'(sum);
            cdif = (dif < (TW+1)'(clip_lo)) ? clip_lo : (dif > (TW+1)'(clip_hi)) ? clip_hi : TW'(dif);
            if (s_op[s] == 2'd1) begin              // B
                wv_a[s] = s_flip[s] ? TW'(ry) : TW'(rx);
                wv_b[s] = s_flip[s] ? TW'(rx) : TW'(ry);
            end else begin                          // H: T[aa] = clip(x+y), T[bb] = clip(x-y)
                wv_a[s] = s_flip[s] ? cdif : csum;  // index a is "bb" when flipped
                wv_b[s] = s_flip[s] ? csum : cdif;
            end
        end
    end
    /* verilator lint_on UNUSEDSIGNAL */

    // ------------------------------------------------------------------ permutation index functions (spec 7.13.2.2 / .4 / .5)
    function automatic int perm_dct_idx(input int i, input int nn);
        int t;
        t = 0;
        for (int b = 0; b < nn; b++) t = t | (((i >> b) & 1) << (nn - 1 - b));
        if (i < (1 << nn)) perm_dct_idx = t; else perm_dct_idx = i;
    endfunction
    function automatic int perm_adst_in_idx(input int i, input int nn);
        int n0;
        n0 = 1 << nn;
        if (i >= n0)            perm_adst_in_idx = i;
        else if ((i & 1) != 0)  perm_adst_in_idx = i - 1;
        else                    perm_adst_in_idx = n0 - i - 1;
    endfunction
    function automatic int perm_adst_out_idx(input int i, input int nn);
        int a, b, c, d;
        a = (i >> 3) & 1;
        b = ((i >> 2) & 1) ^ ((i >> 3) & 1);
        c = ((i >> 1) & 1) ^ ((i >> 2) & 1);
        d = (i & 1) ^ ((i >> 1) & 1);
        if (i >= (1 << nn)) perm_adst_out_idx = i;
        else                perm_adst_out_idx = ((d << 3) | (c << 2) | (b << 1) | a) >> (4 - nn);
    endfunction

    // ------------------------------------------------------------------ whole-vector operations
    logic signed [TW-1:0] T_ops   [64];
    logic signed [TW-1:0] T_perm  [64];
    logic signed [TW-1:0] T_ident [64];
    logic signed [TW-1:0] T_adst4 [4];
    logic signed [TW-1:0] T_wht   [4];

    always_comb begin
        // pair ops write-back
        for (int i = 0; i < 64; i++) begin
            T_ops[i] = T[i];
            for (int s = 0; s < NS; s++) begin
                if (s_op[s] != 2'd0) begin
                    if (6'(i) == s_a[s]) T_ops[i] = wv_a[s];
                    if (6'(i) == s_b[s]) T_ops[i] = wv_b[s];
                end
            end
        end
        // permutations. The source index depends only on the lane and on l_n (5 values), so each
        // lane is a 5:1 mux of fixed registers, not a 64:1 crossbar. Lanes >= 1<<n are don't-care.
        for (int i = 0; i < 64; i++) begin
            T_perm[i] = T[i];
            case (kind)
                K_PERM_DCT: case (l_n)
                    3'd2: T_perm[i] = T[perm_dct_idx(i, 2)];
                    3'd3: T_perm[i] = T[perm_dct_idx(i, 3)];
                    3'd4: T_perm[i] = T[perm_dct_idx(i, 4)];
                    3'd5: T_perm[i] = T[perm_dct_idx(i, 5)];
                    default: T_perm[i] = T[perm_dct_idx(i, 6)];
                endcase
                K_PERM_ADST_IN: case (l_n)
                    3'd3: T_perm[i] = T[perm_adst_in_idx(i, 3)];
                    default: T_perm[i] = T[perm_adst_in_idx(i, 4)];
                endcase
                K_PERM_ADST_OUT: case (l_n)
                    3'd3: T_perm[i] = ((i & 1) != 0) ? -T[perm_adst_out_idx(i, 3)] : T[perm_adst_out_idx(i, 3)];
                    default: T_perm[i] = ((i & 1) != 0) ? -T[perm_adst_out_idx(i, 4)] : T[perm_adst_out_idx(i, 4)];
                endcase
                default: ;
            endcase
        end
        // identity (size-dependent scaling): 4: Round2(T*5793,12)  8: T*2  16: Round2(T*11586,12) = Round2(T*5793,11)  32: T*4
        for (int i = 0; i < 64; i++) begin
            logic signed [TW+13:0] p1;
            p1 = T[i] * 14'sd5793;          // signed; a ternary with '0 would turn it unsigned
            if (i >= 32) p1 = '0;
            case (l_n)
                3'd2:    T_ident[i] = TW'((p1 + (TW+14)'(2048)) >>> 12);
                3'd3:    T_ident[i] = T[i] <<< 1;
                3'd4:    T_ident[i] = TW'((p1 + (TW+14)'(1024)) >>> 11);
                default: T_ident[i] = T[i] <<< 2;
            endcase
        end
        // ADST4 (spec 7.13.2.6)
        begin
            logic signed [TW+13:0] s0, s1, s2, s3, s4, s5, s6, x0, x1, x2, x3;
            logic signed [TW+1:0]  a7, b7;
            s0 = T[0] * 13'sd1321;
            s1 = T[0] * 13'sd2482;
            s2 = T[1] * 13'sd3344;
            s3 = T[2] * 13'sd3803;
            s4 = T[2] * 13'sd1321;
            s5 = T[3] * 13'sd2482;
            s6 = T[3] * 13'sd3803;
            a7 = (TW+2)'(T[0]) - (TW+2)'(T[2]);
            b7 = a7 + (TW+2)'(T[3]);
            s0 = s0 + s3;
            s1 = s1 - s4;
            s3 = s2;
            s2 = b7 * 13'sd3344;
            s0 = s0 + s5;
            s1 = s1 - s6;
            x0 = s0 + s3;
            x1 = s1 + s3;
            x2 = s2;
            x3 = s0 + s1 - s3;
            T_adst4[0] = TW'((x0 + (TW+14)'(2048)) >>> 12);
            T_adst4[1] = TW'((x1 + (TW+14)'(2048)) >>> 12);
            T_adst4[2] = TW'((x2 + (TW+14)'(2048)) >>> 12);
            T_adst4[3] = TW'((x3 + (TW+14)'(2048)) >>> 12);
        end
        // WHT (spec 7.13.2.10)
        begin
            logic signed [TW-1:0] a, b, c, d, e;
            a = T[0] >>> l_wht;
            c = T[1] >>> l_wht;
            d = T[2] >>> l_wht;
            b = T[3] >>> l_wht;
            a = a + c;
            d = d - b;
            e = (a - d) >>> 1;
            b = e - b;
            c = e - c;
            a = a - b;
            d = d + c;
            T_wht[0] = a; T_wht[1] = b; T_wht[2] = c; T_wht[3] = d;
        end
    end

    // ------------------------------------------------------------------ sequencing
    always_ff @(posedge clk) begin
        done <= 1'b0;
        if (rst) begin
            busy <= 1'b0;
            pc   <= 8'd0;
        end else if (!busy) begin
            if (start) begin
                for (int i = 0; i < 64; i++) T[i] <= in_vec[i*TW +: TW];
                pc    <= prog_addr;
                l_n   <= n;
                l_r   <= r;
                l_wht <= wht_shift;
                busy  <= 1'b1;
            end
        end else begin
            pc <= pc + 8'd1;
            case (kind)
                K_OPS:           for (int i = 0; i < 64; i++) T[i] <= T_ops[i];
                K_PERM_DCT, K_PERM_ADST_IN, K_PERM_ADST_OUT:
                                 for (int i = 0; i < 64; i++) T[i] <= T_perm[i];
                K_IDENT:         for (int i = 0; i < 64; i++) T[i] <= T_ident[i];
                K_ADST4:         for (int i = 0; i < 4;  i++) T[i] <= T_adst4[i];
                K_WHT:           for (int i = 0; i < 4;  i++) T[i] <= T_wht[i];
                K_END: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end
                default: ;
            endcase
        end
    end

    always_comb for (int i = 0; i < 64; i++) out_vec[i*TW +: TW] = T[i];
endmodule
