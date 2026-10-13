// openav1 — the one-row 1D transforms that are not butterfly networks: ADST4 (spec 7.13.2.6), identity
// (7.13.2.15) and WHT (7.13.2.10). Combinational on one row (lanes 0 .. N-1 of in_vec); itx2d streams rows
// through it one per clock, while the DCT / ADST8 / ADST16 rows go through the level engine (itx_lvl).
// Same arithmetic as the dedicated single-cycle ops of the microcoded itx1d. Lanes beyond the transform are 0.
module itx_row #(
    parameter int TW = 20
) (
    input  logic [1:0]       kind,       // 0 ADST4, 1 identity, 2 WHT
    input  logic [2:0]       n,          // log2 length (identity 2..5)
    input  logic [1:0]       wht_shift,  // 2 for rows, 0 for columns
    input  logic [32*TW-1:0] in_vec,    // lanes 0 .. 31 (the longest of these transforms is identity 32)
    output logic [32*TW-1:0] out_vec
);
    logic signed [TW-1:0] T [32];
    always_comb for (int i = 0; i < 32; i++) T[i] = in_vec[i*TW +: TW];

    logic signed [TW-1:0] T_ident [32];
    logic signed [TW-1:0] T_adst4 [4];
    logic signed [TW-1:0] T_wht   [4];
    always_comb begin
        // identity (size-dependent scaling): 4: Round2(T*5793,12)  8: T*2  16: Round2(T*11586,12) = Round2(T*5793,11)  32: T*4
        for (int i = 0; i < 32; i++) begin
            logic signed [TW+13:0] p1;
            p1 = T[i] * 14'sd5793;
            case (n)
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
            a = T[0] >>> wht_shift;
            c = T[1] >>> wht_shift;
            d = T[2] >>> wht_shift;
            b = T[3] >>> wht_shift;
            a = a + c;
            d = d - b;
            e = (a - d) >>> 1;
            b = e - b;
            c = e - c;
            a = a - b;
            d = d + c;
            T_wht[0] = a; T_wht[1] = b; T_wht[2] = c; T_wht[3] = d;
        end
        out_vec = '0;
        case (kind)
            2'd0:    for (int i = 0; i < 4;  i++) out_vec[i*TW +: TW] = T_adst4[i];
            2'd1:    for (int i = 0; i < 32; i++) if (i < (32'd1 << n)) out_vec[i*TW +: TW] = T_ident[i];
            default: for (int i = 0; i < 4;  i++) out_vec[i*TW +: TW] = T_wht[i];
        endcase
    end
endmodule
