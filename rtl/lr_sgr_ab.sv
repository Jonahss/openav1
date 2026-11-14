// Self-guided loop restoration (spec 7.17.3) for the streaming filter chain: a2 and b2 of one box position from its
// box sums, four clocks (lr_top's arithmetic lifted out).
//   box_a = the sum of the squares, box_b = the sum of the 25 (r1 = 0: radius 2) or 9 (r1 = 1) samples;
//   s = the pass's value from lr_pkg::sgr_s. They travel with the position.
module lr_sgr_ab
  import lr_pkg::*;
(
    input  logic        clk,
    input  logic        rst,
    input  logic        v_in,
    input  logic [3:0]  bd,
    input  logic        r1,
    input  logic [11:0] s,
    input  logic [29:0] box_a,
    input  logic [17:0] box_b,
    output logic        v_out,
    output logic [8:0]  a2,
    output logic [21:0] b2
);
    logic        v1, v2, v3;
    logic        q1_r1, q2_r1, q3_r1;
    logic [11:0] q1_s;
    logic [36:0] q1_p;
    logic [17:0] q1_b, q2_b;
    logic [28:0] q2_z;
    logic [8:0]  q3_a2;
    logic [26:0] q3_m1;
    logic [4:0]  sh2;
    logic [3:0]  sh1;
    assign sh1 = bd - 4'd8;
    assign sh2 = {sh1, 1'b0};
    always_ff @(posedge clk) begin
        v1 <= v_in && !rst; v2 <= v1 && !rst; v3 <= v2 && !rst; v_out <= v3 && !rst;
        // p = max(0, a * n - d * d) of the sums rounded to 8 bits
        begin
            logic [29:0] a_r; logic [17:0] d_r; logic [35:0] an, dd;
            a_r = (bd == 4'd8) ? box_a : 30'((box_a + ((30'd1 << sh2) >> 1)) >> sh2);
            d_r = (bd == 4'd8) ? box_b : 18'((box_b + ((18'd1 << sh1) >> 1)) >> sh1);
            an = 36'(a_r) * (r1 ? 36'd9 : 36'd25);
            dd = 36'(d_r) * 36'(d_r);
            q1_p <= (an > dd) ? 37'(an - dd) : 37'd0;
        end
        q1_b <= box_b; q1_s <= s; q1_r1 <= r1;
        // z = (p * s + 2^19) >> 20
        begin
            logic [48:0] pz;
            pz = 49'(q1_p) * 49'(q1_s);
            q2_z <= 29'((pz + (49'd1 << 19)) >> 20);
        end
        q2_b <= q1_b; q2_r1 <= q1_r1;
        // a2 from the table, (256 - a2) * b
        begin
            logic [8:0] a;
            if (q2_z >= 29'd255) a = 9'd256;
            else if (q2_z == 29'd0) a = 9'd1;                      // (the table's default gives 1 as well)
            else a = sgr_a2(8'(q2_z));
            q3_a2 <= a;
            q3_m1 <= 27'(9'd256 - a) * 27'(q2_b);
        end
        q3_r1 <= q2_r1;
        // b2 = ((256 - a2) * b * oneOverN + 2^11) >> 12
        begin
            logic [42:0] m;
            m = 43'(q3_m1) * 43'(sgr_one_over_n(q3_r1 ? 2'd1 : 2'd2));
            b2 <= 22'((m + (43'd1 << 11)) >> 12);
        end
        a2 <= q3_a2;
    end
endmodule
