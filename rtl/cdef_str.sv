// CDEF block parameters (spec 7.15.1 / 7.15.2) for one plane of an 8x8 block, combinational: cdef_top's C_DIR step
// lifted out for the streaming CDEF stage (docs/filter-chain.md). From the frame's strengths, the block's cdef_idx
// and the luma direction and variance (rtl/cdef_dir.sv) to what the line filter (rtl/cdef_row.sv) takes.
//   luma : the plane is luma (variance-adjusted primary strength, the luma direction, the frame's damping);
//          chroma takes Cdef_Uv_Dir of the luma direction, the strength as coded and the damping less one
//   dr = 0 when the primary strength as coded is 0; pri is 0 when the variance is 0 (luma)
//   adjp / adjs : the constrain() shifts max(0, damping - FloorLog2(strength)); priodd = (pri >> cs) & 1
module cdef_str
  import cdef_pkg::*;
  import syn_pkg::*;
(
    input  cdef_hdr_t   ch,
    input  logic [3:0]  cs,            // bit_depth - 8
    input  logic        ssx, ssy,
    input  logic [2:0]  idx,           // the block's cdef_idx
    input  logic        luma,
    input  logic [2:0]  ydir,
    input  logic [31:0] var_i,
    output logic [2:0]  dr,
    output logic [11:0] pri, sec,
    output logic [4:0]  adjp, adjs,
    output logic        priodd
);
    function automatic logic [3:0] floor_log2_12(input logic [11:0] v);
        floor_log2_12 = 4'd0;
        for (int i = 11; i >= 0; i--) if (floor_log2_12 == 4'd0 && v[i]) floor_log2_12 = 4'(i);
    endfunction
    function automatic logic [4:0] constrain_adj(input logic [11:0] thr, input logic [4:0] damp);
        constrain_adj = (damp > 5'(floor_log2_12(thr))) ? damp - 5'(floor_log2_12(thr)) : 5'd0;
    endfunction
    logic [4:0] damping;
    always_comb begin
        logic [5:0] sv; logic [3:0] p4, s4; logic [11:0] ps; logic [3:0] var_str; logic [31:0] v6;
        sv = luma ? ch.y_str[6 * idx +: 6] : ch.uv_str[6 * idx +: 6];
        p4 = 4'(sv >> 2); s4 = 4'(sv[1:0]) + ((sv[1:0] == 2'd3) ? 4'd1 : 4'd0);
        ps = 12'(p4) << cs;
        v6 = var_i >> 6;
        var_str = 4'd0;
        if (v6 != 32'd0) begin
            for (int b = 31; b >= 0; b--) if (var_str == 4'd0 && v6[b]) var_str = (b > 12) ? 4'd12 : 4'(b);
        end
        if (luma) begin
            dr = (ps == 12'd0) ? 3'd0 : ydir;
            pri = (var_i != 32'd0) ? 12'((16'(ps) * (16'd4 + 16'(var_str)) + 16'd8) >> 4) : 12'd0;
            damping = 5'(ch.damping) + 5'(cs);
        end else begin
            dr = (ps == 12'd0) ? 3'd0 : cdef_uv_dir(ssx, ssy, ydir);
            pri = ps;
            damping = 5'(ch.damping) + 5'(cs) - 5'd1;
        end
        sec = 12'(s4) << cs;
        adjp = constrain_adj(pri, damping);
        adjs = constrain_adj(sec, damping);
        priodd = pri[cs];
    end
endmodule
