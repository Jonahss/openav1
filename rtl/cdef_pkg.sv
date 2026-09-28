// CDEF constants (spec 7.15): Cdef_Directions, Cdef_Uv_Dir, Div_Table, Cdef_Pri_Taps, Cdef_Sec_Taps.
package cdef_pkg;
    // Cdef_Directions[dir][k] = {dy, dx} (signed 3 bits each)
    function automatic logic signed [2:0] cdef_dy(input logic [2:0] dir, input logic k);
        case ({dir, k})
            4'b000_0: cdef_dy = -3'sd1; 4'b000_1: cdef_dy = -3'sd2;
            4'b001_0: cdef_dy =  3'sd0; 4'b001_1: cdef_dy = -3'sd1;
            4'b010_0: cdef_dy =  3'sd0; 4'b010_1: cdef_dy =  3'sd0;
            4'b011_0: cdef_dy =  3'sd0; 4'b011_1: cdef_dy =  3'sd1;
            4'b100_0: cdef_dy =  3'sd1; 4'b100_1: cdef_dy =  3'sd2;
            4'b101_0: cdef_dy =  3'sd1; 4'b101_1: cdef_dy =  3'sd2;
            4'b110_0: cdef_dy =  3'sd1; 4'b110_1: cdef_dy =  3'sd2;
            default:  cdef_dy =  {dir, k} == 4'b111_0 ? 3'sd1 : 3'sd2;
        endcase
    endfunction
    function automatic logic signed [2:0] cdef_dx(input logic [2:0] dir, input logic k);
        case ({dir, k})
            4'b000_0: cdef_dx =  3'sd1; 4'b000_1: cdef_dx =  3'sd2;
            4'b001_0: cdef_dx =  3'sd1; 4'b001_1: cdef_dx =  3'sd2;
            4'b010_0: cdef_dx =  3'sd1; 4'b010_1: cdef_dx =  3'sd2;
            4'b011_0: cdef_dx =  3'sd1; 4'b011_1: cdef_dx =  3'sd2;
            4'b100_0: cdef_dx =  3'sd1; 4'b100_1: cdef_dx =  3'sd2;
            4'b101_0: cdef_dx =  3'sd0; 4'b101_1: cdef_dx =  3'sd1;
            4'b110_0: cdef_dx =  3'sd0; 4'b110_1: cdef_dx =  3'sd0;
            default:  cdef_dx =  {dir, k} == 4'b111_0 ? 3'sd0 : -3'sd1;
        endcase
    endfunction
    // Cdef_Uv_Dir[subX][subY][dir]
    function automatic logic [2:0] cdef_uv_dir(input logic sx, input logic sy, input logic [2:0] dir);
        case ({sx, sy})
            2'b00, 2'b11: cdef_uv_dir = dir;
            2'b01: case (dir) 3'd0: cdef_uv_dir = 3'd1; 3'd1: cdef_uv_dir = 3'd2; 3'd2: cdef_uv_dir = 3'd2; 3'd3: cdef_uv_dir = 3'd2;
                              3'd4: cdef_uv_dir = 3'd3; 3'd5: cdef_uv_dir = 3'd4; 3'd6: cdef_uv_dir = 3'd6; default: cdef_uv_dir = 3'd0; endcase
            default: case (dir) 3'd0: cdef_uv_dir = 3'd7; 3'd1: cdef_uv_dir = 3'd0; 3'd2: cdef_uv_dir = 3'd2; 3'd3: cdef_uv_dir = 3'd4;
                                3'd4: cdef_uv_dir = 3'd5; 3'd5: cdef_uv_dir = 3'd6; 3'd6: cdef_uv_dir = 3'd6; default: cdef_uv_dir = 3'd6; endcase
        endcase
    endfunction
    // Div_Table[0..8]
    function automatic logic [9:0] div_table(input logic [3:0] i);
        case (i)
            4'd1: div_table = 10'd840; 4'd2: div_table = 10'd420; 4'd3: div_table = 10'd280; 4'd4: div_table = 10'd210;
            4'd5: div_table = 10'd168; 4'd6: div_table = 10'd140; 4'd7: div_table = 10'd120; 4'd8: div_table = 10'd105;
            default: div_table = 10'd0;
        endcase
    endfunction
    // Cdef_Pri_Taps[(priStr >> coeffShift) & 1][k], Cdef_Sec_Taps[..][k]
    function automatic logic [2:0] pri_tap(input logic odd, input logic k);
        pri_tap = odd ? 3'd3 : (k ? 3'd2 : 3'd4);
    endfunction
    function automatic logic [2:0] sec_tap(input logic odd, input logic k);
        sec_tap = k ? 3'd1 : 3'd2;
    endfunction
endpackage
