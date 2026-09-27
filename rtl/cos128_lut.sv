// cos128(angle) = round(4096 * cos(angle * pi / 128)), spec 7.13.2.1, for any 8-bit angle.
module cos128_lut (
    input  logic [7:0]         angle,
    output logic signed [12:0] val
);
    logic [6:0]         idx;     // 0..64
    logic               neg;
    logic signed [12:0] mag;
    always_comb begin
        // quadrant fold
        if (angle <= 8'd64)       begin idx = 7'(angle);          neg = 1'b0; end
        else if (angle <= 8'd128) begin idx = 7'(8'd128 - angle); neg = 1'b1; end
        else if (angle <= 8'd192) begin idx = 7'(angle - 8'd128); neg = 1'b1; end
        else                      begin idx = 7'(9'd256 - angle); neg = 1'b0; end
        case (idx)
            7'd0: mag = 4096;  7'd1: mag = 4095;  7'd2: mag = 4091;  7'd3: mag = 4085;
            7'd4: mag = 4076;  7'd5: mag = 4065;  7'd6: mag = 4052;  7'd7: mag = 4036;
            7'd8: mag = 4017;  7'd9: mag = 3996;  7'd10: mag = 3973; 7'd11: mag = 3948;
            7'd12: mag = 3920; 7'd13: mag = 3889; 7'd14: mag = 3857; 7'd15: mag = 3822;
            7'd16: mag = 3784; 7'd17: mag = 3745; 7'd18: mag = 3703; 7'd19: mag = 3659;
            7'd20: mag = 3612; 7'd21: mag = 3564; 7'd22: mag = 3513; 7'd23: mag = 3461;
            7'd24: mag = 3406; 7'd25: mag = 3349; 7'd26: mag = 3290; 7'd27: mag = 3229;
            7'd28: mag = 3166; 7'd29: mag = 3102; 7'd30: mag = 3035; 7'd31: mag = 2967;
            7'd32: mag = 2896; 7'd33: mag = 2824; 7'd34: mag = 2751; 7'd35: mag = 2675;
            7'd36: mag = 2598; 7'd37: mag = 2520; 7'd38: mag = 2440; 7'd39: mag = 2359;
            7'd40: mag = 2276; 7'd41: mag = 2191; 7'd42: mag = 2106; 7'd43: mag = 2019;
            7'd44: mag = 1931; 7'd45: mag = 1842; 7'd46: mag = 1751; 7'd47: mag = 1660;
            7'd48: mag = 1567; 7'd49: mag = 1474; 7'd50: mag = 1380; 7'd51: mag = 1285;
            7'd52: mag = 1189; 7'd53: mag = 1092; 7'd54: mag = 995;  7'd55: mag = 897;
            7'd56: mag = 799;  7'd57: mag = 700;  7'd58: mag = 601;  7'd59: mag = 501;
            7'd60: mag = 401;  7'd61: mag = 301;  7'd62: mag = 201;  7'd63: mag = 101;
            default: mag = 0;
        endcase
        val = neg ? -mag : mag;
    end
endmodule
