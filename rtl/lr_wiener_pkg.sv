// Wiener loop restoration (spec 7.17.4) as two line modules for the streaming filter chain (docs/filter-chain.md):
// lr_top's arithmetic lifted out with the lane count as a parameter.
//   lr_wiener_h : one line, N output positions: 7 taps over N + 6 source samples (the caller has applied the plane
//                 and stripe rules to them) -> N intermediate values (21 bits, signed), two clocks.
//   lr_wiener_v : N positions: 7 intermediate lines -> N samples, two clocks.
// Coefficients as the block records carry them: three signed 7-bit values c0, c1, c2 for taps 0 / 6, 1 / 5, 2 / 4;
// tap 3 is 128 - 2 (c0 + c1 + c2). Rounding by bit depth: InterRound0 = 3 (5 at 12 bits), InterRound1 = 11 (9).
package lr_wiener_pkg;
    function automatic logic signed [9:0] wcoef(input logic [20:0] c3, input logic [2:0] t);
        logic signed [6:0] c0, c1, c2;
        c0 = c3[6:0]; c1 = c3[13:7]; c2 = c3[20:14];
        case (t)
            3'd0, 3'd6: wcoef = 10'(c0);
            3'd1, 3'd5: wcoef = 10'(c1);
            3'd2, 3'd4: wcoef = 10'(c2);
            default: wcoef = 10'sd128 - 10'sd2 * (10'(c0) + 10'(c1) + 10'(c2));
        endcase
    endfunction
endpackage
