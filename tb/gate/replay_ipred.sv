// Gate-level replay for ipred: drives the recorded per-cycle input vectors (tools/vcd2vec.py) into the DUT and
// compares its outputs with the recorded ones. Works on the RTL (control) and on a post-synthesis netlist.
//   inputs  (MSB first): rst edge_we edge_side[1:0] edge_idx[7:0] edge_data[11:0] edge_we4 edge_data4[47:0] start
//                        mode[3:0] use_filter_intra filter_intra_mode[2:0] angle_delta[2:0] log2w[2:0] log2h[2:0]
//                        bit_depth[3:0] have_left have_above filter_type edge_filter_en above_px[6:0] left_px[6:0]
//   outputs (MSB first): busy done out_valid out_x[5:0] out_y[5:0] out_pix4[47:0]
`timescale 1ns/1ps
module replay_ipred;
    localparam int NI = 1+1+2+8+12+1+48+1+4+1+3+3+3+3+4+1+1+1+1+7+7;   // 113
    localparam int NO = 1+1+1+6+6+48;                                    // 63
    localparam int MAXV = 400000;
    logic [NI-1:0] vin [0:MAXV-1];
    logic [NO-1:0] vout [0:MAXV-1];
    logic clk = 0;
    logic rst, edge_we, edge_we4, start, use_filter_intra, have_left, have_above, filter_type, edge_filter_en;
    logic [1:0] edge_side; logic [7:0] edge_idx; logic [11:0] edge_data; logic [47:0] edge_data4;
    logic [3:0] mode, bit_depth; logic [2:0] filter_intra_mode, log2w, log2h; logic signed [2:0] angle_delta;
    logic [6:0] above_px, left_px;
    logic busy, done, out_valid; logic [5:0] out_x, out_y; logic [47:0] out_pix4;
    ipred dut (.*);
`ifdef RTL_CONTROL
    // the RTL has no reset on its memories; Verilator starts them at zero, xsim at X (and X in a compare or a
    // case select turns into a definite wrong value). Start the control run the way Verilator and the FPGA do.
    initial begin
        for (int i = 0; i < 65; i++) begin
            dut.A0[i] = '0; dut.A1[i] = '0; dut.A2[i] = '0; dut.A3[i] = '0;
            dut.L0[i] = '0; dut.L1[i] = '0; dut.L2[i] = '0; dut.L3[i] = '0;
        end
        for (int i = 0; i < 128; i++) dut.fi_buf[i] = '0;
    end
`endif
    int n, errs, cmp, fi_cycles, nz, nv, preroll;
    string vecfile;
    initial begin
        if (!$value$plusargs("vec=%s", vecfile)) vecfile = "vectors.hex";
        for (int i = 0; i < MAXV; i++) vin[i] = 'x;
        $readmemh(vecfile, vin, 0, MAXV-1);
    end
    initial begin
        if (!$value$plusargs("vec=%s", vecfile)) vecfile = "vectors.hex";
        $readmemh({vecfile, ".out"}, vout, 0, MAXV-1);
    end
    task automatic drive(input logic [NI-1:0] v);
        {rst, edge_we, edge_side, edge_idx, edge_data, edge_we4, edge_data4, start, mode, use_filter_intra,
         filter_intra_mode, angle_delta, log2w, log2h, bit_depth, have_left, have_above, filter_type,
         edge_filter_en, above_px, left_px} = v;
    endtask
    initial begin
        errs = 0; cmp = 0; fi_cycles = 0; nz = 0; nv = 0;
        if (!$value$plusargs("preroll=%d", preroll)) preroll = 15;   // the funcsim netlist holds GSR for 100 ns
        for (int i = 0; i < preroll; i++) begin drive(vin[0]); #5 clk = 1; #5 clk = 0; end
        n = 0;
        while (n < MAXV && vin[n] !== 'x) begin
            drive(vin[n]);
            #4;
            // compare the DUT's outputs (settled before the edge) with the recorded pre-edge values
            if (n > 2) begin
                cmp++;
                if (out_valid) begin nv++; if (out_pix4 != 0) nz++; end
                if ({busy, done, out_valid} !== vout[n][NO-1 -: 3] ||
                    (vout[n][NO-3] && {out_x, out_y, out_pix4} !== vout[n][NO-4:0])) begin
                    errs++;
                    if (errs <= 100000) $display("MISMATCH cycle %0d: got busy=%b done=%b v=%b x=%0d y=%0d pix=%h exp %h", n,
                                             busy, done, out_valid, out_x, out_y, out_pix4, vout[n]);
                end
            end
            #1 clk = 1; #5 clk = 0;
            n++;
        end
        $display("REPLAY %s: %0d cycles, %0d compared, %0d mismatches (%0d valid, %0d nonzero) -> %s", vecfile, n, cmp, errs, nv, nz, errs == 0 ? "PASS" : "FAIL");
        $finish;
    end
endmodule
