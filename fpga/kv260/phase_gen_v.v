// Verilog-2001 shell around rtl/phase_gen.sv for the block design's module-reference flow.
module phase_gen_v (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *)
    input  wire clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk2x CLK" *)
    input  wire clk2x,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rst_in RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_HIGH" *)
    input  wire rst_in,
    output wire phase
);
    phase_gen u (.clk(clk), .clk2x(clk2x), .rst_in(rst_in), .phase(phase));
endmodule
