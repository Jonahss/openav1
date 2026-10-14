// The double-pumped frame memory's phase flag (rtl/frame_mem_uram.sv): 1 at the mid-cycle clk2x edge, 0 at the
// clk-aligned one. clk and clk2x come from the same MMCM with aligned rising edges (every clk edge is a clk2x
// edge). A clk2x toggle, aligned by a clk-domain reset: rst_q changes at a clk edge, so the first clk2x edge that
// samples it low is the mid-cycle one, where phase goes 1 -> 0; from then on phase is 0 before every clk-aligned
// edge and 1 before every mid-cycle edge. (The rst_q -> phase path is one clk2x period, not the two the
// clk -> clk2x multicycle constraint allows: fpga/kv260/pre_opt.tcl singles it out.)
module phase_gen (
    input  logic clk,
    input  logic clk2x,
    input  logic rst_in,       // clk-synchronous or longer than a few clk cycles (proc_sys_reset: peripheral_reset)
    output logic phase
);
    logic rst_s, rst_q;
    always_ff @(posedge clk) begin
        rst_s <= rst_in;
        rst_q <= rst_s;
    end
    always_ff @(posedge clk2x) begin
        if (rst_q) phase <= 1'b1;
        else       phase <= ~phase;
    end
endmodule
