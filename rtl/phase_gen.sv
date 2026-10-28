// The double-pumped frame memory's phase flag (rtl/frame_mem_uram.sv): 1 at the mid-cycle clk2x edge, 0 at the
// clk-aligned one. clk and clk2x come from the same MMCM with aligned rising edges (every clk edge is a clk2x edge).
//
// Derived from clk itself, sampled on the FALLING edge of clk2x: a quarter clk period after the clk-aligned edge clk
// is high, a quarter after the mid-cycle edge it is low (50 % duty), so the sample is a race-free alignment
// indicator with half a clk2x period of margin on each side; the rising-edge register then turns it into the flag
// one clk2x cycle later. The earlier generator (until 2026-10-27) aligned a clk2x toggle by sampling a clk-domain
// reset at the clk edge, a hold race that Vivado's replicas of the phase register each resolved on their own
// (replication keyed on the reset), so some builds ran part of the frame memory half a cycle off: 50 MHz images
// decoded wrongly while their timing closed. rst_in is kept for the shell / block design; it is not needed.
module phase_gen (
    input  logic clk,
    input  logic clk2x,
    input  logic rst_in,       // unused since the clock-sampled alignment (2026-10-27)
    output logic phase
);
    logic clk_s;                                             // clk seen at the falling clk2x edge
    logic unused_rst;
    assign unused_rst = rst_in;
    always_ff @(negedge clk2x) clk_s <= clk;
    always_ff @(posedge clk2x) phase <= ~clk_s;              // after the clk-aligned edge: 1 (writes at the mid-cycle one), after it: 0
endmodule
