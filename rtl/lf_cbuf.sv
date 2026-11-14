// A small one-write one-read memory with an unregistered read: distributed RAM (the deblock stage's column buffers
// and strength buffers, rtl/lf_stream.sv).
module lf_cbuf #(
    parameter int DW = 192,
    parameter int AW = 5
) (
    input  logic          clk,
    input  logic          we,
    input  logic [AW-1:0] wa,
    input  logic [DW-1:0] wd,
    input  logic [AW-1:0] ra,
    output logic [DW-1:0] rd
);
    (* ram_style = "distributed" *) logic [DW-1:0] mem [1 << AW];
    always_ff @(posedge clk) if (we) mem[wa] <= wd;
    assign rd = mem[ra];
endmodule
