// A first-in first-out memory of the streaming filter chain: W bits wide, 1 << AW entries of block RAM, valid / ready
// on both sides. An entry is at the output two clocks after it went in at the earliest (the memory's read, then the
// output register). `free` counts the memory's empty entries; the two entries in the read and output registers are
// room beyond it.
module fc_lfifo #(
    parameter int W = 105,
    parameter int AW = 8
) (
    input  logic          clk,
    input  logic          rst,
    input  logic          in_valid,
    output logic          in_ready,
    input  logic [W-1:0]  in_data,
    output logic          out_valid,
    input  logic          out_ready,
    output logic [W-1:0]  out_data,
    output logic [AW:0]   free
);
    (* ram_style = "block" *) logic [W-1:0] mem [1 << AW];
    logic [AW:0]  wp, rp;
    logic [W-1:0] m_q;
    logic         m_v, o_v;
    logic         m_move, rd;
    always_comb begin
        free = (AW+1)'(1 << AW) - (wp - rp);
        in_ready = free != '0;
        m_move = m_v && (!o_v || out_ready);       // the read register's entry goes to the output register
        rd = (rp != wp) && (!m_v || m_move);
    end
    assign out_valid = o_v;
    always_ff @(posedge clk) begin
        if (in_valid && in_ready) mem[wp[AW-1:0]] <= in_data;
        if (rd) m_q <= mem[rp[AW-1:0]];
        if (m_move) out_data <= m_q;
        if (rst) begin wp <= '0; rp <= '0; m_v <= 1'b0; o_v <= 1'b0; end
        else begin
            if (in_valid && in_ready) wp <= wp + (AW+1)'(1);
            if (rd) rp <= rp + (AW+1)'(1);
            m_v <= rd || (m_v && !m_move);
            o_v <= m_move || (o_v && !out_ready);
        end
    end
endmodule
