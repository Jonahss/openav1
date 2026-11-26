// Two AXI4 read masters on one read port (K60-104): the reference fetch of each prediction lane (K60-94) reads through
// the decoder's one reference read master, so the card's shell and its interconnect stay as they are. Read addresses
// are taken in turn (the side that did not go last wins a tie) into one register stage; every burst sent is noted in
// a FIFO of owners. The memory answers in order (one ID), so the beats of the response at the head of that FIFO
// belong to its owner, and its last beat retires it. The read data is not registered here: both sides see the same
// data, response and last flag, each with its own valid. A side must accept every beat (ref_fetch does: rready = 1).
module rf_axi_mux #(
    parameter int LD = 6                                     // log2 of the bursts that may be outstanding (both sides together)
) (
    input  logic            clk,
    input  logic            rst,
    // the two sides
    input  logic [39:0]     s_araddr [2],
    input  logic [7:0]      s_arlen [2],
    input  logic [1:0]      s_arvalid,
    output logic [1:0]      s_arready,
    output logic [1:0]      s_rvalid,
    // the port
    output logic [39:0]     m_araddr,
    output logic [7:0]      m_arlen,
    output logic            m_arvalid,
    input  logic            m_arready,
    input  logic            m_rlast,
    input  logic            m_rvalid
);
    // ---- owners of the outstanding bursts, oldest first
    logic            own [1 << LD];
    logic [LD:0]     wp, rp;
    logic            full, empty;
    assign full  = (wp[LD] != rp[LD]) && (wp[LD-1:0] == rp[LD-1:0]);
    assign empty = wp == rp;
    // ---- read address: one register stage, loaded when it is free or being taken
    logic            last_q;                                 // the side that went last
    logic            sel;
    logic            can, take;
    assign can  = (!m_arvalid || m_arready) && !full;
    assign sel  = (s_arvalid == 2'b11) ? !last_q : s_arvalid[1];
    assign take = can && (|s_arvalid);
    assign s_arready = {take && sel, take && !sel};
    always_ff @(posedge clk) begin
        if (rst) begin m_arvalid <= 1'b0; last_q <= 1'b0; wp <= '0; end
        else begin
            if (take) begin
                m_arvalid <= 1'b1; m_araddr <= s_araddr[32'(sel)]; m_arlen <= s_arlen[32'(sel)];
                last_q <= sel; own[wp[LD-1:0]] <= sel; wp <= wp + 1'b1;
            end else if (m_arready) m_arvalid <= 1'b0;
        end
    end
    // ---- read data: the beats of the oldest outstanding burst go to its owner
    logic head;
    assign head = own[rp[LD-1:0]];
    assign s_rvalid = {m_rvalid && !empty && head, m_rvalid && !empty && !head};
    always_ff @(posedge clk) begin
        if (rst) rp <= '0;
        else if (m_rvalid && m_rlast && !empty) rp <= rp + 1'b1;
    end
`ifndef SYNTHESIS
    always_ff @(posedge clk) if (!rst && m_rvalid && empty) $error("rf_axi_mux: a read beat with no burst outstanding");
`endif
endmodule
