// Two readers on one AXI4 read channel (one ID, responses in request order): reference fetch and the motion-field
// projection read the DDR during the same decode (docs/motion-field-band.md). A request goes downstream when the
// channel is free of a pending one and the owner queue has room; the queue remembers whose burst each is, and the
// data beats go to the reader at its head until that burst's last beat. Reader 1 goes first when both ask (the
// projection: few, short bursts; reference fetch keeps many in flight).
// Each reader must hold its request until it is taken and must be able to take the beats it asked for (both reserve
// room before they ask); the data, response and last signals themselves pass by outside this module.
module axi_rd_share #(
    parameter int QL2 = 5                                    // log2 of the bursts in flight over both readers
) (
    input  logic              clk,
    input  logic              rst,
    // the two readers
    input  logic [1:0][39:0]  s_araddr,
    input  logic [1:0][7:0]   s_arlen,
    input  logic [1:0]        s_arvalid,
    output logic [1:0]        s_arready,
    output logic [1:0]        s_rvalid,
    input  logic [1:0]        s_rready,
    // the channel
    output logic [39:0]       m_araddr,
    output logic [7:0]        m_arlen,
    output logic              m_arvalid,
    input  logic              m_arready,
    input  logic              m_rvalid,
    input  logic              m_rlast,
    output logic              m_rready
);
    localparam int Q = 1 << QL2;
    logic [Q-1:0]   own;                                     // the reader of each burst in flight, oldest at rp
    logic [QL2:0]   wp, rp, n;
    logic           full, head;
    assign n    = wp - rp;
    assign full = (n == (QL2+1)'(Q));
    assign head = own[rp[QL2-1:0]];
    // a request presented downstream stays until it is taken: the choice is held while it waits
    logic           lock, sel_q, sel;
    assign sel       = lock ? sel_q : s_arvalid[1];
    assign m_araddr  = s_araddr[sel];
    assign m_arlen   = s_arlen[sel];
    assign m_arvalid = s_arvalid[sel] && !full;
    assign s_arready = {sel && m_arready && !full, !sel && m_arready && !full};
    assign s_rvalid  = {m_rvalid && (n != '0) && head, m_rvalid && (n != '0) && !head};
    assign m_rready  = (n != '0) && s_rready[head];
    always_ff @(posedge clk) begin
        if (rst) begin wp <= '0; rp <= '0; lock <= 1'b0; sel_q <= 1'b0; end
        else begin
            lock <= m_arvalid && !m_arready;
            sel_q <= sel;
            if (m_arvalid && m_arready) begin own[wp[QL2-1:0]] <= sel; wp <= wp + 1'b1; end
            if (m_rvalid && m_rready && m_rlast) rp <= rp + 1'b1;
        end
    end
`ifndef SYNTHESIS
    always_ff @(posedge clk) if (!rst && m_rvalid && n == '0) $error("[axi_rd_share] a data beat with no burst in flight");
`endif
endmodule
