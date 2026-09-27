// Symbol sequencer: one adaptive symbol (or bool) end to end -- CDF row read from cdf_store, msac
// request, response, adapted-row write-back. Every syntax-decoding FSM drives one of these.
//
//   go (with addr/n/kind)  ->  cycle+1: msac request (row read overlaps the go cycle)
//                          ->  cycle+2 (if msac accepted at once): done=1, sym valid, row written back
// kind: 0 adaptive symbol (row at addr), 1 fixed-probability bool (row at addr, no adaptation),
//       2 equiprobable bool (no CDF access at all). n = N-1.
module sym_seq
  import cdf_map_pkg::*;
(
    input  logic              clk,
    input  logic              rst,
    input  logic              go,
    input  logic [CDF_AW-1:0] addr,
    input  logic [3:0]        n,
    input  logic [1:0]        kind,
    output logic              busy,
    output logic              done,
    output logic [3:0]        sym,
    // cdf store
    output logic              cdf_rd_en,
    output logic [CDF_AW-1:0] cdf_rd_addr,
    input  logic [245:0]      cdf_rd_data,
    output logic              cdf_wb_we,
    output logic [CDF_AW-1:0] cdf_wb_addr,
    output logic [245:0]      cdf_wb_data,
    // msac
    output logic              req_valid,
    input  logic              req_ready,
    output logic [1:0]        req_kind,
    output logic [3:0]        req_n,
    output logic [239:0]      req_cdf,
    output logic [5:0]        req_cnt,
    input  logic              resp_valid,
    input  logic [3:0]        resp_sym,
    input  logic [239:0]      resp_cdf,
    input  logic [5:0]        resp_cnt
);
    typedef enum logic [1:0] {S_IDLE, S_REQ, S_RESP} st_t;
    st_t st;
    logic [CDF_AW-1:0] l_addr;
    logic [3:0]        l_n;
    logic [1:0]        l_kind;
    logic [245:0]      row;           // CDF row captured for the request (held while waiting for ready)

    assign cdf_rd_en   = go && kind != 2'd2;
    assign cdf_rd_addr = addr;

    // a new `go` is accepted when idle or in the very cycle the previous symbol completes (back-to-back
    // symbols; the previous row's write-back and the new row's read happen together -- cdf_store forwards)
    wire accept_go = go && (st == S_IDLE || (st == S_RESP && resp_valid));

    always_ff @(posedge clk) begin
        if (rst) begin
            st <= S_IDLE;
        end else begin
            if (accept_go) begin
                l_addr <= addr; l_n <= n; l_kind <= kind;
                st <= S_REQ;
            end else begin
                case (st)
                    S_REQ:  if (req_ready) st <= S_RESP;
                    S_RESP: if (resp_valid) st <= S_IDLE;
                    default: st <= S_IDLE;
                endcase
            end
        end
    end

    // request: in S_REQ the row read issued at `go` is on cdf_rd_data (first S_REQ cycle); hold a copy
    logic first_req;
    always_ff @(posedge clk) first_req <= accept_go;
    always_ff @(posedge clk) if (first_req) row <= cdf_rd_data;
    wire [245:0] row_now = first_req ? cdf_rd_data : row;

    assign req_valid = (st == S_REQ);
    assign req_kind  = l_kind;
    assign req_n     = l_n;
    assign req_cdf   = row_now[239:0];
    assign req_cnt   = row_now[245:240];

    assign busy = (st != S_IDLE);
    assign done = (st == S_RESP) && resp_valid;
    assign sym  = resp_sym;

    assign cdf_wb_we   = done && (l_kind == 2'd0);
    assign cdf_wb_addr = l_addr;
    assign cdf_wb_data = {resp_cnt, resp_cdf};
endmodule
