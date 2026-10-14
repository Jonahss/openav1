// Timing wrapper for the symbol loop: cdf_store + one sym_seq + msac, with the sequencer driven from ports, so
// the loop (CDF row read -> range multiply -> compare -> value / range update -> write-back) can be synthesised
// and timed in minutes, out of context (synth/vivado/run.sh -t sym_path). Not part of the decoder.
module sym_path
  import cdf_map_pkg::*;
(
    input  logic              clk,
    input  logic              rst,
    input  logic [7:0]        in_data,
    input  logic              in_valid,
    output logic              in_ready,
    input  logic              in_eos,
    input  logic              init,
    input  logic              cdf_update_en,
    input  logic              def_we,
    input  logic [CDF_AW-1:0] def_addr,
    input  logic [245:0]      def_data,
    input  logic              cdf_init,
    output logic              cdf_busy,
    input  logic              go,
    input  logic [CDF_AW-1:0] addr,
    input  logic [3:0]        n,
    input  logic [1:0]        kind,
    input  logic [15:0]       f_in,
    output logic              busy,
    output logic              done,
    output logic [3:0]        sym,
    output logic [245:0]      row_out
);
    logic req_valid, req_ready, resp_valid;
    logic [1:0] req_kind; logic [3:0] req_n; logic [239:0] req_cdf; logic [5:0] req_cnt;
    logic [3:0] resp_sym; logic [15:0] resp_rng; logic [239:0] resp_cdf; logic [5:0] resp_cnt;
    logic rd_en, wb_we, wb_pre; logic [CDF_AW-1:0] rd_addr, wb_addr, wb_pre_addr; logic [245:0] rd_data, wb_data;
    msac u_msac (.clk, .rst, .in_data, .in_valid, .in_ready, .in_eos, .init, .cdf_update_en,
                 .req_valid, .req_ready, .req_kind, .req_n, .req_cdf, .req_cnt,
                 .resp_valid, .resp_sym, .resp_wb_valid(), .resp_rng, .resp_cdf, .resp_cnt);
    cdf_store u_cdf (.clk, .rst, .def_we, .def_addr, .def_data, .init(cdf_init), .busy(cdf_busy),
                     .rd_en, .rd_addr, .rd_data, .wb_we, .wb_addr, .wb_data, .wb_pre, .wb_pre_addr);
    sym_seq u_sq (.clk, .rst, .go, .addr, .n, .kind, .f_in, .busy, .done, .sym, .row_out,
                  .cdf_rd_en(rd_en), .cdf_rd_addr(rd_addr), .cdf_rd_data(rd_data),
                  .cdf_wb_we(wb_we), .cdf_wb_addr(wb_addr), .cdf_wb_data(wb_data), .cdf_wb_pre(wb_pre), .cdf_wb_pre_addr(wb_pre_addr),
                  .req_valid, .req_ready, .req_kind, .req_n, .req_cdf, .req_cnt,
                  .resp_valid, .resp_sym, .resp_cdf, .resp_cnt);
endmodule
