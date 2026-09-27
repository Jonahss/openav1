// Intra tile decoder top: tile_syntax (entropy decoding + syntax) -> recon_top (prediction, transforms,
// reconstruction) -> frame_mem. Software supplies the parsed headers (hdr_t / rec_hdr_t), the CDF defaults
// and the tile bytes; the picture (pre loop filter) is read back through the frame buffer's host port.
module dec_top
  import cdf_map_pkg::*;
  import syn_pkg::*;
#(
    parameter int FBX = 10,                  // 1024 x 512 frame buffer: covers the test corpus (<= 640x360 + overhang)
    parameter int FBY = 9
) (
    input  logic              clk,
    input  logic              rst,
    input  hdr_t              hdr,
    input  rec_hdr_t          rh,
    // tile bytes
    input  logic [7:0]        in_data,
    input  logic              in_valid,
    output logic              in_ready,
    input  logic              in_eos,
    // cdf defaults
    input  logic              def_we,
    input  logic [CDF_AW-1:0] def_addr,
    input  logic [245:0]      def_data,
    // control
    input  logic              tile_start,
    output logic              tile_busy,
    output logic              tile_done,           // syntax finished and every block reconstructed
    output logic              unsupported,
    // observability (records as they stream between the stages)
    output logic              blk_done,
    output blk_rec_t          blk_rec,
    output logic              tx_done,
    output tx_rec_t           tx_rec,
    output logic              lr_done,
    output lr_rec_t           lr_rec,
    // frame buffer host read port (registered)
    input  logic [1:0]        h_plane,
    input  logic [FBX-1:0]    h_x,
    input  logic [FBY-1:0]    h_y,
    output logic [11:0]       h_rdata
);
    logic ts_done, tx_ack, sb_start, blk_info, pal_hold, pm_plane; logic [10:0] sb_r, sb_c;
    logic [9:0] q_addr; logic signed [20:0] q_data; logic [5:0] pm_x, pm_y; logic [2:0] pm_idx;
    tile_syntax u_ts (.clk, .rst, .hdr, .in_data, .in_valid, .in_ready, .in_eos, .def_we, .def_addr, .def_data,
                      .tile_start, .tile_busy, .tile_done(ts_done), .unsupported,
                      .sb_start_o(sb_start), .sb_r_o(sb_r), .sb_c_o(sb_c), .blk_info, .blk_done, .blk_rec,
                      .tx_done, .tx_rec, .tx_ack, .lr_done, .lr_rec, .q_addr, .q_data,
                      .pal_hold, .blk_ack(pal_hold), .pm_plane, .pm_x, .pm_y, .pm_idx);

    logic fb_re, fb_we, rc_busy; logic [1:0] fb_plane; logic [FBX-1:0] fb_x; logic [FBY-1:0] fb_y; logic [11:0] fb_wdata, fb_rdata;
    recon_top #(.FBX(FBX), .FBY(FBY)) u_rc (.clk, .rst, .hdr, .rh, .sb_start, .sb_r, .sb_c, .blk_info, .blk_rec,
                                            .tx_done, .tx_rec, .tx_ack, .q_addr, .q_data, .pm_plane, .pm_x, .pm_y, .pm_idx, .busy(rc_busy),
                                            .fb_re, .fb_we, .fb_plane, .fb_x, .fb_y, .fb_wdata, .fb_rdata);

    frame_mem #(.FBX(FBX), .FBY(FBY), .PW(12)) u_fb (.clk, .re(fb_re), .we(fb_we), .plane(fb_plane), .x(fb_x), .y(fb_y), .wdata(fb_wdata), .rdata(fb_rdata),
                                                     .h_plane, .h_x, .h_y, .h_rdata);

    // tile_done once the syntax is finished and the reconstruction stage has drained
    logic done_pend;
    always_ff @(posedge clk) begin
        tile_done <= 1'b0;
        if (rst) done_pend <= 1'b0;
        else begin
            if (ts_done) done_pend <= 1'b1;
            if ((done_pend || ts_done) && !rc_busy) begin done_pend <= 1'b0; tile_done <= 1'b1; end
        end
    end
endmodule
