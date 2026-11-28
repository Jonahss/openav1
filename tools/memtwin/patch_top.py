#!/usr/bin/env python3
"""patch_top.py <imports dir> [sc|bridge <sys_sc_mem_0_stub.v> [<module>]] (K60-108): put twin_player in place of the
example design's AXI traffic generator in example_top.sv (the original is kept as example_top.orig and always patched
from there).

  (nothing)  the player sits on the controller's AXI port, in the controller's clock.
  sc         the image build's memory interconnect (sys_sc_mem_0, its simulation netlist) sits between them as on the
             card: the player on S02 (reference fetch's read-only 512-bit port) in the decoder clock, S00 (XDMA) and
             S01 (the 64-bit master) idle.
  bridge     the proposed path: the player (decoder clock) goes through rf_ddr_bridge to the controller; the
             interconnect's master port goes through the same bridge; a second player (tag B, XDMA's clock) drives
             the interconnect's S00 so that the two kinds of reads and the writes mix (+READSB= +WRITESB= +CHECKB=).

<module>: the interconnect's module name when it is one made again by mk_sc.tcl (<name>_wrapper, same ports)."""
import os, re, shutil, sys
d = sys.argv[1]
mode = sys.argv[2] if len(sys.argv) > 3 else "direct"
assert mode in ("direct", "sc", "bridge"), mode
orig, top = os.path.join(d, "example_top.orig"), os.path.join(d, "example_top.sv")
if not os.path.exists(orig):
    shutil.copy(top, orig)
L = open(orig).read().split("\n")
a = next(i for i, l in enumerate(L) if l.startswith("ddr4_v2_2_29_axi_tg_top #("))
b = next(i for i in range(a + 20, len(L)) if L[i].strip() == ");")
tie = """assign boot_mode_done = 1'b0; assign prbs_mode_done = 1'b0;
assign c0_ddr4_data_msmatch_err = 1'b0; assign c0_ddr4_write_err = 1'b0; assign c0_ddr4_read_err = 1'b0;
assign c0_ddr4_tg_data_err_status_rdy = 1'b0; assign c0_ddr4_tg_data_err_status = 2'b0;"""
if mode == "direct":
    inst = """twin_player u_player (
  .clk(c0_ddr4_clk), .rst(c0_ddr4_rst), .go(c0_init_calib_complete_r3),
  .araddr(c0_ddr4_s_axi_araddr), .arlen(c0_ddr4_s_axi_arlen), .arvalid(c0_ddr4_s_axi_arvalid), .arready(c0_ddr4_s_axi_arready),
  .rvalid(c0_ddr4_s_axi_rvalid), .rlast(c0_ddr4_s_axi_rlast), .rdata_lo(c0_ddr4_s_axi_rdata[63:0]), .rready(c0_ddr4_s_axi_rready),
  .awaddr(c0_ddr4_s_axi_awaddr), .awlen(c0_ddr4_s_axi_awlen), .awvalid(c0_ddr4_s_axi_awvalid), .awready(c0_ddr4_s_axi_awready),
  .wdata(c0_ddr4_s_axi_wdata), .wstrb(c0_ddr4_s_axi_wstrb), .wlast(c0_ddr4_s_axi_wlast), .wvalid(c0_ddr4_s_axi_wvalid), .wready(c0_ddr4_s_axi_wready),
  .bvalid(c0_ddr4_s_axi_bvalid), .bready(c0_ddr4_s_axi_bready), .other_done(1'b1), .done());
assign c0_ddr4_s_axi_arsize = 3'd6; assign c0_ddr4_s_axi_arburst = 2'b01; assign c0_ddr4_s_axi_arcache = 4'b0011;
assign c0_ddr4_s_axi_awsize = 3'd6; assign c0_ddr4_s_axi_awburst = 2'b01; assign c0_ddr4_s_axi_awcache = 4'b0011; assign c0_ddr4_s_axi_awprot = 3'b0;
assign c0_ddr4_s_axi_arid = 1'b0; assign c0_ddr4_s_axi_awid = 1'b0;
""" + tie
else:
    stub = open(sys.argv[3]).read()
    module = sys.argv[4] if len(sys.argv) > 4 else "sys_sc_mem_0"
    ports = re.findall(r"(input|output)\s*(\[[0-9:]*\])?\s*([A-Za-z0-9_]+)\s*(?:/\*[^*]*\*/)?;", stub)
    assert {"aclk", "aclk1", "aclk2", "aresetn"} <= {n for _, _, n in ports}, "a clock or reset port was not found in the stub"
    conn = {"aclk": "tw_clk_x", "aclk1": "tw_clk_dec", "aclk2": "c0_ddr4_clk", "aresetn": "tw_x_aresetn"}
    if mode == "sc":
        conn.update({"S02_AXI_araddr": "{6'b0, tw_araddr}", "S02_AXI_arlen": "tw_arlen", "S02_AXI_arsize": "3'd6",
                     "S02_AXI_arburst": "2'b01", "S02_AXI_arcache": "4'b0011", "S02_AXI_arvalid": "tw_arvalid",
                     "S02_AXI_arready": "tw_arready", "S02_AXI_rlast": "tw_rlast", "S02_AXI_rvalid": "tw_rvalid",
                     "S02_AXI_rready": "tw_rready", "S02_AXI_rdata": "tw_rdata"})
    else:
        conn.update({"S00_AXI_araddr": "{30'b0, tb_araddr}", "S00_AXI_arlen": "tb_arlen", "S00_AXI_arsize": "3'd6",
                     "S00_AXI_arburst": "2'b01", "S00_AXI_arcache": "4'b0011", "S00_AXI_arvalid": "tb_arvalid",
                     "S00_AXI_arready": "tb_arready", "S00_AXI_rlast": "tb_rlast", "S00_AXI_rvalid": "tb_rvalid",
                     "S00_AXI_rready": "tb_rready", "S00_AXI_rdata": "tb_rdata",
                     "S00_AXI_awaddr": "{30'b0, tb_awaddr}", "S00_AXI_awlen": "tb_awlen", "S00_AXI_awsize": "3'd6",
                     "S00_AXI_awburst": "2'b01", "S00_AXI_awcache": "4'b0011", "S00_AXI_awvalid": "tb_awvalid",
                     "S00_AXI_awready": "tb_awready", "S00_AXI_wdata": "tb_wdata", "S00_AXI_wstrb": "tb_wstrb",
                     "S00_AXI_wlast": "tb_wlast", "S00_AXI_wvalid": "tb_wvalid", "S00_AXI_wready": "tb_wready",
                     "S00_AXI_bvalid": "tb_bvalid", "S00_AXI_bready": "tb_bready"})
    lines = []
    for dirn, width, name in ports:
        if name in conn:
            c = conn[name]
        elif name.startswith("M00_AXI_"):
            c = ("sc_" if mode == "bridge" else "c0_ddr4_s_axi_") + name[8:]
            if name[8:] in ("awlock", "awqos", "arlock", "arqos", "arprot"):
                c = ""
        elif dirn == "input":
            c = "1'b1" if name.endswith(("_rready", "_bready")) else "'0"
        else:
            c = ""
        lines.append("  .%s(%s)" % (name, c))
    common = """wire tw_clk_dec, tw_clk_x, tw_x_aresetn, tw_go, tw_rst_dec;
wire [33:0] tw_araddr; wire [7:0] tw_arlen; wire tw_arvalid, tw_arready, tw_rvalid, tw_rlast, tw_rready; wire [511:0] tw_rdata;
twin_clocks u_clocks (.calib(c0_init_calib_complete_r3), .clk_dec(tw_clk_dec), .clk_x(tw_clk_x), .x_aresetn(tw_x_aresetn), .go(tw_go), .rst_dec(tw_rst_dec));
"""
    if mode == "sc":
        inst = common + """twin_player u_player (
  .clk(tw_clk_dec), .rst(tw_rst_dec), .go(tw_go),
  .araddr(tw_araddr), .arlen(tw_arlen), .arvalid(tw_arvalid), .arready(tw_arready),
  .rvalid(tw_rvalid), .rlast(tw_rlast), .rdata_lo(tw_rdata[63:0]), .rready(tw_rready),
  .awaddr(), .awlen(), .awvalid(), .awready(1'b0), .wdata(), .wstrb(), .wlast(), .wvalid(), .wready(1'b0), .bvalid(1'b0), .bready(),
  .other_done(1'b1), .done());
assign c0_ddr4_s_axi_arid = 1'b0; assign c0_ddr4_s_axi_awid = 1'b0;
%s u_sc (
""" % module + ",\n".join(lines) + ");\n" + tie
    else:
        inst = common + """wire a_done, b_done;
twin_player u_player (
  .clk(tw_clk_dec), .rst(tw_rst_dec), .go(tw_go),
  .araddr(tw_araddr), .arlen(tw_arlen), .arvalid(tw_arvalid), .arready(tw_arready),
  .rvalid(tw_rvalid), .rlast(tw_rlast), .rdata_lo(tw_rdata[63:0]), .rready(tw_rready),
  .awaddr(), .awlen(), .awvalid(), .awready(1'b0), .wdata(), .wstrb(), .wlast(), .wvalid(), .wready(1'b0), .bvalid(1'b0), .bready(),
  .other_done(b_done), .done(a_done));
// the second player: XDMA's place on the interconnect (512-bit, its clock)
wire [33:0] tb_araddr, tb_awaddr; wire [7:0] tb_arlen, tb_awlen; wire tb_arvalid, tb_arready, tb_rvalid, tb_rlast, tb_rready;
wire tb_awvalid, tb_awready, tb_wlast, tb_wvalid, tb_wready, tb_bvalid, tb_bready; wire [511:0] tb_rdata, tb_wdata; wire [63:0] tb_wstrb;
reg tb_g1 = 1'b0, tb_go = 1'b0; always @(posedge tw_clk_x) begin tb_g1 <= tw_go; tb_go <= tb_g1; end
twin_player #(.TAG("B")) u_player_b (
  .clk(tw_clk_x), .rst(!tw_x_aresetn), .go(tb_go),
  .araddr(tb_araddr), .arlen(tb_arlen), .arvalid(tb_arvalid), .arready(tb_arready),
  .rvalid(tb_rvalid), .rlast(tb_rlast), .rdata_lo(tb_rdata[63:0]), .rready(tb_rready),
  .awaddr(tb_awaddr), .awlen(tb_awlen), .awvalid(tb_awvalid), .awready(tb_awready),
  .wdata(tb_wdata), .wstrb(tb_wstrb), .wlast(tb_wlast), .wvalid(tb_wvalid), .wready(tb_wready), .bvalid(tb_bvalid), .bready(tb_bready),
  .other_done(a_done), .done(b_done));
// the interconnect's master port, into the bridge
wire [33:0] sc_awaddr, sc_araddr; wire [7:0] sc_awlen, sc_arlen; wire [2:0] sc_awsize, sc_arsize, sc_awprot; wire [1:0] sc_awburst, sc_arburst, sc_bresp, sc_rresp;
wire [3:0] sc_awcache, sc_arcache; wire sc_awvalid, sc_awready, sc_wlast, sc_wvalid, sc_wready, sc_bvalid, sc_bready, sc_arvalid, sc_arready, sc_rlast, sc_rvalid, sc_rready;
wire [511:0] sc_wdata, sc_rdata; wire [63:0] sc_wstrb;
%s u_sc (
""" % module + ",\n".join(lines) + """);
rf_ddr_bridge u_bridge (
  .dclk(tw_clk_dec), .s_rf_araddr(tw_araddr), .s_rf_arlen(tw_arlen), .s_rf_arvalid(tw_arvalid), .s_rf_arready(tw_arready),
  .s_rf_rdata(tw_rdata), .s_rf_rresp(), .s_rf_rlast(tw_rlast), .s_rf_rvalid(tw_rvalid), .s_rf_rready(tw_rready),
  .uclk(c0_ddr4_clk), .urst(c0_ddr4_rst),
  .s_sc_awaddr(sc_awaddr), .s_sc_awlen(sc_awlen), .s_sc_awsize(sc_awsize), .s_sc_awburst(sc_awburst), .s_sc_awcache(sc_awcache), .s_sc_awprot(sc_awprot),
  .s_sc_awvalid(sc_awvalid), .s_sc_awready(sc_awready), .s_sc_wdata(sc_wdata), .s_sc_wstrb(sc_wstrb), .s_sc_wlast(sc_wlast), .s_sc_wvalid(sc_wvalid), .s_sc_wready(sc_wready),
  .s_sc_bresp(sc_bresp), .s_sc_bvalid(sc_bvalid), .s_sc_bready(sc_bready),
  .s_sc_araddr(sc_araddr), .s_sc_arlen(sc_arlen), .s_sc_arsize(sc_arsize), .s_sc_arburst(sc_arburst), .s_sc_arcache(sc_arcache), .s_sc_arvalid(sc_arvalid), .s_sc_arready(sc_arready),
  .s_sc_rdata(sc_rdata), .s_sc_rresp(sc_rresp), .s_sc_rlast(sc_rlast), .s_sc_rvalid(sc_rvalid), .s_sc_rready(sc_rready),
  .m_awid(c0_ddr4_s_axi_awid), .m_awaddr(c0_ddr4_s_axi_awaddr), .m_awlen(c0_ddr4_s_axi_awlen), .m_awsize(c0_ddr4_s_axi_awsize), .m_awburst(c0_ddr4_s_axi_awburst),
  .m_awcache(c0_ddr4_s_axi_awcache), .m_awprot(c0_ddr4_s_axi_awprot), .m_awvalid(c0_ddr4_s_axi_awvalid), .m_awready(c0_ddr4_s_axi_awready),
  .m_wdata(c0_ddr4_s_axi_wdata), .m_wstrb(c0_ddr4_s_axi_wstrb), .m_wlast(c0_ddr4_s_axi_wlast), .m_wvalid(c0_ddr4_s_axi_wvalid), .m_wready(c0_ddr4_s_axi_wready),
  .m_bresp(c0_ddr4_s_axi_bresp), .m_bvalid(c0_ddr4_s_axi_bvalid), .m_bready(c0_ddr4_s_axi_bready),
  .m_arid(c0_ddr4_s_axi_arid), .m_araddr(c0_ddr4_s_axi_araddr), .m_arlen(c0_ddr4_s_axi_arlen), .m_arsize(c0_ddr4_s_axi_arsize), .m_arburst(c0_ddr4_s_axi_arburst),
  .m_arcache(c0_ddr4_s_axi_arcache), .m_arvalid(c0_ddr4_s_axi_arvalid), .m_arready(c0_ddr4_s_axi_arready),
  .m_rid(c0_ddr4_s_axi_rid), .m_rdata(c0_ddr4_s_axi_rdata), .m_rresp(c0_ddr4_s_axi_rresp), .m_rlast(c0_ddr4_s_axi_rlast), .m_rvalid(c0_ddr4_s_axi_rvalid), .m_rready(c0_ddr4_s_axi_rready));
""" + tie
open(top, "w").write("\n".join(L[:a] + inst.split("\n") + L[b + 1:]))
print("patched (%s): lines %d to %d of the original replaced" % (mode, a + 1, b + 1))
