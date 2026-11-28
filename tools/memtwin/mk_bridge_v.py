#!/usr/bin/env python3
"""mk_bridge_v.py <out.v> (K60-108): writes fpga/u200/rf_ddr_bridge_v.v, the Verilog-2001 shell around
rf_ddr_bridge.sv for the block design's module-reference flow, with an X_INTERFACE_INFO attribute on every port so
that the block design sees three AXI interfaces. Signals a master offers that the bridge does not use (reference
fetch's size / burst / cache / protection and its address bits above 34; the smartconnect's read protection) are
ports of the shell only."""
import sys
AW = 34
SIG = {"aw": [("i", "awaddr", AW), ("i", "awlen", 8), ("i", "awsize", 3), ("i", "awburst", 2), ("i", "awcache", 4), ("i", "awprot", 3), ("i", "awvalid", 1), ("o", "awready", 1)],
       "w": [("i", "wdata", 512), ("i", "wstrb", 64), ("i", "wlast", 1), ("i", "wvalid", 1), ("o", "wready", 1)],
       "b": [("o", "bresp", 2), ("o", "bvalid", 1), ("i", "bready", 1)],
       "ar": [("i", "araddr", AW), ("i", "arlen", 8), ("i", "arsize", 3), ("i", "arburst", 2), ("i", "arcache", 4), ("i", "arprot", 3), ("i", "arvalid", 1), ("o", "arready", 1)],
       "r": [("o", "rdata", 512), ("o", "rresp", 2), ("o", "rlast", 1), ("o", "rvalid", 1), ("i", "rready", 1)]}
def chan(bus, master, chans, idw=0, addr_w=AW):
    out = []
    for c in chans:
        l = list(SIG[c])
        if idw and c in ("aw", "ar"):
            l.insert(0, ("i", c + "id", idw))
        if idw and c == "r":
            l.insert(0, ("o", "rid", idw))
        for dr, n, w in l:
            if n.endswith("addr"):
                w = addr_w
            if master:
                dr = "o" if dr == "i" else "i"
            out.append((dr, bus + "_" + n, w, bus, n))
    return out
ports = chan("s_rf_axi", False, ["ar", "r"], addr_w=40) + chan("s_sc_axi", False, ["aw", "w", "b", "ar", "r"]) + chan("m_axi", True, ["aw", "w", "b", "ar", "r"], idw=1)
ports = [p for p in ports if p[1] != "m_axi_arprot"]            # the controller ties its own default
SHELL_ONLY = {"s_rf_axi_arsize", "s_rf_axi_arburst", "s_rf_axi_arcache", "s_rf_axi_arprot", "s_sc_axi_arprot"}
PAR = {"s_rf_axi": "READ_WRITE_MODE READ_ONLY, NUM_READ_OUTSTANDING 32, MAX_BURST_LENGTH 16, SUPPORTS_NARROW_BURST 0",
       "s_sc_axi": "NUM_READ_OUTSTANDING 32, NUM_WRITE_OUTSTANDING 16, SUPPORTS_NARROW_BURST 0",
       "m_axi": "NUM_READ_OUTSTANDING 32, NUM_WRITE_OUTSTANDING 16, SUPPORTS_NARROW_BURST 0"}
L = ["""// Verilog-2001 shell around rf_ddr_bridge.sv for the block design's module-reference flow (which refuses a
// SystemVerilog top file), with the interface attributes that let the block design see three AXI interfaces:
// s_rf_axi (reference fetch's read-only master, decoder clock), s_sc_axi (the smartconnect's master port) and m_axi
// (the DDR4 controller's slave port), the last two in the controller's clock. Written by tools/memtwin/mk_bridge_v.py:
// change that, not this file.
module rf_ddr_bridge_v (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 dclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_rf_axi" *)
    input  wire         dclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 uclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_sc_axi:m_axi, ASSOCIATED_RESET urst" *)
    input  wire         uclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 urst RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_HIGH" *)
    input  wire         urst,"""]
seen = set()
for i, (dr, n, w, bus, sig) in enumerate(ports):
    if bus not in seen:
        seen.add(bus)
        L.append('    (* X_INTERFACE_PARAMETER = "%s" *)' % PAR[bus])
    decl = ("input  wire" if dr == "i" else "output wire") + (" [%d:0]" % (w - 1) if w > 1 or sig.endswith("id") else "")
    L.append('    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 %s %s" *) %s %s%s' % (bus, sig.upper(), decl, n, "," if i < len(ports) - 1 else ""))
L.append(");")
def inner(n):
    return ("s_rf_" + n[9:]) if n.startswith("s_rf_axi_") else ("s_sc_" + n[9:]) if n.startswith("s_sc_axi_") else ("m_" + n[6:])
L.append("    rf_ddr_bridge u (")
L.append("        .dclk(dclk), .uclk(uclk), .urst(urst),")
conns = []
for dr, n, w, bus, sig in ports:
    if n in SHELL_ONLY:
        continue
    conns.append("        .%s(%s)" % (inner(n), n + ("[%d:0]" % (AW - 1) if n == "s_rf_axi_araddr" else "")))
L.append(",\n".join(conns) + ");")
L.append("endmodule")
open(sys.argv[1], "w").write("\n".join(L) + "\n")
print(len(ports), "ports,", len(conns), "connected to rf_ddr_bridge")
