# Placement of the PCIe endpoint + DMA (sys_i/xdma): in the clock regions around the PCIe hard block (PCIE40E4_X1Y2,
# right edge of SLR1), so the decoder cannot crowd the 500 MHz core-clock paths into the block. Build 2 (100 MHz,
# Performance_ExtraTimingOpt, 2026-10-23) met every decoder constraint and missed only inside XDMA: 44 endpoints,
# -0.047 ns, zero logic levels, 1.9 ns of routing from the 512-bit interface registers to the PCIE40E4 inputs.
#
# The box leaves X4Y6 .. X4Y8 alone (2026-11-23, K60-85): the DDR4 controller sits there, next to its I/O bank
# (45,212 cells in those three regions in the routed 4K images), and the first box, X4Y5:X5Y8, let XDMA put 21,486
# cells in among them. Three builds of the wide read port then missed timing inside one vendor block or the other
# (a DDR4 PHY pulse-width check, XDMA and DDR setup by 0.05 to 0.07 ns) with the decoder's clocks met. XDMA gets the
# X5 column from Y4 to Y9 and the X4 regions above and below the DDR4 controller instead.
create_pblock pb_pcie
add_cells_to_pblock [get_pblocks pb_pcie] [get_cells -quiet sys_i/xdma]
resize_pblock [get_pblocks pb_pcie] -add {CLOCKREGION_X5Y4:CLOCKREGION_X5Y9 CLOCKREGION_X4Y4:CLOCKREGION_X4Y5 CLOCKREGION_X4Y9:CLOCKREGION_X4Y9}
