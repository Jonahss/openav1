# Optional placement (U200_PBLOCK=1 in build.sh): keep the PCIe endpoint + DMA in the clock regions around the
# PCIe hard block (PCIE40E4_X1Y2 is at the right edge of SLR1; corundum's U200 design uses the same box), so the
# decoder cannot crowd the 500 MHz core-clock paths into the block. Build 2 (100 MHz, Performance_ExtraTimingOpt,
# 2026-10-23) met every decoder constraint and missed only inside XDMA: 44 endpoints, -0.047 ns, zero logic
# levels, 1.9 ns of routing from the 512-bit interface registers to the PCIE40E4 inputs.
create_pblock pb_pcie
add_cells_to_pblock [get_pblocks pb_pcie] [get_cells -quiet sys_i/xdma]
resize_pblock [get_pblocks pb_pcie] -add {CLOCKREGION_X4Y5:CLOCKREGION_X5Y8}
