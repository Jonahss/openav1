# Warning census: u200-fc6clte2s90p-e062b8f

decoder: 3455 printed, 3455 open; shell: 24 printed, 24 open; vendor: 785 printed, 785 open. Kinds Vivado cut off at its message limit (counts are lower bounds): 41.


## decoder

| ID | severity | printed | waived | open | cut off | first open sample / waiver reason |
|---|---|---:|---:|---:|---|---|
| DPIP-2 | warning | 1133 | 0 | 1133 |  | Input pipelining DSP sys_i/host/inst/u/u_dec/g_chain.u_fc/g_p[0].u_pl/u_cdef/u_str/pri2 input sys_i/host/inst/u/u_dec/g_chain.u_fc/g_p[0].u_pl/u_cdef/u_str/pri2/A[29:0] is not pipelined. Pipelining DS |
| DPOP-4 | warning | 602 | 0 | 602 |  | MREG Output pipelining DSP sys_i/host/inst/u/u_dec/g_chain.u_fc/g_p[0].u_pl/u_cdef/u_dir/sq_r_reg[0][7] multiplier stage sys_i/host/inst/u/u_dec/g_chain.u_fc/g_p[0].u_pl/u_cdef/u_dir/sq_r_reg[0][7]/P[ |
| DPOP-3 | warning | 524 | 0 | 524 |  | PREG Output pipelining DSP sys_i/host/inst/u/u_dec/g_chain.u_fc/g_p[0].u_pl/u_cdef/u_dir/p_1_out__0__0 output sys_i/host/inst/u/u_dec/g_chain.u_fc/g_p[0].u_pl/u_cdef/u_dir/p_1_out__0__0/P[47:0] is not |
| REQP-1860 | warning | 384 | 0 | 384 |  | URAM288_USE_EXT_CE_FALSE_tie_OREG_CE_high URAM288 sys_i/host/inst/u/u_dec/g_fb0_uram.u_fb/g_uram[0].u has USE_EXT_CE_A is set to FALSE and/or OREG_A is set to FALSE, so the OREG_CE_A pin should be tie |
| REQP-1861 | warning | 384 | 0 | 384 |  | URAM288_USE_EXT_CE_FALSE_tie_OREG_ECC_CE_high URAM288 sys_i/host/inst/u/u_dec/g_fb0_uram.u_fb/g_uram[0].u has USE_EXT_CE_A is set to FALSE and/or OREG_ECC_A is set to FALSE, so the OREG_ECC_CE_A pin s |
| Synth 8-6014 | warning | 100 | 0 | 100 | yes | Unused sequential element cur_reg[row_end] was removed.  [rtl/frame_state_w.sv:168] |
| Synth 8-3936 | warning | 100 | 0 | 100 | yes | Found unconnected internal register 'g_p[2].rrow_reg' and it is trimmed from '13' to '5' bits. [rtl/fc_rstore.sv:53] |
| Synth 8-3332 | warning | 97 | 0 | 97 |  | Sequential element (prod_reg[47]) is unused and will be removed from module warp_setup. |
| Synth 8-3917 | warning | 84 | 0 | 84 | yes | design cdef_stream__GCBM0 has port O205[3] driven by constant 1 |
| Synth 8-6841 | warning | 24 | 0 | 24 |  | Block RAM (g_p[2].g_l[0].mem_reg) originally specified as a Byte Wide Write Enable RAM cannot take advantage of ByteWide feature and is implemented with single write enable per RAM due to following re |
| Synth 8-6057 | warning | 12 | 0 | 12 |  | Memory: "lf_info_b_reg" defined in module: "mi_store__GBM1" implemented as Ultra-Ram has no pipeline registers. It is recommended to use pipeline registers to achieve high performance. |
| Synth 8-4767 | warning | 6 | 0 | 6 |  | Trying to implement RAM ' ' in registers. Block RAM or DRAM implementation is not possible; see log for reasons. [rtl/mi_store.sv:163] |
| Synth 8-11357 | warning | 1 | 0 | 1 |  | Potential Runtime issue for 3D-RAM or RAM from Record/Structs for RAM  line_reg with 15360 registers |
| Synth 8-6013 | warning | 1 | 0 | 1 |  | UltraRAM "u_mvm/mem_reg" is under-pipelined and may not meet performance target : Pipeline stages found = 1; Recommended pipeline stages =3. |
| DPREG-7 | warning | 1 | 0 | 1 |  | DSP48E2_PregDynOpmodeZmuxP: The DSP48E2 cell sys_i/host/inst/u/u_dec/u_rc/p_1_out with the given dynamic OPMODE[8:0] connections may lead to an unregistered asynchronous feedback path without the PREG |
| CLKC-30 | advisory | 1 | 0 | 1 |  | MMCME4 not driven by IO has BUFG in feedback loop The MMCME4 cell sys_i/clkgen/inst/mmcm has a BUFGCE sys_i/clkgen/inst/b_fb clock buffer in the feedback loop, but the clock input is not directly driv |
| CLKC-56 | advisory | 1 | 0 | 1 |  | MMCME4 with global clock driver has no LOC The MMCME4_ADV cell sys_i/clkgen/inst/mmcm CLKIN1 or CLKIN2 pin is driven by global Clock buffer(s) sys_i/ddr/inst/u_ddr4_infrastructure/u_bufg_addn_ui_clk_1 |

## shell

| ID | severity | printed | waived | open | cut off | first open sample / waiver reason |
|---|---|---:|---:|---:|---|---|
| IP_Flow 19-5661 | warning | 4 | 0 | 4 |  | Bus Interface 'clk_in1' does not have any bus interfaces associated with it. |
| IP_Flow 19-11770 | warning | 4 | 0 | 4 |  | Clock interface 'clk' has no FREQ_HZ parameter. |
| filemgmt 56-443 | warning | 4 | 0 | 4 |  | The ECC Algorithm string is empty. Setting the Memory Map to default ECC value to ECC_NONE. |
| BD 41-3281 | warning | 2 | 0 | 2 |  | Instance '/host' is connected on both sides by SmartConnects and cannot automatically configure itself to match the endpoints. Please manually configure its AXI interfaces as required for the design. |
| BD 41-2671 | warning | 2 | 0 | 2 |  | The dangling interface net <m00_aw_node_M_AXIS_ARB> will not be written out to the BD file. |
| Vivado 12-1815 | warning | 2 | 0 | 2 |  | Setting property 'IOSTANDARD' is not allowed for GT terminals. |
| Vivado 12-7122 | warning | 1 | 0 | 1 |  | Auto Incremental Compile:: No reference checkpoint was found in run synth_1. Auto-incremental flow will not be run, the standard flow will be run instead. |
| Place 46-14 | warning | 1 | 0 | 1 |  | The placer has determined that this design is highly congested and may have difficulty routing. Run report_design_analysis -congestion for a detailed report. |
| Route 35-468 | warning | 1 | 0 | 1 |  | The router encountered 17 pins that are both setup-critical and hold-critical and tried to fix hold violations at the expense of setup slack. Such pins are: |
| Timing 38-436 | warning | 1 | 0 | 1 |  | There are set_bus_skew constraint(s) in this design. Please run report_bus_skew to ensure that bus skew requirements are met. |
| Power 33-332 | warning | 1 | 0 | 1 |  | Found switching activity that implies high-fanout reset nets being asserted for excessive periods of time which may result in inaccurate power analysis. |
| TIMING-9 | warning | 1 | 0 | 1 |  | Unknown CDC Logic One or more asynchronous Clock Domain Crossing has been detected between 2 clock domains through a set_false_path or a set_clock_groups or set_max_delay -datapath_only constraint but |

## vendor

| ID | severity | printed | waived | open | cut off | first open sample / waiver reason |
|---|---|---:|---:|---:|---|---|
| Synth 8-6014 | warning | 132 | 0 | 132 | yes | Unused sequential element cmd_cmd_cas_reg was removed.  [rtl/controller/ddr4_v2_2_mc_group.sv:890] |
| Synth 8-3332 | warning | 114 | 0 | 114 | yes | Sequential element (POR_FF_I) is unused and will be removed from module lmb_v10. |
| Synth 8-7071 | warning | 104 | 0 | 104 |  | port 'c0_init_calib_complete' of module 'sys_ddr_0' is unconnected for instance 'ddr' [synth/sys.v:280] |
| Synth 8-7129 | warning | 100 | 0 | 100 | yes | Port axaddr[31] in module ddr4_v2_2_29_axi_ctrl_addr_decode__parameterized0 is either unconnected or has no load |
| Synth 8-3917 | warning | 92 | 0 | 92 |  | design ddr4_v2_2_29_mc__GB0 has port O4[2] driven by constant 0 |
| DCIRST-1 | warning | 40 | 0 | 40 |  | DCIRESET is required for design IO port ddr4_sdram_c1_act_n is using DCI IO standard SSTL12_DCI but no DCIRESET primitive is used in the design. Please add a DCIRESET in the design to ensure the prope |
| Vivado_Tcl 4-2057 | warning | 36 | 0 | 36 |  | Waiver ID 'DPIP-1' object list specifies likely unusable hierarchical instance 'sys_i/ddr/inst/u_ddr4_mem_intfc/u_ddr_cal_riu/mcs0/inst/microblaze_I/U0/MicroBlaze_Core_I/Performance.Core/Data_Flow_I/M |
| TIMING-16 | warning | 28 | 0 | 28 |  | Large setup violation There is a large setup violation of -1.002 ns between sys_i/xdma/inst/pcie4_ip_i/inst/sys_xdma_0_pcie4_ip_pcie_4_0_pipe_inst/pcie_4_0_e4_inst/CORECLK (clocked by GTYE4_CHANNEL_TX |
| Synth 8-7023 | warning | 24 | 0 | 24 |  | instance 'ddr' of module 'sys_ddr_0' has 78 connections declared, but only 72 given [synth/sys.v:280] |
| REQP-1858 | warning | 22 | 0 | 22 |  | RAMB36E2_writefirst_collision_advisory Synchronous clocking is detected for BRAM (sys_i/xdma/inst/pcie4_ip_i/inst/sys_xdma_0_pcie4_ip_pcie_4_0_pipe_inst/pcie_4_0_bram_inst/RAM32K.bram_comp_inst/bram_1 |
| Synth 8-3936 | warning | 18 | 0 | 18 |  | Found unconnected internal register 'cplx_config_chip_select_reg' and it is trimmed from '4' to '1' bits. [rtl/cal/ddr4_v2_2_cal_cplx.sv:280] |
| Netlist 29-47 | warning | 14 | 0 | 14 |  | Cannot modify dont-touch cell 'sys_i/ddr/inst/u_ddr4_mem_intfc/u_mig_ddr4_phy'. |
| LUTAR-1 | warning | 13 | 0 | 13 |  | LUT drives async reset alert LUT cell dbg_hub/inst/BSCANID.u_xsdbm_id/CORE_XSDB.UUT_MASTER/U_ICON_INTERFACE/U_CMD6_RD/U_RD_FIFO/SUBCORE_FIFO.xsdbm_v3_0_5_rdfifo_inst/inst_fifo_gen/gconvfifo.rf/grf.rf/ |
| PDCN-1569 | warning | 9 | 0 | 9 |  | LUT equation term check Used physical LUT pin 'A1' of cell dbg_hub/inst/BSCANID.u_xsdbm_id/SWITCH_N_EXT_BSCAN.bscan_switch/temp_curid[31]_i_1 (pin dbg_hub/inst/BSCANID.u_xsdbm_id/SWITCH_N_EXT_BSCAN.bs |
| Vivado_Tcl 4-920 | warning | 6 | 0 | 6 |  | Waiver ID 'CDC-1' -to list has >10000 objects! [smartconnect.xdc:4] |
| Synth 8-3848 | warning | 6 | 0 | 6 |  | Net ddr4_c in module/entity sys_ddr_0_phy_ddr4 does not have driver. [rtl/phy/sys_ddr_0_phy_ddr4.sv:177] |
| Synth 8-13161 | warning | 6 | 0 | 6 |  | RAM (inst/s01_nodesi_2/\s01_aw_node/inst /\inst_mi_handler/gen_normal_area.gen_fifo_req.inst_fifo_req/gen_xpm_memory_fifo.inst_fifo/gen_mem_rep[0].inst_xpm_memory /xpm_memory_base_inst/i_0) is optimiz |
| Constraints 18-550 | warning | 3 | 0 | 3 |  | Could not create 'DRIVE' constraint because net 'sys_i/ddr/inst/u_ddr4_mem_intfc/u_mig_ddr4_phy/cal_RESET_n[0]' is not directly connected to top level port. 'DRIVE' is ignored for synthesis but preser |
| Netlist 29-1115 | warning | 3 | 0 | 3 |  | Found multi-term driver net: sys_i/refbuf/U0/<const0>. |
| Vivado_Tcl 4-921 | warning | 2 | 0 | 2 |  | Waiver ID 'CDC-1' -to list should not be empty. [smartconnect.xdc:3] |
| TIMING-18 | warning | 2 | 0 | 2 |  | Missing input or output delay An input delay is missing on pcie_perstn relative to the rising and/or falling clock edge(s) of pcie_refclk_clk_p. |
| BD 41-3281 | warning | 1 | 0 | 1 |  | Instance '/xdma' is connected on both sides by SmartConnects and cannot automatically configure itself to match the endpoints. Please manually configure its AXI interfaces as required for the design. |
| Vivado_Tcl 4-939 | warning | 1 | 0 | 1 |  | Waiver ID 'TIMING-54' object list should not be empty (perhaps an invalid wildcard was used or only unsupported objects). [synth/sys_xdma_0_pcie4_ip_late.xdc:73] |
| Constraints 18-401 | warning | 1 | 0 | 1 |  | set_max_delay: 'sys_i/sc_mem/inst/s01_nodes/s01_w_node/inst/inst_mi_handler/gen_normal_area.gen_fifo_req.inst_fifo_req/gen_xpm_memory_fifo.inst_fifo/gen_mem_rep[0].inst_xpm_memory/xpm_memory_base_inst |
| Synth 8-3321 | warning | 1 | 0 | 1 |  | set_false_path : Empty through list for constraint at line 340 of sys_ddr_0.xdc. [sys_ddr_0.xdc:340] |
| Constraints 18-619 | warning | 1 | 0 | 1 |  | A clock with name 'IBUF_DS_P[0]' already exists, overwriting the previous clock with the same name. [sys_refbuf_0.xdc:4] |
| Synth 8-565 | warning | 1 | 0 | 1 |  | redefining clock 'IBUF_DS_P[0]' |
| Synth 8-10940 | warning | 1 | 0 | 1 |  | macro 'XPREG' is redefined [sys_xdma_0_pcie4_ip_pipe.v:66] |
| Synth 8-4767 | warning | 1 | 0 | 1 |  | Trying to implement RAM 'MemArray_reg' in registers. Block RAM or DRAM implementation is not possible; see log for reasons. [axidma_fifo.vh:1118] |
| Synth 8-7137 | warning | 1 | 0 | 1 |  | Register freq_cnt_o_reg in module gtwizard_ultrascale_v1_7_22_gtye4_cpll_cal_freq_counter has both Set and reset with same priority. This may cause simulation mismatches. Consider rewriting code  [gtw |
| Synth 8-6026 | warning | 1 | 0 | 1 |  | Ignoring keep related attribute (keep/mark_debug/dont_touch) applied on memory |
| RTSTAT-10 | warning | 1 | 0 | 1 |  | No routable loads 556 net(s) have no routable loads. The problem bus(es) and/or net(s) are sys_i/ddr/inst/u_ddr4_mem_intfc/u_ddr_cal_riu/LMB_CE_riu, |

