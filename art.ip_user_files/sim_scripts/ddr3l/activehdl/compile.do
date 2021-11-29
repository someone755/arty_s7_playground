vlib work
vlib activehdl

vlib activehdl/xpm
vlib activehdl/xil_defaultlib

vmap xpm activehdl/xpm
vmap xil_defaultlib activehdl/xil_defaultlib

vlog -work xpm  -sv2k12 \
"C:/Xilinx/Vivado/2019.2/data/ip/xpm/xpm_cdc/hdl/xpm_cdc.sv" \

vcom -work xpm -93 \
"C:/Xilinx/Vivado/2019.2/data/ip/xpm/xpm_VCOMP.vhd" \

vlog -work xil_defaultlib  -v2k5 \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/clocking/mig_7series_v4_2_clk_ibuf.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/clocking/mig_7series_v4_2_infrastructure.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/clocking/mig_7series_v4_2_iodelay_ctrl.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/clocking/mig_7series_v4_2_tempmon.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/controller/mig_7series_v4_2_arb_mux.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/controller/mig_7series_v4_2_arb_row_col.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/controller/mig_7series_v4_2_arb_select.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/controller/mig_7series_v4_2_bank_cntrl.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/controller/mig_7series_v4_2_bank_common.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/controller/mig_7series_v4_2_bank_compare.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/controller/mig_7series_v4_2_bank_mach.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/controller/mig_7series_v4_2_bank_queue.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/controller/mig_7series_v4_2_bank_state.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/controller/mig_7series_v4_2_col_mach.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/controller/mig_7series_v4_2_mc.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/controller/mig_7series_v4_2_rank_cntrl.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/controller/mig_7series_v4_2_rank_common.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/controller/mig_7series_v4_2_rank_mach.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/controller/mig_7series_v4_2_round_robin_arb.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/ecc/mig_7series_v4_2_ecc_buf.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/ecc/mig_7series_v4_2_ecc_dec_fix.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/ecc/mig_7series_v4_2_ecc_gen.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/ecc/mig_7series_v4_2_ecc_merge_enc.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/ecc/mig_7series_v4_2_fi_xor.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/ip_top/mig_7series_v4_2_memc_ui_top_std.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/ip_top/mig_7series_v4_2_mem_intfc.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_byte_group_io.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_byte_lane.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_calib_top.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_if_post_fifo.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_mc_phy.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_mc_phy_wrapper.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_of_pre_fifo.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_4lanes.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_ck_addr_cmd_delay.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_dqs_found_cal.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_dqs_found_cal_hr.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_init.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_ocd_cntlr.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_ocd_data.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_ocd_edge.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_ocd_lim.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_ocd_mux.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_ocd_po_cntlr.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_ocd_samp.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_oclkdelay_cal.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_prbs_rdlvl.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_rdlvl.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_tempmon.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_top.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_wrcal.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_wrlvl.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_phy_wrlvl_off_delay.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_prbs_gen.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_ddr_skip_calib_tap.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_poc_cc.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_poc_edge_store.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_poc_meta.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_poc_pd.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_poc_tap_base.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/phy/mig_7series_v4_2_poc_top.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/ui/mig_7series_v4_2_ui_cmd.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/ui/mig_7series_v4_2_ui_rd_data.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/ui/mig_7series_v4_2_ui_top.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/ui/mig_7series_v4_2_ui_wr_data.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/ddr3l_mig_sim.v" \
"../../../../../art_ip/ddr3l/ddr3l/user_design/rtl/ddr3l.v" \

vlog -work xil_defaultlib \
"glbl.v"

