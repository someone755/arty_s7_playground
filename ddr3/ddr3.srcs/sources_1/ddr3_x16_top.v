`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/29/2022 08:30:26 PM
// Design Name: 
// Module Name: ddr3_x16_top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module ddr3_x16_top(

	input DDR3_CLK100,
	// ### BEGIN DDR3 IO ###
	// Inouts
	inout	[15:0]	ddr3_dq,
	inout 	[1:0]	ddr3_dqs_n,
	inout	[1:0]	ddr3_dqs_p,
	
	// Outputs
	output	[13:0]	ddr3_addr,
	output	[2:0]	ddr3_ba,
	output	ddr3_ras_n,
	output	ddr3_cas_n,
	output	ddr3_we_n,
	output  ddr3_reset_n,
	output	[0:0]	ddr3_ck_p,
	output	[0:0]	ddr3_ck_n,
	output	[0:0]	ddr3_cke,
	output	[0:0]	ddr3_cs_n,
	output 	[1:0]	ddr3_dm,
	output	[0:0]	ddr3_odt
);
   
wire	w_clk_ddr;	// CTRL logic clock
wire	w_clk_ddr_270;
wire	w_clk_idelayctrl;
clk_wiz_1 clkgen_ddr3ctrl_instance (
	// Clock out ports
	.clk_out1_ddr(w_clk_ddr),
	.clk_out2_ddr_270(w_clk_ddr_270),
	.clk_out3_ref(w_clk_idelayctrl),
	// Status and control signals
	.reset(1'b0),
	.locked(w_pll_locked),
	// Clock in ports
	.clk_in1(DDR3_CLK100)
);

wire	[63:0]	w64_ctrl_rddata;
ddr3_x16_ctrl ctrl_instance(
	.i_clk_ddr(w_clk_ddr),
	.i_clk_ddr_270(w_clk_ddr_270),
	.i_clk_200(w_clk_idelayctrl),
	
	// Write data must be presented with write command
	.i128_ctrl_wrdata(128'hAAAA_AAAA_AAAA_AAAA_AAAA_AAAA_AAAA_AAAA),
	
	.o64_ctrl_rddata(w64_ctrl_rddata),
	.o_clk_ddr(),
	

	// CONNECTION TO DRAM pass-through to PHY (DQ, DQS, CK)
	.o_ddr_ck_p(ddr3_ck_p),
	.o_ddr_ck_n(ddr3_ck_n),	

	.io16_ddr_dq(ddr3_dq),
	.io2_ddr_dqs_p(ddr3_dqs_p),
	.io2_ddr_dqs_n(ddr3_dqs_n),
	// CONNECTION TO DRAM used by CTRL (ADDR, BANK, RAS/CAS/WE, ODT, CKE, UDM/LDM)
	// CS is tied HIGH in hardware
	.o2_ddr_dm(ddr3_dm),

	.o14_ddr_addr(ddr3_addr),
	.o3_ddr_bank(ddr3_ba),

	.o_ddr_nrst(ddr3_reset_n),
	.o_ddr_cke(ddr3_cke),
	.o_ddr_ncs(ddr3_cs_n),
	.o_ddr_nras(ddr3_ras_n),
	.o_ddr_ncas(ddr3_cas_n),
	.o_ddr_nwe(ddr3_we_n),
	.o_ddr_odt(ddr3_odt)
);

//ila_ddr_cust ila_inst_ddr3 (
//.clk(w_clk_ddr),
///*input [63 : 0]*/ .probe0(w64_ctrl_rddata),
///*input [127 : 0]*/ .probe1(128'b0),
///*input [0 : 0]*/ .probe2(ddr3_reset_n),
///*input [3 : 0]*/ .probe3({ddr3_cs_n,ddr3_we_n,ddr3_cas_n,ddr3_ras_n}),
///*input [0 : 0]*/ .probe4(ddr3_cke)
//);
endmodule
