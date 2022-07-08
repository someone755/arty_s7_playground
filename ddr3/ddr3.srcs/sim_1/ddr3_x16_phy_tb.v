`timescale 1ns / 1ps

`define SIMULATION
`define h_period	(500.0/300.0) // 1000/2/DDR_FERQ = half of ddr clock period

module ddr3_x16_phy_tb;

reg DDR3_CLK100 = 1'b0;
initial begin: clk_gen // 100 MHz
	forever begin
		#(5) // delay half period
		DDR3_CLK100 = ~DDR3_CLK100;
	end
end
initial begin: rst_deassert
	#(40*`h_period)
	r_phy_rst <= 1'b0;
end

reg	r_phy_rst = 1'b1;

reg	[16:0]	r16_dq_tb = 16'b0;
reg	[1:0]	r2_dqs_p_tb = 2'b0;
wire	[1:0]	r2_dqs_n_tb = {2{~r2_dqs_p_tb[0]}};
reg	RD = 1'b0;
reg r_calib_prev = 1'b1;
always @(posedge w_clk_ddr) begin: wait_for_calib
	r_calib_prev <= w_calib_done;
	if((w_calib_done == 1) & (r_calib_prev == 0)) begin
		#(/*9*/7*`h_period-0.4)
		RD <= 1;
		r2_dqs_p_tb<='b00;
		#(2*`h_period)//10
		r2_dqs_p_tb<='b11;
		r16_dq_tb<='h0000;
		#`h_period
		r2_dqs_p_tb<='b00;
		r16_dq_tb<='h0000;
		#`h_period
		r2_dqs_p_tb<='b11;
		r16_dq_tb<='h0000;
		#`h_period
		r2_dqs_p_tb<='b00;
		r16_dq_tb<='h0000;
		#`h_period
		r2_dqs_p_tb<='b11;
		r16_dq_tb<='h0101;
		#`h_period
		r2_dqs_p_tb<='b00;
		r16_dq_tb<='h0101;
		#`h_period
		r2_dqs_p_tb<='b11;
		r16_dq_tb<='h0101;
		#`h_period
		r2_dqs_p_tb<='b00;
		r16_dq_tb<='h0101;
		
		// WORD 2
		#`h_period
		r2_dqs_p_tb<='b11;
		r16_dq_tb<='h0000;
		#`h_period
		r2_dqs_p_tb<='b00;
		r16_dq_tb<='h0101;
		#`h_period
		r2_dqs_p_tb<='b11;
		r16_dq_tb<='h0000;
		#`h_period
		r2_dqs_p_tb<='b00;
		r16_dq_tb<='h0101;
		#`h_period
		r2_dqs_p_tb<='b11;
		r16_dq_tb<='h0101;
		#`h_period
		r2_dqs_p_tb<='b00;
		r16_dq_tb<='h0000;
		#`h_period
		r2_dqs_p_tb<='b11;
		r16_dq_tb<='h0101;
		#`h_period
		r2_dqs_p_tb<='b00;
		r16_dq_tb<='h0000;
		#`h_period
		RD <= 0;
		#(40*`h_period) $finish;
//		$stop;
	end
end


wire w_clk_ddr;
wire w_clk_ddr_90;
wire w_clk_idelayctrl;
wire w_clk_div;

clk_wiz_1 clkgen_ddr3ctrl_instance (
	// Clock out ports
	.clk_out1_ddr(w_clk_ddr),		// fast clock, in sync with DQS
	.clk_out2_ddr_90(w_clk_ddr_90),	// fast clock delayed by 90°, aligns to DQ
	.clk_out3_ref(w_clk_idelayctrl),// IDELAYCTRL, 200 MHz
	.clk_out4_div(w_clk_div),		// slow clock is 1:2 slower
	// Status and control signals
	.reset(1'b0),
	.locked(),
	// Clock in ports
	.clk_in1(DDR3_CLK100)
);
wire	[15:0]	w16_ddr_dq;
wire	[1:0]	w2_ddr_dqs_p;
wire	[1:0]	w2_ddr_dqs_n;

assign w16_ddr_dq = (RD==1) ? r16_dq_tb : {16{1'bz}};
assign w2_ddr_dqs_p = (RD==1) ? r2_dqs_p_tb : {2{1'bz}};
assign w2_ddr_dqs_n = (RD==1) ? r2_dqs_n_tb : {2{1'bz}};

wire	w_calib_done;
ddr3_x16_phy #(
	.p_DDR_FREQ_MHZ(80)
) phy_instance (
	//.o4_test_state(w4_test_state),
	.i_test_redo(1'b1),
	//.o256_test(w256_test),
	//.o64_iserdes(w64_iserdes),
	.i_clk_ddr(w_clk_ddr),//	input	i_clk_ddr,	// chip clock frequency
	.i_clk_ddr_90(w_clk_ddr_90),//	input	i_clk_ddr_90,	// same but delayed by 90°, used to generate output DQ from OSERDES
	.i_clk_ref(w_clk_idelayctrl),//	input	i_clk_ref,	// 200 MHz, used for IDELAYCTRL, which controls taps for input DQS IDELAY
		
	.i_clk_div(w_clk_div),//	input	i_clk_div,
		
	.i_phy_rst(r_phy_rst),//	input	i_phy_rst,	// active high reset for ODDR, OSERDES, ISERDES, IDELAYCTRL, hold HIGH until all clocks are generated
	
	.i14_phy_addr(14'b0),//	input	[13:0]	i14_phy_addr,
	.i128_phy_wrdata(128'b0),//	input	[127:0]	i128_phy_wrdata,	// eight words of write data for OSERDES (out of 8 for a total of BL8)
	//	output	[63:0]	o64_phy_rddata,	// four words of read data from ISERDES (out of 8 for a total of BL8)
	
	.o_calib_done(w_calib_done),
	
	//	// CONNECTION TO DRAM	
	.io16_ddr_dq(w16_ddr_dq),//	inout	[15:0]	io16_ddr_dq,
	.io2_ddr_dqs_p(w2_ddr_dqs_p),//	inout	[1:0]	io2_ddr_dqs_p,
	.io2_ddr_dqs_n(w2_ddr_dqs_n)//	inout	[1:0]	io2_ddr_dqs_n,
		
	//	output	[13:0]	o14_ddr_addr,
	
	//	output	o_ddr_ck_p,
	//	output	o_ddr_ck_n,
		
	//	// CONNECTION TO DRAM used by CTRL (ADDR, BANK, CS/RAS/CAS/WE, ODT, CKE, UDM/LDM)
	//	output	[1:0]	o2_ddr_dm,
	//	output	[2:0]	o3_ddr_bank,
	
	//	output	o_ddr_nrst,
	//	output	o_ddr_cke,
	//	output	o_ddr_ncs,
	//	output	o_ddr_nras,
	//	output	o_ddr_ncas,
	//	output	o_ddr_nwe,
	//	output	o_ddr_odt
);
endmodule
