`timescale 1ns / 1ps

`define SIMULATION
`define h_period	(500.0/350.0) // 1000/2/DDR_FERQ = half of ddr clock period

module ddr3_x16_phy_cust_tb;

reg DDR3_CLK100 = 1'b0;
initial begin: clk_gen // 100 MHz
	forever begin
		#(5) // delay half period
		DDR3_CLK100 = ~DDR3_CLK100;
	end
end
initial begin: rst_deassert
	#(`h_period)
	r_phy_rst <= 1'b1;
	#(4000*`h_period)
	r_phy_rst <= 1'b0;
end

reg	r_phy_rst = 1'b1;

reg	[16:0]	r16_dq_tb = 16'b0;
reg	[1:0]	r2_dqs_p_tb = 2'b0;
wire	[1:0]	r2_dqs_n_tb = {2{~r2_dqs_p_tb[0]}};
reg	RD = 1'b0;
reg r_init_prev = 1'b1;
always @(posedge w_clk_ddr) begin: wait_for_calib
	r_init_prev <= w_init_done;
	if((w_init_done == 1) & (r_init_prev == 0)) begin
		#(3*`h_period)
		//r_cmd_en <= 1'b1;
		#(40*`h_period)
		//r_cmd_en <= 1'b0;//1'b0; // TODO: Create fifo pattern generator.
		#(800*`h_period)
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
		#(400*`h_period) $finish;
//		$stop;
	end
end
reg fifo_fill = 1'b1;
/*always @(posedge w_clk_div) begin: fifo_pattern_generate
	if (fifo_fill == 1'b1) begin
		r_cmd_en <= 1'b1;
		r10_col <= r10_col + 'b1000;
		r128_wrdata <= r128_wrdata + 1;
	end else
		r_cmd_en <= 1'b0;
	if (w_fifo_full)
		fifo_fill <= 1'b0;
	//{r_cmd_sel, r3_bank, r14_row, r10_col, r128_wrdata, r8_wrdm} <= {1, 3'd0, 
end*/
reg btn0_prev = 1'b0;
reg btn1_prev = 1'b0;
reg btn2_prev = 1'b0;
reg btn3_prev = 1'b0;
reg btn0 = 1'b0;
reg btn1 = 1'b0;
reg btn2 = 1'b0;
reg btn3 = 1'b0;
reg startflag = 1'b0;
reg	[9:0]	rn_test_tmr = 'b0;
initial begin: btn_trig
	#(4000*`h_period)
	btn0 <= 1'b1;
	#(4000*`h_period)
	btn1 <= 1'b1;
	#(4000*`h_period)
	btn2 <= 1'b1;
	#(4000*`h_period)
	btn3 <= 1'b1;
end
always @(posedge w_clk_div) begin: wr_rd_test
	btn0_prev <= btn0;
	btn1_prev <= btn1;
	btn2_prev <= btn2;
	btn3_prev <= btn3;
	r_cmd_en <= 1'b0;
	if (!btn0_prev && btn0) begin
		r3_bank <= 3'b100;
		r14_row <= 14'd13;
		r10_col <= 10'd8;
		r128_wrdata <= 'h0011_2233_4455_6677_8899_aabb_ccdd_eeff;
		r_cmd_en <= 1'b1;
		r_cmd_sel <= 1'b0;
	end
	if (!btn1_prev && btn1) begin
		r3_bank <= 3'b101;
		r14_row <= 14'd14;
		r10_col <= 10'd16;
		r128_wrdata <= 'h0102_0304_0506_0708_090a_0b0c_0d0e_0f00;
		r_cmd_en <= 1'b1;
		r_cmd_sel <= 1'b0;
	end
	if (!btn2_prev && btn2) begin
		r3_bank <= 3'b100;
		r14_row <= 14'd13;
		r10_col <= 10'd8;
		//r128_wrdata <= 'h0011_2233_4455_6677_8899_aabb_ccdd_eeff;
		r_cmd_en <= 1'b1;
		r_cmd_sel <= 1'b1;
	end
	if (!btn3_prev && btn3) begin	
		r3_bank <= 3'b101;
		r14_row <= 14'd14;
		r10_col <= 10'd16;
		//r128_wrdata <= 'h0102_0304_0506_0708_090a_0b0c_0d0e_0f00;
		r_cmd_en <= 1'b1;
		r_cmd_sel <= 1'b1;
	end
end
/*always @(posedge w_clk_div) begin: wr_rd_test
	//btn1_prev <= btn;//BTN[1];
	r_cmd_en <= 1'b0;
	if (!btn1_prev && btn1 && !startflag && !r_phy_rst) begin
		btn1_prev <= btn1;//BTN[1];
		startflag <= 1'b1;
		//r2_num_words <= SW[3:2];
		//r2_word_ctr <= 2'd0;
		//r_cmd_en <= 1'b1;
		r_cmd_sel <= 1'b0;
		//r128_wrdata <= r128_wrdata + 1'b1;
		rn_test_tmr <= 'b0;
	end
	if (startflag && !w_fifo_full)
		rn_test_tmr <= rn_test_tmr + 'd1;
	if (startflag && rn_test_tmr < 'd30 && !w_fifo_full) begin
		r10_col <= r10_col + 8;
		r128_wrdata <= r128_wrdata + 1'b1;
		//startflag <= 1'b0;
		r_cmd_en <= 1'b1;
		r_cmd_sel <= 1'b0;//!(rn_test_tmr < 'd5);//~r_cmd_sel;//1'b0;
	end
	if (startflag && rn_test_tmr > 'd30 && !w_fifo_full) begin
		r128_wrdata <= r128_wrdata + 1'b1;
		//startflag <= 1'b0;
		r_cmd_en <= 1'b1;
		r_cmd_sel <= 1'b0;//!(rn_test_tmr < 'd5);//~r_cmd_sel;//1'b0;
	end
	if (startflag && rn_test_tmr%8==0) begin
		r3_bank <= r3_bank + 1;
	end
	if (startflag && rn_test_tmr%7==0) begin
		r14_row <= r14_row + 1;
	end
	if (rn_test_tmr == 'd30) begin
		//r128_wrdata <= r128_wrdata + 1'b1;
		startflag <= 1'b0;
		//r_cmd_en <= 1'b1;
		//r_cmd_sel <= 1'b0;
		rn_test_tmr <= 'd0;
	end
end*/


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

wire	w_fifo_full;
wire	w_init_done;
reg	r_cmd_en = 1'b0;
reg	r_cmd_sel = 1'b0;
reg	[2:0]	r3_bank	= 3'b0;
reg	[13:0]	r14_row	= 14'b0;
reg	[9:0]	r10_col	= 10'b0;
reg	[127:0]	r128_wrdata = {128{1'b1}};
reg	[7:0]	r8_wrdm	= 8'hff;
//wire	[163:0]	w164_fifoin = {r_cmd_sel, r3_bank, r14_row, r10_col, r128_wrdata, r8_wrdm};//{164{1'b1}};
ddr3_x16_phy_cust phy_instance (
	.i2_iserdes_ce(2'b11),//	input	[1:0]	i2_iserdes_ce,
	//	/*output	[12:0]	o13_init_ctr,
	//	output	[3:0]	o4_test_state,*/
	//	input	i_test_redo,
	//	/*output	[255:0]	o256_test,*/
	//.i164_fifoin(w164_fifoin),
		
	.i_clk_ddr(w_clk_ddr),//	input	i_clk_ddr,	// memory bus clock frequency
	.i_clk_ddr_90(w_clk_ddr_90),//	input	i_clk_ddr_90,	// same but delayed by 90°, used to generate output DQ from OSERDES
	.i_clk_ref(w_clk_idelayctrl),//	input	i_clk_ref,	// 200 MHz, used for IDELAYCTRL, which controls taps for input DQS IDELAY
	.i_clk_div(w_clk_div),//	input	i_clk_div,	// half of bus clock frequency
		
	.i_phy_rst(r_phy_rst),//	input	i_phy_rst,	// active high reset for ODDR, OSERDES, ISERDES, IDELAYCTRL, hold HIGH until all clocks are generated
		
	.i_phy_cmd_en(r_cmd_en),//	input	i_phy_cmd_en,	// Active high strobe for inputs: cmd_sel, addr, 
	.i_phy_cmd_sel(r_cmd_sel),//	input	i_phy_cmd_sel,	// Command for current request: 'b0 = WRITE || 'b1 = READ
	.o_fifo_full(w_fifo_full),
	//	output	o_phy_cmd_rdy,	// Active high indicates UI ready to accept commands
	
	.in_phy_bank(r3_bank),//	input	[p_BANK_W-1:0]	in_phy_bank,
	.in_phy_row(r14_row),//	input	[p_ROW_W-1:0]	in_phy_row,
	.in_phy_col(r10_col),//	input	[p_COL_W-1:0]	in_phy_col,
	.in_phy_wrdata(r128_wrdata),//	input	[(8*p_DQ_W)-1:0]	in_phy_wrdata,	// eight words of write data for OSERDES (out of 8 for a total of BL8)
	.i8_phy_wrdm(r8_wrdm),//	input	[7:0]	i8_phy_wrdm,	// write data mask input, 1 bit per word in burst
	//	output	[(4*p_DQ_W)-1:0]	on_phy_rddata,	// four words of read data from ISERDES (out of 8 for a total of BL8)
	//	output	o_phy_rddata_valid, // output data valid flag
	//	output	o_phy_rddata_end,	// last burst of read data
		
	.o_init_done(w_init_done),//	output	o_init_done,
			
	//	 CONNECTION TO DRAM by PHY CORE
	.ion_ddr_dq(w16_ddr_dq),//	inout	[p_DQ_W-1:0]	ion_ddr_dq,
	.ion_ddr_dqs_p(w2_ddr_dqs_p),//	inout	[(p_DQ_W/8)-1:0]	ion_ddr_dqs_p,
	.ion_ddr_dqs_n(w2_ddr_dqs_n)//	inout	[(p_DQ_W/8)-1:0]	ion_ddr_dqs_n,
		
	//	output	[p_ADDR_W-1:0]	on_ddr_addr,
	
	//	output	o_ddr_ck_p,
	//	output	o_ddr_ck_n,
		
	//	 CONNECTION TO DRAM by LOGIC CORE (ADDR, BANK, CS/RAS/CAS/WE, ODT, CKE, UDM/LDM)
	//	output	[(p_DQ_W/8)-1:0]	on_ddr_dm,
	//	output	[p_BANK_W-1:0]	on_ddr_bank,
	
	//	output	o_ddr_nrst,
	//	output	o_ddr_cke,
	//	output	o_ddr_ncs,
	//	output	o_ddr_nras,
	//	output	o_ddr_ncas,
	//	output	o_ddr_nwe,
	//	output	o_ddr_odt
);
endmodule