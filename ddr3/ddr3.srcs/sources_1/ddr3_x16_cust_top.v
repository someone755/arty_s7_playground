`timescale 1ns / 1ps

module ddr3_x16_cust_top(

	input 	DDR3_CLK100,
	input 	[3:0]	SW,
	input	[3:0]	BTN,
	output	[3:0]	LED,
	
	input	UART_TXD_IN,
	output	UART_RXD_OUT,
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

wire	w_clk_ddr, w_clk_ddr_n;
wire	w_clk_ddr_90, w_clk_ddr_90_n;
wire	w_clk_div, w_clk_div_n;
wire	w_clk_idelayctrl;
clk_wiz_1 clkgen_ddr3ctrl_instance (
	// Clock out ports
	.clk_out1_ddr(w_clk_ddr),		// fast clock, in sync with DQS
	.clk_out1_ddr_n(w_clk_ddr_n),
	.clk_out2_ddr_90(w_clk_ddr_90),	// fast clock delayed by 90?, aligns to DQ
	.clk_out2_ddr_90_n(w_clk_ddr_90_n),
	.clk_out3_ref(w_clk_idelayctrl),// IDELAYCTRL, 200 MHz
	.clk_out4_div(w_clk_div),		// slow clock is 1:2 slower
	.clk_out4_div_n(w_clk_div_n),
	// Status and control signals
	.reset(1'b0),
	.locked(),
	// Clock in ports
	.clk_in1(DDR3_CLK100)
);
wire	w_init_done;
wire	[63:0]	w64_rddata;
wire	w_data_valid;
assign LED[0] = w_init_done;
 
reg	[2:0]	r3_bank = 3'b0;
reg [13:0]	r14_row = 14'b0;
reg	[9:0]	r10_col = 10'b0;
reg	[127:0]	r128_wrdata = {128{1'b1}};
reg	r_phy_cmd_en = 1'b0;
reg	r_phy_cmd_sel = 1'b0;
reg r_phy_rst = 1'b0;
ddr3_x16_phy_cust #(
	.p_IDELAY_INIT_DQS(2),//31,
	.p_IDELAY_INIT_DQ(0),
	.p_DDR_FREQ_MHZ(300)
) phy_instance (
	.i2_iserdes_ce(SW[1:0]),//2'b11),//	input	[1:0]	i2_iserdes_ce,

	.i_clk_ddr(w_clk_ddr),//	input	i_clk_ddr,	// memory bus clock frequency
	.i_clk_ddr_n(w_clk_ddr_n),
	.i_clk_ddr_90(w_clk_ddr_90),//	input	i_clk_ddr_90,	// same but delayed by 90?, used to generate output DQ from OSERDES
	.i_clk_ddr_90_n(w_clk_ddr_90_n),
	.i_clk_ref(w_clk_idelayctrl),//	input	i_clk_ref,	// 200 MHz, used for IDELAYCTRL, which controls taps for input DQS IDELAY
	.i_clk_div(w_clk_div),//	input	i_clk_div,	// half of bus clock frequency
	.i_clk_div_n(w_clk_div_n),
		
	.i_phy_rst(SW[3]),//	input	i_phy_rst,	// active high reset for ODDR, OSERDES, ISERDES, IDELAYCTRL, hold HIGH until all clocks are generated
		
	.i_phy_cmd_en(r_phy_cmd_en),//	input	i_phy_cmd_en,	// Active high strobe for inputs: cmd_sel, addr, 
	.i_phy_cmd_sel(r_phy_cmd_sel),//	input	i_phy_cmd_sel,	// Command for current request: 'b0 = WRITE || 'b1 = READ
	.o_phy_cmd_full(w_fifo_full),
	//	output	o_phy_cmd_rdy,	// Active high indicates UI ready to accept commands
	
	.in_phy_bank(r3_bank),//	input	[p_BANK_W-1:0]	in_phy_bank,
	.in_phy_row(r14_row),//	input	[p_ROW_W-1:0]	in_phy_row,
	.in_phy_col(r10_col),//	input	[p_COL_W-1:0]	in_phy_col,
	.in_phy_wrdata(r128_wrdata),//	input	[(8*p_DQ_W)-1:0]	in_phy_wrdata,	// eight words of write data for OSERDES (out of 8 for a total of BL8)
	.i8_phy_wrdm(8'b0),//	input	[7:0]	i8_phy_wrdm,	// write data mask input, 1 bit per word in burst
	.on_phy_rddata(w64_rddata),//	output	[(4*p_DQ_W)-1:0]	on_phy_rddata,	// four words of read data from ISERDES (out of 8 for a total of BL8)
	.o_phy_rddata_valid(w_data_valid),//output	o_phy_rddata_valid, // output data valid flag
	//	output	o_phy_rddata_end,	// last burst of read data
		
	.o_phy_init_done(w_init_done),//	output	o_init_done,
	
	.in_dqs_delay_inc(2'b00),//input	[(p_DQ_W/8)-1:0]	in_dqs_delay_inc,	// DQS IDELAY tap control
	.in_dqs_delay_ce(2'b00),//input	[(p_DQ_W/8)-1:0]	in_dqs_delay_ce,
	
	.in_dq_delay_inc(2'b00),//input	[(p_DQ_W/8)-1:0]	in_dq_delay_inc,	// DQ IDELAY tap control
	.in_dq_delay_ce(2'b00),//input	[(p_DQ_W/8)-1:0]	in_dq_delay_ce,
			
	//	 CONNECTION TO DRAM by PHY CORE
	.ion_ddr_dq(ddr3_dq),//	inout	[p_DQ_W-1:0]	ion_ddr_dq,
	.ion_ddr_dqs_p(ddr3_dqs_p),//	inout	[(p_DQ_W/8)-1:0]	ion_ddr_dqs_p,
	.ion_ddr_dqs_n(ddr3_dqs_n),//	inout	[(p_DQ_W/8)-1:0]	ion_ddr_dqs_n,
		
	.on_ddr_addr(ddr3_addr),//	output	[p_ADDR_W-1:0]	on_ddr_addr,
	
	.o_ddr_ck_p(ddr3_ck_p),//	output	o_ddr_ck_p,
	.o_ddr_ck_n(ddr3_ck_n),//	output	o_ddr_ck_n,
		
	//	 CONNECTION TO DRAM by LOGIC CORE (ADDR, BANK, CS/RAS/CAS/WE, ODT, CKE, UDM/LDM)
	.on_ddr_dm(ddr3_dm),//	output	[(p_DQ_W/8)-1:0]	on_ddr_dm,
	.on_ddr_bank(ddr3_ba),//	output	[p_BANK_W-1:0]	on_ddr_bank,
	
	.o_ddr_nrst(ddr3_reset_n),//	output	o_ddr_nrst,
	.o_ddr_cke(ddr3_cke),//	output	o_ddr_cke,
	.o_ddr_ncs(ddr3_cs_n),//	output	o_ddr_ncs,
	.o_ddr_nras(w_ddr3_ras_n),//	output	o_ddr_nras,
	.o_ddr_ncas(w_ddr3_cas_n),//	output	o_ddr_ncas,
	.o_ddr_nwe(w_ddr3_we_n),//	output	o_ddr_nwe,
	.o_ddr_odt(ddr3_odt)//	output	o_ddr_odt
);
wire w_ddr3_ras_n;
wire w_ddr3_cas_n;
wire w_ddr3_we_n;
assign ddr3_ras_n = w_ddr3_ras_n;
assign ddr3_cas_n = w_ddr3_cas_n;
assign ddr3_we_n = w_ddr3_we_n;




reg r2_num_words = 'd0;
reg r2_word_ctr = 2'd0;
reg btn0_prev, btn1_prev, btn2_prev, btn3_prev = 1'b0;
reg startflag = 1'b0;
reg	[5:0]	rn_test_tmr = 'b0;
reg [3:0]	state = 0;
always @(posedge w_clk_div) begin: wr_rd_test
	btn3_prev <= BTN[3];
	btn2_prev <= BTN[2];
	btn1_prev <= BTN[1];
	btn0_prev <= BTN[0];
	r_phy_cmd_en <= 1'b0;
	rn_test_tmr <= rn_test_tmr + 1;
	case (state)
	0: begin
		if (!btn0_prev && BTN[0]) begin
			r3_bank <= 3'b100;
			r14_row <= 14'd13;
			r10_col <= 10'd16;
			r128_wrdata <= 'h0011_2233_4455_6677_8899_aabb_ccdd_eeff;
			r_phy_cmd_en <= 1'b1;
			r_phy_cmd_sel <= 1'b0;
			
			rn_test_tmr <= 'd0;
			state <= 1;
		end
	end
	1: begin
		if (rn_test_tmr == 'd16) begin
			r3_bank <= 3'b101;
			r14_row <= 14'd14;
			r10_col <= 10'd8;
			r128_wrdata <= 'h0102_0304_0506_0708_090a_0b0c_0d0e_0f00;
			r_phy_cmd_en <= 1'b1;
			r_phy_cmd_sel <= 1'b0;
			
			rn_test_tmr = 'd0;
			state <= 2;
		end
	end
	2: begin
		if (rn_test_tmr == 'd16) begin
			r_phy_cmd_en <= 1'b1;
			r_phy_cmd_sel <= 1'b1;
			//r128_wrdata <= 'h0102_0304_0506_0708_090a_0b0c_0d0e_0f00;
			r3_bank <= 3'b100;
			r14_row <= 14'd13;
			r10_col <= 10'd16;
			
			rn_test_tmr <= 'd0;
			state <= 3;
		end
	end
	3: begin
		if (rn_test_tmr == 'd16) begin
			r_phy_cmd_en <= 1'b1;
			r_phy_cmd_sel <= 1'b1;
			//r128_wrdata <= 'h0102_0304_0506_0708_090a_0b0c_0d0e_0f00;
			r3_bank <= 3'b101;
			r14_row <= 14'd14;
			r10_col <= 10'd8;
			
			rn_test_tmr <= 'd0;
			state <= 0;
		end	
	end
	default: if (SW[3]) state <= 0;
	endcase
end
wire w_rd = (w_ddr3_ras_n && !w_ddr3_cas_n && w_ddr3_we_n) ? 1'b1 : 1'b0;
reg	r_rd_prev = 1'b0;
always @(w_clk_div) begin: butter
	r_rd_prev <= w_rd;
end
ila_ddr_cust ila_inst_ddr3 (
	.clk(w_clk_div),
	.probe0(w64_rddata),/*input [63 : 0]*/
	.probe1(r_rd_prev),/*input [2 : 0]*/
	.probe2(r128_wrdata),/*input [127 : 0]*/
	.probe3(w_ddr3_ras_n),
	.probe4(w_ddr3_cas_n),
	.probe5(w_ddr3_we_n),
	.probe6({btn0_prev, btn1_prev, w_data_valid, btn3_prev})
);
endmodule