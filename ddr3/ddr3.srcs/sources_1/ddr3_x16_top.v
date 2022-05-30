`timescale 1ns / 1ps

module ddr3_x16_top(

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

wire	w_clk_ddr;
wire	w_clk_ddr_90;
wire	w_clk_div;
wire	w_clk_idelayctrl;
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
wire w_calib_done;
wire	[255:0]	w256_test;	// ideally 'h0000000000000000_8080808080808080_8080000080800000_0000808000008080
wire	[63:0]	w64_iserdes;
wire	[3:0]	w4_test_state;
wire	[12:0]	w13_init_ctr;
assign LED[0] = w_calib_done;
assign LED[1] = (w256_test == 256'b0);
assign LED[2] = (w256_test != 256'b0);
 

ddr3_x16_phy phy_instance (
	.i2_iserdes_ce({SW[1],SW[0]}),
	.o13_init_ctr(w13_init_ctr),
	.o4_test_state(w4_test_state),
	.i_test_redo(BTN[0]),
	.o256_test(w256_test),
	.o64_iserdes(w64_iserdes),
	.i_clk_ddr(w_clk_ddr),			//	input	i_clk_ddr,	// chip clock frequency
	.i_clk_ddr_90(w_clk_ddr_90),	//	input	i_clk_ddr_90,	// same but delayed by 90°, used to generate output DQ from OSERDES
	.i_clk_ref(w_clk_idelayctrl),	//	input	i_clk_ref,	// 200 MHz, used for IDELAYCTRL, which controls taps for input DQS IDELAY
		
	.i_clk_div(w_clk_div),			//	input	i_clk_div,
		
	.i_phy_rst(1'b0),				//	input	i_phy_rst,	// active high reset for ODDR, OSERDES, ISERDES, IDELAYCTRL, hold HIGH until all clocks are generated
	
	.i14_phy_addr(14'b0),	//	input	[13:0]	i14_phy_addr,
	.i128_phy_wrdata(128'b0),	//	input	[127:0]	i128_phy_wrdata,	// eight words of write data for OSERDES (out of 8 for a total of BL8)
	.o64_phy_rddata(),//	output	[63:0]	o64_phy_rddata,	// four words of read data from ISERDES (out of 8 for a total of BL8)
	
	.o_calib_done(w_calib_done),
	
	//	// CONNECTION TO DRAM	
	.io16_ddr_dq(ddr3_dq),		//	inout	[15:0]	io16_ddr_dq,
	.io2_ddr_dqs_p(ddr3_dqs_p),	//	inout	[1:0]	io2_ddr_dqs_p,
	.io2_ddr_dqs_n(ddr3_dqs_n),	//	inout	[1:0]	io2_ddr_dqs_n,
		
	.o14_ddr_addr(ddr3_addr),	//	output	[13:0]	o14_ddr_addr,
	
	.o_ddr_ck_p(ddr3_ck_p),		//	output	o_ddr_ck_p,
	.o_ddr_ck_n(ddr3_ck_n),		//	output	o_ddr_ck_n,
		
	//	// CONNECTION TO DRAM used by CTRL (ADDR, BANK, CS/RAS/CAS/WE, ODT, CKE, UDM/LDM)
	.o2_ddr_dm(ddr3_dm),	//	output	[1:0]	o2_ddr_dm,
	.o3_ddr_bank(ddr3_ba),	//	output	[2:0]	o3_ddr_bank,
	
	.o_ddr_nrst(ddr3_reset_n),	//	output	o_ddr_nrst,
	.o_ddr_cke(ddr3_cke),	//	output	o_ddr_cke,
	.o_ddr_ncs(ddr3_cs_n),	//	output	o_ddr_ncs,
	.o_ddr_nras(ddr3_ras_n),	//	output	o_ddr_nras,
	.o_ddr_ncas(ddr3_cas_n),	//	output	o_ddr_ncas,
	.o_ddr_nwe(ddr3_we_n),	//	output	o_ddr_nwe,
	.o_ddr_odt(ddr3_odt)	//	output	o_ddr_odt
);
wire w_uart_tx_rdy;
wire w_uart_tx_byte_done;
reg r_uart_tx_send_en = 1'b0;
reg [7:0] r8_uart_tx_data;
UART_TX_CTRL #(
	.p_BAUDRATE(9600),
	.p_CLK_FREQ(200_000_000)//25_000_000)
)
uart_tx_instance (
	.IN_UART_TX_SEND(r_uart_tx_send_en),
	.IN8_UART_TX_DATA(r8_uart_tx_data),
	.IN_CLK(w_clk_div),
	
	.OUT_UART_TX_READY(w_uart_tx_rdy),
	.OUT_UART_TX_BYTE_DONE(w_uart_tx_byte_done),
	.OUT_UART_TX_LINE(UART_RXD_OUT)
);

reg	[4:0]	r5_uart_byte_index = 'b11111;
reg	[1:0]	rn_uart_tx = 'b0;
reg	[255:0]	r256_test_buff = {255{1'b1}};
reg	[10:0]	r11_calib_tmr = 11'b0;
always @(posedge w_clk_div) begin: uart_tx
	if (w_calib_done && (rn_uart_tx == 'd0))
		r11_calib_tmr <= r11_calib_tmr + 1;
		
	case (rn_uart_tx)
	'd0: begin
		if (r11_calib_tmr == 'd11) begin
			r256_test_buff <= w256_test;
			rn_uart_tx <= 'd1;
		end
	end
	'd1: begin
		r8_uart_tx_data <= r256_test_buff[r5_uart_byte_index*8 +: 8];
		r_uart_tx_send_en <= 1;
		rn_uart_tx <= 'd2;
	end
	'd2: begin
		r_uart_tx_send_en <= 1'b0;
		if (w_uart_tx_byte_done) begin
			r5_uart_byte_index <= r5_uart_byte_index - 1; // always increment, no resets, overflow
			if (r5_uart_byte_index == 5'b00000) // rd buffer sent
				rn_uart_tx <= 'd3;
			else // rd buffer not sent, setup next buffer byte
				rn_uart_tx <= 'd1;
		end
	end
	'd3: begin
		r_uart_tx_send_en <= 1'b0;
		r5_uart_byte_index <= 'b11111;
		r11_calib_tmr <= 11'b0;
		if (BTN[0] == 1)
			rn_uart_tx <= 'd0;
	end
	default: ;
	endcase
end
ila_ddr_cust ila_inst_ddr3 (
	.clk(w_clk_div),
/*input [255 : 0]*/	//.probe0(256'b0),//w256_test),
/*input [63 : 0]*/	.probe0(w64_iserdes),
/*input [0 : 0]*/	.probe1(w_calib_done),
/*input [3 : 0]*/	.probe2(w4_test_state),
/*input [12 : 0]*/	.probe3(w13_init_ctr),
/*input [4 : 0]*/	.probe4(r5_uart_byte_index)
);
endmodule
