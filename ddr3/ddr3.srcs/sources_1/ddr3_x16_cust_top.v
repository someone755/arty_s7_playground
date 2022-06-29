`timescale 1ns / 1ps

module ddr3_x16_cust_top(

	input 	DDR3_CLK100,
	input 	[3:0]	SW,
	input	[3:0]	BTN,
	output	[3:0]	LED,
	output	[2:0]	RGBLED0,
	output	[2:0]	RGBLED1,
	
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
wire	w_pll_locked;
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
	.locked(w_pll_locked),
	// Clock in ports
	.clk_in1(DDR3_CLK100)
);
localparam lp_DDR_FREQ = 124;
localparam nCK_PER_CLK = 2;

/* uart clock signal */
wire w_uart_clk;
assign w_uart_clk = w_clk_div;
localparam lp_UART_CLK_FREQ = lp_DDR_FREQ*500_000;
localparam lp_UART_BAUDRATE = 3_000_000;

wire	w_init_done;
wire	[127:0]	w128_rddata;
wire	w_data_valid;
assign LED[0] = w_init_done;
assign LED[1] = w_pll_locked;
assign LED[2] = w_idelay_rdy;
 
reg	[2:0]	r3_bank = 3'b0;
reg [13:0]	r14_row = 14'b0;
reg	[9:0]	r10_col = 10'hfff;
reg	[127:0]	r128_wrdata = {128{1'b1}};
reg	r_phy_cmd_en = 1'b0;
reg	r_phy_cmd_sel = 1'b0;
reg r_phy_rst = 1'b0;
reg	[1:0]	r2_dq_delay_ce = 2'b0;
reg	[1:0]	r2_dq_delay_inc = 2'b0;
reg	[1:0]	r2_dqs_delay_ce = 2'b0;
reg	[1:0]	r2_dqs_delay_inc = 2'b0;
wire	[9:0]	w10_dq_delay_cnt;
wire	[9:0]	w10_dqs_delay_cnt;
wire w_idelay_rdy;

wire w_ddr3_ras_n;
wire w_ddr3_cas_n;
wire w_ddr3_we_n;
assign ddr3_ras_n = w_ddr3_ras_n;
assign ddr3_cas_n = w_ddr3_cas_n;
assign ddr3_we_n = w_ddr3_we_n;

wire [2:0] w_ba;
wire [13:0] w_row;
wire [9:0] w_col;

wire [63:0]	w64_iserdes;

ddr3_x16_phy_cust #(
	.p_IDELAY_INIT_DQS(5),//31,
	.p_IDELAY_INIT_DQ(0),
	.p_DDR_FREQ_MHZ(lp_DDR_FREQ),
	.p_RD_DELAY(3)
) phy_instance (
	.on_iserdes_par(w64_iserdes),
	.i2_iserdes_ce(SW[1:0]),//2'b11),//	input	[1:0]	i2_iserdes_ce,

	.i_clk_ddr(w_clk_ddr),//	input	i_clk_ddr,	// memory bus clock frequency
	.i_clk_ddr_n(w_clk_ddr_n),
	.i_clk_ddr_90(w_clk_ddr_90),//	input	i_clk_ddr_90,	// same but delayed by 90?, used to generate output DQ from OSERDES
	.i_clk_ddr_90_n(w_clk_ddr_90_n),
	.i_clk_ref(w_clk_idelayctrl),//	input	i_clk_ref,	// 200 MHz, used for IDELAYCTRL, which controls taps for input DQS IDELAY
	.i_clk_div(w_clk_div),//	input	i_clk_div,	// half of bus clock frequency
	.i_clk_div_n(w_clk_div_n),
		
	.i_phy_rst(r_phy_rst),//	input	i_phy_rst,	// active high reset for ODDR, OSERDES, ISERDES, IDELAYCTRL, hold HIGH until all clocks are generated
		
	.i_phy_cmd_en(r_phy_cmd_en),//	input	i_phy_cmd_en,	// Active high strobe for inputs: cmd_sel, addr, 
	.i_phy_cmd_sel(r_phy_cmd_sel),//	input	i_phy_cmd_sel,	// Command for current request: 'b0 = WRITE || 'b1 = READ
	.o_phy_cmd_full(w_fifo_full),
	//	output	o_phy_cmd_rdy,	// Active high indicates UI ready to accept commands
	
	.in_phy_bank(w_ba),//	input	[p_BANK_W-1:0]	in_phy_bank,
	.in_phy_row(w_row),//	input	[p_ROW_W-1:0]	in_phy_row,
	.in_phy_col(w_col),//	input	[p_COL_W-1:0]	in_phy_col,
	.in_phy_wrdata(r128_wrdata),//	input	[(8*p_DQ_W)-1:0]	in_phy_wrdata,	// eight words of write data for OSERDES (out of 8 for a total of BL8)
	.i8_phy_wrdm(8'b0),//	input	[7:0]	i8_phy_wrdm,	// write data mask input, 1 bit per word in burst
	.on_phy_rddata(w128_rddata),//	output	[(4*p_DQ_W)-1:0]	on_phy_rddata,	// four words of read data from ISERDES (out of 8 for a total of BL8)
	.o_phy_rddata_valid(w_data_valid),//output	o_phy_rddata_valid, // output data valid flag
	//	output	o_phy_rddata_end,	// last burst of read data
		
	.o_phy_init_done(w_init_done),//	output	o_init_done,
	.o_phy_idelay_rdy(w_idelay_rdy),
	
	.in_dqs_delay_inc(r2_dqs_delay_inc),//input	[(p_DQ_W/8)-1:0]	in_dqs_delay_inc,	// DQS IDELAY tap control
	.in_dqs_delay_ce(r2_dqs_delay_ce),//input	[(p_DQ_W/8)-1:0]	in_dqs_delay_ce,
	
	.in_dq_delay_inc(r2_dq_delay_inc),//input	[(p_DQ_W/8)-1:0]	in_dq_delay_inc,	// DQ IDELAY tap control
	.in_dq_delay_ce(r2_dq_delay_ce),//input	[(p_DQ_W/8)-1:0]	in_dq_delay_ce,
	
	.on_dqs_idelay_cnt(w10_dqs_delay_cnt),//output	[(p_DQ_W/8)*5-1:0]	on_dqs_idelay_cnt,	// IDELAY tap value
	.on_dq_idelay_cnt(w10_dq_delay_cnt),//output	[(p_DQ_W/8)*5-1:0]	on_dq_idelay_cnt,
			
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

localparam lp_RST_CTR_INITVAL = 50000;
reg	[$clog2(lp_RST_CTR_INITVAL)-1:0] rn_rst_ctr;
always @(posedge w_clk_div) begin: rst_ctrl
	if (SW[3] || !w_pll_locked) begin
		rn_rst_ctr <= lp_RST_CTR_INITVAL;
		r_phy_rst <= 1'b1;
	end else if (rn_rst_ctr > 0)
		rn_rst_ctr <= rn_rst_ctr - 1'b1;
	else
		r_phy_rst <= 1'b0;
end

/** BEGIN UART_RX module */
	reg r_uart_rx_en = 1'b0;
	wire w_uart_rx_done;
	wire [7:0] w8_uart_rx_data;
UART_RX_CTRL #(
	.p_BAUDRATE(lp_UART_BAUDRATE),
	.p_CLK_FREQ(lp_UART_CLK_FREQ)
	)
uart_rx_instance (
	.IN_CLK(w_uart_clk),
	.IN_UART_RX_ENABLE(r_uart_rx_en),
	.IN_UART_RX(UART_TXD_IN),
	
	.OUT_UART_RX_DONE(w_uart_rx_done),
	.OUT8_UART_RX_DATA(w8_uart_rx_data)
	);
/** End UART_RX module */
/** Begin UART_TX module */
	wire w_uart_tx_rdy;
	wire w_uart_tx_byte_done;
	reg r_uart_tx_send_en = 1'b0;
	reg [7:0] r8_uart_tx_data;
UART_TX_CTRL #(
	.p_BAUDRATE(lp_UART_BAUDRATE),
	.p_CLK_FREQ(lp_UART_CLK_FREQ)
)
uart_tx_instance (
	.IN_UART_TX_SEND(r_uart_tx_send_en),
	.IN8_UART_TX_DATA(r8_uart_tx_data),
	.IN_CLK(w_uart_clk),
	
	.OUT_UART_TX_READY(w_uart_tx_rdy),
	.OUT_UART_TX_BYTE_DONE(w_uart_tx_byte_done),
	.OUT_UART_TX_LINE(UART_RXD_OUT)
);
/* End UART_TX module */


reg r2_num_words = 'd0;
reg r2_word_ctr = 2'd0;
reg btn0_prev, btn1_prev, btn2_prev, btn3_prev = 1'b0;
reg startflag = 1'b0;
reg	[5:0]	rn_test_tmr = 'b0;
reg [3:0]	state = 0;

reg [4:0]	r5_dqs_delay_cnt = 5'b0;
reg	[4:0]	r5_dq_delay_cnt = 5'b0;

reg [2:0] r3_uart_state = 3'b000;
reg [3:0] r4_uart_byte_index = 4'b1111; // counts bytes in DDR read vector for uart tx

reg [127:0] r128_ddr_rd_buffer = 128'b0; // 128 bit (read) buffer from DDR
reg [127:0] r128_2_rx_buff [0:1]; // 2x128 bit rx buffer
reg [3:0] r4_rx_byte_index = 4'b1111; // counts 16 bytes across 128 bit words in 2x128 bit rx buffer
reg r1_rx_word_index = 1'b0; // counts 128 bit words in rx buffer
reg r1_rx_word_index_delay = 1'b0;
reg r1_rx_word_index_prev_read = 1'b0;

reg [14+10+3-1:0] app_addr = 'b0;
assign {w_ba, w_row, w_col} = app_addr;
reg	[14+10+3-1:0] r27_start_addr = 'b0;
reg	[14+10+3-1:0] r27_rd_addr_max, r27_end_addr = 'b0;

assign RGBLED1[0] = r4_rx_byte_index[0]; // blue 1 toggles with each byte received
assign RGBLED0[1] = (r1_rx_word_index_prev_read ^ r1_rx_word_index_delay) ? ~RGBLED0[1] : RGBLED0[1];
assign RGBLED1[2] = r1_rx_word_index; // red 1 toggles with each 128-bit word received
always @(posedge w_clk_div) begin: uart_state_machine
	if (w_uart_rx_done) begin
		r128_2_rx_buff[r1_rx_word_index][r4_rx_byte_index*8 +: 8] <= w8_uart_rx_data;
		r4_rx_byte_index <= r4_rx_byte_index - 1; // keep overflowing 16 byte counter
		if (r4_rx_byte_index == 4'b0000) begin
			r1_rx_word_index <= ~r1_rx_word_index;
		end
	end
	r_uart_rx_en <= 1'b1;
	
	r1_rx_word_index_delay <= r1_rx_word_index; // delay signal going into other process to avoid metastability/timing issues
	
case (r3_uart_state)
	'b000: begin // TAKE DATA FROM RX BUFFER, decide next state based on buffer contents
		r_uart_tx_send_en <= 1'b0;
		r_phy_cmd_en <= 1'b0;
		r_phy_cmd_sel <= 1'b0;
		
		r1_rx_word_index_prev_read <= r1_rx_word_index_delay; 	// check for signal edge even if edge happens
																// in another state by storing previous value
		if (r1_rx_word_index_prev_read ^ r1_rx_word_index_delay) begin // index_delay has toggled since last checked
			//RGBLED0[1] <= ~RGBLED0[1]; // green 0 toggles with each 128-bit word received -- should be about in sync with red 1
			if (r128_2_rx_buff[~r1_rx_word_index] == 128'h66666666_66666666_66666666_66666666) begin
				app_addr <= 27'b0;
				r_uart_tx_send_en <= 1'b1;
				r8_uart_tx_data <= 8'h8a;
			end else if ((r128_2_rx_buff[~r1_rx_word_index][127:64] == 64'h77777777_77777777) // ASCII 'w'
					//& (app_addr != r27_start_addr)
					//) begin // 16 0x66 bytes signify READ command
				
			//end else if (r128_2_rx_buff[~r1_rx_word_index][127:64] == 64'h61616161_61616161) begin
				// 25 0xaa bytes signify SET ADDR command
			&& (r128_2_rx_buff[~r1_rx_word_index][58:32] != r128_2_rx_buff[~r1_rx_word_index][26:0])
					) begin
				r3_uart_state <= 3'b011; // go to read/tx loop
				r27_start_addr <= r128_2_rx_buff[~r1_rx_word_index][58:32]; // send end addr for RD op
				r27_end_addr <= r128_2_rx_buff[~r1_rx_word_index][26:0]; // set start addr for RD op
				app_addr <= r128_2_rx_buff[~r1_rx_word_index][26:0]; // set address of next write
				
			end else if (r128_2_rx_buff[~r1_rx_word_index][127:64] == 64'h61616161_61616161) begin // ASCII '!'
				app_addr <= r128_2_rx_buff[~r1_rx_word_index][26:0];
			end else begin // all other data is written to DDR
				r128_wrdata <= r128_2_rx_buff[~r1_rx_word_index]; // wr data
				r3_uart_state <= 3'b001; // DDR WR
			end
		end
	end
	'b001: begin // END WR CMD, SIGNAL ENABLE
		r_phy_cmd_en <= 1'b1;
		r_phy_cmd_sel <= 1'b0;
		r3_uart_state <= 3'b010;
	end
	'b010: begin // STOP WRITE, INCREMENT ADDR, BACK TO IDLE
		r_phy_cmd_en <= 1'b0;
		
		app_addr <= app_addr + 8;
		//r_uart_tx_send_en <= 1'b1;
		r8_uart_tx_data <= 8'h8a;
		r3_uart_state <= 3'b000;
	end
	'b011: begin // DDR/TX STATE MACHINE BEGIN, addr start/end setup
		//r27_rd_addr_max <= {app_addr[26:3] - 1'b1, app_addr[2:0]};
		app_addr <= r27_start_addr;
		r4_uart_byte_index <= 4'b1111;
		r3_uart_state <= 3'b100;
	end
	'b100: begin // REQUEST DATA FROM DDR BLOCK
		// (is separate state for easier looping from 'b111)
		r_phy_cmd_en <= 1'b1;
		r_phy_cmd_sel <= 1'b1;
		r3_uart_state <= 3'b101;
	end
	'b101: begin // WAIT FOR DDR DATA VALID, BUFFER RD DATA
		r_phy_cmd_en <= 1'b0;
		if (w_data_valid) begin
			r128_ddr_rd_buffer <= w128_rddata;
			r3_uart_state <= 3'b110;
		end
	end
	'b110: begin // SETUP (NEXT) TX BYTE AND SEND ENABLE
		r8_uart_tx_data <= r128_ddr_rd_buffer[r4_uart_byte_index*8 +: 8];
		r_uart_tx_send_en <= 1;
		r3_uart_state <= 3'b111;
	end
	'b111: begin // WAIT FOR TX BYTE DONE
		r_uart_tx_send_en <= 0;
		if (w_uart_tx_byte_done) begin
			r4_uart_byte_index <= r4_uart_byte_index - 1; // always increment, no resets, overflow
			if (r4_uart_byte_index == 4'b0000) begin // rd buffer sent
				if (app_addr == r27_end_addr) begin // read finished
					app_addr <= r27_start_addr;
					r3_uart_state <= 3'b000;
				end else begin // read not finished, read from next addr
					app_addr <= app_addr + 8;
					r3_uart_state <= 3'b100;
				end
			end else begin // rd buffer not sent, setup next buffer byte
				r3_uart_state <= 3'b110;
			end
		end
	end
	default: ; // should not be reached
endcase	
end


/*always @(posedge w_clk_div) begin: wr_rd_test
	btn3_prev <= BTN[3];
	btn2_prev <= BTN[2];
	btn1_prev <= BTN[1];
	btn0_prev <= BTN[0];
	
	r_phy_cmd_en <= 1'b0;
	
	r2_dq_delay_ce <= 2'b00;
	r2_dqs_delay_ce <= 2'b00;
	//r2_dq_delay_inc <= 2'b0;
	//r2_dqs_delay_inc <= 2'b0;
	
	rn_test_tmr <= rn_test_tmr + 1;
	if (SW[2] == 1'b1) begin
		if (!btn0_prev && BTN[0]) begin
			r3_bank <= 3'b100;
			r14_row <= 14'd13;
			r10_col <= 10'd16;
			r128_wrdata <=
			//'h0000_ffff_0000_ffff_0000_ffff_0000_ffff;
			'h00_01_02_03_04_05_06_07_08_09_0a_0b_0c_0d_0e_0f;
			//'h00_18_20_38_40_58_60_78_80_98_a0_b8_c0_d8_e0_f8;
			//0011_2233_4455_6677_8899_aabb_ccdd_eeff;
			r_phy_cmd_en <= 1'b1;
			r_phy_cmd_sel <= 1'b0;
			
			rn_test_tmr <= 'd0;
			state <= 1;
		end
		if (!btn1_prev && BTN[1]) begin
			r3_bank <= 3'b101;
			r14_row <= 14'd14;
			r10_col <= 10'd8;
			r128_wrdata <=
			//'hffff_ffff_ffff_ffff_ffff_ffff_ffff_ffff;
			'h00_10_20_30_40_50_60_70_80_90_a0_b0_c0_d0_e0_f0;
			//'h00_81_02_83_04_85_06_87_08_89_0a_8b_0c_8d_0e_8f;
			//0102_0304_0506_0708_090a_0b0c_0d0e_0f00;
			r_phy_cmd_en <= 1'b1;
			r_phy_cmd_sel <= 1'b0;
			
			rn_test_tmr = 'd0;
			state <= 2;
		end
		if (!btn2_prev && BTN[2]) begin
			r_phy_cmd_en <= 1'b1;
			r_phy_cmd_sel <= 1'b1;
			//r128_wrdata <= 'h0102_0304_0506_0708_090a_0b0c_0d0e_0f00;
			r3_bank <= 3'b100;
			r14_row <= 14'd13;
			r10_col <= 10'd16;
			
			rn_test_tmr <= 'd0;
			state <= 3;
		end
		if (!btn3_prev && BTN[3]) begin
			r_phy_cmd_en <= 1'b1;
			r_phy_cmd_sel <= 1'b1;
			//r128_wrdata <= 'h0102_0304_0506_0708_090a_0b0c_0d0e_0f00;
			r3_bank <= 3'b101;
			r14_row <= 14'd14;
			r10_col <= 10'd8;
			
			rn_test_tmr <= 'd0;
			state <= 0;
		end	
	end else begin // if SW[2] == 1'b0
		if (!btn0_prev && BTN[0]) begin
			r2_dqs_delay_inc <= 2'b11;
			r2_dqs_delay_ce <= 2'b11;
			r5_dqs_delay_cnt <= r5_dqs_delay_cnt + 1'b1;
		end
		if (!btn1_prev && BTN[1]) begin
			r2_dqs_delay_inc <= 2'b00;
			r2_dqs_delay_ce <= 2'b11;
			r5_dqs_delay_cnt <= r5_dqs_delay_cnt - 1'b1;
		end
		if (!btn2_prev && BTN[2]) begin
			r2_dq_delay_inc <= 2'b11;
			r2_dq_delay_ce <= 2'b11;
			r5_dq_delay_cnt <= r5_dq_delay_cnt + 1'b1;
		end
		if (!btn3_prev && BTN[3]) begin
			r2_dq_delay_inc <= 2'b00;
			r2_dq_delay_ce <= 2'b11;
			r5_dq_delay_cnt <= r5_dq_delay_cnt - 1'b1;
		end
	end
end*/
wire w_rd = (w_ddr3_ras_n && !w_ddr3_cas_n && w_ddr3_we_n) ? 1'b1 : 1'b0;
reg	r_rd_prev = 1'b0;
wire w_btnpress = (!btn0_prev && BTN[0]) || (!btn1_prev && BTN[1]) || (!btn2_prev && BTN[2]) || (!btn3_prev && BTN[3]);
always @(w_clk_div) begin: butter
	r_rd_prev <= w_rd;
end
ila_ddr_cust ila_inst_ddr3 (
	.clk(w_clk_div),
	.probe0(w128_rddata),/*input [63 : 0]*/
	.probe1(w_btnpress),//r_rd_prev),//w_data_valid),/*input [2 : 0]*/
	.probe2(r128_wrdata),/*input [127 : 0]*/
	.probe3(w_data_valid),//w_ddr3_ras_n),
	.probe4(r1_rx_word_index_delay),//w_ddr3_cas_n),
	.probe5(w_uart_rx_done),//w_pll_locked),//w_ddr3_we_n),
	.probe6({btn0_prev, btn1_prev, btn2_prev, btn3_prev}),
	.probe7(w10_dqs_delay_cnt[9:5]),
	.probe8(w10_dqs_delay_cnt[4:0]),
	.probe9(w10_dq_delay_cnt[9:5]),
	.probe10(w10_dq_delay_cnt[4:0]),
	.probe11(r128_2_rx_buff[0]),
	.probe14(r128_2_rx_buff[1]),
	.probe12(w8_uart_rx_data),
	.probe13(w64_iserdes),
	.probe15(r3_uart_state),
	.probe16(app_addr),
	.probe17(r27_end_addr)
);
endmodule