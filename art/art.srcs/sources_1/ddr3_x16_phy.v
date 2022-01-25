`timescale 1ns / 1ps

module ddr3_x16_phy #(
	parameter	REFCLK_FREQUENCY	= 200.0
)(
	input	i_clk_ddr,	// chip clock frequency
	input	i_clk_ddr_90,	// same but delayed by 90°, used to generate output DQS
	input	i_clk_ref,	// 200 MHz, used for IDELAYCTRL, which controls taps for input DQS IDELAY
	
	input	i_phy_rst,	// active high reset for ODDR, OSERDES, ISERDES, IDELAYCTRL, hold HIGH until all clocks are generated
	input	i_phy_tristate_en,	// DQ/DQS IOBUF/IOBUFDS tristate (1 = Z, 0 = output enabled)
	
	input	[127:0]	i128_phy_wrdata,	// eight words of write data for OSERDES (out of 8 for a total of BL8)
	output	[63:0]	o64_phy_rddata,	// four words of read data from ISERDES (out of 8 for a total of BL8)
	
	input 	i_phy_dq_oserdes_en,	// DQ OSERDES enable
		
	input	i_phy_dqs_oddr_d1en,	// DQS ODDR D1 input
	
	// CONNECTION TO DRAM
	inout	[15:0]	io16_ddr_dq,
	inout	[1:0]	io2_ddr_dqs_p,
	inout	[1:0]	io2_ddr_dqs_n,
	
	output	o_ddr_ck_p,
	output	o_ddr_ck_n
);

wire	[1:0]	w2_dqs_rd;	// IOBUFDS -> IDELAY
wire	[1:0]	w2_dqs_rd_delayed;	// IDELAY -> ISERDES; Delayed to middle of data eye
wire	[1:0]	w2_dqs_wr;	// ODDR -> IOBUFDS
wire	[1:0]	w2_dqs_oddr_d1en;	// Connected to ODDR D1 input. D2 is 1'b0.
									// 1: ODDR output is i_clk_ddr_90
									// 0: ODDR output is 1'b0.
assign w2_dqs_oddr_d1en = {(2){i_phy_dqs_oddr_d1en}};

wire	[1:0]	w2_tristate_en;	// DQ IOBUF and DQS IOBUFDS tristate
assign w2_tristate_en = {(2){i_phy_tristate_en}};

wire	[15:0]	w16_dq_wr;	// OSERDES -> IOBUF
wire	[15:0]	w16_dq_rd;	// IOBUF -> ISERDES

genvar i; // loop variable for generate blocks

/////////////////////////////////////////////////
// DDR CLOCK DIFFERENTIAL OUTPUT BUFFER
/////////////////////////////////////////////////
OBUFDS #(
	.IOSTANDARD("DIFF_SSTL135")
) obufds_ck_inst (
	.O(o_ddr_ck_p),
	.OB(o_ddr_ck_n),
	.I(i_clk_ddr)
);

/////////////////////////////////////////////////
// DQS DIFFERENTIAL IO BUFFER
/////////////////////////////////////////////////
generate
for (i = 0; i < 2; i = i+1) begin
	IOBUFDS #(
		.IOSTANDARD("DIFF_SSTL135")	// Specify the I/O standard
	) iobufds_dqs_inst (
		.O(w2_dqs_rd[i]),	// Buffer output
		.IO(io2_ddr_dqs_p[i]),	// Diff_p inout (connect directly to top-level port)
		.IOB(io2_ddr_dqs_n[i]),	// Diff_n inout (connect directly to top-level port)
		.I(w2_dqs_wr[i]),	// Buffer input
		.T(w2_tristate_en[i])	// 3-state enable input, high=input, low=output
	);
end
endgenerate

/////////////////////////////////////////////////
// DQS INPUT DELAY
/////////////////////////////////////////////////
generate
for (i = 0; i < 2; i = i+1) begin
	IDELAYE2 #(
		.HIGH_PERFORMANCE_MODE("TRUE"), // Reduced jitter ("TRUE"), Reduced power ("FALSE")
		.IDELAY_TYPE("FIXED"), // FIXED, VARIABLE, VAR_LOAD, VAR_LOAD_PIPE
		.IDELAY_VALUE(28), // Input delay tap setting (0-31)
		.REFCLK_FREQUENCY(REFCLK_FREQUENCY) // IDELAYCTRL clock input frequency in MHz (190.0-210.0, 290.0-310.0).
	) idelay_dqs_inst (
		.CNTVALUEOUT(), // 5-bit output: Counter value output
		.DATAOUT(w2_dqs_rd_delayed[i]), // 1-bit output: Delayed data output
		.C(i_clk_ddr_90), // 1-bit input: Clock input
		.CE(1'b0), // 1-bit input: Active high enable increment/decrement input
		.CINVCTRL(1'b0), // 1-bit input: Dynamic clock inversion input
		.CNTVALUEIN(5'b0), // 5-bit input: Counter value input
		.DATAIN(1'b0), // 1-bit input: Internal delay data input
		.IDATAIN(w2_dqs_rd[i]), // 1-bit input: Data input from the I/O
		.INC(1'b0), // 1-bit input: Increment / Decrement tap delay input
		.LD(1'b0), // 1-bit input: Load IDELAY_VALUE input
		.LDPIPEEN(1'b0), // 1-bit input: Enable PIPELINE register to load data input
		.REGRST(1'b0) // 1-bit input: Active-high reset tap-delay input
	);
end
endgenerate

/////////////////////////////////////////////////
// DQS ODDR
/////////////////////////////////////////////////
generate
for (i = 0; i < 2; i = i+1) begin
	ODDR #(
		.DDR_CLK_EDGE("SAME_EDGE"), // "OPPOSITE_EDGE" or "SAME_EDGE"
		.INIT(1'b0) // Initial value of Q: 1'b0 or 1'b1
	) oddr_dqs_inst (
		.Q(w2_dqs_wr[i]), // 1-bit DDR output
		.C(i_clk_ddr_90), // 1-bit clock input
		.CE(1'b1), // 1-bit clock enable input
		.D1(w2_dqs_oddr_d1en[i]), // 1-bit data input (positive edge)
		.D2(1'b0), // 1-bit data input (negative edge)
		.R(i_phy_rst), // 1-bit reset
		.S(1'b0) // 1-bit set
	);
end
endgenerate

/////////////////////////////////////////////////
// IDELAYCTRL to calibrate IDELAY/ODELAY blocks
/////////////////////////////////////////////////
IDELAYCTRL IDELAYCTRL_inst (
	.RDY(w_idelay_rdy), // 1-bit output: Ready output
	.REFCLK(i_clk_ref), // 1-bit input: Reference clock input
	.RST(i_phy_rst) // 1-bit input: Active high reset input
);

/////////////////////////////////////////////////
// DQ IO BUFFER
/////////////////////////////////////////////////
generate
for (i = 0; i < 16; i = i+1) begin
	IOBUF #(
		.IOSTANDARD("SSTL135"),	// Specify the I/O standard
		.SLEW("FAST")	// Specify the output slew rate
	) iobuf_dq_inst (
		.O(w16_dq_rd[i]),	// Buffer output [signal coming into fpga]
		.IO(io16_ddr_dq[i]),	// Buffer inout port (connect directly to top-level port)
		.I(w16_dq_wr[i]),	// Buffer input [signal going out of fpga]
		.T(w2_tristate_en[i/8])	// 3-state enable input, high=input, low=output
	);
end
endgenerate

/////////////////////////////////////////////////
// DQ ISERDES
/////////////////////////////////////////////////
generate
for (i = 0; i < 16; i = i+1) begin
	ISERDESE2 #(
		.DATA_RATE("DDR"), // DDR, SDR
		.DATA_WIDTH(4), // Parallel data width (2-8,10,14)
						// In MEMORY + DDR mode, only DATA_WIDTH 4 supported, UG471 Table 3-3
		.INTERFACE_TYPE("MEMORY"), // MEMORY, MEMORY_DDR3, MEMORY_QDR, NETWORKING, OVERSAMPLE
		.IOBDELAY("NONE"), // NONE, BOTH, IBUF, IFD
		.NUM_CE(2) // Number of clock enables (1,2)
	) iserdes_dq_inst (
		.O(), // 1-bit output: Combinatorial output
		// Q1 - Q8: 1-bit (each) output: Registered data outputs
		.Q1(o64_phy_rddata[i]),
		.Q2(o64_phy_rddata[i+16]),
		.Q3(o64_phy_rddata[i+32]),
		.Q4(o64_phy_rddata[i+48]),
		.Q5(),
		.Q6(),
		.Q7(),
		.Q8(),
		// SHIFTOUT1, SHIFTOUT2: 1-bit (each) output: Data width expansion output ports
		.SHIFTOUT1(),
		.SHIFTOUT2(),
		.BITSLIP(1'b0), // 1-bit input: The BITSLIP pin performs a Bitslip operation synchronous to
		// CLKDIV when asserted (active High). Subsequently, the data seen on the Q1
		// to Q8 output ports will shift, as in a barrel-shifter operation, one
		// position every time Bitslip is invoked (DDR operation is different from
		// SDR).
		// CE1, CE2: 1-bit (each) input: Data register clock enable inputs
		.CE1(1'b1),
		.CE2(1'b1),
		.CLKDIVP(1'b0), // 1-bit input: TBD
		// Clocks: 1-bit (each) input: ISERDESE2 clock input ports
		.CLK(w2_dqs_rd_delayed[i/8]), // 1-bit input: High-speed clock
		.CLKB(~w2_dqs_rd_delayed[i/8]), // 1-bit input: High-speed secondary clock
		.CLKDIV(i_clk_ddr), // 1-bit input: Divided clock
		.OCLK(i_clk_ddr), // 1-bit input: High speed output clock used when INTERFACE_TYPE="MEMORY"
		// Dynamic Clock Inversions: 1-bit (each) input: Dynamic clock inversion pins to switch clock polarity
		.DYNCLKDIVSEL(1'b0), // 1-bit input: Dynamic CLKDIV inversion
		.DYNCLKSEL(1'b0), // 1-bit input: Dynamic CLK/CLKB inversion
		// Input Data: 1-bit (each) input: ISERDESE2 data input ports
		.D(w16_dq_rd[i]), // 1-bit input: Data input
		.DDLY(1'b0), // 1-bit input: Serial data from IDELAYE2
		.OFB(1'b0), // 1-bit input: Data feedback from OSERDESE2
		.OCLKB(~i_clk_ddr), // 1-bit input: High speed negative edge output clock
		.RST(i_phy_rst), // 1-bit input: Active high asynchronous reset
		// SHIFTIN1, SHIFTIN2: 1-bit (each) input: Data width expansion input ports
		.SHIFTIN1(1'b0),
		.SHIFTIN2(1'b0)
	);
end
endgenerate
//generate
//for (i = 0; i < 16; i = i+1) begin
//	IDDR #(
//		.DDR_CLK_EDGE("SAME_EDGE_PIPELINED"), // "OPPOSITE_EDGE", "SAME_EDGE"
//		// or "SAME_EDGE_PIPELINED"
//		.INIT_Q1(1'b0), // Initial value of Q1: 1'b0 or 1'b1
//		.INIT_Q2(1'b0), // Initial value of Q2: 1'b0 or 1'b1
//		.SRTYPE("SYNC") // Set/Reset type: "SYNC" or "ASYNC"
//	) IDDR_inst (
//		.Q1(o32_phy_rddata[i]), // 1-bit output for positive edge of clock
//		.Q2(o32_phy_rddata[i+16]), // 1-bit output for negative edge of clock
//		.C(w2_dqs_rd_delayed[i/8]), // 1-bit clock input
//		.CE(1'b1), // 1-bit clock enable input
//		.D(w16_dq_rd[i]), // 1-bit DDR data input
//		.R(i_phy_rst), // 1-bit reset
//		.S(1'b0) // 1-bit set
//	);
//end
//endgenerate

/////////////////////////////////////////////////
// DQ OSERDES
/////////////////////////////////////////////////
generate
for (i = 0; i < 16; i = i+1) begin
	// For 8:1 data width, OQ latency is 3, 4, or 5 CLK cycles (UG471)(?)
	// Simulation shows delay is 5 CLK cycles. ILA testing recommended.
	OSERDESE2 #(
		.DATA_RATE_OQ("DDR"), // DDR, SDR
		.DATA_RATE_TQ("BUF"), // DDR, BUF, SDR
		.DATA_WIDTH(8), // Parallel data width (2-8,10,14)
		.TRISTATE_WIDTH(1) // 3-state converter width (1,4)
	) oserdes_dq_inst (
		.OFB(), // 1-bit output: Feedback path for data
		.OQ(w16_dq_wr[i]), // 1-bit output: Data path output
		// SHIFTOUT1 / SHIFTOUT2: 1-bit (each) output: Data output expansion (1-bit each)
		.SHIFTOUT1(),
		.SHIFTOUT2(),
		.TBYTEOUT(), // 1-bit output: Byte group tristate
		.TFB(), // 1-bit output: 3-state control
		.TQ(), // 1-bit output: 3-state control
		.CLK(i_clk_ddr), // 1-bit input: High speed clock
		.CLKDIV(i_clk_ddr), // 1-bit input: Divided clock
		// D1 - D8: 1-bit (each) input: Parallel data inputs (1-bit each)
		.D1(i128_phy_wrdata[i+16*0]),
		.D2(i128_phy_wrdata[i+16*1]),
		.D3(i128_phy_wrdata[i+16*2]),
		.D4(i128_phy_wrdata[i+16*3]),
		.D5(i128_phy_wrdata[i+16*4]),
		.D6(i128_phy_wrdata[i+16*5]),
		.D7(i128_phy_wrdata[i+16*6]),
		.D8(i128_phy_wrdata[i+16*7]),
		.OCE(i_phy_dq_oserdes_en),//1'b1), // 1-bit input: Output data clock enable
		.RST(i_phy_rst), // 1-bit input: Reset
		// SHIFTIN1 / SHIFTIN2: 1-bit (each) input: Data input expansion (1-bit each)
		.SHIFTIN1(1'b0),
		.SHIFTIN2(1'b0),
		// T1 - T4: 1-bit (each) input: Parallel 3-state inputs
		.T1(1'b0),
		.T2(),//w2_dqs_iobuf_out_nen[i/8]),
		.T3(),//w2_dqs_iobuf_out_nen[i/8]),
		.T4(),//w2_dqs_iobuf_out_nen[i/8]),
		.TBYTEIN(1'b0), // 1-bit input: Byte group tristate
		.TCE(1'b1) // 1-bit input: 3-state clock enable
	);
end
endgenerate
//generate
//for (i = 0; i < 16; i = i+1) begin
//	ODDR #(
//		.DDR_CLK_EDGE("SAME_EDGE"), // "OPPOSITE_EDGE" or "SAME_EDGE"
//		.INIT(1'b0) // Initial value of Q: 1'b0 or 1'b1
//	) oddr_dq_inst (
//		.Q(w16_dq_wr[i]), // 1-bit DDR output
//		.C(i_clk_ddr), // 1-bit clock input
//		.CE(i_phy_dq_oddr_en), // 1-bit clock enable input
//		.D1(i32_phy_wrdata[i]), // 1-bit data input (positive edge)
//		.D2(i32_phy_wrdata[i+16]), // 1-bit data input (negative edge)
//		.R(i_phy_rst), // 1-bit reset
//		.S(1'b0) // 1-bit set
//	);
//end
//endgenerate

endmodule
