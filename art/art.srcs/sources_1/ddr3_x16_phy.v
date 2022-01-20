`timescale 1ns / 1ps

module ddr3_x16_phy #(
	parameter	REFCLK_FREQUENCY	= 200.0,
	parameter	SIMULATION	= "FALSE"
)(
	input	i_clk_ui,	// 4:1 ratio between ui and ddr clocks
	input	i_clk_ddr,
	input	i_clk_ddr_90,
	input	i_clk_ref,	// 200 or 300 MHz, see REFCLK_FREQUENCY, used for IDELAYCTRL
	
	input	i_phy_rst,	// active high reset for OSERDES, IDELAYCTRL, hold HIGH until clocks are ready (approx. 900 ns as per simulation)
	
	input	[127:0]	i128_phy_wrdata,
	
	input	[1:0]	i2_phy_dqs,
	input	[1:0]	i2_dqs_iobuf_out_nen,
	input	i_dq_oserdes_en,
	
	output	o_phy_idelay_rdy,
	
	// CONNECTION TO DRAM
	inout	[15:0]	io16_ddr_dq,
	inout	[1:0]	io2_ddr_dqs_p,
	inout	[1:0]	io2_ddr_dqs_n,
	
	output	o_ddr_ck_p,
	output	o_ddr_ck_n,
	
	output	o14_ddr_addr

);
localparam tXPR = 1000;

wire w_idelay_rdy;	// idelay ready output
assign o_phy_idelay_rdy = w_idelay_rdy;

wire	[1:0]	w2_dqs_in;
wire	[1:0]	w2_dqs_in_delayed; // Delayed to middle of data eye
wire	[1:0]	w2_dqs_out;	// LDQS [0] and UDQS [1]
wire	[1:0]	w2_dqs_outen;
assign w2_dqs_outen = i2_phy_dqs;

wire	[1:0]	w2_dqs_iobuf_out_nen;
assign w2_dqs_iobuf_out_nen = i2_dqs_iobuf_out_nen;	// TODO: connect dq/dqs buffer output enable (likely module input?)

wire	[15:0]	w16_dq_in;
wire	[15:0]	w16_dq_out;
wire	[15:0]	w16_4_dq_deser	[3:0];
wire	[15:0]	w16_dq_iobuf_out_nen;

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
		.O(w2_dqs_in[i]),	// Buffer output
		.IO(io2_ddr_dqs_p[i]),	// Diff_p inout (connect directly to top-level port)
		.IOB(io2_ddr_dqs_n[i]),	// Diff_n inout (connect directly to top-level port)
		.I(w2_dqs_out[i]),	// Buffer input
		.T(w2_dqs_iobuf_out_nen[i])	// 3-state enable input, high=input, low=output
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
		.IDELAY_VALUE(16), // Input delay tap setting (0-31)
		.REFCLK_FREQUENCY(REFCLK_FREQUENCY) // IDELAYCTRL clock input frequency in MHz (190.0-210.0, 290.0-310.0).
	) idelay_dqs_inst (
		.CNTVALUEOUT(), // 5-bit output: Counter value output
		.DATAOUT(w2_dqs_in_delayed[i]), // 1-bit output: Delayed data output
		.C(i_clk_ui), // 1-bit input: Clock input
		.CE(1'b0), // 1-bit input: Active high enable increment/decrement input
		.CINVCTRL(1'b0), // 1-bit input: Dynamic clock inversion input
		.CNTVALUEIN(5'b0), // 5-bit input: Counter value input
		.DATAIN(1'b0), // 1-bit input: Internal delay data input
		.IDATAIN(w2_dqs_in[i]), // 1-bit input: Data input from the I/O
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
		.Q(w2_dqs_out[i]), // 1-bit DDR output
		.C(i_clk_ddr_90), // 1-bit clock input
		.CE(1'b1), // 1-bit clock enable input
		.D1(w2_dqs_outen[i]), // 1-bit data input (positive edge)
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
		.O(w16_dq_in[i]),	// Buffer output [signal coming into fpga]
		.IO(io16_ddr_dq[i]),	// Buffer inout port (connect directly to top-level port)
		.I(w16_dq_out[i]),	// Buffer input [signal going out of fpga]
		.T(w16_dq_iobuf_out_nen[i])	// 3-state enable input, high=input, low=output
	);
end
endgenerate

/////////////////////////////////////////////////
// DQ INPUT SERIAL-TO-PARALLEL
/////////////////////////////////////////////////
generate
for (i = 0; i < 16; i = i+1) begin
	ISERDESE2 #(
		.DATA_RATE("DDR"), // DDR, SDR
		.DATA_WIDTH(4), // Parallel data width (2-8,10,14)
		.INTERFACE_TYPE("MEMORY"), // MEMORY, MEMORY_DDR3, MEMORY_QDR, NETWORKING, OVERSAMPLE
		.IOBDELAY("NONE"), // NONE, BOTH, IBUF, IFD
		.NUM_CE(1) // Number of clock enables (1,2)
	) iserdes_dq_inst (
		.O(), // 1-bit output: Combinatorial output
		// Q1 - Q8: 1-bit (each) output: Registered data outputs
		.Q1(w16_4_dq_deser[0][i]),
		.Q2(w16_4_dq_deser[1][i]),
		.Q3(w16_4_dq_deser[2][i]),
		.Q4(w16_4_dq_deser[3][i]),
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
		.CE2(),
		.CLKDIVP(1'b0), // 1-bit input: TBD
		// Clocks: 1-bit (each) input: ISERDESE2 clock input ports
		.CLK(w2_dqs_in_delayed[i/8]), // 1-bit input: High-speed clock
		.CLKB(~w2_dqs_in_delayed[i/8]), // 1-bit input: High-speed secondary clock
		.CLKDIV(i_clk_ui), // 1-bit input: Divided clock
		.OCLK(i_clk_ddr), // 1-bit input: High speed output clock used when INTERFACE_TYPE="MEMORY"
		// Dynamic Clock Inversions: 1-bit (each) input: Dynamic clock inversion pins to switch clock polarity
		.DYNCLKDIVSEL(DYNCLKDIVSEL), // 1-bit input: Dynamic CLKDIV inversion
		.DYNCLKSEL(DYNCLKSEL), // 1-bit input: Dynamic CLK/CLKB inversion
		// Input Data: 1-bit (each) input: ISERDESE2 data input ports
		.D(w16_dq_in[i]), // 1-bit input: Data input
		.DDLY(1'b0), // 1-bit input: Serial data from IDELAYE2
		.OFB(OFB), // 1-bit input: Data feedback from OSERDESE2
		.OCLKB(~i_clk_ddr), // 1-bit input: High speed negative edge output clock
		.RST(i_phy_rst), // 1-bit input: Active high asynchronous reset
		// SHIFTIN1, SHIFTIN2: 1-bit (each) input: Data width expansion input ports
		.SHIFTIN1(1'b0),
		.SHIFTIN2(1'b0)
	);
end
endgenerate

/////////////////////////////////////////////////
// DQ OUTPUT PARALLEL-TO-SERIAL
/////////////////////////////////////////////////
generate
for (i = 0; i < 16; i = i+1) begin
	// for 8:1 data width, OQ latency is 3-5 CLK cycles (UG471)(?)
	OSERDESE2 #(
		.DATA_RATE_OQ("DDR"), // DDR, SDR
		.DATA_RATE_TQ("BUF"), // DDR, BUF, SDR
		.DATA_WIDTH(8), // Parallel data width (2-8,10,14)
		.TRISTATE_WIDTH(1) // 3-state converter width (1,4)
	) oserdes_dq_inst (
		.OFB(), // 1-bit output: Feedback path for data
		.OQ(w16_dq_out[i]), // 1-bit output: Data path output
		// SHIFTOUT1 / SHIFTOUT2: 1-bit (each) output: Data output expansion (1-bit each)
		.SHIFTOUT1(),
		.SHIFTOUT2(),
		.TBYTEOUT(), // 1-bit output: Byte group tristate
		.TFB(), // 1-bit output: 3-state control
		.TQ(w16_dq_iobuf_out_nen[i]), // 1-bit output: 3-state control
		.CLK(i_clk_ddr), // 1-bit input: High speed clock
		.CLKDIV(i_clk_ui), // 1-bit input: Divided clock
		// D1 - D8: 1-bit (each) input: Parallel data inputs (1-bit each)
		.D1(i128_phy_wrdata[i+16*0]),
		.D2(i128_phy_wrdata[i+16*1]),
		.D3(i128_phy_wrdata[i+16*2]),
		.D4(i128_phy_wrdata[i+16*3]),
		.D5(i128_phy_wrdata[i+16*4]),
		.D6(i128_phy_wrdata[i+16*5]),
		.D7(i128_phy_wrdata[i+16*6]),
		.D8(i128_phy_wrdata[i+16*7]),
		.OCE(i_dq_oserdes_en),//1'b1), // 1-bit input: Output data clock enable
		.RST(i_phy_rst), // 1-bit input: Reset
		// SHIFTIN1 / SHIFTIN2: 1-bit (each) input: Data input expansion (1-bit each)
		.SHIFTIN1(1'b0),
		.SHIFTIN2(1'b0),
		// T1 - T4: 1-bit (each) input: Parallel 3-state inputs
		.T1(w2_dqs_iobuf_out_nen[i/8]),
		.T2(),//w2_dqs_iobuf_out_nen[i/8]),
		.T3(),//w2_dqs_iobuf_out_nen[i/8]),
		.T4(),//w2_dqs_iobuf_out_nen[i/8]),
		.TBYTEIN(1'b0), // 1-bit input: Byte group tristate
		.TCE(1'b1) // 1-bit input: 3-state clock enable
	);
end
endgenerate

endmodule
