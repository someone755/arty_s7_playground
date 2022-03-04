`timescale 1ns / 1ps

/**
 *	SERDES demo/proof of concept for the
 *	Digilent Arty S7-50 development board.
 *
 *	Tested and working at 20 MHz (ddr clock)
 *	and lower (MMCM lower limit is 12.5 MHz)
 *	via ordinary jumper wires, no attempt at
 *	line length or impedance matching.
 *
 *	Tangentially based on XAPP721.
 */

module serdes #(
	parameter	p16_TMR_INIT	= 'hFFFFFFFF
)(
	input	DDR3_CLK100,	// 100 MHz oscillator input
	input	[3:0]	SW,
	input	[3:0]	BTN,
	
	output	[3:0]	LED,
	output	[2:0]	RGBLED0, RGBLED1,
	
	// Refer to UG475
	// All are HR (CSGA324 has no HP), Memory Byte Group 2, Bank 14
	// Strobe pins are U17/U18, marked as DQS pins in pinout
	output	jc4_s_o,	// out strobe
	output	jc7_d_o,	// out data
	input	jc2_d_i,	// in data
	input	jc3_s_i		// in strobe
);
// Input/output wires
wire	w_out_dqs,	w_out_dq;
assign jc4_s_o = w_out_dqs;
assign jc7_d_o = w_out_dq;
wire	w_out_dqs_obuft, w_out_dq_obuft;

wire	w_in_strobe = jc3_s_i;
wire	w_in_data = jc2_d_i;

// Clock generation wires
wire	w_clk_ddr;
wire	w_clk_ddr_90;
wire	w_clk_idelayctrl;
wire	w_clk_div;
wire	w_clk_div_90;

// IOLOGIC connections
// Notice 0:3 declaration: par[0] (leftmost/LSB bit) is shifted out first
reg	[0:3]	r4_oserdes_dq_par  = 4'h0;	// DQ output: X-X-D0-D1 -> D2-D3-D4-D5 -> D6-D7-X-X
reg	[0:3]	r4_oserdes_dqs_par = 4'h0;	// DQS output: 0 -> 5 -> 5 -> 0

reg	[0:3]	r4_tristate_dq  = 'hF;	// DQ tristate:  ...F -> C -> 0 -> 0 -> 3 -> F ...
reg	[0:3]	r4_tristate_dqs = 'hF;	// DQS tristate: ...F -> E -> 0 -> 0 -> 7 -> F...
wire	w_dq_tristate, w_dqs_tristate;	// OSERDES output to IOBUF

wire	[3:0]	w4_iserdes_par;	// ISERDES parallel output

assign LED = r8_iserdes_out[3:0];
assign RGBLED0 =	(SW == LED) ? 3'b010 :
					//(w4_iserdes_par == ) ? 3'b010 :
					3'b100;
assign RGBLED1[2] = (r16_init_tmr > 0);

reg	r_selectio_rst = 1'b1;

reg	r_write_start = 1'b0;

reg	[1:0]	r2_btn_pipe = 2'b0;
reg [15:0]	r16_init_tmr = p16_TMR_INIT;
reg	[2:0]	r3_dqs_state = 'b000;
always @(posedge w_clk_div) begin: dqs_ctrl
	if(r16_init_tmr > 0)
		r16_init_tmr <= r16_init_tmr - 1;
	else
		r_selectio_rst <= 1'b0;

	// metastability pipeline
	r2_btn_pipe <= {BTN[0], r2_btn_pipe[1]};

	// signal to dq/command sequential blocks
	if ((r16_init_tmr == 0) & (r2_btn_pipe[0] == 1))
		r_write_start <= 1'b1;
	else
		r_write_start <= 1'b0;

	case (r3_dqs_state)
	'd0: begin
		r4_tristate_dqs <= 'hF;
		if (r_write_start == 1)
			// next div cycle
			r3_dqs_state <= 'd2;
	end
	'd1: 
		r3_dqs_state <= 'd2;
	'd2: begin
		// setup dqs oserdes
		r4_oserdes_dqs_par <= 4'b0;
		r4_tristate_dqs <= 'hE;
		r3_dqs_state <= 'd3;
	end
	'd3: begin
		// next 4 dq bits -- two div cycles (8 bits transmitted)
		r4_oserdes_dqs_par <= 4'h5;
		r4_tristate_dqs <= 'h0;
		// next div cycle
		r3_dqs_state <= 'd4;
	end
	'd4: begin
		// next 4 dq bits -- two div cycles (8 bits transmitted)
		r4_oserdes_dqs_par <= 4'h5;
		r4_tristate_dqs <= 'h0;
		// next div cycle?
		if (r_write_start == 1)
			r3_dqs_state <= 'd3;
		else
			r3_dqs_state <= 'd5;
	end
	'd5: begin
		// final 4 dq bits
		r4_oserdes_dqs_par <= 4'h0;
		r4_tristate_dqs <= 'h7;
		r3_dqs_state <= 'd0;
	end
	default: ;
	endcase
end

reg	[7:0]	r8_iserdes_out;
reg	[1:0]	r2_write_start_pipe = 2'b0;
reg [3:0]	r4_dq_tmr = 4'b1;
reg	[2:0]	r3_dq_state = 'b000;
always @(posedge w_clk_div_90) begin: dq_ctrl
	if (r4_dq_tmr > 0)
		r4_dq_tmr <= r4_dq_tmr - 1;

	if (r4_dq_tmr == 2)
		r8_iserdes_out[3:0] <= w4_iserdes_par;
	if (r4_dq_tmr == 1)
		r8_iserdes_out[7:4] <= w4_iserdes_par;

	// metastability pipeline
	r2_write_start_pipe <= {r_write_start, r2_write_start_pipe[1]};

	case (r3_dq_state)
	'd0: begin
		if (r2_write_start_pipe[0] == 1) begin
			r4_oserdes_dq_par <= SW; //{2'b0, SW[0], SW[1]};
			r4_tristate_dq <= 'hC;
			r3_dq_state <= 'd1;
		end else begin
			r4_tristate_dq <= 'hF;
			r4_oserdes_dq_par <= 4'b0;			
		end
	end
	'd1: begin
		r4_tristate_dq <= 4'b0;
		r3_dq_state <= 'd2;
	end
	'd2: begin
		r4_tristate_dq <= 4'b0;
		r3_dq_state <= 'd3;
	end
	'd3: begin
		r4_dq_tmr <= 'd3;
		if (r2_write_start_pipe[0] == 1) begin
			r4_oserdes_dq_par <= SW;
			r4_tristate_dq <= 4'b0;
			r3_dq_state <= 'd2;
		end else begin
			r4_tristate_dq <= 4'h3;
			r3_dq_state <= 'd0;
		end
	end
	default: ;
	endcase
end

/////////////////////////////////////////////////
// Clock generation
/////////////////////////////////////////////////
clk_wiz_1 clkgen_ddr3ctrl_instance (
	// Clock out ports
	.clk_out1_ddr(w_clk_ddr),		// fast clock, in sync with DQS
	.clk_out2_ddr_90(w_clk_ddr_90),	// fast clock delayed by 90°, aligns to DQ
	.clk_out3_ref(w_clk_idelayctrl),// IDELAYCTRL, 200 MHz
	.clk_out4_div(w_clk_div),		// slow clock is 1:2 slower
	.clk_out5_div_90(w_clk_div_90),	// same as above, delayed by 45° (in phase with fast 90° clock)
	// Status and control signals
	.reset(1'b0),
	.locked(),
	// Clock in ports
	.clk_in1(DDR3_CLK100)
);
/////////////////////////////////////////////////
// OUTPUT
/////////////////////////////////////////////////
// ### Strobe IO buffer ###
wire w_iobuf_dqs_o;
IOBUF #(
	.IOSTANDARD("LVCMOS33"),	// Specify the I/O standard
	.SLEW("FAST")	// Specify the output slew rate
) iobuf_dqs_inst (
	.O(w_iobuf_dqs_o),	// Buffer output [signal coming into fpga]
	.IO(w_out_dqs),	// Buffer inout port (connect directly to top-level port)
	.I(w_out_dqs_obuft),	// Buffer input [signal going out of fpga]
	.T(w_dqs_tristate)	// 3-state enable input, high=input, low=output
);
// ### Strobe generation ###
OSERDESE2 #(
	.DATA_RATE_OQ("DDR"), // DDR, SDR
	.DATA_RATE_TQ("DDR"), // DDR, BUF, SDR
	.DATA_WIDTH(4), // Parallel data width (2-8,10,14)
	.TRISTATE_WIDTH(4), // 3-state converter width (1,4)
	.SERDES_MODE("MASTER")
) oserdes_dqs_inst (
	.OFB(), // 1-bit output: Feedback path for data
	.OQ(w_out_dqs_obuft), // 1-bit output: Data path output
	// SHIFTOUT1 / SHIFTOUT2: 1-bit (each) output: Data output expansion (1-bit each)
	.SHIFTOUT1(),
	.SHIFTOUT2(),
	.TBYTEOUT(), // 1-bit output: Byte group tristate
	.TFB(), // 1-bit output: 3-state control
	.TQ(w_dqs_tristate), // 1-bit output: 3-state control
	.CLK(w_clk_ddr), // 1-bit input: High speed clock
	.CLKDIV(w_clk_div), // 1-bit input: Divided clock
	// D1 - D8: 1-bit (each) input: Parallel data inputs (1-bit each)
	.D1(r4_oserdes_dqs_par[0]),
	.D2(r4_oserdes_dqs_par[1]),
	.D3(r4_oserdes_dqs_par[2]),
	.D4(r4_oserdes_dqs_par[3]),
	.D5(),
	.D6(),
	.D7(),
	.D8(),
	.OCE(1'b1), // 1-bit input: Output data clock enable
	.RST(r_selectio_rst), // 1-bit input: Reset
	// SHIFTIN1 / SHIFTIN2: 1-bit (each) input: Data input expansion (1-bit each)
	.SHIFTIN1(1'b0),
	.SHIFTIN2(1'b0),
	// T1 - T4: 1-bit (each) input: Parallel 3-state inputs
	.T1(r4_tristate_dqs[0]),
	.T2(r4_tristate_dqs[1]),
	.T3(r4_tristate_dqs[2]),
	.T4(r4_tristate_dqs[3]),
	.TBYTEIN(1'b0), // 1-bit input: Byte group tristate
	.TCE(1'b1) // 1-bit input: 3-state clock enable
);

// ### Output buffer ###
wire w_iobuf_dq_o;
IOBUF #(
	.IOSTANDARD("LVCMOS33"),	// Specify the I/O standard
	.SLEW("FAST")	// Specify the output slew rate
) iobuf_dq_inst (
	.O(w_iobuf_dq_o),	// Buffer output [signal coming into fpga]
	.IO(w_out_dq),	// Buffer inout port (connect directly to top-level port)
	.I(w_out_dq_obuft),	// Buffer input [signal going out of fpga]
	.T(w_dq_tristate)	// 3-state enable input, high=input, low=output
);
// ### Output data ###
// For 8:1 data width, OQ latency is 3, 4, or 5 CLK cycles (UG471)(?)
// Simulation shows delay is 5 CLK cycles. ILA testing recommended.
OSERDESE2 #(
	.DATA_RATE_OQ("DDR"), // DDR, SDR
	.DATA_RATE_TQ("DDR"), // DDR, BUF, SDR
	.DATA_WIDTH(4), // Parallel data width (2-8,10,14)
	.TRISTATE_WIDTH(4), // 3-state converter width (1,4)
	.SERDES_MODE("MASTER")
) oserdes_dq_inst (
	.OFB(), // 1-bit output: Feedback path for data
	.OQ(w_out_dq_obuft), // 1-bit output: Data path output
	// SHIFTOUT1 / SHIFTOUT2: 1-bit (each) output: Data output expansion (1-bit each)
	.SHIFTOUT1(),
	.SHIFTOUT2(),
	.TBYTEOUT(), // 1-bit output: Byte group tristate
	.TFB(), // 1-bit output: 3-state control
	.TQ(w_dq_tristate), // 1-bit output: 3-state control
	.CLK(w_clk_ddr_90), // 1-bit input: High speed clock
	.CLKDIV(w_clk_div_90), // 1-bit input: Divided clock
	// D1 - D8: 1-bit (each) input: Parallel data inputs (1-bit each)
	.D1(r4_oserdes_dq_par[0]),
	.D2(r4_oserdes_dq_par[1]),
	.D3(r4_oserdes_dq_par[2]),
	.D4(r4_oserdes_dq_par[3]),
	.D5(),
	.D6(),
	.D7(),
	.D8(),
	.OCE(1'b1), // 1-bit input: Output data clock enable
	.RST(r_selectio_rst), // 1-bit input: Reset
	// SHIFTIN1 / SHIFTIN2: 1-bit (each) input: Data input expansion (1-bit each)
	.SHIFTIN1(1'b0),
	.SHIFTIN2(1'b0),
	// T1 - T4: 1-bit (each) input: Parallel 3-state inputs
	.T1(r4_tristate_dq[0]),
	.T2(r4_tristate_dq[1]),
	.T3(r4_tristate_dq[2]),
	.T4(r4_tristate_dq[3]),
	.TBYTEIN(1'b0), // 1-bit input: Byte group tristate
	.TCE(1'b1) // 1-bit input: 3-state clock enable
);
/////////////////////////////////////////////////
// INPUT
/////////////////////////////////////////////////
// ### Input data ###
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
	.Q1(w4_iserdes_par[0]),
	.Q2(w4_iserdes_par[1]),
	.Q3(w4_iserdes_par[2]),
	.Q4(w4_iserdes_par[3]),
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
	.CLK(w_in_strobe), // 1-bit input: High-speed clock
	.CLKB(~w_in_strobe), // 1-bit input: High-speed secondary clock
	.CLKDIV(w_clk_div_90), // 1-bit input: Divided clock
	.OCLK(w_clk_ddr_90), // 1-bit input: High speed output clock used when INTERFACE_TYPE="MEMORY"
	// Dynamic Clock Inversions: 1-bit (each) input: Dynamic clock inversion pins to switch clock polarity
	.DYNCLKDIVSEL(1'b0), // 1-bit input: Dynamic CLKDIV inversion
	.DYNCLKSEL(1'b0), // 1-bit input: Dynamic CLK/CLKB inversion
	// Input Data: 1-bit (each) input: ISERDESE2 data input ports
	.D(w_in_data), // 1-bit input: Data input
	.DDLY(1'b0), // 1-bit input: Serial data from IDELAYE2
	.OFB(1'b0), // 1-bit input: Data feedback from OSERDESE2
	.OCLKB(~w_clk_ddr_90), // 1-bit input: High speed negative edge output clock
	.RST(r_selectio_rst), // 1-bit input: Active high asynchronous reset
	// SHIFTIN1, SHIFTIN2: 1-bit (each) input: Data width expansion input ports
	.SHIFTIN1(1'b0),
	.SHIFTIN2(1'b0)
);
// ### Input strobe delay ###
//IDELAYE2 #(
//	.HIGH_PERFORMANCE_MODE("TRUE"), // Reduced jitter ("TRUE"), Reduced power ("FALSE")
//	.IDELAY_TYPE("FIXED"), // FIXED, VARIABLE, VAR_LOAD, VAR_LOAD_PIPE
//	.IDELAY_VALUE(28), // Input delay tap setting (0-31)
//	.REFCLK_FREQUENCY(REFCLK_FREQUENCY) // IDELAYCTRL clock input frequency in MHz (190.0-210.0, 290.0-310.0).
//) idelay_dqs_inst (
//	.CNTVALUEOUT(), // 5-bit output: Counter value output
//	.DATAOUT(w2_dqs_rd_delayed[i]), // 1-bit output: Delayed data output
//	.C(w_clk_ddr), // 1-bit input: Clock input
//	.CE(1'b0), // 1-bit input: Active high enable increment/decrement input
//	.CINVCTRL(1'b0), // 1-bit input: Dynamic clock inversion input
//	.CNTVALUEIN(5'b0), // 5-bit input: Counter value input
//	.DATAIN(1'b0), // 1-bit input: Internal delay data input
//	.IDATAIN(w2_dqs_rd[i]), // 1-bit input: Data input from the I/O
//	.INC(1'b0), // 1-bit input: Increment / Decrement tap delay input
//	.LD(1'b0), // 1-bit input: Load IDELAY_VALUE input
//	.LDPIPEEN(1'b0), // 1-bit input: Enable PIPELINE register to load data input
//	.REGRST(1'b0) // 1-bit input: Active-high reset tap-delay input
//);

//IDELAYCTRL IDELAYCTRL_inst (
//	.RDY(w_idelay_rdy), // 1-bit output: Ready output
//	.REFCLK(w_clk_idelayctrl), // 1-bit input: Reference clock input
//	.RST(1'b0) // 1-bit input: Active high reset input
//);

//ila_serdes ila_serdes_inst (
//		.clk(w_clk_ddr),

//		.probe0(w_in_strobe),
//		.probe1(BTN[0]),
///*3:0*/	.probe2(w4_iserdes_par),
///*3:0*/	.probe3(r4_oserdes_par),
//		.probe4(w_in_data),
///*15:0*/.probe5(r16_init_tmr)
//);
endmodule
