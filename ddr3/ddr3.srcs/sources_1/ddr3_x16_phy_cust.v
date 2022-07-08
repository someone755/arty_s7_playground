`timescale 1ns / 1ps

`define SIMULATION

`define	max2(v1, v2) ((v1) > (v2) ? (v1) : (v2))
`define max3(v1, v2, v3) (`max2((v1), `max2((v2), (v3))))
`define	ck2ps(ddrfreq) (1_000_000/``ddrfreq``) // use ck2ps(p_DDR_FREQ_MHZ) to get period in ps

module ddr3_x16_phy_cust #(
	parameter	p_IDELAY_TYPE	= "VARIABLE",	// "VARIABLE" or "VAR_LOAD"
	parameter	p_IDELAY_INIT_DQS	= 6,	// max 31; Should be '0' for IDELAY_TYPE = "VAR_LOAD"
	parameter	p_IDELAY_INIT_DQ	= 10,
	
	parameter	p_RD_DELAY	= 2,	// delay in CK from RD CMD to valid ISERDES data
									// DLL = "OFF"
									// 	p_OUTPUT_PIPE = "FALSE":	"3"
									// 	p_OUTPUT_PIPE = "TRUE":		"4"
									// DLL = "ON", CK < 333 MHz
									//	p_OUTPUT_PIPE = "TRUE":		"5"
	
	parameter	p_OUTPUT_PIPE	= "TRUE",	// Should be "TRUE" for DLL="ON" speeds, or DRAM timing fails
											// Shouldn't be necessary for DLL="OFF"

	parameter	p_BANK_W	= 3,	// bank width; 3
	parameter	p_ROW_W		= 14,	// row width; 14 or 15
	parameter	p_COL_W		= 10,	// column width; 10 (or 11 TODO)
	parameter	p_DQ_W		= 16,	// # of DQ pins; 8 or 16
	parameter	p_ADDR_W	= 14,	// # of ADDR pins; 14 or 15 (likely equal to ROW_W)

	parameter	REFCLK_FREQUENCY	= 200.0,	// IDELAY resolution = 1000/(32 x 2 x REFCLK_FREQUENCY) [ns]
												// For 200 MHz, tap delay is 0.078125 ns
	parameter	p_DDR_FREQ_MHZ	= 300,	// use ck2ps(p_DDR_FREQ_MHZ) to get period in ps
										// JEDEC allows > 300 MHz with DLL ON, or < 125 MHZ with DLL OFF
	parameter	p_DDR_CK_PS		= `ck2ps(p_DDR_FREQ_MHZ),

								// (ps)					// Description
	parameter	p_CKE			= `max2(3*p_DDR_CK_PS, 5_000),	// CKE minimum pulse width
	parameter	p_FAW			= 450_000,				// Four Activate Window (4x ACTIVATE to 5th ACTIVATE)
	parameter	p_RAS			= 36_000,				// ACTIVATE-to-PRECHARGE (min; max is 9xtREFI)
	parameter	p_RCD			= 13_500,				// ACTIVATE-to-READ or ACTIVATE-to-WRITE delay
	parameter	p_REFI			= 7_800_000,			// Average periodic refresh interval
	parameter	p_RFC_MIN		= 160_000,				// REFRESH to ACTIVATE or REFRESH to REFRESH
	parameter	p_RFC_MAX		= 70_200_000,
	parameter	p_RP			= 13_500,				// Precharge command period
	parameter	p_RRD			= `max2(4*p_DDR_CK_PS, 10_000),	// ACTIVATE to ACTIVATE in different banks
	parameter	p_RTP			= `max2(4*p_DDR_CK_PS, 7_500),	// READ to PRECHARGE
	parameter	p_WTR			= `max2(4*p_DDR_CK_PS, 7_500),	// WRITE to READ
	parameter	p_WR			= `max2(4*p_DDR_CK_PS, 15_000),	// WRITE recovery time (WRITE to PRECHARGE)
	parameter	p_XPR			= `max2(5*p_DDR_CK_PS, p_RFC_MIN + 10_000),	// Exit reset from CKE HIGH to valid command
	parameter	p_MOD			= `max2(12*p_DDR_CK_PS, 15_000),	// MRS-to-non-MRS (MRS update delay)
	parameter	p_ZQINIT		= `max2(512*p_DDR_CK_PS, 640_000)	// ZQ Calibration Long time from reset
)(
	output	[(4*p_DQ_W)-1:0]	on_iserdes_par,
	input	[1:0]	i2_iserdes_ce,
	
	input	i_clk_ddr,		// memory bus clock frequency
	input	i_clk_ddr_n,
	input	i_clk_ddr_90,	// same but delayed by 90 deg, used to generate output DQ from OSERDES
	input	i_clk_ddr_90_n,
	input	i_clk_div,		// half of bus clock frequency
	input	i_clk_div_n,
	input	i_clk_ref,	// 200 MHz, used for IDELAYCTRL, which controls taps for input DQS IDELAY
	
	input	i_phy_rst,	// active high reset for ODDR, OSERDES, ISERDES, IDELAYCTRL, hold HIGH until all clocks are generated
	
	input	i_phy_cmd_en,	// Active high strobe for inputs: cmd_sel, addr, 
	input	i_phy_cmd_sel,	// Command for current request: 'b0 = WRITE || 'b1 = READ
	output	o_phy_cmd_full,	// Active high indicates input command FIFO full

	input	[p_BANK_W-1:0]	in_phy_bank,
	input	[p_ROW_W-1:0]	in_phy_row,
	input	[p_COL_W-1:0]	in_phy_col,
	input	[(8*p_DQ_W)-1:0]	in_phy_wrdata,	// eight words of write data for OSERDES (out of 8 for a total of BL8)
	input	[7:0]	i8_phy_wrdm,	// write data mask input, 1 bit per word in burst
	output	[(8*p_DQ_W)-1:0]	on_phy_rddata,	// eight words of read data from ISERDES (out of 8 for a total of BL8)
	output	o_phy_rddata_valid, // output data valid flag
	output	o_phy_rddata_end,	// second half of read burst
	
	output	o_phy_init_done,
	output	o_phy_idelay_rdy,
	
		// Tap control for x16 SDRAM, used in read calibration
		//	For a 16-bit word of "0xAABB":
		//		* INC/CE/LD [0] controls 8 MSB bits (0xAA)
		//		* INC/CE/LD [1] controls 8 LSB bits (0xBB)
		//		* delay_cnt [9:5] from 8 MSB bits (0xAA)
		//		* delay_cnt [4:0] from 8 LSB bits (0xBB)
	input	[(p_DQ_W/8)-1:0]	in_dqs_delay_ce,	// IDELAY tap change enable
	input	[(p_DQ_W/8)-1:0]	in_dq_delay_ce,

	input	[(p_DQ_W/8)-1:0]	in_dqs_delay_inc,	// IDELAY tap increment
	input	[(p_DQ_W/8)-1:0]	in_dq_delay_inc,

	input	[(p_DQ_W/8)-1:0]	in_dqs_delay_ld,	// IDELAY tap load from on_idelay_cnt
	input	[(p_DQ_W/8)-1:0]	in_dq_delay_ld,

	input	[(p_DQ_W/8)*5-1:0]	in_dqs_idelay_cnt,	// IDELAY tap INPUT (load) value
	input	[(p_DQ_W/8)*5-1:0]	in_dq_idelay_cnt,

	output	[(p_DQ_W/8)*5-1:0]	on_dqs_idelay_cnt,	// IDELAY tap OUTPUT value
	output	[(p_DQ_W/8)*5-1:0]	on_dq_idelay_cnt,
		
	// CONNECTION TO DRAM by PHY CORE
	inout	[p_DQ_W-1:0]	ion_ddr_dq,
	inout	[(p_DQ_W/8)-1:0]	ion_ddr_dqs_p,
	inout	[(p_DQ_W/8)-1:0]	ion_ddr_dqs_n,
		// For x16 SDRAM
		// 	LDQS -> DQ[7:0]
		// 	UDQS -> DQ[15:0]
	
	output	[p_ADDR_W-1:0]	on_ddr_addr,

	output	o_ddr_ck_p,
	output	o_ddr_ck_n,
	
	// CONNECTION TO DRAM by LOGIC CORE (ADDR, BANK, CS/RAS/CAS/WE, ODT, CKE, UDM/LDM)
	output	[(p_DQ_W/8)-1:0]	on_ddr_dm,
	output	[p_BANK_W-1:0]	on_ddr_bank,

	output	o_ddr_nrst,
	output	o_ddr_cke,
	output	o_ddr_ncs,
	output	o_ddr_nras,
	output	o_ddr_ncas,
	output	o_ddr_nwe,
	output	o_ddr_odt
);
`include "ddr3_x16_phy_params.vh"
/////////////////////////////////////////////////
// PHY primitive connections
/////////////////////////////////////////////////
reg	r_init_done = 1'b0;
reg	r_phy_rst = 1'b1;

// IOB -> IDELAY
wire	[(p_DQ_W/8)-1:0]	wn_dqs_rd;
wire	[p_DQ_W-1:0]	wn_dq_rd;

// IDELAY -> ISERDES (DQ) or -> BUFIO (DQS)
wire	[(p_DQ_W/8)-1:0]	wn_dqs_rd_delayed;
wire	[p_DQ_W-1:0]	wn_dq_rd_delayed;

// BUFIO -> ISERDES (for DQS)
wire	[(p_DQ_W/8)-1:0] wn_dqs_rd_delayed_bufio;

// OSERDES -> IOB (data)
wire	[(p_DQ_W/8)-1:0]	wn_dqs_wr;	
wire	[p_DQ_W-1:0]	wn_dq_wr;

// OSERDES -> IOB (tristate ctrl)
wire	[(p_DQ_W/8)-1:0]	wn_dqs_iob_tristate;	
wire	[p_DQ_W-1:0]	wn_dq_iob_tristate;

// DDR data values (into OSERDES)
reg	[0:3]	r4_oserdes_dqs_par = 'hF;
reg	[0:(4*p_DQ_W)-1]	rn_oserdes_dq_par = {(4*p_DQ_W){1'b1}};

// DDR tristate values (into OSERDES)
reg	[0:3]	r4_tristate_dqs = 'hF;	
reg	[0:3]	r4_tristate_dq = 'hF;

wire	[0:(4*p_DQ_W)-1]	wn_iserdes_par;	// data read from memory (from ISERDES)

wire	[2:1]	w2_iserdes_ce;	// ISERDES primitive clock enable (2:1); either 'b00 or 'b01/10
assign w2_iserdes_ce = i2_iserdes_ce;//-2'b11;

// IDELAY tap value counter output
wire	[(p_DQ_W/8)*5-1:0]	wn_dqs_idelay_cnt;
wire	[4:0]	wn_dq_idelay_cnt_many	[p_DQ_W-1:0];
wire	[(p_DQ_W/8)*5-1:0]	wn_dq_idelay_cnt_few;
if (p_DQ_W > 8)
	assign wn_dq_idelay_cnt_few = {wn_dq_idelay_cnt_many[8], wn_dq_idelay_cnt_many[0]};
else
	assign wn_dq_idelay_cnt_few = wn_dq_idelay_cnt_many[0];


// W/R command FIFO
wire	[lp_CMDFIFO_WIDTH-1:0]	wn_cmdfifo_din;
assign wn_cmdfifo_din = {i_phy_cmd_sel, in_phy_bank, in_phy_row, in_phy_col, in_phy_wrdata, i8_phy_wrdm};
wire	[lp_CMDFIFO_WIDTH-1:0]	wn_cmdfifo_dout;
wire	w_cmdfifo_full;
wire	w_cmdfifo_empty;
reg	r_cmdfifo_rd	= 1'b0;

wire	w_cmdfifo_op;
wire	[p_BANK_W-1:0]	wn_cmdfifo_bank;
wire	[p_ROW_W-1:0]	wn_cmdfifo_row;
wire	[p_COL_W-1:0]	wn_cmdfifo_col;
wire	[(8*p_DQ_W)-1:0]	wn_cmdfifo_wrd;
wire	[7:0]	wn_cmdfifo_wrm;
assign	{w_cmdfifo_op, wn_cmdfifo_bank, wn_cmdfifo_row, wn_cmdfifo_col, wn_cmdfifo_wrd, wn_cmdfifo_wrm}	= wn_cmdfifo_dout;

//###############################################
//## PRIMITIVE DECLARATIONS:
//	[x]	clk obuf
//	[x] idelayctrl
//	[x]	dqs iobuf
//	[x]	dqs idelay
//	[x]	dqs oserdes
//	[x]	dq iobuf
//	[x]	dq idelay
//	[x]	dq iserdes
//	[x]	dq oserdes
//###############################################
genvar i; // loop variable for generate blocks
/////////////////////////////////////////////////
// DDR CLOCK DIFFERENTIAL OUTPUT BUFFER
/////////////////////////////////////////////////
OBUFDS #(
	.IOSTANDARD("DIFF_SSTL135")
) obufds_ck_inst (
	.O(o_ddr_ck_p),
	.OB(o_ddr_ck_n),
	.I(i_clk_ddr_n)
);
/////////////////////////////////////////////////
// IDELAYCTRL to calibrate IDELAY/ODELAY blocks
/////////////////////////////////////////////////
IDELAYCTRL IDELAYCTRL_inst (
	.RDY(w_idelay_rdy), // 1-bit output: Ready output
	.REFCLK(i_clk_ref), // 1-bit input: Reference clock input
	.RST(r_phy_rst) // 1-bit input: Active high reset input
);
/////////////////////////////////////////////////
// DQS DIFFERENTIAL IO BUFFER
/////////////////////////////////////////////////
generate
for (i = 0; i < (p_DQ_W/8); i = i+1) begin
	IOBUFDS #(
		.IOSTANDARD("DIFF_SSTL135")	// Specify the I/O standard
	) iobufds_dqs_inst (
		.O(wn_dqs_rd[i]),	// Buffer output
		.IO(ion_ddr_dqs_p[i]),	// Diff_p inout (connect directly to top-level port)
		.IOB(ion_ddr_dqs_n[i]),	// Diff_n inout (connect directly to top-level port)
		.I(wn_dqs_wr[i]),	// Buffer input
		.T(wn_dqs_iob_tristate[i])	// 3-state enable input, high=input, low=output
	);
end
endgenerate
/////////////////////////////////////////////////
// DQS LOCAL CLOCK BUFFER FOR I/O
/////////////////////////////////////////////////
/*generate
for (i = 0; i < (p_DQ_W/8); i = i+1) begin
	BUFIO bufio_dqs_inst (
		.O(wn_dqs_rd_delayed_bufio[i]), // 1-bit output: Clock output (connect to I/O clock loads).
		.I(wn_dqs_rd_delayed[i]) // 1-bit input: Clock input (connect to an IBUF or BUFMR).
	);
end
endgenerate*/
assign wn_dqs_rd_delayed_bufio = wn_dqs_rd_delayed;
/////////////////////////////////////////////////
// DQS INPUT DELAY
/////////////////////////////////////////////////
generate
for (i = 0; i < (p_DQ_W/8); i = i+1) begin
	IDELAYE2 #(
		.HIGH_PERFORMANCE_MODE("TRUE"), // Reduced jitter ("TRUE"), Reduced power ("FALSE")
		.IDELAY_TYPE(p_IDELAY_TYPE), // FIXED, VARIABLE, VAR_LOAD, VAR_LOAD_PIPE
		.IDELAY_VALUE(p_IDELAY_INIT_DQS), // Input delay tap setting (0-31); Ignored for VAR_LOAD
		.REFCLK_FREQUENCY(REFCLK_FREQUENCY), // IDELAYCTRL clock input frequency in MHz (190.0-210.0, 290.0-310.0).
		.SIGNAL_PATTERN("CLOCK")
	) idelay_dqs_inst (
		.CNTVALUEOUT(wn_dqs_idelay_cnt[i*5+:5]), // 5-bit output: Counter value output
		.DATAOUT(wn_dqs_rd_delayed[i]), // 1-bit output: Delayed data output
		.C(i_clk_div), // 1-bit input: Clock input
		.CE(in_dqs_delay_ce[i]), // 1-bit input: Active high enable increment/decrement input
		.CINVCTRL(1'b0), // 1-bit input: Dynamic clock inversion input
		.CNTVALUEIN(in_dqs_idelay_cnt[i*5+:5]), // 5-bit input: Counter value input
		.DATAIN(1'b0), // 1-bit input: Internal delay data input
		.IDATAIN(wn_dqs_rd[i]), // 1-bit input: Data input from the I/O
		.INC(in_dqs_delay_inc[i]), // 1-bit input: Increment / Decrement tap delay input
		.LD(in_dqs_delay_ld[i]), // 1-bit input: Load IDELAY_VALUE input
		.LDPIPEEN(1'b0), // 1-bit input: Enable PIPELINE register to load data input
		.REGRST(1'b0) // 1-bit input: Active-high reset tap-delay input
	);
end
endgenerate
/////////////////////////////////////////////////
// DQS OSERDES
/////////////////////////////////////////////////
generate
for (i = 0; i < (p_DQ_W/8); i = i+1) begin
	OSERDESE2 #(
		.DATA_RATE_OQ("DDR"), // DDR, SDR
		.DATA_RATE_TQ("DDR"), // DDR, BUF, SDR
		.DATA_WIDTH(4), // Parallel data width (2-8,10,14)
		.TRISTATE_WIDTH(4), // 3-state converter width (1,4)
		.SERDES_MODE("MASTER")
	) oserdes_dqs_inst (
		.OFB(), // 1-bit output: Feedback path for data
		.OQ(wn_dqs_wr[i]), // 1-bit output: Data path output
		// SHIFTOUT1 / SHIFTOUT2: 1-bit (each) output: Data output expansion (1-bit each)
		.SHIFTOUT1(),
		.SHIFTOUT2(),
		.TBYTEOUT(), // 1-bit output: Byte group tristate
		.TFB(), // 1-bit output: 3-state control
		.TQ(wn_dqs_iob_tristate[i]), // 1-bit output: 3-state control
		.CLK(i_clk_ddr), // 1-bit input: High speed clock
		.CLKDIV(i_clk_div), // 1-bit input: Divided clock
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
		.RST(r_phy_rst), // 1-bit input: Reset
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
end
endgenerate
/////////////////////////////////////////////////
// DQ IO BUFFER
/////////////////////////////////////////////////
generate
for (i = 0; i < p_DQ_W; i = i+1) begin
	IOBUF #(
		.IOSTANDARD("SSTL135"),	// Specify the I/O standard
		.SLEW("FAST")	// Specify the output slew rate
	) iobuf_dq_inst (
		.O(wn_dq_rd[i]),	// Buffer output [signal coming into fpga]
		.IO(ion_ddr_dq[i]),	// Buffer inout port (connect directly to top-level port)
		.I(wn_dq_wr[i]),	// Buffer input [signal going out of fpga]
		.T(wn_dq_iob_tristate[i])	// 3-state enable input, high=input, low=output
	);
end
endgenerate
/////////////////////////////////////////////////
// DQ IDELAY
/////////////////////////////////////////////////
generate
for (i = 0; i < p_DQ_W; i = i+1) begin
	IDELAYE2 #(
		.HIGH_PERFORMANCE_MODE("TRUE"), // Reduced jitter ("TRUE"), Reduced power ("FALSE")
		.IDELAY_TYPE(p_IDELAY_TYPE), // FIXED, VARIABLE, VAR_LOAD, VAR_LOAD_PIPE
		.IDELAY_VALUE(p_IDELAY_INIT_DQ), // Input delay tap setting (0-31)
		.REFCLK_FREQUENCY(REFCLK_FREQUENCY), // IDELAYCTRL clock input frequency in MHz (190.0-210.0, 290.0-310.0).
		.SIGNAL_PATTERN("DATA")
	) idelay_dq_inst (
		.CNTVALUEOUT(wn_dq_idelay_cnt_many[i]), // 5-bit output: Counter value output
		.DATAOUT(wn_dq_rd_delayed[i]), // 1-bit output: Delayed data output
		.C(i_clk_div), // 1-bit input: Clock input
		.CE(in_dq_delay_ce[i/8]), // 1-bit input: Active high enable increment/decrement input
		.CINVCTRL(1'b0), // 1-bit input: Dynamic clock inversion input
		.CNTVALUEIN(in_dq_idelay_cnt[(i/8)*5+:5]), // 5-bit input: Counter value input
		.DATAIN(1'b0), // 1-bit input: Internal delay data input
		.IDATAIN(wn_dq_rd[i]), // 1-bit input: Data input from the I/O
		.INC(in_dq_delay_inc[i/8]), // 1-bit input: Increment / Decrement tap delay input
		.LD(in_dq_delay_ld[i/8]), // 1-bit input: Load IDELAY_VALUE input
		.LDPIPEEN(1'b0), // 1-bit input: Enable PIPELINE register to load data input
		.REGRST(1'b0) // 1-bit input: Active-high reset tap-delay input
	);
end
endgenerate
/////////////////////////////////////////////////
// DQ ISERDES
/////////////////////////////////////////////////
generate
for (i = 0; i < p_DQ_W; i = i+1) begin
	ISERDESE2 #(
		.DATA_RATE("DDR"), // DDR, SDR
		.DATA_WIDTH(4), // Parallel data width (2-8,10,14)
						// In MEMORY + DDR mode, only DATA_WIDTH 4 supported, UG471 Table 3-3
		.INTERFACE_TYPE("MEMORY"), // MEMORY, MEMORY_DDR3, MEMORY_QDR, NETWORKING, OVERSAMPLE
		.IOBDELAY("IFD"/*"BOTH"*/), // NONE, BOTH, IBUF, IFD
		.NUM_CE(2) // Number of clock enables (1,2)
	) iserdes_dq_inst (
		.O(), // 1-bit output: Combinatorial output
		// Q1 - Q8: 1-bit (each) output: Registered data outputs
		.Q1(wn_iserdes_par[i+16*3]),
		.Q2(wn_iserdes_par[i+16*2]),
		.Q3(wn_iserdes_par[i+16*1]),
		.Q4(wn_iserdes_par[i+16*0]),
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
		.CE1(w2_iserdes_ce[1]),
		.CE2(w2_iserdes_ce[2]),
		.CLKDIVP(1'b0), // 1-bit input: TBD
		// Clocks: 1-bit (each) input: ISERDESE2 clock input ports
		.CLK(wn_dqs_rd_delayed_bufio[i/8]), // 1-bit input: High-speed clock
		.CLKB(~wn_dqs_rd_delayed_bufio[i/8]), // 1-bit input: High-speed secondary clock
		.CLKDIV(i_clk_div), // 1-bit input: Divided clock
		.OCLK(i_clk_ddr_90), // 1-bit input: High speed output clock used when INTERFACE_TYPE="MEMORY"
		// Dynamic Clock Inversions: 1-bit (each) input: Dynamic clock inversion pins to switch clock polarity
		.DYNCLKDIVSEL(1'b0), // 1-bit input: Dynamic CLKDIV inversion
		.DYNCLKSEL(1'b0), // 1-bit input: Dynamic CLK/CLKB inversion
		// Input Data: 1-bit (each) input: ISERDESE2 data input ports
		.D(1'b0), // 1-bit input: Data input
		.DDLY(wn_dq_rd_delayed[i]), // 1-bit input: Serial data from IDELAYE2
		.OFB(1'b0), // 1-bit input: Data feedback from OSERDESE2
		.OCLKB(~i_clk_ddr_90), // 1-bit input: High speed negative edge output clock
		.RST(r_phy_rst), // 1-bit input: Active high asynchronous reset
		// SHIFTIN1, SHIFTIN2: 1-bit (each) input: Data width expansion input ports
		.SHIFTIN1(1'b0),
		.SHIFTIN2(1'b0)
	);
end
endgenerate

/////////////////////////////////////////////////
// DQ OSERDES
/////////////////////////////////////////////////
generate
for (i = 0; i < p_DQ_W; i = i+1) begin
	OSERDESE2 #(
		.DATA_RATE_OQ("DDR"), // DDR, SDR
		.DATA_RATE_TQ("DDR"), // DDR, BUF, SDR
		.DATA_WIDTH(4), // Parallel data width (2-8,10,14)
		.TRISTATE_WIDTH(4), // 3-state converter width (1,4)
		.SERDES_MODE("MASTER")
	) oserdes_dq_inst (
		.OFB(), // 1-bit output: Feedback path for data
		.OQ(wn_dq_wr[i]), // 1-bit output: Data path output
		// SHIFTOUT1 / SHIFTOUT2: 1-bit (each) output: Data output expansion (1-bit each)
		.SHIFTOUT1(),
		.SHIFTOUT2(),
		.TBYTEOUT(), // 1-bit output: Byte group tristate
		.TFB(), // 1-bit output: 3-state control
		.TQ(wn_dq_iob_tristate[i]), // 1-bit output: 3-state control
		.CLK(i_clk_ddr_90), // 1-bit input: High speed clock
		.CLKDIV(i_clk_div), // 1-bit input: Divided clock
		// D1 - D8: 1-bit (each) input: Parallel data inputs (1-bit each)
		.D1(rn_oserdes_dq_par[i+16*0]),
		.D2(rn_oserdes_dq_par[i+16*1]),
		.D3(rn_oserdes_dq_par[i+16*2]),
		.D4(rn_oserdes_dq_par[i+16*3]),
		.D5(),
		.D6(),
		.D7(),
		.D8(),
		.OCE(1'b1), // 1-bit input: Output data clock enable
		.RST(r_phy_rst), // 1-bit input: Reset
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
end
endgenerate

/////////////////////////////////////////////////
// INPUT READ/WRITE COMMAND & ADDR FIFO
/////////////////////////////////////////////////
fifo_generator_0 cmdfifo_inst (
    .clk(i_clk_div),	// : IN STD_LOGIC;
    .srst(r_phy_rst),	// : IN STD_LOGIC;
    .din(wn_cmdfifo_din),	// : IN STD_LOGIC_VECTOR(lp_CMDFIFO_WIDTH-1 DOWNTO 0);
    .wr_en(i_phy_cmd_en),	// : IN STD_LOGIC;
    .rd_en(r_cmdfifo_rd),	// : IN STD_LOGIC;
    .dout(wn_cmdfifo_dout),	// : OUT STD_LOGIC_VECTOR(lp_CMDFIFO_WIDTH-1 DOWNTO 0);
    .full(w_cmdfifo_full),	// : OUT STD_LOGIC;
    .empty(w_cmdfifo_empty)// : OUT STD_LOGIC
);

//###############################################
//## LOGIC:
//###############################################
// state counter pipe
reg	[3:0]	rn_state_curr	= STATE_INIT;	// state of memory
reg	[3:0]	rn_state_1ahead	= STATE_MRS;	// next state of memory
reg	[3:0]	rn_state_2ahead	= STATE_MRS;	// next next state of memory
reg	[3:0]	rn_state_3ahead;	// combinational logic assigned to above
/*reg	[3:0]	rn_state_initseries [0:4];
initial begin: init_state_store
	rn_state_initseries[0] = STATE_INIT;
	rn_state_initseries[1] = STATE_MRS;
	rn_state_initseries[2] = STATE_MRS;
	rn_state_initseries[3] = STATE_MRS;
	rn_state_initseries[4] = STATE_MRS;
	rn_state_initseries[5] = STATE_ZQCL;
end */ // init_state_store

// refresh timer
localparam lp_REF_TMR_WIDTH = $clog2(lpdiv_RFC_MAX);
reg	[lp_REF_TMR_WIDTH-1:0]	rn_ref_tmr	= lpdiv_RFC_MAX - 1'b1;	// refresh timer
reg	r_ref_req	= 1'b0;	// refresh request flag
reg	r_act_req	= 1'b0;
reg	r_pre_req	= 1'b0;

// state timer
reg [lp_STATE_TMR_WIDTH-1:0]	rn_state_tmr	= lpdiv_CKE_LO-1;	// active state down-timer
reg [lp_STATE_TMR_WIDTH-1:0]	rn_state_tmr_next	= lpdiv_XPR-1;	// next starting value of state timer
reg	[lp_STATE_TMR_WIDTH-1:0]	rn_state_tmr_next_tmp;	// assigned to above

// init command counter (MRS -> MRS -> MRS -> MRS -> ZQCL)
reg	[2:0]	r3_init_cmd_ctr = 3'b0;

reg	[(p_BANK_W + p_ADDR_W)-1:0]	rn_mrs_addr	[1:4];
initial begin: init_mr_store
	rn_mrs_addr[1] = DLL.lp_MR2;
	rn_mrs_addr[2] = DLL.lp_MR3;
	rn_mrs_addr[3] = DLL.lp_MR1;
	rn_mrs_addr[4] = DLL.lp_MR0;
end // init_mr_store: Store MR register values

// read data valid signals
reg	r_rd_op = 1'b0;	// high when r3_cmd == lp_CMD_RD
reg	[5:0]	rn_rd_op_delayed = 'b0;	// r_rd_op pipe (delay)
reg	[1:0]	r2_iserdes_valid = 'b0;	// counts 2*divCLK cycles per r_rd_op
reg	r_rddata_valid = 1'b0;			// full BL8 read word valid 

reg	[8*p_DQ_W-1:0]	rn_rddata = {(p_DQ_W*8){1'b0}};// pipe (delay) of r2_rd_valid_prep[1]

// fifo pipe
reg	[p_BANK_W-1:0]		rn_bank_pipe	[0:2];
reg	[p_ROW_W-1:0]		rn_row_pipe	[1:2];
reg	[p_COL_W-1:0]		rn_col_pipe	[0:2];
reg	[(8*p_DQ_W)-1:0]	rn_wrd_pipe	[0:2];
reg	[7:0]				rn_wrm_pipe	[0:2];

// oserdes state machine control
reg	[2:0]	r3_dqs_state = 3'd0;
reg	r_oserdes_trig = 1'b0;	// state machine trigger
reg	r_oserdes_trig_pipe = 1'b0;
reg	[(p_DQ_W*8)-1:0]	rn_write_data = {(p_DQ_W*8){1'b1}};
reg	[(p_DQ_W*8)-1:0]	rn_wrdata_buf = {(p_DQ_W*8){1'b1}};

// direct output
reg	r_ddr_nrst = 1'b0;
reg	r_ddr_nrst_pipe = 1'b0;
reg	r_ddr_cke = 1'b0;
reg	r_ddr_cke_pipe = 1'b0;

reg	[(p_BANK_W-1):0]	rn_ddr_bank		= {p_BANK_W{1'b0}};
reg	[(p_BANK_W-1):0]	rn_ddr_bank_pipe	= {p_BANK_W{1'b0}};
reg	[(p_ADDR_W-1):0]	rn_ddr_addr	= {p_ADDR_W{1'b0}};
reg	[(p_ADDR_W-1):0]	rn_ddr_addr_pipe	= {p_ADDR_W{1'b0}};

reg	[2:0]	r3_cmd	= lp_CMD_NOP; // command bus {nRAS, nCAS, nWE}
reg	[2:0]	r3_cmd_pipe	= lp_CMD_NOP;

// OSERDES TRIGGER FLAG & OSERDES DATA
always @(posedge i_clk_div) begin: oserdes_signal
	// The OSERDES trigger flag is 1'b0 until timer reaches 1 or 0
	//	(depends on WL, which depends on frequency) in WR state and remains
	//	high until WR state is exited. This works for single bursts and
	//	sequential writes.
	case (rn_state_curr)
	STATE_WR: begin
		if (rn_state_tmr == (lpdiv_WL_MAX - DLL.lpdiv_WL)) begin
			r_oserdes_trig <= 1'b1;
			rn_write_data <= rn_wrd_pipe[0];
		end
	end
	default: begin
		r_oserdes_trig <= 1'b0;
	end
	endcase
end
// CKE PIN
always @(posedge i_clk_div) begin: cke_ctrl
	if (i_phy_rst)
		r_ddr_cke <= 1'b0;
		
	case (rn_state_curr)
	STATE_INIT: begin
		if (rn_state_tmr == 0)
			r_ddr_cke <= 1'b1;
	end
	default: r_ddr_cke <= 1'b1;
	endcase
end
// RST PIN
always @(posedge i_clk_div) begin: rst_ctrl
	r_ddr_nrst <= ~i_phy_rst;	// output pin to SDRAM
	r_phy_rst <= i_phy_rst;	// primitive reset
end
// CURRENT CMD
always @(posedge i_clk_div) begin: curr_cmd
	r3_cmd <= lp_CMD_NOP;	// command is NOP unless STATE timer is 0
	case (rn_state_curr)
	STATE_INIT: begin // CKE LO->HI
		//r3_cmd <= lp_CMD_NOP;
	end
	STATE_MRS: begin
		if (rn_state_tmr == 0)
			r3_cmd <= lp_CMD_MRS;
	end
	STATE_IDLE: begin
		/*if (rn_state_tmr == 0)
			r3_cmd <= lp_CMD_NOP;*/
	end
	STATE_REF: begin
		if (rn_state_tmr == 0)
			r3_cmd <= lp_CMD_REF;
	end
	STATE_PRE: begin
		if (rn_state_tmr == 0)
			r3_cmd <= lp_CMD_PRE;
	end
	STATE_ACT: begin
		if (rn_state_tmr == 0)
			r3_cmd <= lp_CMD_ACT;
	end
	STATE_WR: begin
		if (rn_state_tmr == 0)
			r3_cmd <= lp_CMD_WR;
	end
	STATE_RD: begin
		if (rn_state_tmr == 0)
			r3_cmd <= lp_CMD_RD;
	end
	STATE_IDLE: begin
		if (rn_state_tmr == 0)
			r3_cmd <= lp_CMD_NOP;
	end
	STATE_ZQCL: begin
		if (rn_state_tmr == 0)
			r3_cmd <= lp_CMD_ZQCL;
	end
	default: r3_cmd <= lp_CMD_NOP;
	endcase
end
// CONTROL OF INIT CMD SEQUENCE: CKE HIGH -> MR2 -> MR3 -> MR1 -> MR0 -> ZQINIT -> IDLE
always @(posedge i_clk_div) begin: init_cmd_ctr
	if (i_phy_rst)
		r3_init_cmd_ctr <= 2'b0;
	else if (!r_init_done)
		if (rn_state_tmr == 'd0)
			r3_init_cmd_ctr <= r3_init_cmd_ctr + 1;
end
always @(posedge i_clk_div) begin: init_flag_ctrl
	if (i_phy_rst)
		r_init_done <= 1'b0;
	else if (rn_state_curr == STATE_IDLE)
		r_init_done <= 1'b1;
end

// UPCOMING STATE
always @(posedge i_clk_div) begin: state_2ahead
	r_cmdfifo_rd <= 1'b0;
	if (i_phy_rst)
		rn_state_2ahead <= STATE_MRS;
		
	case (rn_state_2ahead)
	STATE_INIT: begin // impossible
		r_act_req <= 1'b0;
		r_pre_req <= 1'b0;
		if (rn_state_tmr == 0)
			rn_state_2ahead <= STATE_MRS;
	end
	STATE_MRS: begin
		r_act_req <= 1'b0;
		r_pre_req <= 1'b0; // reset values
		if (rn_state_tmr == 0)
			if (r3_init_cmd_ctr == 2)
				rn_state_2ahead <= STATE_ZQCL;	
			else
				rn_state_2ahead <= STATE_MRS;
	end
	STATE_REF: begin
		if (rn_state_tmr == 0)
			rn_state_2ahead <= STATE_IDLE;
	end
	STATE_PRE: begin
		if (rn_state_tmr == 0) begin
			if (r_ref_req)
				rn_state_2ahead <= STATE_REF;
			else
				rn_state_2ahead <= STATE_IDLE;
			r_pre_req <= 1'b0;
	end
	end
	STATE_ACT: begin
		if (rn_state_tmr == 0) begin
			if (w_cmdfifo_op == lp_CMDFIFO_OP_RD)
				rn_state_2ahead <= STATE_RD;
			else //if (w_cmdfifo_op == lp_CMDFIFO_OP_WR)
				rn_state_2ahead <= STATE_WR;
			if (!w_cmdfifo_empty)
				r_cmdfifo_rd <= 1'b1;
			else
				r_pre_req <= 1'b1;
			r_act_req <= 1'b0;
		end
	end
	// Note (regarding WRITE- or READ-to-PRECHARGE): Every write or read operation is
	//	followed by a precharge. This simplifies controller design -- insofar as further
	//	row open/close operations are concerned, -- but in turn kills random access speeds.
	
	// Note (regarding random access speeds): Aside from automatically precharging after
	//	r/w, random access is also harmed by no command parallelization (no bank machines),
	//	and no command reordering. The command buffer (state variable pipe) is only a pipe.
	
	// Note (regarding tRTP_max): Maximum row open time (tRTP_max = 9 * tREFI) cannot be
	//	exceeded. The write/read operation is interrupted if the refresh request flag is raised.
	STATE_WR: begin
		if (rn_state_tmr == 0) begin
			if (r_ref_req || r_pre_req || {rn_bank_pipe[2], rn_row_pipe[2], w_cmdfifo_op} != {wn_cmdfifo_bank, wn_cmdfifo_row, lp_CMDFIFO_OP_WR}) begin
				rn_state_2ahead <= STATE_PRE;
				if ({rn_bank_pipe[2], rn_row_pipe[2], w_cmdfifo_op} != {wn_cmdfifo_bank, wn_cmdfifo_row, lp_CMDFIFO_OP_WR})
					r_act_req <= 1'b1;
				if (r_ref_req && !r_pre_req)
					r_act_req <= 1'b1;
			end else begin /*if ({rn_bank_pipe[2], rn_row_pipe[2], w_cmdfifo_op} == {wn_cmdfifo_bank, wn_cmdfifo_row, lp_CMDFIFO_OP_WR}) begin*/
				rn_state_2ahead <= STATE_WR;
				if (w_cmdfifo_empty)
					r_pre_req <= 1'b1;
				else
					r_cmdfifo_rd <= 1'b1;
				/*if (w_cmdfifo_op == lp_CMDFIFO_OP_WR)
					rn_state_2ahead <= STATE_WR;
				else
					rn_state_3ahead <= STATE_RD;*/
			end /*else begin
				rn_state_2ahead <= STATE_PRE;
				r_act_req <= 1'b1;
			end*/
		end
	end
	STATE_RD: begin
		if (rn_state_tmr == 0) begin
			if (r_ref_req || r_pre_req || {rn_bank_pipe[2], rn_row_pipe[2], w_cmdfifo_op} != {wn_cmdfifo_bank, wn_cmdfifo_row, lp_CMDFIFO_OP_RD}) begin
				rn_state_2ahead <= STATE_PRE;
				if ({rn_bank_pipe[2], rn_row_pipe[2], w_cmdfifo_op} != {wn_cmdfifo_bank, wn_cmdfifo_row, lp_CMDFIFO_OP_RD})
					r_act_req <= 1'b1;
				if (r_ref_req && !r_pre_req)
					r_act_req <= 1'b1;
			end else begin /*if ({rn_bank_pipe[2], rn_row_pipe[2], w_cmdfifo_op} == {wn_cmdfifo_bank, wn_cmdfifo_row, lp_CMDFIFO_OP_WR}) begin*/
				rn_state_2ahead <= STATE_RD;
				if (w_cmdfifo_empty)
					r_pre_req <= 1'b1;
				else
					r_cmdfifo_rd <= 1'b1;
				/*if (w_cmdfifo_op == lp_CMDFIFO_OP_WR)
					rn_state_2ahead <= STATE_WR;
				else
					rn_state_3ahead <= STATE_RD;*/
			end /*else begin
				rn_state_2ahead <= STATE_PRE;
				r_act_req <= 1'b1;
			end*/
		end
	end
	STATE_IDLE: begin
		if (rn_state_tmr == 0)
			if (r_ref_req)
				// Note: Unless row is being acvitely accessed (R/W), all rows are
				//	precharged. Otherwise could check for r8_bank_active. But if banks
				//	are already precharged, PRECHARGE cmd is interpreted as NOP.
				rn_state_2ahead <= STATE_REF;
			else if (!w_cmdfifo_empty || r_act_req) begin
				rn_state_2ahead <= STATE_ACT;
				if (!r_act_req)
					r_cmdfifo_rd <= 1'b1;
				/*else
					r_cmdfifo_rd <= 1'b0;*/
			end else
				rn_state_2ahead <= STATE_IDLE;
	end
	STATE_ZQCL: begin
		if (rn_state_tmr == 0)
			rn_state_2ahead <= STATE_REF;
	end
	default: rn_state_2ahead <= STATE_IDLE;
	endcase
end
// REFRESH TIMER
always @(posedge i_clk_div) begin: refresh_timer
	if (i_phy_rst)
		 rn_ref_tmr <= lpdiv_REFI;
	else if (r_init_done)
		if (rn_ref_tmr > 0)
			rn_ref_tmr <= rn_ref_tmr - 1'b1;
		else // if (rn_ref_tmr == 0)
			rn_ref_tmr <= lpdiv_REFI;
end
// REFRESH FLAG
always @(posedge i_clk_div) begin: refresh_flag
	if (i_phy_rst)
		r_ref_req <= 1'b0;
	else if (rn_ref_tmr == 'd0)
		r_ref_req <= 1'b1;
	else if (rn_state_2ahead == STATE_REF)
		r_ref_req <= 1'b0; 
end
// STATE VAR PIPE
always @(posedge i_clk_div) begin: state_curr
	if (i_phy_rst) begin
		rn_state_curr <= STATE_INIT;
		rn_state_1ahead <= STATE_MRS;
	end else if (rn_state_tmr == 0) begin
		rn_state_curr <= rn_state_1ahead;
		rn_state_1ahead <= rn_state_2ahead;
	end
end
// CURRENT STATE TIMER
always @(posedge i_clk_div) begin: state_curr_timer
	if (i_phy_rst) begin
		rn_state_tmr <= lpdiv_CKE_LO;
		rn_state_tmr_next <= lpdiv_XPR;
	end else if (rn_state_tmr == 0) begin
		rn_state_tmr <= rn_state_tmr_next;
		rn_state_tmr_next <= rn_state_tmr_next_tmp;
	end else if (rn_state_tmr > 0)
		rn_state_tmr <= rn_state_tmr - 1;
end
// NEXT STATE COMMAND DELAY SETUP
always @(posedge i_clk_div) begin: next_state_timer
	/* Transitions to STATE_IDLE assume worst state transitions */
	//rn_state_tmr_next_tmp = 'd1; // Only here to avoid latch.
	case (rn_state_1ahead)
	STATE_INIT: begin	/*0*/
		rn_state_tmr_next_tmp <= lpdiv_XPR;
	end
	STATE_MRS: begin	/*1*/
		if (rn_state_2ahead == STATE_MRS)
			rn_state_tmr_next_tmp <= lpdiv_MRD;
		else // if (rn_state_2ahead == STATE_ZQCL)
			rn_state_tmr_next_tmp <= lpdiv_MOD;
	end
	STATE_REF: begin	/*2*/
		rn_state_tmr_next_tmp <= lpdiv_RFC_MIN;
		// Note: tREFI is automatically met (max 16 REFRESHes allowed within 2 x tREFI).
		//	This core refreshes approx. once per whatever is set in the "refresh_timer"
		//	block (ctrl+f).
	end
	STATE_PRE: begin	/*3*/
		rn_state_tmr_next_tmp <= ((lpdiv_RP == 0) ? 1'b1 : lpdiv_RP);
	end
	STATE_ACT: begin	/*4*/
		rn_state_tmr_next_tmp <= ((lpdiv_RCD == 0) ? 1'b1 : lpdiv_RCD);
		// TODO: Meet tFAW (only 4 ACTs within tFAW)
		// tRRD (ACT-to-ACT in different banks) is automatically met
	end
	STATE_WR: begin	/*5*/
		if (rn_state_2ahead == STATE_WR)
			rn_state_tmr_next_tmp <= lpdiv_CCD;
		else
			rn_state_tmr_next_tmp <= DLL.lpdiv_WR;
	end
	STATE_RD: begin	/*6*/
		if (rn_state_2ahead == STATE_RD)
			rn_state_tmr_next_tmp <= lpdiv_CCD;
		else
			rn_state_tmr_next_tmp <= lpdiv_RTP;
	end
	STATE_IDLE: begin	/*7*/
		rn_state_tmr_next_tmp <= 'd1;
		// TODO: Can we set this to 0?
	end
	STATE_ZQCL: begin	/*8*/
		rn_state_tmr_next_tmp <= lpdiv_ZQINIT;
	end
	default: rn_state_tmr_next_tmp <= 'd1;
	endcase
end
always @(posedge i_clk_div) begin: addr_ctrl
	case (rn_state_curr)
	STATE_MRS: begin
		{rn_ddr_bank, rn_ddr_addr} <= rn_mrs_addr[r3_init_cmd_ctr];
	end
	STATE_ACT: begin
		rn_ddr_addr[13:0] <= rn_row_pipe[1];
		rn_ddr_bank <= rn_bank_pipe[1];
	end
	STATE_WR, STATE_RD: begin
		rn_ddr_addr[13:0] <= {4'b0000, rn_col_pipe[0]};
		//rn_ddr_addr[10] <= 1'b0;	// disable AP
		rn_ddr_bank <= rn_bank_pipe[0];
	end
	STATE_PRE: begin
		rn_ddr_addr[13:0] <= 14'h0400;
		//rn_ddr_addr[10] <= 1'b1;	// precharge all banks
	end
	STATE_ZQCL: begin
		rn_ddr_addr[13:0] <= 14'h400;
		//rn_ddr_addr[10] <= 1'b1;	// ZQ cal LONG
	end
	default: begin
		//;
		rn_ddr_addr <= 14'b0;
		rn_ddr_bank <= 3'b0;
	end
	endcase
end
always @(posedge i_clk_div) begin: rd_valid_ctrl
	r_rd_op <= 1'b0;
	case (rn_state_curr)
	STATE_RD: begin
		if (rn_state_tmr == 0)
			r_rd_op <= 1'b1;
	end
	default: r_rd_op <= 1'b0;
	endcase
	rn_rd_op_delayed <= {rn_rd_op_delayed[4:0], r_rd_op};
	
	case (r2_iserdes_valid)
	'b01: begin
		if (rn_rd_op_delayed[p_RD_DELAY])
			r2_iserdes_valid <= 2'b10;
	end
	'b10: begin
		r2_iserdes_valid <= 2'b11;
	end
	'b11: begin
		if (rn_rd_op_delayed[p_RD_DELAY])
			r2_iserdes_valid <= 2'b10;
		else
			r2_iserdes_valid <= 2'b01;
	end
	default: r2_iserdes_valid <= 2'b01;
	endcase
	
	if (r2_iserdes_valid[1])
		if(!r2_iserdes_valid[0])
			rn_rddata[8*p_DQ_W-1 : 4*p_DQ_W] <= wn_iserdes_par;
		else// if (r2_iserdes_valid[0])
			rn_rddata[4*p_DQ_W-1:0] <= wn_iserdes_par;
			
	if (r2_iserdes_valid == 2'b11)
		r_rddata_valid <= 1'b1;
	else
		r_rddata_valid <= 1'b0;
end
always @(posedge i_clk_div) begin: fifo_ctrl
	if (rn_state_tmr == 0) begin
		// pipe[2] as buffer between fifo and pipe[1]
		rn_bank_pipe[2] <= wn_cmdfifo_bank;
		rn_row_pipe[2] <= wn_cmdfifo_row;
		rn_col_pipe[2] <= wn_cmdfifo_col;
		rn_wrd_pipe[2] <= wn_cmdfifo_wrd;
		rn_wrm_pipe[2] <= wn_cmdfifo_wrm;
			
		// pipe[1] for ACT
		rn_bank_pipe[1] <= rn_bank_pipe[2];
		rn_row_pipe[1] <= rn_row_pipe[2];
		rn_col_pipe[1] <= rn_col_pipe[2];
		rn_wrd_pipe[1] <= rn_wrd_pipe[2];
		rn_wrm_pipe[1] <= rn_wrm_pipe[2];
		
		// pipe[0] for RD/WR
		rn_bank_pipe[0] <= rn_bank_pipe[1];
		rn_col_pipe[0] <= rn_col_pipe[1];
		rn_wrd_pipe[0] <= rn_wrd_pipe[1];
		rn_wrm_pipe[0] <= rn_wrm_pipe[1];
	end
end
always @(posedge i_clk_div) begin: oserdes_ctrl
	rn_wrdata_buf <= rn_write_data;
	
	case (r3_dqs_state)
	'd0: begin
		if (w_oserdes_trig) begin
		//if ((rn_state_curr == STATE_WR) && (rn_state_tmr == (lpdiv_WL_MAX - DLL.lpdiv_WL))) begin			
			// setup dqs oserdes for preamble
			r4_oserdes_dqs_par <= 'h1;//4'h1;
			r4_tristate_dqs <= 'hE; //'hE;
			
			// setup dq oserdes for no output
			// r4_oserdes_dq_par <= 
			r4_tristate_dq <= 'hF;
			
			// next div cycle
			r3_dqs_state <= 'd3;
		end else begin
			r4_tristate_dqs <= 'hF;
			r4_tristate_dq <= 'hF;
		end
	end
	'd3: begin
		// dqs oserdes normal toggle 1/2
		r4_oserdes_dqs_par <= 4'h5;
		r4_tristate_dqs <= 'h0;
		
		// dq oserdes
		rn_oserdes_dq_par <= rn_wrdata_buf[(p_DQ_W*8)-1:(p_DQ_W*4)];
		r4_tristate_dq <= 'h3;
				
		r3_dqs_state <= 'd4;
	end
	// burst write continuation:
	'd7: begin
		// dqs oserdes normal toggle 1/2 (continued from last word)
		r4_oserdes_dqs_par <= 4'h5;
		r4_tristate_dqs <= 'h0;
		
		// dq oserdes
		rn_oserdes_dq_par <= rn_wrdata_buf[(p_DQ_W*8)-1:(p_DQ_W*4)];
		r4_tristate_dq <= 'h0;
		
		r3_dqs_state <= 'd4;
	end
	'd4: begin
		// dqs normal toggle 2/2
		r4_oserdes_dqs_par <= 4'h5;
		r4_tristate_dqs <= 'h0;
		
		rn_oserdes_dq_par <= rn_wrdata_buf[(p_DQ_W*4)-1:0];
		r4_tristate_dq <= 'h0;
		
		//r4_dq_tmr <= 'd2;
		// next div cycle?
		if (w_oserdes_trig) begin
			r3_dqs_state <= 'd7;
		end else
			r3_dqs_state <= 'd5;
	end
	'd5: begin
		// final 4 dq bits
		r4_oserdes_dqs_par <= 4'h0;
		r4_tristate_dqs <= 'h7;
		
		r4_tristate_dq <= 'hC;
		
		r3_dqs_state <= 'd0;
	end
	default: ;
	endcase
end
always @(posedge i_clk_div) begin: output_sig_pipe
	r_oserdes_trig_pipe <= r_oserdes_trig;
	
	r_ddr_cke_pipe <= r_ddr_cke;
	
	r3_cmd_pipe <= r3_cmd;
	
	rn_ddr_bank_pipe <= rn_ddr_bank;
	rn_ddr_addr_pipe <= rn_ddr_addr;
end
//// output multiplexers: make output 1 divCK delayed or no (could improve timing for high frequencies)
wire	w_oserdes_trig	= (p_OUTPUT_PIPE == "TRUE") ? r_oserdes_trig_pipe : r_oserdes_trig;

wire	w_ddr_cke	= (p_OUTPUT_PIPE == "TRUE") ? r_ddr_cke_pipe : r_ddr_cke;

wire	[2:0]	w3_cmd	= (p_OUTPUT_PIPE == "TRUE") ? r3_cmd_pipe : r3_cmd;

wire	[(p_BANK_W-1):0]	wn_ddr_bank	= (p_OUTPUT_PIPE == "TRUE") ? rn_ddr_bank_pipe : rn_ddr_bank;
wire	[(p_ADDR_W-1):0]	wn_ddr_addr	= (p_OUTPUT_PIPE == "TRUE") ? rn_ddr_addr_pipe : rn_ddr_addr;
 

assign o_phy_rddata_valid = r_rddata_valid;//r_init_done;
assign on_phy_rddata = rn_rddata;
assign on_iserdes_par = wn_iserdes_par;

//assign o_phy_rddata_end = (r2_rd_valid_prep == 2'b11) ? 1'b1 : 1'b0;

assign o_phy_init_done = r_init_done;

assign o_phy_idelay_rdy = w_idelay_rdy;


assign o_phy_cmd_full = w_cmdfifo_full;


assign on_dq_idelay_cnt = wn_dq_idelay_cnt_few;
assign on_dqs_idelay_cnt = wn_dqs_idelay_cnt;

// Hardware out assigns
assign o_ddr_nrst	= r_ddr_nrst;

assign o_ddr_ncs	= (DLL.lp_CWL % 2) ? i_clk_div_n : i_clk_div;
assign o_ddr_nras 	= w3_cmd[2];
assign o_ddr_ncas 	= w3_cmd[1];
assign o_ddr_nwe 	= w3_cmd[0];

assign on_ddr_dm	= 2'b00;

assign o_ddr_cke	= w_ddr_cke;

assign o_ddr_odt	= 1'b0;

assign on_ddr_bank	= wn_ddr_bank;
assign on_ddr_addr	= wn_ddr_addr;
endmodule
