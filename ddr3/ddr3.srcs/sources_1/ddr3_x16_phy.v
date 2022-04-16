`timescale 1ns / 1ps

`define SIMULATION

`define	max2(v1, v2) ((v1) > (v2) ? (v1) : (v2))
`define	ck2ps(ddrfreq) (1_000_000/``ddrfreq``) // use ck2ps(p_DDR_FREQ_MHZ) to get period in ps

module ddr3_x16_phy #(
	parameter	p_IDELAY_INIT_DQS	= 5,//31,
	parameter	p_IDELAY_INIT_DQ	= 0,
	
	parameter	REFCLK_FREQUENCY	= 200.0,	// IDELAY resolution = 1000/(32 x 2 x REFCLK_FREQUENCY) [ns]
												// For 200 MHz, tap delay is 0.078125 ns
	
	parameter	p_DDR_FREQ_MHZ	= 320,	// use ck2ps(p_DDR_FREQ_MHZ) to get period in ps
										// JEDEC allows > 300 MHz with DLL ON, or < 125 MHZ with DLL OFF
	parameter	p_DDR_CK_PS		= `ck2ps(p_DDR_FREQ_MHZ),
	
								// (ps)					// Description
	parameter	p_CKE			= `max2(3*p_DDR_CK_PS, 5_000),		// CKE minimum pulse width
	parameter	p_FAW			= 450_000,				// Four Activate Window (4x ACTIVATE to 5th ACTIVATE)
	parameter	p_RAS			= 36_000,				// ACTIVATE-to-PRECHARGE
	parameter	p_RCD			= 13_500,				// ACTIVATE-to-READ or ACTIVATE-to-WRITE delay
	//parameter	p_REFI			= 7800_000,				// Average periodic refresh interval
	parameter	p_RFC_MIN		= 160_000,				// REFRESH to ACTIVATE or REFRESH to REFRESH
	parameter	p_RFC_MAX		= 7_020_000,
	parameter	p_RP			= 13_500,				// Precharge command period
	parameter	p_RRD			= `max2(4*p_DDR_CK_PS, 10_000),	// ACTIVATE to ACTIVATE in different banks
	parameter	p_RTP			= 7_500,				// READ to PRECHARGE
	parameter	p_WTR			= 7_500,				// WRITE to READ
	parameter	p_WR			= `max2(4*p_DDR_CK_PS, 15000),	// WRITE recovery time (WRITE to PRECHARGE)
	parameter	p_XPR			= `max2(5*p_DDR_CK_PS, p_RFC_MIN + 10_000),	// Exit reset from CKE HIGH to valid command
	parameter	p_MOD			= `max2(12*p_DDR_CK_PS, 15_000),		// MRS-to-non-MRS (MRS update delay)
	parameter	p_ZQINIT		= `max2(512*p_DDR_CK_PS, 640_000)		// ZQ Calibration Long time from reset
)(
	input	[1:0]	i2_iserdes_ce,
	output	[12:0]	o13_init_ctr,
	output	[3:0]	o4_test_state,
	input	i_test_redo,
	output	[255:0]	o256_test,
	output	[63:0]	o64_iserdes,
	input	i_clk_ddr,	// chip clock frequency
	input	i_clk_ddr_90,	// same but delayed by 90°, used to generate output DQ from OSERDES
	input	i_clk_ref,	// 200 MHz, used for IDELAYCTRL, which controls taps for input DQS IDELAY
	
	input	i_clk_div,
	
	input	i_phy_rst,	// active high reset for ODDR, OSERDES, ISERDES, IDELAYCTRL, hold HIGH until all clocks are generated

	input	[13:0]	i14_phy_addr,
	input	[127:0]	i128_phy_wrdata,	// eight words of write data for OSERDES (out of 8 for a total of BL8)
	output	[63:0]	o64_phy_rddata,	// four words of read data from ISERDES (out of 8 for a total of BL8)
	
	output	o_calib_done,
		
	// CONNECTION TO DRAM	
	inout	[15:0]	io16_ddr_dq,
	inout	[1:0]	io2_ddr_dqs_p,
	inout	[1:0]	io2_ddr_dqs_n,
	
	output	[13:0]	o14_ddr_addr,

	output	o_ddr_ck_p,
	output	o_ddr_ck_n,
	
	// CONNECTION TO DRAM used by CTRL (ADDR, BANK, CS/RAS/CAS/WE, ODT, CKE, UDM/LDM)
	output	[1:0]	o2_ddr_dm,
	output	[2:0]	o3_ddr_bank,

	output	o_ddr_nrst,
	output	o_ddr_cke,
	output	o_ddr_ncs,
	output	o_ddr_nras,
	output	o_ddr_ncas,
	output	o_ddr_nwe,
	output	o_ddr_odt
);

/////////////////////////////////////////////////
// Memory configuration and parametrization
/////////////////////////////////////////////////
// #################### DLL OFF, <125 MHZ #################### 
if	(p_DDR_CK_PS > 8000) begin: DLL
localparam lp_CL = 6;
localparam lp_CWL = 6;
localparam lp_AL = 0;
localparam lp_WL = lp_CWL + lp_AL;
localparam lp_RL = lp_CL + lp_AL - 1;	// DLL OFF: -1
localparam	lpdiv_WL	= lp_WL/2+1;
localparam	lpdiv_RL	= lp_RL/2+1;

// MRx is {BA[3:0], A[13:0]}
					  /* 13 12 11 10 ~9 ~8 ~7 ~6 ~5 ~4 ~3 ~2 ~1 ~0 */
localparam lp_MR0 = 14'b__0__0__0__0__1__1__0__0__1__0__0__0__0__0;
localparam lp_MR1 = 14'b__0__0__0__0__0__0__0__0__0__0__0__0__0__0;
localparam lp_MR2 = 14'b__0__0__0__0__0__0__0__0__0__0__1__0__0__0;
localparam lp_MR3 = 14'b__0__0__0__0__0__0__0__0__0__0__0__0__0__0;

// #################### 300-333 MHz #################### 
end else if ((p_DDR_CK_PS > 2999) & (p_DDR_CK_PS < 3334)) begin: DLL
localparam lp_CL = 5;
localparam lp_CWL = 5;
localparam lp_AL = 0;
localparam lp_WL = lp_CWL + lp_AL;
localparam lp_RL = lp_CL + lp_AL;
localparam	lpdiv_WL	= lp_WL/2+1;
localparam	lpdiv_RL	= lp_RL/2+1;
					  /* 13 12 11 10 ~9 ~8 ~7 ~6 ~5 ~4 ~3 ~2 ~1 ~0 */
localparam lp_MR0 = 14'b__0__0__0__0__1__1__0__0__0__1__0__0__0__0;
localparam lp_MR1 = 14'b__0__0__0__0__0__0__0__1__0__0__0__1__0__0;
localparam lp_MR2 = 14'b__0__0__0__0__0__0__0__0__0__0__0__0__0__0;
localparam lp_MR3 = 14'b__0__0__0__0__0__0__0__0__0__0__0__0__0__0;

// #################### TODO 300-400 MHz #################### 
end else if ((p_DDR_CK_PS > 2499) & (p_DDR_CK_PS < 3334)) begin: DLL
localparam lp_CL = 6;
localparam lp_CWL = 5;
localparam lp_AL = 0;
localparam lp_WL = lp_CWL + lp_AL;
localparam lp_RL = lp_CL + lp_AL;
localparam	lpdiv_WL	= lp_WL/2+1;
localparam	lpdiv_RL	= lp_RL/2+1;
					  /* 13 12 11 10 ~9 ~8 ~7 ~6 ~5 ~4 ~3 ~2 ~1 ~0 */
localparam lp_MR0 = 14'b__0__0__0__1__0__1__0__0__1__0__0__0__0__0;
localparam lp_MR1 = 14'b__0__0__0__0__0__0__0__1__0__0__0__1__0__0;
localparam lp_MR2 = 14'b__0__0__0__0__0__0__0__0__0__0__0__0__0__0;
localparam lp_MR3 = 14'b__0__0__0__0__0__0__0__0__0__0__0__0__0__0;


// #################### 125-300, >400 MHz ERROR #################### 
end else begin: DLL
localparam lp_CL = 0;
localparam lp_CWL = 0;
localparam lp_AL = 0;
unsupported_frequency_error_generation BAD_FREQ();
end

localparam	lpdiv_WL_MAX = 6/2+1;	// only CWL 5, 6 supported (no way this does > 533 MHz)
									// if CWL = 5, OSERDES must be triggered one CLKDIV period earlier
localparam lp_MR3_MPR = 14'b__0__0__0__0__0__0__0__0__0__0__0__1__0__0; // to be used when reading 1010 from MPR
/////////////////////////////////////////////////
// Timing parameter to DIV CK conversion
/////////////////////////////////////////////////

localparam	lpdiv_MRD	= 4/2;	//520/2;// MRS cycle time (4 CK)
localparam	lpdiv_CCD	= 4/2;	// WRITE-to-WRITE (actually just BL/2)(CCD = CAS#-to-CAS# delay)

localparam	lp_DIV_FREQ_MHZ	= p_DDR_FREQ_MHZ/2;
localparam	lp_DIV_CK_PS	= `ck2ps(lp_DIV_FREQ_MHZ);

localparam	lpdiv_RCD	= p_RCD/lp_DIV_CK_PS+1;	//-500_000/lp_DIV_CK_PS;// ACTIVATE-to-READ or ACTIVATE-to-WRITE delay
//localparam	lpdiv_REFI	= p_REFI/lp_DIV_CK_PS+1;	// Average periodic refresh interval
localparam	lpdiv_RFC_MIN	= p_RFC_MIN/lp_DIV_CK_PS+1;	// REFRESH-to-ACTIVATE or REFRESH-to-REFRESH
localparam	lpdiv_RFC_MAX	= p_RFC_MAX/lp_DIV_CK_PS+1;
localparam	lpdiv_RP	= p_RP/lp_DIV_CK_PS+1;	// PRECHARGE-to-PRECHARGE
localparam	lpdiv_RRD	= p_RRD/lp_DIV_CK_PS+1;	// ACTIVATE-to-ACTIVATE in different banks
localparam	lpdiv_RTP	= p_RTP/lp_DIV_CK_PS+1;	// READ-to-PRECHARGE
localparam	lpdiv_WTR	= p_WTR/lp_DIV_CK_PS+1;	// WRITE-to-READ
localparam	lpdiv_WR	= p_WR/lp_DIV_CK_PS+1;	// WRITE-to-PRECHARGE (WRITE recovery time)
localparam	lpdiv_XPR	= p_XPR/lp_DIV_CK_PS+1;	//-500_000/lp_DIV_CK_PS;// Exit reset from CKE HIGH to valid command
localparam	lpdiv_MOD	= p_MOD/lp_DIV_CK_PS+1;	//-500_000/lp_DIV_CK_PS;// MRS-to-non-MRS (MRS update delay)
localparam	lpdiv_ZQINIT	= p_ZQINIT/lp_DIV_CK_PS+1;	//-1_000_000/lp_DIV_CK_PS;// Long calibration time


//localparam	lpdiv_WL	= lp_WL/2+1;
//localparam	lpdiv_RL	= lp_RL/2+1;

// timer values
`ifdef SIMULATION
localparam lpdiv_NRST_LO = 1 * p_DDR_FREQ_MHZ/2; // RESET#: After power stable, RESET# held LOW for >200 us.
localparam lpdiv_CKE_LO = 2 * p_DDR_FREQ_MHZ/2; // CKE: After RESET# transitions HIGH wait >500 us with CKE LOW.
`else
localparam lpdiv_NRST_LO = 250 * p_DDR_FREQ_MHZ/2; // RESET#: After power stable, RESET# held LOW for >200 us.
localparam lpdiv_CKE_LO = 501 * p_DDR_FREQ_MHZ/2; // CKE: After RESET# transitions HIGH wait >500 us with CKE LOW.
`endif
localparam lpdiv_INIT_CTR_PLAY = 50/2;
localparam lpdiv_INIT_CTR_START = lpdiv_NRST_LO + lpdiv_CKE_LO + lpdiv_XPR + 3 * lpdiv_MRD + lpdiv_MOD + lpdiv_ZQINIT + lpdiv_INIT_CTR_PLAY;

localparam lp_INIT_CTR_WIDTH = $clog2(lpdiv_INIT_CTR_START);

/////////////////////////////////////////////////
// PHY primitive connections
/////////////////////////////////////////////////
reg	r_phy_rst = 1'b1;	// primitive reset
reg	r_calib_done = 1'b0;

// IOB -> IDELAY
wire	[1:0]	w2_dqs_rd;
wire	[15:0]	w16_dq_rd;

// IDELAY -> ISERDES
wire	[1:0]	w2_dqs_rd_delayed;
wire	[15:0]	w16_dq_rd_delayed;

// OSERDES -> IOB (data)
wire	[1:0]	w2_dqs_wr;	
wire	[15:0]	w16_dq_wr;

// OSERDES -> IOB (tristate ctrl)
wire	[1:0]	w2_dqs_iob_tristate;	
wire	[15:0]	w16_dq_iob_tristate;

// DDR data values (into OSERDES)
reg	[0:3]	r4_oserdes_dqs_par = 'hF;
reg	[0:63]	r64_oserdes_dq_par = {64{1'b1}};

// DDR tristate values (into OSERDES)
reg	[0:3]	r4_tristate_dqs = 'hF;	
reg	[0:3]	r4_tristate_dq = 'hF;

wire	[0:63]	w64_iserdes_par;	// data read from memory (from ISERDES)

wire	[2:1]	w2_iserdes_ce;	// ISERDES primitive clock enable (2:1); either 'b00 or 'b01/10
//assign w2_iserdes_ce[1] = 1'b1;//~i_clk_div;
wire	[1:0]	w2_iserdes_en;	// Set 'b10 or 'b01 to iserdes CE ^
reg	r_iserdes_en = 1'b0;		// Logic control to enable or disable ISERDES operation/CE
assign w2_iserdes_en = (DLL.lp_RL%2==0) ? 2'b01 : 2'b10;
assign w2_iserdes_ce = i2_iserdes_ce;//-2'b11;
//assign w2_iserdes_ce = (r_iserdes_en) ? w2_iserdes_en : 2'b00;


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
	.I(~i_clk_ddr)
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
for (i = 0; i < 2; i = i+1) begin
	IOBUFDS #(
		.IOSTANDARD("DIFF_SSTL135")	// Specify the I/O standard
	) iobufds_dqs_inst (
		.O(w2_dqs_rd[i]),	// Buffer output
		.IO(io2_ddr_dqs_p[i]),	// Diff_p inout (connect directly to top-level port)
		.IOB(io2_ddr_dqs_n[i]),	// Diff_n inout (connect directly to top-level port)
		.I(w2_dqs_wr[i]),	// Buffer input
		.T(w2_dqs_iob_tristate[i])	// 3-state enable input, high=input, low=output
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
		.IDELAY_TYPE("VARIABLE"), // FIXED, VARIABLE, VAR_LOAD, VAR_LOAD_PIPE
		.IDELAY_VALUE(p_IDELAY_INIT_DQS), // Input delay tap setting (0-31)
		.REFCLK_FREQUENCY(REFCLK_FREQUENCY) // IDELAYCTRL clock input frequency in MHz (190.0-210.0, 290.0-310.0).
	) idelay_dqs_inst (
		.CNTVALUEOUT(), // 5-bit output: Counter value output
		.DATAOUT(w2_dqs_rd_delayed[i]), // 1-bit output: Delayed data output
		.C(i_clk_div), // 1-bit input: Clock input
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
// DQS OSERDES
/////////////////////////////////////////////////
generate
for (i = 0; i < 2; i = i+1) begin
	OSERDESE2 #(
		.DATA_RATE_OQ("DDR"), // DDR, SDR
		.DATA_RATE_TQ("DDR"), // DDR, BUF, SDR
		.DATA_WIDTH(4), // Parallel data width (2-8,10,14)
		.TRISTATE_WIDTH(4), // 3-state converter width (1,4)
		.SERDES_MODE("MASTER")
	) oserdes_dqs_inst (
		.OFB(), // 1-bit output: Feedback path for data
		.OQ(w2_dqs_wr[i]), // 1-bit output: Data path output
		// SHIFTOUT1 / SHIFTOUT2: 1-bit (each) output: Data output expansion (1-bit each)
		.SHIFTOUT1(),
		.SHIFTOUT2(),
		.TBYTEOUT(), // 1-bit output: Byte group tristate
		.TFB(), // 1-bit output: 3-state control
		.TQ(w2_dqs_iob_tristate[i]), // 1-bit output: 3-state control
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
for (i = 0; i < 16; i = i+1) begin
	IOBUF #(
		.IOSTANDARD("SSTL135"),	// Specify the I/O standard
		.SLEW("FAST")	// Specify the output slew rate
	) iobuf_dq_inst (
		.O(w16_dq_rd[i]),	// Buffer output [signal coming into fpga]
		.IO(io16_ddr_dq[i]),	// Buffer inout port (connect directly to top-level port)
		.I(w16_dq_wr[i]),	// Buffer input [signal going out of fpga]
		.T(w16_dq_iob_tristate[i])	// 3-state enable input, high=input, low=output
	);
end
endgenerate
/////////////////////////////////////////////////
// DQ IDELAY
/////////////////////////////////////////////////
generate
for (i = 0; i < 16; i = i+1) begin
	IDELAYE2 #(
		.HIGH_PERFORMANCE_MODE("TRUE"), // Reduced jitter ("TRUE"), Reduced power ("FALSE")
		.IDELAY_TYPE("VARIABLE"), // FIXED, VARIABLE, VAR_LOAD, VAR_LOAD_PIPE
		.IDELAY_VALUE(p_IDELAY_INIT_DQ), // Input delay tap setting (0-31)
		.REFCLK_FREQUENCY(REFCLK_FREQUENCY) // IDELAYCTRL clock input frequency in MHz (190.0-210.0, 290.0-310.0).
	) idelay_dq_inst (
		.CNTVALUEOUT(), // 5-bit output: Counter value output
		.DATAOUT(w16_dq_rd_delayed[i]), // 1-bit output: Delayed data output
		.C(i_clk_div), // 1-bit input: Clock input
		.CE(1'b0), // 1-bit input: Active high enable increment/decrement input
		.CINVCTRL(1'b0), // 1-bit input: Dynamic clock inversion input
		.CNTVALUEIN(5'b0), // 5-bit input: Counter value input
		.DATAIN(1'b0), // 1-bit input: Internal delay data input
		.IDATAIN(w16_dq_rd[i]), // 1-bit input: Data input from the I/O
		.INC(1'b0), // 1-bit input: Increment / Decrement tap delay input
		.LD(1'b0), // 1-bit input: Load IDELAY_VALUE input
		.LDPIPEEN(1'b0), // 1-bit input: Enable PIPELINE register to load data input
		.REGRST(1'b0) // 1-bit input: Active-high reset tap-delay input
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
		.IOBDELAY(/*"IFD"*/"BOTH"), // NONE, BOTH, IBUF, IFD
		.NUM_CE(2) // Number of clock enables (1,2)
	) iserdes_dq_inst (
		.O(), // 1-bit output: Combinatorial output
		// Q1 - Q8: 1-bit (each) output: Registered data outputs
		.Q1(w64_iserdes_par[i+16*0]),
		.Q2(w64_iserdes_par[i+16*1]),
		.Q3(w64_iserdes_par[i+16*2]),
		.Q4(w64_iserdes_par[i+16*3]),
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
		.CLK(w2_dqs_rd_delayed[i/8]), // 1-bit input: High-speed clock
		.CLKB(~w2_dqs_rd_delayed[i/8]), // 1-bit input: High-speed secondary clock
		.CLKDIV(i_clk_div), // 1-bit input: Divided clock
		.OCLK(i_clk_ddr_90), // 1-bit input: High speed output clock used when INTERFACE_TYPE="MEMORY"
		// Dynamic Clock Inversions: 1-bit (each) input: Dynamic clock inversion pins to switch clock polarity
		.DYNCLKDIVSEL(1'b0), // 1-bit input: Dynamic CLKDIV inversion
		.DYNCLKSEL(1'b0), // 1-bit input: Dynamic CLK/CLKB inversion
		// Input Data: 1-bit (each) input: ISERDESE2 data input ports
		.D(1'b0), // 1-bit input: Data input
		.DDLY(w16_dq_rd_delayed[i]), // 1-bit input: Serial data from IDELAYE2
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
for (i = 0; i < 16; i = i+1) begin
	OSERDESE2 #(
		.DATA_RATE_OQ("DDR"), // DDR, SDR
		.DATA_RATE_TQ("DDR"), // DDR, BUF, SDR
		.DATA_WIDTH(4), // Parallel data width (2-8,10,14)
		.TRISTATE_WIDTH(4), // 3-state converter width (1,4)
		.SERDES_MODE("MASTER")
	) oserdes_dq_inst (
		.OFB(), // 1-bit output: Feedback path for data
		.OQ(w16_dq_wr[i]), // 1-bit output: Data path output
		// SHIFTOUT1 / SHIFTOUT2: 1-bit (each) output: Data output expansion (1-bit each)
		.SHIFTOUT1(),
		.SHIFTOUT2(),
		.TBYTEOUT(), // 1-bit output: Byte group tristate
		.TFB(), // 1-bit output: 3-state control
		.TQ(w16_dq_iob_tristate[i]), // 1-bit output: 3-state control
		.CLK(i_clk_ddr_90), // 1-bit input: High speed clock
		.CLKDIV(i_clk_div), // 1-bit input: Divided clock
		// D1 - D8: 1-bit (each) input: Parallel data inputs (1-bit each)
		.D1(r64_oserdes_dq_par[i+16*0]),
		.D2(r64_oserdes_dq_par[i+16*1]),
		.D3(r64_oserdes_dq_par[i+16*2]),
		.D4(r64_oserdes_dq_par[i+16*3]),
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


//###############################################
//## PHY LOGIC:
//###############################################

/////////////////////////////////////////////////
// State machine state definitions
/////////////////////////////////////////////////
localparam	lp_STATE_RST	= 4'b0000;
localparam	lp_STATE_INIT	= 4'b0001;

reg	[5:0]	rn_state_curr = 'd13;//lp_STATE_RST;
/////////////////////////////////////////////////
// Command defines; {(nCS,) nRAS, nCAS, nWE}; See Micron datasheet Table 87
/////////////////////////////////////////////////
localparam lp_CMD_MRS	= 3'b0000;	// MODE REGISTER SET
localparam lp_CMD_REF	= 3'b0001;	// REFRESH
localparam lp_CMD_PRE	= 3'b0010;	// & A10 LOW: Single-bank PRECHARGE
									// & A10 HIGH: PRECHARGE all banks
localparam lp_CMD_ACT	= 3'b0011;	// Bank ACTIVATE
localparam lp_CMD_WR	= 3'b0100;	// & A10 LOW: normal WRITE (assuming BL8MRS)
									// & A10 HIGH: WRITE with auto precharge
localparam lp_CMD_RD	= 3'b0101;	// & A10 LOW: normal READ
									// & A10 HIGH: READ with auto precharge
localparam lp_CMD_NOP	= 3'b0111;	// NO OPERATION
localparam lp_CMD_ZQCL	= 3'b0110;	// ZQ CALIBRATION LONG

// command bus {nRAS, nCAS, nWE}
reg	[2:0]	r3_cmd	= lp_CMD_NOP; // slow clock
reg	r_ddr_nras	= 1'b1; // last three tied to out pins (hw)
reg	r_ddr_ncas	= 1'b1;
reg	r_ddr_nwe	= 1'b1;

reg r_ddr_ncs	= 1'b0; // toggled by fast clock
reg	r_ddr_ncs_pipe = 1'b0;
reg	r_ddr_ncs_fast = 1'b0;
reg r_ddr_ncs_fast_xor = 1'b0;



reg [lp_INIT_CTR_WIDTH-1:0]	rn_init_ctr = lpdiv_NRST_LO;

localparam lp_REF_CTR_WIDTH = $clog2(lpdiv_RFC_MAX - lpdiv_INIT_CTR_PLAY);
reg	[lp_REF_CTR_WIDTH-1:0] rn_refresh_ctr = lpdiv_RFC_MAX-lpdiv_INIT_CTR_PLAY;
reg	r_ddr_nrst = 1'b0;
reg	r_ddr_cke = 1'b0;

reg	[2:0]	r3_ddr_bank		= 3'b0;
reg	[13:0]	r14_ddr_addr	= 14'b0;

reg	r_refresh_flag = 1'b0;

reg	[6:0]	r7_write_words = 7'd0;
reg	[127:0]	r128_write_data = {128{1'b1}};
reg	[127:0]	r128_wrdata_buf = {128{1'b1}};

reg	[255:0]	r256_test = {256{1'b1}};//'h8080_8080_8080_8080_8080_8080_8080_8080;

always @(posedge i_clk_div) begin: slow_logic
	// Initialization counter decrement
	if (rn_init_ctr > 0)
		rn_init_ctr <= rn_init_ctr - 1;

	if (rn_refresh_ctr > 0) begin // reset to p_RFC_MAX (- lp_INIT_CTR_PLAY)
		rn_refresh_ctr <= rn_refresh_ctr - 1;
		r_refresh_flag <= 1'b0;
	end else
		r_refresh_flag <= 1'b1;
		
	// when no cmd
	r3_cmd <= lp_CMD_NOP;
	
	case (rn_state_curr)
// #################### set RESET# pin to HIGH (initialized as 0)
	'd0: begin
		if (i_phy_rst == 1'b0)
		`ifndef SIMULATION
			if (i_test_redo == 1'b0)
		`endif
			if (rn_init_ctr == 0) begin
				r_ddr_nrst <= 1'b1;
				r_ddr_cke <= 1'b0;
				
				r_phy_rst <= 1'b0;
				
				rn_init_ctr <= lpdiv_CKE_LO-1;
				rn_state_curr <= 'd1;
			end
		//else
			//;//rn_init_ctr <= lp_NRST_LO;
			
	end
// #################### set CKE pin to HIGH (initialized as 0)
	'd1: begin
		if (rn_init_ctr == 0) begin
			r_ddr_cke <= 1'b1;
			
			rn_init_ctr <= lpdiv_XPR-1;
			rn_state_curr <= 'd2;
		end
	end
// #################### MRS MR2 -> MR3 -> MR1 -> MR0
	'd2: begin
		if (rn_init_ctr == 0) begin
			r3_cmd <= lp_CMD_MRS;
			r3_ddr_bank <= 3'h2;
			r14_ddr_addr <= DLL.lp_MR2;
			
			rn_init_ctr <= lpdiv_MRD-1;
			rn_state_curr <= 'd3;
		end
	end
	'd3: begin
		if (rn_init_ctr == 0) begin
			r3_cmd <= lp_CMD_MRS;
			r3_ddr_bank <= 3'h3;
			r14_ddr_addr <= DLL.lp_MR3;
			
			rn_init_ctr <= lpdiv_MRD-1;
			rn_state_curr <= 'd4;
		end
	end
	'd4: begin
		if (rn_init_ctr == 0) begin
			r3_cmd <= lp_CMD_MRS;
			r3_ddr_bank <= 3'h1;
			r14_ddr_addr <= DLL.lp_MR1;
			
			rn_init_ctr <= lpdiv_MRD-1;
			rn_state_curr <= 'd5;
		end
	end
	'd5: begin
		if (rn_init_ctr == 0) begin
			r3_cmd <= lp_CMD_MRS;
			r3_ddr_bank <= 3'h0;
			r14_ddr_addr <= DLL.lp_MR0;
			
			rn_init_ctr <= lpdiv_MOD-1;
			rn_state_curr <= 'd6;
		end else
			r3_cmd <= lp_CMD_NOP;
	end
// #################### ZQ init calibration
	'd6: begin
		if (rn_init_ctr == 0) begin
			r3_cmd <= lp_CMD_ZQCL;
			r14_ddr_addr[10] <= 1'b1;	
					
			rn_init_ctr <= lpdiv_ZQINIT-1;
			rn_state_curr <= 'd7;//-'d15;
		end else
			r3_cmd <= lp_CMD_NOP;
	end
// #################### Begin read calibration
	'd7: begin
		if (rn_init_ctr == 0) begin
			r3_cmd <= lp_CMD_ACT;
			r3_ddr_bank <= 3'h0;	// bank select
			r14_ddr_addr <= 14'b0;	// row select
			
			rn_init_ctr <= lpdiv_RCD-1;
			rn_state_curr <= 'd8; //- maybe goto read?
		end else
			r3_cmd <= lp_CMD_NOP;
	end
	'd8: begin
		// WRITE burst 1: 0000, 1111
		if (rn_init_ctr == (lpdiv_WL_MAX - DLL.lpdiv_WL)) begin
			r3_cmd <= lp_CMD_NOP;
			r7_write_words <= 'd1;
			r128_write_data <= 128'h0000_0000_0000_0000_ffff_ffff_ffff_ffff;//{128{1'b1}};
			
		end // not else: both can be true if CWL = 6
		if (rn_init_ctr == 0) begin
			r3_cmd <= lp_CMD_WR;
			r14_ddr_addr[9:0]	<= 10'b0;	// starting column address
											// column addr [2:0] sets burst order
			r14_ddr_addr[10] <= 1'b0;	// auto precharge (off)
			
			// addr[13:11] is Don't Care

			rn_init_ctr <= lpdiv_CCD-1; //-50// 2 div cycles delay = BL/2 fast cycles
			rn_state_curr <= 'd9;
		end else
			r3_cmd <= lp_CMD_NOP;
	end	
	'd9: begin
		// WRITE burst 2: 0101, 1010
		if (rn_init_ctr == (lpdiv_WL_MAX - DLL.lpdiv_WL)) begin
			r3_cmd <= lp_CMD_NOP;
			r7_write_words <= 'd1;
			r128_write_data <= 128'h0000_ffff_0000_ffff_ffff_0000_ffff_0000;
			
		end // not else: both can be true if CWL = 6
		if (rn_init_ctr == 0) begin
			r3_cmd <= lp_CMD_WR;
			r14_ddr_addr[9:0]	<= 10'h8;	// starting column address
			
			r14_ddr_addr[10] <= 1'b0;	// auto precharge (off)
			
			rn_init_ctr <= DLL.lpdiv_WL+4/2+lpdiv_WTR-1; //-50;
			rn_state_curr <= 'd10; 
		end
	end
	'd10: begin
		r7_write_words <= 'd0;
		if (rn_init_ctr == 0) begin
			r3_cmd <= lp_CMD_RD;
			r14_ddr_addr[9:0]	<= 10'h0;	// starting column address
			
			r14_ddr_addr[10] <= 1'b0;	// auto precharge (off)
			
			rn_init_ctr <= lpdiv_CCD-1;
			rn_state_curr <= 'd11;
			
			r_calib_done <= 1'b1;/////////TODO
		end else
			r3_cmd <= lp_CMD_NOP;
	end
	'd11: begin
		if (rn_init_ctr == 0) begin
			r3_cmd <= lp_CMD_RD;
			r14_ddr_addr[9:0]	<= 10'h8;	// starting column address
			
			r14_ddr_addr[10] <= 1'b0;	// auto precharge (off)
			
			//rn_init_ctr <= (lp_WL+4+p_WTR+1)/2-1;
			rn_init_ctr <= 7;
			rn_state_curr <= 'd12;
		end else
			r3_cmd <= lp_CMD_NOP;
	end
	'd12: begin
		r3_cmd <= lp_CMD_NOP;
		if(rn_init_ctr == 4)
			r256_test[255:192] <= w64_iserdes_par;
		if(rn_init_ctr == 3)
			r256_test[191:128] <= w64_iserdes_par;
		if(rn_init_ctr == 2)
			r256_test[127:64] <= w64_iserdes_par;
		if(rn_init_ctr == 1)
			r256_test[63:0] <= w64_iserdes_par;
		if(rn_init_ctr == 0)
			rn_state_curr <= 'd13;
	end
	'd13: begin
		r3_cmd <= lp_CMD_NOP;
		r_ddr_nrst <= 1'b0;

		if (i_test_redo) begin
			rn_state_curr <= 'd0;
			rn_init_ctr <= lpdiv_NRST_LO-1;
			r_calib_done <= 1'b0;
		end else begin
			r_calib_done <= 1'b1;
			rn_init_ctr <= 'b0;
		end
	end
	'd15: begin // MPR special
		if (rn_init_ctr == 0) begin
			r3_cmd <= lp_CMD_MRS;
			r3_ddr_bank <= 3'h3;
			r14_ddr_addr <= lp_MR3_MPR;
			
			rn_init_ctr <= 50;
			rn_state_curr <= 'd10;
		end else
			r3_cmd <= lp_CMD_NOP;
	
	end
	default: ;
	endcase
end
assign o256_test = r256_test;
// 6:0 width assumes maximum burst of 128 words (all columns)
// does not violate REFRESH period: 128×BL/2 = 128×4CK < 7800 ns
// Also assumes there will be a 128 word write buffer.
// (There won't. TODO: Decide buffer size.)
reg	[6:0]	r7_oserdes_words = 7'd0;
reg	[2:0]	r3_dqs_state = 3'd0;
always @(posedge i_clk_div) begin: oserdes_ctrl
	r128_wrdata_buf <= r128_write_data;
	
	case (r3_dqs_state)
	'd0: begin
		if (r7_write_words > 0) begin
			//r7_oserdes_words <= r7_write_words - 1;
			
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
		r64_oserdes_dq_par <= r128_wrdata_buf/*r128_write_data*/[127:64];
		r4_tristate_dq <= 'h3;
				
		r3_dqs_state <= 'd4;
	end
	// burst write continuation:
	'd7: begin
		// dqs oserdes normal toggle 1/2 (continued from last word)
		r4_oserdes_dqs_par <= 4'h5;
		r4_tristate_dqs <= 'h0;
		
		// dq oserdes
		r64_oserdes_dq_par <= r128_wrdata_buf/*r128_write_data*/[127:64];
		r4_tristate_dq <= 'h0;
		
		r3_dqs_state <= 'd4;
	end
	'd4: begin
		// dqs normal toggle 2/2
		r4_oserdes_dqs_par <= 4'h5;
		r4_tristate_dqs <= 'h0;
		
		r64_oserdes_dq_par <= r128_wrdata_buf/*r128_write_data*/[63:0];
		r4_tristate_dq <= 'h0;
		
		//r4_dq_tmr <= 'd2;
		// next div cycle?
		if (r7_write_words > 0) begin
			//r7_oserdes_words <= r7_oserdes_words - 1;
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

always @(posedge i_clk_div) begin: iserdes_ctrl
	if(r3_cmd == lp_CMD_RD) begin
		r_iserdes_en <= 1'b1;
	end
end

assign o_calib_done = r_calib_done;

// Hardware out assigns
assign o_ddr_nrst	= r_ddr_nrst;

assign o_ddr_ncs	= (DLL.lp_CWL % 2) ? ~i_clk_div : i_clk_div;//r_ddr_ncs_fast;
assign o_ddr_nras 	= r3_cmd[2];
assign o_ddr_ncas 	= r3_cmd[1];
assign o_ddr_nwe 	= r3_cmd[0];

assign o2_ddr_dm	= 2'b00;

assign o_ddr_cke	= r_ddr_cke;

assign o_ddr_odt	= 1'b0;

assign o3_ddr_bank	= r3_ddr_bank;
assign o14_ddr_addr	= r14_ddr_addr;

assign o4_test_state = rn_state_curr;
assign o64_iserdes = w64_iserdes_par;

assign o13_init_ctr = rn_init_ctr;

endmodule
