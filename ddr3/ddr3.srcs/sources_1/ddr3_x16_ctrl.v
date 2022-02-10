`timescale 1ns / 1ps

// Macro "function" returns bigger of two arguments
`define max2(v1, v2) ((v1) > (v2) ? (v1) : (v2))

module ddr3_x16_ctrl #(
	parameter	p_DLL 			= "OFF",
	parameter	p_DDR_FREQ_MHZ	= 100,
	
	//parameter	p_DDR_DATA_RATE	= 1333.0,	// Data Rate
	parameter	p_DDR_CK		= 10.0,	// Cycle time (ns)
			// Timing params in nCK	 // (ns)	// Description
	parameter	p_CKE			= 3, // max(3CK or (5.0))	// CKE minimum pulse width
	parameter	p_FAW			= 5, // (45.0)	// Four Address Width
	parameter	p_RAS			= 4, // (36.0)	// Active to Precharge command
	parameter	p_RCD			= 2, // (13.5)	// Active to Read or Write delay
	parameter	p_REFI			= 780, // (7800.0)	// Average periodic refresh interval
	parameter	p_RFC_MIN		= 16, // (160.0)	// Refresh to Active or Refresh to Refresh
	parameter	p_RFC_MAX		= 702, // (7020.0)	
	parameter	p_RP			= 2, // (13.5)	// Precharge command period
	parameter	p_RRD			= 1, // (7.5)	// ACTIVATE to ACTIVATE in different banks
	parameter	p_RTP			= 1, // (7.5)	// READ to PRECHARGE
	parameter	p_WTR			= 1, // (7.5)	// WRITE to READ
	parameter	p_WR			= 5, // (15.0)	// WRITE recovery time
	parameter	p_XPR			= 17, // max(5.0, tRFC + 10)	// Exit reset from CKE HIGH to valid command
	parameter	p_MRD			= 4,	// MRS cycle time
	parameter	p_MOD			= 12, // max(12CK, 15.0)	// MRS update delay
	parameter	p_ZQINIT		= 512	// Long calibration time
)(
	input	i_clk_ddr,
	input	i_clk_ddr_270,
	input	i_clk_200,
	
	input	[127:0]	i128_ctrl_wrdata,
	
	output	[63:0]	o64_ctrl_rddata,
	
	output	o_clk_ddr,
	

	// CONNECTION TO DRAM pass-through to PHY (DQ, DQS, CK)
	output	o_ddr_ck_p,
	output	o_ddr_ck_n,	

	inout	[15:0]	io16_ddr_dq,
	inout	[1:0]	io2_ddr_dqs_p,
	inout	[1:0]	io2_ddr_dqs_n,
	// CONNECTION TO DRAM used by CTRL (ADDR, BANK, CS/RAS/CAS/WE, ODT, CKE, UDM/LDM)
	output	[1:0]	o2_ddr_dm,

	output	[13:0]	o14_ddr_addr,
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
// Configuration and parametrization
/////////////////////////////////////////////////
// DLL OFF/ON changes
//if (p_DLL == "OFF") begin
	localparam lp_CL = 6;
	localparam lp_CWL = 6;
	localparam lp_AL = 0;	
	// MRx is {BA[3:0], A[13:0]}
					      /* 13 12 11 10 ~9 ~8 ~7 ~6 ~5 ~4 ~3 ~2 ~1 ~0 */
	localparam lp_MR0 = 14'b__0__0__0__0__1__1__0__0__1__0__0__0__0__0;
	localparam lp_MR1 = 14'b__0__0__0__0__0__0__0__0__0__0__0__0__0__0;
	localparam lp_MR2 = 14'b__0__0__0__0__0__0__0__0__0__0__1__0__0__0;
	localparam lp_MR3 = 14'b__0__0__0__0__0__0__0__0__0__0__0__0__0__0;
	// Start of write recovery is on rising clk edge four cycles after WL
//end else begin
//	localparam lp_tWR = 15;
//end

/////////////////////////////////////////////////
// State machine state definitions
/////////////////////////////////////////////////
reg	[3:0]	rn_state_curr = lp_STATE_INIT;
localparam lp_STATE_INIT = 4'b0000;


/////////////////////////////////////////////////
// Command defines; {nCS, nRAS, nCAS, nWE}; See Micron datasheet Table 87
/////////////////////////////////////////////////
localparam lp_CMD_MRS	= 4'b0000;	// MODE REGISTER SET
localparam lp_CMD_REF	= 4'b0001;	// REFRESH
localparam lp_CMD_PRE	= 4'b0010;	// & A10 LOW: Single-bank PRECHARGE
									// & A10 HIGH: PRECHARGE all banks
localparam lp_CMD_ACT	= 4'b0011;	// Bank ACTIVATE
localparam lp_CMD_WR	= 4'b0100;	// & A10 LOW: normal WRITE (assuming BL8MRS)
									// & A10 HIGH: WRITE with auto precharge
localparam lp_CMD_RD	= 4'b0101;	// & A10 LOW: normal READ
									// & A10 HIGH: READ with auto precharge
localparam lp_CMD_NOP	= 4'b0111;	// NO OPERATION
localparam lp_CMD_ZQCL	= 4'b0110;	// ZQ CALIBRATION LONG

/////////////////////////////////////////////////
// Hardware pin controls (non-PHY)(Fig. 46)
/////////////////////////////////////////////////
reg	[3:0]	r4_ddr_cmd = lp_CMD_NOP;
//RESET# and CKE initialization
localparam lp_NRST_LO = 250 * p_DDR_FREQ_MHZ; // RESET#: After power stable, RESET# held LOW for >200 us.
localparam lp_CKE_LO = 501 * p_DDR_FREQ_MHZ; // CKE: After RESET# transitions HIGH wait >500 us with CKE LOW.
localparam lp_INIT_CTR_PLAY = 50;
localparam lp_INIT_CTR_START = lp_NRST_LO + lp_CKE_LO + p_XPR + 3 * p_MRD + p_MOD + p_ZQINIT + lp_INIT_CTR_PLAY;

localparam lp_INIT_CTR_WIDTH = $clog2(lp_INIT_CTR_START);

reg [lp_INIT_CTR_WIDTH-1:0]	rn_init_ctr = lp_INIT_CTR_START;
reg	r_ddr_nrst = 1'b0;
reg	r_ddr_cke = 1'b0;

// ODT: Must be valid tIS prior to CKE going HIGH. Recommended LOW.
reg	r_ddr_odt = 1'b0;

reg	[2:0]	r3_ddr_bank;
reg	[13:0]	r14_ddr_addr;
reg	[63:0]	r64_ctrl_rddata;
assign o64_ctrl_rddata = r64_ctrl_rddata;

reg r_refresh_flag = 1'b0;

always @(posedge i_clk_ddr) begin: state_machine
	// Initialization counter decrement
	if (rn_init_ctr > 0)
		rn_init_ctr <= rn_init_ctr - 1;
	else if (rn_init_ctr == 0) begin
		rn_init_ctr <= p_RFC_MAX-20; // reuse init ctr to keep refresh timing
		r_refresh_flag <= 1'b1;	// refresh-is-needed-shortly flag
	end
	
	case (rn_state_curr)
	lp_STATE_INIT: begin
		// RESET# pin goes HIGH
		if (rn_init_ctr < (lp_INIT_CTR_START - lp_NRST_LO))
			r_ddr_nrst <= 1'b1;
		// CKE pin goes HIGH (initialized as 0)
		if (rn_init_ctr < lp_INIT_CTR_START - lp_NRST_LO - lp_CKE_LO)
			r_ddr_cke <= 1'b1;
		// MRS MR2 -> MR3 -> MR1 -> MR0
		if (rn_init_ctr == lp_INIT_CTR_START - lp_NRST_LO - lp_CKE_LO - p_XPR) begin
			r4_ddr_cmd <= lp_CMD_MRS;
			r3_ddr_bank <= 3'h2;
			r14_ddr_addr <= lp_MR2;
		end
		if (rn_init_ctr == 2 * p_MRD + p_MOD + p_ZQINIT + lp_INIT_CTR_PLAY) begin
			r4_ddr_cmd <= lp_CMD_MRS;
			r3_ddr_bank <= 3'h3;
			r14_ddr_addr <= lp_MR3;
		end
		if (rn_init_ctr == p_MRD + p_MOD + p_ZQINIT + lp_INIT_CTR_PLAY) begin
			r4_ddr_cmd <= lp_CMD_MRS;
			r3_ddr_bank <= 3'h1;
			r14_ddr_addr <= lp_MR1;
		end
		if (rn_init_ctr == p_MOD + p_ZQINIT + lp_INIT_CTR_PLAY) begin
			r4_ddr_cmd <= lp_CMD_MRS;
			r3_ddr_bank <= 3'h0;
			r14_ddr_addr <= lp_MR0;
		end
		// ZQ init calibration
		if (rn_init_ctr == p_ZQINIT + lp_INIT_CTR_PLAY) begin
			r4_ddr_cmd <= lp_CMD_ZQCL;
			r14_ddr_addr[10] <= 1'b1;
		end
		if (rn_init_ctr == lp_INIT_CTR_PLAY) begin
			r4_ddr_cmd <= lp_CMD_ACT;
			r3_ddr_bank <= 3'h0;
			r14_ddr_addr <= 14'b0;
		end
		if (rn_init_ctr == lp_INIT_CTR_PLAY - 1)
			r4_ddr_cmd <= lp_CMD_NOP;
		if (rn_init_ctr == lp_INIT_CTR_PLAY - p_RCD) begin
			r4_ddr_cmd <= lp_CMD_WR;
			r14_ddr_addr[10] <= 1'b0; // auto precharge OFF
			r14_ddr_addr[11] <= 1'b0; // column addr
			r14_ddr_addr[9:0] <= 10'b0;
		end
		if (rn_init_ctr == lp_INIT_CTR_PLAY - p_RCD - 1) begin
			r4_ddr_cmd <= lp_CMD_NOP; // NO OP until next valid cmd
		end
		if (rn_init_ctr == lp_INIT_CTR_PLAY - p_RCD - lp_CWL + 5) begin
			r128_phy_wrdata <= i128_ctrl_wrdata;
		end
		if (rn_init_ctr == lp_INIT_CTR_PLAY - p_RCD - lp_CWL + 1) begin
			r_phy_tristate <= 1'b1;
			r2_phy_dqs_d <= 2'b01;
		end
		if (rn_init_ctr == lp_INIT_CTR_PLAY - p_RCD - lp_CWL - 4) begin
			r2_phy_dqs_d <= 2'b00;
			r_phy_tristate <= 1'b0;
		end
		// READ tWTR cycles after r2_phy_dqs_d<=2'b00
		if (rn_init_ctr == lp_INIT_CTR_PLAY - p_RCD - lp_CWL - 4 - p_WTR) begin
			r4_ddr_cmd <= lp_CMD_RD;
			r14_ddr_addr[10] <= 1'b1; // auto precharge ON
			r14_ddr_addr[11] <= 1'b0; // column addr
			r14_ddr_addr[9:0] <= 10'b0;
		end
		if (rn_init_ctr == lp_INIT_CTR_PLAY - p_RCD - lp_CWL - 4 - p_WTR - 1)
			r4_ddr_cmd <= lp_CMD_NOP;
		if (rn_init_ctr == lp_INIT_CTR_PLAY - p_RCD - lp_CWL - 4 - p_WTR - lp_CL - 4) begin
			r64_ctrl_rddata <= w64_rddata;
			rn_state_curr <= 4'b1;
			r_ddr_nrst <= 1'b0;
		end
	end
	default: ;
	endcase
end

/////////////////////////////////////////////////
// PHY and connections
/////////////////////////////////////////////////
reg	[1:0]	r2_phy_dqs_d = 1'b0;	// PHY expects flow 'b00 -> 'b01 -> 'b11
wire	w_phy_dq_burst_enable = (r2_phy_dqs_d[0] ^ r2_phy_dqs_d[1]);	// OSERDES only active while DQS is toggling
reg	r_phy_rst = 1'b1;
reg	r_phy_tristate = 1'b1;

reg	[127:0]	r128_phy_wrdata = 128'b0;
wire	[63:0]	w64_phy_rddata;

ddr3_x16_phy phy_instance (
	.i_clk_ddr(i_clk_ddr),	// chip clock frequency
	.i_clk_ddr_270(i_clk_ddr_270),	// same but delayed by 270°, used to generate output DQ from OSERDES
	.i_clk_ref(i_clk_200),	// 200 MHz, used for IDELAYCTRL, which controls taps for input DQS IDELAY
	
	.i_phy_rst(r_phy_rst),	// active high reset for ODDR, OSERDES, ISERDES, IDELAYCTRL, hold HIGH until all clocks are generated// active high reset for OSERDES, IDELAYCTRL
	.i_phy_tristate_en(r_phy_tristate),	// DQ/DQS IOBUF/IOBUFDS tristate (1 = Z, 0 = output enabled)
	
	.i128_phy_wrdata(r128_phy_wrdata),	// eight words of write data for OSERDES (out of 8 for a total of BL8)
						// MUST be available exactly 5 i_clk_ddr before data is needed on DQ bus.
	.o64_phy_rddata(w64_rddata),	// four words of read data from ISERDES (out of 8 for a total of BL8)
	
	.i_phy_dq_oserdes_en(w_phy_dq_burst_enable),	// DQ OSERDES enable
	
	.i_phy_dqs_oddr_d1en(r2_phy_dqs_d[0]),	// DQS ODDR D1 input
	.i_phy_dqs_oddr_d2en(r2_phy_dqs_d[1]),	// DQS ODDR D2 input
			// NOTE about dq_oserdes_en and dqs_oddr_dXen:
			// Keep in mind necessary DQS preamble when raising these inputs.
			// WR op requires a dummy crossover DQ cycle (Micron datasheet Figure 82) as preamble
			// WR op requires a dummy extra positive DQ cross as postamble

	// CONNECTION TO DRAM
	.io16_ddr_dq(io16_ddr_dq),
	.io2_ddr_dqs_p(io2_ddr_dqs_p),
	.io2_ddr_dqs_n(io2_ddr_dqs_n),
	
	.o_ddr_ck_p(o_ddr_ck_p),
	.o_ddr_ck_n(o_ddr_ck_n)
);

assign o_ddr_nrst	= r_ddr_nrst;
assign o_ddr_cke 	= r_ddr_cke;
assign o_ddr_odt 	= r_ddr_odt;
// r4_cmd <= {nCS, nRAS, nCAS, nWE}
assign o_ddr_ncs	= r4_ddr_cmd[3];
assign o_ddr_nras 	= r4_ddr_cmd[2];
assign o_ddr_ncas 	= r4_ddr_cmd[1];
assign o_ddr_nwe 	= r4_ddr_cmd[0];

assign o3_ddr_bank	= r3_ddr_bank;
assign o14_ddr_addr	= r14_ddr_addr;

assign o2_ddr_dm	= 'b00;

ila_ddr_cust ila_inst_ddr3 (
.clk(i_clk_ddr),
/*input [63 : 0]*/ .probe0(r64_ctrl_rddata),
/*input [127 : 0]*/ .probe1(r128_phy_wrdata),
/*input [0 : 0]*/ .probe2(r_ddr_nrst),
/*input [3 : 0]*/ .probe3(r4_ddr_cmd),
/*input [0 : 0]*/ .probe4(r_ddr_cke)
);

endmodule
