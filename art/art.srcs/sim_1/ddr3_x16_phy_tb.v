`timescale 1ns / 1ps

module ddr3_x16_phy_tb;

/////////////////////////////////////////////////
// CLK GEN
/////////////////////////////////////////////////
localparam REFCLK_FREQUENCY	= 200.0;
localparam ddr_ck_period	= 10;	// 100 MHz
localparam clk_ref_period	= 5;	// 200 MHz

	reg clk_ddr_100;
	reg clk_ddr_100_90deg;
	reg clk_ui_25;
	reg clk_ref;
	
reg phy_rst = 1;
	

initial begin: phy_clk_gen // 100 MHz
	clk_ddr_100 = 0;
	forever begin
		#(ddr_ck_period/2) // delay half period
		clk_ddr_100 = ~clk_ddr_100;
	end
end
initial begin: phy_clk_90_gen // 100 MHz, phase shift
	clk_ddr_100_90deg = 0;
	#(2.5)
	forever begin
		#(ddr_ck_period/2) // delay half period
		clk_ddr_100_90deg = ~clk_ddr_100_90deg;
	end
end
initial begin: ui_clk_gen // 25 MHz
	#25
	clk_ui_25 = 0;
	forever begin
		#20 // delay half period
		clk_ui_25 = ~clk_ui_25;
	end
end
initial begin: clk_ref_gen // 200 MHz
	clk_ref = 0;
	forever begin
		#(clk_ref_period/2) // delay half period
		clk_ref = ~clk_ref;
	end
end

reg dq_oserdes_en = 0;
reg	r_dqs_in;
wire	[1:0]	w2_dqs_in;
assign w2_dqs_in[0] = r_dqs_in;
assign w2_dqs_in[1] = r_dqs_in;

reg	r_dqs_out_nen;
wire	[1:0]	w2_dqs_out_nen;
assign w2_dqs_out_nen[0] = r_dqs_out_nen;
assign w2_dqs_out_nen[1] = r_dqs_out_nen;

wire	[15:0]	w16_io_dq;
assign w16_io_dq = {(16){1'bZ}};

wire	[1:0]	w2_io_dqs_p;
assign w2_io_dqs_p = 2'bZZ;

wire	[1:0]	w2_io_dqs_n;
assign w2_io_dqs_n = 2'bZZ;

ddr3_x16_phy #(
	.REFCLK_FREQUENCY(200.0)
) phy_inst (
	.i_clk_ui(clk_ui_25),	// 4:1 ratio between ui and ddr clocks
	.i_clk_ddr(clk_ddr_100),
	.i_clk_ddr_90(clk_ddr_100_90deg),
	.i_clk_ref(clk_ref),	// 200 or 300 MHz, see REFCLK_FREQUENCY, used for IDELAYCTRL
	
	.i_phy_rst(phy_rst),	// active high reset for OSERDES, IDELAYCTRL
	
	.i128_phy_wrdata(128'h0000_1111_2222_3333_4444_5555_6666_7777),
	
	.i2_phy_dqs(w2_dqs_in),
	.i2_dqs_iobuf_out_nen(w2_dqs_out_nen),
	.i_dq_oserdes_en(dq_oserdes_en),


	.io16_ddr_dq(w16_io_dq),
	.io2_ddr_dqs_p(w2_io_dqs_p),
	.io2_ddr_dqs_n(w2_io_dqs_n),
	
	.o_ddr_ck_p(),
	.o_ddr_ck_n(),
	
	.o14_ddr_addr()
);

initial begin: test
	#5
	r_dqs_in = 0;
	r_dqs_out_nen = 1;
	dq_oserdes_en = 0;
	#40
	phy_rst = 0;
	//r_dqs_in = 1;
	#40
	r_dqs_out_nen = 0;
	#40
	r_dqs_in = 1;
	dq_oserdes_en = 1;
	#40
	dq_oserdes_en = 0;
	r_dqs_in = 0;
	r_dqs_out_nen = 1;
	#50
	$stop;
end // test
wire w_oserdes_out;
reg oserdes_oce = 1'b0;
OSERDESE2 #(
		.DATA_RATE_OQ("DDR"), // DDR, SDR
		.DATA_RATE_TQ("BUF"), // DDR, BUF, SDR
		.DATA_WIDTH(8), // Parallel data width (2-8,10,14)
		.TRISTATE_WIDTH(1), // 3-state converter width (1,4)
		.SERDES_MODE("MASTER")
	) oserdes_dq_inst (
		.OFB(), // 1-bit output: Feedback path for data
		.OQ(w_oserdes_out), // 1-bit output: Data path output
		// SHIFTOUT1 / SHIFTOUT2: 1-bit (each) output: Data output expansion (1-bit each)
		.SHIFTOUT1(),
		.SHIFTOUT2(),
		.TBYTEOUT(), // 1-bit output: Byte group tristate
		.TFB(), // 1-bit output: 3-state control
		.TQ(), // 1-bit output: 3-state control
		.CLK(clk_ddr_100), // 1-bit input: High speed clock
		.CLKDIV(clk_ui_25), // 1-bit input: Divided clock
		// D1 - D8: 1-bit (each) input: Parallel data inputs (1-bit each)
		.D1(1'b1),
		.D2(1'b0),
		.D3(1'b0),
		.D4(1'b1),
		.D5(1'b0),
		.D6(1'b0),
		.D7(1'b1),
		.D8(1'b0),
		.OCE(oserdes_oce), // 1-bit input: Output data clock enable
		.RST(phy_rst), // 1-bit input: Reset
		// SHIFTIN1 / SHIFTIN2: 1-bit (each) input: Data input expansion (1-bit each)
		.SHIFTIN1(1'b0),
		.SHIFTIN2(1'b0),
		// T1 - T4: 1-bit (each) input: Parallel 3-state inputs
		.T1(1'b1),//w2_dqs_iobuf_out_nen[i/8]),
		.T2(),//w2_dqs_iobuf_out_nen[i/8]),
		.T3(),//w2_dqs_iobuf_out_nen[i/8]),
		.T4(),//w2_dqs_iobuf_out_nen[i/8]),
		.TBYTEIN(1'b0), // 1-bit input: Byte group tristate
		.TCE(1'b1) // 1-bit input: 3-state clock enable
	);

wire selectio_out;	
wire selectio_clk_out;
selectio_wiz_0
   // width of the data for the system
 #( .SYS_W(1),
   // width of the data for the device
   .DEV_W (8))
ass (
  // From the device out to the system
	.data_out_from_device(8'b1001_0010),
	.data_out_to_pins(selectio_out),
	.clk_in(clk_ddr_100),        // Single ended clock from IOB
	.clk_div_out(selectio_clk_out),   // Slow clock output
	.clk_reset(0),
	.io_reset(phy_rst));
endmodule
