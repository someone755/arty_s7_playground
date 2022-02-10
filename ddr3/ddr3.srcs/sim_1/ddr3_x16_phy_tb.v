`timescale 1ns / 1ps

module ddr3_x16_phy_tb;

/////////////////////////////////////////////////
// CLK GEN
/////////////////////////////////////////////////
localparam REFCLK_FREQUENCY	= 200.0;
localparam ddr_ck_period	= 10;	// 100 MHz
localparam clk_ref_period	= 5;	// 200 MHz

	reg clk_ddr;
	reg clk_ddr_270deg;
	reg clk_ref;
	
reg phy_rst = 1; 

initial begin: phy_clk_gen // 100 MHz
	clk_ddr = 0; 
	forever begin
		#(ddr_ck_period/2) // delay half period
		clk_ddr = ~clk_ddr;
	end
end
initial begin: phy_clk_270_gen // 100 MHz, phase shift
	clk_ddr_270deg = 0;
	#(7.5)
	forever begin
		#(ddr_ck_period/2) // delay half period
		clk_ddr_270deg = ~clk_ddr_270deg;
	end
end
initial begin: clk_ref_gen // 200 MHz
	#3
	clk_ref = 0;
	forever begin
		#(clk_ref_period/2) // delay half period
		clk_ref = ~clk_ref;
	end
end

reg	r_dq_oserdes_en = 0;
reg	r_dqs_d1en;
reg	r_dqs_d2en;

reg	r_phy_tristate;
wire	[1:0]	w2_dqs_out_nen;
assign w2_dqs_out_nen[0] = r_phy_tristate;
assign w2_dqs_out_nen[1] = r_phy_tristate;

reg		[127:0]	dq_wr_data_phyin = 128'b0;
wire	[63:0]	dq_rd_data_phyout;
reg		[15:0]	dq_rd_data_tb_gen = 16'b0;
wire	[15:0]	w16_io_dq = (r_phy_tristate) ? dq_rd_data_tb_gen : {(16){1'bZ}};

reg r_dqs_rdstrobe_en = 0;

wire	r_dqs_read = (r_dqs_rdstrobe_en) ? clk_ddr : 0;
wire	dqs_p_io = (r_phy_tristate) ? r_dqs_read : 1'bZ;
wire	[1:0]	w2_io_dqs_p;
assign w2_io_dqs_p = {dqs_p_io, dqs_p_io};

wire	dqs_n_io = (r_phy_tristate) ? ~r_dqs_read : 1'bZ;
wire	[1:0]	w2_io_dqs_n;
assign w2_io_dqs_n = {dqs_n_io, dqs_n_io};

ddr3_x16_phy #(
	.REFCLK_FREQUENCY(200.0)
) phy_inst (
	.i_clk_ddr(clk_ddr),
	.i_clk_ddr_270(clk_ddr_270deg),
	.i_clk_ref(clk_ref),	// 200 or 300 MHz, see REFCLK_FREQUENCY, used for IDELAYCTRL
	
	.i_phy_rst(phy_rst),	// active high reset for OSERDES, IDELAYCTRL
	.i_phy_tristate_en(r_phy_tristate),
	
	.i128_phy_wrdata(dq_wr_data_phyin),
	.o64_phy_rddata(dq_rd_data_phyout),
	
	.i_phy_dq_oserdes_en(r_dq_oserdes_en),
	
	.i_phy_dqs_oddr_d1en(r_dqs_d1en),
	.i_phy_dqs_oddr_d2en(r_dqs_d2en),

	.io16_ddr_dq(w16_io_dq),
	.io2_ddr_dqs_p(w2_io_dqs_p),
	.io2_ddr_dqs_n(w2_io_dqs_n),
	
	.o_ddr_ck_p(),
	.o_ddr_ck_n()
);

initial begin: test
	#5
	r_dqs_d1en = 0;
	r_dqs_d2en = 0;
	r_phy_tristate = 1;
	r_dq_oserdes_en = 0;
	#40
	phy_rst = 0;
	//r_dqs_d1en = 1;
	#40
	r_phy_tristate = 0;
	#40
	// See MR2 definition: Since our clock is always tCK > 2.5 ns, CWL MUST be 5 CK.
	// Hence #50 delay to r_dq_oserdes_en comes in handy. (10 ns is 100 MHz "DDR clock" period.)
	// TODO I guess: replace fixed delays with "#ddr_ck_period"
	dq_wr_data_phyin = 'haaaa_bbbb_cccc_dddd_eeee_ffff_1111_2222;
	#40
	r_dqs_d1en = 1;
	r_dqs_d2en = 0;
	r_dq_oserdes_en = 1;
	#0
	dq_wr_data_phyin = 'h1234_2345_3456_4567_5678_6789_7890_890a;
	#90
	r_dq_oserdes_en = 0;
	r_dqs_d1en = 0;
	r_dqs_d2en = 0;
	#10	// TODO: what if read data strobe is not synchronous as in DLL_disable?
	r_phy_tristate = 1;
	#40
	r_dqs_rdstrobe_en = 1;
	dq_rd_data_tb_gen = 16'haaaa;
	#5
	dq_rd_data_tb_gen = 16'hbbbb;
	#5
	dq_rd_data_tb_gen = 16'h3333;
	#5
	dq_rd_data_tb_gen = 16'h4444;
	#5
	dq_rd_data_tb_gen = 16'h5555;
	#5
	dq_rd_data_tb_gen = 16'h6666;
	#5
	dq_rd_data_tb_gen = 16'h7777;
	#5
	dq_rd_data_tb_gen = 16'h8888;
	r_dqs_rdstrobe_en = 0;
	#40
	#40
	$stop;
end // test
endmodule
