`timescale 1ns / 1ps

module bram_tb;
localparam period = 1000; // period*timescale

	/** IN_CLK */
	reg IN_CLK = 0;
	
initial begin: clk_gen
	IN_CLK = 0;
	forever begin
		#(period/2) // delay half period
		IN_CLK = ~IN_CLK;
	end
end // clk_gen

/** Begin BRAM module */
localparam lp_BRAM_WIDTH = 8;
wire [lp_BRAM_WIDTH-1:0] w8_bram_dataout;
reg [lp_BRAM_WIDTH-1:0] r8_bram_datain;

localparam lp_BRAM_DEPTH = 4;
localparam lp_BRAM_ADDR_MAX = $clog2(lp_BRAM_DEPTH)-1;
reg [lp_BRAM_ADDR_MAX:0] rN_bram_addr;

reg r_bram_write_en = 1'b0;

//reg r_bram_write_en = 1'b0;
bram #(
	.p_RAM_WIDTH(lp_BRAM_WIDTH),
	.p_RAM_DEPTH(lp_BRAM_DEPTH)
	)
bram_instance (
	.IN_CLK(IN_CLK),
	.IN_BRAM_ADDR(rN_bram_addr),
	.IN_BRAM_DATAIN(r8_bram_datain),
	.IN_BRAM_WRITE_EN(r_bram_write_en),
	
	.OUT_BRAM_DATAOUT(w8_bram_dataout)
	);
/** End BRAM module */

initial begin: test
	r_bram_write_en = 0;
	rN_bram_addr = 0;
	r8_bram_datain = 8'hff;
	#(period*3-period/2);
	r_bram_write_en = 1;
	#period
	r_bram_write_en = 0;
	#(period*2)
	rN_bram_addr = 1;
	$stop;
end // test
endmodule
