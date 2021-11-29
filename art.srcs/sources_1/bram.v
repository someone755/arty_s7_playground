`timescale 1ns / 1ps

module bram
	#(
	parameter p_RAM_WIDTH = 8,	// word size
	parameter p_RAM_DEPTH = 32	// number of addresses
	)
	(
	input IN_CLK,
	input [$clog2(p_RAM_DEPTH)-1:0] IN_BRAM_ADDR,
	input [p_RAM_WIDTH-1:0] IN_BRAM_DATAIN,
	input IN_BRAM_WRITE_EN,
	
	output [p_RAM_WIDTH-1:0] OUT_BRAM_DATAOUT
	);

(* ram_style = "block" *) reg [p_RAM_WIDTH-1:0] rN_ram [p_RAM_DEPTH-1:0];

reg [p_RAM_WIDTH-1:0] r_OUT_BRAM_DATAOUT;
assign OUT_BRAM_DATAOUT = r_OUT_BRAM_DATAOUT;

// Xilinx claims a register will not be absorbed into BRAM when its initial value
//	is not declared as 0. The below initial block is supposedly synthesizable, but
//	unnecessary; Vivado's synthesis reports rN_ram is absorbed properly into BRAM
// and automatically initialized to 0. 
// https://support.xilinx.com/s/article/64049?language=en_US
//integer i = 0;
//initial begin: init_to_zero
//	for (i=0; i<p_RAM_DEPTH; i=i+1)
//		rN_ram[i] = {p_RAM_WIDTH{1'b0}};
//end // init_to_zero

always @(posedge IN_CLK) begin: bram
	if ( IN_BRAM_WRITE_EN ) begin
		rN_ram[IN_BRAM_ADDR] <= IN_BRAM_DATAIN;
		r_OUT_BRAM_DATAOUT <= IN_BRAM_DATAIN;
	end else
		r_OUT_BRAM_DATAOUT <= rN_ram[IN_BRAM_ADDR];
end // bram

endmodule
