`timescale 1ns / 1ps

module bram
	#(
	parameter p_RAM_WIDTH = 8,	// word size
	parameter p_RAM_DEPTH = 32	// number of addresses
	)
	(
	input i_clk,
	input [$clog2(p_RAM_DEPTH)-1:0] in_addr,
	input [p_RAM_WIDTH-1:0] in_datain,
	input i_wren,
	
	output [p_RAM_WIDTH-1:0] on_dataout
	);

(* ram_style = "block" *) reg [p_RAM_WIDTH-1:0] rn_ram [p_RAM_DEPTH-1:0];

reg [p_RAM_WIDTH-1:0] rn_dataout;
assign on_dataout = rn_dataout;

// Xilinx claims a register will not be absorbed into BRAM when its initial value
//	is not declared as 0. The below initial block is supposedly synthesizable, but
//	unnecessary; Vivado's synthesis reports rn_ram is absorbed properly into BRAM
// and automatically initialized to 0. 
// https://support.xilinx.com/s/article/64049?language=en_US
//integer i = 0;
//initial begin: init_to_zero
//	for (i=0; i<p_RAM_DEPTH; i=i+1)
//		rn_ram[i] = {p_RAM_WIDTH{1'b0}};
//end // init_to_zero

always @(posedge i_clk) begin: bram_proc
	if (i_wren) begin
		rn_ram[in_addr] <= in_datain;
		rn_dataout <= in_datain;
	end else
		rn_dataout <= rn_ram[in_addr];
end // bram_proc

endmodule // bram
