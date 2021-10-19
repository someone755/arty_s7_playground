`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/19/2021 12:11:09 PM
// Design Name: 
// Module Name: debouncer_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module debouncer_tb;

/** UCLK */
	reg UCLK_tmp = 0;
	wire UCLK;
	assign UCLK = UCLK_tmp;

/** BTN */
	reg	[3:0] BTN_tmp = 0;
	wire [3:0] BTN;
	assign BTN = BTN_tmp;
/** LED */	
	wire [3:0] LED;

localparam period = 5; // 5*timescale
	
debouncer #(
	.p_DEBNC_CLOCKS(100),
	.p_PORT_WIDTH(4) 
)
dbnc_instance_uut (
	.CLK_I(UCLK),
	.SIGNAL_I(BTN),
	.SIGNAL_O(LED)
);

initial begin: clk_gen
	UCLK_tmp = 0;
	forever begin
		#(period/2) // delay half period
		UCLK_tmp = ~UCLK_tmp;
	end
end // clk_gen


initial begin
	BTN_tmp = 'b0000;
	#(period*120);
	BTN_tmp = 'b1111;
	#(period*50);
	BTN_tmp = 'b0000;
	#(period*120);
	BTN_tmp = 'b1111;
	#(period*120);
	
	$stop;
end
endmodule
