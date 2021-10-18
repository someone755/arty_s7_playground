`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/15/2021 03:37:58 PM
// Design Name: 
// Module Name: top
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

`define	M_12E6	12_000_000	// UCLK freq
`define M_CTR_WDTH		24	// maximum width of counter; 12e6 fits into 24 bit
`define	M_CTR_FREQ		1	// counter frequency

module top

	/*#(
	parameter
	)*/
	
	(
	input	UCLK,		// 12 MHZ oscillator input
	input	[3:0] SW,
	input	[3:0] BTN,
	
	output [3:0] LED,
	output	LED0_R, LED0_G, LED0_B,
	output	LED1_R, LED1_G, LED1_B
	
	);

reg	[3:0]	r4_led;
reg	r_led0_r, r_led0_g, r_led0_b;
reg	r_led1_r, r_led1_g, r_led1_b;

reg	[`M_CTR_WDTH-1:0]	r20_counter;

always @(posedge UCLK) begin: counter_generation
	r20_counter = r20_counter + 1;
	
	// generates 12e6/N Hz at edge of r20_counter[COUNTER_WIDTH-1]
	if ( r20_counter == (`M_12E6/`M_CTR_FREQ) ) begin
		r20_counter = 0;
		r4_led = r4_led + 1;
	end // if
end // counter_generation

assign LED = r4_led;

endmodule // top
