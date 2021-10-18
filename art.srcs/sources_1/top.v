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

endmodule // top
