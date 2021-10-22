`timescale 1ns / 1ps

`define	M_12E6	12_000_000	// UCLK freq
`define	M_CTR_FREQ		10	// counter frequency
`define M_CTR_WDTH		($clog2(`M_12E6 / `M_CTR_FREQ))	// maximum width of counter; 12e6 fits into 24 bit

module top
	/*#(
	parameter
	)*/
	(
	input	UCLK,		// 12 MHZ oscillator input
	input	[3:0] SW,
	input	[3:0] BTN,
	
	output	[3:0] LED,
	output	[2:0] RGBLED0, RGBLED1	
	);

//reg	r_led0_r, r_led0_g, r_led0_b;
//reg	r_led1_r, r_led1_g, r_led1_b;
	
reg [2:0] r3_rgb_led;

reg	[`M_CTR_WDTH-1:0]	r20_counter;

always @(posedge UCLK) begin: counter_generation
	r20_counter = r20_counter + 1;
	
	// generates 12e6/N Hz at edge of r20_counter[COUNTER_WIDTH-1]
	if ( r20_counter == (`M_12E6/`M_CTR_FREQ) ) begin
		r20_counter = 0;
		r3_rgb_led = r3_rgb_led + 1;
	end
end // counter_generation

assign RGBLED0 = r3_rgb_led;
assign {RGBLED1[0], RGBLED1[1], RGBLED1[2]} = { r3_rgb_led[1], r3_rgb_led[2], r3_rgb_led[0] };

debouncer #(
	.p_DEBNC_CLOCKS(2**16),
	.p_PORT_WIDTH(4) 
)
dbnc_instance (
	.CLK_I(UCLK),
	.SIGNAL_I(BTN),
	.SIGNAL_O(LED)
);

endmodule // top
