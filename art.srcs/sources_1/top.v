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
	output	[2:0] RGBLED0, RGBLED1,
	
	input	UART_TXD_IN,
	output	UART_RXD_OUT
	);

reg [2:0] r3_rgb_led;

//reg	[`M_CTR_WDTH-1:0]	r20_counter;

//always @(posedge UCLK) begin: counter_generation
//	r20_counter = r20_counter + 1;
	
//	// generates 12e6/N Hz at edge of r20_counter[COUNTER_WIDTH-1]
//	if ( r20_counter == (`M_12E6/`M_CTR_FREQ) ) begin
//		r20_counter = 0;
//	end
//end // counter_generation

/** BEGIN BTN debounce module */
wire [3:0] w_BTN_dbnc;
debouncer #(
	.p_DEBNC_CLOCKS(2**16),
	.p_PORT_WIDTH(4) 
)
dbnc_instance (
	.CLK_I(UCLK),
	.SIGNAL_I(BTN),
	.SIGNAL_O(w_BTN_dbnc)
);
/* END BTN debounce module */

/** BEGIN UART_TX module */
reg [7:0] r8_uart_tx_msg = 33;
reg r_uart_send = 1'b0;
wire OUT_READY;
UART_TX_CTRL #(
	.p_BAUDRATE(9600),
	.p_CLK_FREQ(12_000_000)
)
uart_tx_instance (
	.IN_SEND(r_uart_send),
	.IN8_DATA(r8_uart_tx_msg),
	.IN_CLK(UCLK),
	
	.OUT_READY(OUT_READY),
	.OUT_UART_TX(UART_RXD_OUT)
);
/* END UART_TX module */

reg BTN0_prev = 1'b0;
always @(posedge UCLK) begin: uart_tx_process
	if ( (w_BTN_dbnc[0]) ^ (BTN0_prev) ) begin
		BTN0_prev = ~BTN0_prev;
		if (w_BTN_dbnc & OUT_READY) r_uart_send = 1;
	end else
		r_uart_send = 0;
end

endmodule // top
