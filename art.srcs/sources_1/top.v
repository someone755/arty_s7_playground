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

/** BEGIN BTN debounce module */
//debouncer #(
//	.p_DEBNC_CLOCKS(2**16),
//	.p_PORT_WIDTH(4) 
//)
//dbnc_instance (
//	.CLK_I(UCLK),
//	.SIGNAL_I(BTN),
//	.SIGNAL_O(LED)
//);
/* END BTN debounce module */

/** BEGIN UART_TX module */
wire [7:0] w8_uart_buffer;
wire w_OUT_UART_TX_READY;
wire w_OUT_UART_RX_DONE;

UART_TX_CTRL #(
	.p_BAUDRATE(9600),
	.p_CLK_FREQ(12_000_000)
)
uart_tx_instance (
	.IN_UART_TX_SEND(w_OUT_UART_RX_DONE),
	.IN8_UART_TX_DATA(w8_uart_buffer),
	.IN_CLK(UCLK),
	
	.OUT_UART_TX_READY(w_OUT_UART_TX_READY),
	.OUT_UART_TX_LINE(UART_RXD_OUT)
);
/* END UART_TX module */

/** BEGIN UART_RX module */
wire w_UART_RX_ENABLE = SW[0];
UART_RX_CTRL #(
	.p_BAUDRATE(9600),
	.p_CLK_FREQ(12_000_000)
	)
uart_rx_ctrl_uut (
	.IN_CLK(UCLK),
	.IN_UART_RX_ENABLE(w_UART_RX_ENABLE),
	.IN_UART_RX(UART_TXD_IN),
	
	.OUT_UART_RX_DONE(w_OUT_UART_RX_DONE),
	.OUT8_UART_RX_DATA(w8_uart_buffer)
	);
/** End UART_RX module */

assign LED = {w_UART_RX_ENABLE, w_OUT_UART_RX_DONE, 1'b0, w_OUT_UART_TX_READY};

endmodule // top
