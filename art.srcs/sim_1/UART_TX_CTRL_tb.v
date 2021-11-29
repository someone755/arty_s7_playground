`timescale 1ns / 1ps

module UART_TX_CTRL_tb;
localparam period = 83.333; // period*timescale

/** Begin input/output declaration */
	/** IN_CLK */
	reg IN_CLK = 0;
	
	/** IN_SEND */
	reg IN_UART_TX_SEND = 0;
	
	/** IN8_DATA */
	reg [7:0] IN8_UART_TX_DATA = 33; // ASCII "!"
	
	/** OUT_READY */
	wire OUT_UART_TX_READY;
	
	/** OUT_UART_TX_BYTE_DONE */
	wire OUT_UART_TX_BYTE_DONE;
	
	/** OUT_UART_TX */
	wire OUT_UART_TX_LINE;
	
/** End input/output declaration */

/** Begin UUT declaration */
UART_TX_CTRL #(
	.p_BAUDRATE(9600),
	.p_CLK_FREQ(12_000_000)
)
uart_tx_ctrl_uut (
	.IN_UART_TX_SEND(IN_UART_TX_SEND),
	.IN8_UART_TX_DATA(IN8_UART_TX_DATA),
	.IN_CLK(IN_CLK),
	
	.OUT_UART_TX_READY(OUT_UART_TX_READY),
	.OUT_UART_TX_BYTE_DONE(OUT_UART_TX_BYTE_DONE),
	.OUT_UART_TX_LINE(OUT_UART_TX_LINE)
);
/** End UUT declaration */

initial begin: clk_gen
	IN_CLK = 0;
	forever begin
		#(period/2) // delay half period
		IN_CLK = ~IN_CLK;
	end
end // clk_gen

initial begin: test
	IN_UART_TX_SEND = 0;
	#(1+period*20);
	IN_UART_TX_SEND = 1;
	#period;
	IN_UART_TX_SEND = 1;
	#(period*12_000_0/96*15);
	$stop;
end // test
endmodule