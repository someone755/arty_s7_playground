`timescale 1ns / 1ps

module UART_TX_CTRL_tb;
localparam period = 83.333; // period*timescale

/** Begin input/output declaration */
	/** IN_CLK */
	reg IN_CLK = 0;
	
	/** IN_SEND */
	reg IN_SEND = 0;
	
	/** IN8_DATA */
	reg [7:0] IN8_DATA = 33; // ASCII "!"
	
	/** OUT_READY */
	wire OUT_READY;
	
	/** OUT_UART_TX */
	wire OUT_UART_TX;
	
/** End input/output declaration */

/** Begin UUT declaration */
UART_TX_CTRL #(
	.p_BAUDRATE(9600),
	.p_CLK_FREQ(12_000_000)
)
uart_tx_ctrl_uut (
	.IN_SEND(IN_SEND),
	.IN8_DATA(IN8_DATA),
	.IN_CLK(IN_CLK),
	
	.OUT_READY(OUT_READY),
	.OUT_UART_TX(OUT_UART_TX)
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
	IN_SEND = 0;
	#(period*1250*3);
	IN_SEND = 1;
	#period;
	IN_SEND = 0;
	#(period*12_000_0/96*15);
	$stop;
end // test
endmodule