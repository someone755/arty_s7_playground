`timescale 1ns / 1ps

module UART_RX_CTRL_tb;
localparam period = 83.333; // period*timescale

/** Begin input/output declaration */
/* Common */
	/** IN_CLK */
	reg IN_CLK = 0;
	
	/** UART_LINE */
	wire UART_LINE;
	
/* TX module */
	/** IN_UART_TX_SEND */
	reg IN_UART_TX_SEND = 0;
	
	/** IN8_UART_TX_DATA */
	reg [7:0] IN8_UART_TX_DATA = 33; // ASCII "!"
	
	/** OUT_READY */
	wire OUT_UART_TX_READY;
	
/* RX module */	
	/** IN_UART_RX_ENABLE */
	reg IN_UART_RX_ENABLE = 0;
	
	/** OUT8_UART_RX_DATA */
	wire [7:0] OUT8_UART_RX_DATA;
		
	/** OUT_UART_RX_DONE */
	wire OUT_UART_RX_DONE;	
	
/** End input/output declaration */

/** Begin RX declaration */
UART_RX_CTRL #(
	.p_BAUDRATE(9600),
	.p_CLK_FREQ(12_000_000)
	)
uart_rx_ctrl_uut (
	.IN_CLK(IN_CLK),
	.IN_UART_RX_ENABLE(IN_UART_RX_ENABLE),
	.IN_UART_RX(UART_LINE),
	
	.OUT_UART_RX_DONE(OUT_UART_RX_DONE),
	.OUT8_UART_RX_DATA(OUT8_UART_RX_DATA)
	);
/** End RX declaration */

/** Begin TX declaration */
UART_TX_CTRL #(
	.p_BAUDRATE(9600),
	.p_CLK_FREQ(12_000_000)
)
uart_tx_ctrl_uut (
	.IN_UART_TX_SEND(IN_UART_TX_SEND),
	.IN8_UART_TX_DATA(IN8_UART_TX_DATA),
	.IN_CLK(IN_CLK),
	
	.OUT_UART_TX_READY(OUT_UART_TX_READY),
	.OUT_UART_TX_LINE(UART_LINE)
);
/** End TX declaration */

initial begin: clk_gen
	IN_CLK = 0;
	forever begin
		#(period/2) // delay half period
		IN_CLK = ~IN_CLK;
	end
end // clk_gen

initial begin: test
	IN_UART_TX_SEND = 0;
	#(period*1250*3);
	IN_UART_RX_ENABLE = 1;
	IN_UART_TX_SEND = 1;
	#period;
	IN_UART_TX_SEND = 1;
	#(period*12_000_0/96*25);
	IN_UART_TX_SEND = 1;
	IN_UART_RX_ENABLE = 0;
	#(period*12_000_0/96*25);
	$stop;
end // test
endmodule
