`timescale 1ns / 1ps

module bram_uart_tb;
localparam period = 1000; // period*timescale

/** Begin input/output declaration */
/* Common */
	/** IN_CLK */
	reg IN_CLK = 0;
	
/** End input/output declaration */

/** ### BEGIN INPUT UART SEGMENT ### **/
/* TX1 module */
	/** IN_UART1_TX_SEND */
	reg IN_UART1_TX_SEND = 0;
	
	/** IN8_UART_TX_DATA */
	reg [7:0] IN8_UART_TX_DATA = 8'd33; // ASCII "!"
	
	/** OUT_READY */
	wire OUT_UART1_TX_READY;
	
	/** IN_UART1_LINE */
	wire IN_UART1_LINE; // shared w/ UART1 RX
/** Begin TX1 declaration */
uart_tx #(
	.p_BAUDRATE(10_000),
	.p_CLK_FREQ(1_000_000)
) uart_tx_inst1 (
	.i_en(IN_UART1_TX_SEND),
	.i8_txdata(IN8_UART_TX_DATA),
	.i_clk(IN_CLK),
	
	.o_ready(OUT_UART1_TX_READY),
	.o_uart_tx(IN_UART1_LINE)
);
/** End TX1 declaration */

/* RX module */	
	/** IN_UART1_RX_ENABLE */
	reg IN_UART1_RX_ENABLE = 0;
	
	/** OUT8_UART1_RX_DATA */
	wire [7:0] OUT8_UART1_RX_DATA;
		
	/** OUT_UART1_RX_DONE */
	wire OUT_UART1_RX_DONE;	
/** Begin RX declaration */
uart_rx #(
	.p_BAUDRATE(10_000),
	.p_CLK_FREQ(1_000_000)
) uart_rx_inst (
	.i_clk(IN_CLK),
	.i_en(IN_UART1_RX_ENABLE),
	.i_uart_rx(IN_UART1_LINE),
	
	.o_done(OUT_UART1_RX_DONE),
	.o8_rxdata(OUT8_UART1_RX_DATA)
	);
/** End RX declaration */
/** ### END INPUT UART SEGMENT ### **/

/** Begin BRAM module */
localparam lp_BRAM_WIDTH = 8;
wire [lp_BRAM_WIDTH-1:0] w8_bram_dataout;

localparam lp_BRAM_DEPTH = 4;
localparam lp_BRAM_ADDR_MAX = $clog2(lp_BRAM_DEPTH)-1;
reg [lp_BRAM_ADDR_MAX:0] rN_bram_addr;

//reg r_bram_write_en = 1'b0;
bram #(
	.p_RAM_WIDTH(lp_BRAM_WIDTH),
	.p_RAM_DEPTH(lp_BRAM_DEPTH)
	)
bram_instance (
	.IN_CLK(~IN_CLK),
	.IN_BRAM_ADDR(rN_bram_addr),
	.IN_BRAM_DATAIN(OUT8_UART1_RX_DATA),
	.IN_BRAM_WRITE_EN(OUT_UART1_RX_DONE),
	
	.OUT_BRAM_DATAOUT(w8_bram_dataout)
	);
/** End BRAM module */

/* TX1 module */
	/** IN_UART1_TX_SEND */
	reg IN_UART2_TX_SEND = 0;
	
	/** IN8_UART_TX_DATA */
	reg [7:0] IN8_UART_TX_DATA = 8'd33; // ASCII "!"
	
	/** OUT_READY */
	wire OUT_UART2_TX_READY;
	
	/** OUT_UART2_LINE */
	wire OUT_UART2_LINE;
/** Begin TX2 declaration */
uart_tx #(
	.p_BAUDRATE(10_000),
	.p_CLK_FREQ(1_000_000)
) uart_tx_inst2 (
	.i_en(IN_UART2_TX_SEND),
	.i8_txdata(w8_bram_dataout),
	.i_clk(IN_CLK),
	
	.o_ready(OUT_UART2_TX_READY),
	.o_uart_tx(OUT_UART2_LINE)
);
/** End TX2 declaration */

initial begin: clk_gen
	IN_CLK = 0;
	forever begin
		#(period/2) // delay half period
		IN_CLK = ~IN_CLK;
	end
end // clk_gen

initial begin: test
	IN_UART1_TX_SEND = 0;
	#(period*100-500);
	IN_UART1_RX_ENABLE = 1;
	IN_UART1_TX_SEND = 1;
	rN_bram_addr = 0;
	IN8_UART_TX_DATA = 8'b11111111;
	#(period*1000);
	rN_bram_addr = 1;
	IN8_UART_TX_DATA = 8'b10101010;
	#(period*1000);
	rN_bram_addr = 2;
	IN8_UART_TX_DATA = 8'b01010101;
	#(period*1000);
	IN_UART1_TX_SEND = 0;
	IN_UART2_TX_SEND = 1;
	rN_bram_addr = 0;
	#(period*1000);
	rN_bram_addr = 1;
	#(period*1000);
	rN_bram_addr = 2;
	#(period*1000);
	$stop;
end // test
endmodule
