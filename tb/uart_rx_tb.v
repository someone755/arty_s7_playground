`timescale 1ns / 1ps
module uart_rx_tb;

initial begin
	$dumpfile("uart_rx_tb.fst");
	$dumpvars(0,uart_rx_tb);
end

// clock generation
localparam period = 1e9/12e6; // 83.333 ns at 12 MHz
localparam freq = 1e9/period;
localparam baud = 9600;
reg clk = 1'b0;
initial begin: clk_gen
	clk = 1'b0;
	forever begin
		#(period/2) // delay half period
		clk = ~clk;
	end
end // clk_gen

// uut signals
wire w_uart; // uart line between TX and RX module

reg r_tx_en = 0;
wire [7:0] w8_txdata = 'h21; // ASCII "!"
wire w_tx_ready;

reg r_rx_en = 0;
wire [7:0] w8_rxdata;
wire w_rx_done;	

// uut instantiation
uart_rx #(
	.p_BAUDRATE(baud),
	.p_CLK_FREQ(freq)
) uart_rx_inst (
	.i_clk(clk),
	.i_en(r_rx_en),
	.i_uart_rx(w_uart),

	.o_done(w_rx_done),
	.o8_rxdata(w8_rxdata)
);

uart_tx #(
	.p_BAUDRATE(baud),
	.p_CLK_FREQ(freq)
) uart_tx_inst (
	.i_en(r_tx_en),
	.i8_txdata(w8_txdata),
	.i_clk(clk),

	.o_ready(w_tx_ready),
	.o_uart_tx(w_uart),
	.o_done()
);

// signal excitation
initial begin: test
	r_tx_en <= 0;
	#(period*1250*3);
	r_rx_en <= 1;
	r_tx_en <= 1;
	#period;
	r_tx_en <= 1;
	#(1e9/baud*25);
	r_tx_en <= 1;
	r_rx_en <= 0;
	#(1e9/baud*25);
	$stop;
end // test
endmodule
