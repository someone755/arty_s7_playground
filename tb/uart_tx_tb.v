`timescale 1ns / 1ps
module uart_tx_tb;

initial begin
	$dumpfile("uart_tx_tb.fst");
	$dumpvars(0,uart_tx_tb);
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
reg r_en = 0;
wire [7:0] w8_txdata = 'h21; // ASCII "!"
wire w_ready;
wire w_done;
wire w_uart_tx;

// uut instantiation
uart_tx #(
	.p_BAUDRATE(baud),
	.p_CLK_FREQ(freq)
) uart_tx_inst (
	.i_en(r_en),
	.i8_txdata(w8_txdata),
	.i_clk(clk),

	.o_ready(w_ready),
	.o_done(w_done),
	.o_uart_tx(w_uart_tx)
);

// signal excitation
initial begin: test
	#(period/2)
	r_en = 0;
	#(period*5);
	r_en = 1;
	#period;
	r_en = 1;
	#(1e9/baud*15);
	$stop;
	$finish;
end // test
endmodule
