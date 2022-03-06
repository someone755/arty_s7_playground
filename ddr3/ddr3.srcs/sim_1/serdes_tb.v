`timescale 1ns / 1ps

module serdes_tb;

reg clk_ddr = 0;
initial begin: clk_gen // 100 MHz
	clk_ddr = 0; 
	forever begin
		#(5) // delay half period
		clk_ddr = ~clk_ddr;
	end
end

// "div" clk period
localparam period = 
//6.66; // 150 MHz -- fails in hardware
//20; // 50 MHz -- fails in hardware
//40; // 25 MHz -- partly working in hardware
100; // 10 MHz -- ok in hardware
//10*16; // 6.25 MHz

reg BTN = 1'b0;
reg	[3:0]	SW = 4'b0101;
initial begin: serdes_tb
	BTN = 0;
	#(1200*period)
	BTN = 1;
	#(100*period)
	BTN = 0;
	#(100*period)
	BTN = 1;
	SW = 'b1001;
	#(200*period)
	SW = 'b0110;
	#(200*period)
	SW = 'b0001;
	#(100*period)
	BTN = 0;
	#(100*period)
	BTN = 1;
	#period
	BTN = 0;
	#(50*period)
	$stop;
end

wire strobe;
wire data;

serdes #(
	.p16_TMR_INIT(1)
) serdes_inst (
	/*input*/	.DDR3_CLK100(clk_ddr),	// 100 MHz oscillator input
	/*input	[3:0]*/	.SW(SW),
	/*input	[3:0]*/	.BTN(BTN),
		
	// Refer to UG475
	// All are HR (CSGA324 has no HP), Memory Byte Group 2, Bank 14
	// Strobe pins are U17/U18, marked as DQS pins in pinout
	/*output*/	.jc4_s_o(strobe),	// out strobe
	/*output*/	.jc7_d_o(data),	// out data
	/*input*/	.jc2_d_i(data),	// in data
	/*input*/	.jc3_s_i(strobe)	// in strobe
);
endmodule
