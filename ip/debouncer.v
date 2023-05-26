`timescale 1ns / 1ps
//----------------------------------------------------------------------------
//--	debouncer.vhd -- Signal Debouncer
//----------------------------------------------------------------------------
//-- Author:  Sam Bobrowicz
//--          Copyright 2011 Digilent, Inc.
//----------------------------------------------------------------------------
//--
//----------------------------------------------------------------------------
//-- This module is used to debounce signals. It is designed to
//-- independently debounce a variable number of signals, the number of which
//-- are set using the PORT_WIDTH parameter. Debouncing is done by only 
//-- registering a change in a button state if it remains constant for 
//-- the number of clocks determined by the DEBNC_CLOCKS parameter. 
//--         				
//-- Parameter Descriptions:
//--
//--   PORT_WIDTH - The number of signals to debounce. determines the width
//--                of the SIGNAL_I and SIGNAL_O inputs (wires)
//--   DEBNC_CLOCKS - The number of clocks (CLK_I) to wait before registering
//--                  a change.
//--
//-- Port Descriptions:
//--
//--   SIGNAL_I - The input signals. A vector of width equal to PORT_WIDTH
//--   CLK_I  - Input clock
//--   SIGNAL_O - The debounced signals. A vector of width equal to PORT_WIDTH
//--   											
//----------------------------------------------------------------------------
//--
//----------------------------------------------------------------------------
//-- Revision History:
//--  08/08/2011(SamB): Created using Xilinx Tools 13.2
//--  08/29/2013(SamB): Improved reuseability by using generics
//--  10/19/2021(JariB): Ported design into Verilog-2001
//----------------------------------------------------------------------------


module debouncer #(
	parameter	p_DEBNC_CLOCKS	= 2**16,
	parameter	p_PORT_WIDTH	= 4
)(
	input	i_clk,
	input	[p_PORT_WIDTH-1:0] in_sig,

	output	[p_PORT_WIDTH-1:0] on_sig
);

reg	[p_PORT_WIDTH-1:0]	rn_out = 0;

localparam	lp_CNTR_WIDTH = $clog2(p_DEBNC_CLOCKS);
reg	[lp_CNTR_WIDTH-1:0]	rn_counters [p_PORT_WIDTH-1:0];


always @(posedge i_clk) begin: debounce_process
	integer i;
	for (i = 0; i < p_PORT_WIDTH; i = i+1) begin
		if (rn_counters[i] == p_DEBNC_CLOCKS-1)
			rn_out[i] = ~rn_out[i];			
	end
end // debounce_process

always @(posedge i_clk) begin: counter_process		
	integer i;
	for (i = 0; i < p_PORT_WIDTH; i = i+1) begin
		if (in_sig[i] ^ rn_out[i]) begin
			if (rn_counters[i] == p_DEBNC_CLOCKS-1)
				rn_counters[i] = 0;
			else
				rn_counters[i] = rn_counters[i] + 1;
		end	else
			rn_counters[i] = 0;
	end
end // counter_process

assign on_sig = rn_out;

endmodule // debouncer
