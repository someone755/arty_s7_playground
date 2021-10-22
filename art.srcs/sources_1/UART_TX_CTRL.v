`timescale 1ns / 1ps
//----------------------------------------------------------------------------
//--	UART_TX_CTRL.vhd -- UART Data Transfer Component
//----------------------------------------------------------------------------
//-- Author:  Sam Bobrowicz
//--          Copyright 2011 Digilent, Inc.
//----------------------------------------------------------------------------
//--
//----------------------------------------------------------------------------
//--	This component may be used to transfer data over a UART device. It will
//-- serialize a byte of data and transmit it over a TXD line. The serialized
//-- data has the following characteristics:
//--         *configurable Baud Rate
//--         *8 data bits, LSB first
//--         *1 stop bit
//--         *no parity
//--         				
//-- Port Descriptions:
//--
//--    SEND - Used to trigger a send operation. The upper layer logic should 
//--           set this signal high for a single clock cycle to trigger a 
//--           send. When this signal is set high DATA must be valid . Should 
//--           not be asserted unless READY is high.
//--    DATA - The parallel data to be sent. Must be valid the clock cycle
//--           that SEND has gone high.
//--    CLK  - Configurable clock signal
//--   READY - This signal goes low once a send operation has begun and
//--           remains low until it has completed and the module is ready to
//--           send another byte.
//-- UART_TX - This signal should be routed to the appropriate TX pin of the 
//--           external UART device.
//--   
//----------------------------------------------------------------------------
//--
//----------------------------------------------------------------------------
//-- Revision History:
//--  08/08/2011(SamB): Created using Xilinx Tools 13.2
//--  10/22/2021(JariB): Ported design into Verilog-2001
//----------------------------------------------------------------------------


module UART_TX_CTRL
	#(
	parameter p_BAUDRATE = 9600,
	parameter p_CLK_FREQ = 12_000_000
	)
	(
	input IN_SEND,
	input [7:0] IN8_DATA,
	input IN_CLK,
	
	output OUT_READY,
	output OUT_UART_TX
	);

localparam	lp_RDY = 		2'd0,
			lp_LOAD_BIT =	2'd1,
			lp_SEND_BIT =	2'd2;
			
reg [1:0] r2_txState = lp_RDY;

parameter lp_BIT_TMR_MAX = p_CLK_FREQ/p_BAUDRATE;
localparam lp_BIT_INDEX_MAX = 10; // start + 8*data + stop

//--Counter that keeps track of the number of clock cycles the current bit has been held stable over the
//--UART TX line. It is used to signal when the ne
reg [$clog2(lp_BIT_TMR_MAX)-1:0] rN_bitTmr = 0;

//--combinatorial logic that goes high when bitTmr has counted to the proper value to ensure
//--a 9600 baud rate
wire w_bitDone;

//--Contains the index of the next bit in txData that needs to be transferred; range 0..10
reg [3:0] r4_bitIndex = 0;

//--a register that holds the current data being sent over the UART TX line
reg r_txBit = 1'b0;

//--A register that contains the whole data packet to be sent, including start and stop bits. 
reg [9:0] r10_txData = 0;

always @(posedge IN_CLK) begin: next_txState
case (r2_txState)
	lp_RDY:
		if ( IN_SEND == 1 ) r2_txState = lp_LOAD_BIT;
	lp_LOAD_BIT:
		r2_txState = lp_SEND_BIT;
	lp_SEND_BIT:
		begin
		if ( w_bitDone == 1 ) begin
			if ( r4_bitIndex == lp_BIT_INDEX_MAX )
				r2_txState = lp_RDY;
			else
				r2_txState = lp_LOAD_BIT;
		end
		end
	default:
		r2_txState = lp_RDY; // should never be reached
endcase
end // next_txState

always @(posedge IN_CLK) begin: bit_timing
	if ( r2_txState == lp_RDY )
		rN_bitTmr = 0;
	else begin
		if ( w_bitDone == 1 )
			rN_bitTmr = 0;
		else
			rN_bitTmr = rN_bitTmr + 1;
	end
end // bit_timing

assign w_bitDone = 	(rN_bitTmr == lp_BIT_TMR_MAX) ? 1'b1 :
					1'b0;

always @(posedge IN_CLK) begin: bit_counting
	if (r2_txState == lp_RDY)
		r4_bitIndex = 0;
	else if (r2_txState == lp_LOAD_BIT )
		r4_bitIndex = r4_bitIndex + 1;
end // bit_counting

always @(posedge IN_CLK) begin: tx_data_latch
	if ( IN_SEND == 1 )
		r10_txData = {1'b1, IN8_DATA, 1'b0};
end // tx_data_latch

always @(posedge IN_CLK) begin: tx_bit
	if ( r2_txState == lp_RDY )
		r_txBit = 1;
	else if ( r2_txState == lp_LOAD_BIT )
		r_txBit = r10_txData[r4_bitIndex-1];
end // tx_bit

assign OUT_UART_TX = r_txBit;
assign OUT_READY =	(r2_txState == lp_RDY) ? 1 :
					0;

endmodule
