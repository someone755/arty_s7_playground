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


module uart_tx #(
	parameter	p_BAUDRATE	= 9600,
	parameter	p_CLK_FREQ	= 12_000_000
)(
	input	i_clk,
	input	i_en,
	
	input	[7:0]	i8_txdata,
	// HW TX line
	output	o_uart_tx,

	output	o_done,
	output	o_ready
);

localparam	STATE_READY	= 2'd0,
			STATE_LOAD	= 2'd1,
			STATE_SEND	= 2'd2;
			
reg [1:0] r2_state = STATE_READY;

localparam lp_BIT_TIMER_MAX = p_CLK_FREQ/p_BAUDRATE;
localparam lp_BIT_INDEX_MAX = 9; // start + 8*data + stop

//--Counter that keeps track of the number of clock cycles the current bit has been held stable over the
//--UART TX line. It is used to signal when the ne
reg [$clog2(lp_BIT_TIMER_MAX)-1:0] rn_bit_timer = 0;

//--combinatorial logic that goes high when bitTmr has counted to the proper value to ensure
//--a correct baud rate
wire w_done;

//--Contains the index of the next bit in txData that needs to be transferred; range 0..10
reg [3:0] r4_bit_index = 4'b0;

//--a register that holds the current data being sent over the UART TX line
reg r_tx_bit = 1'b1;

//--A register that contains the whole data packet to be sent, including start and stop bits. 
reg [9:0] r10_txdata = {9'b0, 1'b1};

always @(posedge i_clk) begin: next_txState
	case (r2_state)
	STATE_READY: begin
		r_tx_bit <= 1'b1;
		r4_bit_index <= 4'b0;
		if (i_en)
			r2_state <= STATE_LOAD;
	end
	STATE_LOAD: begin
		r_tx_bit <= r10_txdata[r4_bit_index];
		r2_state <= STATE_SEND;
	end
	STATE_SEND: begin
		if (w_done) begin
			if (r4_bit_index == lp_BIT_INDEX_MAX)
				r2_state <= STATE_READY;
			else begin
				r4_bit_index <= r4_bit_index + 1;
				r2_state <= STATE_LOAD;
			end
		end
	end
	default: ;
	endcase
end

always @(posedge i_clk) begin: bit_timing
	if (r2_state == STATE_READY)
		rn_bit_timer <= 0;
	else begin
		if (w_done == 1)
			rn_bit_timer <= 0;
		else
			rn_bit_timer <= rn_bit_timer + 1;
	end
end // bit_timing

/* verilator lint_off WIDTH */
assign w_done =		(rn_bit_timer == lp_BIT_TIMER_MAX) ? 1'b1 :
					1'b0;
/* verilator lint_on WIDTH */


always @(posedge i_clk) begin: tx_data_latch
	if (i_en)
		r10_txdata <= {1'b1, i8_txdata, 1'b0};
end // tx_data_latch

assign o_uart_tx = r_tx_bit;
assign o_done =	(w_done && (r4_bit_index == lp_BIT_INDEX_MAX)) ? 1'b1 : 1'b0;
assign o_ready =	(r2_state == STATE_READY) ? 1'b1
					: 1'b0;

endmodule
