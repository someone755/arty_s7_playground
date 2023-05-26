`timescale 1ns / 1ps
module uart_rx #(
	parameter	p_BAUDRATE	= 9600,
	parameter	p_CLK_FREQ	= 12_000_000
)(
	input	i_clk,
	input	i_en,
	// HW RX line
	input	i_uart_rx,

	output	o_done,
	output	[7:0]	o8_rxdata
);

localparam	STATE_IDLE	= 3'd0,
			STATE_START	= 3'd1,
			STATE_DATA	= 3'd2,
			STATE_STOP	= 3'd3,
			STATE_DONE	= 3'd4;
			
reg [2:0] r3_state = STATE_IDLE;

localparam integer lp_DATA_BIT_TMR_MAX = p_CLK_FREQ/p_BAUDRATE;
localparam lp_BIT_INDEX_MAX = 7; // 8 data bits, 0...7

//--Counter that keeps track of the number of clock cycles the current bit has been held stable over the
//--UART RX line. It is used to signal when the ne
reg [$clog2(lp_DATA_BIT_TMR_MAX)-1:0] rn_bit_tmr = 0;

//--Contains the index of the next bit in txData that needs to be transferred; range 0..7
reg [2:0] r3_bit_index = 0;

//--A register that contains the whole data packet to be received, w/o start or stop bits
reg [7:0] r8_rxdata = 0;
assign o8_rxdata = r8_rxdata;

// Logic 1 for single clock cycle in "DONE" state
reg r_done = 1'b0;
assign o_done = r_done;

always @(posedge i_clk) begin: rx_state_machine
	case (r3_state)
	STATE_IDLE: begin
		r_done <= 1'b0;
		rn_bit_tmr <= 0;
		r3_bit_index <= 0;
		if (~i_uart_rx & i_en)
			r3_state <= STATE_START;				
	end
	STATE_START: begin
		if (rn_bit_tmr < lp_DATA_BIT_TMR_MAX/2)
			rn_bit_tmr <= rn_bit_tmr + 1;
		else begin
			if ( i_uart_rx )
				r3_state <= STATE_IDLE;
			else begin
				rn_bit_tmr <= 0;
				r3_state <= STATE_DATA;
			end
		end
	end
	STATE_DATA: begin
		if (rn_bit_tmr < lp_DATA_BIT_TMR_MAX)
			rn_bit_tmr <= rn_bit_tmr + 1;
		else begin
			rn_bit_tmr <= 0;
			r8_rxdata[r3_bit_index] <= i_uart_rx;
			
			if (r3_bit_index == lp_BIT_INDEX_MAX)
				r3_state <= STATE_STOP;
			else
				r3_bit_index <= r3_bit_index + 1;
		end
	end
	STATE_STOP: begin
		if (rn_bit_tmr < lp_DATA_BIT_TMR_MAX/2)
			rn_bit_tmr <= rn_bit_tmr + 1;
		else
			r3_state <= STATE_DONE;
	end
	STATE_DONE: begin
		r_done <= 1'b1;
		r3_state <= STATE_IDLE;
	end
	default:
		r3_state <= STATE_IDLE;
	endcase
end // rx_state_machine
endmodule // uart_rx
