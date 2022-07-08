`timescale 1ns / 1ps

module UART_RX_CTRL
    #(
	parameter p_BAUDRATE = 9600,
	parameter p_CLK_FREQ = 12_000_000
	)
	(
	input IN_CLK,
	input IN_UART_RX_ENABLE,
	input IN_UART_RX,
	
	output OUT_UART_RX_DONE,
	output [7:0] OUT8_UART_RX_DATA
	);

localparam	lp_STATE_IDLE = 	3'd0,
			lp_STATE_START =	3'd1,
			lp_STATE_DATA =		3'd2,
			lp_STATE_STOP =		3'd3,
			lp_STATE_DONE =		3'd4;
			
reg [2:0] r3_rxState = lp_STATE_IDLE;

localparam lp_DATA_BIT_TMR_MAX = p_CLK_FREQ/p_BAUDRATE;
localparam lp_BIT_INDEX_MAX = 7; // 8 data bits, 0...7

//--Counter that keeps track of the number of clock cycles the current bit has been held stable over the
//--UART RX line. It is used to signal when the ne
reg [$clog2(lp_DATA_BIT_TMR_MAX)-1:0] rN_bitTmr = 0;

//--Contains the index of the next bit in txData that needs to be transferred; range 0..7
reg [2:0] r3_bitIndex = 0;

//--A register that contains the whole data packet to be received, w/o start or stop bits
reg [7:0] r8_rxData = 0;
assign OUT8_UART_RX_DATA = r8_rxData;

// Logic 1 for single clock cycle in "DONE" state
reg r_UART_OUT_RX_BYTE_DONE = 1'b0;
assign OUT_UART_RX_DONE = r_UART_OUT_RX_BYTE_DONE;

always @(posedge IN_CLK) begin: rx_state_machine
	case (r3_rxState)
		lp_STATE_IDLE:
			begin
				r_UART_OUT_RX_BYTE_DONE <= 0;
				rN_bitTmr <= 0;
				r3_bitIndex <= 0;
				if ( ~IN_UART_RX & IN_UART_RX_ENABLE )
					r3_rxState <= lp_STATE_START;				
			end
		lp_STATE_START:
			begin
				if ( rN_bitTmr < lp_DATA_BIT_TMR_MAX/2 )
					rN_bitTmr <= rN_bitTmr + 1;
				else begin
					if ( IN_UART_RX )
						r3_rxState <= lp_STATE_IDLE;
					else begin
						rN_bitTmr <= 0;
						r3_rxState <= lp_STATE_DATA;
					end
				end
			end
		lp_STATE_DATA:
			begin
				if ( rN_bitTmr < lp_DATA_BIT_TMR_MAX )
					rN_bitTmr <= rN_bitTmr + 1;
				else begin
					rN_bitTmr <= 0;
					r8_rxData[r3_bitIndex] <= IN_UART_RX;
					
					if ( r3_bitIndex == lp_BIT_INDEX_MAX )
						r3_rxState <= lp_STATE_STOP;
					else
						r3_bitIndex <= r3_bitIndex + 1;
				end
			end
		lp_STATE_STOP:
			begin
				if ( rN_bitTmr < lp_DATA_BIT_TMR_MAX/2 )
					rN_bitTmr <= rN_bitTmr + 1;
				else
					r3_rxState <= lp_STATE_DONE;
			end
		lp_STATE_DONE:
			begin
				r_UART_OUT_RX_BYTE_DONE <= 1;
				r3_rxState <= lp_STATE_IDLE;
			end
		default:
			r3_rxState <= lp_STATE_IDLE;
	endcase
end
endmodule
