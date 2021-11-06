`timescale 1ns / 1ps

`define	M_12E6	12_000_000	// UCLK freq
`define	M_CTR_FREQ		10	// counter frequency
`define M_CTR_WDTH		($clog2(`M_12E6 / `M_CTR_FREQ))	// maximum width of counter; 12e6 fits into 24 bit

module top (
	input	UCLK,		// 12 MHZ oscillator input
	input	[3:0] SW,
	input	[3:0] BTN,
	
	output	[3:0] LED,
	output	[2:0] RGBLED0, RGBLED1,
	
	input	UART_TXD_IN,
	output	UART_RXD_OUT 
);
	
/** BEGIN UART_RX module */
	reg r_uart_rx_en = 1'b1;
	wire w_uart_rx_done;
	wire [7:0] w8_uart_rx_data;
UART_RX_CTRL #(
	.p_BAUDRATE(9600),
	.p_CLK_FREQ(12_000_000)
	)
uart_rx_instance (
	.IN_CLK(UCLK),
	.IN_UART_RX_ENABLE(r_uart_rx_en),
	.IN_UART_RX(UART_TXD_IN),
	
	.OUT_UART_RX_DONE(w_uart_rx_done),
	.OUT8_UART_RX_DATA(w8_uart_rx_data)
	);
/** End UART_RX module */
/** Begin BRAM module */
	localparam lp_BRAM_WIDTH = 8;
	wire [lp_BRAM_WIDTH-1:0] w8_bram_dataout;
	wire [lp_BRAM_WIDTH-1:0] w8_bram_datain;
	
	localparam lp_BRAM_DEPTH = 16;
	localparam lp_BRAM_ADDR_WIDTH = $clog2(lp_BRAM_DEPTH)-1;
	reg [lp_BRAM_ADDR_WIDTH:0] rN_bram_addr;
	
	reg r_bram_write_en = 1'b0;
bram #(
	.p_RAM_WIDTH(lp_BRAM_WIDTH),
	.p_RAM_DEPTH(lp_BRAM_DEPTH)
	)
bram_instance (
	.IN_CLK(UCLK),
	.IN_BRAM_ADDR(rN_bram_addr),
	.IN_BRAM_DATAIN(w8_bram_datain),
	.IN_BRAM_WRITE_EN(r_bram_write_en),
	
	.OUT_BRAM_DATAOUT(w8_bram_dataout)
	);
/** End BRAM module */
/** Begin UART_TX module */
	wire w_uart_tx_rdy;
	reg r_uart_tx_send_en;
	wire [7:0] w8_uart_tx_data;
UART_TX_CTRL #(
	.p_BAUDRATE(9600),
	.p_CLK_FREQ(12_000_000)
)
uart_tx_instance (
	.IN_UART_TX_SEND(r_uart_tx_send_en),
	.IN8_UART_TX_DATA(w8_uart_tx_data),
	.IN_CLK(UCLK),
	
	.OUT_UART_TX_READY(w_uart_tx_rdy),
	.OUT_UART_TX_LINE(UART_RXD_OUT)
);
/* End UART_TX module */

assign w8_bram_datain = w8_uart_rx_data; // wire RX received byte to BRAM datain
assign w8_uart_tx_data = w8_bram_dataout; // wire BRAM dataout to TX transmit byte 

localparam	lp_s_RECEIVE =		2'd0,
			lp_s_PREP_TX_BYTE = 	2'd1,
			lp_s_HOLD_TX_BYTE =		2'd2,
			lp_s_WRITE_LAST_BYTE = 	2'd3;
reg [1:0] r_state = 2'd0;
reg r_uart_tx_rdy_prev = 1'b0; 	// TODO: resolve dirty hack
	// Currently (apparently) uart_tx requires two clk edges to begin
	// a transmission. This register is used to bypass this. Further
	// debugging recommended (but not required).
reg  [lp_BRAM_ADDR_WIDTH:0] rN_bram_write_addr = 0;
reg  [lp_BRAM_ADDR_WIDTH:0] rN_bram_read_addr = 0;

assign LED[0] = r_state;

always @(posedge UCLK) begin: bram_test_state_machine
case (r_state)
	lp_s_RECEIVE:
	begin
		if ( !w_uart_rx_done ) begin
			rN_bram_read_addr <= 0; // keep read addr reset
			r_bram_write_en <= 1'b0; // do not write unless RX byte is valid
			r_uart_rx_en <= 1'b1; // keep UART RX enabled
			//w8_bram_datain <= w8_uart_rx_data; // sequential connection done outside always block
			rN_bram_addr <= rN_bram_write_addr; // set BRAM address line to write counter for next received byte
		end else begin
			if ( (w8_uart_rx_data == 8'd13) && (rN_bram_write_addr > 0) ) begin // look for CR if bram not empty
				/* In this implementation, the CR byte is not saved; instead, a subtraction is made on the write address
					(read address counter maximal value). An alternative to avoid this subtraction would be to also save
					and transmit the CR byte */
				rN_bram_write_addr <= rN_bram_write_addr - 1;
				rN_bram_addr <= 0; // = rN_bram_read_addr, should be 0 at this point
				r_bram_write_en <= 1'b0;
				r_uart_rx_en <= 1'b0; // disable RX during SEND
				r_state <= lp_s_PREP_TX_BYTE;
			end else begin // data is received and can be written
				r_bram_write_en <= 1; // write to current write address (set in previous clk period)
				if ( rN_bram_addr == lp_BRAM_DEPTH-1 ) begin // if BRAM is full after write
					r_state <= lp_s_WRITE_LAST_BYTE; 	// cannot immediately change bram address
														// or current byte will be written to addr 0
				end else begin
					rN_bram_write_addr <= rN_bram_write_addr + 1; // avoid overflow on increment
				end
			end
		end
	end
	lp_s_PREP_TX_BYTE:
	begin
		r_bram_write_en <= 1'b0; // do not write during SEND
		r_uart_rx_en <= 1'b0; // keep UART RX disabled
		r_uart_tx_send_en = 1'b1;
		
		r_state <= lp_s_HOLD_TX_BYTE;
		r_uart_tx_rdy_prev = 1;
	end
	lp_s_HOLD_TX_BYTE:
	begin
		r_uart_tx_send_en <= 1'b0; // keep TX send sig low during transmit
		if ( w_uart_tx_rdy & ~r_uart_tx_rdy_prev ) begin
			if ( rN_bram_addr == rN_bram_write_addr ) begin
				rN_bram_write_addr <= 0;
				r_state <= lp_s_RECEIVE; // done sending last byte
			end else begin
				rN_bram_addr <= rN_bram_addr + 1; // increment read address for next byte
				r_state <= lp_s_PREP_TX_BYTE; // dont sending <max byte
			end
		end
		r_uart_tx_rdy_prev <= w_uart_tx_rdy;
	end
	lp_s_WRITE_LAST_BYTE:
	begin
		r_bram_write_en <= 1'b0;
		rN_bram_addr <= 0; // = rN_bram_read_addr, should be 0 at this point
		rN_bram_read_addr <= 0;
		r_uart_rx_en <= 1'b0; // disable RX during SEND
		r_state <= lp_s_PREP_TX_BYTE;
	end
endcase
end
endmodule // top