`timescale 1ns / 1ps

`define	M_12E6	12_000_000	// UCLK freq
`define	M_CTR_FREQ		10	// counter frequency
`define M_CTR_WDTH		($clog2(`M_12E6 / `M_CTR_FREQ))	// maximum width of counter; 12e6 fits into 24 bit

module top
	/*#(
	parameter
	)*/
	(
	input	UCLK,		// 12 MHZ oscillator input
	input	[3:0] SW,
	input	[3:0] BTN,
	
	output	[3:0] LED,
	output	[2:0] RGBLED0, RGBLED1,
	
	input	UART_TXD_IN,
	output	UART_RXD_OUT
	);

/** BEGIN BTN debounce module */
//debouncer #(
//	.p_DEBNC_CLOCKS(2**16),
//	.p_PORT_WIDTH(4) 
//)
//dbnc_instance (
//	.CLK_I(UCLK),
//	.SIGNAL_I(BTN),
//	.SIGNAL_O(LED)
//);
/* END BTN debounce module */

/** Begin BRAM module */
localparam lp_BRAM_WIDTH = 8;
wire [lp_BRAM_WIDTH-1:0] w8_bram_dataout;
reg [lp_BRAM_WIDTH-1:0] r8_bram_datain;

localparam lp_BRAM_DEPTH = 16;
localparam lp_BRAM_ADDR_MAX = $clog2(lp_BRAM_DEPTH)-1;
reg [lp_BRAM_ADDR_MAX:0] rN_bram_addr = 0;

reg r_bram_write_en = 1'b0;

bram #(
	.p_RAM_WIDTH(lp_BRAM_WIDTH),
	.p_RAM_DEPTH(lp_BRAM_DEPTH)
	)
bram_instance (
	.IN_CLK(UCLK),
	.IN_BRAM_ADDR(rN_bram_addr),
	.IN_BRAM_DATAIN(r8_bram_datain),
	.IN_BRAM_WRITE_EN(r_bram_write_en),
	
	.OUT_BRAM_DATAOUT(w8_bram_dataout)
	);
/** End BRAM module */

/** BEGIN UART_TX module */
wire w_uart_tx_rdy;
reg r_uart_tx_send_en;
reg [7:0] r8_uart_tx_data;

UART_TX_CTRL #(
	.p_BAUDRATE(9600),
	.p_CLK_FREQ(12_000_000)
)
uart_tx_instance (
	.IN_UART_TX_SEND(r_uart_tx_send_en),
	.IN8_UART_TX_DATA(r8_uart_tx_data),
	.IN_CLK(UCLK),
	
	.OUT_UART_TX_READY(w_uart_tx_rdy),
	.OUT_UART_TX_LINE(UART_RXD_OUT)
);
/* END UART_TX module */

/** BEGIN UART_RX module */
reg r_uart_rx_en;
wire w_uart_rx_done;
wire [7:0] r8_uart_rx_data;

UART_RX_CTRL #(
	.p_BAUDRATE(9600),
	.p_CLK_FREQ(12_000_000)
	)
uart_rx_instance (
	.IN_CLK(UCLK),
	.IN_UART_RX_ENABLE(r_uart_rx_en),
	.IN_UART_RX(UART_TXD_IN),
	
	.OUT_UART_RX_DONE(w_uart_rx_done),
	.OUT8_UART_RX_DATA(r8_uart_rx_data)
	);
/** End UART_RX module */

localparam	lp_s_RECEIVE =	2'd0,
			lp_s_SEND =		2'd1,
			lp_s_FULL =		2'd2;
reg [1:0] r2_state;
reg r_uart_tx_rdy_prev = 1'b0; 	// TODO: resolve dirty hack
	// Currently (apparently) uart_tx requires two clk edges to begin
	// a transmission. This register is used to bypass this. Further
	// debugging recommended (but not required).
always @(posedge UCLK) begin: bram_test_state_machine
	case (r2_state)
		lp_s_RECEIVE:
			begin
				r_bram_write_en = 0;
				r_uart_rx_en = 1'b1;
				if ( w_uart_rx_done ) begin
					if ( (r8_uart_rx_data == 8'd13) & (rN_bram_addr > 0) ) begin // look for CR if bram not empty
						r2_state = lp_s_SEND;
						r_uart_tx_rdy_prev = 0;
					end else begin
						r8_bram_datain = r8_uart_rx_data;
						r_bram_write_en = 1;
						if ( rN_bram_addr == lp_BRAM_DEPTH-1 )
							r2_state = lp_s_FULL;
						else
							rN_bram_addr = rN_bram_addr + 1;
					end
				end
			end		
		lp_s_SEND: // bram read addr == bram write addr - 1
			begin
				r_bram_write_en = 0;
				r_uart_tx_send_en = 1'b0;
				r_uart_rx_en = 1'b0;
				if ( w_uart_tx_rdy & ~r_uart_tx_rdy_prev ) begin
					if ( rN_bram_addr == 0 )
						r2_state = lp_s_RECEIVE; // transmission done
					else begin
						r_uart_tx_send_en = 1'b1;
						rN_bram_addr = rN_bram_addr - 1;
						r8_uart_tx_data = w8_bram_dataout;
					end
				end
				r_uart_tx_rdy_prev = w_uart_tx_rdy;
			end
		lp_s_FULL: // bram read addr == bram write addr == lp_BRAM_DEPTH-1
			begin
				r_bram_write_en = 0;
				r_uart_tx_send_en = 1'b0;
				r_uart_rx_en = 1'b0;
				if ( w_uart_tx_rdy ) begin
					r8_uart_tx_data = w8_bram_dataout;
					r_uart_tx_send_en = 1'b1;
					r2_state = lp_s_SEND;
					r_uart_tx_rdy_prev = 0;
				end
			end
	endcase
end // bram_test_state_machine

//assign LED = {w_UART_RX_ENABLE, w_OUT_UART_RX_DONE, 1'b0, w_OUT_UART_TX_READY};

endmodule // top
