`timescale 1ns / 1ps

`define	M_12E6	12_000_000	// UCLK freq
`define	M_CTR_FREQ		10	// counter frequency
`define M_CTR_WDTH		($clog2(`M_12E6 / `M_CTR_FREQ))	// maximum width of counter; 12e6 fits into 24 bit

module top (
	//input	UCLK,		// 12 MHZ oscillator input
	input	DDR3_CLK100,	// 100 MHz oscillator input
	input	[3:0]	SW,
	input	[3:0]	BTN,
	
	output	[3:0]	LED,
	output reg	[2:0]	RGBLED0, RGBLED1,
	
	input	UART_TXD_IN,
	output	UART_RXD_OUT,
	
	// ### BEGIN DDR3 IO ###
	// Inouts
	inout	[15:0]	ddr3_dq,
	inout 	[1:0]	ddr3_dqs_n,
	inout	[1:0]	ddr3_dqs_p,
	
	// Outputs
	output	[13:0]	ddr3_addr,
	output	[2:0]	ddr3_ba,
	output	ddr3_ras_n,
	output	ddr3_cas_n,
	output	ddr3_we_n,
	output  ddr3_reset_n,
	output	[0:0]	ddr3_ck_p,
	output	[0:0]	ddr3_ck_n,
	output	[0:0]	ddr3_cke,
	output	[0:0]	ddr3_cs_n,
	output 	[1:0]	ddr3_dm,
	output	[0:0]	ddr3_odt
	
	// Inputs
	// Single-ended system clock
	//input	sys_clk_i, -- 100 MHz, see DDR3_CLK100
	// Single-ended iodelayctrl clk (reference clock)
	//input	clk_ref_i, -- 200 MHz
	
	// ### END DDR3 IO ###
);
	/* uart clock signal */
	wire w_uart_clk;
	assign w_uart_clk = ui_clk;
	localparam lp_UART_CLK_FREQ = 325_000_000/nCK_PER_CLK;
	localparam lp_UART_BAUDRATE = 6_000_000;
	
/** BEGIN UART_RX module */
	reg r_uart_rx_en = 1'b0;
	wire w_uart_rx_done;
	wire [7:0] w8_uart_rx_data;
UART_RX_CTRL #(
	.p_BAUDRATE(lp_UART_BAUDRATE),
	.p_CLK_FREQ(lp_UART_CLK_FREQ)
	)
uart_rx_instance (
	.IN_CLK(w_uart_clk),
	.IN_UART_RX_ENABLE(r_uart_rx_en),
	.IN_UART_RX(UART_TXD_IN),
	
	.OUT_UART_RX_DONE(w_uart_rx_done),
	.OUT8_UART_RX_DATA(w8_uart_rx_data)
	);
/** End UART_RX module */
/** Begin UART_TX module */
	wire w_uart_tx_rdy;
	wire w_uart_tx_byte_done;
	reg r_uart_tx_send_en = 1'b0;
	reg [7:0] r8_uart_tx_data;
UART_TX_CTRL #(
	.p_BAUDRATE(lp_UART_BAUDRATE),
	.p_CLK_FREQ(lp_UART_CLK_FREQ)
)
uart_tx_instance (
	.IN_UART_TX_SEND(r_uart_tx_send_en),
	.IN8_UART_TX_DATA(r8_uart_tx_data),
	.IN_CLK(w_uart_clk),
	
	.OUT_UART_TX_READY(w_uart_tx_rdy),
	.OUT_UART_TX_BYTE_DONE(w_uart_tx_byte_done),
	.OUT_UART_TX_LINE(UART_RXD_OUT)
);
/* End UART_TX module */
/* Begin 200 MHz clock generator */
wire w_clk200_reset = 0;
wire w_clk200_locked;
wire w_clk200;
clk_wiz_0 clk200_instance (
	.clk_out1(w_clk200), // Clock out
	.reset(w_clk200_reset),
	.locked(w_clk200_locked), // Status signal
	.clk_in1(DDR3_CLK100) // Clock in
);
/* End 200 MHz clock generator */
/* Begin DDR3 */
// signal parameters
localparam ADDR_WIDTH =	28;
localparam DATA_WIDTH =	16;
localparam nCK_PER_CLK =	4;
localparam APP_DATA_WIDTH =	2 * nCK_PER_CLK * DATA_WIDTH;
localparam APP_MASK_WIDTH =	APP_DATA_WIDTH/8;
// App signals to user || reg denotes input || wire denotes output unless otherwise stated
// Explanations taken from UG586 pg. 50 and PG150 pg. 118
reg	[ADDR_WIDTH-1:0]	app_addr;	// Address for current request, 28 wide
reg	[2:0]				app_cmd;	// Command for current request: 3'b000 = WRITE || 3'b0001 = READ
reg						app_en;		// Active high strobe for inputs: app_addr, app_cmd
wire					app_rdy;	// Active high indicates UI ready to accept commands
wire	[APP_DATA_WIDTH-1:0]	app_rd_data;		// Provides output data from read commands, 128-wide
wire							app_rd_data_end;	// Active high indicates current clock cycle is last cycle of output data on app_rd_data[]. Safe to ignore at 4:1.
wire							app_rd_data_valid;	// Active high indicates app_rd_data is valid
reg	[APP_DATA_WIDTH-1:0]		app_wdf_data =	128'b0;	// Data for write commands, 128-wide
wire							app_wdf_end;			// Active high indicates current clock cycle is last cycle of input data on app_wdf_data[]. Tied to app_wdf_wren.
wire	[APP_MASK_WIDTH-1:0]	app_wdf_mask = 	8'b0;	// Input; Mask for app_wdf_data[], 8-wide
wire							app_wdf_rdy;			// Indicates that write data FIFO is ready to receive data. Ready when (app_wdf_rdy & app_wdf_wren) = 1'b1
reg								app_wdf_wren;			// Active high strobe for app_wdf_data[]
assign app_wdf_end = 1'b1;	// Docs will tell you to wire this to app_wdf_wren.

wire	app_sr_req =	1'b0;	// Input; Undocumented functionality
wire	app_sr_active;			// Undocumented functionality; NOT CONNECTED
wire	app_ref_req =	1'b0;	// Input; User refresh request
wire	app_ref_ack;			// User refresh request completed; NOT CONNECTED
wire	app_zq_req = 	1'b0;	// Input; User ZQCS command request
wire	app_zq_ack;				// User ZQCS command request completed; NOT CONNECTED

wire	ui_clk;				// UI clock is one half of the DRAM clock
wire	ui_clk_sync_rst;	// Active high UI reset

wire	[11:0]	device_temp;	// MIG claims signal used for calibration (undocumented elsewhere); NOT CONNECTED
wire init_calib_complete;	// Active high when calibration complete

// System reset - Default polarity of sys_rst pin is Active Low.
wire	ddr_sys_rst =	1'b1;	// Input

ddr3l u_ddr3l (
// Memory interface ports
// Used by generated memory controller to interface with DDR3 memory
	.ddr3_addr				(ddr3_addr),
	.ddr3_ba				(ddr3_ba),
	.ddr3_cas_n				(ddr3_cas_n),
	.ddr3_ck_n				(ddr3_ck_n),
	.ddr3_ck_p				(ddr3_ck_p),
	.ddr3_cke				(ddr3_cke),
	.ddr3_ras_n				(ddr3_ras_n),
	.ddr3_we_n				(ddr3_we_n),
	.ddr3_dq				(ddr3_dq),
	.ddr3_dqs_n				(ddr3_dqs_n),
	.ddr3_dqs_p				(ddr3_dqs_p),
	.ddr3_reset_n			(ddr3_reset_n),
	.init_calib_complete	(init_calib_complete),
	.ddr3_cs_n				(ddr3_cs_n),
	.ddr3_dm				(ddr3_dm),
	.ddr3_odt				(ddr3_odt),

// Application interface ports
	.app_addr			(app_addr),
	.app_cmd			(app_cmd),
	.app_en				(app_en),
	.app_wdf_data		(app_wdf_data),
	.app_wdf_end		(app_wdf_end),
	.app_wdf_wren		(app_wdf_wren),
	.app_rd_data		(app_rd_data),
	.app_rd_data_end	(app_rd_data_end),
	.app_rd_data_valid	(app_rd_data_valid),
	.app_rdy			(app_rdy),
	.app_wdf_rdy		(app_wdf_rdy),
	.app_sr_req			(1'b0),
	.app_ref_req		(1'b0),
	.app_zq_req			(1'b0),
	.app_sr_active		(app_sr_active),
	.app_ref_ack		(app_ref_ack),
	.app_zq_ack			(app_zq_ack),
	.ui_clk				(ui_clk),
	.ui_clk_sync_rst	(ui_clk_sync_rst),
	.app_wdf_mask		(app_wdf_mask),

// System Clock Ports
	.sys_clk_i	(DDR3_CLK100),

// Reference Clock Ports
	.clk_ref_i				(w_clk200),
	.device_temp			(device_temp),
	.sys_rst				(ddr_sys_rst)
);
/* End DDR3 */

reg [2:0] r3_uart_state = 3'b0; // state machine select
reg [127:0] r128_ddr_rd_buffer = 128'b0; // 128 bit (read) buffer from DDR
reg [3:0] r4_uart_byte_index = 4'b1111; // counts bytes in DDR read vector for uart tx

reg [127:0] r128_2_rx_buff [0:1]; // 2x128 bit rx buffer
reg [3:0] r4_rx_byte_index = 4'b1111; // counts 16 bytes across 128 bit words in 2x128 bit rx buffer
reg r1_rx_word_index = 1'b0; // counts 128 bit words in rx buffer
reg r1_rx_word_index_delay = 1'b0;
reg r1_rx_word_index_prev_read = 1'b0;

reg [27:0] r28_rd_addr_max = 28'b0; // max read address
reg [27:0] r28_start_addr = 28'b0; // operation start address

assign LED = w8_uart_rx_data[7:4];

always @(posedge ui_clk) begin: rx_state_machine
	if (w_uart_rx_done) begin
		r128_2_rx_buff[r1_rx_word_index][r4_rx_byte_index*8 +: 8] <= w8_uart_rx_data;
		r4_rx_byte_index <= r4_rx_byte_index - 1; // keep overflowing 16 byte counter
		if (r4_rx_byte_index == 4'b0000/*4'b1111*/) begin
			r1_rx_word_index <= ~r1_rx_word_index;
		end
	end
	r_uart_rx_en <= 1'b1;
	RGBLED1[0] <= r4_rx_byte_index[0]; // blue 1 toggles with each byte received
	RGBLED1[2] <= r1_rx_word_index; // red 1 toggles with each 128-bit word received
	r1_rx_word_index_delay <= r1_rx_word_index; // delay signal going into other process to avoid metastability/timing issues
end

always @(posedge ui_clk) begin: uart_state_machine
case (r3_uart_state)
	'b000: begin // TAKE DATA FROM RX BUFFER, decide next state based on buffer contents
		r_uart_tx_send_en <= 1'b0;
		app_en <= 1'b0;
		r1_rx_word_index_prev_read <= r1_rx_word_index_delay; 	// check for signal edge even if edge happens
																// in another state by storing previous value
		if (r1_rx_word_index_prev_read ^ r1_rx_word_index_delay) begin // index_delay has toggled since last checked
			RGBLED0[1] <= ~RGBLED0[1]; // green 0 toggles with each 128-bit word received -- should be about in sync with red 1
			if ((r128_2_rx_buff[~r1_rx_word_index] == 128'h66666666_66666666_66666666_66666666)
					& (app_addr > r28_start_addr)) begin // 16 0x66 bytes signify READ command
				r3_uart_state <= 3'b011; // go to read/tx loop
			end else if (r128_2_rx_buff[~r1_rx_word_index][127:64] == 64'haaaaaaaa_aaaaaaaa) begin
				// 25 0xaa bytes signify SET ADDR command
				r28_start_addr <= r128_2_rx_buff[~r1_rx_word_index][27:0]; // set start addr for RD op
				app_addr <= r128_2_rx_buff[~r1_rx_word_index][27:0]; // set address of next write
			end else begin // all other data is written to DDR
				app_cmd <= 0;
				app_wdf_wren <= 1;
				app_wdf_data <= r128_2_rx_buff[~r1_rx_word_index]; // wr data
				r3_uart_state <= 3'b001; // DDR WR
			end
		end
	end
	'b001: begin // END WR CMD, SIGNAL ENABLE
		if (app_wdf_rdy) begin
			app_wdf_wren <= 0;
			app_en <= 1;
			r3_uart_state <= 3'b010;
		end
	end
	'b010: begin // STOP WRITE, INCREMENT ADDR, BACK TO IDLE
		if (app_rdy)
			app_en <= 0;
		if (~app_en) begin
			app_addr <= app_addr + 8;
			r_uart_tx_send_en <= 1'b1;
			r8_uart_tx_data <= 8'h8a;
			r3_uart_state <= 3'b000;
		end
	end
	'b011: begin // DDR/TX STATE MACHINE BEGIN, addr start/end setup
		r28_rd_addr_max <= app_addr - 8;
		app_addr <= r28_start_addr;
		r4_uart_byte_index <= 4'b1111;
		r3_uart_state <= 3'b100;
	end
	'b100: begin // REQUEST DATA FROM DDR BLOCK
		// (is separate state for easier looping from 'b111)
		if (app_rdy) begin
			app_cmd <= 1;
			app_en <= 1;
			r3_uart_state <= 3'b101;
		end
	end
	'b101: begin // WAIT FOR DDR DATA VALID, BUFFER RD DATA
		if (app_rd_data_valid) begin
			r128_ddr_rd_buffer <= app_rd_data;
			app_en <= 0;
			r3_uart_state <= 3'b110;
		end
	end
	'b110: begin // SETUP (NEXT) TX BYTE AND SEND ENABLE
		r8_uart_tx_data <= r128_ddr_rd_buffer[r4_uart_byte_index*8 +: 8];
		r_uart_tx_send_en <= 1;
		r3_uart_state <= 3'b111;
	end
	'b111: begin // WAIT FOR TX BYTE DONE
		r_uart_tx_send_en <= 0;
		if (w_uart_tx_byte_done) begin
			r4_uart_byte_index <= r4_uart_byte_index - 1; // always increment, no resets, overflow
			if (r4_uart_byte_index == 4'b0000) begin // rd buffer sent
				if (app_addr == r28_rd_addr_max) begin // read finished
					app_addr <= r28_start_addr;
					r3_uart_state <= 3'b000;
				end else begin // read not finished, read from next addr
					app_addr <= app_addr + 8;
					r3_uart_state <= 3'b100;
				end
			end else begin // rd buffer not sent, setup next buffer byte
				r3_uart_state <= 3'b110;
			end
		end
	end
	default: ; // should not be reached
endcase
end
wire trig_in = (r1_rx_word_index_prev_read ^ r1_rx_word_index_delay);
ila_0 ila_instance (
	.clk(ui_clk),
	.trig_in(trig_in),
	.trig_in_ack(),
	/*[127 : 0]*/	.probe0(r128_ddr_rd_buffer),
	/*[127 : 0]*/	.probe1(app_wdf_data),
	/*[0 : 0]*/		.probe2(app_en),
	/*[0 : 0]*/		.probe3(app_cmd[0]),
	/*[0 : 0]*/		.probe4(app_wdf_wren),
	/*[0 : 0]*/		.probe5(app_rd_data_valid),
	/*[0 : 0]*/		.probe6(app_rdy),
	/*[27 : 0]*/	.probe7(app_addr),
	/*[27 : 0]*/	.probe8(r28_start_addr),
	/*[127 : 0]*/	.probe9(r128_2_rx_buff[0]),
	/*[127 : 0]*/	.probe10(r128_2_rx_buff[1])
	);
	endmodule // top
