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
	localparam lp_UART_BAUDRATE = 12_000_000;
	
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
localparam nCK_PER_CLK =	2;
localparam APP_DATA_WIDTH =	2 * nCK_PER_CLK * DATA_WIDTH;
localparam APP_MASK_WIDTH =	APP_DATA_WIDTH/8;
// App signals to user || reg denotes input || wire denotes output unless otherwise stated
// Explanations taken from UG586 pg. 50 and PG150 pg. 118
reg	[ADDR_WIDTH-1:0]	app_addr;	// Address for current request, 28 wide
reg	[2:0]				app_cmd;	// Command for current request: 3'b000 = WRITE || 3'b0001 = READ
reg						app_en;		// Active high strobe for inputs: app_addr, app_cmd
wire					app_rdy;	// Active high indicates UI ready to accept commands
wire	[APP_DATA_WIDTH-1:0]	app_rd_data;		// Provides output data from read commands, 64-wide
wire							app_rd_data_end;	// Active high indicates current clock cycle is last cycle of output data on app_rd_data[]
wire							app_rd_data_valid;	// Active high indicates app_rd_data is valid
reg	[APP_DATA_WIDTH-1:0]		app_wdf_data =	64'b0;	// Data for write commands, 64-wide
reg								app_wdf_end =	1'b0;	// Active high indicates current clock cycle is last cycle of input data on app_wdf_data[]
wire	[APP_MASK_WIDTH-1:0]	app_wdf_mask = 	8'b0;	// Input; Mask for app_wdf_data[], 8-wide
wire							app_wdf_rdy;			// Indicates that write data FIFO is ready to receive data. Ready when (app_wdf_rdy & app_wdf_wren) = 1'b1
reg								app_wdf_wren;			// Active high strobe for app_wdf_data[]

wire	app_sr_req =	1'b0;	// Input; Undocumented functionality
wire	app_sr_active;			// Undocumented functionality; NOT CONNECTED
wire	app_ref_req =	1'b0;	// Input; User refresh request
wire	app_ref_ack;			// User refresh request completed; NOT CONNECTED
wire	app_zq_req = 	1'b0;	// Input; User ZQCS command request
wire	app_zq_ack;				// User ZQCS command request completed; NOT CONNECTED

wire	ui_clk;				// UI clock is one half of the DRAM clock
wire	ui_clk_sync_rst;	// Active high UI reset (despite being output?)

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

// Due to the delay inherent in the feedback loop, uart_tx requires two clk
// periods from the posedge of RX_RDY to begin a transmission. This is not a
// bug. This register is used to find the posedge of RX_RDY.
reg r_uart_tx_rdy_prev = 1'b0;

reg r_uart_rx_64_done = 1'b0; // pulses high for one clk; signals 8 bytes received to ddr block
reg [1:0] r2_rx_state = 2'b0;
reg [2:0] r3_uart_state = 3'b0; // state machine select
reg [3:0] r4_uart_byte_index = 4'b0; // counts 8 bytes to send to DDR
reg [127:0] r128_ddr_rd_buffer = 128'b0; // 64 bit (read) buffer from DDR
reg [127:0] r128_2_rx_buff [0:1];
reg [3:0] r4_rx_byte_index = 4'b0; // counts 8 bytes for 64x2-bit RX vector
reg r1_rx_word_index = 1'b0; // counts 64 bit words in 64x2-bit RX vector
reg r1_rx_word_index_prev = 1'b0;
reg r_uart_rx_64_done_prev;

reg r_rd_half_done;
reg r_wr_half_done;


reg [27:0] r28_rd_addr_max = 28'b0; // max read address
reg r_ddr_rd_req; // request read op from ddr block
reg r_ddr_data_valid; // raised by ddr block when DDR read data is valid
reg r_ddr_wr_done; // raised by ddr block when DDR write 

//assign LED[0] = (r3_uart_state == 0) ? 1 : 0;
//assign LED[1] = (r3_uart_state == 1) ? 1 : 0;
//assign LED[2] = (r3_uart_state == 5) ? 1 : 0;
//assign LED[3] = (r3_uart_state == 7) ? 1 : 0;
assign LED = app_rd_data[63:56];

always @(posedge ui_clk) begin: rx_state_machine
	if (w_uart_rx_done) begin
		r128_2_rx_buff[r1_rx_word_index][r4_rx_byte_index*8 +: 8] <= w8_uart_rx_data;
		r4_rx_byte_index <= r4_rx_byte_index + 1; // keep overflowing 16 byte counter
		if (r4_rx_byte_index == 4'b1111) begin
			r1_rx_word_index <= ~r1_rx_word_index;
			r_uart_rx_64_done <= 1;
		end
	end
	r_uart_rx_64_done <= 0;
	r_uart_rx_en <= 1'b1;
	RGBLED1[0] <= r4_rx_byte_index[0];
	RGBLED1[2] <= r1_rx_word_index;
end

always @(posedge ui_clk) begin: uart_state_machine
case (r3_uart_state)
	'b000: begin // SIGNAL WR CMD TO DDR BLOCK
		r_uart_tx_send_en <= 1'b0;
		r1_rx_word_index_prev <= r1_rx_word_index;
		if (r1_rx_word_index_prev ^ r1_rx_word_index) begin
			RGBLED0[1] <= ~RGBLED0[1];
			if ((r128_2_rx_buff[~r1_rx_word_index] == 128'h66666666_66666666_66666666_66666666)
					& (app_addr > 0)) begin // SETUP DDR RD
				//r_uart_rx_en <= 1'b0;
				r3_uart_state <= 3'b011; // TX TRANSMIT CYCLE
			end else begin // SETUP DDR WR
				app_cmd <= 0;
				app_wdf_data <= r128_2_rx_buff[~r1_rx_word_index][63:0]; // wr data 1/2
				r3_uart_state <= 3'b001; // DDR WR
			end
		end
	end
	'b001: begin // SIGNAL WR CMD TO DDR BLOCK
		if (app_wdf_rdy) begin // rise1: raise wr request to MIG FIFO 1/2
			app_wdf_wren <= 1;
		end
		if (app_wdf_rdy & app_wdf_wren) begin
			app_wdf_data <= r128_2_rx_buff[~r1_rx_word_index][127:64]; // wr data 2/2
			app_wdf_end <= 1; // indicate second half of BL8
		end
		if (app_wdf_end) begin
			app_wdf_wren <= 0;
			app_wdf_end <= 0;
			app_en <= 1;
			r3_uart_state <= 3'b010;
		end
	end
	'b010: begin // SIGNAL TO UART MASTER: 8 BYTES WRITTEN TO DDR
		app_en <= 0;
		app_wdf_wren <= 0;
		app_wdf_end <= 0;
		
		r8_uart_tx_data <= 8'h8a; // TX confirmation byte to master
		r_uart_tx_send_en <= 1'b1;
		
		r4_uart_byte_index <= 3'b0; // reset byte counter
		app_addr <= app_addr + 8; // increment ddr addr 64 bits
		r3_uart_state <= 3'b000; // rx next 64 bits
	end
	'b011: begin // BEGIN TX STATE MACHINE
		r28_rd_addr_max <= app_addr - 8;
		app_addr <= 0;
		r4_uart_byte_index <= 0;
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
	'b101: begin // WAIT FOR DDR DATA VALID
		if (app_rd_data_valid & ~r_rd_half_done) begin
			//app_en <= 0;
			r128_ddr_rd_buffer[63:0] <= app_rd_data;
			r_rd_half_done <= 1;
		end
		if (r_rd_half_done & app_rd_data_end) begin
			app_en <= 0;
			r128_ddr_rd_buffer[127:64] <= app_rd_data;
			r3_uart_state <= 3'b110;
			r_rd_half_done <= 0;
		end
	end
	'b110: begin // SETUP (NEXT) TX BYTE AND SEND
		r8_uart_tx_data <= r128_ddr_rd_buffer[r4_uart_byte_index*8 +: 8];
		r_uart_tx_send_en <= 1;
		r3_uart_state <= 3'b111;
	end
	'b111: begin // WAIT FOR TX BYTE
		r_uart_tx_send_en <= 0;
		if (w_uart_tx_byte_done) begin
			r4_uart_byte_index <= r4_uart_byte_index + 1;
			if (r4_uart_byte_index == 4'b1111) begin
				if (app_addr == r28_rd_addr_max) begin
					app_addr <= 0;
					r3_uart_state <= 'b000;
				end else begin
					app_addr <= app_addr + 8;
					r3_uart_state <= 'b100;
				end
			end else begin
				r3_uart_state <= 'b110;
			end
		end
	end
	default: ; // should not be reached
endcase
end
endmodule // top
