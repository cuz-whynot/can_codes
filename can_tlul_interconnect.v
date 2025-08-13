module can_crc16 (
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    input  wire [7:0]  data_in,
    input  wire        clear,
    output reg  [15:0] crc_out
);
    parameter POLY = 16'h1021;
    integer i;
    reg [15:0] crc_next;

    // Corrected always block structure
    always @(posedge clk or posedge rst) begin
        if (rst) begin // Asynchronous reset has the highest priority
            crc_out <= 16'hFFFF;
        end else begin // All other logic is synchronous
            if (clear) begin // Synchronous clear
                crc_out <= 16'hFFFF;
            end else if (enable) begin
                crc_next = crc_out ^ (data_in << 8);
                for (i = 0; i < 8; i = i + 1) begin
                    if (crc_next[15])
                        crc_next = (crc_next << 1) ^ POLY;
                    else
                        crc_next = (crc_next << 1);
                end
                crc_out <= crc_next;
            end
        end
    end
endmodule

module can_transmitter(
    input wire clk,
    input wire rst,
    input wire start_tx,
    input wire [10:0] id_a,
    input wire ide,
    input wire rtr,
    input wire [3:0] dlc,
    input wire [63:0] data,
    input wire ack_in,
    output reg tx,
    output reg busy,
    output reg [8:0] tx_bit_count,
    output reg [242:0] tx_shift_reg,
    output wire [15:0] crc_out
);

    reg [3:0] state;
    reg [7:0] bit_cnt;
    reg [7:0] index;
    reg [10:0] id_a_reg;
    reg [3:0] dlc_reg;
    reg [63:0] data_reg;

    reg crc_enable;
    reg crc_clear;
    reg [7:0] crc_data;

    can_crc16 crc_inst (
        .clk(clk),
        .rst(rst),
        .enable(crc_enable),
        .data_in(crc_data),
        .clear(crc_clear),
        .crc_out(crc_out)
    );

    wire data_done;
    wire data_bit;
    assign data_bit = data_reg[63 - bit_cnt];
    assign data_done = (bit_cnt == (dlc_reg * 8 - 1));

    localparam IDLE=4'd0, SOF=4'd1, ID=4'd2, RTR=4'd3, IDE=4'd4, R0=4'd5,
               DLC=4'd6, DATA=4'd7, CRC=4'd8, ACK=4'd9, EOF=4'd10, DONE=4'd11;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            tx <= 1;
            busy <= 0;
            state <= IDLE;
            bit_cnt <= 0;
            tx_bit_count <= 0;
            tx_shift_reg <= 0;
            index <= 0;
            id_a_reg <= 0;
            dlc_reg <= 0;
            data_reg <= 0;
            crc_enable <= 0;
            crc_clear <= 1;
            crc_data <= 0;
        end else begin
            crc_enable <= 0;
            if (state != IDLE && state != DONE)
                tx_shift_reg <= {tx_shift_reg[241:0], tx};

            case (state)
                IDLE: begin
                    tx <= 1;
                    busy <= 0;
                    crc_clear <= 1;
                    if (start_tx) begin
                        busy <= 1;
                        state <= SOF;
                        id_a_reg <= id_a;
                        dlc_reg <= dlc;
                        data_reg <= data;
                        bit_cnt <= 0;
                        tx_bit_count <= 0;
                        index <= 10;
                    end
                end
                SOF: begin
                    tx <= 0;
                    tx_bit_count <= tx_bit_count + 1;
                    state <= ID;
                    crc_clear <= 0;
                end
                ID: begin
                    tx <= id_a_reg[index];
                    crc_data <= {7'b0, id_a_reg[index]};
                    crc_enable <= 1;
                    tx_bit_count <= tx_bit_count + 1;
                    if (index == 0) state <= RTR;
                    else index <= index - 1;
                end
                RTR: begin
                    tx <= rtr;
                    crc_data <= {7'b0, rtr};
                    crc_enable <= 1;
                    tx_bit_count <= tx_bit_count + 1;
                    state <= IDE;
                end
                IDE: begin
                    tx <= ide;
                    crc_data <= {7'b0, ide};
                    crc_enable <= 1;
                    tx_bit_count <= tx_bit_count + 1;
                    state <= R0;
                end
                R0: begin
                    tx <= 0;
                    crc_data <= 8'h00;
                    crc_enable <= 1;
                    tx_bit_count <= tx_bit_count + 1;
                    state <= DLC;
                    index <= 3;
                end
                DLC: begin
                    tx <= dlc_reg[index];
                    crc_data <= {7'b0, dlc_reg[index]};
                    crc_enable <= 1;
                    tx_bit_count <= tx_bit_count + 1;
                    if (index == 0)
                        state <= DATA;
                    else
                        index <= index - 1;
                end
                DATA: begin
                    tx <= data_bit;
                    if (bit_cnt[2:0] == 3'b000) begin
                        crc_data <= data_reg[63 - bit_cnt -: 8];
                        crc_enable <= 1;
                    end
                    tx_bit_count <= tx_bit_count + 1;
                    if (!data_done)
                        bit_cnt <= bit_cnt + 1;
                    else begin
                        state <= CRC;
                        bit_cnt <= 15;
                    end
                end
                CRC: begin
                    tx <= crc_out[bit_cnt];
                    tx_bit_count <= tx_bit_count + 1;
                    if (bit_cnt == 0)
                        state <= ACK;
                    else
                        bit_cnt <= bit_cnt - 1;
                end
                ACK: begin
                    tx <= 1;
                    tx_bit_count <= tx_bit_count + 1;
                    if (ack_in == 0) state <= EOF;
                    else state <= DONE;
                end
                EOF: begin
                    tx <= 1;
                    tx_bit_count <= tx_bit_count + 1;
                    bit_cnt <= bit_cnt + 1;
                    if (bit_cnt == 6) state <= DONE;
                end
                DONE: begin
                    tx <= 1;
                    busy <= 0;
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule



module can_decoder (
    input wire clk,
    input wire rst,
    input wire rx_bit,
    input wire sample_point,
    input wire [10:0] own_id,

    output reg [10:0] id_a,
    output reg ide,
    output reg rtr,
    output reg [3:0] dlc,
    output reg [63:0] data,
    output reg ack_drive,
    output reg valid,
    output reg crc_valid
);

    reg [7:0] bit_cnt;
    reg [63:0] data_shift;
    reg [10:0] id_shift;
    reg [3:0] state;
    reg accept_msg;
    reg crc_enable;
    reg crc_clear;
    reg [7:0] crc_data;

    wire [15:0] crc_out;

    can_crc16 crc_inst (
        .clk(clk),
        .rst(rst),
        .enable(crc_enable),
        .data_in(crc_data),
        .clear(crc_clear),
        .crc_out(crc_out)
    );

    localparam IDLE=4'd0, ID_A=4'd1, RTR=4'd2, IDE=4'd3, R0=4'd4, DLC=4'd5,
               DATA=4'd6, CRC=4'd7, ACK_SLOT=4'd8, EOF=4'd9, DONE=4'd10;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            bit_cnt <= 0;
            id_a <= 0;
            id_shift <= 0;
            rtr <= 0;
            ide <= 0;
            dlc <= 0;
            data <= 0;
            ack_drive <= 0;
            data_shift <= 0;
            valid <= 0;
            accept_msg <= 0;
            crc_enable <= 0;
            crc_clear <= 1;
            crc_data <= 0;
            crc_valid <= 0;
        end else if (sample_point) begin
            crc_enable <= 0;

            case (state)
                IDLE: begin
                    ack_drive <= 0;
                    crc_clear <= 1;
                    if (rx_bit == 0) begin
                        valid <= 0;
                        crc_valid <= 0;
                        bit_cnt <= 0;
                        id_shift <= 0;
                        state <= ID_A;
                        crc_clear <= 0;
                    end
                end
                ID_A: begin
                    id_shift <= {id_shift[9:0], rx_bit};
                    crc_data <= {7'b0, rx_bit};
                    crc_enable <= 1;
                    bit_cnt <= bit_cnt + 1;
                    if (bit_cnt == 10) begin
                        id_a <= {id_shift[9:0], rx_bit};
                        accept_msg <= ({id_shift[9:0], rx_bit} == own_id);
                        bit_cnt <= 0;
                        state <= RTR;
                    end
                end
                RTR: begin
                    rtr <= rx_bit;
                    crc_data <= {7'b0, rx_bit};
                    crc_enable <= 1;
                    state <= IDE;
                end
                IDE: begin
                    ide <= rx_bit;
                    crc_data <= {7'b0, rx_bit};
                    crc_enable <= 1;
                    state <= R0;
                end
                R0: begin
                    crc_data <= 8'h00;
                    crc_enable <= 1;
                    state <= DLC;
                    bit_cnt <= 0;
                end
                DLC: begin
                    dlc[3 - bit_cnt] <= rx_bit;
                    crc_data <= {7'b0, rx_bit};
                    crc_enable <= 1;
                    bit_cnt <= bit_cnt + 1;
                    if (bit_cnt == 3) begin
                        bit_cnt <= 0;
                        data_shift <= 0;
                        state <= DATA;
                    end
                end
                DATA: begin
                    if (accept_msg)
                        data_shift[63 - bit_cnt] <= rx_bit;
                    if (bit_cnt[2:0] == 3'b000) begin
                        crc_data <= data_shift[63 - bit_cnt -: 8];
                        crc_enable <= 1;
                    end
                    bit_cnt <= bit_cnt + 1;
                    if (bit_cnt == (8 * dlc - 1)) begin
                        if (accept_msg) begin
                            data <= data_shift;
                            valid <= 1;
                        end
                        bit_cnt <= 15;
                        state <= CRC;
                    end
                end
                CRC: begin
                    if (crc_out[bit_cnt] == rx_bit)
                        crc_valid <= 1;
                    else
                        crc_valid <= 0;
                    if (bit_cnt == 0)
                        state <= ACK_SLOT;
                    else
                        bit_cnt <= bit_cnt - 1;
                end
                ACK_SLOT: begin
                    ack_drive <= 1;
                    state <= EOF;
                end
                EOF: begin
                    ack_drive <= 0;
                    bit_cnt <= bit_cnt + 1;
                    if (bit_cnt == 6) begin
                        bit_cnt <= 0;
                        state <= DONE;
                    end
                end
                DONE: begin
                    ack_drive <= 0;
                    valid <= 0;        // Clear valid flag when message processing is complete
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule


module can_node (
    input  wire clk,
    input  wire rst,
    input  wire start_tx,
    input  wire [10:0] id_a,
    input  wire [3:0] dlc,
    input  wire [63:0] data_tx,
    input  wire rx_bit,
    input  wire ack_in,
    input  wire [10:0] own_id,

    output wire tx,
    output wire busy,
    output wire [63:0] data_rx,
    output wire ack_drive,
    output wire valid
);
    wire [10:0] id_a_rx;
    wire ide_rx, rtr_rx;
    wire [3:0] dlc_rx;

    can_transmitter u_tx (
        .clk(clk), .rst(rst), .start_tx(start_tx), .id_a(id_a), .ide(1'b0), .rtr(1'b0),
        .dlc(dlc), .data(data_tx), .ack_in(ack_in), .tx(tx), .busy(busy),
        .tx_bit_count(), .tx_shift_reg()
    );

    can_decoder u_rx (
        .clk(clk), .rst(rst), .rx_bit(rx_bit), .sample_point(1'b1),
        .own_id(own_id),
        .id_a(id_a_rx), .ide(ide_rx), .rtr(rtr_rx), .dlc(dlc_rx), .data(data_rx),
        .ack_drive(ack_drive), .valid(valid)
    );
endmodule

module tlul_can_interconnect #(
    parameter TL_ADDR_WIDTH = 64,
    parameter TL_DATA_WIDTH = 64
)(
    input  wire                     clk,
    input  wire                     rst,

    // TL-UL A Channel
    input  wire                     enable,
    output reg                      a_ready,
    input  wire [2:0]               a_opcode,
    input  wire [TL_ADDR_WIDTH-1:0] a_address,
    input  wire [TL_DATA_WIDTH-1:0] a_data,

    // TL-UL D Channel
    output wire                      valid,
    output reg  [63:0]              d_data,

    // CAN interface
    output reg                      can_start,
    //output wire [7:0]               can_cmd,
    output wire [10:0]              can_addr,
    output wire [63:0]              can_data,
    input  wire                     can_done,
    //input  wire                     data_end,
    input  wire [63:0]              can_resp,
    input  wire                     can_rx_valid,
    output wire                     response_ready
  	//output reg                      re,
  	//output reg                      we
);

    //reg [7:0]   cmd_reg;
    reg [63:0]  addr_reg;
    reg [63:0]  data_reg;
    reg [63:0]  data_buffer;
    reg         transaction_in_progress;
    //wire         response_ready;

    //assign can_cmd  = cmd_reg;
    assign can_addr = addr_reg[10:0];
    assign can_data = data_reg[63:0];
    // assign response_ready = (data_buffer != 64'h0); // Indicates buffered data is available for read
    assign response_ready = (can_rx_valid);

    // Consider using a register to hold the ready state
    /*reg response_ready_reg;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            response_ready_reg <= 1'b0;
        end else if (can_rx_valid) begin
            response_ready_reg <= 1'b1;
        end else //if (a_opcode == 3'd4 && transaction_in_progress) 
        begin
            response_ready_reg <= 1'b0; // Clear after read
        end
    end
assign response_ready = response_ready_reg; */

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            //cmd_reg <= 8'h00;
            addr_reg <= 64'h0;
            data_reg <= 64'h0;
            can_start <= 1'b0;
            transaction_in_progress <= 1'b0;
            //response_ready <= 1'b0;
            a_ready <= 1'b0;
            //we <= 1'b0;
           // re <= 1'b0;
        end else begin
            can_start <= 1'b0;
            a_ready <= !transaction_in_progress;
            //we <= 1'b0;
            //re <= 1'b0;

            if (enable && !transaction_in_progress) begin
                if (a_opcode == 3'd0) begin // write
                    addr_reg <= a_address;
                    data_reg <= a_data;
                    can_start <= 1'b1;
                    transaction_in_progress <= 1'b1;
				end	               
                end 
            if(!transaction_in_progress) begin
                if (a_opcode == 3'd4) begin // read
                    addr_reg <= a_address;
                    data_reg <= 64'd0;
                    //can_start <= 1'b1; //do something abt this
                    d_data <= data_buffer;
                    transaction_in_progress <= 1'b1;
                end
            end 
           
          if (can_done && a_opcode == 3'd0 ) begin
               transaction_in_progress <= 1'b0;
//                valid <= 1'b1;
                		//response_ready <= 1'b1;
                		//re<=1'b1;
           end
          if (can_rx_valid && a_opcode == 3'd4) begin
              transaction_in_progress <= 1'b0;
              //response_ready <= 1'b1;
              //re<=1'b1;
           end

        end
    end

always@(posedge clk) begin
//    data_buffer <= (can_rx_valid)? can_resp:0;
      data_buffer <= can_resp;
end
assign valid = can_done;
//assign data_buffer = (can_rx_valid)? can_resp:0;  //check variables
//assign response_ready = (can_rx_valid)? 1:0;
 
endmodule

module top_tlul_can (
    input  wire         clk,
    input  wire         rst,
    input  wire         enable,
    input  wire [2:0]   a_opcode,
    input  wire [63:0]  a_address,
    input  wire [63:0]  a_data,
    output wire         a_ready,
    output wire         valid,
    output wire [63:0]  d_data,

    output wire         can_tx,
    output wire         response_ready,
    input wire          can_rx
);

    wire        can_start;
    wire [10:0] can_addr;
    wire [63:0] can_data;
    wire        can_done;
    wire [63:0] can_resp;
   // wire [10:0] id_rx;
    wire        busy;
    wire        can_rx_valid;  // This should be connected to can_node.valid

    
    // Local node ID
    parameter NODE_ID = 11'h123;

    tlul_can_interconnect interconnect_inst (
        .clk(clk),
        .rst(rst),
        .enable(enable),
        .a_ready(a_ready),
        .a_opcode(a_opcode),
        .a_address(a_address),
        .a_data(a_data),
        .valid(valid),
        .d_data(d_data),
        .can_start(can_start),
        .can_addr(can_addr),
        .can_data(can_data),
        .can_done(can_done),
        .can_resp(can_resp),
        .can_rx_valid(can_rx_valid),  // Now properly connected
        .response_ready(response_ready)
    );

    can_node u_can_node (
        .clk(clk),
        .rst(rst),
        .start_tx(can_start),
        .id_a(can_addr),
        .dlc(4'h8),
        .data_tx(can_data),
        .rx_bit(can_rx),
        .ack_in(1'b0),
        .own_id(NODE_ID),
        .tx(can_tx),
        .busy(busy),
        .data_rx(can_resp),
        .ack_drive(can_done),
        .valid(can_rx_valid)  // This is the key connection!
    );

endmodule

module can_test_node #(
    parameter NODE_ID = 11'h456
)(
    input wire clk,
    input wire rst,
    
    //Control interface for tlul
    input  wire         enable,
    input  wire [2:0]   a_opcode,
    input  wire [63:0]  a_address,
    input  wire [63:0]  a_data,
    output wire         a_ready,
    output wire         valid,
    output wire [63:0]  d_data,

    // Control interface of testing node 
    input wire start_tx,
    input wire [10:0] id_tx,
    input wire [63:0] data_tx,
    
    // Status outputs
    output wire busy,
    output wire response_ready,
    //output wire [10:0] id_rx,
    output wire [63:0] data_rx,
    output wire valid_rx
);
    // CAN bus connection
    wire can_rx;
    wire can_tx;
    wire ack_drive;

    // CAN bus signals
    wire can_tx_main, can_tx_test;
    wire can_bus;
    
    // CAN bus (wired-AND for dominant/recessive logic)
    assign can_bus = can_tx_main & can_tx_test;

    top_tlul_can  dut (
    .clk(clk),
    .rst(rst),
    .enable(enable),
    .a_opcode(a_opcode),
    .a_address(a_address),
    .a_data(a_data),
    .a_ready(a_ready),
    .valid(valid),
    .d_data(d_data),
    .can_tx(can_tx_main),
    .response_ready(response_ready),
    .can_rx(can_bus)
);

    // CAN node instantiation
    can_node u_test_can_node (
        .clk(clk),
        .rst(rst),
        .start_tx(start_tx),
        .id_a(id_tx),
        .dlc(4'h8),
        .data_tx(data_tx),
        .rx_bit(can_bus),
        .ack_in(1'b0),
        .own_id(NODE_ID), //node id of test node is already given to it
        .tx(can_tx_test),
        .busy(busy),
       // .id_a_rx(id_rx),
        .data_rx(data_rx),
        .ack_drive(ack_drive),
        .valid(valid_rx)
    );

endmodule








/*******************************************************************************************************************************************

This top-level module sets up signals for the slave instance of the TileLink UL protocol. It will work in the low speed domain with 
peripherals like GPIO and Flash.

*******************************************************************************************************************************************/

 
module tilelink_ul_slave_top #( 

	//////////////////////////////////////////////////////////////////
	////////////////////// Core interface widths /////////////////////
	//////////////////////////////////////////////////////////////////
	
	parameter TL_ADDR_WIDTH     = 64,            		// Address width
	parameter TL_DATA_WIDTH     = 64,            		// Data width
	parameter TL_STRB_WIDTH     = TL_DATA_WIDTH / 8, 	// Byte mask width/Byte strobe. Each bit represents one byte of the data.

	
	//////////////////////////////////////////////////////////////////
	//////////////////// TileLink metadata widths ////////////////////
	//////////////////////////////////////////////////////////////////
	
	// Check again! Bigger the source width, more the number of active transactions.
	parameter TL_SOURCE_WIDTH   = 3,			 // Tags each request with a unique ID. The same ID must appear in the corresponding response.
	parameter TL_SINK_WIDTH     = 3,			 // Tags each response with an ID that matches that of the request.
	parameter TL_OPCODE_WIDTH   = 3,			 // Opcode width for instructions
	parameter TL_PARAM_WIDTH    = 3,             // Currently reserved for future performance hints and must be 0 
	parameter TL_SIZE_WIDTH     = 8,             // Width of size field, value of which determines data beat size in bytes as 2^size.
	

	// Define opcodes for channels
	// A Channel Opcodes
	parameter PUT_FULL_DATA_A     = 3'd0,
	parameter PUT_PARTIAL_DATA_A  = 3'd1,
	parameter ARITHMETIC_DATA_A   = 3'd2,
	parameter LOGICAL_DATA_A      = 3'd3,
	parameter GET_A               = 3'd4,
	parameter INTENT_A            = 3'd5,
	parameter ACQUIRE_BLOCK_A     = 3'd6,
	parameter ACQUIRE_PERM_A      = 3'd7,

	// D Channel Opcodes
	parameter ACCESS_ACK_D      = 3'd0,
	parameter ACCESS_ACK_DATA_D = 3'd1,
	parameter HINT_ACK_D        = 3'd2,
	parameter GRANT_D           = 3'd4,
	parameter GRANT_DATA_D      = 3'd5,
	parameter RELEASE_ACK_D     = 3'd6,
	
	// Slave FSM States
	parameter REQUEST 			 = 2'd1,
	parameter RESPONSE   		 = 2'd2,

	// Memory parameters
	parameter MEM_BASE_ADDR 	  = 64'h0000_0000_0000_0000, // Base address for memory
	parameter DEPTH           = 512,                      // Memory depth (number of entries)

    //CAN parameters
    parameter NODE_ID = 11'h456
)(
	input  wire                              clk,
	input  wire                              rst,

	// A Channel: Received from MASTER
	output wire                              a_ready,		// Slave sends a_ready to Master to indicate that it is ready to accept data.
	input  wire                              a_valid, 		// Asserted to indicate valid instruction
	input  wire [TL_OPCODE_WIDTH-1:0]        a_opcode,		// Opcode for instruction
	input  wire [TL_PARAM_WIDTH-1:0]         a_param,		// Reserved, always 0.
	input  wire [TL_ADDR_WIDTH-1:0]          a_address,	    // Address 
	input  wire [TL_SIZE_WIDTH-1:0]          a_size,		// Width of full data sent in one go = 2^size. For TLUL, size = Data Width of Channel.
	input  wire [TL_STRB_WIDTH-1:0]          a_mask,		// Bit masking
	input  wire [TL_DATA_WIDTH-1:0]          a_data,		// Incoming data
	input  wire [TL_SOURCE_WIDTH-1:0]        a_source,		// Transaction ID

	// D Channel Sent to MASTER
	output reg 									     d_valid,
	input  wire                              d_ready, 		// Master ends d_ready to Slave to indicate that it is ready to accept data.
	output reg [TL_OPCODE_WIDTH-1:0]        d_opcode,
	output reg [TL_PARAM_WIDTH-1:0]         d_param,
	output reg [TL_SIZE_WIDTH-1:0]          d_size,
	output reg [TL_SINK_WIDTH-1:0]          d_sink,
	output reg [TL_SOURCE_WIDTH-1:0]        d_source,	
	output reg [TL_DATA_WIDTH-1:0]          d_data,
	output reg                              d_error,

    //Test connections for CAN module 
    input wire start_tx,
    input wire [10:0] id_tx,
    input wire [63:0] data_tx,
    output wire busy,
    output wire [63:0] data_rx,
    output wire response_ready,
    output wire valid_rx
);

	// State variable for slave FSM
	reg [1:0] slave_state;

	// State flags
	wire in_idle;
	wire in_request;
	wire in_response;
	wire in_cleanup;
	
	// Memory Flags
	
	reg  [TL_ADDR_WIDTH-1:0] waddr;
	reg              	   wen,ren;
	reg  [TL_DATA_WIDTH-1:0] wdata;
	reg  [TL_ADDR_WIDTH-1:0] raddr;
	wire [TL_DATA_WIDTH-1:0] rdata;
	
	
	// For loop variables
	integer i,j;

	
	// Registers for A Channel (Slave side input)
	reg                             a_ready_reg;
	reg                             a_valid_reg;
	reg [TL_OPCODE_WIDTH-1:0]       a_opcode_reg;
	reg [TL_PARAM_WIDTH-1:0]        a_param_reg;
	reg [TL_ADDR_WIDTH-1:0]         a_address_reg;
	reg [TL_SIZE_WIDTH-1:0]         a_size_reg;
	reg [TL_STRB_WIDTH-1:0]         a_mask_reg;
	reg [TL_DATA_WIDTH-1:0]         a_data_reg;
	reg [TL_SOURCE_WIDTH-1:0]       a_source_reg;
	
	wire check;
	


	
	reg response_pending;


	wire mem_acc_done;
	wire mem_write_done;

	reg response_done;
	
	
	// Wait signal
	
	reg wait_flag;
	reg mem_enable;
	assign check = a_valid | wait_flag;

	// Enabled during write or read operations
	wire tlul_can_bridge_enable;
	assign tlul_can_bridge_enable = ren | wen;

	// For Interconnect

	wire bridge_output_valid;

	assign mem_acc_done = bridge_output_valid; // Memory access done signal from the memory block
	assign mem_write_done = bridge_output_valid; // Memory write done signal from the memory block

	/////////////////////////////////////////////////////////////
	//////////// 				  FSM BLOCK    	 	 ////////////
	/////////////////////////////////////////////////////////////	

	// Assign flags based on state
	assign in_request  = (slave_state == REQUEST);
	assign in_response = (slave_state == RESPONSE);
	
	// assign response_done = d_ready & response_pending;
	
    // assign raddr = in_request      ? a_address :
    //                wait_flag   ? a_address_reg :
    //                             64'd0;
       
	
	// State machine

	always @(posedge clk or posedge rst) begin
		if (rst) begin
			slave_state <= REQUEST;
		end
		else begin
			case (slave_state)

				REQUEST: begin
				// wait_flag <= 0;
				if (a_valid)
					slave_state <= RESPONSE;
				else
					slave_state <= REQUEST;
				end

				RESPONSE: begin
				    if (d_ready & response_pending) begin
				        slave_state <= REQUEST; 
						// wait_flag <= 0;
				    end
                                    else begin
						slave_state <= RESPONSE;
						// wait_flag <= 1;
					end
				end

				default: slave_state <= REQUEST;

			endcase
		end
	end 
					// else if (!d_ready & d_ready_reg) begin // Means that response is done



	
	/////////////////////////////////////////////////////////////
	//////////// 			   DATAPATH        	 	 ////////////
	/////////////////////////////////////////////////////////////		
	
	assign a_ready =  in_request;

	
	
	// Slave response
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset logic for any control registers or outputs
            wen   <= 1'b0;
            waddr <= {TL_ADDR_WIDTH{1'b0}};
            wdata <= {TL_DATA_WIDTH{1'b0}};
			response_pending <= 0;
			mem_enable <= 0;
			response_done <= 0;
			ren   <= 1'b0;
            // Add others as needed
        end else if (in_response) begin
            case (a_opcode_reg)
                PUT_FULL_DATA_A: begin
                    // Full memory write

					// Raise write enable and set address/data only for 1 cycle.
					if (!mem_enable & !wen) begin
						wen   <= 1'b1;
						waddr <= a_address_reg;
						wdata <= a_data_reg;
						mem_enable <= 1'b1; // Enable memory write
						response_pending <= 1'b0; // Reset response pending flag
					end else begin
						wen   <= 1'b0;
						waddr <= {TL_ADDR_WIDTH{1'b0}};
						wdata <= {TL_DATA_WIDTH{1'b0}};
						mem_enable <= mem_enable;
					end
            
                    // Latch response_pending when mem_write_done is high
                                        if (mem_write_done && !response_pending) begin
                                               response_pending <= 1'b1;

                                               d_valid   <= 1'b1;
                        		       d_opcode  <= ACCESS_ACK_D; 
                        		       d_param   <= {TL_PARAM_WIDTH{1'b0}};
                                               d_size    <= a_size_reg;
                                               d_sink    <= {TL_SINK_WIDTH{1'b0}};
                                               d_source  <= a_source_reg;
                                               d_data    <= 0; 
                                               d_error   <= 1'b0;
                                        end else if (response_pending) begin
                                            if(!d_ready) begin
							d_valid   <= 1'b1;
							d_opcode  <= ACCESS_ACK_D; 
							d_param   <= {TL_PARAM_WIDTH{1'b0}};
							d_size    <= a_size_reg;
							d_sink    <= {TL_SINK_WIDTH{1'b0}};
							d_source  <= a_source_reg;
							d_data    <= 0; 
							d_error   <= 1'b0;
						end
						else begin
                                                        response_pending <= 1'b0;
							mem_enable <= 0;
							d_valid   <= 1'b0;
							d_opcode  <= 0;
							d_param   <= {TL_PARAM_WIDTH{1'b0}};
							d_size    <= 0;
							d_sink    <= {TL_SINK_WIDTH{1'b0}};
							d_source  <= 0;
							d_data    <= {TL_DATA_WIDTH{1'b0}};
							d_error   <= 0;							
                        end						
					end
					else begin
						d_valid   <= 1'b0;
						d_opcode  <= 0;
						d_param   <= {TL_PARAM_WIDTH{1'b0}};
						d_size    <= 0;
						d_sink    <= {TL_SINK_WIDTH{1'b0}};
						d_source  <= 0;
						d_data    <= {TL_DATA_WIDTH{1'b0}};
						d_error   <= 0;
					end
					
					ren <= 1'b0;
                end

                PUT_PARTIAL_DATA_A: begin
                    // Partial write example (you may use mask  to gate bytes)

					// Raise write enable and set address/data only for 1 cycle.
					if (!mem_enable & !wen) begin
						wen   <= 1'b1;
						waddr <= a_address_reg;
						
                        for (i = 0; i < TL_STRB_WIDTH; i=i+1)
                            for (j = 0; j < TL_STRB_WIDTH; j=j+1) 
                                wdata[j + i*8] <= a_data_reg [j + i*8] & a_mask_reg [i];


						mem_enable <= 1'b1; // Enable memory write
						response_pending <= 1'b0; // Reset response pending flag
					end else begin
						wen   <= 1'b0;
						waddr <= {TL_ADDR_WIDTH{1'b0}};
						wdata <= {TL_DATA_WIDTH{1'b0}};
						mem_enable <= mem_enable;
					end
            
                    // Latch response_pending when mem_write_done is high
                    if (mem_write_done && !response_pending) begin
                        response_pending <= 1'b1;

                        d_valid   <= 1'b1;
                        d_opcode  <= ACCESS_ACK_D; 
                        d_param   <= {TL_PARAM_WIDTH{1'b0}};
                        d_size    <= a_size_reg;
                        d_sink    <= {TL_SINK_WIDTH{1'b0}};
                        d_source  <= a_source_reg;
                        d_data    <= 0; 
                        d_error   <= 1'b0;
                    end else if (response_pending) begin
                        if(!d_ready) begin
							d_valid   <= 1'b1;
							d_opcode  <= ACCESS_ACK_D; 
							d_param   <= {TL_PARAM_WIDTH{1'b0}};
							d_size    <= a_size_reg;
							d_sink    <= {TL_SINK_WIDTH{1'b0}};
							d_source  <= a_source_reg;
							d_data    <= 0; 
							d_error   <= 1'b0;
						end
						else begin
                            response_pending <= 1'b0;
							mem_enable <= 0;
							d_valid   <= 1'b0;
							d_opcode  <= 0;
							d_param   <= {TL_PARAM_WIDTH{1'b0}};
							d_size    <= 0;
							d_sink    <= {TL_SINK_WIDTH{1'b0}};
							d_source  <= 0;
							d_data    <= {TL_DATA_WIDTH{1'b0}};
							d_error   <= 0;							
                        end						
					end
					else begin
						d_valid   <= 1'b0;
						d_opcode  <= 0;
						d_param   <= {TL_PARAM_WIDTH{1'b0}};
						d_size    <= 0;
						d_sink    <= {TL_SINK_WIDTH{1'b0}};
						d_source  <= 0;
						d_data    <= {TL_DATA_WIDTH{1'b0}};
						d_error   <= 0;
					end

					ren <= 1'b0;

                end

                GET_A: begin
                    // Read only - no write enable
					// Raise write enable and set address/data only for 1 cycle.

					if (!mem_enable & !ren) begin
						ren   <= 1'b1;
						raddr <= a_address_reg;
						wdata <= {TL_DATA_WIDTH{1'b0}}; // No write data for read
						mem_enable <= 1'b1; 			// Enable memory write
						response_pending <= 1'b0; 		// Reset response pending flag
					end else begin
						ren   <= 1'b0;
						raddr <= {TL_ADDR_WIDTH{1'b0}};
						wdata <= {TL_DATA_WIDTH{1'b0}};
						mem_enable <= mem_enable;
					end
            
                    // Latch response_pending when mem_write_done is high
                    if (mem_acc_done && !response_pending) begin
                        response_pending <= 1'b1;

                        d_valid   <= 1'b1;
                        d_opcode  <= ACCESS_ACK_DATA_D; 
                        d_param   <= {TL_PARAM_WIDTH{1'b0}};
                        d_size    <= a_size_reg;
                        d_sink    <= {TL_SINK_WIDTH{1'b0}};
                        d_source  <= a_source_reg;
                        d_data    <= rdata; 
                        d_error   <= 1'b0;
                    end else if (response_pending) begin
                        if(!d_ready) begin
							d_valid   <= 1'b1;
							d_opcode  <= ACCESS_ACK_DATA_D; 
							d_param   <= {TL_PARAM_WIDTH{1'b0}};
							d_size    <= a_size_reg;
							d_sink    <= {TL_SINK_WIDTH{1'b0}};
							d_source  <= a_source_reg;
							d_data    <= d_data; 
							d_error   <= 1'b0;
						end
						else begin
                            response_pending <= 1'b0;
							mem_enable <= 0;
							d_valid   <= 1'b0;
							d_opcode  <= 0;
							d_param   <= {TL_PARAM_WIDTH{1'b0}};
							d_size    <= 0;
							d_sink    <= {TL_SINK_WIDTH{1'b0}};
							d_source  <= 0;
							d_data    <= {TL_DATA_WIDTH{1'b0}};
							d_error   <= 0;							
                        end						
					end
					else begin
						d_valid   <= 1'b0;
						d_opcode  <= 0;
						d_param   <= {TL_PARAM_WIDTH{1'b0}};
						d_size    <= 0;
						d_sink    <= {TL_SINK_WIDTH{1'b0}};
						d_source  <= 0;
						d_data    <= {TL_DATA_WIDTH{1'b0}};
						d_error   <= 0;
					end

					wen <= 1'b0; // Deassert write when not responding
                end

                default: begin
                    wen <= 1'b0;
					ren <= 1'b0; // Deassert read when not responding
					response_done <= 0;
					mem_enable <= 0; // Disable memory access

					// Default response for unsupported opcodes
                    d_valid   <= 1'b0;                                                                                                      
                    d_opcode  <= 0;                                                                                              
                    d_param   <= {TL_PARAM_WIDTH{1'b0}};     // Reserved at 0                                                               
                    d_size    <= 0 ;                 // Same as from MASTER                                                            
                    d_sink    <= {TL_SINK_WIDTH{1'b0}};      // Ignored                                                                     
                    d_source  <= 0 ;               // Same as sent by MASTER                                                         
                    d_data    <= {TL_DATA_WIDTH{1'b0}};      // Ignored. Dataless response                                                  
                    d_error   <= 1'b0;                       // No error. Change later! Add error logic from memory (failed mem access etc).                    
                end
            endcase

        end else begin
            wen <= 1'b0; // Deassert write when not responding
			ren <= 1'b0; // Deassert read when not responding
			response_done <= 0;
			mem_enable <= 0; // Disable memory access
            d_valid   <= 1'b0;                                                                                                      
            d_opcode  <= 0;                                                                                              
            d_param   <= {TL_PARAM_WIDTH{1'b0}};     // Reserved at 0                                                               
            d_size    <= 0 ;                 // Same as from MASTER                                                            
            d_sink    <= {TL_SINK_WIDTH{1'b0}};      // Ignored                                                                     
            d_source  <= 0 ;               // Same as sent by MASTER                                                         
            d_data    <= {TL_DATA_WIDTH{1'b0}};      // Ignored. Dataless response                                                  
            d_error   <= 1'b0;                       // No error. Change later! Add error logic from memory (failed mem access etc).             
			mem_enable <= 0;
		end
    end


	always @(posedge clk or posedge rst) begin
		if (rst) begin
			// A Channel Registers
			a_ready_reg   <= 1'b0;
			a_opcode_reg  <= {TL_OPCODE_WIDTH{1'b0}};
			a_param_reg   <= {TL_PARAM_WIDTH{1'b0}};
			a_address_reg <= {TL_ADDR_WIDTH{1'b0}};
			a_size_reg    <= {TL_SIZE_WIDTH{1'b0}};
			a_mask_reg    <= {TL_STRB_WIDTH{1'b0}};
			a_data_reg    <= {TL_DATA_WIDTH{1'b0}};
			a_source_reg  <= {TL_SOURCE_WIDTH{1'b0}};
			a_valid_reg <= 0;

			
		end
		else begin
		    // d_ready_reg <= d_ready;
			if (a_valid) begin
				a_opcode_reg  <= a_opcode;
				a_param_reg   <= a_param;
				a_address_reg <= a_address;
				a_size_reg    <= a_size;
				a_mask_reg    <= a_mask;
				a_data_reg    <= a_data;
				a_source_reg  <= a_source;
				a_valid_reg <= a_valid;
			end
		end
	end




   /* top_tlul_spi u_top_tlul_spi (
        .clk       (clk),
        .rst       (rst),
        .enable    (tlul_spi_bridge_enable),
      .a_opcode  (a_opcode_reg),
      .a_address (a_address_reg),
        .a_data    (wdata),
        .a_ready   (),
        .valid     (bridge_output_valid),
        .d_data    (rdata)
    );	*/

can_test_node #(
        .NODE_ID(NODE_ID)
    ) test_env (
        .clk(clk),
        .rst(rst),
        .enable(tlul_can_bridge_enable),
        .a_opcode(a_opcode_reg),
        .a_address(a_address_reg),
        .a_data(wdata),
        .a_ready(),
        .valid(bridge_output_valid),
        .d_data(rdata),

        .start_tx(start_tx),
        .id_tx(id_tx),
        .data_tx(data_tx),
        .busy(busy),
        .response_ready(response_ready),
        .data_rx(data_rx),
        .valid_rx(valid_rx)
    );

	
	
	
// memory_block #(
//     .TL_DATA_WIDTH(TL_DATA_WIDTH),      // Data width for memory
// 	.MEM_BASE_ADDR(MEM_BASE_ADDR),         // Base address for memory
//     .DEPTH(DEPTH),                		// Memory depth (number of entries)
//     .TL_ADDR_WIDTH(TL_ADDR_WIDTH),       // Address width
// 	.LATENCY(3)                          // Latency for memory access
// ) memory_inst (
//     .clk(clk),                  // Clock input
//     .rst(rst),                // Reset input
// 	.ren(ren),
//     .waddr(waddr),              // Write address from slave logic
//     .wen(wen),                  // Write enable from slave logic
//     .wdata(wdata),              // Write data from slave logic
//     .raddr(raddr),              // Read address from slave logic
//     .rdata(rdata),              // Read data to be sent back to master
// 	.mem_write_done(mem_write_done), // Memory write done signal
// 	.mem_acc_done(mem_acc_done)      // Memory access done signal
// );      
      
      
endmodule
      
      
      

	/////////////////////////////////////////////////////////////
	//////////// 				CPU MEMORY     	 	 	 ////////////
	/////////////////////////////////////////////////////////////	
/*
module memory_block #(
    parameter TL_DATA_WIDTH = 64,
    parameter MEM_BASE_ADDR = 64'h0000_0000_0000_0000, // Slave-specific base address
    parameter DEPTH = 512,
    parameter TL_ADDR_WIDTH = $clog2(DEPTH),
	parameter LATENCY = 3
)(
    input  wire                        clk,
    input  wire                        rst,

	input  wire					      ren,
    input  wire [TL_ADDR_WIDTH-1:0]   waddr,
    input  wire                        wen,
    input  wire [TL_DATA_WIDTH-1:0]   wdata,
    input  wire [TL_ADDR_WIDTH-1:0]   raddr,
    output wire [TL_DATA_WIDTH-1:0]   rdata,
	output 							  mem_write_done, 
	output wire 					  mem_acc_done
);

    // 1. Local memory (addressed from 0 to DEPTH-1)
    reg [TL_DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // 2. Local address offset computation
    wire [TL_ADDR_WIDTH-1:0] local_waddr = waddr - MEM_BASE_ADDR[TL_ADDR_WIDTH-1:0];
    wire [TL_ADDR_WIDTH-1:0] local_raddr = raddr - MEM_BASE_ADDR[TL_ADDR_WIDTH-1:0];

    // 3. Output register
    reg [TL_DATA_WIDTH-1:0] r_rdata;
    assign rdata = r_rdata;

	// Pipeline registers to induce latency for memory access
	reg [TL_DATA_WIDTH-1:0] rdata_pipe [0:LATENCY-1];
	reg [TL_ADDR_WIDTH-1:0] waddr_pipe [0:LATENCY-1];
	reg [TL_DATA_WIDTH-1:0] wdata_pipe [0:LATENCY-1];
	reg [TL_ADDR_WIDTH-1:0] raddr_pipe [0:LATENCY-1];
	reg ren_pipe [0:LATENCY-1];
	reg wen_pipe [0:LATENCY-1];
	reg r_mem_acc_done, r_mem_write_done;
	
	
	assign mem_write_done = r_mem_write_done;
	assign mem_acc_done = r_mem_acc_done;
	

    // 4. Memory logic
    integer i;
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < DEPTH; i = i + 1)
                mem[i] <= 0;
            r_rdata <= 0;
        end else begin

			// Update pipeline registers
			rdata_pipe[0] <= r_rdata; // First stage gets the current read data
			raddr_pipe[0] <= local_raddr; // First stage gets the current read address
			ren_pipe[0] <= ren; // First stage gets the current read enable
			wen_pipe[0] <= wen; // First stage gets the current write enable
			waddr_pipe[0] <= local_waddr; // First stage gets the current write address
			wdata_pipe[0] <= wdata; // First stage gets the current write data

			// Shift pipeline registers
			for (i = LATENCY-1; i > 0; i = i - 1) begin
				rdata_pipe[i] <= rdata_pipe[i-1];
				raddr_pipe[i] <= raddr_pipe[i-1];
				ren_pipe[i] <= ren_pipe[i-1];
				wen_pipe[i] <= wen_pipe[i-1];
				waddr_pipe[i] <= waddr_pipe[i-1];
				wdata_pipe[i] <= wdata_pipe[i-1];
			end

			// Last stage of the pipeline
			if (ren_pipe[LATENCY-1]) begin
				// Read operation
				r_rdata <= mem[raddr_pipe[LATENCY-1]];
				r_mem_acc_done <= 1'b1; // Indicate memory access is done
			end 
			else begin
				r_rdata <= 0; // Hold previous read data if not reading
				r_mem_acc_done <= 1'b0; // Indicate memory access is not done
			end
			
			if (wen_pipe[LATENCY-1]) begin
				// Write operation
				mem[waddr_pipe[LATENCY-1]] <= wdata_pipe[LATENCY-1]; // Write data to the local address
				r_mem_write_done <= 1'b1; // Indicate memory write is done
			end
			else begin
				r_mem_write_done <= 1'b0; // Indicate memory write is not done
			end

        end
    end

endmodule*/
/*******************************************************************************************************************************************

This top-level module sets up signals for the master instance of the TileLink UL protocol. It will work in the low speed domain with 
peripherals like GPIO and Flash.

*******************************************************************************************************************************************/


module tilelink_ul_master_top #( 

	//////////////////////////////////////////////////////////////////
	////////////////////// Core interface widths /////////////////////
	//////////////////////////////////////////////////////////////////
	
	parameter TL_ADDR_WIDTH     = 64,            		// Address width
	parameter TL_DATA_WIDTH     = 64,            		// Data width
	parameter TL_STRB_WIDTH     = TL_DATA_WIDTH / 8, 	// Byte mask width/Byte strobe

	//////////////////////////////////////////////////////////////////
	//////////////////// TileLink metadata widths ////////////////////
	//////////////////////////////////////////////////////////////////
	
	parameter TL_SOURCE_WIDTH   = 3,			        // Request ID
	parameter TL_SINK_WIDTH     = 3,			        // Response ID
	parameter TL_OPCODE_WIDTH   = 3,			        // Opcode width
	parameter TL_PARAM_WIDTH    = 3,                  // Reserved (0)
	parameter TL_SIZE_WIDTH     = 8,                  // log2(size in bytes)
	
	// Opcodes for A Channel
	parameter PUT_FULL_DATA_A     = 3'd0,
	parameter PUT_PARTIAL_DATA_A  = 3'd1,
	parameter ARITHMETIC_DATA_A   = 3'd2,
	parameter LOGICAL_DATA_A      = 3'd3,
	parameter GET_A               = 3'd4,
	parameter INTENT_A            = 3'd5,
	parameter ACQUIRE_BLOCK_A     = 3'd6,
	parameter ACQUIRE_PERM_A      = 3'd7,

	// Opcodes for D Channel
	parameter ACCESS_ACK_D        = 3'd0,
	parameter ACCESS_ACK_DATA_D   = 3'd1,
	parameter HINT_ACK_D          = 3'd2,
	parameter GRANT_D             = 3'd4,
	parameter GRANT_DATA_D        = 3'd5,
	parameter RELEASE_ACK_D       = 3'd6,

	// Master FSM States
	parameter REQUEST 			 = 2'd1,
	parameter RESPONSE   		 = 2'd2,
	parameter CLEANUP 			 = 2'd3,
	parameter IDLE   			 = 2'd0	

)(
	input  wire                              clk,
	input  wire                              rst,

	// Inputs for commands from testbench, defined as a_valid_in, a_opcode_in, etc.
	input  wire                              a_valid_in,		// Slave ready to accept data
	input  wire [TL_OPCODE_WIDTH-1:0]        a_opcode_in,
	input  wire [TL_PARAM_WIDTH-1:0]         a_param_in,		// Reserved, 0
	input  wire [TL_ADDR_WIDTH-1:0]          a_address_in,
	input  wire [TL_SIZE_WIDTH-1:0]          a_size_in,
	input  wire [TL_STRB_WIDTH-1:0]          a_mask_in,
	input  wire [TL_DATA_WIDTH-1:0]          a_data_in,
	input  wire [TL_SOURCE_WIDTH-1:0]        a_source_in,

	// A Channel: Sent TO SLAVE (Master drives it)
	input  wire                              a_ready,			// Slave ready to accept data
	output reg                               a_valid, 			// Master asserts to send valid request
	output reg  [TL_OPCODE_WIDTH-1:0]        a_opcode,
	output reg  [TL_PARAM_WIDTH-1:0]         a_param,			// Reserved, 0
	output reg  [TL_ADDR_WIDTH-1:0]          a_address,
	output reg  [TL_SIZE_WIDTH-1:0]          a_size,
	output reg  [TL_STRB_WIDTH-1:0]          a_mask,
	output reg  [TL_DATA_WIDTH-1:0]          a_data,
	output reg  [TL_SOURCE_WIDTH-1:0]        a_source,

	// D Channel: Received FROM SLAVE
	input  wire                              d_valid, 			// Slave responds
	output wire                              d_ready, 			// Master ready to accept
	input  wire [TL_OPCODE_WIDTH-1:0]        d_opcode,
	input  wire [TL_PARAM_WIDTH-1:0]         d_param,
	input  wire [TL_SIZE_WIDTH-1:0]          d_size,
	input  wire [TL_SINK_WIDTH-1:0]          d_sink,
	input  wire [TL_SOURCE_WIDTH-1:0]        d_source,
	input  wire [TL_DATA_WIDTH-1:0]          d_data,
	input  wire                              d_error
);

	// State variable for slave FSM
	reg [1:0] master_state, next_state;

	// State flags
	wire is_request;
	wire is_response;
	
	reg r_d_valid;

	// Registers for flopping A Channel signals
	// These registers hold the values for the A Channel signals that are sent to the slave.
	// They are useful in case the slave is not ready to accept the request in the current cycle.
	// They will be used to send the request in the next cycle when the slave is ready.
	reg                               r_a_valid; 			// Masterr_asserts to send valid request
	reg   [TL_OPCODE_WIDTH-1:0]       r_a_opcode;
	reg   [TL_PARAM_WIDTH-1:0]        r_a_param;			// Reserved; 0
	reg   [TL_ADDR_WIDTH-1:0]         r_a_address;
	reg   [TL_SIZE_WIDTH-1:0]         r_a_size;
	reg   [TL_STRB_WIDTH-1:0]         r_a_mask;
	reg   [TL_DATA_WIDTH-1:0]         r_a_data;
	reg   [TL_SOURCE_WIDTH-1:0]       r_a_source;


	///////////////////////////////////////////////////////////////
	//////////// 			 STATE MACHINE     	 	   ////////////
	///////////////////////////////////////////////////////////////
	
	assign is_request  = (master_state == REQUEST);   // High when in REQUEST state
	assign is_response = (master_state == RESPONSE);  // High when in RESPONSE state
	

	// State machine for the master
	always @(posedge clk or posedge rst) begin
		if (rst) begin
			master_state <= REQUEST; // Reset to IDLE state
		end else begin
			master_state <= next_state; // Transition to the next state
		end
	end

	// Next state logic

	always @(*) begin
		case (master_state)
				
			REQUEST: begin
				if (a_valid_in & a_ready) begin
					next_state = RESPONSE; // Transition to RESPONSE if slave response is valid
				end else begin
					next_state = REQUEST; // Stay in REQUEST state otherwise
				end
			end
			
			RESPONSE: begin
			    if (d_valid) begin
			         next_state = REQUEST;
				end else begin
					next_state = RESPONSE; // Stay in RESPONSE state otherwise
				end
			end
			
		
			default: next_state = REQUEST; // Default case to handle unexpected states
			
		endcase
	end




	/////////////////////////////////////////////////////////////
	//////////// 			   DATAPATH        	 	 ////////////
	/////////////////////////////////////////////////////////////
	
    assign d_ready = is_response;
	
	// Block for flopping A Channel signals
	// This block is used to flop the A Channel signals that are sent to the slave.
	
	always @(posedge clk or posedge rst) begin
		if (rst) begin
			// Reset logic for A Channel registers
			r_a_valid <= 1'b0; // Master is not ready to send data
			r_a_opcode <= 0;
			r_a_param  <= 0; // Reserved, 0
			r_a_address <= 0;
			r_a_size   <= 0;
			r_a_mask   <= 0;
			r_a_data   <= 0;
			r_a_source <= 0; 
		end 
		else if (a_valid_in) begin
			// Flop the A Channel signals
			r_a_valid <= a_valid_in;
			r_a_opcode <= a_opcode_in;
			r_a_param  <= a_param_in;
			r_a_address <= a_address_in;
			r_a_size   <= a_size_in;
			r_a_mask   <= a_mask_in;
			r_a_data   <= a_data_in;
			r_a_source <= a_source_in; 
		end
		else if (is_response) begin
			// If in RESPONSE state, clear the valid signal and reset the values
			r_a_valid <= 1'b0; // Master is not ready to send data
			r_a_opcode <= 0;
			r_a_param  <= 0; // Reserved, 0
			r_a_address <= 0;
			r_a_size   <= 0;
			r_a_mask   <= 0;
			r_a_data   <= 0;
			r_a_source <= 0;			
		end
		// If not in RESPONSE state, keep the previous values
		else begin
			// Keep the previous values
			r_a_valid <= r_a_valid;
			r_a_opcode <= r_a_opcode;
			r_a_param  <= r_a_param;
			r_a_address <= r_a_address;
			r_a_size   <= r_a_size;
			r_a_mask   <= r_a_mask;
			r_a_data   <= r_a_data;
			r_a_source <= r_a_source;
		end
	end


	always @(*) begin
		if (rst) begin
			// Reset logic for A Channel registers
			a_valid   = 1'b0;
			a_opcode  = 0;
			a_param   = 0;
			a_address = 0;
			a_size    = 0;
			a_mask    = 0;
			a_data    = 0;
			a_source  = 0;
		end else if (a_valid_in) begin
			// New transaction in same cycle
			a_valid   = a_valid_in;
			a_opcode  = a_opcode_in;
			a_param   = a_param_in;
			a_address = a_address_in;
			a_size    = a_size_in;
			a_mask    = a_mask_in;
			a_data    = a_data_in;
			a_source  = a_source_in;
		end else if (is_request) begin
			// Hold previous values (registered versions)
			a_valid   = r_a_valid;
			a_opcode  = r_a_opcode;
			a_param   = r_a_param;
			a_address = r_a_address;
			a_size    = r_a_size;
			a_mask    = r_a_mask;
			a_data    = r_a_data;
			a_source  = r_a_source;
		end else begin
			// Default case: no transaction
			a_valid   = 1'b0;
			a_opcode  = 0;
			a_param   = 0;
			a_address = 0;
			a_size    = 0;
			a_mask    = 0;
			a_data    = 0;
			a_source  = 0;
		end
	end

	// Response logic
	// This logic is for the D channel, which is the response from the slave to the master.
	// Opcode is used to determine the type of response.
	 always @(posedge clk) begin
	 	if(rst) begin
	 		r_d_valid <=  0;
	 	end
	 	else begin
	 		r_d_valid <= d_valid;
	 	end
	 end

endmodule

/*******************************************************************************************************************************************

This top-level module acts as a wrapper for the TileLink UltraLite (TL-UL) interface.
It connects the TileLink UltraLite interface to a TileLink UltraLite master and slave.

*******************************************************************************************************************************************/


module tilelink_wrapper_top #( 
    parameter TL_ADDR_WIDTH     = 64,
    parameter TL_DATA_WIDTH     = 64,
    parameter TL_STRB_WIDTH     = TL_DATA_WIDTH / 8,
    parameter TL_SOURCE_WIDTH   = 3,
    parameter TL_SINK_WIDTH     = 3,
    parameter TL_OPCODE_WIDTH   = 3,
    parameter TL_PARAM_WIDTH    = 3,
    parameter TL_SIZE_WIDTH     = 8,

	parameter MEM_BASE_ADDR 	  = 64'h0000_0000_0000_0000, // Base address for memory
	parameter DEPTH           = 512,                      // Memory depth (number of entries)    
    
    //CAN parameters
    parameter NODE_ID = 11'h456
)(
    input  wire                              clk,
    input  wire                              rst,

    // Inputs to drive the master from testbench
    input  wire                              a_valid_in,
    input  wire [TL_OPCODE_WIDTH-1:0]        a_opcode_in,
    input  wire [TL_PARAM_WIDTH-1:0]         a_param_in,
    input  wire [TL_ADDR_WIDTH-1:0]          a_address_in,
    input  wire [TL_SIZE_WIDTH-1:0]          a_size_in,
    input  wire [TL_STRB_WIDTH-1:0]          a_mask_in,
    input  wire [TL_DATA_WIDTH-1:0]          a_data_in,
    input  wire [TL_SOURCE_WIDTH-1:0]        a_source_in,

    // Outputs for testbench
    // Make all the following output wires 
    output wire a_valid_tb,
    output wire [TL_OPCODE_WIDTH-1:0] a_opcode_tb,
    output wire [TL_PARAM_WIDTH-1:0]  a_param_tb,
    output wire [TL_ADDR_WIDTH-1:0]   a_address_tb,
    output wire [TL_SIZE_WIDTH-1:0]   a_size_tb,
    output wire [TL_STRB_WIDTH-1:0]   a_mask_tb,
    output wire [TL_DATA_WIDTH-1:0]   a_data_tb,
    output wire [TL_SOURCE_WIDTH-1:0] a_source_tb,
    output wire a_ready_tb,

    output wire d_valid_tb,
    output wire d_ready_tb,
    output wire [TL_OPCODE_WIDTH-1:0] d_opcode_tb,
    output wire [TL_PARAM_WIDTH-1:0]  d_param_tb,
    output wire [TL_SIZE_WIDTH-1:0]   d_size_tb,
    output wire [TL_SINK_WIDTH-1:0]   d_sink_tb,
    output wire [TL_SOURCE_WIDTH-1:0] d_source_tb,
    output wire [TL_DATA_WIDTH-1:0]   d_data_tb,
    output wire d_error_tb,

    //test connections for CAN 
    input wire start_tx,
    input wire [10:0] id_tx,
    input wire [63:0] data_tx,
    output wire busy,
    output wire [63:0] data_rx,
    output wire response_ready,
    output wire valid_rx
);

    /////////////////////////////////////////
    //////// Localparams for opcodes ////////
    /////////////////////////////////////////

    localparam PUT_FULL_DATA_A     = 3'd0;
    localparam PUT_PARTIAL_DATA_A  = 3'd1;
    localparam ARITHMETIC_DATA_A   = 3'd2;
    localparam LOGICAL_DATA_A      = 3'd3;
    localparam GET_A               = 3'd4;
    localparam INTENT_A            = 3'd5;
    localparam ACQUIRE_BLOCK_A     = 3'd6;
    localparam ACQUIRE_PERM_A      = 3'd7;

    localparam ACCESS_ACK_D        = 3'd0;
    localparam ACCESS_ACK_DATA_D   = 3'd1;
    localparam HINT_ACK_D          = 3'd2;
    localparam GRANT_D             = 3'd4;
    localparam GRANT_DATA_D        = 3'd5;
    localparam RELEASE_ACK_D       = 3'd6;

    localparam IDLE     = 2'd0;
    localparam REQUEST  = 2'd1;
    localparam RESPONSE = 2'd2;
    localparam CLEANUP  = 2'd3;


    // Internal wires for A and D channels between master and slave
    wire a_ready;
    wire a_valid;
    wire [TL_OPCODE_WIDTH-1:0] a_opcode;
    wire [TL_PARAM_WIDTH-1:0]  a_param;
    wire [TL_ADDR_WIDTH-1:0]   a_address;
    wire [TL_SIZE_WIDTH-1:0]   a_size;
    wire [TL_STRB_WIDTH-1:0]   a_mask;
    wire [TL_DATA_WIDTH-1:0]   a_data;
    wire [TL_SOURCE_WIDTH-1:0] a_source;

    wire d_valid;
    wire d_ready;
    wire [TL_OPCODE_WIDTH-1:0] d_opcode;
    wire [TL_PARAM_WIDTH-1:0]  d_param;
    wire [TL_SIZE_WIDTH-1:0]   d_size;
    wire [TL_SINK_WIDTH-1:0]   d_sink;
    wire [TL_SOURCE_WIDTH-1:0] d_source;
    wire [TL_DATA_WIDTH-1:0]   d_data;
    wire                       d_error;



    // Assign outputs to internal wires for testbench visibility
    assign a_valid_tb    = a_valid;
    assign a_opcode_tb   = a_opcode;
    assign a_param_tb    = a_param;
    assign a_address_tb  = a_address;
    assign a_size_tb     = a_size;
    assign a_mask_tb     = a_mask;
    assign a_data_tb     = a_data;
    assign a_source_tb   = a_source;
    assign a_ready_tb       = a_ready;

    assign d_valid_tb    = d_valid;
    assign d_ready_tb    = d_ready;
    assign d_opcode_tb   = d_opcode;
    assign d_param_tb    = d_param;
    assign d_size_tb     = d_size;
    assign d_sink_tb     = d_sink;
    assign d_source_tb   = d_source;
    assign d_data_tb     = d_data;
    assign d_error_tb    = d_error;

	// Instantiate the TileLink Uncached Lightweight master

    tilelink_ul_master_top #(
        .TL_ADDR_WIDTH     (TL_ADDR_WIDTH),
        .TL_DATA_WIDTH     (TL_DATA_WIDTH),
        .TL_STRB_WIDTH     (TL_STRB_WIDTH),
        .TL_SOURCE_WIDTH   (TL_SOURCE_WIDTH),
        .TL_SINK_WIDTH     (TL_SINK_WIDTH),
        .TL_OPCODE_WIDTH   (TL_OPCODE_WIDTH),
        .TL_PARAM_WIDTH    (TL_PARAM_WIDTH),
        .TL_SIZE_WIDTH     (TL_SIZE_WIDTH),
        .PUT_FULL_DATA_A   (PUT_FULL_DATA_A),
        .PUT_PARTIAL_DATA_A(PUT_PARTIAL_DATA_A),
        .ARITHMETIC_DATA_A (ARITHMETIC_DATA_A),
        .LOGICAL_DATA_A    (LOGICAL_DATA_A),
        .GET_A             (GET_A),
        .INTENT_A          (INTENT_A),
        .ACQUIRE_BLOCK_A   (ACQUIRE_BLOCK_A),
        .ACQUIRE_PERM_A    (ACQUIRE_PERM_A),
        .ACCESS_ACK_D      (ACCESS_ACK_D),
        .ACCESS_ACK_DATA_D (ACCESS_ACK_DATA_D),
        .HINT_ACK_D        (HINT_ACK_D),
        .GRANT_D           (GRANT_D),
        .GRANT_DATA_D      (GRANT_DATA_D),
        .RELEASE_ACK_D     (RELEASE_ACK_D),
        .REQUEST           (REQUEST),
        .RESPONSE          (RESPONSE),
        .CLEANUP           (CLEANUP),
        .IDLE              (IDLE)
    ) master_inst (
        .clk           (clk),
        .rst           (rst),

        .a_valid_in    (a_valid_in),
        .a_opcode_in   (a_opcode_in),
        .a_param_in    (a_param_in),
        .a_address_in  (a_address_in),
        .a_size_in     (a_size_in),
        .a_mask_in     (a_mask_in),
        .a_data_in     (a_data_in),
        .a_source_in   (a_source_in),

        .a_ready       (a_ready),
        .a_valid       (a_valid),
        .a_opcode      (a_opcode),
        .a_param       (a_param),
        .a_address     (a_address),
        .a_size        (a_size),
        .a_mask        (a_mask),
        .a_data        (a_data),
        .a_source      (a_source),

        .d_valid       (d_valid),
        .d_ready       (d_ready),
        .d_opcode      (d_opcode),
        .d_param       (d_param),
        .d_size        (d_size),
        .d_sink        (d_sink),
        .d_source      (d_source),
        .d_data        (d_data),
        .d_error       (d_error)
    );

	// Instantiate the TileLink Uncached Lightweight slave

    tilelink_ul_slave_top #(
        .TL_ADDR_WIDTH      (TL_ADDR_WIDTH),
        .TL_DATA_WIDTH      (TL_DATA_WIDTH),
        .TL_STRB_WIDTH      (TL_STRB_WIDTH),
        .TL_SOURCE_WIDTH    (TL_SOURCE_WIDTH),
        .TL_SINK_WIDTH      (TL_SINK_WIDTH),
        .TL_OPCODE_WIDTH    (TL_OPCODE_WIDTH),
        .TL_PARAM_WIDTH     (TL_PARAM_WIDTH),
        .TL_SIZE_WIDTH      (TL_SIZE_WIDTH),
        .PUT_FULL_DATA_A    (PUT_FULL_DATA_A),
        .PUT_PARTIAL_DATA_A (PUT_PARTIAL_DATA_A),
        .ARITHMETIC_DATA_A  (ARITHMETIC_DATA_A),
        .LOGICAL_DATA_A     (LOGICAL_DATA_A),
        .GET_A              (GET_A),
        .INTENT_A           (INTENT_A),
        .ACQUIRE_BLOCK_A    (ACQUIRE_BLOCK_A),
        .ACQUIRE_PERM_A     (ACQUIRE_PERM_A),
        .ACCESS_ACK_D       (ACCESS_ACK_D),
        .ACCESS_ACK_DATA_D  (ACCESS_ACK_DATA_D),
        .HINT_ACK_D         (HINT_ACK_D),
        .GRANT_D            (GRANT_D),
        .GRANT_DATA_D       (GRANT_DATA_D),
        .RELEASE_ACK_D      (RELEASE_ACK_D),
        .REQUEST            (REQUEST),
        .RESPONSE           (RESPONSE),

        
        .MEM_BASE_ADDR(MEM_BASE_ADDR),
        .DEPTH(DEPTH),

        .NODE_ID(NODE_ID)
      )slave_inst (
        .clk        (clk),
        .rst        (rst),

        .a_ready    (a_ready),
        .a_valid    (a_valid),
        .a_opcode   (a_opcode),
        .a_param    (a_param),
        .a_address  (a_address),
        .a_size     (a_size),
        .a_mask     (a_mask),
        .a_data     (a_data),
        .a_source   (a_source),

        .d_valid    (d_valid),
        .d_ready    (d_ready),
        .d_opcode   (d_opcode),
        .d_param    (d_param),
        .d_size     (d_size),
        .d_sink     (d_sink),
        .d_source   (d_source),
        .d_data     (d_data),
        .d_error    (d_error),
// test for can 
        .start_tx   (start_tx),
        .id_tx      (id_tx),
        .data_tx    (data_tx),
        .busy       (busy),
        .data_rx    (data_rx),
        .response_ready (response_ready),
        .valid_rx   (valid_rx)
    );
	


endmodule
