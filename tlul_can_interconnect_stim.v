`timescale 1ns/1ps

module can_tlul_stimulus #(
    parameter TL_ADDR_WIDTH   = 64,
    parameter TL_DATA_WIDTH   = 64,
    parameter TL_STRB_WIDTH   = TL_DATA_WIDTH / 8,
    parameter TL_SOURCE_WIDTH = 3,
    parameter TL_SINK_WIDTH   = 3,
    parameter TL_OPCODE_WIDTH = 3,
    parameter TL_PARAM_WIDTH  = 3,
    parameter TL_SIZE_WIDTH   = 8,
    parameter MEM_BASE_ADDR   = 64'h0,
    parameter DEPTH           = 512,
    parameter NODE_ID_DUT     = 11'h123,
    parameter NODE_ID_TEST    = 11'h456,
    parameter [63:0] TEST_DATA_WRITE = 64'hDEADBEEF_CAFEBABE,
    parameter [63:0] TEST_DATA_READ  = 64'hABCDEF12_34567890
)(
    // Clock and reset
    output reg clk,
    output reg rst,

    // TL-UL master input signals (to DUT)
    output reg                       a_valid_in,
    output reg [TL_OPCODE_WIDTH-1:0] a_opcode_in,
    output reg [TL_PARAM_WIDTH-1:0]  a_param_in,
    output reg [TL_ADDR_WIDTH-1:0]   a_address_in,
    output reg [TL_SIZE_WIDTH-1:0]   a_size_in,
    output reg [TL_STRB_WIDTH-1:0]   a_mask_in,
    output reg [TL_DATA_WIDTH-1:0]   a_data_in,
    output reg [TL_SOURCE_WIDTH-1:0] a_source_in,

    // TL-UL monitor inputs (from DUT)
    input  wire                      a_valid_tb,
    input  wire [TL_OPCODE_WIDTH-1:0] a_opcode_tb,
    input  wire [TL_PARAM_WIDTH-1:0]  a_param_tb,
    input  wire [TL_ADDR_WIDTH-1:0]   a_address_tb,
    input  wire [TL_SIZE_WIDTH-1:0]   a_size_tb,
    input  wire [TL_STRB_WIDTH-1:0]   a_mask_tb,
    input  wire [TL_DATA_WIDTH-1:0]   a_data_tb,
    input  wire [TL_SOURCE_WIDTH-1:0] a_source_tb,
    input  wire                      a_ready_tb,

    input  wire                      d_valid_tb,
    input  wire                      d_ready_tb,
    input  wire [TL_OPCODE_WIDTH-1:0] d_opcode_tb,
    input  wire [TL_PARAM_WIDTH-1:0]  d_param_tb,
    input  wire [TL_SIZE_WIDTH-1:0]   d_size_tb,
    input  wire [TL_SINK_WIDTH-1:0]   d_sink_tb,
    input  wire [TL_SOURCE_WIDTH-1:0] d_source_tb,
    input  wire [TL_DATA_WIDTH-1:0]   d_data_tb,
    input  wire                      d_error_tb,

    // CAN test control (to DUT)
    output reg         start_tx,
    output reg [10:0]  id_tx,
    output reg [63:0]  data_tx,

    // CAN monitor (from DUT)
    input  wire        busy,
    input  wire [63:0] data_rx,
    input  wire        response_ready,
    input  wire        valid_rx
);
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz clock
    end

    // Test sequence logic
    initial begin
        // Initialize all driving signals
        rst = 1;
        a_valid_in = 0;
        a_opcode_in = 0;
        a_param_in = 0;
        a_address_in = 0;
        a_size_in = 0;
        a_mask_in = 0;
        a_data_in = 0;
        a_source_in = 0;
        start_tx = 0;
        id_tx = 0;
        data_tx = 0;

        // Apply reset
        #100;
        rst = 0;
        #50;

        // == TEST 1: Drive TL-UL WRITE Transaction ==
        @(posedge clk);
        a_valid_in   = 1;
        a_opcode_in  = 3'b000;      // PUT_FULL_DATA_A
        a_param_in   = 3'b000;      // Reserved
        a_address_in = NODE_ID_TEST;
        a_size_in    = 8'd3;        // 2^3 = 8 bytes
        a_mask_in    = 8'hFF;       // Enable all bytes
        a_data_in    = TEST_DATA_WRITE;
        a_source_in  = 3'b001;      // Transaction ID

        // Wait for the DUT to accept the request before de-asserting
        wait(a_ready_tb && a_valid_tb);
        @(posedge clk);
        a_valid_in = 0;
        a_opcode_in = 0;
        a_param_in = 0;
        a_address_in = 0;
        a_size_in = 0;
        a_mask_in = 0;
        a_data_in = 0;
        a_source_in = 0;

        // Wait for the TL-UL write response to know the transaction is acknowledged by the slave
        wait(d_valid_tb);
        
        #200; // Delay between tests

        // == TEST 2: Drive CAN transmission from the test node ==
        start_tx = 1;
        id_tx   = NODE_ID_DUT; // Target the main DUT
        data_tx  = TEST_DATA_READ;

        @(posedge clk);
        start_tx= 0;

        // Wait for test node to finish sending the CAN message
        //wait(!busy);
        
        // Wait for the main DUT to signal that received CAN data is ready
        wait(response_ready);

        // Now, initiate a TL-UL read to fetch the data from the DUT
        @(posedge clk);
        a_valid_in   = 1;
        a_opcode_in  = 3'd4;        // GET_A
        a_param_in   = 3'b000;      // Reserved
        a_address_in = 64'h1000;     // Read address (can be arbitrary for this logic)
        a_size_in    = 8'd3;        // 8 bytes
        a_mask_in    = 8'hFF;
        a_data_in    = 64'h0;
        a_source_in  = 3'b010;      // New transaction ID

        // Wait for the DUT to accept the read request
        wait(a_ready_tb && a_valid_tb);
        @(posedge clk);
        a_valid_in = 0;
    end

endmodule



