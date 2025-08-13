module CAN_TLUL_mon (
    input  wire clk,
    input wire rst,
    // TL-UL master input signals
    input  wire a_valid_in,
    input  wire [2:0] a_opcode_in,
    input  wire [2:0] a_param_in,
    input  wire [63:0] a_address_in,
    input  wire [7:0] a_size_in,
    input  wire [7:0] a_mask_in,
    input  wire [63:0] a_data_in,
    input  wire [2:0] a_source_in,

    // TL-UL master/slave outputs from wrapper
    input  wire a_valid_tb,
    input  wire [2:0] a_opcode_tb,
    input  wire [2:0] a_param_tb,
    input  wire [63:0] a_address_tb,
    input  wire [7:0] a_size_tb,
    input  wire [7:0] a_mask_tb,
    input  wire [63:0] a_data_tb,
    input  wire [2:0] a_source_tb,
    input  wire a_ready_tb,

    input  wire d_valid_tb,
    input  wire d_ready_tb,
    input  wire [2:0] d_opcode_tb,
    input  wire [2:0] d_param_tb,
    input  wire [7:0] d_size_tb,
    input  wire [2:0] d_sink_tb,
    input  wire [2:0] d_source_tb,
    input  wire [63:0] d_data_tb,
    input  wire d_error_tb,

    // Test node control signals
    input  wire 	start_tx,
    input  wire [10:0] 	id_tx,
    input  wire [63:0] 	data_tx,

    // Test node outputs
    input  wire 	busy,
    input  wire [63:0] 	data_rx,
    input  wire         response_ready,
    input  wire 	valid_rx
);
    always @(posedge clk) begin
        // Monitor TL-UL master input
        if (a_valid_in)
            $display("Time %0t - TL-UL Master Input: opcode=%0d, addr=%h, data=%h", 
                     $time, a_opcode_in, a_address_in, a_data_in);

        // Monitor TL-UL output to slave
        if (a_valid_tb)
            $display("Time %0t - TL-UL Output to Slave: opcode=%0d, addr=%h, data=%h", 
                     $time, a_opcode_tb, a_address_tb, a_data_tb);

        // Monitor TL-UL slave return channel
        if (d_valid_tb)
            $display("Time %0t - TL-UL Slave Response: opcode=%0d, data=%h", 
                     $time, d_opcode_tb, d_data_tb);

        // Monitor CAN test node TX
        if (start_tx)
            $display("Time %0t - Test Node Starting TX: ID=%h, Data=%h", 
                     $time, id_tx, data_tx);

        // Monitor CAN test node RX
        if (valid_rx)
            $display("Time %0t - Test Node Received Data: %h", 
                     $time, data_rx);

        // Optional: monitor busy flag
        //if (busy)
          //  $display("Time %0t - Test Node Busy transmitting", $time);
    end
endmodule

