`timescale 1ns / 1ns

module skid_buffer_nopass #(
    parameter type T = logic [7:0]
)(
    input logic clk,
    input logic reset,
    input logic flush_in,
    
    input logic valid_in,
    output logic ready_in,
    input T data_in,
    
    output logic valid_out,
    input logic ready_out,
    output T data_out,
    //testing
    output logic full,
    output logic read_en,
    output logic write_en
//    output logic empty,
//    output logic use_fifo_out,
//    output T test_fifo_out,
//    output T test_hold_data,
//    output logic test_write_en,
//    output logic test_read_en
//    output logic full_prev
  );
//    logic empty = 1;
//    logic full = 0;
    logic use_fifo_out = 0;
//    logic write_en = 0;
//    logic read_en = 0;
    
//    logic full;
//    logic empty;
    T hold_data = 'haa;
    T fifo_out = 0;
    
    always_comb begin
        ready_in = !full || (full && ready_out);
        write_en = valid_in && ready_in;
        read_en = (ready_out && full);
        use_fifo_out = read_en; //can omit
//      ready_in = empty;//producer checks this before asserting valid
        valid_out = full;
//        valid_out = !empty || valid_in;
    end
    
    always_ff @(posedge clk) begin
        if (reset) begin 
            hold_data <= 'haa;
            full <= 0;
        end else if (flush_in) begin
            full <= 0;
        end else begin
            unique case ({write_en, read_en})
                2'b10: begin //only write
                    hold_data <= data_in;
                    full <= 1;
                end
                2'b01: begin //only read 
                    full <= 0;
                end
                2'b11: begin
                    hold_data <= data_in;
                    full <= 1;
                end
                2'b00: begin
                end
            endcase 
        end
    end
    
//    assign test_hold_data = hold_data; //testing
//    assign test_write_en = write_en;
//    assign test_read_en = read_en;
    assign data_out = hold_data;
    
endmodule
