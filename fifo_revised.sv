`timescale 1ns / 1ns

module fifo #(
    parameter type T = logic[31:0],
    parameter DEPTH = 8
) (
    // Clock and Reset
    input  logic             clk_i,
    input  logic             reset_i,
    // Write Interface
    input  T wdata_i,
    input  logic             wr_en_i,
    output logic             full_o,
    // Read Interface
    output T rdata_o,
    input  logic             rd_en_i,
    output logic             empty_o
);

    // Local parameters
    localparam ADDR_WIDTH = $clog2(DEPTH);

    // Internal signals
    logic [ADDR_WIDTH-1:0] rptr, wptr;
    logic full, empty;
    logic last_was_read;

    // Memory array
    T mem [0:DEPTH-1];

    // Write operation
    always_ff @(posedge clk_i or posedge reset_i) begin
        if (reset_i) begin
            wptr <= '0;
        end else begin
            if (wr_en_i && !full) begin
                mem[wptr] <= wdata_i;
                wptr <= wptr + 1'b1;
            end
        end
    end

    // Read operation
    always_ff @(posedge clk_i or posedge reset_i) begin
        if (reset_i) begin
            rptr <= '0;
        end else begin
            if (rd_en_i && !empty) begin
                rptr <= rptr + 1'b1;
            end
        end
    end
   
    // Continuous read data assignment
    assign rdata_o = mem[rptr];

    // Last operation tracker
    always_ff @(posedge clk_i or posedge reset_i) begin
        if (reset_i) begin
            last_was_read <= 1'b1; // Initialize as empty
        end else begin
            if (rd_en_i && !empty) begin
                last_was_read <= 1'b1;
            end else if (wr_en_i && !full) begin
                last_was_read <= 1'b0;
            end
            // else maintain current state
        end
    end
   
    // Status flag generation
    assign full  = (wptr == rptr) && !last_was_read;
    assign empty = (wptr == rptr) &&  last_was_read;
   
    assign full_o  = full;
    assign empty_o = empty;
endmodule
	
