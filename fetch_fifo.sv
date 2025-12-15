`timescale 1ns/1ps
//chat gpt written to possible reduce timing path
//with current skid: 
//ready_out (from decode) → combinational in skid → ready_in → combinational in fetch → 
//fire → PC-enable / address → instruction → combinational back into skid.
//with fifo: decode comb path is separate from fetch comb path 
module fifo2 #(
    parameter type T = logic [31:0]
)(
    input  logic clk,
    input  logic reset,     // async reset like your skid; change if you want sync
    input  logic flush_in,  // synchronous flush (e.g., redirect)

    // producer side (upstream)
    input  logic valid_in,
    output logic ready_in,
    input  T     data_in,

    // consumer side (downstream)
    output logic valid_out,
    input  logic ready_out,
    output T     data_out,

    // optional status
    output logic full,
    output logic empty,
    
    //test
    output logic [1:0] test_count_out
);

    // storage
    T mem [0:1];

    // pointers and occupancy
    logic        rptr, wptr;       // 1-bit pointers for depth=2
    logic [1:0]  count;            // 0,1,2

    // handshakes
    logic push, pop;
    
//    //test
//    logic 

    // purely from registered state -> no comb loop
//    assign ready_in = flush_in ? 1'b1 : (count != 2);//avoids ready_in = 0 during flush_in reads
    assign ready_in = flush_in ? 1'b1 : //if flush, then ready
        ((count == 2 && pop) ? 1'b1 :  //else if currently full but will not be next cycle, then ready
            (count < 2) ? 1'b1 : //else if currently not full, then ready
                1'b0); //else if currently full and will ont be ready next cycle, then not ready. 
    //if flush in 
    assign valid_out = (count != 0);

    assign full  = (count == 2);
    assign empty = (count == 0);

    assign push = valid_in  && ready_in;
    assign pop  = valid_out && ready_out;

    // combinational read of head element
    assign data_out = mem[rptr];

    always_ff @(posedge clk) begin
        if (reset) begin
            rptr  <= 1'b0;
            wptr  <= 1'b0;
            count <= 2'd0;
        end else if (flush_in) begin
            // drop all buffered beats on redirect/flush
            rptr  <= 1'b0;
            wptr  <= 1'b0;
            count <= 2'd0;
            
            // BUT accept new-path beat immediately if offered
            if (valid_in) begin
                mem[0] <= data_in;
                rptr   <= 1'b0;
                wptr   <= 1'b1;
                count  <= 2'd1;
            end 
        end else begin
            unique case ({push, pop})
                2'b10: begin // push only
                    mem[wptr] <= data_in;
                    wptr <= ~wptr;
                    count <= count + 2'd1;
                end

                2'b01: begin // pop only
                    rptr <= ~rptr;
                    count <= count - 2'd1;
                end

                2'b11: begin // push and pop same cycle
                    // overwrite tail while consuming head
                    mem[wptr] <= data_in;
                    wptr <= ~wptr;
                    rptr <= ~rptr;
                    // count unchanged
                end

                default: ; // 2'b00
            endcase
        end
    end

endmodule
