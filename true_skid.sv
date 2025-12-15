`timescale 1ns / 1ns
module skid_buffer_revised #(
    parameter type T = logic [7:0]
)(
    input  logic clk,
    input  logic reset,
    input  logic flush_in,

    input  logic valid_in,
    output logic ready_in,
    input  T     data_in,

    output logic valid_out,
    input  logic ready_out,
    output T     data_out,

    // debug
    output logic full,
    output logic read_en,
    output logic write_en
);

    T hold_data;

    // ----------------------------
    // FIX: true elastic behavior
    // - bypass when empty
    // - store only when downstream stalls
    // ----------------------------
    wire bypass_active = !full;

    // Output mux
    assign data_out  = (full) ? hold_data : data_in;        // FIX
    assign valid_out = (full) ? 1'b1      : valid_in;       // FIX

    // Upstream ready: can accept if we have no stored item,
    // or if downstream is taking the current output this cycle.
    assign ready_in  = bypass_active ? ready_out : (ready_out); // FIX: same expression but explicit

    // Store when input fires but downstream can't take it (classic skid)
    assign write_en = valid_in && ready_in && !ready_out;   // FIX
    assign read_en  = full && ready_out;                    // FIX

    always_ff @(posedge clk) begin
        if (reset) begin
            full      <= 1'b0;
            hold_data <= '0;
        end else if (flush_in) begin
            full      <= 1'b0;
        end else begin
            if (write_en) begin
                hold_data <= data_in;
                full      <= 1'b1;
            end else if (read_en) begin
                full      <= 1'b0;
            end
        end
    end

endmodule
