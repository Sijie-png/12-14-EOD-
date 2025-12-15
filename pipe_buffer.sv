`timescale 1ns / 1ns

module pipe_buffer #(
    parameter type T     = logic [31:0],
    parameter int  DEPTH = 2
)(
    input  logic clk_i,
    input  logic rst_i,
    input  logic flush_i,

    // Upstream
    input  T     data_in,
    input  logic valid_in,
    output logic ready_in,

    // Downstream
    output T     data_out,
    output logic valid_out,
    input  logic ready_out
);

    logic fifo_full, fifo_empty;
    T     fifo_rdata;
    logic fifo_wr_en, fifo_rd_en;

    // Write when producer fires and we're not flushing
    assign ready_in   = !fifo_full && !flush_i;
    assign fifo_wr_en = valid_in && ready_in;

    // FWFT behavior: output is just "front" of FIFO
    assign data_out   = fifo_rdata;
    assign valid_out  = !fifo_empty;

    // Pop when consumer takes data
    assign fifo_rd_en = valid_out && ready_out && !flush_i;

    fifo #(
        .T    (T),
        .DEPTH(DEPTH)
    ) u_fifo (
        .clk_i   (clk_i),
        .reset_i (rst_i || flush_i),

        .wdata_i (data_in),
        .wr_en_i (fifo_wr_en),
        .full_o  (fifo_full),

        .rdata_o (fifo_rdata),
        .rd_en_i (fifo_rd_en),
        .empty_o (fifo_empty)
    );

endmodule
