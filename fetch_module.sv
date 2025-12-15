`timescale 1ns / 1ps
import buffer_pkgs::*;   // fetch_t {pc,instr}

module fetch #(
    parameter int DEPTH = 64,
    parameter type F = fetch_t
)(
    input  logic        clk_i,
    input  logic        reset_i,

    // downstream (fetch -> decode)
    output logic        valid_cons_o,
    input  logic        ready_cons_i,
    output F            data_o,

    // redirect / recovery control (branch/jump mispredict)
    input  logic        redirect_i,
    input  logic [31:0] redirect_pc_i
);

    localparam int AW = $clog2(DEPTH);
    logic [31:0] pc_now;

    // ROM read
    logic [AW-1:0] rom_addr;
    logic [31:0]   instr_now;

    assign rom_addr = pc_now[AW+1:2];

    i_cache my_icache (
        .a   (rom_addr),
        .d   (32'b0),
        .clk (1'b0),
        .we  (1'b0),
        .spo (instr_now)
    );

    // Fetch -> queue interface
    logic valid_prod_q;
    logic ready_prod_q;
    F     data_prod_q;
    logic fire;

    assign valid_prod_q    = !reset_i;
    assign data_prod_q.pc  = pc_now;
    assign data_prod_q.instr = instr_now;

    assign fire = valid_prod_q && ready_prod_q;

    // PC update:
    // - redirect_i wins immediately (even if stalled)
    // - otherwise, only advance when we successfully enqueue one fetch word
    always_ff @(posedge clk_i) begin
        if (reset_i) begin
            pc_now <= 32'd0;
        end else if (redirect_i) begin
            pc_now <= redirect_pc_i;
        end else if (fire) begin
            pc_now <= pc_now + 32'd4;
        end
        // else hold pc_now (stall)
    end

    // Queue between fetch and decode; flush on redirect
    fifo2 #(.T(F)) u_fetch_q (
        .clk       (clk_i),
        .reset     (reset_i),
        .flush_in  (redirect_i),

        .valid_in  (valid_prod_q),
        .ready_in  (ready_prod_q),
        .data_in   (data_prod_q),

        .valid_out (valid_cons_o),
        .ready_out (ready_cons_i),
        .data_out  (data_o),

        .full      (),
        .empty     ()
    );

endmodule
