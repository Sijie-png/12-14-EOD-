`timescale 1ns / 1ps

module i_cache (
    input  logic [5:0]  a,
    input  logic [31:0] d,
    input  logic        clk,
    input  logic        we,
    output logic [31:0] spo
);
    logic [31:0] mem [0:63];

    initial begin
        $readmemh("processor_instructions.mem", mem);
    end

    assign spo = mem[a];
endmodule