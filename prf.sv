`timescale 1ns/1ns
module prf #(
    parameter int NREGS = 128,
    parameter int XLEN  = 32
)(
    input  logic clk_i,
    input  logic rst_i,

    input  logic                     wb_en_i,
    input  logic [$clog2(NREGS)-1:0]  wb_addr_i,
    input  logic [XLEN-1:0]           wb_data_i,

    input  logic [$clog2(NREGS)-1:0]  rs1_i,
    input  logic                      rs1_used_i,
    output logic [XLEN-1:0]           rs1_data_o,

    input  logic [$clog2(NREGS)-1:0]  rs2_i,
    input  logic                      rs2_used_i,
    output logic [XLEN-1:0]           rs2_data_o
);

    logic [XLEN-1:0] pregs [0:NREGS-1];
    integer i;

    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            for (i = 0; i < NREGS; i++) pregs[i] <= '0;
        end else begin
            if (wb_en_i && (wb_addr_i != '0))
                pregs[wb_addr_i] <= wb_data_i;
        end
    end

    always_comb begin
        rs1_data_o = '0;
        rs2_data_o = '0;

        // ----------------------------
        // FIX: forwarding for same-cycle read-after-write
        // ----------------------------
        if (rs1_used_i) begin
            if (rs1_i == '0) rs1_data_o = '0;
            else if (wb_en_i && (wb_addr_i == rs1_i) && (wb_addr_i != '0))
                rs1_data_o = wb_data_i;              // FIX
            else
                rs1_data_o = pregs[rs1_i];
        end

        if (rs2_used_i) begin
            if (rs2_i == '0) rs2_data_o = '0;
            else if (wb_en_i && (wb_addr_i == rs2_i) && (wb_addr_i != '0))
                rs2_data_o = wb_data_i;              // FIX
            else
                rs2_data_o = pregs[rs2_i];
        end
    end

endmodule
