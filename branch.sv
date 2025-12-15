`timescale 1ns / 1ps
import buffer_pkgs::*;

module branch #(
    parameter type BE = branch_entry_t,
    parameter type BO = branch_out_t
) (
    input  logic clk_i,
    input  logic reset_i,
    input  logic flush_i,

    // Input ready-valid
    input  logic req_valid_i,
    output logic req_ready_o,
    input  BE    data_i,

    // Output ready-valid
    output logic resp_valid_o,
    input  logic resp_ready_i,
    output BO    data_o
);

    // Registered output payload
    BO    data_q;
    logic resp_valid_q;

    assign data_o       = data_q;
    assign resp_valid_o = resp_valid_q;

    // Handshake
    wire fire_in  = req_valid_i && req_ready_o;
    wire fire_out = resp_valid_q && resp_ready_i;

    // Ready if we are not flushing AND either no pending result or consumer ready
    always_comb begin
        req_ready_o = (!flush_i) && (!resp_valid_q || resp_ready_i);
    end

    // Compute result for accepted input
    BO branch_out_next;

    always_comb begin
        branch_out_next = '0;

        // defaults
        branch_out_next.rd_addr      = '0;
        branch_out_next.rd_val       = 32'd0;
        branch_out_next.dest_addr    = data_i.pc + 32'd4;
        branch_out_next.branch_taken = 1'b0;
        branch_out_next.completed    = 1'b1;
        branch_out_next.mispredict   = 1'b0;
        branch_out_next.ROB_tag      = data_i.ROB_tag;

        if (req_valid_i) begin
            // BNE
            if (data_i.branch) begin
                logic actual_taken;
                actual_taken = (data_i.rs1_val != data_i.rs2_val);

                branch_out_next.branch_taken = actual_taken;

                if (actual_taken)
                    branch_out_next.dest_addr = data_i.pc + data_i.imm_val;
                else
                    branch_out_next.dest_addr = data_i.pc + 32'd4;

                // Always-not-taken predictor => predicted_taken = data_i.took_branch (should be 0)
                branch_out_next.mispredict = (data_i.took_branch != actual_taken);

                // no rd write for bne
                branch_out_next.rd_addr = '0;
                branch_out_next.rd_val  = 32'd0;
            end

            // JALR
            else if (data_i.jump) begin
                branch_out_next.branch_taken = 1'b1;

                branch_out_next.rd_addr   = data_i.rd_addr;
                branch_out_next.rd_val    = data_i.pc + 32'd4;
                branch_out_next.dest_addr = (data_i.rs1_val + data_i.imm_val) & ~32'd1;

                // ALWAYS NOT-TAKEN predictor => JALR always mispredicted
                branch_out_next.mispredict = 1'b1;
            end
        end

        if (flush_i) begin
            branch_out_next.completed  = 1'b0;
            branch_out_next.mispredict = 1'b0;
        end
    end

    // Register output with ready/valid semantics
    always_ff @(posedge clk_i) begin
        if (reset_i) begin
            data_q       <= '0;
            resp_valid_q <= 1'b0;
        end else if (flush_i) begin
            data_q       <= '0;
            resp_valid_q <= 1'b0;
        end else begin
            if (fire_out)
                resp_valid_q <= 1'b0;

            if (fire_in) begin
                data_q       <= branch_out_next;
                resp_valid_q <= 1'b1;
            end
        end
    end

endmodule
