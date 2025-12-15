`timescale 1ns / 1ps
import buffer_pkgs::*;

module alu #( 
    parameter type AE = alu_entry_t,
    parameter type AO = alu_out_t
)(
    input  logic clk_i,
    input  logic reset_i,
    input  logic flush_i,
    
    // ===========================
    // Input ready-valid handshake
    // ===========================
    input  logic req_valid_i,
    output logic req_ready_o,
    
    // ============================
    // Output ready-valid handshake
    // ============================
    output logic resp_valid_o,  // NEW
    input  logic resp_ready_i,  // NEW
    
    input  AE data_i,
    output AO data_o
);
    // -------------------------
    // Core ALU control signals
    // -------------------------
    logic [3:0] aluop;
    assign aluop = data_i.aluop;

    logic rs1_used;
    logic rs2_used;
    logic imm_used;
    logic [31:0] result_next;
    AO alu_out_next;

    // -------------------------
    // Handshake / pipeline regs
    // -------------------------
    logic fire_in;              // NEW: input handshake fire
    logic fire_out;             // NEW: output handshake fire

    AO   data_q;                // NEW: registered output payload
    logic resp_valid_q;         // NEW: registered output valid

    // NEW: connect regs to outputs
    assign data_o       = data_q;
    assign resp_valid_o = resp_valid_q;

    // NEW: handshake fires
    assign fire_in  = req_valid_i && req_ready_o;     // input side
    assign fire_out = resp_valid_q && resp_ready_i;   // output side

    // ==================================
    // Combinational: ALU operation + req_ready_o
    // ==================================
    always_comb begin
        rs1_used = data_i.rs1_used;
        rs2_used = data_i.rs2_used;
        imm_used = data_i.imm_used;

        // NEW: default (input ready depends on output state and flush)
        req_ready_o = !flush_i && (!resp_valid_q || resp_ready_i);

        // default result_next to avoid X's
        result_next = 32'hffff_ffff;  // same as your default

        // ALU core functionality (unchanged)
        case (aluop) 
            ADD: begin //add and addi
                if (rs2_used) begin //add
                    result_next = signed'(data_i.rs1_val) + signed'(data_i.rs2_val); 
                end else if (imm_used) begin //addi
                    result_next = signed'(data_i.rs1_val) + signed'(data_i.imm_val);
                end
            end
            SUB: begin //sub and bne, but only sub is executed by alu 
                result_next = signed'(data_i.rs1_val) - signed'(data_i.rs2_val);
            end
            ANDD: begin //and
                result_next = signed'(data_i.rs1_val) & signed'(data_i.rs2_val);
            end
            ORR: begin //ori
                result_next = signed'(data_i.rs1_val) | signed'(data_i.imm_val);
            end
            RSHIFT: begin //sra
                result_next = signed'(data_i.rs1_val) >>> data_i.rs2_val[4:0];
            end
            SLTIU: begin //set (if) less than immediate unsigned 
                result_next = (unsigned'(data_i.rs1_val) < unsigned'(data_i.imm_val)) ? 32'd1 : 32'd0; 
            end
            default: begin //NA
                result_next = 32'hffff_ffff;
            end
        endcase 
        
        // Build next-cycle output struct (unchanged semantics)
        alu_out_next.rd_addr   = data_i.rd_addr;
        alu_out_next.rd_val    = result_next;
        alu_out_next.ROB_tag   = data_i.ROB_tag;
        alu_out_next.completed = 1'b1;

        // flush affects only the produced result's "completed" flag
        if (flush_i) begin
            alu_out_next.completed = 1'b0;
            // req_ready_o is already !flush_i && ...
        end
    end
    
    // ==================================
    // 1-cycle latency + output handshake
    // ==================================
    always_ff @(posedge clk_i) begin
        if (reset_i) begin
            data_q       <= '0;        // NEW
            resp_valid_q <= 1'b0;      // NEW
        end else if (flush_i) begin    // NEW: clear output on flush
            data_q       <= '0;
            resp_valid_q <= 1'b0;
        end else begin
            // If consumer takes the current result
            if (fire_out) begin        // NEW
                resp_valid_q <= 1'b0;
            end

            // If we accept a new request this cycle
            if (fire_in) begin         // NEW
                data_q       <= alu_out_next;
                resp_valid_q <= 1'b1;
            end
        end
    end
endmodule
