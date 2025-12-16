`timescale 1ns / 1ns
import buffer_pkgs::*;

module decode_module #(
    parameter type F = fetch_t,
    parameter type D = decode_t
)(
    input  logic clk_i,
    input  logic reset_i,
    
    // upstream (prod to skid)
    input  logic valid_prod_i,
    output logic ready_prod_o,
    input  F     data_i,
    
    // downstream (skid to cons)
    input  logic ready_cons_i,
    output logic valid_cons_o,
    output D     data_o
);

    // ------------------------------------------------------------
    // Internal decode fields
    // ------------------------------------------------------------
    logic [6:0] opcode;
    logic [2:0] func3;

    assign opcode = data_i.instr[6:0];
    assign func3  = data_i.instr[14:12];

    // internal vars to pass through to skid buffer:
    logic [31:0] pc_cons_i;
    assign pc_cons_i = data_i.pc;
    logic [31:0] instr_cons_i;
    assign instr_cons_i = data_i.instr;

    logic        decode_valid;
    logic        valid_dec_in;
    D            data_o_comb; 
    logic        supported_instr;

    // ------------------------------------------------------------
    // Main decode logic
    // ------------------------------------------------------------
    always_comb begin
        // defaults
        data_o_comb      = '0;
        supported_instr  = 1'b0;
        valid_dec_in     = 1'b0;

        // pass-throughs
        data_o_comb.pc     = pc_cons_i;
        data_o_comb.opcode = opcode;
        data_o_comb.instr = instr_cons_i;

        // -----------------------------
        // ALU op + "supported" decision
        // -----------------------------
        case (opcode)
            7'b0100011 : begin // STORE: sh, sw
                data_o_comb.aluop = NA; // LSU will handle addr calc/data
                if (func3 == 3'b001 || func3 == 3'b010) // sh / sw
                    supported_instr = 1'b1;
            end

            7'b0000011 : begin // LOAD: lw, lbu
                data_o_comb.aluop = NA;
                if (func3 == 3'b010 || func3 == 3'b100) // lw / lbu
                    supported_instr = 1'b1;
            end

            7'b1100111 : begin // JALR
                data_o_comb.aluop = ADD;
                if (func3 == 3'b000)
                    supported_instr = 1'b1;
            end

            7'b1100011 : begin // BNE
                data_o_comb.aluop = SUB;
                if (func3 == 3'b001)
                    supported_instr = 1'b1;
            end

            7'b0110111 : begin // LUI
                data_o_comb.aluop = LUI;
                supported_instr   = 1'b1;
            end

            7'b0010011 : begin // ADDI, SLTIU, ORI (I-type)
                case (func3)
                    3'b000: begin // ADDI
                        data_o_comb.aluop = ADD;
                        supported_instr   = 1'b1;
                    end
                    3'b011: begin // SLTIU
                        data_o_comb.aluop = SLTIU;
                        supported_instr   = 1'b1;
                    end
                    3'b110: begin // ORI
                        data_o_comb.aluop = ORR;
                        supported_instr   = 1'b1;
                    end
                    default: begin
                        supported_instr   = 1'b0;
                    end
                endcase
            end

            7'b0110011 : begin // AND, SUB, SRA (R-type)
                case (func3)
                    3'b000: begin // SUB (assuming funct7 encodes SUB)
                        data_o_comb.aluop = SUB;
                        supported_instr   = 1'b1;
                    end
                    3'b111: begin // AND
                        data_o_comb.aluop = ANDD;
                        supported_instr   = 1'b1;
                    end
                    3'b101: begin // SRA
                        data_o_comb.aluop = RSHIFT;
                        supported_instr   = 1'b1;
                    end
                    default: begin
                        supported_instr   = 1'b0;
                    end
                endcase
            end

            default: begin
                data_o_comb.aluop = NA;
                supported_instr   = 1'b0;
            end
        endcase

        // -----------------------------
        // Immediate + immUsed
        // -----------------------------
        case (opcode)
            // I-type: JALR, loads, I-ALU
            7'b1100111, // jalr
            7'b0000011, // lw, lbu
            7'b0010011 : begin // addi, sltiu, ori
                data_o_comb.imm     = {{20{data_i.instr[31]}}, data_i.instr[31:20]};
                data_o_comb.immUsed = 1'b1;
            end

            // S-type: stores
            7'b0100011 : begin // sw, sh   
                data_o_comb.imm     = {{20{data_i.instr[31]}},
                                       data_i.instr[31:25],
                                       data_i.instr[11:7]};
                data_o_comb.immUsed = 1'b1;
            end        

            // U-type: LUI
            7'b0110111 : begin // lui
                data_o_comb.imm     = {data_i.instr[31:12], 12'b0};
                data_o_comb.immUsed = 1'b1;
            end

            // B-type: BNE
            7'b1100011 : begin // bne
                data_o_comb.imm     = {{19{data_i.instr[31]}},
                                       data_i.instr[31],
                                       data_i.instr[7],
                                       data_i.instr[30:25],
                                       data_i.instr[11:8],
                                       1'b0};
                data_o_comb.immUsed = 1'b1;
            end

            default: begin // SUB, AND, SRA (R-type)
                data_o_comb.imm     = '0;
                data_o_comb.immUsed = 1'b0;
            end
        endcase

        // -----------------------------
        // Functional unit / LSU info
        // -----------------------------
        case (opcode)
            7'b0100011 : begin // sh, sw
                data_o_comb.funcU     = MEM;
                data_o_comb.loadStore = STORE;
                if (func3 == 3'b001)       // sh
                    data_o_comb.lsType = BHW;
                else if (func3 == 3'b010)  // sw
                    data_o_comb.lsType = WORD;
            end

            7'b0000011 : begin // lbu, lw
                data_o_comb.funcU     = MEM;
                data_o_comb.loadStore = LOAD;
                if (func3 == 3'b100)       // lbu
                    data_o_comb.lsType = BHW;
                else if (func3 == 3'b010)  // lw
                    data_o_comb.lsType = WORD;
            end

            // Branch / jump could go to BRANCH FU if you want:
            7'b1100111,
            7'b1100011 : begin
                data_o_comb.funcU     = BRANCH;
                data_o_comb.loadStore = 1'b0;
                data_o_comb.lsType    = BHW; // don't-care
            end

            default : begin 
                data_o_comb.funcU     = ALU;
                data_o_comb.loadStore = 1'b0;
                data_o_comb.lsType    = BHW; // default
            end
        endcase 

        // -----------------------------
        // Branch / jump flags
        // -----------------------------
        case (opcode)
            7'b1100111 : begin // jalr
                data_o_comb.branch = 1'b0;
                data_o_comb.jump   = 1'b1;
            end
            7'b1100011 : begin // bne
                data_o_comb.branch = 1'b1;
                data_o_comb.jump   = 1'b0;
            end
            default : begin
                data_o_comb.branch = 1'b0;
                data_o_comb.jump   = 1'b0;
            end
        endcase

        // -----------------------------
        // Register fields (rs1/rs2/rd)
        // -----------------------------
        case (opcode)
            // I-type: addi, sltiu, ori, lw, lbu, jalr
            7'b0010011,
            7'b0000011,
            7'b1100111 : begin
                data_o_comb.rs1     = data_i.instr[19:15];
                data_o_comb.rs1Used = 1'b1;

                data_o_comb.rs2     = '0;
                data_o_comb.rs2Used = 1'b0;

                data_o_comb.rd      = data_i.instr[11:7];
                data_o_comb.rdUsed  = 1'b1;
            end

            // B-type and S-type: bne, sw, sh
            7'b1100011,
            7'b0100011 : begin
                data_o_comb.rs1     = data_i.instr[19:15];
                data_o_comb.rs1Used = 1'b1;       

                data_o_comb.rs2     = data_i.instr[24:20];
                data_o_comb.rs2Used = 1'b1;

                data_o_comb.rd      = '0;
                data_o_comb.rdUsed  = 1'b0;
            end

            // R-type: and, sra, sub
            7'b0110011 : begin
                data_o_comb.rs1     = data_i.instr[19:15];
                data_o_comb.rs1Used = 1'b1; 

                data_o_comb.rs2     = data_i.instr[24:20];
                data_o_comb.rs2Used = 1'b1;

                data_o_comb.rd      = data_i.instr[11:7];
                data_o_comb.rdUsed  = 1'b1;
            end

            // U-type: lui
            7'b0110111 : begin
                data_o_comb.rs1     = '0;
                data_o_comb.rs1Used = 1'b0;

                data_o_comb.rs2     = '0;
                data_o_comb.rs2Used = 1'b0;

                data_o_comb.rd      = data_i.instr[11:7];
                data_o_comb.rdUsed  = 1'b1;
            end

            default: begin
                data_o_comb.rs1     = '0;
                data_o_comb.rs1Used = 1'b0;
                data_o_comb.rs2     = '0;
                data_o_comb.rs2Used = 1'b0;
                data_o_comb.rd      = '0;
                data_o_comb.rdUsed  = 1'b0;
            end
        endcase 

        // -----------------------------
        // Final valid gating
        // -----------------------------
        valid_dec_in  = supported_instr;
        decode_valid  = valid_prod_i & valid_dec_in;
    end

    // ------------------------------------------------------------
    // Skid buffer
    // ------------------------------------------------------------
   D     buffer_output_d;
    logic buffer_ready_i;
    logic buffer_valid_o;

    skid_buffer_nopass #(.T(D)) decode_skid (
        .clk       (clk_i),
        .reset     (reset_i),

        .flush_in  (1'b0),        // [CHG] was empty (). Tie off until redirect is wired.

        .valid_in  (decode_valid),
        .ready_in  (buffer_ready_i),
        .data_in   (data_o_comb),

        .valid_out (buffer_valid_o),
        .ready_out (ready_cons_i),
        .data_out  (buffer_output_d)
    );

    assign data_o       = buffer_output_d;
    assign ready_prod_o = buffer_ready_i;
    assign valid_cons_o = buffer_valid_o;
        
endmodule
