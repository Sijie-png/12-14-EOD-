`timescale 1ns/1ps
import buffer_pkgs::*;

module processor_tb();

    // ------------------------------------------------------------
    // Clock / reset
    // ------------------------------------------------------------
    logic clk, reset;
    always #5 clk = ~clk;

    // ------------------------------------------------------------
    // DUT interface (dispatch->execute probe points)
    // ------------------------------------------------------------
    alu_entry_t    alu_pkt;
    lsu_entry_t    lsu_pkt;
    branch_entry_t br_pkt;

    logic alu_v, alu_r;
    logic lsu_v, lsu_r;
    logic br_v, br_r;

    // ------------------------------------------------------------
    // DEBUG (rename/dispatch)
    // ------------------------------------------------------------
    rename_t dbg_rename_pkt;
    logic    dbg_rename_v, dbg_rename_r;
    logic    dbg_alu_fire, dbg_lsu_fire, dbg_br_fire;

    logic dbg_pipe_v, dbg_pipe_r, dbg_pipe_fire;
    logic dbg_alu_ins, dbg_mem_ins, dbg_br_ins;
    logic dbg_disp_fire;
    logic [3:0]  dbg_disp_tag, dbg_disp_fu;
    logic [31:0] dbg_disp_pc;

    // ============================================================
    // [CHG] NEW: Probe writeback/debug ports exported by processor
    // ============================================================
    logic       dbg_wb_valid;
    logic       dbg_wb_ready;
    wb_packet_t dbg_wb_packet;

    logic              dbg_prf_wb_en;
    logic [PREG_W-1:0] dbg_prf_wb_addr;
    logic [31:0]       dbg_prf_wb_data;

    logic              dbg_cdb_valid;
    logic [PREG_W-1:0] dbg_cdb_tag;
    logic [31:0]       dbg_cdb_data;

    logic                  dbg_rob_complete_valid;
    logic [$clog2(16)-1:0] dbg_rob_complete_idx;
    logic                  dbg_rob_complete_mispredict;

    logic        dbg_recover;
    logic        dbg_redirect_valid;
    logic [31:0] dbg_redirect_pc;

    // ============================================================
    // [CHG] Program image (fetch ROM contains these; TB uses as ref)
    //   Assumption: fetch starts at PC=0 and increments by +4.
    // ============================================================
    localparam int PROG_LEN = 24;
    logic [31:0] prog [0:PROG_LEN-1];
    initial begin
        prog[ 0] = 32'h123452b7;
        prog[ 1] = 32'h6782e293;
        prog[ 2] = 32'h00abc337;
        prog[ 3] = 32'hdef36313;
        prog[ 4] = 32'h0062f3b3;
        prog[ 5] = 32'h40628e33;
        prog[ 6] = 32'h00d00f13;
        prog[ 7] = 32'h41ee5eb3;
        prog[ 8] = 32'h400ebf93;
        prog[ 9] = 32'h0ff00413;
        prog[10] = 32'h400ebf93;
        prog[11] = 32'h0ff00413;
        prog[12] = 32'h008efbb3;
        prog[13] = 32'h001b8b93;
        prog[14] = 32'h100beb93;
        prog[15] = 32'h000084b7;
        prog[16] = 32'h3c04e493;
        prog[17] = 32'h41748c33;
        prog[18] = 32'h41ec5c33;
        prog[19] = 32'h800c3c93;
        prog[20] = 32'h009c7533;
        prog[21] = 32'h13579937;
        prog[22] = 32'h24696913;
        prog[23] = 32'h005975b3;
    end

    // ============================================================
    // [CHG] Helpers: sign-extend + expected decode for ALU op
    // ============================================================
    function automatic logic [31:0] sext(input logic [31:0] x, input int bits);
        logic [31:0] m;
        begin
            m = 32'h1 << (bits-1);
            sext = (x ^ m) - m;
        end
    endfunction

    function automatic logic [31:0] imm_i(input logic [31:0] instr);
        imm_i = sext({{20{instr[31]}}, instr[31:20]}, 32); // already 32, but keep style
    endfunction

    function automatic logic [31:0] imm_u(input logic [31:0] instr);
        imm_u = {instr[31:12], 12'b0};
    endfunction

    function automatic logic [3:0] exp_aluop_from_instr(input logic [31:0] instr);
        logic [6:0] opc;
        logic [2:0] f3;
        logic [6:0] f7;
        begin
            opc = instr[6:0];
            f3  = instr[14:12];
            f7  = instr[31:25];

            // Default to NA
            exp_aluop_from_instr = buffer_pkgs::NA;

            unique case (opc)
                7'b0110111: begin // LUI
                    exp_aluop_from_instr = buffer_pkgs::LUI;
                end

                7'b0010011: begin // OP-IMM
                    unique case (f3)
                        3'b000: exp_aluop_from_instr = buffer_pkgs::ADD;    // ADDI
                        3'b110: exp_aluop_from_instr = buffer_pkgs::ORR;    // ORI
                        3'b111: exp_aluop_from_instr = buffer_pkgs::ANDD;   // ANDI  [CHG] TB checks if you accidentally don't support it
                        3'b011: exp_aluop_from_instr = buffer_pkgs::SLTIU;  // SLTIU
                        3'b101: exp_aluop_from_instr = buffer_pkgs::RSHIFT; // SRLI/SRAI (we treat both as RSHIFT here)
                        default: exp_aluop_from_instr = buffer_pkgs::NA;
                    endcase
                end

                7'b0110011: begin // OP
                    unique case (f3)
                        3'b000: begin
                            // ADD/SUB
                            if (f7 == 7'b0100000) exp_aluop_from_instr = buffer_pkgs::SUB;
                            else                  exp_aluop_from_instr = buffer_pkgs::ADD;
                        end
                        3'b111: exp_aluop_from_instr = buffer_pkgs::ANDD;
                        3'b110: exp_aluop_from_instr = buffer_pkgs::ORR;
                        3'b101: exp_aluop_from_instr = buffer_pkgs::RSHIFT; // SRL/SRA
                        default: exp_aluop_from_instr = buffer_pkgs::NA;
                    endcase
                end

                default: exp_aluop_from_instr = buffer_pkgs::NA;
            endcase
        end
    endfunction

    // ============================================================
    // [CHG] Golden ALU result recomputed from *issued* ALU packet
    //       (so we don't need arch->phys mapping/commit)
    // ============================================================
    function automatic logic [31:0] golden_alu_result(input alu_entry_t e);
        logic [31:0] opA, opB;
        logic [4:0]  shamt;
        begin
            opA = (e.rs1_used) ? e.rs1_val : 32'b0;
            opB = (e.imm_used) ? e.imm_val : ((e.rs2_used) ? e.rs2_val : 32'b0);
            shamt = opB[4:0];

            unique case (e.aluop)
                buffer_pkgs::ADD:    golden_alu_result = opA + opB;
                buffer_pkgs::SUB:    golden_alu_result = opA - opB;
                buffer_pkgs::ANDD:   golden_alu_result = opA & opB;
                buffer_pkgs::ORR:    golden_alu_result = opA | opB;
                buffer_pkgs::RSHIFT: golden_alu_result = $signed(opA) >>> shamt;
                buffer_pkgs::SLTIU:  golden_alu_result = ($unsigned(opA) < $unsigned(opB)) ? 32'd1 : 32'd0;
                buffer_pkgs::LUI:    golden_alu_result = e.imm_val;
                default:             golden_alu_result = 32'hDEAD_BEEF; // catches unsupported decode paths
            endcase
        end
    endfunction

    // ============================================================
    // DUT
    // ============================================================
    processor dut (
        .clk_i(clk),
        .reset_i(reset),

        .data_alu_dispatch_o (alu_pkt),
        .valid_alu_dispatch_o(alu_v),
        .ready_alu_dispatch_i(alu_r),

        .data_lsu_dispatch_o (lsu_pkt),
        .valid_lsu_dispatch_o(lsu_v),
        .ready_lsu_dispatch_i(lsu_r),

        .data_br_dispatch_o  (br_pkt),
        .valid_br_dispatch_o (br_v),
        .ready_br_dispatch_i (br_r),

        // Rename debug
        .dbg_rename_valid_o  (dbg_rename_v),
        .dbg_rename_ready_o  (dbg_rename_r),
        .dbg_rename_data_o   (dbg_rename_pkt),

        // Issue debug
        .dbg_alu_issue_fire_o(dbg_alu_fire),
        .dbg_lsu_issue_fire_o(dbg_lsu_fire),
        .dbg_br_issue_fire_o (dbg_br_fire),

        // Dispatch debug
        .dbg_disp_pipe_valid_o   (dbg_pipe_v),
        .dbg_disp_pipe_ready_o   (dbg_pipe_r),
        .dbg_disp_pipe_fire_o    (dbg_pipe_fire),

        .dbg_disp_alu_insert_o   (dbg_alu_ins),
        .dbg_disp_mem_insert_o   (dbg_mem_ins),
        .dbg_disp_br_insert_o    (dbg_br_ins),

        .dbg_disp_dispatch_fire_o(dbg_disp_fire),

        .dbg_disp_tag_o          (dbg_disp_tag),
        .dbg_disp_pc_o           (dbg_disp_pc),
        .dbg_disp_fu_o           (dbg_disp_fu),

        // ============================================================
        // [CHG] hook up writeback debug ports
        // ============================================================
        .dbg_wb_valid_o              (dbg_wb_valid),
        .dbg_wb_ready_o              (dbg_wb_ready),
        .dbg_wb_packet_o             (dbg_wb_packet),

        .dbg_prf_wb_en_o             (dbg_prf_wb_en),
        .dbg_prf_wb_addr_o           (dbg_prf_wb_addr),
        .dbg_prf_wb_data_o           (dbg_prf_wb_data),

        .dbg_cdb_valid_o             (dbg_cdb_valid),
        .dbg_cdb_tag_o               (dbg_cdb_tag),
        .dbg_cdb_data_o              (dbg_cdb_data),

        .dbg_rob_complete_valid_o    (dbg_rob_complete_valid),
        .dbg_rob_complete_idx_o      (dbg_rob_complete_idx),
        .dbg_rob_complete_mispredict_o(dbg_rob_complete_mispredict),

        .dbg_recover_o               (dbg_recover),
        .dbg_redirect_valid_o        (dbg_redirect_valid),
        .dbg_redirect_pc_o           (dbg_redirect_pc)
    );

    // ------------------------------------------------------------
    // Backpressure generator
    // ------------------------------------------------------------
    // NOTE: In your processor, execute drives real ready internally.
    // These ready_*_dispatch_i ports are probe-only. We still wiggle
    // them so older wiring assumptions don't break sims.  [CHG]
    always_ff @(posedge clk) begin
        if (reset) begin
            alu_r <= 0;
            lsu_r <= 0;
            br_r  <= 0;
        end else begin
            alu_r <= ($urandom_range(0,3) != 0); // ~75%
            lsu_r <= ($urandom_range(0,1) != 0); // ~50%
            br_r  <= ($urandom_range(0,2) != 0); // ~66%
        end
    end

    // ============================================================
    // [CHG] Scoreboard state keyed by ROB tag (0..15)
    //   - We learn instr for a tag at DISPATCH time from dbg_disp_pc.
    //   - We learn actual operands/control at ISSUE time from alu_pkt.
    //   - We validate correctness at WRITEBACK time using wb_packet.
    // ============================================================
    localparam int ROB_DEPTH = 16;

    logic [31:0] tag_instr   [0:ROB_DEPTH-1]; // instr associated with tag
    bit          tag_has_ins [0:ROB_DEPTH-1];

    alu_entry_t  tag_alu_issue [0:ROB_DEPTH-1];
    bit          tag_has_issue [0:ROB_DEPTH-1];

    logic [31:0] tag_exp_val  [0:ROB_DEPTH-1];
    logic [PREG_W-1:0] tag_exp_rd [0:ROB_DEPTH-1];
    bit          tag_has_exp  [0:ROB_DEPTH-1];

    bit          tag_wb_seen  [0:ROB_DEPTH-1];

    int dispatch_count;
    int issue_count;
    int wb_count;

    // ------------------------------------------------------------
    // Dispatch tracking: bind tag -> instruction via PC
    // ------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (reset) begin
            dispatch_count <= 0;
            for (int i=0;i<ROB_DEPTH;i++) begin
                tag_has_ins[i] <= 1'b0;
                tag_instr[i]   <= '0;
            end
        end else if (dbg_pipe_fire) begin
            int idx;
            idx = dbg_disp_pc[31:2]; // pc/4

            dispatch_count++;

            if (idx < PROG_LEN) begin
                tag_has_ins[dbg_disp_tag] <= 1'b1;
                tag_instr[dbg_disp_tag]   <= prog[idx];

                $display("[%0t] DISPATCH tag=%0d pc=%h idx=%0d instr=%08x fu=%0d",
                         $time, dbg_disp_tag, dbg_disp_pc, idx, prog[idx], dbg_disp_fu);
            end else begin
                $error("[%0t] DISPATCH PC OUT OF RANGE: pc=%h idx=%0d (PROG_LEN=%0d)",
                       $time, dbg_disp_pc, idx, PROG_LEN);
            end

            // Invariant: exactly one RS insert on pipe_fire
            assert (dbg_alu_ins ^ dbg_mem_ins ^ dbg_br_ins)
                else $error("[%0t] DISPATCH DROP/DOUBLE INSERT tag=%0d pc=%h ins(A/M/B)=(%0b/%0b/%0b)",
                            $time, dbg_disp_tag, dbg_disp_pc, dbg_alu_ins, dbg_mem_ins, dbg_br_ins);
        end
    end

    // ------------------------------------------------------------
    // Issue tracking + decode checks (ALU only for this test)
    // ------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (reset) begin
            issue_count <= 0;
            for (int i=0;i<ROB_DEPTH;i++) begin
                tag_has_issue[i] <= 1'b0;
                tag_has_exp[i]   <= 1'b0;
                tag_exp_val[i]   <= '0;
                tag_exp_rd[i]    <= '0;
            end
        end else begin
            if (dbg_alu_fire) begin
                logic [3:0] tag;
                logic [31:0] instr;
                logic [3:0] exp_op;

                tag = alu_pkt.ROB_tag;

                if (!tag_has_ins[tag]) begin
                    $error("[%0t] ALU ISSUE WITHOUT DISPATCH-INSTR: tag=%0d", $time, tag);
                    instr  = 32'h0;
                    exp_op = buffer_pkgs::NA;
                end else begin
                    instr  = tag_instr[tag];
                    exp_op = exp_aluop_from_instr(instr);
                end

                // [CHG] Check decode control reaching execute
                if (alu_pkt.aluop != exp_op) begin
                    $error("[%0t] ALUOP MISMATCH tag=%0d instr=%08x exp_aluop=%0d got_aluop=%0d",
                           $time, tag, instr, exp_op, alu_pkt.aluop);
                end

                // Record issue packet and expected result
                tag_alu_issue[tag] <= alu_pkt;
                tag_has_issue[tag] <= 1'b1;

                tag_exp_val[tag]   <= golden_alu_result(alu_pkt);
                tag_exp_rd[tag]    <= alu_pkt.rd_addr;
                tag_has_exp[tag]   <= 1'b1;

                issue_count++;

                $display("[%0t] ISSUE ALU tag=%0d rd_p=%0d aluop=%0d exp=0x%08x",
                         $time, tag, alu_pkt.rd_addr, alu_pkt.aluop, golden_alu_result(alu_pkt));
            end

            // If LSU/BR ever issue during this test, print (not fatal)
            if (dbg_lsu_fire) begin
                $display("[%0t] ISSUE LSU tag=%0d (rd_p=%0d) (test focuses on ALU correctness)",
                         $time, lsu_pkt.ROB_tag, lsu_pkt.rd_addr);
            end
            if (dbg_br_fire) begin
                $display("[%0t] ISSUE BR  tag=%0d (test focuses on ALU correctness)",
                         $time, br_pkt.ROB_tag);
            end
        end
    end

    // ------------------------------------------------------------
    // Writeback tracking + value checks (from exported dbg_wb_*)
    // ------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (reset) begin
            wb_count <= 0;
            for (int i=0;i<ROB_DEPTH;i++) tag_wb_seen[i] <= 1'b0;
        end else begin
            if (dbg_wb_valid && dbg_wb_ready) begin
                logic [3:0] tag;
                tag = dbg_wb_packet.ROB_tag;

                wb_count++;
                tag_wb_seen[tag] <= 1'b1;

                // Basic sanity
                assert (dbg_wb_packet.completed)
                    else $error("[%0t] WB WITHOUT completed=1 tag=%0d packet=%p", $time, tag, dbg_wb_packet);

                // Only check ALU packets for this test (src_fu==0)
                if (dbg_wb_packet.src_fu == 2'd0) begin
                    if (!tag_has_exp[tag]) begin
                        $error("[%0t] ALU WB WITHOUT EXPECTATION (no prior ALU issue?) tag=%0d packet=%p",
                               $time, tag, dbg_wb_packet);
                    end else begin
                        // rd tag must match the issued destination physreg
                        if (dbg_wb_packet.rd_addr != tag_exp_rd[tag]) begin
                            $error("[%0t] WB RD_ADDR MISMATCH tag=%0d exp_rd_p=%0d got_rd_p=%0d",
                                   $time, tag, tag_exp_rd[tag], dbg_wb_packet.rd_addr);
                        end

                        // value must match recomputed golden result
                        if (dbg_wb_packet.rd_val !== tag_exp_val[tag]) begin
                            $error("[%0t] WB VALUE MISMATCH tag=%0d exp=0x%08x got=0x%08x instr=%08x aluop=%0d",
                                   $time, tag, tag_exp_val[tag], dbg_wb_packet.rd_val,
                                   tag_has_ins[tag] ? tag_instr[tag] : 32'h0,
                                   tag_has_issue[tag] ? tag_alu_issue[tag].aluop : 4'hF);
                        end else begin
                            $display("[%0t] WB OK  tag=%0d rd_p=%0d val=0x%08x",
                                     $time, tag, dbg_wb_packet.rd_addr, dbg_wb_packet.rd_val);
                        end
                    end
                end
            end
        end
    end

    // ------------------------------------------------------------
    // End-of-test checks
    // ------------------------------------------------------------
    initial begin
        clk = 0;
        reset = 1;

        repeat (3) @(posedge clk);
        reset = 0;

        // [CHG] Run long enough to fill ROB and see writebacks.
        // Without commit, dispatch will eventually stall at ROB_DEPTH.
        repeat (400) @(posedge clk);

        $display("------------------------------------------------");
        $display("Dispatch fires  : %0d", dispatch_count);
        $display("ALU issues      : %0d", issue_count);
        $display("WB handshakes   : %0d", wb_count);
        $display("------------------------------------------------");

        // [CHG] For every ALU-issued tag, we require an ALU writeback
        for (int t=0; t<ROB_DEPTH; t++) begin
            if (tag_has_issue[t] && !tag_wb_seen[t]) begin
                $error("MISSING WB for issued tag=%0d instr=%08x",
                       t, tag_has_ins[t] ? tag_instr[t] : 32'h0);
            end
        end

        // [CHG] Recovery should be inactive for this straight-line test
        if (dbg_recover || dbg_redirect_valid) begin
            $error("Unexpected recovery/redirect: recover=%0b redirect_valid=%0b redirect_pc=%h",
                   dbg_recover, dbg_redirect_valid, dbg_redirect_pc);
        end

        $display("TEST COMPLETE");
        $finish;
    end

endmodule
