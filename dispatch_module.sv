`timescale 1ns / 1ps
import buffer_pkgs::*;

module dispatch_module #(
    parameter type R  = rename_t,
    parameter type DI = rs_entry_t
)(
    input  logic clk_i,
    input  logic rst_i,

    // GLOBAL RECOVERY
    input  logic recover_i,

    // From rename
    // =========================
    // DEBUG / DIAGNOSTICS PORTS
    // =========================
    output logic dbg_pipe_valid_o,
    output logic dbg_pipe_ready_o,
    output logic dbg_pipe_fire_o,

    output logic dbg_alu_insert_o,
    output logic dbg_mem_insert_o,
    output logic dbg_br_insert_o,

    output logic dbg_dispatch_fire_o,

    output logic [3:0] dbg_tag_o,
    output logic [31:0] dbg_pc_o,
    output logic [3:0] dbg_fu_o,

    input  R     data_i,
    input  logic valid_prod_i,
    output logic ready_prod_o,

    // CDB (wakeups)
    input  logic              cdb_valid_i,
    input  logic [PREG_W-1:0] cdb_tag_i,

    // PRF write port (from WB)
    input  logic              prf_wb_en_i,
    input  logic [PREG_W-1:0] prf_wb_addr_i,
    input  logic [31:0]       prf_wb_data_i,

    // ROB completion (from WB)
    input  logic                  rob_complete_valid_i,
    input  logic [$clog2(16)-1:0] rob_complete_idx_i,
    input  logic                  rob_complete_mispredict_i,

    // ROB recovery payload (from controller)
    input  logic [$clog2(16)-1:0] rob_recover_tail_i,
    input  logic [$clog2(16)  :0] rob_recover_used_i,

    // ROB checkpoint write -> controller
    output logic                  rob_chkpt_we_o,
    output logic [3:0]            rob_chkpt_tag_o,
    output logic [$clog2(16)-1:0] rob_chkpt_tail_o,
    output logic [$clog2(16)  :0] rob_chkpt_used_o,

    // ROB -> Commit stage interface
    input  logic                  rob_commit_ready_i,
    output logic                  rob_commit_valid_o,
    output rob_entry_t            rob_commit_entry_o,
    output logic [$clog2(16)-1:0] rob_commit_index_o,

    // Issue outputs
    output logic         alu_issue_valid_o,
    input  logic         alu_issue_ready_i,
    output alu_entry_t   alu_issue_o,

    output logic         lsu_issue_valid_o,
    input  logic         lsu_issue_ready_i,
    output lsu_entry_t   lsu_issue_o,

    output logic          br_issue_valid_o,
    input  logic          br_issue_ready_i,
    output branch_entry_t br_issue_o
);

    // ----------------------------
    // Pipe buffer between rename and dispatch
    // ----------------------------
    R     buffer_o;
    logic buffer_valid_o;
    logic buffer_ready_i;

    pipe_buffer #(.T(R), .DEPTH(2)) u_pipe (
        .clk_i     (clk_i),
        .rst_i     (rst_i),
        .flush_i   (recover_i),

        .data_in   (data_i),
        .valid_in  (valid_prod_i),
        .ready_in  (ready_prod_o),
        .data_out  (buffer_o),
        .valid_out (buffer_valid_o),
        .ready_out (buffer_ready_i)
    );

    // ============================================================
    // Operand-usage decode
    // ============================================================
    function automatic logic calc_rs1_used(input logic [6:0] opc);
        unique case (opc)
            7'b0110111: calc_rs1_used = 1'b0; // LUI
            7'b0010111: calc_rs1_used = 1'b0; // AUIPC
            7'b1101111: calc_rs1_used = 1'b0; // JAL
            default:    calc_rs1_used = 1'b1;
        endcase
    endfunction

    function automatic logic calc_rs2_used(input logic [6:0] opc, input logic imm_used);
        unique case (opc)
            7'b0110011: calc_rs2_used = 1'b1;           // OP
            7'b1100011: calc_rs2_used = 1'b1;           // BRANCH
            7'b0100011: calc_rs2_used = 1'b1;           // STORE
            7'b0010011: calc_rs2_used = 1'b0;           // OP-IMM
            7'b0000011: calc_rs2_used = 1'b0;           // LOAD
            7'b1100111: calc_rs2_used = 1'b0;           // JALR
            7'b1101111: calc_rs2_used = 1'b0;           // JAL
            7'b0110111: calc_rs2_used = 1'b0;           // LUI
            7'b0010111: calc_rs2_used = 1'b0;           // AUIPC
            default:    calc_rs2_used = (!imm_used);
        endcase
    endfunction

    logic rs1_used_d, rs2_used_d;
    always_comb begin
        rs1_used_d = buffer_valid_o ? calc_rs1_used(buffer_o.opcode) : 1'b0;
        rs2_used_d = buffer_valid_o ? calc_rs2_used(buffer_o.opcode, buffer_o.imm_used) : 1'b0;
    end

    // ============================================================
    // Physreg busy scoreboard
    //   [CHG] Restore from ROB pending-dest bitmap on recover
    // ============================================================
    logic [PREGS-1:0] preg_busy;
    logic [PREGS-1:0] rob_busy_pregs;      // [CHG]

    logic dispatch_fire;

    // ----------------------------
    // PRF read
    // ----------------------------
    localparam int XLEN = 32;
    logic [XLEN-1:0] prf_rs1_val, prf_rs2_val;

    prf #(.NREGS(buffer_pkgs::PREGS), .XLEN(XLEN)) u_prf (
        .clk_i      (clk_i),
        .rst_i      (rst_i),
        .wb_en_i    (prf_wb_en_i),
        .wb_addr_i  (prf_wb_addr_i),
        .wb_data_i  (prf_wb_data_i),

        .rs1_i      (buffer_valid_o ? buffer_o.rs1 : '0),
        .rs1_used_i (buffer_valid_o ? rs1_used_d : 1'b0),
        .rs1_data_o (prf_rs1_val),

        .rs2_i      (buffer_valid_o ? buffer_o.rs2 : '0),
        .rs2_used_i (buffer_valid_o ? rs2_used_d : 1'b0),
        .rs2_data_o (prf_rs2_val)
    );

    // ----------------------------
    // ROB
    // ----------------------------
    localparam int ROB_DEPTH = 16;
    localparam int ROB_PTR_W = $clog2(ROB_DEPTH);

    logic rob_alloc_ready;
    logic [ROB_PTR_W-1:0] rob_chkpt_tail_int;
    logic [ROB_PTR_W  :0] rob_chkpt_used_int;

    logic                  rob_commit_valid_int;
    rob_entry_t            rob_commit_entry_int;
    logic [ROB_PTR_W-1:0]  rob_commit_index_int;
    
    logic [ROB_PTR_W-1:0] rob_alloc_idx;

    rob_module #(.DEPTH(ROB_DEPTH), .PREGS(buffer_pkgs::PREGS)) u_rob (  // [CHG]
        .clk_i                (clk_i),
        .rst_i                (rst_i),
        .data_i               (buffer_o),

        .alloc_valid_i        (dispatch_fire),
        .alloc_ready_o        (rob_alloc_ready),

        .alloc_has_dest_i     (buffer_o.rd_used && (buffer_o.arch_rd != 5'd0) && (buffer_o.rd != '0)),
        .alloc_arch_rd_i      (buffer_o.arch_rd),
        .alloc_dest_preg_i    (buffer_o.rd),
        .alloc_old_preg_i     (buffer_o.old_rd),
        .alloc_is_branch_i    (buffer_o.branch || buffer_o.jump),
        .alloc_pc_i           (buffer_o.pc),
        .alloc_rob_index_o    (rob_alloc_idx),

        .chkpt_tail_o         (rob_chkpt_tail_int),
        .chkpt_used_count_o   (rob_chkpt_used_int),

        .recover_i            (recover_i),
        .recover_tail_i       (rob_recover_tail_i),
        .recover_used_count_i (rob_recover_used_i),

        .complete_valid_i      (rob_complete_valid_i),
        .complete_rob_index_i  (rob_complete_idx_i),
        .complete_mispredict_i (rob_complete_mispredict_i),

        .commit_ready_i        (rob_commit_ready_i),
        .commit_valid_o        (rob_commit_valid_int),
        .commit_entry_o        (rob_commit_entry_int),
        .commit_rob_index_o    (rob_commit_index_int),

        .busy_pregs_o          (rob_busy_pregs)          // [CHG]
    );

    assign rob_commit_valid_o = rob_commit_valid_int;
    assign rob_commit_entry_o = rob_commit_entry_int;
    assign rob_commit_index_o = rob_commit_index_int;

    // [CHG] Scoreboard update with correct recovery behavior
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            preg_busy <= '0;
        end else if (recover_i) begin
            preg_busy <= rob_busy_pregs;  // [CHG] restore pending producers
        end else begin
            // clear then set (set wins on same tag)
            if (cdb_valid_i && (cdb_tag_i != '0)) begin
                preg_busy[cdb_tag_i] <= 1'b0;
            end
            if (dispatch_fire && buffer_o.rd_used && (buffer_o.arch_rd != 5'd0) && (buffer_o.rd != '0)) begin
                preg_busy[buffer_o.rd] <= 1'b1;
            end
        end
    end

    // ----------------------------
    // Build RS entry (unchanged)
    // ----------------------------
    DI disp_entry;

    always_comb begin
        disp_entry = '0;

        if (buffer_valid_o) begin
            disp_entry.valid = 1'b1;

            disp_entry.rs1    = buffer_o.rs1;
            disp_entry.rs2    = buffer_o.rs2;
            disp_entry.rd     = buffer_o.rd;
            disp_entry.old_rd = buffer_o.old_rd;

            disp_entry.ROB_tag = dispatch_fire ? rob_alloc_idx : '0;   // instead of buffer_o.ROB_tag

            disp_entry.pc   = buffer_o.pc;
            disp_entry.opcode = buffer_o.opcode;
            disp_entry.imm  = buffer_o.imm;
            disp_entry.imm_used = buffer_o.imm_used;
            disp_entry.rd_used  = buffer_o.rd_used;
            disp_entry.aluop    = buffer_o.aluop;
            disp_entry.funcU    = buffer_o.funcU;
            disp_entry.load_store_type = buffer_o.load_store_type;
            disp_entry.jump  = buffer_o.jump;
            disp_entry.branch= buffer_o.branch;
            disp_entry.speculative = buffer_o.speculative;

            disp_entry.rs1_val = prf_rs1_val;
            disp_entry.rs2_val = prf_rs2_val;

            disp_entry.rs1_tag = buffer_o.rs1;
            disp_entry.rs2_tag = buffer_o.rs2;

            disp_entry.rs1_rdy = (!rs1_used_d) ? 1'b1
                             : (buffer_o.rs1 == '0) ? 1'b1
                             : !preg_busy[buffer_o.rs1];

            disp_entry.rs2_rdy = (!rs2_used_d) ? 1'b1
                             : (buffer_o.rs2 == '0) ? 1'b1
                             : !preg_busy[buffer_o.rs2];
        end
    end

    // ----------------------------
    // RS instances + steering (unchanged)
    // ----------------------------
    FUNCU which_RS;
    assign which_RS = FUNCU'(buffer_o.funcU);

    logic alu_disp_valid, alu_disp_ready;
    logic mem_disp_valid, mem_disp_ready;
    logic br_disp_valid,  br_disp_ready;

    DI    alu_issue_entry, mem_issue_entry, br_issue_entry;
    logic alu_issue_valid, mem_issue_valid, br_issue_valid;

    reservation_station #(.RS_ENTRY_T(DI), .RS_DEPTH(8)) u_alu_rs (
        .clk_i        (clk_i),
        .rst_i        (rst_i),
        .recover_i    (recover_i),
        .disp_valid_i (alu_disp_valid),
        .disp_ready_o (alu_disp_ready),
        .disp_entry_i (disp_entry),
        .issue_valid_o(alu_issue_valid),
        .issue_ready_i(alu_issue_ready_i),
        .issue_entry_o(alu_issue_entry),
        .cdb_valid_i  (cdb_valid_i),
        .cdb_tag_i    (cdb_tag_i)
    );

    reservation_station #(.RS_ENTRY_T(DI), .RS_DEPTH(8)) u_mem_rs (
        .clk_i        (clk_i),
        .rst_i        (rst_i),
        .recover_i    (recover_i),
        .disp_valid_i (mem_disp_valid),
        .disp_ready_o (mem_disp_ready),
        .disp_entry_i (disp_entry),
        .issue_valid_o(mem_issue_valid),
        .issue_ready_i(lsu_issue_ready_i),
        .issue_entry_o(mem_issue_entry),
        .cdb_valid_i  (cdb_valid_i),
        .cdb_tag_i    (cdb_tag_i)
    );

    reservation_station #(.RS_ENTRY_T(DI), .RS_DEPTH(8)) u_br_rs (
        .clk_i        (clk_i),
        .rst_i        (rst_i),
        .recover_i    (recover_i),
        .disp_valid_i (br_disp_valid),
        .disp_ready_o (br_disp_ready),
        .disp_entry_i (disp_entry),
        .issue_valid_o(br_issue_valid),
        .issue_ready_i(br_issue_ready_i),
        .issue_entry_o(br_issue_entry),
        .cdb_valid_i  (cdb_valid_i),
        .cdb_tag_i    (cdb_tag_i)
    );

    logic can_dispatch;

    always_comb begin
        alu_disp_valid = 1'b0;
        mem_disp_valid = 1'b0;
        br_disp_valid  = 1'b0;
        can_dispatch   = 1'b0;

        if (!recover_i && buffer_valid_o && rob_alloc_ready) begin
            unique case (which_RS)
                ALU:    if (alu_disp_ready) begin can_dispatch=1'b1; alu_disp_valid=1'b1; end
                MEM:    if (mem_disp_ready) begin can_dispatch=1'b1; mem_disp_valid=1'b1; end
                BRANCH: if (br_disp_ready)  begin can_dispatch=1'b1; br_disp_valid =1'b1; end
                default: can_dispatch = 1'b0;
            endcase
        end
    end

    assign buffer_ready_i = can_dispatch;
    assign dispatch_fire  = buffer_valid_o && can_dispatch && !recover_i;
    
    // ----------------------------
    // DEBUG SIGNALS
    // ----------------------------
    assign dbg_pipe_valid_o = buffer_valid_o;
    assign dbg_pipe_ready_o = buffer_ready_i;

    // This is the true "we popped the pipe buffer" event
    assign dbg_pipe_fire_o  = buffer_valid_o && buffer_ready_i && !recover_i;

    // Tag/pc/fu associated with the current pipe head (when valid)
    assign dbg_tag_o = disp_entry.ROB_tag;
    assign dbg_pc_o  = buffer_o.pc;
    assign dbg_fu_o  = buffer_o.funcU;

    // Insert pulses (these should be mutually exclusive)
    assign dbg_alu_insert_o = alu_disp_valid && alu_disp_ready && buffer_valid_o && !recover_i;
    assign dbg_mem_insert_o = mem_disp_valid && mem_disp_ready && buffer_valid_o && !recover_i;
    assign dbg_br_insert_o  = br_disp_valid  && br_disp_ready  && buffer_valid_o && !recover_i;

    assign dbg_dispatch_fire_o = dispatch_fire;
    
    `ifdef DISPATCH_ASSERTS
        // If we consumed a pipe entry, we must have inserted into exactly one RS
        always_ff @(posedge clk_i) begin
            if (!rst_i && !recover_i) begin
                if (dbg_pipe_fire_o) begin
                    assert (dbg_alu_insert_o ^ dbg_mem_insert_o ^ dbg_br_insert_o)
                        else $error("[%0t] DISPATCH DROP/DOUBLE-INSERT: pc=%h tag=%0d fu=%0d  ins_alu=%0b ins_mem=%0b ins_br=%0b",
                                    $time, dbg_pc_o, dbg_tag_o, dbg_fu_o,
                                    dbg_alu_insert_o, dbg_mem_insert_o, dbg_br_insert_o);
                end
            end
        end
    
        // dispatch_fire should match pipe_fire exactly
        always_ff @(posedge clk_i) begin
            if (!rst_i && !recover_i) begin
                assert (dbg_pipe_fire_o == dbg_dispatch_fire_o)
                    else $error("[%0t] FIRE MISMATCH: pipe_fire=%0b dispatch_fire=%0b pc=%h tag=%0d",
                                $time, dbg_pipe_fire_o, dbg_dispatch_fire_o, dbg_pc_o, dbg_tag_o);
            end
        end
    `endif



    // ----------------------------
    // ROB checkpoint write to controller (unchanged)
    // ----------------------------
    always_comb begin
        rob_chkpt_we_o   = 1'b0;
        rob_chkpt_tag_o  = '0;
        rob_chkpt_tail_o = '0;
        rob_chkpt_used_o = '0;

        if (dispatch_fire && (buffer_o.branch || buffer_o.jump)) begin
            rob_chkpt_we_o  = 1'b1;
            rob_chkpt_tag_o = rob_alloc_idx; 

            rob_chkpt_tail_o = rob_chkpt_tail_int + ROB_PTR_W'(1);
            rob_chkpt_used_o = rob_chkpt_used_int + (ROB_PTR_W+1)'(1);
        end
    end

    // ----------------------------
    // FU issue conversion (unchanged)
    // ----------------------------
    assign alu_issue_valid_o = alu_issue_valid;
    always_comb begin
        alu_issue_o = '0;
        alu_issue_o.rd_addr  = alu_issue_entry.rd;
        alu_issue_o.rs1_val  = alu_issue_entry.rs1_val;
        alu_issue_o.rs1_used = calc_rs1_used(alu_issue_entry.opcode);
        alu_issue_o.rs2_val  = alu_issue_entry.rs2_val;
        alu_issue_o.rs2_used = calc_rs2_used(alu_issue_entry.opcode, alu_issue_entry.imm_used);
        alu_issue_o.imm_val  = alu_issue_entry.imm;
        alu_issue_o.imm_used = alu_issue_entry.imm_used;
        alu_issue_o.aluop    = alu_issue_entry.aluop;
        alu_issue_o.ROB_tag  = alu_issue_entry.ROB_tag;
    end

    assign lsu_issue_valid_o = mem_issue_valid;
    always_comb begin
        lsu_issue_o = '0;
        lsu_issue_o.rd_addr   = mem_issue_entry.rd;
        lsu_issue_o.rs1_val   = mem_issue_entry.rs1_val;
        lsu_issue_o.rs1_used  = 1'b1;
        lsu_issue_o.rs2_val   = mem_issue_entry.rs2_val;
        lsu_issue_o.rs2_used  = (mem_issue_entry.load_store_type[1] == STORE);
        lsu_issue_o.imm_val   = mem_issue_entry.imm;
        lsu_issue_o.imm_used  = 1'b1;
        lsu_issue_o.lsType    = mem_issue_entry.load_store_type[0];
        lsu_issue_o.loadStore = mem_issue_entry.load_store_type[1];
        lsu_issue_o.pc        = mem_issue_entry.pc;
        lsu_issue_o.ROB_tag   = mem_issue_entry.ROB_tag;
    end

    assign br_issue_valid_o = br_issue_valid;
    always_comb begin
        br_issue_o = '0;
        br_issue_o.rd_addr     = br_issue_entry.rd;
        br_issue_o.rs1_val     = br_issue_entry.rs1_val;
        br_issue_o.rs1_used    = calc_rs1_used(br_issue_entry.opcode);
        br_issue_o.rs2_val     = br_issue_entry.rs2_val;
        br_issue_o.rs2_used    = (br_issue_entry.opcode == 7'b1100011);
        br_issue_o.imm_val     = br_issue_entry.imm;
        br_issue_o.imm_used    = br_issue_entry.imm_used;
        br_issue_o.jump        = br_issue_entry.jump;
        br_issue_o.branch      = br_issue_entry.branch;
        br_issue_o.pc          = br_issue_entry.pc;
        br_issue_o.took_branch = 1'b0;
        br_issue_o.ROB_tag     = br_issue_entry.ROB_tag;
    end

endmodule
