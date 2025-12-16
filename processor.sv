`timescale 1ns / 1ps
import buffer_pkgs::*;

module processor #(
    parameter type F  = fetch_t,
    parameter type D  = decode_t,
    parameter type R  = rename_t,
    parameter type DI = dispatch_t,
    parameter type AE = alu_entry_t,
    parameter type AO = alu_out_t,
    parameter type BE = branch_entry_t,
    parameter type BO = branch_out_t,
    parameter type LE = lsu_entry_t,
    parameter type LO = lsu_out_t
) (
    input logic clk_i,
    input logic reset_i,

    // ============================================================
    // Existing debug: DISPATCH -> EXECUTE issue interface (KEEP)
    // ============================================================
    output AE    data_alu_dispatch_o,
    output logic valid_alu_dispatch_o,
    input  logic ready_alu_dispatch_i,

    output BE    data_br_dispatch_o,
    output logic valid_br_dispatch_o,
    input  logic ready_br_dispatch_i,

    output LE    data_lsu_dispatch_o,
    output logic valid_lsu_dispatch_o,
    input  logic ready_lsu_dispatch_i,

    // ---- DEBUG: Rename → Dispatch interface ----
    output logic dbg_rename_valid_o,
    output logic dbg_rename_ready_o,
    output R     dbg_rename_data_o,

    // ---- DEBUG: Dispatch → Issue events ----
    output logic dbg_alu_issue_fire_o,
    output logic dbg_lsu_issue_fire_o,
    output logic dbg_br_issue_fire_o,

    output logic dbg_disp_pipe_valid_o,
    output logic dbg_disp_pipe_ready_o,
    output logic dbg_disp_pipe_fire_o,

    output logic dbg_disp_alu_insert_o,
    output logic dbg_disp_mem_insert_o,
    output logic dbg_disp_br_insert_o,

    output logic dbg_disp_dispatch_fire_o,

    output logic [3:0]  dbg_disp_tag_o,
    output logic [31:0] dbg_disp_pc_o,
    output logic [3:0]  dbg_disp_fu_o,
    
    // debug writeback outputs (probe from TB)
    // ============================================================
    output logic       dbg_wb_valid_o,
    output logic       dbg_wb_ready_o,
    output wb_packet_t dbg_wb_packet_o,

    output logic              dbg_prf_wb_en_o,
    output logic [PREG_W-1:0] dbg_prf_wb_addr_o,
    output logic [31:0]       dbg_prf_wb_data_o,

    output logic              dbg_cdb_valid_o,
    output logic [PREG_W-1:0] dbg_cdb_tag_o,
    output logic [31:0]       dbg_cdb_data_o,

    output logic                  dbg_rob_complete_valid_o,
    output logic [$clog2(16)-1:0] dbg_rob_complete_idx_o,
    output logic                  dbg_rob_complete_mispredict_o,

    output logic        dbg_recover_o,
    output logic        dbg_redirect_valid_o,
    output logic [31:0] dbg_redirect_pc_o
);

    // -----------------------------
    // Fetch <-> Decode
    // -----------------------------
    logic fetch_decode_valid;
    logic fetch_decode_ready;
    F     fetch_o;

    // -----------------------------
    // Decode <-> Rename
    // -----------------------------
    logic decode_rename_valid;
    logic decode_rename_ready;
    D     decode_o;

    // -----------------------------
    // Rename <-> Dispatch
    // -----------------------------
    logic rename_dispatch_valid;
    logic rename_dispatch_ready;
    R     rename_o;

    // ============================================================
    // [CHG] Dispatch -> Execute issue channels (internal)
    //   - dispatch drives *_issue_valid/data
    //   - execute drives *_issue_ready (backpressure)
    //   - top-level probe ports mirror these channels
    // ============================================================
    logic alu_issue_valid;  logic alu_issue_ready;  AE alu_issue_data;
    logic br_issue_valid;   logic br_issue_ready;   BE br_issue_data;
    logic lsu_issue_valid;  logic lsu_issue_ready;  LE lsu_issue_data;

    // [CHG] Mirror internal issue channels onto existing probe ports
    assign valid_alu_dispatch_o = alu_issue_valid;     // [CHG]
    assign data_alu_dispatch_o  = alu_issue_data;      // [CHG]
    assign valid_br_dispatch_o  = br_issue_valid;      // [CHG]
    assign data_br_dispatch_o   = br_issue_data;       // [CHG]
    assign valid_lsu_dispatch_o = lsu_issue_valid;     // [CHG]
    assign data_lsu_dispatch_o  = lsu_issue_data;      // [CHG]

    // [CHG] IMPORTANT: ready comes from EXECUTE, not the probe inputs
    // (keeps correctness: RS issues only when execute can accept)
    // The *_dispatch_i ports remain for probing only; we do not use them.
    /* verilator lint_off UNUSED */
    wire _unused_ready_alu = ready_alu_dispatch_i;     // [CHG] probe-only
    wire _unused_ready_br  = ready_br_dispatch_i;      // [CHG] probe-only
    wire _unused_ready_lsu = ready_lsu_dispatch_i;     // [CHG] probe-only
    /* verilator lint_on UNUSED */

    // ============================================================
    // [CHG] Execute outputs (skid-buffer consumer side) exposed as
    //       internal top-level nets for easy waveform probing.
    //       (No writeback yet; hold ready high so skid buffers drain.)
    // ============================================================
    logic alu_wb_valid; AO alu_wb_data; logic alu_wb_ready;  // [CHG]
    logic br_wb_valid;  BO br_wb_data;  logic br_wb_ready;   // [CHG]
    logic lsu_wb_valid; LO lsu_wb_data; logic lsu_wb_ready;  // [CHG]

//    assign alu_wb_ready = 1'b1;  // [CHG] always-ready until WB integrated
//    assign br_wb_ready  = 1'b0;  //since ignoring branches 
//    assign lsu_wb_ready = 1'b1;  // [CHG]

    // -----------------------------
    // Fetch
    // -----------------------------
    fetch #(.DEPTH(64), .F(F)) my_fetch (
        .clk_i(clk_i),
        .reset_i(reset_i),

        .valid_cons_o(fetch_decode_valid),
        .ready_cons_i(fetch_decode_ready),
        .data_o(fetch_o),

        .redirect_i(1'b0),          // TODO: hook up branch redirect
        .redirect_pc_i(32'b0)
    );

    // -----------------------------
    // Decode
    // -----------------------------
    decode_module #(.F(F), .D(D)) my_decode (
        .clk_i(clk_i),
        .reset_i(reset_i),

        .valid_prod_i(fetch_decode_valid),
        .ready_prod_o(fetch_decode_ready),
        .data_i(fetch_o),

        .ready_cons_i(decode_rename_ready),
        .valid_cons_o(decode_rename_valid),
        .data_o(decode_o)
    );

    // -----------------------------
    // Rename
    // -----------------------------
    rename_module #(.D(D), .R(R), .ROB_DEPTH(16)) my_rename (
        .clk_i(clk_i),
        .rst_i(reset_i),

        .data_i(decode_o),
        .valid_prod_i(decode_rename_valid),
        .ready_prod_o(decode_rename_ready),

        .data_o(rename_o),
        .ready_cons_i(rename_dispatch_ready),
        .valid_cons_o(rename_dispatch_valid),

        // writeback / recovery interface (still tied off for now)
        .rat_recover_i(),
        .rat_recover_map_i(),
        .fl_recover_i(),
        .fl_recover_head_i(),
        .fl_recover_tail_i(),
        .fl_recover_free_count_i(),

        .ratfl_chkpt_we_o(),
        .ratfl_chkpt_tag_o(),
        .ratfl_chkpt_rat_map_o(),
        .ratfl_chkpt_fl_head_o(),
        .ratfl_chkpt_fl_tail_o(),
        .ratfl_chkpt_fl_free_count_o()
    );

    assign dbg_rename_valid_o = rename_dispatch_valid;
    assign dbg_rename_ready_o = rename_dispatch_ready;
    assign dbg_rename_data_o  = rename_o;

    // -----------------------------
    // Dispatch
    // -----------------------------
    logic recover_i;
    logic cdb_valid_i;
    logic [PREG_W-1:0] cdb_tag_i;

    logic prf_wb_en_i;
    logic [PREG_W-1:0] prf_wb_addr_i;
    logic [31:0] prf_wb_data_i;

    logic rob_complete_valid_i;
    logic [$clog2(16)-1:0] rob_complete_idx_i;
    logic rob_complete_mispredict_i;

    logic [$clog2(16)-1:0] rob_recover_tail_i;
    logic [$clog2(16):0]   rob_recover_used_i;

    logic rob_commit_ready_i;

    // Tie-offs (for now)
    assign recover_i                = 1'b0;
//    assign cdb_valid_i              = 1'b0;
//    assign cdb_tag_i                = '0;

//    assign prf_wb_en_i              = 1'b0;
//    assign prf_wb_addr_i            = '0;
//    assign prf_wb_data_i            = '0;

//    assign rob_complete_valid_i      = 1'b0;
//    assign rob_complete_idx_i        = '0;
//    assign rob_complete_mispredict_i = 1'b0;

    assign rob_recover_tail_i        = '0;
    assign rob_recover_used_i        = '0;

    // For now, set commit_ready=0 so commit never pops,
    assign rob_commit_ready_i        = 1'b0;

    dispatch_module #(.R(R), .DI(DI)) my_dispatch (
        .clk_i(clk_i),
        .rst_i(reset_i),

        // ---------------- DEBUG ----------------
        .dbg_pipe_valid_o    (dbg_disp_pipe_valid_o),
        .dbg_pipe_ready_o    (dbg_disp_pipe_ready_o),
        .dbg_pipe_fire_o     (dbg_disp_pipe_fire_o),

        .dbg_alu_insert_o    (dbg_disp_alu_insert_o),
        .dbg_mem_insert_o    (dbg_disp_mem_insert_o),
        .dbg_br_insert_o     (dbg_disp_br_insert_o),

        .dbg_dispatch_fire_o (dbg_disp_dispatch_fire_o),

        .dbg_tag_o           (dbg_disp_tag_o),
        .dbg_pc_o            (dbg_disp_pc_o),
        .dbg_fu_o            (dbg_disp_fu_o),
        // ---------------------------------------

        .data_i(rename_o),
        .valid_prod_i(rename_dispatch_valid),
        .ready_prod_o(rename_dispatch_ready),

        // recover from writeback (still tied off)
        .recover_i(recover_i),

        // wakeups from writeback (still tied off)
        .cdb_valid_i(cdb_valid_i),
        .cdb_tag_i  (cdb_tag_i),

        // update prf from writeback (still tied off)
        .prf_wb_en_i  (prf_wb_en_i),
        .prf_wb_addr_i(prf_wb_addr_i),
        .prf_wb_data_i(prf_wb_data_i),

        // update ROB from writeback (still tied off)
        .rob_complete_valid_i     (rob_complete_valid_i),
        .rob_complete_idx_i       (rob_complete_idx_i),
        .rob_complete_mispredict_i(rob_complete_mispredict_i),

        // ROB recovery from controller in writeback (still tied off)
        .rob_recover_tail_i(rob_recover_tail_i),
        .rob_recover_used_i(rob_recover_used_i),

        // ROB checkpoint to controller after predicting (for bne)
        .rob_chkpt_we_o(),
        .rob_chkpt_tag_o(),
        .rob_chkpt_tail_o(),
        .rob_chkpt_used_o(),

        // ROB to update commit
        .rob_commit_ready_i(rob_commit_ready_i),
        .rob_commit_valid_o(),
        .rob_commit_entry_o(),
        .rob_commit_index_o(),

        // ========================================================
        // [CHG] Issue outputs now go into internal issue channels
        // ========================================================
        .alu_issue_valid_o(alu_issue_valid),   // [CHG]
        .alu_issue_ready_i(alu_issue_ready),   // [CHG] from execute
        .alu_issue_o      (alu_issue_data),    // [CHG]

        .lsu_issue_valid_o(lsu_issue_valid),   // [CHG]
        .lsu_issue_ready_i(lsu_issue_ready),   // [CHG] from execute
        .lsu_issue_o      (lsu_issue_data),    // [CHG]

        .br_issue_valid_o (br_issue_valid),    // [CHG]
        .br_issue_ready_i (br_issue_ready),    // [CHG] from execute
        .br_issue_o       (br_issue_data)      // [CHG]
    );

    // [CHG] Fire events should use the real ready (from execute)
    assign dbg_alu_issue_fire_o = alu_issue_valid && alu_issue_ready; // [CHG]
    assign dbg_lsu_issue_fire_o = lsu_issue_valid && lsu_issue_ready; // [CHG]
    assign dbg_br_issue_fire_o  = br_issue_valid  && br_issue_ready;  // [CHG]

    // -----------------------------
    // Execute
    // -----------------------------
    execute_module #(.AE(AE), .AO(AO), .BE(BE), .BO(BO), .LE(LE), .LO(LO)) my_execute (
        .clk_i(clk_i),
        .reset_i(reset_i),

        // alu inputs
        .alu_valid_i(alu_issue_valid),
        .alu_ready_o(alu_issue_ready),   // [CHG] drives dispatch issue_ready
        .alu_data_i (alu_issue_data),
        .alu_flush_i(1'b0),

        // branch inputs
        .b_valid_i  (br_issue_valid),
        .b_ready_o  (br_issue_ready),    // [CHG] drives dispatch issue_ready
        .b_data_i   (br_issue_data),
        .b_flush_i  (1'b0),

        // lsu inputs
        .lsu_valid_i(lsu_issue_valid),
        .lsu_ready_o(lsu_issue_ready),   // [CHG] drives dispatch issue_ready
        .lsu_data_i (lsu_issue_data),
        .lsu_flush_i(1'b0),

        // ========================================================
        // [CHG] Expose execute outputs as internal nets for probing
        //       (and keep skid buffers draining with ready=1)
        // ========================================================
        .asb_cons_ready_i(alu_wb_ready),     // [CHG]
        .asb_cons_valid_o(alu_wb_valid),     // [CHG]
        .asb_cons_data_o (alu_wb_data),      // [CHG]

        .bsb_cons_ready_i(br_wb_ready),      // [CHG]
        .bsb_cons_valid_o(br_wb_valid),      // [CHG]
        .bsb_cons_data_o (br_wb_data),       // [CHG]

        .lsb_cons_ready_i(lsu_wb_ready),     // [CHG]
        .lsb_cons_valid_o(lsu_wb_valid),     // [CHG]
        .lsb_cons_data_o (lsu_wb_data)       // [CHG]
    );
    
    
    // Execute -> Writeback arbitration
    // -----------------------------
    logic       wb_valid;
    wb_packet_t wb_packet;
    logic       wb_ready;   // from writeback_module (it is always 1 in your design)
    
    // simple fixed-priority (LSU > ALU > BR) - BR unused in your subset
    typedef enum logic [1:0] {SRC_ALU=2'd0, SRC_LSU=2'd1, SRC_BR=2'd2} wb_src_t;
    wb_src_t wb_src_sel;
    
    always_comb begin
        // pick one producer
        if (lsu_wb_valid)      wb_src_sel = SRC_LSU;
        else if (alu_wb_valid) wb_src_sel = SRC_ALU;
        else if (br_wb_valid)  wb_src_sel = SRC_BR;
        else                   wb_src_sel = SRC_ALU; // don't care
    
        wb_valid  = (lsu_wb_valid || alu_wb_valid || br_wb_valid);
    
        // backpressure: only the selected skid buffer may pop
        alu_wb_ready = wb_ready && alu_wb_valid && (wb_src_sel == SRC_ALU);
        lsu_wb_ready = wb_ready && lsu_wb_valid && (wb_src_sel == SRC_LSU);
        br_wb_ready  = wb_ready && br_wb_valid && (wb_src_sel == SRC_BR);
    
        // default packet
        wb_packet = '0;
    
        // fill from selected source
        unique case (wb_src_sel)
            SRC_ALU: begin
                wb_packet.src_fu    = 2'd0;
                wb_packet.completed = 1;   // or just 1'b1 if your AO always means "done"
                wb_packet.ROB_tag   = alu_wb_data.ROB_tag;
                wb_packet.rd_addr   = alu_wb_data.rd_addr;
                wb_packet.rd_val    = alu_wb_data.rd_val;
            end
    
            SRC_LSU: begin
                wb_packet.src_fu    = 2'd1;
                wb_packet.completed = 1;
                wb_packet.ROB_tag   = lsu_wb_data.ROB_tag;
                wb_packet.rd_addr   = lsu_wb_data.rd_addr;     // for SW/SH this should be '0 in your LSU output
                wb_packet.rd_val    = lsu_wb_data.rd_val;      // for stores can be don't-care
            end
    
            SRC_BR: begin
                wb_packet.src_fu       = 2'd2;
                wb_packet.completed    = 1;
                wb_packet.ROB_tag      = br_wb_data.ROB_tag;
                wb_packet.rd_addr      = br_wb_data.rd_addr;   // usually x0 for branches
                wb_packet.rd_val       = br_wb_data.rd_val;
                wb_packet.branch_taken = br_wb_data.branch_taken;
                wb_packet.dest_addr    = br_wb_data.dest_addr;
                wb_packet.mispredict   = br_wb_data.mispredict;
            end
        endcase
    end
    
    
    // -----------------------------
    // Writeback
    // -----------------------------
    logic prf_wb_en_o;
    logic [PREG_W-1:0] prf_wb_addr_o;
    logic [31:0]       prf_wb_data_o;
    
    logic cdb_valid_o;
    logic [PREG_W-1:0] cdb_tag_o;
    logic [31:0]       cdb_data_o;
    
    logic rob_complete_valid_o;
    logic [$clog2(16)-1:0] rob_complete_idx_o;
    logic rob_complete_mispredict_o;
    
    // ignore recovery for now
    logic recover_o, redirect_valid_o;
    logic [31:0] redirect_pc_o;
    logic rob_recover_o, rat_recover_o, fl_recover_o;
    logic [$clog2(16)-1:0] rob_recover_tail_o;
    logic [$clog2(16)  :0] rob_recover_used_o;
    logic [32*PREG_W-1:0] rat_recover_map_o;
    logic [PREG_W-1:0] fl_recover_head_o, fl_recover_tail_o;
    logic [$clog2(PREGS):0] fl_recover_free_count_o;
    
    writeback_module #(.ROB_DEPTH(16), .AREG(32)) my_wb (
        .clk_i(clk_i),
        .rst_i(reset_i),
    
        .wb_valid_i (wb_valid),
        .wb_packet_i(wb_packet),
        .wb_ready_o (wb_ready),
    
        // if you truly ignore recovery, these can be tied to 0
        .ratfl_chkpt_we_i(1'b0),
        .ratfl_chkpt_tag_i('0),
        .ratfl_chkpt_rat_map_i('0),
        .ratfl_chkpt_fl_head_i('0),
        .ratfl_chkpt_fl_tail_i('0),
        .ratfl_chkpt_fl_free_count_i('0),
    
        .rob_chkpt_we_i(1'b0),
        .rob_chkpt_tag_i('0),
        .rob_chkpt_tail_i('0),
        .rob_chkpt_used_i('0),
    
        .recover_o(recover_o),
        .redirect_valid_o(redirect_valid_o),
        .redirect_pc_o(redirect_pc_o),
    
        .rob_recover_o(rob_recover_o),
        .rob_recover_tail_o(rob_recover_tail_o),
        .rob_recover_used_o(rob_recover_used_o),
    
        .rat_recover_o(rat_recover_o),
        .rat_recover_map_o(rat_recover_map_o),
    
        .fl_recover_o(fl_recover_o),
        .fl_recover_head_o(fl_recover_head_o),
        .fl_recover_tail_o(fl_recover_tail_o),
        .fl_recover_free_count_o(fl_recover_free_count_o),
    
        .recover_rob_tag_o(),
    
        .prf_wb_en_o  (prf_wb_en_o),
        .prf_wb_addr_o(prf_wb_addr_o),
        .prf_wb_data_o(prf_wb_data_o),
    
        .cdb_valid_o(cdb_valid_o),
        .cdb_tag_o  (cdb_tag_o),
        .cdb_data_o (cdb_data_o),
    
        .rob_complete_valid_o     (rob_complete_valid_o),
        .rob_complete_idx_o       (rob_complete_idx_o),
        .rob_complete_mispredict_o(rob_complete_mispredict_o)
    );
    
    assign cdb_valid_i         = cdb_valid_o;
    assign cdb_tag_i           = cdb_tag_o;
    
    assign prf_wb_en_i         = prf_wb_en_o;
    assign prf_wb_addr_i       = prf_wb_addr_o;
    assign prf_wb_data_i       = prf_wb_data_o;
    
    assign rob_complete_valid_i      = rob_complete_valid_o;
    assign rob_complete_idx_i        = rob_complete_idx_o;
    assign rob_complete_mispredict_i = rob_complete_mispredict_o;
    
    // ============================================================
    // [CHG] Mirror internal WB signals to top-level debug outputs
    // ============================================================
    assign dbg_wb_valid_o   = wb_valid;
    assign dbg_wb_ready_o   = wb_ready;
    assign dbg_wb_packet_o  = wb_packet;

    assign dbg_prf_wb_en_o   = prf_wb_en_o;
    assign dbg_prf_wb_addr_o = prf_wb_addr_o;
    assign dbg_prf_wb_data_o = prf_wb_data_o;

    assign dbg_cdb_valid_o = cdb_valid_o;
    assign dbg_cdb_tag_o   = cdb_tag_o;
    assign dbg_cdb_data_o  = cdb_data_o;

    assign dbg_rob_complete_valid_o      = rob_complete_valid_o;
    assign dbg_rob_complete_idx_o        = rob_complete_idx_o;
    assign dbg_rob_complete_mispredict_o = rob_complete_mispredict_o;

    assign dbg_recover_o        = recover_o;
    assign dbg_redirect_valid_o = redirect_valid_o;
    assign dbg_redirect_pc_o    = redirect_pc_o;
    


endmodule
