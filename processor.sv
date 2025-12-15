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

    // debug / monitor
//    output logic valid_dispatch_o,
//    input  logic ready_dispatch_i

//    output R data_rename_o,
//    output logic valid_rename_o,
//    input logic ready_rename_i

    output AE data_alu_dispatch_o,
    output logic valid_alu_dispatch_o,
    input logic ready_alu_dispatch_i,
    
    output BE data_br_dispatch_o,
    output logic valid_br_dispatch_o,
    input logic ready_br_dispatch_i,
    
    output LE data_lsu_dispatch_o,
    output logic valid_lsu_dispatch_o,
    input logic ready_lsu_dispatch_i,
    
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
    output logic [3:0]  dbg_disp_fu_o
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

//    // ============================================================
//    // [CHG] Dispatch <-> Execute issue channels
//    // ============================================================
//    logic alu_issue_valid;  logic alu_issue_ready;  AE alu_issue_data;
//    logic br_issue_valid;   logic br_issue_ready;   BE br_issue_data;
//    logic mem_issue_valid;  logic mem_issue_ready;  LE mem_issue_data;

//    // ============================================================
//    // [CHG] Execute skid outputs -> Dispatch (WB inputs)
//    // ============================================================
//    logic alu_wb_valid; AO alu_wb_data; logic alu_wb_ready;
//    logic br_wb_valid;  BO br_wb_data;  logic br_wb_ready;
//    logic lsu_wb_valid; LO lsu_wb_data; logic lsu_wb_ready;

//    // ============================================================
//    // [CHG] CDB signals (exported by dispatch, fed into rename)
//    // ============================================================
//    logic       cdb_valid;
//    logic [3:0] cdb_tag;
//    logic [31:0] cdb_data; // optional if you later want rename to take values; rename currently only needs tag

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

//    // -----------------------------
//    // Decode
//    // -----------------------------
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

//    // -----------------------------
//    // Rename
//    // -----------------------------
    rename_module #(.D(D), .R(R), .ROB_DEPTH(16)) my_rename (
        .clk_i(clk_i),
        .rst_i(reset_i),

        .data_i(decode_o),
        .valid_prod_i(decode_rename_valid),
        .ready_prod_o(decode_rename_ready),

        .data_o(rename_o),
        .ready_cons_i(rename_dispatch_ready),
        .valid_cons_o(rename_dispatch_valid),
//        .data_o(data_rename_o), //for debugging
//        .ready_cons_i(ready_rename_i),
//        .valid_cons_o(valid_rename_o),

        //writeback interface 
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
    assign recover_i               = 1'b0;
    assign cdb_valid_i             = 1'b0;
    assign cdb_tag_i               = '0;
    
    assign prf_wb_en_i             = 1'b0;
    assign prf_wb_addr_i           = '0;
    assign prf_wb_data_i           = '0;
    
    assign rob_complete_valid_i     = 1'b0;
    assign rob_complete_idx_i       = '0;
    assign rob_complete_mispredict_i= 1'b0;
    
    assign rob_recover_tail_i       = '0;
    assign rob_recover_used_i       = '0;
    // For now, set commit_ready=0 so commit never pops,
    assign rob_commit_ready_i       = 1'b0;
    
    dispatch_module #(.R(R), .DI(DI)) my_dispatch (
        .clk_i(clk_i),
        .rst_i(reset_i),

        // ---------------- DEBUG ----------------
        .dbg_pipe_valid_o   (dbg_disp_pipe_valid_o),
        .dbg_pipe_ready_o   (dbg_disp_pipe_ready_o),
        .dbg_pipe_fire_o    (dbg_disp_pipe_fire_o),

        .dbg_alu_insert_o   (dbg_disp_alu_insert_o),
        .dbg_mem_insert_o   (dbg_disp_mem_insert_o),
        .dbg_br_insert_o    (dbg_disp_br_insert_o),

        .dbg_dispatch_fire_o(dbg_disp_dispatch_fire_o),

        .dbg_tag_o          (dbg_disp_tag_o),
        .dbg_pc_o           (dbg_disp_pc_o),
        .dbg_fu_o           (dbg_disp_fu_o),
        // ---------------------------------------

        .data_i(rename_o),
        .valid_prod_i(rename_dispatch_valid),
        .ready_prod_o(rename_dispatch_ready),
        
        //recover from writeback
        .recover_i(recover_i),
        
        //wakeups from writeback
        .cdb_valid_i(cdb_valid_i),
        .cdb_tag_i(cdb_tag_i),
        
        //update prf from writeback
        .prf_wb_en_i(prf_wb_en_i),
        .prf_wb_addr_i(prf_wb_addr_i),
        .prf_wb_data_i(prf_wb_data_i),
        
        //update ROB from writeback
        .rob_complete_valid_i(rob_complete_valid_i),
        .rob_complete_idx_i(rob_complete_idx_i),
        .rob_complete_mispredict_i(rob_complete_mispredict_i),
        
        //ROB recovery from controller in writeback
        .rob_recover_tail_i(rob_recover_tail_i),
        .rob_recover_used_i(rob_recover_used_i),
        
        //ROB checkpoint to controller after predicting (for bne)
        .rob_chkpt_we_o(),
        .rob_chkpt_tag_o(),
        .rob_chkpt_tail_o(),
        .rob_chkpt_used_o(),
        
        //ROB to update commit
        .rob_commit_ready_i(rob_commit_ready_i),
        .rob_commit_valid_o(),
        .rob_commit_entry_o(),
        .rob_commit_index_o(),

        //outputs to FU's in Execute
        .alu_issue_valid_o(valid_alu_dispatch_o),
        .alu_issue_ready_i(ready_alu_dispatch_i),
        .alu_issue_o(data_alu_dispatch_o),
        
        .lsu_issue_valid_o(valid_lsu_dispatch_o),
        .lsu_issue_ready_i(ready_lsu_dispatch_i),
        .lsu_issue_o(data_lsu_dispatch_o),
        
        .br_issue_valid_o(valid_br_dispatch_o),
        .br_issue_ready_i(ready_br_dispatch_i),
        .br_issue_o(data_br_dispatch_o)
    );
    
    assign dbg_alu_issue_fire_o = valid_alu_dispatch_o && ready_alu_dispatch_i;
    assign dbg_lsu_issue_fire_o = valid_lsu_dispatch_o && ready_lsu_dispatch_i;
    assign dbg_br_issue_fire_o  = valid_br_dispatch_o  && ready_br_dispatch_i;
    
//    // -----------------------------
//    // Execute
//    // -----------------------------
//    execute_module #(.AE(AE), .AO(AO), .BE(BE), .BO(BO), .LE(LE), .LO(LO)) my_execute (
//        .clk_i(clk_i),
//        .reset_i(reset_i),

//        // [CHG] Drive FUs from dispatch issue channels
//        .alu_valid_i(alu_issue_valid),   // [CHG]
//        .alu_ready_o(alu_issue_ready),   // [CHG]
//        .alu_data_i (alu_issue_data),    // [CHG]
//        .alu_flush_i(1'b0),              // TODO: hook flush

//        .b_valid_i  (br_issue_valid),    // [CHG]
//        .b_ready_o  (br_issue_ready),    // [CHG]
//        .b_data_i   (br_issue_data),     // [CHG]
//        .b_flush_i  (1'b0),

//        .lsu_valid_i(mem_issue_valid),   // [FIX]
//        .lsu_ready_o(mem_issue_ready),   // [FIX]
//        .lsu_data_i (mem_issue_data),    // [FIX]
//        .lsu_flush_i(1'b0),

//        // [CHG] Skid outputs -> dispatch WB inputs
//        .asb_cons_ready_i(alu_wb_ready),     // [CHG]
//        .asb_cons_valid_o(alu_wb_valid),     // [CHG]
//        .asb_cons_data_o (alu_wb_data),      // [CHG]

//        .bsb_cons_ready_i(br_wb_ready),      // [CHG]
//        .bsb_cons_valid_o(br_wb_valid),      // [CHG]
//        .bsb_cons_data_o (br_wb_data),       // [CHG]

//        .lsb_cons_ready_i(lsu_wb_ready),     // [CHG]
//        .lsb_cons_valid_o(lsu_wb_valid),     // [CHG]
//        .lsb_cons_data_o (lsu_wb_data)       // [CHG]
//    );

 

endmodule
