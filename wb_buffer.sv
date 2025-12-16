`timescale 1ns/1ps
import buffer_pkgs::*;

module writeback_module #(
    parameter int ROB_DEPTH = 16,
    parameter int AREG      = 32
)(
    input  logic clk_i,
    input  logic rst_i,

    // Unified writeback input from execute_module
    input  logic       wb_valid_i,
    input  wb_packet_t wb_packet_i,
    output logic       wb_ready_o,     // always-ready in this design

    // Checkpoint writes into recovery controller
    input  logic                   ratfl_chkpt_we_i,
    input  logic [3:0]             ratfl_chkpt_tag_i,
    input  logic [AREG*PREG_W-1:0] ratfl_chkpt_rat_map_i,
    input  logic [PREG_W-1:0]      ratfl_chkpt_fl_head_i,
    input  logic [PREG_W-1:0]      ratfl_chkpt_fl_tail_i,
    input  logic [$clog2(PREGS):0] ratfl_chkpt_fl_free_count_i,

    input  logic                   rob_chkpt_we_i,
    input  logic [3:0]             rob_chkpt_tag_i,
    input  logic [$clog2(ROB_DEPTH)-1:0] rob_chkpt_tail_i,
    input  logic [$clog2(ROB_DEPTH)  :0] rob_chkpt_used_i,

    // GLOBAL RECOVERY OUTPUTS
    output logic        recover_o,
    output logic        redirect_valid_o,
    output logic [31:0] redirect_pc_o,

    output logic        rob_recover_o,
    output logic [$clog2(ROB_DEPTH)-1:0] rob_recover_tail_o,
    output logic [$clog2(ROB_DEPTH)  :0] rob_recover_used_o,

    output logic        rat_recover_o,
    output logic [AREG*PREG_W-1:0] rat_recover_map_o,

    output logic        fl_recover_o,
    output logic [PREG_W-1:0]      fl_recover_head_o,
    output logic [PREG_W-1:0]      fl_recover_tail_o,
    output logic [$clog2(PREGS):0] fl_recover_free_count_o,

    output logic [3:0]  recover_rob_tag_o,

    // PRF write port (single)
    output logic              prf_wb_en_o,
    output logic [PREG_W-1:0] prf_wb_addr_o,
    output logic [31:0]       prf_wb_data_o,

    // CDB broadcast for wakeup (single broadcast per cycle)
    output logic              cdb_valid_o,
    output logic [PREG_W-1:0] cdb_tag_o,
    output logic [31:0]       cdb_data_o,

    // ROB completion (single completion per cycle)
    output logic                          rob_complete_valid_o,
    output logic [$clog2(ROB_DEPTH)-1:0]  rob_complete_idx_o,
    output logic                          rob_complete_mispredict_o
);
    localparam int ROB_PTR_W = $clog2(ROB_DEPTH);

    // ------------------------------------------------------------
    // WB always ready
    // ------------------------------------------------------------
    assign wb_ready_o = 1'b1;

    // ------------------------------------------------------------
    // Drive PRF + CDB + ROB completion from wb_packet
    // ------------------------------------------------------------
    always_comb begin
        prf_wb_en_o   = 1'b0;
        prf_wb_addr_o = '0;
        prf_wb_data_o = '0;

        cdb_valid_o   = 1'b0;
        cdb_tag_o     = '0;
        cdb_data_o    = '0;

        rob_complete_valid_o      = 1'b0;
        rob_complete_idx_o        = '0;
        rob_complete_mispredict_o = 1'b0;

        if (wb_valid_i && wb_packet_i.completed) begin
            // ROB completion for any completed WB beat
            rob_complete_valid_o = 1'b1;
//            rob_complete_idx_o   = wb_packet_i.ROB_tag[$clog2(ROB_DEPTH)-1:0];
            rob_complete_idx_o = wb_packet_i.ROB_tag[ROB_PTR_W-1:0];

            // only branches can mispredict
            rob_complete_mispredict_o =
                (wb_packet_i.src_fu == 2'd2) ? wb_packet_i.mispredict : 1'b0;

            // PRF+CDB only when regwrite
            if (wb_packet_i.rd_addr != '0) begin
                prf_wb_en_o   = 1'b1;
                prf_wb_addr_o = wb_packet_i.rd_addr;
                prf_wb_data_o = wb_packet_i.rd_val;

                cdb_valid_o   = 1'b1;
                cdb_tag_o     = wb_packet_i.rd_addr;
                cdb_data_o    = wb_packet_i.rd_val;
            end
        end
    end

    // ------------------------------------------------------------
    // Convert unified WB packet -> branch_out_t for recovery_controller
    // Only drive valid for BR unit packets
    // ------------------------------------------------------------
    logic        br_wb_valid_i;
    branch_out_t br_wb_i;

    always_comb begin
        br_wb_valid_i = 1'b0;
        br_wb_i       = '0;

        if (wb_valid_i && wb_packet_i.completed && (wb_packet_i.src_fu == 2'd2)) begin
            br_wb_valid_i       = 1'b1;

            br_wb_i.rd_addr     = wb_packet_i.rd_addr;
            br_wb_i.rd_val      = wb_packet_i.rd_val;
            br_wb_i.dest_addr   = wb_packet_i.dest_addr;
            br_wb_i.branch_taken= wb_packet_i.branch_taken;
            br_wb_i.completed   = wb_packet_i.completed;
            br_wb_i.mispredict  = wb_packet_i.mispredict;
            br_wb_i.ROB_tag     = wb_packet_i.ROB_tag;
        end
    end

    // ------------------------------------------------------------
    // Recovery controller (Phase 4 style) still works with BR-only input
    // ------------------------------------------------------------
    recovery_controller #(
        .ROB_DEPTH(ROB_DEPTH),
        .AREG     (AREG),
        .PREGS    (buffer_pkgs::PREGS),
        .PREG_W   (buffer_pkgs::PREG_W)
    ) u_recovery (
        .clk_i                 (clk_i),
        .rst_i                 (rst_i),

        .br_wb_valid_i         (br_wb_valid_i),
        .br_wb_i               (br_wb_i),

        .ratfl_chkpt_we_i      (ratfl_chkpt_we_i),
        .ratfl_chkpt_tag_i     (ratfl_chkpt_tag_i),
        .ratfl_chkpt_rat_map_i (ratfl_chkpt_rat_map_i),
        .ratfl_chkpt_fl_head_i (ratfl_chkpt_fl_head_i),
        .ratfl_chkpt_fl_tail_i (ratfl_chkpt_fl_tail_i),
        .ratfl_chkpt_fl_free_count_i (ratfl_chkpt_fl_free_count_i),

        .rob_chkpt_we_i        (rob_chkpt_we_i),
        .rob_chkpt_tag_i       (rob_chkpt_tag_i),
        .rob_chkpt_tail_i      (rob_chkpt_tail_i),
        .rob_chkpt_used_i      (rob_chkpt_used_i),

        .flush_o               (recover_o),
        .redirect_valid_o      (redirect_valid_o),
        .redirect_pc_o         (redirect_pc_o),

        .rob_recover_o         (rob_recover_o),
        .rob_recover_tail_o    (rob_recover_tail_o),
        .rob_recover_used_o    (rob_recover_used_o),

        .rat_recover_o         (rat_recover_o),
        .rat_recover_map_o     (rat_recover_map_o),

        .fl_recover_o          (fl_recover_o),
        .fl_recover_head_o     (fl_recover_head_o),
        .fl_recover_tail_o     (fl_recover_tail_o),
        .fl_recover_free_count_o (fl_recover_free_count_o),

        .recover_rob_tag_o     (recover_rob_tag_o)
    );
    
    
    always_ff @(posedge clk_i) begin
        if (wb_valid_i && wb_packet_i.completed) begin
            assert(wb_packet_i.ROB_tag < ROB_DEPTH)
                else $fatal("WB ROB_tag out of range");
        end
    end

endmodule