`timescale 1ns/1ps
import buffer_pkgs::*;

module recovery_controller #(
    parameter int ROB_DEPTH = 16,
    parameter int AREG      = 32,
    parameter int PREGS     = buffer_pkgs::PREGS,
    parameter int PREG_W    = buffer_pkgs::PREG_W
)(
    input  logic clk_i,
    input  logic rst_i,

    // -----------------------------
    // From branch writeback
    // -----------------------------
    input  logic        br_wb_valid_i,
    input  branch_out_t br_wb_i,

    // -----------------------------
    // Checkpoint writes from RENAME
    //  (RAT + FreeList snapshot)
    // -----------------------------
    input  logic        ratfl_chkpt_we_i,
    input  logic [3:0]  ratfl_chkpt_tag_i,
    input  logic [AREG*PREG_W-1:0] ratfl_chkpt_rat_map_i,
    input  logic [PREG_W-1:0]      ratfl_chkpt_fl_head_i,
    input  logic [PREG_W-1:0]      ratfl_chkpt_fl_tail_i,
    input  logic [$clog2(PREGS):0] ratfl_chkpt_fl_free_count_i,

    // -----------------------------
    // Checkpoint writes from DISPATCH
    //  (ROB pointer snapshot)
    // -----------------------------
    input  logic        rob_chkpt_we_i,
    input  logic [3:0]  rob_chkpt_tag_i,
    input  logic [$clog2(ROB_DEPTH)-1:0] rob_chkpt_tail_i,
    input  logic [$clog2(ROB_DEPTH)  :0] rob_chkpt_used_i,

    // -----------------------------
    // Global recovery outputs
    // -----------------------------
    output logic        flush_o,

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

    // Optional: export mispredicted branch tag
    output logic [3:0]  recover_rob_tag_o
);

    // ---------------------------------------
    // Checkpoint storage (indexed by ROB_tag)
    // ---------------------------------------
    logic [AREG*PREG_W-1:0] rat_map_table [0:ROB_DEPTH-1];
    logic [PREG_W-1:0]      fl_head_table [0:ROB_DEPTH-1];
    logic [PREG_W-1:0]      fl_tail_table [0:ROB_DEPTH-1];
    logic [$clog2(PREGS):0] fl_cnt_table  [0:ROB_DEPTH-1];

    logic [$clog2(ROB_DEPTH)-1:0] rob_tail_table [0:ROB_DEPTH-1];
    logic [$clog2(ROB_DEPTH)  :0] rob_used_table [0:ROB_DEPTH-1];

    // Write checkpoints
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            // no need to initialize tables for function, but helps sim readability
            for (int i=0; i<ROB_DEPTH; i++) begin
                rat_map_table[i] <= '0;
                fl_head_table[i] <= '0;
                fl_tail_table[i] <= '0;
                fl_cnt_table[i]  <= '0;
                rob_tail_table[i]<= '0;
                rob_used_table[i]<= '0;
            end
        end else begin
            if (ratfl_chkpt_we_i) begin
                rat_map_table[ratfl_chkpt_tag_i[$clog2(ROB_DEPTH)-1:0]] <= ratfl_chkpt_rat_map_i;
                fl_head_table[ratfl_chkpt_tag_i[$clog2(ROB_DEPTH)-1:0]] <= ratfl_chkpt_fl_head_i;
                fl_tail_table[ratfl_chkpt_tag_i[$clog2(ROB_DEPTH)-1:0]] <= ratfl_chkpt_fl_tail_i;
                fl_cnt_table [ratfl_chkpt_tag_i[$clog2(ROB_DEPTH)-1:0]] <= ratfl_chkpt_fl_free_count_i;
            end

            if (rob_chkpt_we_i) begin
                rob_tail_table[rob_chkpt_tag_i[$clog2(ROB_DEPTH)-1:0]] <= rob_chkpt_tail_i;
                rob_used_table[rob_chkpt_tag_i[$clog2(ROB_DEPTH)-1:0]] <= rob_chkpt_used_i;
            end
        end
    end

    // ---------------------------------------
    // Mispredict detect + recovery outputs
    // ---------------------------------------
    wire mispredict_fire = br_wb_valid_i && br_wb_i.mispredict;
    wire [$clog2(ROB_DEPTH)-1:0] idx =
        br_wb_i.ROB_tag[$clog2(ROB_DEPTH)-1:0];

    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            flush_o            <= 1'b0;
            redirect_valid_o   <= 1'b0;
            redirect_pc_o      <= '0;

            rob_recover_o      <= 1'b0;
            rob_recover_tail_o <= '0;
            rob_recover_used_o <= '0;

            rat_recover_o      <= 1'b0;
            rat_recover_map_o  <= '0;

            fl_recover_o            <= 1'b0;
            fl_recover_head_o       <= '0;
            fl_recover_tail_o       <= '0;
            fl_recover_free_count_o <= '0;

            recover_rob_tag_o <= '0;
        end else begin
            // default: pulse-style enables
            flush_o          <= 1'b0;
            redirect_valid_o <= 1'b0;
            rob_recover_o    <= 1'b0;
            rat_recover_o    <= 1'b0;
            fl_recover_o     <= 1'b0;

            if (mispredict_fire) begin
                flush_o          <= 1'b1;

                redirect_valid_o <= 1'b1;
                redirect_pc_o    <= br_wb_i.dest_addr;

                rob_recover_o      <= 1'b1;
                rob_recover_tail_o <= rob_tail_table[idx];
                rob_recover_used_o <= rob_used_table[idx];

                rat_recover_o     <= 1'b1;
                rat_recover_map_o <= rat_map_table[idx];

                fl_recover_o            <= 1'b1;
                fl_recover_head_o       <= fl_head_table[idx];
                fl_recover_tail_o       <= fl_tail_table[idx];
                fl_recover_free_count_o <= fl_cnt_table[idx];

                recover_rob_tag_o <= br_wb_i.ROB_tag;
            end
        end
    end

endmodule