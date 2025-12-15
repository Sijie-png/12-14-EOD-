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
    // [FIX] From unified writeback
    // -----------------------------
    input  logic       wb_valid_i,                // [FIX]
    input  wb_packet_t wb_packet_i,               // [FIX]

    // -----------------------------
    // Checkpoint writes from RENAME
    // -----------------------------
    input  logic        ratfl_chkpt_we_i,
    input  logic [3:0]  ratfl_chkpt_tag_i,
    input  logic [AREG*PREG_W-1:0] ratfl_chkpt_rat_map_i,
    input  logic [PREG_W-1:0]      ratfl_chkpt_fl_head_i,
    input  logic [PREG_W-1:0]      ratfl_chkpt_fl_tail_i,
    input  logic [$clog2(PREGS):0] ratfl_chkpt_fl_free_count_i,

    // -----------------------------
    // Checkpoint writes from DISPATCH (ROB)
    // -----------------------------
    input  logic        rob_chkpt_we_i,
    input  logic [3:0]  rob_chkpt_tag_i,
    input  logic [$clog2(ROB_DEPTH)-1:0] rob_chkpt_tail_i,
    input  logic [$clog2(ROB_DEPTH)  :0] rob_chkpt_used_i,

    // -----------------------------
    // Outputs
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

    integer i;
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            for (i = 0; i < ROB_DEPTH; i++) begin
                rat_map_table[i]  <= '0;
                fl_head_table[i]  <= '0;
                fl_tail_table[i]  <= '0;
                fl_cnt_table[i]   <= '0;
                rob_tail_table[i] <= '0;
                rob_used_table[i] <= '0;
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
    // [FIX] Mispredict detect from unified packet
    // ---------------------------------------
    wire is_br_wb        = wb_valid_i && (wb_packet_i.src_fu == 2'd2); // BR
    wire mispredict_fire = is_br_wb && wb_packet_i.mispredict;

    wire [$clog2(ROB_DEPTH)-1:0] idx =
        wb_packet_i.ROB_tag[$clog2(ROB_DEPTH)-1:0]; // [FIX]

    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            flush_o              <= 1'b0;
            redirect_valid_o     <= 1'b0;
            redirect_pc_o        <= '0;

            rob_recover_o        <= 1'b0;
            rob_recover_tail_o   <= '0;
            rob_recover_used_o   <= '0;

            rat_recover_o        <= 1'b0;
            rat_recover_map_o    <= '0;

            fl_recover_o         <= 1'b0;
            fl_recover_head_o    <= '0;
            fl_recover_tail_o    <= '0;
            fl_recover_free_count_o <= '0;

            recover_rob_tag_o    <= '0;
        end else begin
            // default deassert (pulse-style)
            flush_o          <= 1'b0;
            redirect_valid_o <= 1'b0;

            rob_recover_o    <= 1'b0;
            rat_recover_o    <= 1'b0;
            fl_recover_o     <= 1'b0;

            if (mispredict_fire) begin
                flush_o          <= 1'b1;
                redirect_valid_o <= 1'b1;
                redirect_pc_o    <= wb_packet_i.dest_addr; // [FIX]

                rob_recover_o      <= 1'b1;
                rob_recover_tail_o <= rob_tail_table[idx];
                rob_recover_used_o <= rob_used_table[idx];

                rat_recover_o      <= 1'b1;
                rat_recover_map_o  <= rat_map_table[idx];

                fl_recover_o         <= 1'b1;
                fl_recover_head_o    <= fl_head_table[idx];
                fl_recover_tail_o    <= fl_tail_table[idx];
                fl_recover_free_count_o <= fl_cnt_table[idx];

                recover_rob_tag_o <= wb_packet_i.ROB_tag; // [FIX]
            end
        end
    end

endmodule
