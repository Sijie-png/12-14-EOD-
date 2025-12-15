`timescale 1ns / 1ps
import buffer_pkgs::*;

module commit_stage #(
    parameter int DEPTH = 16
)(
    input  logic clk_i,
    input  logic rst_i,

    // -----------------------------
    // From ROB (head / commit side)
    // -----------------------------
    input  logic                         rob_commit_valid_i,
    input  rob_entry_t                   rob_commit_entry_i,
    input  logic [$clog2(DEPTH)-1:0]     rob_commit_index_i,
    output logic                         rob_commit_ready_o,

    // -----------------------------
    // To map table / rename (arch state)
    // -----------------------------
    output logic                         commit_map_valid_o,
    output logic [4:0]                   commit_arch_rd_o,
    output logic [PREG_W-1:0]            commit_dest_preg_o,

    // -----------------------------
    // To free list / PRF (old phys reg)
    // -----------------------------
    output logic                         commit_free_valid_o,
    output logic [PREG_W-1:0]            commit_free_preg_o
);

    // -------------------------------------------------
    // Commit never stalls (simple in-order commit)
    // -------------------------------------------------
    assign rob_commit_ready_o = 1'b1;

    // -------------------------------------------------
    // [FIX] Commit fires only when valid AND ready
    // -------------------------------------------------
    logic commit_fire;
    assign commit_fire = rob_commit_valid_i && rob_commit_ready_o; // [FIX]

    // -----------------------------
    // Map table / architectural state update
    // -----------------------------
    assign commit_map_valid_o  = commit_fire && rob_commit_entry_i.has_dest;
    assign commit_arch_rd_o    = rob_commit_entry_i.arch_rd;
    assign commit_dest_preg_o  = rob_commit_entry_i.dest_preg;

    // -----------------------------
    // Free-list update
    // -----------------------------
    assign commit_free_valid_o = commit_fire &&
                                 rob_commit_entry_i.has_dest &&
                                 (rob_commit_entry_i.old_preg != PREG_W'(0));
    assign commit_free_preg_o  = rob_commit_entry_i.old_preg;

    // -----------------------------
    // Optional debug
    // -----------------------------
    always_ff @(posedge clk_i) begin
        if (!rst_i && commit_fire) begin
            $display("[%0t][COMMIT] idx=%0d arch_rd=%0d dest_preg=%0d old_preg=%0d pc=0x%08x",
                     $time,
                     rob_commit_index_i,
                     rob_commit_entry_i.arch_rd,
                     rob_commit_entry_i.dest_preg,
                     rob_commit_entry_i.old_preg,
                     rob_commit_entry_i.pc);
        end
    end

endmodule
