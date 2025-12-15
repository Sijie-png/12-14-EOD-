`timescale 1ns / 1ps
import buffer_pkgs::*;

module rob_module#(
    parameter int DEPTH = 16,
    parameter int PREGS = buffer_pkgs::PREGS          // [CHG]
)(
    input  logic clk_i,
    input  logic rst_i,

    input  rename_t data_i,

    input  logic        alloc_valid_i,
    output logic        alloc_ready_o,
    input  logic        alloc_has_dest_i,
    input  logic [4:0]  alloc_arch_rd_i,
    input  logic [PREG_W-1:0] alloc_dest_preg_i,      // [CHG] use PREG_W
    input  logic [PREG_W-1:0] alloc_old_preg_i,       // [CHG] use PREG_W
    input  logic        alloc_is_branch_i,
    input  logic [31:0] alloc_pc_i,
    output logic [$clog2(DEPTH)-1:0] alloc_rob_index_o,

    output logic [$clog2(DEPTH)-1:0] chkpt_tail_o,
    output logic [$clog2(DEPTH)  :0] chkpt_used_count_o,

    input  logic                     recover_i,
    input  logic [$clog2(DEPTH)-1:0] recover_tail_i,
    input  logic [$clog2(DEPTH)  :0] recover_used_count_i,

    input logic complete_valid_i,
    input logic [$clog2(DEPTH)-1:0] complete_rob_index_i,
    input logic complete_mispredict_i,

    input logic commit_ready_i,
    output logic commit_valid_o,
    output rob_entry_t commit_entry_o,
    output logic [$clog2(DEPTH)-1:0] commit_rob_index_o,

    output logic [PREGS-1:0] busy_pregs_o             // [CHG] NEW: pending-dest bitmap
);

    localparam int PTR_W = $clog2(DEPTH);

    rob_entry_t rob[DEPTH];

    logic [PTR_W-1:0] head_current, head_next;
    logic [PTR_W-1:0] tail_current, tail_next;
    logic [PTR_W  :0] num_used_current, num_used_next;

    wire full  = (num_used_current == DEPTH);
    wire empty = (num_used_current == 0);

    logic       do_alloc;
    rob_entry_t alloc_entry;
    logic       do_commit;

    assign chkpt_tail_o        = tail_current;
    assign chkpt_used_count_o  = num_used_current;
    assign alloc_rob_index_o   = tail_current;
    assign commit_rob_index_o  = head_current;

    // [CHG] Busy bitmap (valid & has_dest & !completed)
    always_comb begin
        busy_pregs_o = '0;
        for (int k = 0; k < DEPTH; k++) begin
            if (rob[k].valid && rob[k].has_dest && !rob[k].completed) begin
                if (rob[k].dest_preg != '0)
                    busy_pregs_o[rob[k].dest_preg] = 1'b1;
            end
        end
    end

    // alloc_ready considers same-cycle commit
    assign alloc_ready_o = !recover_i && (!full || do_commit);

    function automatic logic in_range_wrap(
        input logic [PTR_W-1:0] x,
        input logic [PTR_W-1:0] start,
        input logic [PTR_W-1:0] end_
    );
        if (start <= end_)
            return (x >= start) && (x < end_);
        else
            return (x >= start) || (x < end_);
    endfunction

    always_comb begin
        head_next     = head_current;
        tail_next     = tail_current;
        num_used_next = num_used_current;

        do_alloc    = 1'b0;
        alloc_entry = '0;

        // commit output
        commit_valid_o = rob[head_current].valid && rob[head_current].completed;
        commit_entry_o = rob[head_current];
        do_commit      = commit_valid_o && commit_ready_i;

        if (do_commit) begin
            head_next     = head_current + 1'b1;
            num_used_next = num_used_current - 1'b1;
        end

        if (!recover_i && alloc_valid_i && alloc_ready_o) begin
            do_alloc = 1'b1;

            alloc_entry.valid      = 1'b1;
            alloc_entry.completed  = 1'b0;
            alloc_entry.has_dest   = alloc_has_dest_i;
            alloc_entry.arch_rd    = alloc_arch_rd_i;
            alloc_entry.dest_preg  = alloc_dest_preg_i;
            alloc_entry.old_preg   = alloc_old_preg_i;
            alloc_entry.is_branch  = alloc_is_branch_i;
            alloc_entry.pc         = alloc_pc_i;

            tail_next     = tail_current + 1'b1;
            num_used_next = num_used_current + 1'b1;
        end
    end

    integer i;
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            head_current     <= '0;
            tail_current     <= '0;
            num_used_current <= '0;
            for (i=0; i<DEPTH; i++) rob[i] <= '0;
        end
        else if (recover_i) begin
            // restore pointers/count
            tail_current     <= recover_tail_i;
            num_used_current <= recover_used_count_i;

            // invalidate younger entries in [recover_tail_i, old tail_current)
            for (i=0; i<DEPTH; i++) begin
                logic [PTR_W-1:0] idx;
                idx = i[PTR_W-1:0];
                if (in_range_wrap(idx, recover_tail_i, tail_current))
                    rob[i].valid <= 1'b0;
            end
        end
        else begin
            head_current     <= head_next;
            tail_current     <= tail_next;
            num_used_current <= num_used_next;

            if (do_commit)
                rob[head_current].valid <= 1'b0;

            if (do_alloc)
                rob[tail_current] <= alloc_entry;

            if (complete_valid_i && rob[complete_rob_index_i].valid) begin
                rob[complete_rob_index_i].completed <= 1'b1;
            end
        end
    end

endmodule
