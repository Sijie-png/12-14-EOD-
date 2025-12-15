`timescale 1ns / 1ns

module free_list_fifo #(
    parameter int PHYSREG = 128,
    parameter int AREG    = 32
) (
    input  logic clk_i,
    input  logic rst_i,
    
    // allocate interface
    input  logic alloc_en_i,
    output logic alloc_done_o,
    output logic [$clog2(PHYSREG)-1:0] alloc_index_o,
    
    // free interface
    input  logic free_en_i,
    output logic [$clog2(PHYSREG)-1:0] free_index_o,
    
    // status
    output logic empty_o,
    output logic [$clog2(PHYSREG):0] free_count,
    
    // speculation checkpoint
    output logic [$clog2(PHYSREG)-1:0] chkpt_head_o,
    output logic [$clog2(PHYSREG)-1:0] chkpt_tail_o,
    output logic [$clog2(PHYSREG):0]   chkpt_free_count_o,
    
    // recover
    input  logic [$clog2(PHYSREG)-1:0] recover_head_i,
    input  logic [$clog2(PHYSREG)-1:0] recover_tail_i,
    input  logic [$clog2(PHYSREG):0]   recover_free_count_i,
    input  logic                       recover_i
);

    localparam int PREG_W = $clog2(PHYSREG);

    logic [PREG_W-1:0] head_current, head_next;
    logic [PREG_W-1:0] tail_current, tail_next;
    logic [PREG_W:0]   num_free_current, num_free_next;
    
    logic alloc_active, free_active;
    
    assign alloc_active = alloc_en_i && (num_free_current != 0);
    assign free_active  = free_en_i;

    assign alloc_done_o = alloc_active;
    assign empty_o      = (num_free_current == 0);
    assign free_count   = num_free_current;

    // For now, free_index_o just exposes current tail pointer
    assign free_index_o = tail_current;
    
    // Checkpoint mirrors current pointers + count
    assign chkpt_head_o       = head_current;
    assign chkpt_tail_o       = tail_current;
    assign chkpt_free_count_o = num_free_current;
    
    always_comb begin
        head_next      = head_current;
        tail_next      = tail_current;
        num_free_next  = num_free_current;
        
        // default: current head is candidate allocation index
        alloc_index_o = head_current;

        if (alloc_active) begin
            alloc_index_o = head_current;
            head_next     = head_current + 1;
        end
        
        if (free_active) begin
            tail_next = tail_current + 1;
        end       
        
        unique case ({free_active, alloc_active})
            2'b01: num_free_next = num_free_current - 1; // alloc only
            2'b10: num_free_next = num_free_current + 1; // free only
            2'b11: num_free_next = num_free_current;     // alloc & free
            default: ;
        endcase
    end
    
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            // Reset:
            //  - Arch regs 0..AREG-1 are in use
            //  - Free list contains AREG..PHYSREG-1
            head_current     <= AREG[PREG_W-1:0];
            tail_current     <= '0;
            num_free_current <= PHYSREG - AREG;
        end
        else if (recover_i) begin
            head_current     <= recover_head_i;
            tail_current     <= recover_tail_i;
            num_free_current <= recover_free_count_i;
        end
        else begin
            head_current     <= head_next;
            tail_current     <= tail_next;
            num_free_current <= num_free_next;
        end
    end

endmodule
