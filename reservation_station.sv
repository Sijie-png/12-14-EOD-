`timescale 1ns / 1ns
import buffer_pkgs::*;

module reservation_station #(
    parameter type RS_ENTRY_T = rs_entry_t,
    parameter int  RS_DEPTH   = 8
)(
    input  logic      clk_i,
    input  logic      rst_i,
    input  logic      recover_i,

    input  logic      disp_valid_i,
    output logic      disp_ready_o,
    input  RS_ENTRY_T disp_entry_i,

    output logic      issue_valid_o,
    input  logic      issue_ready_i,
    output RS_ENTRY_T issue_entry_o,

    input  logic       cdb_valid_i,
    input  logic [6:0] cdb_tag_i
);

    RS_ENTRY_T entries [RS_DEPTH];

    logic [RS_DEPTH-1:0] valid_vec;
    logic [RS_DEPTH-1:0] ready_vec;

    always_comb begin
        for (int i = 0; i < RS_DEPTH; i++) begin
            valid_vec[i] = entries[i].valid;
            // ----------------------------
            // FIX: ready is derived, not stored
            // ----------------------------
            ready_vec[i] = entries[i].rs1_rdy & entries[i].rs2_rdy; // FIX
        end
    end

    logic [RS_DEPTH-1:0] free_mask; 
    assign free_mask = ~valid_vec;
    logic [$clog2(RS_DEPTH)-1:0] free_idx;
    logic have_free;
    prioritydecoder #(.WIDTH(RS_DEPTH)) u_dec_free (
        .in   (free_mask),
        .out  (free_idx),
        .valid(have_free)
    );

    assign disp_ready_o = have_free && !recover_i;
    wire do_insert = disp_valid_i && disp_ready_o;

    logic [RS_DEPTH-1:0] issue_mask;
    assign issue_mask = valid_vec & ready_vec;
    logic [$clog2(RS_DEPTH)-1:0] issue_idx;
    logic have_issue;
    prioritydecoder #(.WIDTH(RS_DEPTH)) u_dec_issue (
        .in   (issue_mask),
        .out  (issue_idx),
        .valid(have_issue)
    );

    // ============================================================
    // [CHG] Hold register to make issue_entry_o stable under stall
    // ============================================================
    logic                          hold_valid;     // [CHG]
    RS_ENTRY_T                      hold_entry;     // [CHG]
    logic [$clog2(RS_DEPTH)-1:0]    hold_idx;       // [CHG]

    // [CHG] "raw" candidate exists (combinational arbitration)
    wire cand_valid = have_issue && !recover_i;     // [CHG]
    wire do_issue   = issue_valid_o && issue_ready_i; // [CHG] (moved below)

    // [CHG] Latch a candidate when not currently holding one.
    //       Once held, keep it stable until accepted.
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i || recover_i) begin
            hold_valid <= 1'b0;                    // [CHG]
            hold_entry <= '0;                      // [CHG]
            hold_idx   <= '0;                      // [CHG]
        end else begin
            // Capture a new candidate only if we are not already holding one
            if (!hold_valid && cand_valid) begin   // [CHG]
                hold_valid <= 1'b1;                // [CHG]
                hold_entry <= entries[issue_idx];  // [CHG]
                hold_idx   <= issue_idx;           // [CHG]
            end

            // Drop hold once accepted
            if (hold_valid && issue_ready_i) begin // [CHG]
                hold_valid <= 1'b0;                // [CHG]
            end
        end
    end

    // [CHG] Drive issue interface from hold regs (stable under backpressure)
    assign issue_valid_o = hold_valid && !recover_i;      // [CHG]
    assign issue_entry_o = hold_valid ? hold_entry : '0;  // [CHG] (also fixes X-prop)

    // [CHG] Handshake uses held valid
    // NOTE: do_issue is already declared above.
    // wire do_issue = issue_valid_o && issue_ready_i;

    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i || recover_i) begin
            for (int i = 0; i < RS_DEPTH; i++) begin
                entries[i]        <= '0;
                entries[i].valid  <= 1'b0;
                entries[i].rs1_rdy<= 1'b0;
                entries[i].rs2_rdy<= 1'b0;
            end
        end else begin
            // Wakeup
            if (cdb_valid_i) begin
                for (int i=0; i<RS_DEPTH; i++) begin
                    if (entries[i].valid) begin
                        if (!entries[i].rs1_rdy && (entries[i].rs1_tag == cdb_tag_i))
                            entries[i].rs1_rdy <= 1'b1;
                        if (!entries[i].rs2_rdy && (entries[i].rs2_tag == cdb_tag_i))
                            entries[i].rs2_rdy <= 1'b1;
                    end
                end
            end

            // Insert
            if (do_insert) begin
                entries[free_idx]       <= disp_entry_i;
                entries[free_idx].valid <= 1'b1;
            end

            // ============================================================
            // [CHG] Remove on issue uses held index, not comb issue_idx
            //       (prevents removing the wrong entry under backpressure)
            // ============================================================
            if (do_issue && !(do_insert && (hold_idx == free_idx))) begin // [CHG]
                entries[hold_idx].valid   <= 1'b0;                        // [CHG]
                entries[hold_idx].rs1_rdy <= 1'b0;                        // [CHG]
                entries[hold_idx].rs2_rdy <= 1'b0;                        // [CHG]
            end
        end
    end

endmodule
