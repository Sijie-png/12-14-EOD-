`timescale 1ns/1ps
import buffer_pkgs::*;

module processor_tb();

    // ------------------------------------------------------------
    // Clock / reset
    // ------------------------------------------------------------
    logic clk, reset;
    always #5 clk = ~clk;
    
    // ------------------------------------------------------------
    // DUT interface
    // ------------------------------------------------------------
    alu_entry_t    alu_pkt;
    lsu_entry_t    lsu_pkt;
    branch_entry_t br_pkt;
    
    logic alu_v, alu_r;
    logic lsu_v, lsu_r;
    logic br_v, br_r;
    
    // DEBUG
    rename_t dbg_rename_pkt;
    logic    dbg_rename_v, dbg_rename_r;
    logic    dbg_alu_fire, dbg_lsu_fire, dbg_br_fire;

    // Dispatch debug
    logic dbg_pipe_v, dbg_pipe_r, dbg_pipe_fire;
    logic dbg_alu_ins, dbg_mem_ins, dbg_br_ins;
    logic dbg_disp_fire;
    logic [3:0]  dbg_disp_tag, dbg_disp_fu;
    logic [31:0] dbg_disp_pc;

    
  processor dut (
    .clk_i(clk),
    .reset_i(reset),

    .data_alu_dispatch_o (alu_pkt),
    .valid_alu_dispatch_o(alu_v),
    .ready_alu_dispatch_i(alu_r),

    .data_lsu_dispatch_o (lsu_pkt),
    .valid_lsu_dispatch_o(lsu_v),
    .ready_lsu_dispatch_i(lsu_r),

    .data_br_dispatch_o  (br_pkt),
    .valid_br_dispatch_o (br_v),
    .ready_br_dispatch_i (br_r),

    // Rename debug
    .dbg_rename_valid_o  (dbg_rename_v),
    .dbg_rename_ready_o  (dbg_rename_r),
    .dbg_rename_data_o   (dbg_rename_pkt),

    // Issue debug
    .dbg_alu_issue_fire_o(dbg_alu_fire),
    .dbg_lsu_issue_fire_o(dbg_lsu_fire),
    .dbg_br_issue_fire_o (dbg_br_fire),

    // Dispatch debug
    .dbg_disp_pipe_valid_o   (dbg_pipe_v),
    .dbg_disp_pipe_ready_o   (dbg_pipe_r),
    .dbg_disp_pipe_fire_o    (dbg_pipe_fire),
    
    .dbg_disp_alu_insert_o   (dbg_alu_ins),
    .dbg_disp_mem_insert_o   (dbg_mem_ins),
    .dbg_disp_br_insert_o    (dbg_br_ins),

    .dbg_disp_dispatch_fire_o(dbg_disp_fire),

    .dbg_disp_tag_o          (dbg_disp_tag),
    .dbg_disp_pc_o           (dbg_disp_pc),
    .dbg_disp_fu_o           (dbg_disp_fu)
  );

  // ------------------------------------------------------------
  // Scoreboards
  // ------------------------------------------------------------
  int rename_count;
  int issue_count;

  bit seen_rename    [0:15];
  bit seen_issue     [0:15];
  bit seen_dispatch  [0:15];   // [CHG] track "pipe consumed" per tag (dispatch accepted)

  // [CHG] counts for deeper visibility
  int dispatch_count;          // [CHG]
  int insert_alu_count;        // [CHG]
  int insert_mem_count;        // [CHG]
  int insert_br_count;         // [CHG]

  // ------------------------------------------------------------
  // Backpressure generator
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (reset) begin
      alu_r <= 0;
      lsu_r <= 0;
      br_r  <= 0;
    end else begin
      alu_r <= ($urandom_range(0,3) != 0); // ~75%
      lsu_r <= ($urandom_range(0,1) != 0); // ~50%
      br_r  <= ($urandom_range(0,2) != 0); // ~66%
    end
  end

  // ------------------------------------------------------------
  // Rename acceptance tracking
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (reset) begin
      rename_count <= 0;
      for (int i=0;i<16;i++) seen_rename[i] <= 0;
    end else if (dbg_rename_v && dbg_rename_r) begin
      rename_count++;
      seen_rename[dbg_rename_pkt.ROB_tag] <= 1'b1;

      $display("[%0t] RENAME ACCEPT tag=%0d pc=%h",
               $time, dbg_rename_pkt.ROB_tag, dbg_rename_pkt.pc);
    end
  end

  // ------------------------------------------------------------
  // [CHG] Dispatch acceptance tracking (pipe_fire)
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (reset) begin
      dispatch_count    <= 0;           // [CHG]
      insert_alu_count  <= 0;           // [CHG]
      insert_mem_count  <= 0;           // [CHG]
      insert_br_count   <= 0;           // [CHG]
      for (int i=0;i<16;i++) seen_dispatch[i] <= 0;
    end else if (dbg_pipe_fire) begin
      dispatch_count++;
      seen_dispatch[dbg_disp_tag] <= 1'b1;

      // Optional: count inserts by RS
      if (dbg_alu_ins) insert_alu_count++;
      if (dbg_mem_ins) insert_mem_count++;
      if (dbg_br_ins)  insert_br_count++;

      $display("[%0t] DISPATCH FIRE tag=%0d pc=%h fu=%0d  ins(ALU,MEM,BR)=(%0b,%0b,%0b)",
               $time, dbg_disp_tag, dbg_disp_pc, dbg_disp_fu,
               dbg_alu_ins, dbg_mem_ins, dbg_br_ins);
    end
  end

  // ------------------------------------------------------------
  // [CHG] Dispatch invariants (these catch drops/double-inserts)
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (!reset && dbg_pipe_fire) begin
      // Must insert into exactly one RS
      assert (dbg_alu_ins ^ dbg_mem_ins ^ dbg_br_ins)
        else $error("[%0t] DISPATCH DROP OR DOUBLE INSERT tag=%0d pc=%h fu=%0d  ins(ALU,MEM,BR)=(%0b,%0b,%0b)",
                    $time, dbg_disp_tag, dbg_disp_pc, dbg_disp_fu,
                    dbg_alu_ins, dbg_mem_ins, dbg_br_ins);

      // pipe_fire should match dispatch_fire (sanity)
      assert (dbg_pipe_fire == dbg_disp_fire)
        else $error("[%0t] PIPE/DISPATCH FIRE MISMATCH tag=%0d pc=%h pipe_fire=%0b disp_fire=%0b",
                    $time, dbg_disp_tag, dbg_disp_pc, dbg_pipe_fire, dbg_disp_fire);
    end
  end

  // ------------------------------------------------------------
  // Issue tracking
  // ------------------------------------------------------------
  task automatic mark_issue(input logic fire, input logic [3:0] tag, input string fu);
    if (fire) begin
      if (!seen_rename[tag]) begin
        $error("[%0t] ISSUE WITHOUT RENAME: tag=%0d fu=%s",
               $time, tag, fu);
      end
      if (!seen_dispatch[tag]) begin // [CHG] stricter: issue must be after dispatch fire
        $error("[%0t] ISSUE WITHOUT DISPATCH: tag=%0d fu=%s",
               $time, tag, fu);
      end
      if (seen_issue[tag]) begin
        $error("[%0t] DOUBLE ISSUE: tag=%0d fu=%s",
               $time, tag, fu);
      end
      seen_issue[tag] <= 1'b1;
      issue_count++;
      $display("[%0t] ISSUE %-4s tag=%0d", $time, fu, tag);
    end
  endtask

  always_ff @(posedge clk) begin
    if (reset) begin
      issue_count <= 0;                // [CHG] initialize counter
      for (int i=0;i<16;i++) seen_issue[i] <= 0;  // [CHG] init bitmap
    end else begin
      mark_issue(dbg_alu_fire, alu_pkt.ROB_tag, "ALU");
      mark_issue(dbg_lsu_fire, lsu_pkt.ROB_tag, "LSU");
      mark_issue(dbg_br_fire,  br_pkt.ROB_tag,  "BR");
    end
  end

  // ------------------------------------------------------------
  // End-of-test checks
  // ------------------------------------------------------------
  initial begin
    clk = 0;
    reset = 1;

    repeat (3) @(posedge clk);
    reset = 0;

    repeat (200) @(posedge clk);

    $display("------------------------------------------------");
    $display("Rename accepted   : %0d", rename_count);
    $display("Dispatch accepted : %0d", dispatch_count);     // [CHG]
    $display("Issued total      : %0d", issue_count);
    $display("RS inserts (A/M/B): %0d / %0d / %0d",         // [CHG]
             insert_alu_count, insert_mem_count, insert_br_count);

    for (int i=0;i<16;i++) begin
      // [CHG] stage-by-stage loss detection
      if (seen_rename[i] && !seen_dispatch[i]) begin
        $error("LOST BETWEEN RENAME->DISPATCH: ROB tag %0d", i);
      end
      if (seen_dispatch[i] && !seen_issue[i]) begin
        $error("LOST BETWEEN DISPATCH->ISSUE: ROB tag %0d", i);
      end
    end

    $display("TEST COMPLETE");
    $finish;
  end

endmodule
