`timescale 1ns / 1ps
import buffer_pkgs::*;
// ASSUMES 128 DEPTH MEMORY. IF THIS CHANGES, NEED TO CHANGE THE IP BLOCK AND BRAM_ADDRESS SIZE

module lsu #(
    parameter type LE = lsu_entry_t,
    parameter type LO = lsu_out_t
)(
    input  logic clk_i,
    input  logic reset_i,
    input  logic flush_i,
    
    // ===========================
    // Input ready-valid handshake
    // ===========================
    input  logic req_valid_i,   // ready-valid handshake (input)
    output logic req_ready_o,    
    
    // ============================
    // Output ready-valid handshake
    // ============================
    output logic resp_valid_o,  // NEW
    input  logic resp_ready_i,  // NEW
    
    input  LE data_i,
    output LO data_o
);
    
    //local state defs
    typedef enum logic [1:0] {
        STATE_IDLE,
        STATE_WAIT1,
        STATE_WAIT2
    } lsu_state_t;
    lsu_state_t state, state_next;

    //bram interfaces 
    logic        bram_ena;
    logic [3:0]  bram_we;        // byte enables
    logic [6:0]  bram_addr;      // 128 words -> 7 bits
    logic [31:0] bram_din;
    logic [31:0] bram_dout;
    assign bram_ena = 1'b1;

    // Address breakdown
    logic [31:0] addr_calc;      // rs1 + imm (byte address)
    logic [6:0]  addr_word_idx;  // word address = addr[8:2]
    logic [1:0]  addr_byte_off;  // byte offset = addr[1:0]
    logic [6:0]  saved_addr_word_idx; //latched word address index 
    
    //latched metadata 
    logic        saved_is_load;
    logic        saved_is_store;
    logic        saved_lsType;       // 1 = word (LW/SW), 0 = byte/half (LBU/SH)
    logic [1:0]  saved_byte_off;
    logic [6:0]  saved_rd_addr;      // adjust width if your rd_addr is wider
    logic [3:0] saved_ROB_tag;

    // Next-output struct (combinational)
    LO lsu_out_next;
    
    // NEW: registered output + valid flag
    LO    data_q;             // NEW
    logic resp_valid_q;       // NEW
    logic fire_out;           // NEW
    logic result_ready;       // NEW

    // NEW: connect registered outputs
    assign data_o       = data_q;            // NEW
    assign resp_valid_o = resp_valid_q;      // NEW
    assign fire_out     = resp_valid_q && resp_ready_i;  // NEW

    assign result_ready = (state == STATE_WAIT2);        // NEW: result becomes valid in WAIT2
    
    //bram IP block 
    lsu_mem my_lsu_mem (
        .clka (clk_i),
        .addra(bram_addr),
        .dina (bram_din),
        .douta(bram_dout),
        .ena  (bram_ena),
        .wea  (bram_we)
    );
    
    //decode current request: 
    logic req_is_load;
    logic req_is_store;
    logic accept_req;

    // NEW: ready-valid status now depends on output being free and state
    assign req_ready_o = (state == STATE_IDLE) && !flush_i &&
                         (!resp_valid_q || resp_ready_i);  // NEW / CHANGED

    // CHANGED: accept_req uses req_ready_o so it respects both state and output backpressure
    assign accept_req = req_valid_i && req_ready_o &&
                        (req_is_load || req_is_store);     // CHANGED
    
    always_comb begin
        // Effective byte address
        addr_calc      = data_i.rs1_val + data_i.imm_val;
        addr_word_idx  = addr_calc[8:2];
        addr_byte_off  = addr_calc[1:0];

        // Interpret load/store type from entry
        req_is_load    = (data_i.loadStore == LOAD);
        req_is_store   = (data_i.loadStore == STORE);
    end
    
    //state register
    always_ff @(posedge clk_i) begin
        if (reset_i || flush_i) begin //flush resets state
            state <= STATE_IDLE;
        end else begin
            state <= state_next;
        end
    end
    
    //pipelined metadata
    always_ff @(posedge clk_i) begin
        if (reset_i || flush_i) begin
            saved_is_load       <= 1'b0;
            saved_is_store      <= 1'b0;
            saved_lsType        <= 1'b0;
            saved_byte_off      <= 2'b00;
            saved_rd_addr       <= '0;
            saved_ROB_tag <= '0;
            saved_addr_word_idx <= '0;   // NEW (init)
        end else if (accept_req) begin //only latches in on good handshake 
            // Latch metadata for this LSU operation
            saved_is_load       <= req_is_load;
            saved_is_store      <= req_is_store;
            saved_lsType        <= data_i.lsType;      // 1 = word, 0 = byte/halfword
            saved_byte_off      <= addr_byte_off;
            saved_rd_addr       <= data_i.rd_addr;
            saved_ROB_tag <= data_i.ROB_tag;
            saved_addr_word_idx <= addr_word_idx;
        end
    end
    
    //next state + bram control 
    always_comb begin
        // Defaults
        state_next = state;
        bram_we    = 4'b0000;
        bram_addr  = saved_addr_word_idx;
        bram_din   = 32'b0;

        case (state)
            STATE_IDLE: begin
                bram_addr = addr_word_idx;
                if (accept_req) begin // Issue BRAM request for this LSU op
                    if (req_is_store) begin // sw, sh reqs
                        if (data_i.lsType) begin //sw: word aligned 
                            bram_din = data_i.rs2_val;
                            bram_we  = 4'b1111;    // all 4 bytes
                        end else begin
                            case (addr_byte_off)
                                2'b00: begin //addr offset ending in 00, storing at lower half
                                    bram_din        = 32'b0;
                                    bram_din[15:0]  = data_i.rs2_val[15:0];
                                    bram_we         = 4'b0011;
                                end
                                2'b10: begin //storing at upper half
                                    bram_din        = 32'b0;
                                    bram_din[31:16] = data_i.rs2_val[15:0];
                                    bram_we         = 4'b1100;
                                end
                                default: begin //misaligned
                                    bram_din = 32'b0;
                                    bram_we  = 4'b0000;
                                end
                            endcase
                        end
                    end else begin //else, load 
                        bram_we = 4'b0000;
                    end
                    state_next = STATE_WAIT1;
                end
            end
            STATE_WAIT1: begin // First BRAM pipeline stage
                bram_we    = 4'b0000;
                bram_addr  = saved_addr_word_idx;
                state_next = STATE_WAIT2;
            end
            STATE_WAIT2: begin //second bram stage, data out valid now 
                bram_we    = 4'b0000;
                bram_addr  = saved_addr_word_idx;
                state_next = STATE_IDLE;
            end
            default: begin
                state_next = STATE_IDLE;
            end
        endcase
    end
    
    //generating output (combinational next)
    always_comb begin
        // Default: no result this cycle
        lsu_out_next = '0;

        if (state == STATE_WAIT2) begin // Only produce an output when BRAM data is ready
            lsu_out_next.completed = 1'b1;
            lsu_out_next.rd_addr   = saved_rd_addr;
            lsu_out_next.ROB_tag = saved_ROB_tag;

            if (saved_is_load) begin
                if (saved_lsType) begin //lw
                    lsu_out_next.rd_val = bram_dout;
                end else begin //lbu
                    case (saved_byte_off) //which byte to load? 
                        2'b00: lsu_out_next.rd_val = {24'b0, bram_dout[7:0]};
                        2'b01: lsu_out_next.rd_val = {24'b0, bram_dout[15:8]};
                        2'b10: lsu_out_next.rd_val = {24'b0, bram_dout[23:16]};
                        2'b11: lsu_out_next.rd_val = {24'b0, bram_dout[31:24]};
                    endcase
                end
            end else if (saved_is_store) begin
                lsu_out_next.rd_val = 32'b0;
            end
        end
    end
    
    //registered output with output ready-valid handshake
    always_ff @(posedge clk_i) begin
        if (reset_i || flush_i) begin
            data_q       <= '0;        // NEW
            resp_valid_q <= 1'b0;      // NEW
        end else begin
            // If consumer accepts current result
            if (fire_out) begin        // NEW
                resp_valid_q <= 1'b0;
            end

            // If a new result becomes ready this cycle (STATE_WAIT2)
            if (result_ready) begin    // NEW
                data_q       <= lsu_out_next;
                resp_valid_q <= 1'b1;
            end
        end
    end

endmodule
