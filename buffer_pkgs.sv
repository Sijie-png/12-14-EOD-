`timescale 1ns / 1ns


package buffer_pkgs;

    // ---------------------------------------
    // Global physical register config
    // ---------------------------------------
    localparam int PREGS   = 128;
    localparam int PREG_W  = $clog2(PREGS);   // = 7 for 128 regs
    
    //ROB size
    localparam int ROB_DEPTH = 16;
    localparam int ROB_PTR_W = $clog2(ROB_DEPTH);


    // ---------------- Enums ----------------
    typedef enum logic [3:0] {
        NA,
        ADD,
        SUB,
        ANDD,
        ORR,
        RSHIFT,
        SLTIU,
        LUI
    } ALUCTRL;

    typedef enum logic [3:0] {
        NOFUNC,
        ALU,
        MEM,
        BRANCH
    } FUNCU;

    typedef enum logic {
        BHW,
        WORD
    } MEMDATATYPE;

    typedef enum logic {
        STORE,
        LOAD
    } LOADSTORE;

    // ---------------- Fetch stage type ----------------
    typedef struct packed {
        logic [31:0] instr;
        logic [31:0] pc;
    } fetch_t;

    // ---------------- Decode stage type ----------------
    typedef struct packed {
        logic [31:0] instr;
        logic [31:0] pc;        // passthrough

        logic [6:0]  opcode;

        logic [4:0]  rs1;       // architectural regs
        logic        rs1Used;

        logic [4:0]  rs2;
        logic        rs2Used;

        logic [4:0]  rd;
        logic        rdUsed;

        logic [31:0] imm;
        logic        immUsed;

        logic        spec;

        // memory info
        logic        lsType;    // 1 = word, 0 = byte/halfword
        logic        loadStore; // 1 = load, 0 = store

        logic        branch;
        logic        jump;
        logic        speculative;

        logic [3:0]  aluop;
        logic [3:0]  funcU;
    } decode_t;

    // ---------------- Rename stage type ----------------
    typedef struct packed {
        // Physical register indices (PREG_W bits!)
        logic [PREG_W-1:0] rs1;
        logic [PREG_W-1:0] rs2;
        logic [PREG_W-1:0] rd;
        logic [PREG_W-1:0] old_rd;
        logic [4:0] arch_rd;

//        // ROB tags
//        logic [ROB_PTR_W-1:0] ROB_tag;
//        logic [ROB_PTR_W-1:0] ROB_tag_clone; // if still needed


        // Core instruction info
        logic [31:0] pc;
        logic [6:0]  opcode;
        logic [31:0] imm;
        logic        imm_used;
        logic        rd_used;

        // Control information
        logic [3:0]  aluop;
        logic [3:0]  funcU;
        logic [1:0]  load_store_type;   // {loadStore, lsType}
        logic        speculative;
        logic        jump;
        logic        branch;
    } rename_t;

    // ---------------- RS / Dispatch entry type ----------------
    typedef struct packed {
        // RS bookkeeping
        logic        valid;
        logic        ready;

        // Physical register indices
        logic [PREG_W-1:0] rs1;
        logic [PREG_W-1:0] rs2;
        logic [PREG_W-1:0] rd;
        logic [PREG_W-1:0] old_rd;

        // ROB tags
        logic [ROB_PTR_W-1:0] ROB_tag;
        logic [ROB_PTR_W-1:0] ROB_tag_clone; // if still needed


        // Core instruction info
        logic [31:0] pc;
        logic [6:0]  opcode;
        logic [31:0] imm;
        logic        imm_used;
        logic        rd_used;

        // Control information
        logic [3:0]  aluop;
        logic [3:0]  funcU;
        logic [1:0]  load_store_type;
        logic        speculative;
        logic        jump;
        logic        branch;

        // Operand values
        logic [31:0] rs1_val;
        logic [31:0] rs2_val;

        // Wakeup / CDB info
        logic        rs1_rdy;
        logic        rs2_rdy;
        logic [6:0]  rs1_tag;   // you can also switch to [PREG_W-1:0] if tags = phys regs
        logic [6:0]  rs2_tag;
    } rs_entry_t;

    typedef rs_entry_t dispatch_t;

    // ---------------- ROB entry type ----------------
    typedef struct packed {
        logic        valid;
        logic        completed;
        logic        has_dest;

        logic [4:0]        arch_rd;
        logic [PREG_W-1:0] dest_preg;
        logic [PREG_W-1:0] old_preg;

        logic        is_branch;
        logic [31:0] pc;
    } rob_entry_t;
    
    typedef struct packed {
        logic [PREG_W-1:0] rd_addr;
        logic [31:0] rs1_val;
        logic rs1_used;
        logic [31:0] rs2_val;
        logic rs2_used;
        logic [31:0] imm_val;
        logic imm_used;
        logic [3:0] aluop;
        logic [ROB_PTR_W-1:0] ROB_tag;
    } alu_entry_t;
    
    typedef struct packed {
        logic [PREG_W-1:0] rd_addr;
        logic [31:0] rd_val;
        logic completed;
        logic [ROB_PTR_W-1:0] ROB_tag;
    } alu_out_t;
    
    typedef struct packed {
        logic [PREG_W-1:0] rd_addr;
        logic [31:0] rs1_val;
        logic rs1_used;
        logic [31:0] rs2_val;
        logic rs2_used;
        logic [31:0] imm_val;
        logic imm_used;
        logic jump;
        logic branch;
        logic [31:0] pc;
        logic took_branch;
        logic [ROB_PTR_W-1:0] ROB_tag;
    } branch_entry_t;
    
    typedef struct packed {
        logic [PREG_W-1:0] rd_addr;
        logic [31:0] rd_val;
        logic [31:0] dest_addr;
        logic branch_taken; //0 if branch not taken, 1 if jump or branch taken 
        logic completed;
        logic mispredict;
        logic [ROB_PTR_W-1:0] ROB_tag;
    } branch_out_t;
    
    typedef struct packed {
        logic [PREG_W-1:0] rd_addr;
        logic [31:0] rs1_val;
        logic rs1_used;
        logic [31:0] rs2_val;
        logic rs2_used;
        logic [31:0] imm_val;
        logic imm_used;
        logic lsType; //1 = load/store word, 0 = load byte unsigned/store halfword, passthrough
        logic loadStore; //1 = load, 0 = store, passthrough
        logic [31:0] pc;
        logic [ROB_PTR_W-1:0] ROB_tag;
    } lsu_entry_t;
    
    typedef struct packed {
        logic [PREG_W-1:0] rd_addr; 
        logic [31:0] rd_val;
        logic completed;
        logic [ROB_PTR_W-1:0] ROB_tag;
    } lsu_out_t;
    
    // ---------------- Unified writeback packet ----------------
    // Can represent ALU / LSU / BR writeback with one type.
    typedef struct packed {
        // PRF/CDB payload
        logic [PREG_W-1:0] rd_addr;     // destination physreg (0 means no write)
        logic [31:0]       rd_val;      // value to write
    
        // ROB completion
        logic              completed;   // usually 1 when valid
        logic              mispredict;  // only meaningful for branches
        logic [ROB_PTR_W-1:0] ROB_tag; // tag used to mark ROB complete
    
        // Optional redirect info (branch/jump)
        logic              branch_taken; // for BR unit
        logic [31:0]       dest_addr;    // redirect target (BR/JAL/JALR)
    
        // Optional type tagging (helps debug)
        logic [1:0]        src_fu;       // 0=ALU,1=LSU,2=BR,3=reserved
    } wb_packet_t;

    
    

endpackage
