`timescale 1ns / 1ps
import buffer_pkgs::*;

module execute_module #(
    parameter type AE = alu_entry_t,
    parameter type AO = alu_out_t,
    parameter type BE = branch_entry_t,
    parameter type BO = branch_out_t,
    parameter type LE = lsu_entry_t,
    parameter type LO = lsu_out_t
)(
    input logic clk_i,
    input logic reset_i,
    
    //alu inputs
    input logic alu_valid_i,
    output logic alu_ready_o,
    input AE alu_data_i,
    input logic alu_flush_i,
    
    //branch inputs
    input logic b_valid_i,
    output logic b_ready_o,
    input BE b_data_i,
    input logic b_flush_i,
    
    //lsu inputs
    input logic lsu_valid_i,
    output logic lsu_ready_o,
    input LE lsu_data_i,
    input logic lsu_flush_i,

    
    //alu sb outputs
    input logic asb_cons_ready_i,
    output logic asb_cons_valid_o,
    output AO asb_cons_data_o,
    
    //branch sb outputs
    input logic bsb_cons_ready_i,
    output logic bsb_cons_valid_o,
    output BO bsb_cons_data_o,
    
    //lsu sb outputs
    input logic lsb_cons_ready_i,
    output logic lsb_cons_valid_o,
    output LO lsb_cons_data_o
    
    );
    
    //ready-valid signals for FU->SB
    logic asb_prod_ready_o;
    logic asb_prod_valid_i;
    logic bsb_prod_ready_o;
    logic bsb_prod_valid_i;
    logic lsb_prod_ready_o;
    logic lsb_prod_valid_i;
    
    //fu outputs for FU->SB
    AO alu_o;
    BO branch_o;
    LO lsu_o;
    
    //alu and its skid buffer
    alu #(.AE(AE), .AO(AO)) my_alu (
        .clk_i(clk_i),
        .reset_i(reset_i),
        .flush_i(alu_flush_i),
        
        .req_valid_i(alu_valid_i),
        .req_ready_o(alu_ready_o),
        .data_i(alu_data_i),
        
        .resp_valid_o(asb_prod_valid_i),
        .resp_ready_i(asb_prod_ready_o),
        .data_o(alu_o)
    );
    skid_buffer_revised #(.T(AO)) alu_sb (
        .clk(clk_i),
        .reset(reset_i),
        .flush_in(alu_flush_i),
        
        .valid_in(asb_prod_valid_i),
        .ready_in(asb_prod_ready_o),
        .data_in(alu_o),
        
        .valid_out(asb_cons_valid_o),
        .ready_out(asb_cons_ready_i),
        .data_out(asb_cons_data_o)
    );
    
    branch #(.BE(BE), .BO(BO)) my_branch (
        .clk_i(clk_i),
        .reset_i(reset_i),
        .flush_i(b_flush_i),
        
        .req_valid_i(b_valid_i),
        .req_ready_o(b_ready_o),
        .data_i(b_data_i),
        
        .resp_valid_o(bsb_prod_valid_i),
        .resp_ready_i(bsb_prod_ready_o),
        .data_o(branch_o)
    );
    skid_buffer_revised #(.T(BO)) branch_sb (
        .clk(clk_i),
        .reset(reset_i),
        .flush_in(b_flush_i),
        
        .valid_in(bsb_prod_valid_i),
        .ready_in(bsb_prod_ready_o),
        .data_in(branch_o),
        
        .valid_out(bsb_cons_valid_o),
        .ready_out(bsb_cons_ready_i),
        .data_out(bsb_cons_data_o)
    );
    
    lsu #(.LE(LE), .LO(LO)) my_lsu (
        .clk_i(clk_i),
        .reset_i(reset_i),
        .flush_i(lsu_flush_i),
        
        .req_valid_i(lsu_valid_i),
        .req_ready_o(lsu_ready_o),
        .data_i(lsu_data_i),
        
        .resp_valid_o(lsb_prod_valid_i),
        .resp_ready_i(lsb_prod_ready_o),
        .data_o(lsu_o)
    );
    skid_buffer_revised #(.T(LO)) lsu_sb (
        .clk(clk_i),
        .reset(reset_i),
        .flush_in(lsu_flush_i),
        
        .valid_in(lsb_prod_valid_i),
        .ready_in(lsb_prod_ready_o),
        .data_in(lsu_o),
        
        .valid_out(lsb_cons_valid_o),
        .ready_out(lsb_cons_ready_i),
        .data_out(lsb_cons_data_o)
    );
    
endmodule
