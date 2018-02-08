// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba, ETH Zurich
// Date: 15.04.2017
// Description: Description: Instruction decode, contains the logic for decode,
//              issue and read operands.

import ariane_pkg::*;

module id_stage (
    input  logic                                     clk_i,     // Clock
    input  logic                                     rst_ni,    // Asynchronous reset active low

    input  logic                                     flush_i,
    // from IF
    input  fetch_entry_t                             fetch_entry_i,
    input  logic                                     fetch_entry_valid_i,
    output logic                                     decoded_instr_ack_o, // acknowledge the instruction (fetch entry)

    // to ID
    output scoreboard_entry_t                        issue_entry_o,       // a decoded instruction
    output logic                                     issue_entry_valid_o, // issue entry is valid
    output logic                                     is_ctrl_flow_o,      // the instruction we issue is a ctrl flow instructions
    input  logic                                     issue_instr_ack_i,   // issue stage acknowledged sampling of instructions
    // from CSR file
    input  priv_lvl_t                                priv_lvl_i,          // current privilege level
    input  logic                                     tvm_i,
    input  logic                                     tw_i,
    input  logic                                     tsr_i,
    // from branch unit
    input  update_ras_t                              update_ras_i,        // update RAS
    output logic                                     set_pc_id_o,
    output logic [63:0]                              id_pc_o
);
    // register stage
    struct packed {
        logic              valid;
        scoreboard_entry_t sbe;
        logic              is_ctrl_flow;

    } issue_n, issue_q;

    logic                is_control_flow_instr;
    scoreboard_entry_t   instr_from_decoder;
    scoreboard_entry_t   instr_from_early_branch;

    fetch_entry_t        fetch_entry;
    logic                is_illegal;
    logic                [31:0] instruction;
    logic                is_compressed;
    logic                fetch_ack_i;
    logic                fetch_entry_valid;

    // ---------------------------------------------------------
    // 1. Re-align instructions
    // ---------------------------------------------------------
    instr_realigner instr_realigner_i (
        .fetch_entry_0_i         ( fetch_entry_i               ),
        .fetch_entry_valid_0_i   ( fetch_entry_valid_i         ),
        .fetch_ack_0_o           ( decoded_instr_ack_o         ),

        .fetch_entry_o           ( fetch_entry                 ),
        .fetch_entry_valid_o     ( fetch_entry_valid           ),
        .fetch_ack_i             ( fetch_ack_i                 ),
        .*
    );
    // ---------------------------------------------------------
    // 2. Check if they are compressed and expand in case they are
    // ---------------------------------------------------------
    compressed_decoder compressed_decoder_i (
        .instr_i                 ( fetch_entry.instruction     ),
        .instr_o                 ( instruction                 ),
        .illegal_instr_o         ( is_illegal                  ),
        .is_compressed_o         ( is_compressed               )

    );
    // ---------------------------------------------------------
    // 3. Decode and emit instruction to issue stage
    // ---------------------------------------------------------
    decoder decoder_i (
        .pc_i                    ( fetch_entry.address         ),
        .is_compressed_i         ( is_compressed               ),
        .instruction_i           ( instruction                 ),
        .branch_predict_i        ( fetch_entry.branch_predict  ),
        .is_illegal_i            ( is_illegal                  ),
        .ex_i                    ( fetch_entry.ex              ),
        .instruction_o           ( instr_from_decoder          ),
        .is_control_flow_instr_o ( is_control_flow_instr       ),
        .*
    );

    // ------------------
    // Pipeline Register
    // ------------------
    assign issue_entry_o = issue_q.sbe;
    assign issue_entry_valid_o = issue_q.valid;
    assign is_ctrl_flow_o = issue_q.is_ctrl_flow;

    always_comb begin
        issue_n     = issue_q;
        fetch_ack_i = 1'b0;

        // Clear the valid flag if issue has acknowledged the instruction
        if (issue_instr_ack_i)
            issue_n.valid = 1'b0;

        // if we have a space in the register and the fetch is valid, go get it
        // or the issue stage is currently acknowledging an instruction, which means that we will have space
        // for a new instruction
        if ((!issue_q.valid || issue_instr_ack_i) && fetch_entry_valid) begin
            fetch_ack_i = 1'b1;
            issue_n = {1'b1, instr_from_early_branch, is_control_flow_instr};
        end

        // invalidate the pipeline register on a flush except if we are triggering the flush ourself
        // because we requested a new PC from IF stage
        if (flush_i && !set_pc_id_o)
            issue_n.valid = 1'b0;
    end

    // -----------------------
    // Early Branching Logic
    // -----------------------
    logic        pop_ras;
    logic [63:0] ra; // return address
    logic        ret_instr;

    always_comb begin
        automatic logic ret_instr;
        instr_from_early_branch = instr_from_decoder;
        pop_ras = 1'b0;
        // detected a return instruction
        ret_instr = is_ret(instruction_t'(instruction));

        set_pc_id_o = 1'b0;
        // set the PC to return address
        id_pc_o = ra;
        // TODO: this can be more performant as we can redirect the branching logic as soon as we have decoded
        // a valid instruction
        if (ret_instr && fetch_ack_i) begin
            set_pc_id_o = 1'b1;
            pop_ras = 1'b1;
            // update branch-prediction data structure in case we mis-predicted here
            instr_from_early_branch.bp.valid = 1'b1;
            instr_from_early_branch.bp.predict_taken = 1'b1;
            instr_from_early_branch.bp.dont_update = 1'b1;
            instr_from_early_branch.bp.predict_address = id_pc_o;
        end

    end

    // RAS
    ras #(
        .DEPTH ( RAS_DEPTH )
    ) i_ras (
        .push_i ( update_ras_i.valid ),
        .data_i ( update_ras_i.ra    ),
        .data_o ( ra                 ),
        .pop_i  ( pop_ras            ),
        .*
    );

    // -------------------------
    // Registers (ID <-> Issue)
    // -------------------------
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (~rst_ni) begin
            issue_q <= '0;
        end else begin
            issue_q <= issue_n;
        end
    end

endmodule

// Description: Return Address Stack
module ras #(
    parameter int unsigned DEPTH = 2
)(
    input  logic        clk_i,
    input  logic        rst_ni,
    input  logic        push_i,
    input  logic        pop_i,
    input  logic [63:0] data_i,
    output logic [63:0] data_o
);

    logic [DEPTH-1:0][63:0] stack_d, stack_q;

    assign data_o = stack_q[0];

    always_comb begin
        stack_d = stack_q;

        // push on the stack
        if (push_i) begin
            stack_d[0] = data_i;
            stack_d[DEPTH-1:1] = stack_q[DEPTH-2:0];
        end

        if (pop_i) begin
            stack_d[DEPTH-2:0] = stack_q[DEPTH-1:1];
        end
        // just change the uppermost element
        if (push_i && pop_i) begin
            stack_d[0] = data_i;
        end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (~rst_ni) begin
            stack_q <= '0;
        end else begin
            stack_q <= stack_d;
        end
    end
endmodule
