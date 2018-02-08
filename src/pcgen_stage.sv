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
// Date: 20.04.2017
// Description: PC generation stage

import ariane_pkg::*;

module pcgen_stage (
    input  logic               clk_i,              // Clock
    input  logic               rst_ni,             // Asynchronous reset active low
    // control signals
    input  logic               flush_i,            // flush request for PCGEN
    input  logic               flush_bp_i,         // flush branch prediction
    input  logic               fetch_enable_i,
    input  logic               if_ready_i,
    input  branchpredict_t     resolved_branch_i,  // from controller signaling a branch_predict -> update BTB
    // to IF
    output logic [63:0]        fetch_address_o,    // new PC (address because we do not distinguish instructions)
    output logic               fetch_valid_o,      // the PC (address) is valid
    output branchpredict_sbe_t branch_predict_o,   // pass on the information if this is speculative
    // global input
    input  logic [63:0]        boot_addr_i,
    // from commit
    input  logic [63:0]        pc_commit_i,        // PC of instruction in commit stage
    // from ID
    input  logic               set_pc_id_i,
    input  logic [63:0]        id_pc_i,
    // CSR input
    input  logic [63:0]        epc_i,              // exception PC which we need to return to
    input  logic               eret_i,             // return from exception
    input  logic [63:0]        trap_vector_base_i, // base of trap vector
    input  logic               ex_valid_i,         // exception is valid - from commit
    // Debug
    input  logic [63:0]        debug_pc_i,         // PC from debug stage
    input  logic               debug_set_pc_i      // Set PC request from debug
);

    logic [63:0]        npc_n, npc_q;
    // the PC was set to a new region by a higher priority input (e.g.: exception, debug, ctrl return from exception)
    logic               set_pc_n, set_pc_q;
    branchpredict_sbe_t branch_predict_btb;
    // branch-predict input register -> this path is critical
    branchpredict_t     resolved_branch_q;

    btb #(
        .NR_ENTRIES              ( BTB_ENTRIES             ),
        .BITS_SATURATION_COUNTER ( BITS_SATURATION_COUNTER )
    )
    btb_i
    (
        // Use the PC from last cycle to perform branch lookup for the current cycle
        .flush_i                 ( flush_bp_i              ),
        .vpc_i                   ( npc_q                   ),
        .branch_predict_i        ( resolved_branch_q       ), // update port
        .branch_predict_o        ( branch_predict_btb      ), // read port
        .*
    );
    // -------------------
    // Next PC
    // -------------------
    // next PC (NPC) can come from (in order of precedence:
    // 0. Default assignment
    // 1. Branch Predict taken
    // 2. Control flow change request
    // 3. Return from environment call
    // 4. Exception/Interrupt
    // 5. Pipeline Flush because of CSR side effects
    // 6. Debug
    // Mis-predict handling is a little bit different
    always_comb begin : npc_select
        automatic logic [63:0] fetch_address;
        fetch_address = npc_q;

        branch_predict_o = branch_predict_btb;
        branch_predict_o.is_call = 1'b0;
        branch_predict_o.dont_update = 1'b0;

        fetch_valid_o    = 1'b1;
        // this tells us whether it is a consecutive PC or a completely new PC
        set_pc_n         = 1'b0;

        // -------------------------------
        // 2. Control flow change request
        // -------------------------------
        // keep the PC stable if IF by default
        npc_n            = npc_q;
        // check if had a mis-predict the cycle earlier and if we can reset the PC (e.g.: it was a predicted or consecutive PC
        // which was set a cycle earlier)
        if (resolved_branch_q.is_mispredict && !set_pc_q) begin
            // we already got the correct target address
            fetch_address = resolved_branch_q.target_address;
        end

        // -------------------------------
        // 0. Default assignment
        // -------------------------------
        // default is a consecutive PC
        if (if_ready_i && fetch_enable_i)
            // but operate on the current fetch address
            npc_n = {fetch_address[63:2], 2'b0}  + 64'h4;


        // we only need to stall the consecutive and predicted case since in any other case we will flush at least
        // the front-end which means that the IF stage will always be ready to accept a new request

        // -------------------------------
        // 1. Predict taken
        // -------------------------------
        // only predict if the IF stage is ready, otherwise we might take the predicted PC away which will end in a endless loop
        // also check if we fetched on a half word (npc_q[1] == 1), it might be the case that we need the next 16 byte of the following instruction
        // prediction could potentially prevent us from getting them
        if (if_ready_i && branch_predict_btb.valid && branch_predict_btb.predict_taken) begin
            if (!fetch_address[1])
                npc_n = branch_predict_btb.predict_address;
            // invalidate branch-prediction as we need to correct a possible predicted (not-taken path)
            // an example can be a prediction from the previous cycle to a mis-aligned instruction (0x..1e), we will need to
            // potentially fetch another 16 byte at least and loose prediction on it example:
            // 0x20002dda bltu             a5, a7, pc + -76 <- predicted
            // 0x20002d8e bnez             a4, pc + 148     <- predicted as well (but kill prediction as the previous instruction could be 32 bit)
            else
                branch_predict_o.valid = 1'b0;
        end

        // -------------------------------
        // 2. Redirect from ID
        // -------------------------------
        if (set_pc_id_i) begin
            npc_n    = id_pc_i;
            set_pc_n = 1'b1;
            branch_predict_o.valid = 1'b0;
        end

        // -------------------------------
        // 3. Return from environment call
        // -------------------------------
        if (eret_i) begin
            npc_n                  = epc_i;
            branch_predict_o.valid = 1'b0;
            set_pc_n               = 1'b1;
        end

        // -------------------------------
        // 4. Exception/Interrupt
        // -------------------------------
        if (ex_valid_i) begin
            npc_n                  = trap_vector_base_i;
            branch_predict_o.valid = 1'b0;
            set_pc_n               = 1'b1;
        end

        // -----------------------------------------------
        // 5. Pipeline Flush because of CSR side effects
        // -----------------------------------------------
        // On a pipeline flush start fetching from the next address
        // of the instruction in the commit stage
        if (flush_i) begin
            // we came here from a flush request of a CSR instruction,
            // as CSR instructions do not exist in a compressed form
            // we can unconditionally do PC + 4 here
            npc_n    = pc_commit_i + 64'h4;
            set_pc_n = 1'b1;
            branch_predict_o.valid = 1'b0;
        end

        // -------------------------------
        // 6. Debug
        // -------------------------------
        if (debug_set_pc_i) begin
            npc_n = debug_pc_i;
            branch_predict_o.valid = 1'b0;
            set_pc_n = 1'b1;
        end

        // fetch enable
        if (!fetch_enable_i) begin
            fetch_valid_o = 1'b0;
        end

        // set fetch address
        fetch_address_o  = fetch_address;

    end

    // -------------------
    // Sequential Process
    // -------------------
    // PCGEN -> IF Pipeline Stage
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if(~rst_ni) begin
           npc_q             <= boot_addr_i;
           set_pc_q          <= 1'b0;
           resolved_branch_q <= '0;
        end else begin
           npc_q             <= npc_n;
           set_pc_q          <= set_pc_n;
           resolved_branch_q <= resolved_branch_i;
        end
    end
endmodule


// Author: Florian Zaruba, ETH Zurich
// Date: 19.04.2017
// Description: Branch Target Buffer implementation
//
// Copyright (C) 2017 ETH Zurich, University of Bologna
// All rights reserved.
module btb #(
    parameter int NR_ENTRIES = 1024,
    parameter int BITS_SATURATION_COUNTER = 2
)(
    input  logic               clk_i,                     // Clock
    input  logic               rst_ni,                    // Asynchronous reset active low
    input  logic               flush_i,                   // flush the btb

    input  logic [63:0]        vpc_i,                     // virtual PC from IF stage
    input  branchpredict_t     branch_predict_i,          // a mis-predict happened -> update data structure

    output branchpredict_sbe_t branch_predict_o           // branch prediction for issuing to the pipeline
);
    // number of bits which are not used for indexing
    localparam OFFSET = 2;
    localparam ANTIALIAS_BITS = 8;
    // number of bits we should use for prediction
    localparam PREDICTION_BITS = $clog2(NR_ENTRIES) + OFFSET;

    // typedef for all branch target entries
    // we may want to try to put a tag field that fills the rest of the PC in-order to mitigate aliasing effects
    struct packed {
        logic                                   valid;
        logic [63:0]                            target_address;
        logic [BITS_SATURATION_COUNTER-1:0]     saturation_counter;
        logic                                   is_lower_16;
        logic [ANTIALIAS_BITS-1:0]              anti_alias; // store some more PC information to prevent aliasing
    } btb_n [NR_ENTRIES-1:0], btb_q [NR_ENTRIES-1:0];

    logic [$clog2(NR_ENTRIES)-1:0]          index, update_pc;
    logic [ANTIALIAS_BITS-1:0]              anti_alias_index, anti_alias_update_pc;
    logic [BITS_SATURATION_COUNTER-1:0]     saturation_counter;

    // get actual index positions
    // we ignore the 0th bit since all instructions are aligned on
    // a half word boundary
    assign update_pc = branch_predict_i.pc[PREDICTION_BITS - 1:OFFSET];
    assign index     = vpc_i[PREDICTION_BITS - 1:OFFSET];
    // anti-alias portion of PCs
    assign anti_alias_update_pc = branch_predict_i.pc[PREDICTION_BITS + ANTIALIAS_BITS - 1:PREDICTION_BITS];
    assign anti_alias_index = vpc_i[PREDICTION_BITS + ANTIALIAS_BITS - 1:PREDICTION_BITS];

    // we combinatorially predict the branch and the target address
    // check if we are potentially aliasing
    assign branch_predict_o.valid           = (btb_q[index].anti_alias == anti_alias_index) ? btb_q[index].valid :  1'b0;
    assign branch_predict_o.predict_taken   = btb_q[index].saturation_counter[BITS_SATURATION_COUNTER-1];
    assign branch_predict_o.predict_address = btb_q[index].target_address;
    assign branch_predict_o.is_lower_16     = btb_q[index].is_lower_16;
    // -------------------------
    // Update Branch Prediction
    // -------------------------
    // update on a mis-predict
    always_comb begin : update_branch_predict
        btb_n              = btb_q;
        saturation_counter = btb_q[update_pc].saturation_counter;
        if (branch_predict_i.valid) begin

            btb_n[update_pc].valid = 1'b1;
            btb_n[update_pc].anti_alias = anti_alias_update_pc;
            // update saturation counter
            // first check if counter is already saturated in the positive regime e.g.: branch taken
            if (saturation_counter == {BITS_SATURATION_COUNTER{1'b1}}) begin
                // we can safely decrease it
                if (~branch_predict_i.is_taken)
                    btb_n[update_pc].saturation_counter = saturation_counter - 1;
            // then check if it saturated in the negative regime e.g.: branch not taken
            end else if (saturation_counter == {BITS_SATURATION_COUNTER{1'b0}}) begin
                // we can safely increase it
                if (branch_predict_i.is_taken)
                    btb_n[update_pc].saturation_counter = saturation_counter + 1;
            end else begin // otherwise we are not in any boundaries and can decrease or increase it
                if (branch_predict_i.is_taken)
                    btb_n[update_pc].saturation_counter = saturation_counter + 1;
                else
                    btb_n[update_pc].saturation_counter = saturation_counter - 1;
            end
            // the target address is simply updated
            btb_n[update_pc].target_address = branch_predict_i.target_address;
            // as is the information whether this was a compressed branch
            btb_n[update_pc].is_lower_16    = branch_predict_i.is_lower_16;
            // check if we should invalidate this entry, this happens in case we predicted a branch
            // where actually none-is (aliasing)
            if (branch_predict_i.clear) begin
                btb_n[update_pc].valid = 1'b0;
            end
        end
    end

    // sequential process
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if(~rst_ni) begin
            // Bias the branches to be taken upon first arrival
            for (int i = 0; i < NR_ENTRIES; i++)
                btb_q[i] <= '{default: 0};
                for (int unsigned i = 0; i < NR_ENTRIES; i++)
                    btb_q[i].saturation_counter <= 2'b0;
        end else begin
            // evict all entries
            if (flush_i) begin
                for (int i = 0; i < NR_ENTRIES; i++) begin
                    btb_q[i].valid              <=  1'b0;
                    btb_q[i].saturation_counter <= '{default: 0};
                end
            end else begin
                btb_q <=  btb_n;
            end

        end
    end
endmodule
