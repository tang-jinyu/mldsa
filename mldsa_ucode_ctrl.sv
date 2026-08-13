`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

module mldsa_ucode_ctrl (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [1:0]  op_mode,
    input  wire        step_done,
    input  wire        step_retry,
    input  wire        step_fail,
    output logic       uop_valid,
    output logic [7:0] uop_code,
    output logic       busy,
    output logic       done,
    output logic       error,
    output logic [7:0] pc_dbg
);

    localparam logic [7:0] SIGN_ENTRY   = 8'd1;
    localparam logic [7:0] SIGN_RETRY   = 8'd4;
    localparam logic [7:0] VERIFY_ENTRY = 8'd17;
    localparam logic [7:0] KEYGEN_ENTRY = 8'd32;
    logic [1:0] mode_q;

    function automatic logic [7:0] rom(input logic [7:0] pc);
        begin
            unique case (pc)
                8'd0:  rom = MLDSA_UOP_SEED_EXPAND;
                8'd1:  rom = MLDSA_UOP_EXPAND_A;
                8'd2:  rom = MLDSA_UOP_HASH_MU;
                8'd3:  rom = MLDSA_UOP_PREP_T1;
                8'd4:  rom = MLDSA_UOP_SAMPLE_Y;
                8'd5:  rom = MLDSA_UOP_MATVEC_AY;
                8'd6:  rom = MLDSA_UOP_PACK_W1_SIGN;
                8'd7:  rom = MLDSA_UOP_HASH_SIGN_W1;
                8'd8:  rom = MLDSA_UOP_SAMPLE_C_SIGN;
                8'd9:  rom = MLDSA_UOP_MUL_CS1_ADD_Z;
                8'd10: rom = MLDSA_UOP_CHECK_Z;
                8'd11: rom = MLDSA_UOP_MAKE_HINT;
                8'd12: rom = MLDSA_UOP_PACK_HINT;
                8'd13: rom = MLDSA_UOP_SIGN_DONE;
                8'd17: rom = MLDSA_UOP_EXPAND_A;
                8'd18: rom = MLDSA_UOP_HASH_MU;
                8'd19: rom = MLDSA_UOP_SAMPLE_C_VERIFY;
                8'd20: rom = MLDSA_UOP_MATVEC_AZ;
                8'd21: rom = MLDSA_UOP_USE_HINT;
                8'd22: rom = MLDSA_UOP_PACK_W1_VERIFY;
                8'd23: rom = MLDSA_UOP_HASH_VERIFY_W1;
                8'd24: rom = MLDSA_UOP_COMPARE_C;
                8'd25: rom = MLDSA_UOP_VERIFY_DONE;
                8'd32: rom = MLDSA_UOP_SEED_EXPAND;
                8'd33: rom = MLDSA_UOP_EXPAND_A;
                8'd34: rom = MLDSA_UOP_SAMPLE_S1S2;
                8'd35: rom = MLDSA_UOP_PREP_T1;
                8'd36: rom = MLDSA_UOP_KEYGEN_DONE;
                default: rom = MLDSA_UOP_NOP;
            endcase
        end
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy   <= 1'b0;
            done   <= 1'b0;
            error  <= 1'b0;
            pc_dbg <= 8'd0;
            mode_q <= MLDSA_OP_KEYGEN;
        end else begin
            done  <= 1'b0;
            error <= 1'b0;
            if (!busy) begin
                if (start) begin
                    busy   <= 1'b1;
                    mode_q <= op_mode;
                    if (op_mode == MLDSA_OP_VERIFY) pc_dbg <= VERIFY_ENTRY;
                    else if (op_mode == MLDSA_OP_KEYGEN) pc_dbg <= KEYGEN_ENTRY;
                    else pc_dbg <= SIGN_ENTRY;
                end
            end else begin
                if (step_fail) begin
                    busy  <= 1'b0;
                    error <= 1'b1;
                end else if (step_retry && mode_q == MLDSA_OP_SIGN) begin
                    pc_dbg <= SIGN_RETRY;
                end else if (rom(pc_dbg) == MLDSA_UOP_SIGN_DONE || rom(pc_dbg) == MLDSA_UOP_VERIFY_DONE || rom(pc_dbg) == MLDSA_UOP_KEYGEN_DONE) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end else if (step_done) begin
                    pc_dbg <= pc_dbg + 8'd1;
                end
            end
        end
    end

    always_comb begin
        uop_valid = busy;
        uop_code  = rom(pc_dbg);
    end
endmodule

`default_nettype wire
