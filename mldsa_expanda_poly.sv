`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

module mldsa_expanda_poly (
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire                      rho_we,
    input  wire [4:0]                rho_addr,
    input  wire [7:0]                rho_wdata,
    input  wire                      start,
    input  wire [2:0]                row_i,
    input  wire [2:0]                col_j,
    output logic [7:0]               coeff_index,
    output logic [MLDSA_COEFF_W-1:0] coeff_data,
    output logic                     coeff_valid,
    input  wire                      coeff_ready,
    output logic                     busy,
    output logic                     done,
    output logic                     error,
    output logic                     xof_start_o,
    output logic                     xof_stop_o,
    output logic                     xof_mode_shake256_o,
    output logic [7:0]               xof_abs_byte_data_o,
    output logic                     xof_abs_byte_valid_o,
    output logic                     xof_abs_byte_last_o,
    input  wire                      xof_abs_byte_ready_i,
    input  wire [7:0]                xof_sq_byte_i,
    input  wire                      xof_sq_valid_i,
    output logic                     xof_sq_byte_ready_o,
    input  wire                      xof_state_squeezing_i
);

    typedef enum logic [2:0] {
        S_IDLE,
        S_ABSORB,
        S_WAIT_SQUEEZE,
        S_SAMPLER_START,
        S_SQUEEZE,
        S_STOP,
        S_DONE,
        S_ERROR
    } state_e;

    state_e st;

    logic [7:0] rho_mem [0:31];
    logic [5:0] abs_idx;
    logic       sampler_start;
    logic [7:0] sampler_coeff_index;
    logic [MLDSA_COEFF_W-1:0] sampler_coeff_data;
    logic       sampler_coeff_valid;
    logic       sampler_done;
    logic       sampler_error;
    logic       sampler_byte_ready;

    mldsa_sampler u_sampler (
        .clk             (clk),
        .rst_n           (rst_n),
        .start           (sampler_start),
        .sample_mode     (MLDSA_SAMPLE_UNIFORM),
        .eta             (3'd2),
        .gamma1_is_2p19  (1'b1),
        .tau             (6'd0),
        .byte_data       (xof_sq_byte_i),
        .byte_valid      ((st == S_SQUEEZE) && xof_sq_valid_i),
        .byte_ready      (sampler_byte_ready),
        .coeff_index     (sampler_coeff_index),
        .coeff_data      (sampler_coeff_data),
        .coeff_valid     (sampler_coeff_valid),
        .coeff_ready     (coeff_ready),
        .busy            (),
        .done            (sampler_done),
        .error           (sampler_error),
        .state_dbg       ()
    );

    always_comb begin
        xof_start_o = (st == S_IDLE) && start;
        xof_stop_o  = (st == S_STOP);
        xof_mode_shake256_o = 1'b0;
        xof_abs_byte_valid_o = (st == S_ABSORB);
        xof_abs_byte_last_o  = (abs_idx == 6'd33);
        unique case (abs_idx)
            6'd32: xof_abs_byte_data_o = {5'd0, col_j};
            6'd33: xof_abs_byte_data_o = {5'd0, row_i};
            default: xof_abs_byte_data_o = rho_mem[abs_idx[4:0]];
        endcase

        sampler_start = (st == S_SAMPLER_START);
        xof_sq_byte_ready_o = (st == S_SQUEEZE) && sampler_byte_ready;

        coeff_index = sampler_coeff_index;
        coeff_data  = sampler_coeff_data;
        coeff_valid = (st == S_SQUEEZE) && sampler_coeff_valid;
        busy = (st != S_IDLE) && (st != S_DONE) && (st != S_ERROR);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st      <= S_IDLE;
            abs_idx <= 6'd0;
            done    <= 1'b0;
            error   <= 1'b0;
        end else begin
            done  <= 1'b0;
            error <= 1'b0;

            if (rho_we && !busy) begin
                rho_mem[rho_addr] <= rho_wdata;
            end

            unique case (st)
                S_IDLE: begin
                    abs_idx <= 6'd0;
                    if (start) st <= S_ABSORB;
                end
                S_ABSORB: begin
                    if (xof_abs_byte_valid_o && xof_abs_byte_ready_i) begin
                        if (abs_idx == 6'd33) st <= S_WAIT_SQUEEZE;
                        else abs_idx <= abs_idx + 6'd1;
                    end
                end
                S_WAIT_SQUEEZE: begin
                    if (xof_state_squeezing_i) st <= S_SAMPLER_START;
                end
                S_SAMPLER_START: begin
                    st <= S_SQUEEZE;
                end
                S_SQUEEZE: begin
                    if (sampler_error) st <= S_ERROR;
                    else if (sampler_done) st <= S_STOP;
                end
                S_STOP: begin
                    st <= S_DONE;
                end
                S_DONE: begin
                    done <= 1'b1;
                    st   <= S_IDLE;
                end
                S_ERROR: begin
                    error <= 1'b1;
                    st    <= S_IDLE;
                end
                default: st <= S_ERROR;
            endcase
        end
    end
endmodule

`default_nettype wire
