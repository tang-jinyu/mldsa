`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

module mldsa_eta_expand_poly (
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire                      rho_prime_we,
    input  wire [5:0]                rho_prime_addr,
    input  wire [7:0]                rho_prime_wdata,
    input  wire                      start,
    input  wire [15:0]               nonce,
    input  wire [2:0]                eta,
    output logic [7:0]               coeff_index,
    output logic [MLDSA_COEFF_W-1:0] coeff_data,
    output logic                     coeff_valid,
    input  wire                      coeff_ready,
    output logic                     busy,
    output logic                     done,
    output logic                     error,
    output logic [3:0]               state_dbg
);

    typedef enum logic [3:0] {
        S_IDLE,
        S_XOF_START,
        S_ABSORB,
        S_SAMPLER_START,
        S_SQUEEZE,
        S_STOP,
        S_DONE,
        S_ERROR
    } state_e;

    state_e st;
    logic [6:0] absorb_idx;
    logic [5:0] rho_prime_rd_addr;
    logic [7:0] rho_prime_rd_data;

    logic xof_start;
    logic xof_stop;
    logic [7:0] xof_abs_byte_data;
    logic xof_abs_byte_valid;
    logic xof_abs_byte_last;
    logic xof_abs_ready;
    logic [7:0] xof_sq_byte;
    logic xof_sq_valid;
    logic xof_sq_byte_ready;

    logic sampler_start;
    logic sampler_byte_ready;
    logic sampler_done;
    logic sampler_error;

    mldsa_byte_store_async #(.DEPTH(64), .ADDR_W(6)) u_rhop_store (
        .clk    (clk),
        .wr_en  (rho_prime_we),
        .wr_addr(rho_prime_addr),
        .wr_data(rho_prime_wdata),
        .rd_addr(rho_prime_rd_addr),
        .rd_data(rho_prime_rd_data)
    );

    mldsa_xof_byte_engine u_xof_bytes (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (xof_start),
        .stop           (xof_stop),
        .mode_shake256  (1'b1),
        .abs_byte_data  (xof_abs_byte_data),
        .abs_byte_valid (xof_abs_byte_valid),
        .abs_byte_last  (xof_abs_byte_last),
        .abs_byte_ready (xof_abs_ready),
        .sq_byte_data   (xof_sq_byte),
        .sq_byte_valid  (xof_sq_valid),
        .sq_byte_ready  (xof_sq_byte_ready),
        .sq_word_data   (),
        .sq_word_valid  (),
        .sq_word_ready  (1'b0),
        .busy           (),
        .state_absorbing(),
        .state_squeezing()
    );

    mldsa_sampler u_sampler (
        .clk             (clk),
        .rst_n           (rst_n),
        .start           (sampler_start),
        .sample_mode     (MLDSA_SAMPLE_ETA),
        .eta             (eta),
        .gamma1_is_2p19  (1'b1),
        .tau             (6'd0),
        .byte_data       (xof_sq_byte),
        .byte_valid      (xof_sq_valid),
        .byte_ready      (sampler_byte_ready),
        .word_data       (64'd0),
        .word_valid      (1'b0),
        .word_ready      (),
        .coeff_index     (coeff_index),
        .coeff_data      (coeff_data),
        .coeff_valid     (coeff_valid),
        .coeff_ready     (coeff_ready),
        .busy            (),
        .done            (sampler_done),
        .error           (sampler_error),
        .state_dbg       ()
    );

    always_comb begin
        xof_start = 1'b0;
        xof_stop  = 1'b0;
        xof_abs_byte_valid = 1'b0;
        xof_abs_byte_last  = 1'b0;
        xof_abs_byte_data  = 8'd0;
        xof_sq_byte_ready  = 1'b0;
        sampler_start      = 1'b0;
        rho_prime_rd_addr  = absorb_idx[5:0];
        busy               = (st != S_IDLE) && (st != S_DONE) && (st != S_ERROR);
        state_dbg          = st;

        unique case (st)
            S_XOF_START: begin
                xof_start = 1'b1;
            end
            S_ABSORB: begin
                xof_abs_byte_valid = 1'b1;
                xof_abs_byte_last  = (absorb_idx == 7'd65);
                if (absorb_idx < 7'd64) begin
                    xof_abs_byte_data = rho_prime_rd_data;
                end else if (absorb_idx == 7'd64) begin
                    xof_abs_byte_data = nonce[7:0];
                end else begin
                    xof_abs_byte_data = nonce[15:8];
                end
            end
            S_SAMPLER_START: begin
                sampler_start = 1'b1;
            end
            S_SQUEEZE: begin
                xof_sq_byte_ready = sampler_byte_ready;
            end
            S_STOP: begin
                xof_stop = 1'b1;
            end
            default: begin
            end
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st         <= S_IDLE;
            absorb_idx <= 7'd0;
            done       <= 1'b0;
            error      <= 1'b0;
        end else begin
            done <= 1'b0;
            unique case (st)
                S_IDLE: begin
                    error <= 1'b0;
                    if (start) begin
                        absorb_idx <= 7'd0;
                        st <= S_XOF_START;
                    end
                end
                S_XOF_START: begin
                    st <= S_ABSORB;
                end
                S_ABSORB: begin
                    if (xof_abs_ready) begin
                        if (absorb_idx == 7'd65) begin
                            st <= S_SAMPLER_START;
                        end else begin
                            absorb_idx <= absorb_idx + 7'd1;
                        end
                    end
                end
                S_SAMPLER_START: begin
                    st <= S_SQUEEZE;
                end
                S_SQUEEZE: begin
                    if (sampler_error) begin
                        error <= 1'b1;
                        st <= S_ERROR;
                    end else if (sampler_done) begin
                        st <= S_STOP;
                    end
                end
                S_STOP: begin
                    st <= S_DONE;
                end
                S_DONE: begin
                    done <= 1'b1;
                    st <= S_IDLE;
                end
                S_ERROR: begin
                    error <= 1'b1;
                    if (!start) st <= S_IDLE;
                end
                default: st <= S_ERROR;
            endcase
        end
    end
endmodule

`default_nettype wire
