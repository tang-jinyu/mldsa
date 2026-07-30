`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

module mldsa_bfu (
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire                      start,
    input  wire                      inverse,
    input  wire [MLDSA_COEFF_W-1:0]  a_in,
    input  wire [MLDSA_COEFF_W-1:0]  b_in,
    input  wire [MLDSA_COEFF_W-1:0]  zeta,
    output logic [MLDSA_COEFF_W-1:0] a_out,
    output logic [MLDSA_COEFF_W-1:0] b_out,
    output logic                     busy,
    output logic                     done
);

    typedef enum logic [2:0] {S_IDLE, S_MUL_START, S_MUL_WAIT, S_DONE} state_e;
    state_e st;

    logic [MLDSA_COEFF_W-1:0] a_q;
    logic [MLDSA_COEFF_W-1:0] b_q;
    logic [MLDSA_COEFF_W-1:0] zeta_q;
    logic                     inv_q;
    logic                     mul_start;
    logic [MLDSA_COEFF_W-1:0] mul_a;
    logic [MLDSA_COEFF_W-1:0] mul_b;
    logic [MLDSA_COEFF_W-1:0] mul_result;
    logic                     mul_busy;
    logic                     mul_done;

    mldsa_modq_mul u_mul (
        .clk    (clk),
        .rst_n  (rst_n),
        .start  (mul_start),
        .a      (mul_a),
        .b      (mul_b),
        .result (mul_result),
        .busy   (mul_busy),
        .done   (mul_done)
    );

    always_comb begin
        mul_start = (st == S_MUL_START);
        mul_a     = zeta_q;
        mul_b     = inv_q ? modq_sub(a_q, b_q) : b_q;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            st     <= S_IDLE;
            a_q    <= 24'd0;
            b_q    <= 24'd0;
            zeta_q <= 24'd0;
            inv_q  <= 1'b0;
            a_out  <= 24'd0;
            b_out  <= 24'd0;
            busy   <= 1'b0;
            done   <= 1'b0;
        end else begin
            done <= 1'b0;
            unique case (st)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        a_q    <= a_in;
                        b_q    <= b_in;
                        zeta_q <= zeta;
                        inv_q  <= inverse;
                        busy   <= 1'b1;
                        st     <= S_MUL_START;
                    end
                end
                S_MUL_START: begin
                    st <= S_MUL_WAIT;
                end
                S_MUL_WAIT: begin
                    if (mul_done) begin
                        if (inv_q) begin
                            a_out <= modq_add(a_q, b_q);
                            b_out <= mul_result;
                        end else begin
                            a_out <= modq_add(a_q, mul_result);
                            b_out <= modq_sub(a_q, mul_result);
                        end
                        st <= S_DONE;
                    end
                end
                S_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    st   <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule

module mldsa_bfu_pipe (
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire                      in_valid,
    input  wire                      inverse,
    input  wire [MLDSA_COEFF_W-1:0]  a_in,
    input  wire [MLDSA_COEFF_W-1:0]  b_in,
    input  wire [MLDSA_COEFF_W-1:0]  zeta,
    output wire                      in_ready,
    output logic                     out_valid,
    output logic [MLDSA_COEFF_W-1:0] a_out,
    output logic [MLDSA_COEFF_W-1:0] b_out
);

    logic [MLDSA_COEFF_W-1:0] mul_b;
    logic [MLDSA_COEFF_W-1:0] mul_result;
    logic                     mul_in_ready;
    logic                     mul_out_valid;

    logic [MLDSA_COEFF_W-1:0] a_d0;
    logic [MLDSA_COEFF_W-1:0] a_d1;
    logic [MLDSA_COEFF_W-1:0] a_d2;
    logic [MLDSA_COEFF_W-1:0] b_d0;
    logic [MLDSA_COEFF_W-1:0] b_d1;
    logic [MLDSA_COEFF_W-1:0] b_d2;
    logic                     inv_d0;
    logic                     inv_d1;
    logic                     inv_d2;

`ifdef MLDSA_NTT_FOLD_INTT_HALF
    function automatic logic [MLDSA_COEFF_W-1:0] modq_half(
        input logic [MLDSA_COEFF_W-1:0] x
    );
        logic [MLDSA_COEFF_W-1:0] shifted;
        begin
            shifted = x >> 1;
            modq_half = x[0] ? modq_add(shifted, 24'd4190209) : shifted;
        end
    endfunction
`endif

    assign in_ready = mul_in_ready;
    assign mul_b    = inverse ? modq_sub(a_in, b_in) : b_in;

    mldsa_modq_mul_pipe u_mul_pipe (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (in_valid),
        .a         (zeta),
        .b         (mul_b),
        .in_ready  (mul_in_ready),
        .out_valid (mul_out_valid),
        .result    (mul_result)
    );

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            a_d0      <= '0;
            a_d1      <= '0;
            a_d2      <= '0;
            b_d0      <= '0;
            b_d1      <= '0;
            b_d2      <= '0;
            inv_d0    <= 1'b0;
            inv_d1    <= 1'b0;
            inv_d2    <= 1'b0;
            out_valid <= 1'b0;
            a_out     <= '0;
            b_out     <= '0;
        end else begin
            if (in_valid) begin
                a_d0   <= a_in;
                b_d0   <= b_in;
                inv_d0 <= inverse;
            end
            a_d1   <= a_d0;
            a_d2   <= a_d1;
            b_d1   <= b_d0;
            b_d2   <= b_d1;
            inv_d1 <= inv_d0;
            inv_d2 <= inv_d1;

            out_valid <= mul_out_valid;
            if (mul_out_valid) begin
                if (inv_d2) begin
`ifdef MLDSA_NTT_FOLD_INTT_HALF
                    a_out <= modq_half(modq_add(a_d2, b_d2));
                    b_out <= modq_half(mul_result);
`else
                    a_out <= modq_add(a_d2, b_d2);
                    b_out <= mul_result;
`endif
                end else begin
                    a_out <= modq_add(a_d2, mul_result);
                    b_out <= modq_sub(a_d2, mul_result);
                end
            end
        end
    end
endmodule

`default_nettype wire

//前者是单输入的  后者是可连续输入的流水结构
