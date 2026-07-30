`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

module mldsa_fips_poly_pack (
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     start,
    input  wire [2:0]               pack_kind,
    input  wire [2:0]               eta,
    input  wire                     gamma1_is_2p19,
    input  wire [MLDSA_COEFF_W-1:0] coeff_in,
    input  wire                     coeff_in_valid,
    output logic                    coeff_in_ready,
    output logic [7:0]              byte_out,
    output logic                    byte_out_valid,
    input  wire                     byte_out_ready,
    output logic                    busy,
    output logic                    done,
    output logic                    error,
    output logic [3:0]              state_dbg
);

    typedef enum logic [2:0] {
        S_IDLE,
        S_WAIT_COEFF,
        S_EMIT_BYTE,
        S_DONE,
        S_ERROR
    } state_e;

    state_e st;
    logic [2:0] kind_q;
    logic [2:0] eta_q;
    logic gamma1_2p19_q;
    logic [8:0] coeff_count;
    logic [63:0] bitbuf;
    logic [5:0] bitcount;
    logic [5:0] emit_next_bitcount;

    always_comb begin
        if (bitcount >= 6'd8) emit_next_bitcount = bitcount - 6'd8;
        else emit_next_bitcount = 6'd0;
    end

    function automatic integer signed coeff_centered(input logic [MLDSA_COEFF_W-1:0] c);
        integer signed v;
        begin
            v = c;
            if (v > (MLDSA_Q / 2)) v = v - MLDSA_Q;
            coeff_centered = v;
        end
    endfunction

    function automatic logic [4:0] word_bits(
        input logic [2:0] kind,
        input logic [2:0] eta_value,
        input logic gamma1_2p19
    );
        begin
            unique case (kind)
                MLDSA_FIPS_PACK_T1:  word_bits = 5'd10;
                MLDSA_FIPS_PACK_T0:  word_bits = 5'd13;
                MLDSA_FIPS_PACK_ETA: word_bits = (eta_value == 3'd2) ? 5'd3 : 5'd4;
                MLDSA_FIPS_PACK_Z:   word_bits = gamma1_2p19 ? 5'd20 : 5'd18;
                default:             word_bits = 5'd0;
            endcase
        end
    endfunction

    function automatic logic [23:0] pack_word(
        input logic [2:0] kind,
        input logic [2:0] eta_value,
        input logic gamma1_2p19,
        input logic [MLDSA_COEFF_W-1:0] coeff
    );
        integer signed centered;
        integer signed mapped;
        begin
            centered = coeff_centered(coeff);
            unique case (kind)
                MLDSA_FIPS_PACK_T1: begin
                    mapped = coeff[9:0];
                end
                MLDSA_FIPS_PACK_T0: begin
                    mapped = (1 << (MLDSA_D - 1)) - centered;
                end
                MLDSA_FIPS_PACK_ETA: begin
                    mapped = eta_value - centered;
                end
                MLDSA_FIPS_PACK_Z: begin
                    mapped = (gamma1_2p19 ? 20'd524288 : 20'd131072) - centered;
                end
                default: begin
                    mapped = 0;
                end
            endcase
            pack_word = mapped[23:0];
        end
    endfunction

    always_comb begin
        coeff_in_ready = (st == S_WAIT_COEFF) && !byte_out_valid && (bitcount <= 6'd40);
        busy = (st != S_IDLE) && (st != S_DONE) && (st != S_ERROR);
        state_dbg = st;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE;
            kind_q <= MLDSA_FIPS_PACK_T1;
            eta_q <= 3'd4;
            gamma1_2p19_q <= 1'b1;
            coeff_count <= 9'd0;
            bitbuf <= 64'd0;
            bitcount <= 6'd0;
            byte_out <= 8'd0;
            byte_out_valid <= 1'b0;
            done <= 1'b0;
            error <= 1'b0;
        end else begin
            done <= 1'b0;

            if (byte_out_valid && byte_out_ready) begin
                byte_out_valid <= 1'b0;
            end

            unique case (st)
                S_IDLE: begin
                    error <= 1'b0;
                    if (start) begin
                        kind_q <= pack_kind;
                        eta_q <= eta;
                        gamma1_2p19_q <= gamma1_is_2p19;
                        coeff_count <= 9'd0;
                        bitbuf <= 64'd0;
                        bitcount <= 6'd0;
                        byte_out_valid <= 1'b0;
                        if (word_bits(pack_kind, eta, gamma1_is_2p19) == 5'd0) st <= S_ERROR;
                        else st <= S_WAIT_COEFF;
                    end
                end

                S_WAIT_COEFF: begin
                    if (coeff_in_valid && coeff_in_ready) begin
                        bitbuf <= bitbuf | ({40'd0, pack_word(kind_q, eta_q, gamma1_2p19_q, coeff_in)} << bitcount);
                        bitcount <= bitcount + {1'b0, word_bits(kind_q, eta_q, gamma1_2p19_q)};
                        coeff_count <= coeff_count + 9'd1;
                    end
                    if (bitcount >= 6'd8 || (coeff_count == 9'd256 && bitcount != 6'd0)) begin
                        st <= S_EMIT_BYTE;
                    end else if (coeff_count == 9'd256 && bitcount == 6'd0) begin
                        st <= S_DONE;
                    end
                end

                S_EMIT_BYTE: begin
                    if (!byte_out_valid) begin
                        byte_out <= bitbuf[7:0];
                        byte_out_valid <= 1'b1;
                        bitbuf <= bitbuf >> 8;
                        bitcount <= emit_next_bitcount;
                        if (emit_next_bitcount < 6'd8) begin
                            if (coeff_count == 9'd256) begin
                                if (emit_next_bitcount == 6'd0) st <= S_DONE;
                                else st <= S_EMIT_BYTE;
                            end
                            else st <= S_WAIT_COEFF;
                        end
                    end
                end

                S_DONE: begin
                    if (!byte_out_valid) begin
                        done <= 1'b1;
                        st <= S_IDLE;
                    end
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
