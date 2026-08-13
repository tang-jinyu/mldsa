`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

module mldsa_decompose_hint (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         start,
    input  wire [1:0]                   op_code,
    input  wire [MLDSA_COEFF_W-1:0]     coeff_in,
    input  wire [MLDSA_COEFF_W-1:0]     aux_in,
    input  wire [19:0]                  gamma2,
    output logic [MLDSA_COEFF_W-1:0]    high_out,
    output logic [MLDSA_COEFF_W-1:0]    low_out,
    output logic                        hint_out,
    output logic                        busy,
    output logic                        done,
    output logic [2:0]                  state_dbg
);

    typedef enum logic [2:0] {
        S_IDLE,
        S_PREP,
        S_A1_MUL,
        S_A1_POST,
        S_A0_MUL,
        S_FINAL,
        S_DONE
    } state_e;
    state_e st;

    logic [1:0] op_q;
    logic [MLDSA_COEFF_W-1:0] coeff_q;
    logic [MLDSA_COEFF_W-1:0] aux_q;
    logic [19:0] gamma2_q;
    logic signed [31:0] coeff_s_q;
    logic signed [31:0] aux_s_q;
    logic signed [31:0] gamma2_s_q;
    logic signed [31:0] a1_pre_q;
    logic signed [55:0] a1_mul_q;
    logic signed [31:0] a1_q;
    logic signed [55:0] a0_mul_q;
    logic signed [31:0] a0_q;
    logic signed [31:0] p2r_high_q;
    logic signed [31:0] p2r_low_q;
`ifdef MLDSA_TARGET_FPGA
    logic signed [31:0] mh_a1_pre_q;
    logic signed [55:0] mh_a1_mul_q;
    logic signed [31:0] mh_a1_q;
`endif

    function automatic logic [MLDSA_COEFF_W-1:0] highbits_fast(
        input logic [MLDSA_COEFF_W-1:0] coeff,
        input logic [19:0] gamma2_value
    );
        logic signed [31:0] a1_pre;
        logic signed [55:0] a1_mul;
        logic signed [31:0] a1_tmp;
        begin
            a1_pre = ($signed({8'd0, coeff}) + 32'sd127) >>> 7;
            if (gamma2_value == 20'd261888) begin
                a1_mul = (a1_pre * 32'sd1025) + (56'sd1 <<< 21);
                a1_tmp = (a1_mul >>> 22) & 32'sd15;
            end else begin
                a1_mul = (a1_pre * 32'sd11275) + (56'sd1 <<< 23);
                a1_tmp = a1_mul >>> 24;
                if (a1_tmp > 32'sd43) a1_tmp = 32'sd0;
            end
            highbits_fast = a1_tmp[MLDSA_COEFF_W-1:0];
        end
    endfunction

    function automatic logic [MLDSA_COEFF_W-1:0] modq_add_signed(
        input logic [MLDSA_COEFF_W-1:0] coeff,
        input logic signed [31:0] delta
    );
        logic signed [33:0] sum;
        begin
            sum = $signed({10'd0, coeff}) + delta;
            if (sum < 34'sd0) begin
                sum = sum + 34'sd8380417;
            end else if (sum >= 34'sd8380417) begin
                sum = sum - 34'sd8380417;
            end
            modq_add_signed = sum[MLDSA_COEFF_W-1:0];
        end
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_IDLE;
            op_q     <= MLDSA_DH_POWER2ROUND;
            coeff_q  <= '0;
            aux_q    <= '0;
            gamma2_q <= 20'd261888;
            coeff_s_q <= '0;
            aux_s_q <= '0;
            gamma2_s_q <= 32'sd261888;
            a1_pre_q <= '0;
            a1_mul_q <= '0;
            a1_q <= '0;
            a0_mul_q <= '0;
            a0_q <= '0;
            p2r_high_q <= '0;
            p2r_low_q <= '0;
`ifdef MLDSA_TARGET_FPGA
            mh_a1_pre_q <= '0;
            mh_a1_mul_q <= '0;
            mh_a1_q <= '0;
`endif
            high_out <= '0;
            low_out  <= '0;
            hint_out <= 1'b0;
            busy     <= 1'b0;
            done     <= 1'b0;
        end else begin
            done <= 1'b0;
            unique case (st)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        op_q     <= op_code;
                        coeff_q  <= coeff_in;
                        aux_q    <= aux_in;
                        gamma2_q <= gamma2;
                        busy     <= 1'b1;
                        st       <= S_PREP;
                    end
                end
                S_PREP: begin
                    coeff_s_q  <= $signed({8'd0, coeff_q});
                    aux_s_q    <= decode_s24(aux_q);
                    gamma2_s_q <= $signed({12'd0, gamma2_q});
`ifdef MLDSA_TARGET_FPGA
                    if (op_q == MLDSA_DH_MAKE_HINT) begin
                        // FPGA timing path: full_core passes raw w-c*t0 and c*t0.
                        // Register both first, then form highbits(w) vs highbits(w-c*t0).
                        a1_pre_q    <= ($signed({8'd0, modq_add(coeff_q, aux_q)}) + 32'sd127) >>> 7;
                        mh_a1_pre_q <= ($signed({8'd0, coeff_q}) + 32'sd127) >>> 7;
                    end else begin
                        a1_pre_q    <= ($signed({8'd0, coeff_q}) + 32'sd127) >>> 7;
                        mh_a1_pre_q <= ($signed({8'd0, modq_add_signed(coeff_q, decode_s24(aux_q))}) + 32'sd127) >>> 7;
                    end
`else
                    a1_pre_q   <= ($signed({8'd0, coeff_q}) + 32'sd127) >>> 7;
`endif
                    p2r_high_q <= ($signed({8'd0, coeff_q}) + ((32'sd1 <<< (MLDSA_D - 1)) - 32'sd1)) >>> MLDSA_D;
                    p2r_low_q  <= $signed({8'd0, coeff_q}) -
                                  ((($signed({8'd0, coeff_q}) + ((32'sd1 <<< (MLDSA_D - 1)) - 32'sd1)) >>> MLDSA_D) <<< MLDSA_D);
                    hint_out <= 1'b0;
                    unique case (op_q)
                        MLDSA_DH_POWER2ROUND: begin
                            st <= S_FINAL;
                        end
                        MLDSA_DH_DECOMPOSE: begin
                            st <= S_A1_MUL;
                        end
                        MLDSA_DH_MAKE_HINT: begin
`ifdef MLDSA_TARGET_FPGA
                            st <= S_A1_MUL;
`else
                            st <= S_FINAL;
`endif
                        end
                        MLDSA_DH_USE_HINT: begin
                            st <= S_A1_MUL;
                        end
                        default: begin
                            st <= S_FINAL;
                        end
                    endcase
                end
                S_A1_MUL: begin
                    if (gamma2_q == 20'd261888) begin
                        a1_mul_q <= (a1_pre_q * 32'sd1025) + (56'sd1 <<< 21);
`ifdef MLDSA_TARGET_FPGA
                        mh_a1_mul_q <= (mh_a1_pre_q * 32'sd1025) + (56'sd1 <<< 21);
`endif
                    end else begin
                        a1_mul_q <= (a1_pre_q * 32'sd11275) + (56'sd1 <<< 23);
`ifdef MLDSA_TARGET_FPGA
                        mh_a1_mul_q <= (mh_a1_pre_q * 32'sd11275) + (56'sd1 <<< 23);
`endif
                    end
                    st <= S_A1_POST;
                end
                S_A1_POST: begin
                    logic signed [31:0] a1_tmp;
`ifdef MLDSA_TARGET_FPGA
                    logic signed [31:0] mh_a1_tmp;
`endif
                    if (gamma2_q == 20'd261888) begin
                        a1_tmp = (a1_mul_q >>> 22) & 32'sd15;
`ifdef MLDSA_TARGET_FPGA
                        mh_a1_tmp = (mh_a1_mul_q >>> 22) & 32'sd15;
`endif
                    end else begin
                        a1_tmp = a1_mul_q >>> 24;
                        if (a1_tmp > 32'sd43) a1_tmp = 32'sd0;
`ifdef MLDSA_TARGET_FPGA
                        mh_a1_tmp = mh_a1_mul_q >>> 24;
                        if (mh_a1_tmp > 32'sd43) mh_a1_tmp = 32'sd0;
`endif
                    end
                    a1_q <= a1_tmp;
`ifdef MLDSA_TARGET_FPGA
                    mh_a1_q <= mh_a1_tmp;
`endif
                    a0_mul_q <= a1_tmp * (gamma2_s_q <<< 1);
                    st <= S_A0_MUL;
                end
                S_A0_MUL: begin
                    logic signed [31:0] a0_tmp;
                    a0_tmp = coeff_s_q - a0_mul_q[31:0];
                    if (a0_tmp > 32'sd4190208) begin
                        a0_tmp = a0_tmp - 32'sd8380417;
                    end else if (a0_tmp < -32'sd4190208) begin
                        a0_tmp = a0_tmp + 32'sd8380417;
                    end
                    a0_q <= a0_tmp;
                    st <= S_FINAL;
                end
                S_FINAL: begin
                    unique case (op_q)
                        MLDSA_DH_POWER2ROUND: begin
                            high_out <= p2r_high_q[MLDSA_COEFF_W-1:0];
                            low_out  <= encode_s24(p2r_low_q);
                            hint_out <= 1'b0;
                        end
                        MLDSA_DH_DECOMPOSE: begin
                            high_out <= a1_q[MLDSA_COEFF_W-1:0];
                            low_out  <= encode_s24(a0_q);
                            hint_out <= 1'b0;
                        end
                        MLDSA_DH_MAKE_HINT: begin
`ifdef MLDSA_TARGET_FPGA
                            hint_out <= (a1_q != mh_a1_q);
`else
                            hint_out <= (highbits_fast(coeff_q, gamma2_q) !=
                                         highbits_fast(modq_add_signed(coeff_q, aux_s_q), gamma2_q));
`endif
                            high_out <= coeff_q;
                            low_out  <= aux_q;
                        end
                        MLDSA_DH_USE_HINT: begin
                            hint_out <= 1'b0;
                            if (!aux_q[0]) begin
                                high_out <= a1_q[MLDSA_COEFF_W-1:0];
                            end else if (gamma2_q == 20'd261888) begin
                                if (a0_q > 0) high_out <= (a1_q + 32'sd1) & 24'd15;
                                else high_out <= (a1_q - 32'sd1) & 24'd15;
                            end else begin
                                if (a0_q > 0) begin
                                    if (a1_q == 32'sd43) high_out <= '0;
                                    else high_out <= (a1_q + 32'sd1);
                                end else begin
                                    if (a1_q == 32'sd0) high_out <= 24'd43;
                                    else high_out <= (a1_q - 32'sd1);
                                end
                            end
                            low_out <= encode_s24(a0_q);
                        end
                        default: begin
                            high_out <= '0;
                            low_out  <= '0;
                            hint_out <= 1'b0;
                        end
                    endcase
                    st <= S_DONE;
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

    assign state_dbg = st;
endmodule

`default_nettype wire
