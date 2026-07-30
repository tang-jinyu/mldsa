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

    typedef enum logic [1:0] {S_IDLE, S_EXEC, S_DONE} state_e;
    state_e st;

    logic [1:0] op_q;
    logic [MLDSA_COEFF_W-1:0] coeff_q;
    logic [MLDSA_COEFF_W-1:0] aux_q;
    logic [19:0] gamma2_q;

    function automatic integer signed abs_int(input integer signed a);
        begin
            if (a < 0) abs_int = -a;
            else abs_int = a;
        end
    endfunction

    task automatic do_decompose(
        input  logic [MLDSA_COEFF_W-1:0] coeff_value,
        input  logic [19:0]              gamma2_value,
        output integer signed            a0_value,
        output integer signed            a1_value
    );
        integer signed coeff_int;
        integer signed gamma2_int;
        integer signed a1_tmp;
        begin
            coeff_int = coeff_value;
            gamma2_int = gamma2_value;
            a1_tmp = (coeff_int + 127) >>> 7;
            if (gamma2_int == 261888) begin
                a1_tmp = (a1_tmp * 1025 + (1 <<< 21)) >>> 22;
                a1_tmp = a1_tmp & 15;
            end else begin
                a1_tmp = (a1_tmp * 11275 + (1 <<< 23)) >>> 24;
                if (a1_tmp > 43) a1_tmp = 0;
            end
            a0_value = coeff_int - a1_tmp * 2 * gamma2_int;
            if (a0_value > ((MLDSA_Q - 1) / 2)) a0_value = a0_value - MLDSA_Q;
            if (a0_value < -((MLDSA_Q - 1) / 2)) a0_value = a0_value + MLDSA_Q;
            a1_value = a1_tmp;
        end
    endtask

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_IDLE;
            op_q     <= MLDSA_DH_POWER2ROUND;
            coeff_q  <= '0;
            aux_q    <= '0;
            gamma2_q <= 20'd261888;
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
                        st       <= S_EXEC;
                    end
                end
                S_EXEC: begin
                    integer signed coeff_int;
                    integer signed aux_int;
                    integer signed a0_int;
                    integer signed a1_int;
                    integer signed p2r_high;
                    integer signed p2r_low;
                    coeff_int = coeff_q;
                    aux_int = decode_s24(aux_q);
                    a0_int = 0;
                    a1_int = 0;
                    p2r_high = 0;
                    p2r_low = 0;
                    hint_out <= 1'b0;
                    unique case (op_q)
                        MLDSA_DH_POWER2ROUND: begin
                            p2r_high = (coeff_int + ((1 <<< (MLDSA_D - 1)) - 1)) >>> MLDSA_D;
                            p2r_low  = coeff_int - (p2r_high <<< MLDSA_D);
                            high_out <= p2r_high[MLDSA_COEFF_W-1:0];
                            low_out  <= encode_s24(p2r_low);
                        end
                        MLDSA_DH_DECOMPOSE: begin
                            do_decompose(coeff_q, gamma2_q, a0_int, a1_int);
                            high_out <= a1_int[MLDSA_COEFF_W-1:0];
                            low_out  <= encode_s24(a0_int);
                        end
                        MLDSA_DH_MAKE_HINT: begin
                            hint_out <= (aux_int > $signed({1'b0, gamma2_q})) ||
                                        (aux_int < -$signed({1'b0, gamma2_q})) ||
                                        ((aux_int == -$signed({1'b0, gamma2_q})) && (coeff_q != '0));
                            high_out <= coeff_q;
                            low_out  <= aux_q;
                        end
                        MLDSA_DH_USE_HINT: begin
                            do_decompose(coeff_q, gamma2_q, a0_int, a1_int);
                            if (!aux_q[0]) begin
                                high_out <= a1_int[MLDSA_COEFF_W-1:0];
                            end else if (gamma2_q == 20'd261888) begin
                                if (a0_int > 0) high_out <= ((a1_int + 1) & 15);
                                else high_out <= ((a1_int - 1) & 15);
                            end else begin
                                if (a0_int > 0) begin
                                    if (a1_int == 43) high_out <= '0;
                                    else high_out <= (a1_int + 1);
                                end else begin
                                    if (a1_int == 0) high_out <= 24'd43;
                                    else high_out <= (a1_int - 1);
                                end
                            end
                            low_out <= encode_s24(a0_int);
                        end
                        default: begin
                            high_out <= '0;
                            low_out  <= '0;
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

    assign state_dbg = {1'b0, st};
endmodule

`default_nettype wire