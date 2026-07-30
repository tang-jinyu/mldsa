`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

module mldsa_sampler (
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     start,
    input  wire [1:0]               sample_mode,
    input  wire [2:0]               eta,
    input  wire                     gamma1_is_2p19,
    input  wire [5:0]               tau,
    input  wire [7:0]               byte_data,
    input  wire                     byte_valid,
    output logic                    byte_ready,
    output logic [7:0]              coeff_index,
    output logic [MLDSA_COEFF_W-1:0] coeff_data,
    output logic                    coeff_valid,
    input  wire                     coeff_ready,
    output logic                    busy,
    output logic                    done,
    output logic                    error,
    output logic [3:0]              state_dbg
);

    typedef enum logic [4:0] {
        S_IDLE,
        S_UNIFORM_B0,
        S_UNIFORM_B1,
        S_UNIFORM_B2,
        S_ETA_BYTE,
        S_ETA_LOW,
        S_ETA_HIGH,
        S_GAMMA_COLLECT,
        S_GAMMA_EMIT,
        S_CHAL_CLEAR,
        S_CHAL_SIGNS,
        S_CHAL_POS,
        S_CHAL_EMIT,
        S_DONE,
        S_ERROR
    } state_e;

    state_e st;

    logic [1:0] mode_q;
    logic [2:0] eta_q;
    logic       gamma1_2p19_q;
    logic [5:0] tau_q;
    logic [8:0] coeff_count;
    logic       emit_last;
    logic [7:0] b0;
    logic [7:0] b1;
    logic [7:0] eta_byte;
    logic [3:0] eta_nibble;
    logic [7:0] gamma_bytes [0:8];
    logic [3:0] gamma_byte_count;
    logic [2:0] gamma_emit_idx;
    logic [63:0] challenge_signs;
    logic [3:0] signs_count;
    logic [8:0] challenge_i;
    logic [7:0] challenge_emit_idx;
    logic [7:0] challenge_candidate;
    logic [MLDSA_COEFF_W-1:0] challenge_old;
    logic [MLDSA_COEFF_W-1:0] challenge_rd0_data;
    logic [MLDSA_COEFF_W-1:0] challenge_rd1_data;
    logic                     challenge_clr_we;
    logic [7:0]               challenge_clr_addr;
    logic [MLDSA_COEFF_W-1:0] challenge_clr_data;
    logic [7:0]               challenge_rd0_addr;
    logic [7:0]               challenge_rd1_addr;
    logic                     challenge_wr0_en;
    logic [7:0]               challenge_wr0_addr;
    logic [MLDSA_COEFF_W-1:0] challenge_wr0_data;
    logic                     challenge_wr1_en;
    logic [7:0]               challenge_wr1_addr;
    logic [MLDSA_COEFF_W-1:0] challenge_wr1_data;

    function automatic logic [MLDSA_COEFF_W-1:0] signed_small_to_modq(
        input logic [19:0] center,
        input logic [19:0] raw
    );
        begin
            if (raw <= center) signed_small_to_modq = center - raw;
            else signed_small_to_modq = MLDSA_Q_COEFF - (raw - center);
        end
    endfunction

    function automatic logic [MLDSA_COEFF_W-1:0] eta_coeff_to_modq(
        input logic [2:0] eta_value,
        input logic [3:0] raw
    );
        logic [3:0] eta_ext;
        begin
            eta_ext = {1'b0, eta_value};
            if (raw <= eta_ext) eta_coeff_to_modq = eta_ext - raw;
            else eta_coeff_to_modq = MLDSA_Q_COEFF - (raw - eta_ext);
        end
    endfunction

    function automatic logic eta_nibble_accept(
        input logic [2:0] eta_value,
        input logic [3:0] raw
    );
        begin
            if (eta_value == 3'd2) eta_nibble_accept = (raw < 4'd15);
            else eta_nibble_accept = (raw < 4'd9);
        end
    endfunction

    function automatic logic [3:0] eta_nibble_reduce(
        input logic [2:0] eta_value,
        input logic [3:0] raw
    );
        begin
            if (eta_value == 3'd2) begin
                unique case (raw)
                    4'd0, 4'd5, 4'd10: eta_nibble_reduce = 4'd0;
                    4'd1, 4'd6, 4'd11: eta_nibble_reduce = 4'd1;
                    4'd2, 4'd7, 4'd12: eta_nibble_reduce = 4'd2;
                    4'd3, 4'd8, 4'd13: eta_nibble_reduce = 4'd3;
                    default:           eta_nibble_reduce = 4'd4;
                endcase
            end else begin
                eta_nibble_reduce = raw;
            end
        end
    endfunction

    function automatic logic [19:0] gamma_raw_value(
        input logic gamma1_2p19,
        input logic [2:0] idx
    );
        logic [19:0] value;
        begin
            value = 20'd0;
            if (gamma1_2p19) begin
                unique case (idx)
                    3'd0: value = {gamma_bytes[2][3:0], gamma_bytes[1], gamma_bytes[0]};
                    3'd1: value = {gamma_bytes[4], gamma_bytes[3], gamma_bytes[2][7:4]};
                    default: value = 20'd0;
                endcase
            end else begin
                unique case (idx)
                    3'd0: value = {2'd0, gamma_bytes[2][1:0], gamma_bytes[1], gamma_bytes[0]};
                    3'd1: value = {2'd0, gamma_bytes[4][3:0], gamma_bytes[3], gamma_bytes[2][7:2]};
                    3'd2: value = {2'd0, gamma_bytes[6][5:0], gamma_bytes[5], gamma_bytes[4][7:4]};
                    3'd3: value = {2'd0, gamma_bytes[8], gamma_bytes[7], gamma_bytes[6][7:6]};
                    default: value = 20'd0;
                endcase
            end
            gamma_raw_value = value;
        end
    endfunction

    function automatic logic [19:0] gamma_center(input logic gamma1_2p19);
        begin
            gamma_center = gamma1_2p19 ? 20'd524288 : 20'd131072;
        end
    endfunction

    task automatic set_emit(
        input logic [7:0] index_value,
        input logic [MLDSA_COEFF_W-1:0] data_value,
        input logic last_value
    );
        begin
            coeff_index <= index_value;
            coeff_data  <= data_value;
            coeff_valid <= 1'b1;
            emit_last   <= last_value;
        end
    endtask

    always_comb begin
        byte_ready = 1'b0;
        unique case (st)
            S_UNIFORM_B0,
            S_UNIFORM_B1,
            S_UNIFORM_B2,
            S_ETA_BYTE,
            S_GAMMA_COLLECT,
            S_CHAL_SIGNS,
            S_CHAL_POS: byte_ready = !coeff_valid;
            default: byte_ready = 1'b0;
        endcase
        busy = (st != S_IDLE) && (st != S_DONE) && (st != S_ERROR);
        state_dbg = st;
    end

    // Challenge storage currently needs one combinational read in the
    // shuffle phase plus two writes in the same cycle. This is compatible
    // with the original RTL behavior, but not with a direct single-port
    // SRAM swap.
    always_comb begin
        challenge_clr_we   = 1'b0;
        challenge_clr_addr = challenge_i[7:0];
        challenge_clr_data = 24'd0;
        challenge_rd0_addr = (st == S_CHAL_EMIT) ? challenge_emit_idx : byte_data;
        challenge_rd1_addr = byte_data;
        challenge_wr0_en   = 1'b0;
        challenge_wr0_addr = challenge_i[7:0];
        challenge_wr0_data = challenge_rd1_data;
        challenge_wr1_en   = 1'b0;
        challenge_wr1_addr = byte_data;
        challenge_wr1_data = challenge_signs[0] ? (MLDSA_Q_COEFF - 24'd1) : 24'd1;

        if (st == S_CHAL_CLEAR) begin
            challenge_clr_we = 1'b1;
        end

        if (st == S_CHAL_POS && byte_valid && ({1'b0, byte_data} <= challenge_i)) begin
            challenge_wr0_en = 1'b1;
            challenge_wr1_en = 1'b1;
        end
    end

    mldsa_sampler_challenge_mem_compat u_challenge_mem (
        .clk    (clk),
        .clr_we (challenge_clr_we),
        .clr_addr(challenge_clr_addr),
        .clr_data(challenge_clr_data),
        .rd0_addr(challenge_rd0_addr),
        .rd0_data(challenge_rd0_data),
        .rd1_addr(challenge_rd1_addr),
        .rd1_data(challenge_rd1_data),
        .wr0_en (challenge_wr0_en),
        .wr0_addr(challenge_wr0_addr),
        .wr0_data(challenge_wr0_data),
        .wr1_en (challenge_wr1_en),
        .wr1_addr(challenge_wr1_addr),
        .wr1_data(challenge_wr1_data)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st                 <= S_IDLE;
            mode_q             <= MLDSA_SAMPLE_UNIFORM;
            eta_q              <= 3'd2;
            gamma1_2p19_q      <= 1'b1;
            tau_q              <= 6'd0;
            coeff_count        <= 9'd0;
            emit_last          <= 1'b0;
            b0                 <= 8'd0;
            b1                 <= 8'd0;
            eta_byte           <= 8'd0;
            eta_nibble         <= 4'd0;
            gamma_byte_count   <= 4'd0;
            gamma_emit_idx     <= 3'd0;
            challenge_signs    <= 64'd0;
            signs_count        <= 4'd0;
            challenge_i        <= 9'd0;
            challenge_emit_idx <= 8'd0;
            challenge_candidate <= 8'd0;
            challenge_old      <= 24'd0;
            coeff_index        <= 8'd0;
            coeff_data         <= 24'd0;
            coeff_valid        <= 1'b0;
            done               <= 1'b0;
            error              <= 1'b0;
        end else begin
            done <= 1'b0;

            if (coeff_valid && coeff_ready) begin
                coeff_valid <= 1'b0;
                if (emit_last) begin
                    emit_last <= 1'b0;
                    st        <= S_DONE;
                end
            end

            unique case (st)
                S_IDLE: begin
                    error <= 1'b0;
                    if (start) begin
                        mode_q        <= sample_mode;
                        eta_q         <= eta;
                        gamma1_2p19_q <= gamma1_is_2p19;
                        tau_q         <= tau;
                        coeff_count   <= 9'd0;
                        emit_last     <= 1'b0;
                        b0            <= 8'd0;
                        b1            <= 8'd0;
                        eta_byte      <= 8'd0;
                        eta_nibble    <= 4'd0;
                        gamma_byte_count <= 4'd0;
                        gamma_emit_idx   <= 3'd0;
                        coeff_index   <= 8'd0;
                        coeff_data    <= 24'd0;
                        coeff_valid   <= 1'b0;
                        if (sample_mode == MLDSA_SAMPLE_UNIFORM) begin
                            st <= S_UNIFORM_B0;
                        end else if (sample_mode == MLDSA_SAMPLE_ETA) begin
                            st <= S_ETA_BYTE;
                        end else if (sample_mode == MLDSA_SAMPLE_GAMMA1) begin
                            gamma_byte_count <= 4'd0;
                            gamma_emit_idx   <= 3'd0;
                            st               <= S_GAMMA_COLLECT;
                        end else if (sample_mode == MLDSA_SAMPLE_CHALLENGE) begin
                            challenge_i <= 9'd0;
                            st          <= S_CHAL_CLEAR;
                        end else begin
                            st <= S_ERROR;
                        end
                    end
                end

                S_UNIFORM_B0: begin
                    if (!coeff_valid && byte_valid) begin
                        b0 <= byte_data;
                        st <= S_UNIFORM_B1;
                    end
                end
                S_UNIFORM_B1: begin
                    if (!coeff_valid && byte_valid) begin
                        b1 <= byte_data;
                        st <= S_UNIFORM_B2;
                    end
                end
                S_UNIFORM_B2: begin
                    if (!coeff_valid && byte_valid) begin
                        logic [22:0] sample_value;
                        sample_value = {byte_data[6:0], b1, b0};
                        if ({1'b0, sample_value} < MLDSA_Q_COEFF) begin
                            set_emit(coeff_count[7:0], {1'b0, sample_value}, coeff_count == 9'd255);
                            coeff_count <= coeff_count + 9'd1;
                        end
                        st <= S_UNIFORM_B0;
                    end
                end

                S_ETA_BYTE: begin
                    if (!coeff_valid && byte_valid) begin
                        eta_byte   <= byte_data;
                        eta_nibble <= byte_data[3:0];
                        st         <= S_ETA_LOW;
                    end
                end
                S_ETA_LOW: begin
                    if (!coeff_valid) begin
                        if (eta_nibble_accept(eta_q, eta_nibble)) begin
                            set_emit(coeff_count[7:0], eta_coeff_to_modq(eta_q, eta_nibble_reduce(eta_q, eta_nibble)), coeff_count == 9'd255);
                            coeff_count <= coeff_count + 9'd1;
                        end
                        eta_nibble <= eta_byte[7:4];
                        st <= S_ETA_HIGH;
                    end
                end
                S_ETA_HIGH: begin
                    if (!coeff_valid) begin
                        if (eta_nibble_accept(eta_q, eta_nibble)) begin
                            set_emit(coeff_count[7:0], eta_coeff_to_modq(eta_q, eta_nibble_reduce(eta_q, eta_nibble)), coeff_count == 9'd255);
                            coeff_count <= coeff_count + 9'd1;
                        end
                        st <= S_ETA_BYTE;
                    end
                end

                S_GAMMA_COLLECT: begin
                    if (!coeff_valid && byte_valid) begin
                        gamma_bytes[gamma_byte_count] <= byte_data;
                        if ((gamma1_2p19_q && gamma_byte_count == 4'd4) || (!gamma1_2p19_q && gamma_byte_count == 4'd8)) begin
                            gamma_byte_count <= 4'd0;
                            gamma_emit_idx   <= 3'd0;
                            st               <= S_GAMMA_EMIT;
                        end else begin
                            gamma_byte_count <= gamma_byte_count + 4'd1;
                        end
                    end
                end
                S_GAMMA_EMIT: begin
                    if (!coeff_valid) begin
                        set_emit(coeff_count[7:0], signed_small_to_modq(gamma_center(gamma1_2p19_q), gamma_raw_value(gamma1_2p19_q, gamma_emit_idx)), coeff_count == 9'd255);
                        coeff_count <= coeff_count + 9'd1;
                        if ((gamma1_2p19_q && gamma_emit_idx == 3'd1) || (!gamma1_2p19_q && gamma_emit_idx == 3'd3)) begin
                            st <= S_GAMMA_COLLECT;
                        end else begin
                            gamma_emit_idx <= gamma_emit_idx + 3'd1;
                        end
                    end
                end

                S_CHAL_CLEAR: begin
                    if (challenge_i == 9'd255) begin
                        challenge_i     <= 9'd256 - {3'd0, tau_q};
                        signs_count     <= 4'd0;
                        challenge_signs <= 64'd0;
                        st              <= S_CHAL_SIGNS;
                    end else begin
                        challenge_i <= challenge_i + 9'd1;
                    end
                end
                S_CHAL_SIGNS: begin
                    if (byte_valid) begin
                        challenge_signs <= challenge_signs | ({56'd0, byte_data} << (signs_count * 4'd8));
                        if (signs_count == 4'd7) begin
                            st <= S_CHAL_POS;
                        end else begin
                            signs_count <= signs_count + 4'd1;
                        end
                    end
                end
                S_CHAL_POS: begin
                    if (byte_valid) begin
                        challenge_candidate <= byte_data;
                        if ({1'b0, byte_data} <= challenge_i) begin
                            challenge_old <= challenge_rd1_data;
                            challenge_signs <= {1'b0, challenge_signs[63:1]};
                            if (challenge_i == 9'd255) begin
                                challenge_emit_idx <= 8'd0;
                                st <= S_CHAL_EMIT;
                            end else begin
                                challenge_i <= challenge_i + 9'd1;
                            end
                        end
                    end
                end
                S_CHAL_EMIT: begin
                    if (!coeff_valid) begin
                        set_emit(challenge_emit_idx, challenge_rd0_data, challenge_emit_idx == 8'd255);
                        challenge_emit_idx <= challenge_emit_idx + 8'd1;
                    end
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
