`timescale 1ns/1ps
`default_nettype none

module mldsa65_shake_stream (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire        mode_shake256,
    input  wire [15:0] in_len,
    input  wire [15:0] out_len,
    input  wire        in_valid,
    output logic       in_ready,
    input  wire [7:0]  in_data,
    output logic       out_valid,
    input  wire        out_ready,
    output logic [7:0] out_data,
    output logic       busy,
    output logic       done,
    output logic       error
);

`ifdef MLDSA_SYNTH_SHAKE_AREA_MODEL
    logic [15:0] in_count;
    logic [15:0] out_count;
    logic active;

    assign in_ready = active && in_count < in_len;
    assign out_valid = active && in_count >= in_len && out_count < out_len;
    assign out_data = in_data ^ out_count[7:0] ^ {7'd0, mode_shake256};
    assign busy = active;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_count <= 16'd0;
            out_count <= 16'd0;
            active <= 1'b0;
            done <= 1'b0;
            error <= 1'b0;
        end else begin
            done <= 1'b0;
            error <= 1'b0;
            if (start && !active) begin
                in_count <= 16'd0;
                out_count <= 16'd0;
                active <= 1'b1;
            end else if (active) begin
                if (in_valid && in_ready) begin
                    in_count <= in_count + 16'd1;
                end else if (out_ready && out_valid) begin
                    out_count <= out_count + 16'd1;
                    if (out_count + 16'd1 == out_len) begin
                        active <= 1'b0;
                        done <= 1'b1;
                    end
                end else if (in_len == 16'd0 && out_len == 16'd0) begin
                    active <= 1'b0;
                    done <= 1'b1;
                end
            end
        end
    end
`else

    typedef enum logic [2:0] {
        S_IDLE,
        S_ABSORB,
        S_PERMUTE_ABSORB,
        S_PAD_FINAL,
        S_PERMUTE_SQUEEZE,
        S_SQUEEZE,
        S_DONE,
        S_ERROR
    } state_e;

    state_e st;

    logic [1599:0] state_q;
    logic [1599:0] round_mid_q;
    logic [15:0]   in_count;
    logic [15:0]   out_count;
    logic [7:0]    absorb_pos;
    logic [7:0]    squeeze_pos;
    logic [7:0]    rate_q;
    logic [4:0]    round_idx;
    logic [1:0]    perm_phase;
    logic          pad_after_permute;
    (* keep *) logic [24:0] theta_lane_en_q;
    (* keep *) logic [24:0] rho_lane_en_q;
    (* keep *) logic [24:0] chi_lane_en_q;
    logic [1599:0] theta_next;
    logic [1599:0] rho_next;
    logic [1599:0] chi_next;

    function automatic [63:0] rotl64(input [63:0] value, input int amount);
        begin
            if (amount == 0) rotl64 = value;
            else rotl64 = (value << amount) | (value >> (64 - amount));
        end
    endfunction

    function automatic int rotc(input int idx);
        begin
            unique case (idx)
                0: rotc = 1;
                1: rotc = 3;
                2: rotc = 6;
                3: rotc = 10;
                4: rotc = 15;
                5: rotc = 21;
                6: rotc = 28;
                7: rotc = 36;
                8: rotc = 45;
                9: rotc = 55;
                10: rotc = 2;
                11: rotc = 14;
                12: rotc = 27;
                13: rotc = 41;
                14: rotc = 56;
                15: rotc = 8;
                16: rotc = 25;
                17: rotc = 43;
                18: rotc = 62;
                19: rotc = 18;
                20: rotc = 39;
                21: rotc = 61;
                22: rotc = 20;
                23: rotc = 44;
                default: rotc = 0;
            endcase
        end
    endfunction

    function automatic int piln(input int idx);
        begin
            unique case (idx)
                0: piln = 10;
                1: piln = 7;
                2: piln = 11;
                3: piln = 17;
                4: piln = 18;
                5: piln = 3;
                6: piln = 5;
                7: piln = 16;
                8: piln = 8;
                9: piln = 21;
                10: piln = 24;
                11: piln = 4;
                12: piln = 15;
                13: piln = 23;
                14: piln = 19;
                15: piln = 13;
                16: piln = 12;
                17: piln = 2;
                18: piln = 20;
                19: piln = 14;
                20: piln = 22;
                21: piln = 9;
                22: piln = 6;
                23: piln = 1;
                default: piln = 0;
            endcase
        end
    endfunction

    function automatic [63:0] round_constant(input logic [4:0] idx);
        begin
            unique case (idx)
                5'd0:  round_constant = 64'h0000000000000001;
                5'd1:  round_constant = 64'h0000000000008082;
                5'd2:  round_constant = 64'h800000000000808a;
                5'd3:  round_constant = 64'h8000000080008000;
                5'd4:  round_constant = 64'h000000000000808b;
                5'd5:  round_constant = 64'h0000000080000001;
                5'd6:  round_constant = 64'h8000000080008081;
                5'd7:  round_constant = 64'h8000000000008009;
                5'd8:  round_constant = 64'h000000000000008a;
                5'd9:  round_constant = 64'h0000000000000088;
                5'd10: round_constant = 64'h0000000080008009;
                5'd11: round_constant = 64'h000000008000000a;
                5'd12: round_constant = 64'h000000008000808b;
                5'd13: round_constant = 64'h800000000000008b;
                5'd14: round_constant = 64'h8000000000008089;
                5'd15: round_constant = 64'h8000000000008003;
                5'd16: round_constant = 64'h8000000000008002;
                5'd17: round_constant = 64'h8000000000000080;
                5'd18: round_constant = 64'h000000000000800a;
                5'd19: round_constant = 64'h800000008000000a;
                5'd20: round_constant = 64'h8000000080008081;
                5'd21: round_constant = 64'h8000000000008080;
                5'd22: round_constant = 64'h0000000080000001;
                5'd23: round_constant = 64'h8000000080008008;
                default: round_constant = 64'h0;
            endcase
        end
    endfunction

    function automatic [1599:0] xor_state_byte(
        input [1599:0] value,
        input int      byte_pos,
        input [7:0]    byte_value
    );
        logic [1599:0] tmp;
        begin
            tmp = value;
            tmp[byte_pos * 8 +: 8] = tmp[byte_pos * 8 +: 8] ^ byte_value;
            xor_state_byte = tmp;
        end
    endfunction

    function automatic [7:0] state_byte(input [1599:0] value, input int byte_pos);
        begin
            state_byte = value[byte_pos * 8 +: 8];
        end
    endfunction

    function automatic [1599:0] theta_step(input [1599:0] value);
        logic [63:0] a [0:24];
        logic [63:0] bc [0:4];
        logic [63:0] d [0:4];
        logic [1599:0] result;
        int i;
        int j;
        begin
            for (i = 0; i < 25; i++) begin
                a[i] = value[i * 64 +: 64];
            end

            for (i = 0; i < 5; i++) begin
                bc[i] = a[i] ^ a[i + 5] ^ a[i + 10] ^ a[i + 15] ^ a[i + 20];
            end
            for (i = 0; i < 5; i++) begin
                d[i] = bc[(i + 4) % 5] ^ rotl64(bc[(i + 1) % 5], 1);
            end

            result = '0;
            for (i = 0; i < 5; i++) begin
                for (j = 0; j < 25; j += 5) begin
                    result[(j + i) * 64 +: 64] = a[j + i] ^ d[i];
                end
            end
            theta_step = result;
        end
    endfunction

    function automatic [1599:0] rho_pi_step(input [1599:0] value);
        logic [63:0] a [0:24];
        logic [63:0] t;
        logic [63:0] tmp_lane;
        logic [1599:0] result;
        int i;
        int j;
        begin
            for (i = 0; i < 25; i++) begin
                a[i] = value[i * 64 +: 64];
            end

            t = a[1];
            for (i = 0; i < 24; i++) begin
                j = piln(i);
                tmp_lane = a[j];
                a[j] = rotl64(t, rotc(i));
                t = tmp_lane;
            end

            result = '0;
            for (i = 0; i < 25; i++) begin
                result[i * 64 +: 64] = a[i];
            end
            rho_pi_step = result;
        end
    endfunction

    function automatic [1599:0] chi_iota_step(
        input [1599:0] value,
        input logic [4:0] round
    );
        logic [63:0] a [0:24];
        logic [63:0] bc [0:4];
        logic [1599:0] result;
        int i;
        int j;
        begin
            for (i = 0; i < 25; i++) begin
                a[i] = value[i * 64 +: 64];
            end

            result = '0;
            for (j = 0; j < 25; j += 5) begin
                for (i = 0; i < 5; i++) begin
                    bc[i] = a[j + i];
                end
                for (i = 0; i < 5; i++) begin
                    result[(j + i) * 64 +: 64] = bc[i] ^ ((~bc[(i + 1) % 5]) & bc[(i + 2) % 5]);
                end
            end

            result[0 +: 64] = result[0 +: 64] ^ round_constant(round);
            chi_iota_step = result;
        end
    endfunction

    always_comb begin
        theta_next = theta_step(state_q);
        rho_next   = rho_pi_step(round_mid_q);
        chi_next   = chi_iota_step(round_mid_q, round_idx);
        in_ready  = (st == S_ABSORB);
        out_valid = (st == S_SQUEEZE) && (out_count < out_len);
        out_data  = state_byte(state_q, int'(squeeze_pos));
        busy      = (st != S_IDLE) && (st != S_DONE) && (st != S_ERROR);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st          <= S_IDLE;
            state_q     <= '0;
            round_mid_q <= '0;
            in_count    <= 16'd0;
            out_count   <= 16'd0;
            absorb_pos  <= 8'd0;
            squeeze_pos <= 8'd0;
            rate_q      <= 8'd0;
            round_idx   <= 5'd0;
            perm_phase  <= 2'd0;
            pad_after_permute <= 1'b0;
            theta_lane_en_q <= '0;
            rho_lane_en_q <= '0;
            chi_lane_en_q <= '0;
            done        <= 1'b0;
            error       <= 1'b0;
        end else begin
            for (int lane = 0; lane < 25; lane++) begin
                if (theta_lane_en_q[lane]) begin
                    round_mid_q[lane * 64 +: 64] <= theta_next[lane * 64 +: 64];
                end
                if (rho_lane_en_q[lane]) begin
                    round_mid_q[lane * 64 +: 64] <= rho_next[lane * 64 +: 64];
                end
                if (chi_lane_en_q[lane]) begin
                    state_q[lane * 64 +: 64] <= chi_next[lane * 64 +: 64];
                end
            end
            theta_lane_en_q <= '0;
            rho_lane_en_q <= '0;
            chi_lane_en_q <= '0;
            done <= 1'b0;

            unique case (st)
                S_IDLE: begin
                    error <= 1'b0;
                    if (start) begin
                        state_q     <= '0;
                        round_mid_q <= '0;
                        in_count    <= 16'd0;
                        out_count   <= 16'd0;
                        absorb_pos  <= 8'd0;
                        squeeze_pos <= 8'd0;
                        rate_q      <= mode_shake256 ? 8'd136 : 8'd168;
                        round_idx   <= 5'd0;
                        perm_phase  <= 2'd0;
                        pad_after_permute <= 1'b0;
                        if (out_len == 16'd0) begin
                            st <= S_DONE;
                        end else if (in_len == 16'd0) begin
                            state_q <= xor_state_byte(
                                xor_state_byte('0, 0, 8'h1f),
                                mode_shake256 ? 135 : 167,
                                8'h80
                            );
                            perm_phase <= 2'd0;
                            st <= S_PERMUTE_SQUEEZE;
                        end else begin
                            st <= S_ABSORB;
                        end
                    end
                end

                S_ABSORB: begin
                    if (in_valid) begin
                        state_q <= xor_state_byte(state_q, int'(absorb_pos), in_data);
                        in_count <= in_count + 16'd1;
                        if (in_count + 16'd1 == in_len) begin
                            if (absorb_pos == rate_q - 1) begin
                                state_q <= xor_state_byte(state_q, int'(absorb_pos), in_data);
                                absorb_pos <= 8'd0;
                                pad_after_permute <= 1'b1;
                                round_idx <= 5'd0;
                                perm_phase <= 2'd0;
                                st <= S_PERMUTE_ABSORB;
                            end else begin
                                state_q <= xor_state_byte(
                                    xor_state_byte(state_q, int'(absorb_pos), in_data),
                                    int'(absorb_pos) + 1,
                                    8'h1f
                                );
                                state_q <= xor_state_byte(
                                    xor_state_byte(
                                        xor_state_byte(state_q, int'(absorb_pos), in_data),
                                        int'(absorb_pos) + 1,
                                        8'h1f
                                    ),
                                    int'(rate_q) - 1,
                                    8'h80
                                );
                                round_idx <= 5'd0;
                                perm_phase <= 2'd0;
                                st <= S_PERMUTE_SQUEEZE;
                            end
                        end else if (absorb_pos == rate_q - 1) begin
                            absorb_pos <= 8'd0;
                            round_idx <= 5'd0;
                            perm_phase <= 2'd0;
                            st <= S_PERMUTE_ABSORB;
                        end else begin
                            absorb_pos <= absorb_pos + 8'd1;
                        end
                    end
                end

                S_PERMUTE_ABSORB: begin
                    unique case (perm_phase)
                        2'd0: begin
                            theta_lane_en_q <= '1;
                            perm_phase <= 2'd1;
                        end
                        2'd1: begin
                            rho_lane_en_q <= '1;
                            perm_phase <= 2'd2;
                        end
                        2'd2: begin
                            chi_lane_en_q <= '1;
                            perm_phase <= 2'd3;
                        end
                        default: begin
                            perm_phase <= 2'd0;
                            if (round_idx == 5'd23) begin
                                round_idx <= 5'd0;
                                if (pad_after_permute) begin
                                    pad_after_permute <= 1'b0;
                                    st <= S_PAD_FINAL;
                                end else begin
                                    st <= S_ABSORB;
                                end
                            end else begin
                                round_idx <= round_idx + 5'd1;
                            end
                        end
                    endcase
                end

                S_PAD_FINAL: begin
                    state_q <= xor_state_byte(
                        xor_state_byte(state_q, 0, 8'h1f),
                        int'(rate_q) - 1,
                        8'h80
                    );
                    round_idx <= 5'd0;
                    perm_phase <= 2'd0;
                    st <= S_PERMUTE_SQUEEZE;
                end

                S_PERMUTE_SQUEEZE: begin
                    unique case (perm_phase)
                        2'd0: begin
                            theta_lane_en_q <= '1;
                            perm_phase <= 2'd1;
                        end
                        2'd1: begin
                            rho_lane_en_q <= '1;
                            perm_phase <= 2'd2;
                        end
                        2'd2: begin
                            chi_lane_en_q <= '1;
                            perm_phase <= 2'd3;
                        end
                        default: begin
                            perm_phase <= 2'd0;
                            if (round_idx == 5'd23) begin
                                round_idx <= 5'd0;
                                squeeze_pos <= 8'd0;
                                st <= S_SQUEEZE;
                            end else begin
                                round_idx <= round_idx + 5'd1;
                            end
                        end
                    endcase
                end

                S_SQUEEZE: begin
                    if (out_ready && out_valid) begin
                        out_count <= out_count + 16'd1;
                        if (out_count + 16'd1 == out_len) begin
                            st <= S_DONE;
                        end else if (squeeze_pos == rate_q - 1) begin
                            squeeze_pos <= 8'd0;
                            round_idx <= 5'd0;
                            perm_phase <= 2'd0;
                            st <= S_PERMUTE_SQUEEZE;
                        end else begin
                            squeeze_pos <= squeeze_pos + 8'd1;
                        end
                    end
                end

                S_DONE: begin
                    done <= 1'b1;
                    st <= S_IDLE;
                end

                S_ERROR: begin
                    error <= 1'b1;
                    done <= 1'b1;
                    st <= S_IDLE;
                end

                default: begin
                    st <= S_ERROR;
                end
            endcase
        end
    end
`endif
endmodule

`default_nettype wire
