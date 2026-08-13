/*
作者：唐金钰
时间：2026/7/29
概述：把32字节的种子给到XOF挤128字节按区间分成三份
原理：keygen第一步是把32字节种子扩展为三元组(ρ, ρ′, K) = H(ξ, 128字节），
三者分工：
ρ公开用于生成矩阵A
ρ′是私有小系数采样的随机源s1和s2的采样源
K是用于签名时算 ρ′=H(K‖rnd‖μ) 的密钥材料

*/

`timescale 1ns/1ps
`default_nettype none

module mldsa_seed_expand (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        seed_we,
    input  wire [4:0]  seed_addr,
    input  wire [7:0]  seed_data,
    input  wire [3:0]  cfg_k,
    input  wire [3:0]  cfg_l,
    input  wire        expanded_we,
    input  wire [1:0]  expanded_sel,
    input  wire [5:0]  expanded_addr,
    input  wire [7:0]  expanded_data,
    input  wire        start,
    output logic       busy,
    output logic       done,
    output logic       error,
    input  wire [4:0]  rho_rd_addr,
    output logic [7:0] rho_rd_data,
    input  wire [5:0]  rho_prime_rd_addr,
    output logic [7:0] rho_prime_rd_data,
    input  wire [4:0]  k_rd_addr,
    output logic [7:0] k_rd_data,
    output logic [7:0] rho_shadow_rd_data,
    output logic [7:0] rho_prime_shadow_rd_data,
    output logic [7:0] k_shadow_rd_data,
    output logic       xof_start_o,
    output logic       xof_stop_o,
    output logic       xof_mode_shake256_o,
    output logic [7:0] xof_abs_byte_data_o,
    output logic       xof_abs_byte_valid_o,
    output logic       xof_abs_byte_last_o,
    input  wire        xof_abs_byte_ready_i,
    input  wire [7:0]  xof_sq_byte_i,
    input  wire        xof_sq_valid_i,
    output logic       xof_sq_byte_ready_o,
    input  wire        xof_state_squeezing_i
);

    typedef enum logic [2:0] {
        S_IDLE,
        S_ABSORB,
        S_WAIT_SQUEEZE,
        S_SQUEEZE,
        S_STOP,
        S_DONE,
        S_ERROR
    } state_e;

    state_e st;

    logic [7:0] seed_mem [0:31];
    logic [5:0] seed_idx;
    logic [7:0] out_idx;
    logic       rho_wr_en;
    logic [4:0] rho_wr_addr;
    logic [7:0] rho_wr_data;
    logic       rho_prime_wr_en;
    logic [5:0] rho_prime_wr_addr;
    logic [7:0] rho_prime_wr_data;
    logic       k_wr_en;
    logic [4:0] k_wr_addr;
    logic [7:0] k_wr_data;
    logic [7:0] rho_shadow [0:31];
    logic [7:0] rho_prime_shadow [0:63];
    logic [7:0] k_shadow [0:31];

    mldsa_byte_store_async #(.DEPTH(32), .ADDR_W(5)) u_rho_store (
        .clk    (clk),
        .wr_en  (rho_wr_en),
        .wr_addr(rho_wr_addr),
        .wr_data(rho_wr_data),
        .rd_addr(rho_rd_addr),
        .rd_data(rho_rd_data)
    );

    mldsa_byte_store_async #(.DEPTH(64), .ADDR_W(6)) u_rho_prime_store (
        .clk    (clk),
        .wr_en  (rho_prime_wr_en),
        .wr_addr(rho_prime_wr_addr),
        .wr_data(rho_prime_wr_data),
        .rd_addr(rho_prime_rd_addr),
        .rd_data(rho_prime_rd_data)
    );

    mldsa_byte_store_async #(.DEPTH(32), .ADDR_W(5)) u_k_store (
        .clk    (clk),
        .wr_en  (k_wr_en),
        .wr_addr(k_wr_addr),
        .wr_data(k_wr_data),
        .rd_addr(k_rd_addr),
        .rd_data(k_rd_data)
    );

    always_comb begin
        rho_shadow_rd_data       = rho_shadow[rho_rd_addr];
        rho_prime_shadow_rd_data = rho_prime_shadow[rho_prime_rd_addr];
        k_shadow_rd_data         = k_shadow[k_rd_addr];

        xof_start_o          = (st == S_IDLE) && start;
        xof_stop_o           = (st == S_STOP);
        xof_mode_shake256_o  = 1'b1;
        if (seed_idx < 6'd32) xof_abs_byte_data_o = seed_mem[seed_idx[4:0]];
        else if (seed_idx == 6'd32) xof_abs_byte_data_o = {4'd0, cfg_k};
        else xof_abs_byte_data_o = {4'd0, cfg_l};
        xof_abs_byte_valid_o = (st == S_ABSORB);
        xof_abs_byte_last_o  = (seed_idx == 6'd33);
        xof_sq_byte_ready_o  = (st == S_SQUEEZE);

        rho_wr_en          = expanded_we && (expanded_sel == 2'd0);
        rho_wr_addr        = expanded_addr[4:0];
        rho_wr_data        = expanded_data;
        rho_prime_wr_en    = expanded_we && (expanded_sel == 2'd1);
        rho_prime_wr_addr  = expanded_addr;
        rho_prime_wr_data  = expanded_data;
        k_wr_en            = expanded_we && (expanded_sel == 2'd2);
        k_wr_addr          = expanded_addr[4:0];
        k_wr_data          = expanded_data;

        if (st == S_SQUEEZE && xof_sq_valid_i) begin
            if (out_idx < 8'd32) begin
                rho_wr_en   = 1'b1;
                rho_wr_addr = out_idx[4:0];
                rho_wr_data = xof_sq_byte_i;
            end else if (out_idx < 8'd96) begin
                rho_prime_wr_en   = 1'b1;
                rho_prime_wr_addr = out_idx[5:0] - 6'd32;
                rho_prime_wr_data = xof_sq_byte_i;
            end else begin
                k_wr_en   = 1'b1;
                k_wr_addr = out_idx[4:0];
                k_wr_data = xof_sq_byte_i;
            end
        end

        busy = (st != S_IDLE) && (st != S_DONE) && (st != S_ERROR);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_IDLE;
            seed_idx <= 6'd0;
            out_idx  <= 8'd0;
            done     <= 1'b0;
            error    <= 1'b0;
        end else begin
            done  <= 1'b0;
            error <= 1'b0;

            if (seed_we && !busy) begin
                seed_mem[seed_addr] <= seed_data;
            end

            if (rho_wr_en) begin
                rho_shadow[rho_wr_addr] <= rho_wr_data;
            end
            if (rho_prime_wr_en) begin
                rho_prime_shadow[rho_prime_wr_addr] <= rho_prime_wr_data;
            end
            if (k_wr_en) begin
                k_shadow[k_wr_addr] <= k_wr_data;
            end

            unique case (st)
                S_IDLE: begin
                    seed_idx <= 6'd0;
                    out_idx  <= 8'd0;
                    if (start) st <= S_ABSORB;
                end
                S_ABSORB: begin
                    if (xof_abs_byte_valid_o && xof_abs_byte_ready_i) begin
`ifdef MLDSA_DEBUG_DISPLAY
                        if (seed_idx >= 6'd30) begin
                            $display("SEED_EXPAND_ABS idx=%0d data=%02x last=%0b cfg_k=%0d cfg_l=%0d",
                                     seed_idx, xof_abs_byte_data_o, xof_abs_byte_last_o, cfg_k, cfg_l);
                        end
`endif
                        if (seed_idx == 6'd33) st <= S_WAIT_SQUEEZE;
                        else seed_idx <= seed_idx + 6'd1;
                    end
                end
                S_WAIT_SQUEEZE: begin
                    if (xof_state_squeezing_i) st <= S_SQUEEZE;
                end
                S_SQUEEZE: begin
                    if (xof_sq_valid_i) begin
`ifdef MLDSA_DEBUG_DISPLAY
                        if (out_idx < 8'd8) begin
                            $display("SEED_EXPAND_SQ idx=%0d data=%02x", out_idx, xof_sq_byte_i);
                        end
`endif
                        if (out_idx == 8'd127) st <= S_STOP;
                        else out_idx <= out_idx + 8'd1;
                    end
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
