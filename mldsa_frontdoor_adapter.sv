`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;
import mldsa65_mem_pkg::*;
import mldsa_fused_bank_map_pkg::*;

module mldsa_frontdoor_adapter #(
    parameter int BANKS = MLDSA_FUSED_BANKS_65
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         byte_we,
    input  wire                         byte_rd_en,
    input  wire [2:0]                   byte_region,
    input  wire [12:0]                  byte_addr,
    input  wire [7:0]                   byte_wdata,
    output logic [7:0]                  byte_rdata,
    input  wire                         len_we,
    input  wire [2:0]                   len_region,
    input  wire [15:0]                  len_wdata,
    input  wire                         start,
    input  wire [1:0]                   level_sel,
    input  wire [1:0]                   op_mode,
    output logic                        busy,
    output logic                        done,
    output logic                        pass,
    output logic                        error,
    output logic [7:0]                  err_code,
    output logic                        core_load_poly_we,
    output logic [$clog2(BANKS)-1:0]    core_load_poly_bank,
    output logic [7:0]                  core_load_poly_addr,
    output logic [23:0]                 core_load_poly_data,
    output logic                        core_host_rd_en,
    output logic [$clog2(BANKS)-1:0]    core_host_rd_bank,
    output logic [7:0]                  core_host_rd_addr,
    input  wire [23:0]                  core_host_rd_data,
    output logic                        core_load_seed_we,
    output logic [4:0]                  core_load_seed_addr,
    output logic [7:0]                  core_load_seed_data,
    output logic                        core_standard_mode,
    output logic                        core_load_ctx_we,
    output logic [1:0]                  core_load_ctx_sel,
    output logic [12:0]                 core_load_ctx_addr,
    output logic [7:0]                  core_load_ctx_data,
    output logic [12:0]                 core_message_len,
    output logic                        core_start,
    output logic [1:0]                  core_level_sel,
    output logic [1:0]                  core_op_mode,
    input  wire                         core_busy,
    input  wire                         core_done,
    input  wire                         core_pass,
    input  wire                         core_error,
    input  wire [7:0]                   core_err_code
);
    logic [15:0] pk_len_q;
    logic [15:0] sk_len_q;
    logic [15:0] msg_len_q;
    logic [15:0] sig_len_q;
    logic [15:0] ctx_len_q;
    logic [15:0] seed_len_q;
    logic [15:0] aux_len_q;

    logic [7:0] pk_rd_data;
    logic [7:0] sk_rd_data;
    logic [7:0] msg_rd_data;
    logic [7:0] sig_rd_data;
    logic [7:0] ctx_rd_data;
    logic [7:0] seed_rd_data;
    logic [7:0] aux_rd_data;

    mldsa_byte_store_async #(.DEPTH(MLDSA_MAX_PK_BYTES), .ADDR_W(11)) u_pk_store (
        .clk    (clk),
        .wr_en  (byte_we && !core_busy && byte_region == REGION_PK && byte_addr < MLDSA_MAX_PK_BYTES),
        .wr_addr(byte_addr[10:0]),
        .wr_data(byte_wdata),
        .rd_addr(byte_addr[10:0]),
        .rd_data(pk_rd_data)
    );

    mldsa_byte_store_async #(.DEPTH(MLDSA_MAX_SK_BYTES), .ADDR_W(12)) u_sk_store (
        .clk    (clk),
        .wr_en  (byte_we && !core_busy && byte_region == REGION_SK && byte_addr < MLDSA_MAX_SK_BYTES),
        .wr_addr(byte_addr[11:0]),
        .wr_data(byte_wdata),
        .rd_addr(byte_addr[11:0]),
        .rd_data(sk_rd_data)
    );

    mldsa_byte_store_async #(.DEPTH(MLDSA65_MSG_MAX_BYTES), .ADDR_W(13)) u_msg_store (
        .clk    (clk),
        .wr_en  (byte_we && !core_busy && byte_region == REGION_MSG && byte_addr < MLDSA65_MSG_MAX_BYTES),
        .wr_addr(byte_addr[12:0]),
        .wr_data(byte_wdata),
        .rd_addr(byte_addr[12:0]),
        .rd_data(msg_rd_data)
    );

    mldsa_byte_store_async #(.DEPTH(MLDSA_MAX_SIG_BYTES), .ADDR_W(13)) u_sig_store (
        .clk    (clk),
        .wr_en  (byte_we && !core_busy && byte_region == REGION_SIG && byte_addr < MLDSA_MAX_SIG_BYTES),
        .wr_addr(byte_addr[12:0]),
        .wr_data(byte_wdata),
        .rd_addr(byte_addr[12:0]),
        .rd_data(sig_rd_data)
    );

    mldsa_byte_store_async #(.DEPTH(MLDSA65_CTX_MAX_BYTES), .ADDR_W(8)) u_ctx_store (
        .clk    (clk),
        .wr_en  (byte_we && !core_busy && byte_region == REGION_CTX && byte_addr < MLDSA65_CTX_MAX_BYTES),
        .wr_addr(byte_addr[7:0]),
        .wr_data(byte_wdata),
        .rd_addr(byte_addr[7:0]),
        .rd_data(ctx_rd_data)
    );

    mldsa_byte_store_async #(.DEPTH(MLDSA65_SEED_BYTES), .ADDR_W(5)) u_seed_store (
        .clk    (clk),
        .wr_en  (byte_we && !core_busy && byte_region == REGION_SEED && byte_addr < MLDSA65_SEED_BYTES),
        .wr_addr(byte_addr[4:0]),
        .wr_data(byte_wdata),
        .rd_addr(byte_addr[4:0]),
        .rd_data(seed_rd_data)
    );

    mldsa_byte_store_async #(.DEPTH(MLDSA65_AUX_BYTES), .ADDR_W(6)) u_aux_store (
        .clk    (clk),
        .wr_en  (byte_we && !core_busy && byte_region == REGION_AUX && byte_addr < MLDSA65_AUX_BYTES),
        .wr_addr(byte_addr[5:0]),
        .wr_data(byte_wdata),
        .rd_addr(byte_addr[5:0]),
        .rd_data(aux_rd_data)
    );

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            pk_len_q <= 16'd0;
            sk_len_q <= 16'd0;
            msg_len_q <= 16'd0;
            sig_len_q <= 16'd0;
            ctx_len_q <= 16'd0;
            seed_len_q <= 16'd0;
            aux_len_q <= 16'd0;
            byte_rdata <= 8'd0;
            busy <= 1'b0;
            done <= 1'b0;
            pass <= 1'b0;
            error <= 1'b0;
            err_code <= 8'd0;
            core_load_poly_we <= 1'b0;
            core_load_poly_bank <= '0;
            core_load_poly_addr <= '0;
            core_load_poly_data <= '0;
            core_host_rd_en <= 1'b0;
            core_host_rd_bank <= '0;
            core_host_rd_addr <= '0;
            core_load_seed_we <= 1'b0;
            core_load_seed_addr <= '0;
            core_load_seed_data <= '0;
            core_standard_mode <= 1'b1;
            core_load_ctx_we <= 1'b0;
            core_load_ctx_sel <= 2'd0;
            core_load_ctx_addr <= '0;
            core_load_ctx_data <= '0;
            core_message_len <= 13'd0;
            core_start <= 1'b0;
            core_level_sel <= MLDSA_LEVEL_65;
            core_op_mode <= MLDSA_OP_SIGN;
            // Do not bulk-clear the byte-store arrays on reset.
            // DC expands these loops during elaboration and the fused frontdoor
            // becomes unnecessarily hard to compile. The ACVP/KAT benches write
            // the active byte ranges explicitly before start.
        end else begin
            done <= 1'b0;
            core_start <= 1'b0;
            core_load_seed_we <= 1'b0;
            core_load_ctx_we <= 1'b0;
            core_load_poly_we <= 1'b0;
            core_host_rd_en <= 1'b0;
            busy <= core_busy;

            if (len_we && !core_busy) begin
                unique case (len_region)
                    REGION_PK:   pk_len_q <= len_wdata;
                    REGION_SK:   sk_len_q <= len_wdata;
                    REGION_MSG:  msg_len_q <= len_wdata;
                    REGION_SIG:  sig_len_q <= len_wdata;
                    REGION_CTX:  ctx_len_q <= len_wdata;
                    REGION_SEED: seed_len_q <= len_wdata;
                    REGION_AUX:  aux_len_q <= len_wdata;
                    default: ;
                endcase
            end

            if (byte_we && !core_busy) begin
                unique case (byte_region)
                    REGION_CTX:  if (byte_addr < MLDSA65_CTX_MAX_BYTES) begin
                                    core_load_ctx_we <= 1'b1;
                                    core_load_ctx_sel <= (byte_addr < 13'd64) ? 2'd0 : 2'd1;
                                    core_load_ctx_addr <= byte_addr;
                                    core_load_ctx_data <= byte_wdata;
                                end
                    REGION_SEED: if (byte_addr < MLDSA65_SEED_BYTES) begin
                                    core_load_seed_we <= 1'b1;
                                    core_load_seed_addr <= byte_addr[4:0];
                                    core_load_seed_data <= byte_wdata;
                                 end
                    default: ;
                endcase
            end

            if (byte_rd_en && !core_busy) begin
                unique case (byte_region)
                    REGION_PK:   byte_rdata <= (byte_addr < MLDSA_MAX_PK_BYTES) ? pk_rd_data : 8'd0;
                    REGION_SK:   byte_rdata <= (byte_addr < MLDSA_MAX_SK_BYTES) ? sk_rd_data : 8'd0;
                    REGION_MSG:  byte_rdata <= (byte_addr < MLDSA65_MSG_MAX_BYTES) ? msg_rd_data : 8'd0;
                    REGION_SIG:  byte_rdata <= (byte_addr < MLDSA_MAX_SIG_BYTES) ? sig_rd_data : 8'd0;
                    REGION_CTX:  byte_rdata <= (byte_addr < MLDSA65_CTX_MAX_BYTES) ? ctx_rd_data : 8'd0;
                    REGION_SEED: byte_rdata <= (byte_addr < MLDSA65_SEED_BYTES) ? seed_rd_data : 8'd0;
                    REGION_AUX:  byte_rdata <= (byte_addr < MLDSA65_AUX_BYTES) ? aux_rd_data : 8'd0;
                    default:     byte_rdata <= 8'd0;
                endcase
            end

            core_message_len <= msg_len_q[12:0];
            core_level_sel <= level_sel;
            core_op_mode <= op_mode;

            if (start && !core_busy) begin
                core_start <= 1'b1;
            end

            if (core_done) begin
                done <= 1'b1;
                pass <= core_pass;
                error <= core_error;
                err_code <= core_err_code;
            end
        end
    end
endmodule

`default_nettype wire
