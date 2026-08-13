`timescale 1ns/1ps
`default_nettype none

import mldsa_fused_bank_map_pkg::*;

module mldsa_top_wrapper #(
    // Default to the functionally legal 65-bank fused configuration so both
    // direct FPGA projects and pure RTL simulation use the same configuration
    // unless a script deliberately overrides BANKS.
    parameter int BANKS = 65,
    parameter bit EXPANDA_PRIVATE_XOF = 1'b0
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         load_poly_we,
    input  wire [$clog2(BANKS)-1:0]     load_poly_bank,
    input  wire [7:0]                   load_poly_addr,
    input  wire [23:0]                  load_poly_data,
    input  wire                         host_rd_en,
    input  wire [$clog2(BANKS)-1:0]     host_rd_bank,
    input  wire [7:0]                   host_rd_addr,
    input  wire                         load_seed_we,
    input  wire [4:0]                   load_seed_addr,
    input  wire [7:0]                   load_seed_data,
    input  wire                         load_expanded_seed_we,
    input  wire [1:0]                   load_expanded_seed_sel,
    input  wire [5:0]                   load_expanded_seed_addr,
    input  wire [7:0]                   load_expanded_seed_data,
    input  wire                         standard_mode,
    input  wire                         load_ctx_we,
    input  wire [1:0]                   load_ctx_sel,
    input  wire [12:0]                  load_ctx_addr,
    input  wire [7:0]                   load_ctx_data,
    input  wire                         load_sig_hash_we,
    input  wire [5:0]                   load_sig_hash_addr,
    input  wire [7:0]                   load_sig_hash_data,
    input  wire [5:0]                   sig_hash_rd_addr,
    input  wire [12:0]                  message_len,
    input  wire                         start,
    input  wire [1:0]                   level_sel,
    input  wire [1:0]                   op_mode,
    output wire                         busy,
    output wire                         done,
    output wire                         pass,
    output wire                         error,
    output wire [7:0]                   err_code,
    output wire [7:0]                   uop_dbg,
    output wire [7:0]                   state_dbg,
    output wire [7:0]                   sig_hash_rd_data,
    output wire [23:0]                  host_rd_data,
    output wire                         keygen_byte_valid,
    output wire [2:0]                   keygen_byte_region,
    output wire [12:0]                  keygen_byte_addr,
    output wire [7:0]                   keygen_byte_data
);

    mldsa65_full_core #(
        .BANKS(BANKS),
        .EXPANDA_PRIVATE_XOF(EXPANDA_PRIVATE_XOF)
    ) u_core (
        .clk           (clk),
        .rst_n         (rst_n),
        .load_poly_we  (load_poly_we),
        .load_poly_bank(load_poly_bank),
        .load_poly_addr(load_poly_addr),
        .load_poly_data(load_poly_data),
        .host_rd_en    (host_rd_en),
        .host_rd_bank  (host_rd_bank),
        .host_rd_addr  (host_rd_addr),
        .load_seed_we  (load_seed_we),
        .load_seed_addr(load_seed_addr),
        .load_seed_data(load_seed_data),
        .load_expanded_seed_we(load_expanded_seed_we),
        .load_expanded_seed_sel(load_expanded_seed_sel),
        .load_expanded_seed_addr(load_expanded_seed_addr),
        .load_expanded_seed_data(load_expanded_seed_data),
        .standard_mode (standard_mode),
        .load_ctx_we   (load_ctx_we),
        .load_ctx_sel  (load_ctx_sel),
        .load_ctx_addr (load_ctx_addr),
        .load_ctx_data (load_ctx_data),
        .load_sig_hash_we(load_sig_hash_we),
        .load_sig_hash_addr(load_sig_hash_addr),
        .load_sig_hash_data(load_sig_hash_data),
        .sig_hash_rd_addr(sig_hash_rd_addr),
        .message_len   (message_len),
        .start         (start),
        .level_sel     (level_sel),
        .op_mode       (op_mode),
        .busy          (busy),
        .done          (done),
        .pass          (pass),
        .error         (error),
        .err_code      (err_code),
        .uop_dbg       (uop_dbg),
        .state_dbg     (state_dbg),
        .sig_hash_rd_data(sig_hash_rd_data),
        .host_rd_data  (host_rd_data),
        .keygen_byte_valid(keygen_byte_valid),
        .keygen_byte_region(keygen_byte_region),
        .keygen_byte_addr(keygen_byte_addr),
        .keygen_byte_data(keygen_byte_data)
    );
endmodule

`default_nettype wire
