`timescale 1ns/1ps
`default_nettype none

module mldsa_top_wrapper #(
    parameter int BANKS = 65
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
    input  wire                         standard_mode,
    input  wire                         load_ctx_we,
    input  wire [1:0]                   load_ctx_sel,
    input  wire [12:0]                  load_ctx_addr,
    input  wire [7:0]                   load_ctx_data,
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
    output wire [23:0]                  host_rd_data
);

    mldsa65_full_core #(.BANKS(BANKS)) u_core (
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
        .standard_mode (standard_mode),
        .load_ctx_we   (load_ctx_we),
        .load_ctx_sel  (load_ctx_sel),
        .load_ctx_addr (load_ctx_addr),
        .load_ctx_data (load_ctx_data),
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
        .host_rd_data  (host_rd_data)
    );
endmodule

`default_nettype wire
