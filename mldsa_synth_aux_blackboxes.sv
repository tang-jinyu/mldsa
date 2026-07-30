`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

`ifdef MLDSA_DC_FORCE_AUX_STUBS

(* blackbox, syn_black_box, black_box *)
module mldsa_ntt_poly_mem_compat (
    input  wire                      clk,
    input  wire                      load_we,
    input  wire [7:0]                load_addr,
    input  wire [MLDSA_COEFF_W-1:0]  load_data,
    input  wire                      src_bank_sel,
    input  wire                      src_rd_req,
    input  wire                      src_rd_dual,
    input  wire [7:0]                src_rd0_addr,
    input  wire [7:0]                src_rd1_addr,
    output wire                      src_rd_valid,
    output wire [MLDSA_COEFF_W-1:0]  src_rd0_data,
    output wire [MLDSA_COEFF_W-1:0]  src_rd1_data,
    input  wire                      dst_bank_sel,
    input  wire                      dst_wr0_en,
    input  wire [7:0]                dst_wr0_addr,
    input  wire [MLDSA_COEFF_W-1:0]  dst_wr0_data,
    input  wire                      dst_wr1_en,
    input  wire [7:0]                dst_wr1_addr,
    input  wire [MLDSA_COEFF_W-1:0]  dst_wr1_data,
    input  wire                      host_bank_sel,
    input  wire [7:0]                host_rd_addr,
    output wire [MLDSA_COEFF_W-1:0]  host_rd_data
);
endmodule

(* blackbox, syn_black_box, black_box *)
module mldsa_ntt_zeta_rom (
    input  wire                      clk,
    input  wire [7:0]                addr,
    output wire [MLDSA_COEFF_W-1:0]  data
);
endmodule

(* blackbox, syn_black_box, black_box *)
module mldsa_sampler_challenge_mem_compat (
    input  wire                      clk,
    input  wire                      clr_we,
    input  wire [7:0]                clr_addr,
    input  wire [MLDSA_COEFF_W-1:0]  clr_data,
    input  wire [7:0]                rd0_addr,
    output wire [MLDSA_COEFF_W-1:0]  rd0_data,
    input  wire [7:0]                rd1_addr,
    output wire [MLDSA_COEFF_W-1:0]  rd1_data,
    input  wire                      wr0_en,
    input  wire [7:0]                wr0_addr,
    input  wire [MLDSA_COEFF_W-1:0]  wr0_data,
    input  wire                      wr1_en,
    input  wire [7:0]                wr1_addr,
    input  wire [MLDSA_COEFF_W-1:0]  wr1_data
);
endmodule

`endif

`default_nettype wire
