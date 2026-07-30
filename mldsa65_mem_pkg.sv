`timescale 1ns/1ps
`default_nettype none

package mldsa_mem_pkg;
    localparam int MLDSA_Q = 8380417;
    localparam int MLDSA_N = 256;

    // 当前 ASAP7 比赛线采用统一 superset bank 规划。
    // 44/65/87 三个等级共用同一套物理 bank 口径，控制器只使用其中有效子集。
    localparam int MLDSA44_K = 4;
    localparam int MLDSA44_L = 4;
    localparam int MLDSA65_K = 6;
    localparam int MLDSA65_L = 5;
    localparam int MLDSA87_K = 8;
    localparam int MLDSA87_L = 7;

    localparam int MLDSA_UNIFIED_POLY_BANKS = 60;
    localparam int MLDSA44_POLY_BANKS       = MLDSA_UNIFIED_POLY_BANKS;
    localparam int MLDSA65_POLY_BANKS       = MLDSA_UNIFIED_POLY_BANKS;
    localparam int MLDSA87_POLY_BANKS       = MLDSA_UNIFIED_POLY_BANKS;
    localparam int MLDSA_POLY_ADDR_W        = 8;
    localparam int MLDSA_COEFF_W            = 24;

    localparam int MLDSA44_PK_BYTES  = 1312;
    localparam int MLDSA44_SK_BYTES  = 2560;
    localparam int MLDSA44_SIG_BYTES = 2420;
    localparam int MLDSA65_PK_BYTES  = 1952;
    localparam int MLDSA65_SK_BYTES  = 4032;
    localparam int MLDSA65_SIG_BYTES = 3309;
    localparam int MLDSA87_PK_BYTES  = 2592;
    localparam int MLDSA87_SK_BYTES  = 4896;
    localparam int MLDSA87_SIG_BYTES = 4627;

    localparam int MLDSA_MAX_PK_BYTES  = MLDSA87_PK_BYTES;
    localparam int MLDSA_MAX_SK_BYTES  = MLDSA87_SK_BYTES;
    localparam int MLDSA_MAX_SIG_BYTES = MLDSA87_SIG_BYTES;
    localparam int MLDSA_MSG_MAX_BYTES = 8192;
    localparam int MLDSA_CTX_MAX_BYTES = 255;
    localparam int MLDSA_SEED_BYTES    = 32;
    localparam int MLDSA_AUX_BYTES     = 64;

    localparam logic [2:0] REGION_PK   = 3'd0;
    localparam logic [2:0] REGION_SK   = 3'd1;
    localparam logic [2:0] REGION_MSG  = 3'd2;
    localparam logic [2:0] REGION_SIG  = 3'd3;
    localparam logic [2:0] REGION_CTX  = 3'd4;
    localparam logic [2:0] REGION_SEED = 3'd5;
    localparam logic [2:0] REGION_AUX  = 3'd6;

    localparam logic [5:0] BANK_Z_BASE      = 6'd0;
    localparam logic [5:0] BANK_T1_BASE     = 6'd8;
    localparam logic [5:0] BANK_C           = 6'd16;
    localparam logic [5:0] BANK_AZ_BASE     = 6'd24;
    localparam logic [5:0] BANK_CT1_BASE    = 6'd32;
    localparam logic [5:0] BANK_WPRIME_BASE = 6'd40;
    localparam logic [5:0] BANK_S2_BASE     = 6'd48;
    localparam logic [5:0] BANK_T0_BASE     = 6'd54;

    // 生命周期复用别名。是否安全取决于 full_core / siggen FSM 读写调度，不能仅凭包定义保证。
    localparam logic [5:0] BANK_Y_BASE  = BANK_Z_BASE;
    localparam logic [5:0] BANK_S1_BASE = BANK_Z_BASE;

    function automatic int mldsa_level_poly_banks(input logic [1:0] level_sel);
        begin
            unique case (level_sel)
                2'd0:    mldsa_level_poly_banks = MLDSA44_POLY_BANKS;
                2'd1:    mldsa_level_poly_banks = MLDSA65_POLY_BANKS;
                2'd2:    mldsa_level_poly_banks = MLDSA87_POLY_BANKS;
                default: mldsa_level_poly_banks = MLDSA65_POLY_BANKS;
            endcase
        end
    endfunction
endpackage

package mldsa65_mem_pkg;
    localparam int MLDSA65_Q           = mldsa_mem_pkg::MLDSA_Q;
    localparam int MLDSA65_N           = mldsa_mem_pkg::MLDSA_N;
    localparam int MLDSA65_K           = mldsa_mem_pkg::MLDSA65_K;
    localparam int MLDSA65_L           = mldsa_mem_pkg::MLDSA65_L;
    localparam int MLDSA65_POLY_BANKS  = mldsa_mem_pkg::MLDSA65_POLY_BANKS;
    localparam int MLDSA65_POLY_ADDR_W = mldsa_mem_pkg::MLDSA_POLY_ADDR_W;
    localparam int MLDSA65_COEFF_W     = mldsa_mem_pkg::MLDSA_COEFF_W;

    localparam int MLDSA44_PK_BYTES    = mldsa_mem_pkg::MLDSA44_PK_BYTES;
    localparam int MLDSA44_SK_BYTES    = mldsa_mem_pkg::MLDSA44_SK_BYTES;
    localparam int MLDSA44_SIG_BYTES   = mldsa_mem_pkg::MLDSA44_SIG_BYTES;
    localparam int MLDSA65_PK_BYTES    = mldsa_mem_pkg::MLDSA65_PK_BYTES;
    localparam int MLDSA65_SK_BYTES    = mldsa_mem_pkg::MLDSA65_SK_BYTES;
    localparam int MLDSA65_SIG_BYTES   = mldsa_mem_pkg::MLDSA65_SIG_BYTES;
    localparam int MLDSA87_PK_BYTES    = mldsa_mem_pkg::MLDSA87_PK_BYTES;
    localparam int MLDSA87_SK_BYTES    = mldsa_mem_pkg::MLDSA87_SK_BYTES;
    localparam int MLDSA87_SIG_BYTES   = mldsa_mem_pkg::MLDSA87_SIG_BYTES;
    localparam int MLDSA_MAX_PK_BYTES  = mldsa_mem_pkg::MLDSA_MAX_PK_BYTES;
    localparam int MLDSA_MAX_SK_BYTES  = mldsa_mem_pkg::MLDSA_MAX_SK_BYTES;
    localparam int MLDSA_MAX_SIG_BYTES = mldsa_mem_pkg::MLDSA_MAX_SIG_BYTES;
    localparam int MLDSA65_MSG_MAX_BYTES = mldsa_mem_pkg::MLDSA_MSG_MAX_BYTES;
    localparam int MLDSA65_CTX_MAX_BYTES = mldsa_mem_pkg::MLDSA_CTX_MAX_BYTES;
    localparam int MLDSA65_SEED_BYTES    = mldsa_mem_pkg::MLDSA_SEED_BYTES;
    localparam int MLDSA65_AUX_BYTES     = mldsa_mem_pkg::MLDSA_AUX_BYTES;

    localparam logic [2:0] REGION_PK   = mldsa_mem_pkg::REGION_PK;
    localparam logic [2:0] REGION_SK   = mldsa_mem_pkg::REGION_SK;
    localparam logic [2:0] REGION_MSG  = mldsa_mem_pkg::REGION_MSG;
    localparam logic [2:0] REGION_SIG  = mldsa_mem_pkg::REGION_SIG;
    localparam logic [2:0] REGION_CTX  = mldsa_mem_pkg::REGION_CTX;
    localparam logic [2:0] REGION_SEED = mldsa_mem_pkg::REGION_SEED;
    localparam logic [2:0] REGION_AUX  = mldsa_mem_pkg::REGION_AUX;

    localparam logic [5:0] BANK_Z_BASE      = mldsa_mem_pkg::BANK_Z_BASE;
    localparam logic [5:0] BANK_T1_BASE     = mldsa_mem_pkg::BANK_T1_BASE;
    localparam logic [5:0] BANK_C           = mldsa_mem_pkg::BANK_C;
    localparam logic [5:0] BANK_AZ_BASE     = mldsa_mem_pkg::BANK_AZ_BASE;
    localparam logic [5:0] BANK_CT1_BASE    = mldsa_mem_pkg::BANK_CT1_BASE;
    localparam logic [5:0] BANK_WPRIME_BASE = mldsa_mem_pkg::BANK_WPRIME_BASE;
    localparam logic [5:0] BANK_Y_BASE      = mldsa_mem_pkg::BANK_Y_BASE;
    localparam logic [5:0] BANK_S1_BASE     = mldsa_mem_pkg::BANK_S1_BASE;
    localparam logic [5:0] BANK_S2_BASE     = mldsa_mem_pkg::BANK_S2_BASE;
    localparam logic [5:0] BANK_T0_BASE     = mldsa_mem_pkg::BANK_T0_BASE;
endpackage

`default_nettype wire
