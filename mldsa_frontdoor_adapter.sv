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
    output logic                        core_load_expanded_seed_we,
    output logic [1:0]                  core_load_expanded_seed_sel,
    output logic [5:0]                  core_load_expanded_seed_addr,
    output logic [7:0]                  core_load_expanded_seed_data,
    output logic                        core_standard_mode,
    output logic                        core_load_ctx_we,
    output logic [1:0]                  core_load_ctx_sel,
    output logic [12:0]                 core_load_ctx_addr,
    output logic [7:0]                  core_load_ctx_data,
    output logic                        core_load_sig_hash_we,
    output logic [5:0]                  core_load_sig_hash_addr,
    output logic [7:0]                  core_load_sig_hash_data,
    output logic [5:0]                  core_sig_hash_rd_addr,
    input  wire [7:0]                   core_sig_hash_rd_data,
    output logic [12:0]                 core_message_len,
    output logic                        core_start,
    output logic [1:0]                  core_level_sel,
    output logic [1:0]                  core_op_mode,
    input  wire                         core_busy,
    input  wire                         core_done,
    input  wire                         core_pass,
    input  wire                         core_error,
    input  wire [7:0]                   core_err_code,
    input  wire                         core_keygen_byte_valid,
    input  wire [2:0]                   core_keygen_byte_region,
    input  wire [12:0]                  core_keygen_byte_addr,
    input  wire [7:0]                   core_keygen_byte_data
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
    localparam int PK_STORE_ADDR_W = (MLDSA_MAX_PK_BYTES <= 1) ? 1 : $clog2(MLDSA_MAX_PK_BYTES);
    localparam int SK_STORE_ADDR_W = (MLDSA_MAX_SK_BYTES <= 1) ? 1 : $clog2(MLDSA_MAX_SK_BYTES);

    logic [PK_STORE_ADDR_W-1:0] pk_rd_addr;
    logic [SK_STORE_ADDR_W-1:0] sk_rd_addr;
    logic [12:0] sig_store_rd_addr;
    logic        sig_store_wr_en;
    logic [12:0] sig_store_wr_addr;
    logic [7:0]  sig_store_wr_data;
    logic        byte_rd_pending_q;
    logic [2:0]  byte_rd_region_q;
    logic [12:0] byte_rd_addr_q;

    localparam logic [7:0] ERR_FUSED_KEYGEN_UNCLOSED = 8'hE1;
    localparam logic [7:0] ERR_FUSED_LEVEL_UNCLOSED  = 8'hE2;
    localparam int Q = 8380417;
`ifdef MLDSA_CFG_LEVEL_44
    localparam int FD_K = 4;
    localparam int FD_L = 4;
    localparam int FD_ETA = 2;
    localparam int FD_CTILDE_BYTES = 32;
    localparam int FD_Z_BITS = 18;
    localparam int FD_Z_BYTES_PER_POLY = 576;
    localparam int FD_OMEGA = 80;
    localparam int FD_GAMMA1 = 131072;
`elsif MLDSA_CFG_LEVEL_87
    localparam int FD_K = 8;
    localparam int FD_L = 7;
    localparam int FD_ETA = 2;
    localparam int FD_CTILDE_BYTES = 64;
    localparam int FD_Z_BITS = 20;
    localparam int FD_Z_BYTES_PER_POLY = 640;
    localparam int FD_OMEGA = 75;
    localparam int FD_GAMMA1 = 524288;
`else
    localparam int FD_K = 6;
    localparam int FD_L = 5;
    localparam int FD_ETA = 4;
    localparam int FD_CTILDE_BYTES = 48;
    localparam int FD_Z_BITS = 20;
    localparam int FD_Z_BYTES_PER_POLY = 640;
    localparam int FD_OMEGA = 55;
    localparam int FD_GAMMA1 = 524288;
`endif
    localparam int FD_ETA_BYTES_PER_POLY = (FD_ETA == 2) ? 96 : 128;
    localparam int FD_S1_OFF = 128;
    localparam int FD_S1_BYTES = FD_L * FD_ETA_BYTES_PER_POLY;
    localparam int FD_S2_OFF = FD_S1_OFF + FD_S1_BYTES;
    localparam int FD_S2_BYTES = FD_K * FD_ETA_BYTES_PER_POLY;
    localparam int FD_T0_OFF = FD_S2_OFF + FD_S2_BYTES;
    localparam int FD_Z_BYTES = FD_L * FD_Z_BYTES_PER_POLY;
    localparam int FD_Z_GROUP_BYTES = (FD_Z_BITS == 18) ? 9 : 5;
    localparam int FD_Z_GROUPS_PER_POLY = (FD_Z_BITS == 18) ? 64 : 128;
    localparam int FD_Z_GROUP_COUNT = FD_L * FD_Z_GROUPS_PER_POLY;
    // Keep these frontdoor preload/readback bases aligned with
    // mldsa65_full_core.  BANKS >= 120 enables the FPGA resident-A layout, where
    // the immutable A matrix occupies the leading K*L banks and mutable vectors
    // move after it.  Smaller configurations use the compact fused map.
    localparam bit FD_USE_RESIDENT_A = (BANKS >= 120);
    localparam int FD_MAP_K_BANKS = FD_USE_RESIDENT_A ? MLDSA_K_MAX : FD_K;
    localparam int FD_MAP_L_BANKS = FD_USE_RESIDENT_A ? MLDSA_L_MAX : FD_L;
    localparam int FD_A_STORAGE_BANKS = FD_MAP_K_BANKS * FD_MAP_L_BANKS;
    localparam int FD_B_A_BASE     = 0;
    localparam int FD_B_S1_BASE    = FD_USE_RESIDENT_A ? (FD_B_A_BASE + FD_A_STORAGE_BANKS) : 0;
    localparam int FD_B_S1N_BASE   = FD_B_S1_BASE + FD_MAP_L_BANKS;
    localparam int FD_B_T1N_BASE   = FD_B_S1N_BASE + FD_MAP_L_BANKS;
    localparam int FD_B_Y_BASE     = FD_B_T1N_BASE + FD_MAP_K_BANKS;
    localparam int FD_B_ACC_BASE   = FD_B_Y_BASE + FD_MAP_L_BANKS;
    localparam int FD_B_W_BASE     = FD_B_ACC_BASE + FD_MAP_K_BANKS;
    localparam int FD_B_W1_BASE    = FD_B_W_BASE + FD_MAP_K_BANKS;
    localparam int FD_B_C_BASE     = FD_B_W1_BASE + FD_MAP_K_BANKS;
    localparam int FD_B_CN_BASE    = FD_B_C_BASE + 1;
    localparam int FD_B_TMP_BASE   = FD_B_CN_BASE + 1;
    localparam int FD_B_Z_BASE     = FD_B_Y_BASE;
    localparam int FD_B_HINT_BASE  = FD_B_TMP_BASE + 1;
    localparam int FD_B_A_TMP_BASE = FD_B_HINT_BASE + FD_MAP_K_BANKS;
    localparam int FD_B_S2_BASE    = FD_USE_RESIDENT_A ? FD_B_A_TMP_BASE : (FD_B_A_TMP_BASE + 1);
    // The sign frontdoor preloads t0 into the W scratch window used by the
    // fused hint-generation sequence.
    localparam int FD_B_T0_TMP_BASE = FD_B_W_BASE;
    localparam int FD_B_HINT_RESULT_BASE = FD_B_W1_BASE;

    typedef enum logic [6:0] {
        F_IDLE,
        F_LOAD_MU_PREFIX0,
        F_LOAD_MU_PREFIX1,
        F_PRE_S1_RD,
        F_PRE_S1_LO,
        F_PRE_S1_HI,
        F_PRE_S1_ETA2_RD,
        F_PRE_S1_ETA2_CAP,
        F_PRE_S1_ETA2_EMIT,
        F_PRE_S2_RD,
        F_PRE_S2_LO,
        F_PRE_S2_HI,
        F_PRE_S2_ETA2_RD,
        F_PRE_S2_ETA2_CAP,
        F_PRE_S2_ETA2_EMIT,
        F_PRE_T0_RD,
        F_PRE_T0_CAP,
        F_PRE_T0_EMIT,
        F_START_CORE,
        F_WAIT_CORE,
        F_PACK_C_RD,
        F_PACK_C_WAIT,
        F_PACK_C_CAP,
        F_PACK_Z_RD0,
        F_PACK_Z_WAIT0,
        F_PACK_Z_CAP0,
        F_PACK_Z_WAIT1,
        F_PACK_Z_CAP1,
        F_PACK_Z_RD2,
        F_PACK_Z_WAIT2,
        F_PACK_Z_CAP2,
        F_PACK_Z_RD3,
        F_PACK_Z_WAIT3,
        F_PACK_Z_CAP3,
        F_PACK_Z_WR,
        F_PACK_H_CLEAR,
        F_PACK_H_RD,
        F_PACK_H_WAIT,
        F_PACK_H_CAP,
        F_PACK_H_MARK,
        F_VER_MU_PREFIX0,
        F_VER_MU_PREFIX1,
        F_VER_RHO_RD,
        F_VER_RHO_CAP,
        F_VER_C_RD,
        F_VER_C_CAP,
        F_VER_HINT_CLEAR,
        F_VER_Z_RD,
        F_VER_Z_CAP,
        F_VER_Z_WR0,
        F_VER_Z_WR1,
        F_VER_Z_WR2,
        F_VER_Z_WR3,
        F_VER_T1_RD,
        F_VER_T1_CAP,
        F_VER_T1_WR0,
        F_VER_T1_WR1,
        F_VER_T1_WR2,
        F_VER_T1_WR3,
        F_VER_H_MARK_RD,
        F_VER_H_MARK_CAP,
        F_VER_H_POS_RD,
        F_VER_H_POS_CAP,
        F_VER_H_UNUSED_RD,
        F_VER_H_UNUSED_CAP,
        F_REPORT_DONE
    } front_state_e;

    front_state_e fstate;
    logic [2:0] pre_poly;
    logic [7:0] pre_byte_idx;
    logic [7:0] pre_coeff_idx;
    logic [7:0] pre_t0_buf [0:12];
    logic [3:0] pre_t0_byte_idx;
    logic [2:0] pre_t0_lane;
    logic [7:0] pre_eta_byte;

    logic [12:0] pack_sig_addr;
    logic [2:0] pack_poly;
    logic [7:0] pack_coeff;
    logic [3:0] pack_phase;
    logic [23:0] z_coeff0;
    logic [23:0] z_coeff1;
    logic [23:0] z_coeff2;
    logic [23:0] z_coeff3;
    logic [19:0] z_enc0;
    logic [19:0] z_enc1;
    logic [19:0] z_enc2;
    logic [19:0] z_enc3;
    logic [9:0] verify_group_idx;
    logic [3:0] verify_phase;
    logic [7:0] verify_h_start;
    logic [7:0] verify_h_end;
    logic [7:0] verify_h_pos;
    logic [7:0] hint_count;
    logic [7:0] hint_poly_end_count [0:FD_K-1];
    logic pk_store_wr_en;
    logic [PK_STORE_ADDR_W-1:0] pk_store_wr_addr;
    logic [7:0] pk_store_wr_data;
    logic sk_store_wr_en;
    logic [SK_STORE_ADDR_W-1:0] sk_store_wr_addr;
    logic [7:0] sk_store_wr_data;

    function automatic logic sk_tr_byte(input logic [12:0] addr);
        sk_tr_byte = (addr >= 13'd64) && (addr < 13'd128);
    endfunction

    function automatic logic sk_rho_byte(input logic [12:0] addr);
        sk_rho_byte = (addr < 13'd32);
    endfunction

    function automatic logic sk_k_byte(input logic [12:0] addr);
        sk_k_byte = (addr >= 13'd32) && (addr < 13'd64);
    endfunction

    function automatic logic [23:0] signed_small_to_modq(input integer signed value);
        begin
            if (value < 0) signed_small_to_modq = Q[23:0] + value[23:0];
            else signed_small_to_modq = value[23:0];
        end
    endfunction

    function automatic logic [23:0] decode_eta4_low(input logic [7:0] b);
        decode_eta4_low = signed_small_to_modq(4 - integer'(b[3:0]));
    endfunction

    function automatic logic [23:0] decode_eta4_high(input logic [7:0] b);
        decode_eta4_high = signed_small_to_modq(4 - integer'(b[7:4]));
    endfunction

    function automatic logic [2:0] eta2_word(input int lane);
        begin
            unique case (lane)
                0: eta2_word = pre_t0_buf[0][2:0];
                1: eta2_word = pre_t0_buf[0][5:3];
                2: eta2_word = {pre_t0_buf[1][0], pre_t0_buf[0][7:6]};
                3: eta2_word = pre_t0_buf[1][3:1];
                4: eta2_word = pre_t0_buf[1][6:4];
                5: eta2_word = {pre_t0_buf[2][1:0], pre_t0_buf[1][7]};
                6: eta2_word = pre_t0_buf[2][4:2];
                default: eta2_word = pre_t0_buf[2][7:5];
            endcase
        end
    endfunction

    function automatic logic [23:0] decode_eta2(input int lane);
        decode_eta2 = signed_small_to_modq(2 - integer'(eta2_word(lane)));
    endfunction

    function automatic logic [12:0] t0_word65(input int lane);
        begin
            unique case (lane)
                0: t0_word65 = (pre_t0_buf[0] | (pre_t0_buf[1] << 8)) & 13'h1fff;
                1: t0_word65 = ((pre_t0_buf[1] >> 5) | (pre_t0_buf[2] << 3) | (pre_t0_buf[3] << 11)) & 13'h1fff;
                2: t0_word65 = ((pre_t0_buf[3] >> 2) | (pre_t0_buf[4] << 6)) & 13'h1fff;
                3: t0_word65 = ((pre_t0_buf[4] >> 7) | (pre_t0_buf[5] << 1) | (pre_t0_buf[6] << 9)) & 13'h1fff;
                4: t0_word65 = ((pre_t0_buf[6] >> 4) | (pre_t0_buf[7] << 4) | (pre_t0_buf[8] << 12)) & 13'h1fff;
                5: t0_word65 = ((pre_t0_buf[8] >> 1) | (pre_t0_buf[9] << 7)) & 13'h1fff;
                6: t0_word65 = ((pre_t0_buf[9] >> 6) | (pre_t0_buf[10] << 2) | (pre_t0_buf[11] << 10)) & 13'h1fff;
                default: t0_word65 = ((pre_t0_buf[11] >> 3) | (pre_t0_buf[12] << 5)) & 13'h1fff;
            endcase
        end
    endfunction

    function automatic logic [23:0] decode_t0_65(input int lane);
        integer signed tmp;
        begin
            tmp = 4096 - integer'(t0_word65(lane));
            decode_t0_65 = signed_small_to_modq(tmp);
        end
    endfunction

    function automatic integer signed decode_z_signed(input logic [19:0] enc);
        decode_z_signed = FD_GAMMA1 - integer'(enc);
    endfunction

    function automatic logic [23:0] decode_z_modq(input logic [19:0] enc);
        integer signed z;
        begin
            z = decode_z_signed(enc);
            decode_z_modq = signed_small_to_modq(z);
        end
    endfunction

    function automatic logic [19:0] sig_z0_from_buf();
        sig_z0_from_buf = {pre_t0_buf[2][3:0], pre_t0_buf[1], pre_t0_buf[0]};
    endfunction

    function automatic logic [19:0] sig_z1_from_buf();
        sig_z1_from_buf = {pre_t0_buf[4], pre_t0_buf[3], pre_t0_buf[2][7:4]};
    endfunction

    function automatic logic [17:0] sig_z18_0_from_buf();
        sig_z18_0_from_buf = {pre_t0_buf[2][1:0], pre_t0_buf[1], pre_t0_buf[0]};
    endfunction

    function automatic logic [17:0] sig_z18_1_from_buf();
        sig_z18_1_from_buf = {pre_t0_buf[4][3:0], pre_t0_buf[3], pre_t0_buf[2][7:2]};
    endfunction

    function automatic logic [17:0] sig_z18_2_from_buf();
        sig_z18_2_from_buf = {pre_t0_buf[6][5:0], pre_t0_buf[5], pre_t0_buf[4][7:4]};
    endfunction

    function automatic logic [17:0] sig_z18_3_from_buf();
        sig_z18_3_from_buf = {pre_t0_buf[8], pre_t0_buf[7], pre_t0_buf[6][7:6]};
    endfunction

    function automatic logic [9:0] pk_t1_0_from_buf();
        pk_t1_0_from_buf = (10'(pre_t0_buf[0]) | (10'(pre_t0_buf[1]) << 8)) & 10'h3ff;
    endfunction

    function automatic logic [9:0] pk_t1_1_from_buf();
        pk_t1_1_from_buf = ((10'(pre_t0_buf[1]) >> 2) | (10'(pre_t0_buf[2]) << 6)) & 10'h3ff;
    endfunction

    function automatic logic [9:0] pk_t1_2_from_buf();
        pk_t1_2_from_buf = ((10'(pre_t0_buf[2]) >> 4) | (10'(pre_t0_buf[3]) << 4)) & 10'h3ff;
    endfunction

    function automatic logic [9:0] pk_t1_3_from_buf();
        pk_t1_3_from_buf = ((10'(pre_t0_buf[3]) >> 6) | (10'(pre_t0_buf[4]) << 2)) & 10'h3ff;
    endfunction

    function automatic integer signed center_coeff(input logic [23:0] coeff);
        integer signed tmp;
        begin
            tmp = integer'(coeff);
            if (tmp > (Q / 2)) tmp = tmp - Q;
            center_coeff = tmp;
        end
    endfunction

    always_comb begin
        pk_store_wr_en = byte_we && !core_busy && byte_region == REGION_PK && byte_addr < MLDSA_MAX_PK_BYTES;
        pk_store_wr_addr = byte_addr[PK_STORE_ADDR_W-1:0];
        pk_store_wr_data = byte_wdata;
        sk_store_wr_en = byte_we && !core_busy && byte_region == REGION_SK && byte_addr < MLDSA_MAX_SK_BYTES;
        sk_store_wr_addr = byte_addr[SK_STORE_ADDR_W-1:0];
        sk_store_wr_data = byte_wdata;
        if (core_keygen_byte_valid && core_keygen_byte_region == REGION_PK && core_keygen_byte_addr < MLDSA_MAX_PK_BYTES) begin
            pk_store_wr_en = 1'b1;
            pk_store_wr_addr = core_keygen_byte_addr[PK_STORE_ADDR_W-1:0];
            pk_store_wr_data = core_keygen_byte_data;
`ifdef MLDSA_DEBUG_DISPLAY
            if (core_keygen_byte_addr < 13'd8) begin
                $display("FRONTDOOR_KEYGEN_PK_WR addr=%0d data=%02x",
                         core_keygen_byte_addr, core_keygen_byte_data);
            end
`endif
        end
        if (core_keygen_byte_valid && core_keygen_byte_region == REGION_SK && core_keygen_byte_addr < MLDSA_MAX_SK_BYTES) begin
            sk_store_wr_en = 1'b1;
            sk_store_wr_addr = core_keygen_byte_addr[SK_STORE_ADDR_W-1:0];
            sk_store_wr_data = core_keygen_byte_data;
        end

        pk_rd_addr = byte_addr[PK_STORE_ADDR_W-1:0];
        sk_rd_addr = byte_addr[SK_STORE_ADDR_W-1:0];
        if (fstate == F_PRE_S1_RD || fstate == F_PRE_S1_ETA2_RD) begin
            sk_rd_addr = SK_STORE_ADDR_W'(FD_S1_OFF + (pre_poly * FD_ETA_BYTES_PER_POLY) + pre_byte_idx + pre_t0_byte_idx);
        end else if (fstate == F_PRE_S2_RD || fstate == F_PRE_S2_ETA2_RD) begin
            sk_rd_addr = SK_STORE_ADDR_W'(FD_S2_OFF + (pre_poly * FD_ETA_BYTES_PER_POLY) + pre_byte_idx + pre_t0_byte_idx);
        end else if (fstate == F_PRE_T0_RD) begin
            sk_rd_addr = SK_STORE_ADDR_W'(FD_T0_OFF + (pre_poly * 416) + ((pre_byte_idx * 13) + pre_t0_byte_idx));
        end

        if (fstate == F_VER_RHO_RD) begin
            pk_rd_addr = PK_STORE_ADDR_W'(pre_byte_idx[4:0]);
        end else if (fstate == F_VER_T1_RD) begin
            pk_rd_addr = PK_STORE_ADDR_W'(32 + (verify_group_idx * 5) + verify_phase);
        end

        sig_store_rd_addr = byte_addr[12:0];
        if (fstate == F_VER_C_RD) begin
            sig_store_rd_addr = {7'd0, pre_byte_idx[5:0]};
        end else if (fstate == F_VER_Z_RD) begin
            sig_store_rd_addr = 13'(FD_CTILDE_BYTES + (verify_group_idx * FD_Z_GROUP_BYTES) + verify_phase);
        end else if (fstate == F_VER_H_MARK_RD) begin
            sig_store_rd_addr = 13'(FD_CTILDE_BYTES + FD_Z_BYTES + FD_OMEGA + pre_poly);
        end else if (fstate == F_VER_H_POS_RD || fstate == F_VER_H_UNUSED_RD) begin
            sig_store_rd_addr = 13'(FD_CTILDE_BYTES + FD_Z_BYTES + verify_h_pos);
        end
        z_enc0 = 20'(FD_GAMMA1 - center_coeff(z_coeff0));
        z_enc1 = 20'(FD_GAMMA1 - center_coeff(z_coeff1));
        z_enc2 = 20'(FD_GAMMA1 - center_coeff(z_coeff2));
        z_enc3 = 20'(FD_GAMMA1 - center_coeff(z_coeff3));
    end

    mldsa_byte_store_async #(.DEPTH(MLDSA_MAX_PK_BYTES), .ADDR_W(PK_STORE_ADDR_W)) u_pk_store (
        .clk    (clk),
        .wr_en  (pk_store_wr_en),
        .wr_addr(pk_store_wr_addr),
        .wr_data(pk_store_wr_data),
        .rd_addr(pk_rd_addr),
        .rd_data(pk_rd_data)
    );

    mldsa_byte_store_async #(.DEPTH(MLDSA_MAX_SK_BYTES), .ADDR_W(SK_STORE_ADDR_W)) u_sk_store (
        .clk    (clk),
        .wr_en  (sk_store_wr_en),
        .wr_addr(sk_store_wr_addr),
        .wr_data(sk_store_wr_data),
        .rd_addr(sk_rd_addr),
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
        .wr_en  (sig_store_wr_en || (byte_we && !core_busy && byte_region == REGION_SIG && byte_addr < MLDSA_MAX_SIG_BYTES)),
        .wr_addr(sig_store_wr_en ? sig_store_wr_addr : byte_addr[12:0]),
        .wr_data(sig_store_wr_en ? sig_store_wr_data : byte_wdata),
        .rd_addr(sig_store_rd_addr),
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
            core_load_expanded_seed_we <= 1'b0;
            core_load_expanded_seed_sel <= 2'd0;
            core_load_expanded_seed_addr <= '0;
            core_load_expanded_seed_data <= 8'd0;
            core_standard_mode <= 1'b1;
            core_load_ctx_we <= 1'b0;
            core_load_ctx_sel <= 2'd0;
            core_load_ctx_addr <= '0;
            core_load_ctx_data <= '0;
            core_load_sig_hash_we <= 1'b0;
            core_load_sig_hash_addr <= 6'd0;
            core_load_sig_hash_data <= 8'd0;
            core_sig_hash_rd_addr <= 6'd0;
            core_message_len <= 13'd0;
            core_start <= 1'b0;
            core_level_sel <= MLDSA_LEVEL_65;
            core_op_mode <= MLDSA_OP_SIGN;
            sig_store_wr_en <= 1'b0;
            sig_store_wr_addr <= 13'd0;
            sig_store_wr_data <= 8'd0;
            byte_rd_pending_q <= 1'b0;
            byte_rd_region_q <= 3'd0;
            byte_rd_addr_q <= 13'd0;
            fstate <= F_IDLE;
            pre_poly <= 3'd0;
            pre_byte_idx <= 8'd0;
            pre_coeff_idx <= 8'd0;
            pre_t0_byte_idx <= 4'd0;
            pre_t0_lane <= 3'd0;
            pre_eta_byte <= 8'd0;
            pack_sig_addr <= 13'd0;
            pack_poly <= 3'd0;
            pack_coeff <= 8'd0;
            pack_phase <= 3'd0;
            z_coeff0 <= 24'd0;
            z_coeff1 <= 24'd0;
            verify_group_idx <= 10'd0;
            verify_phase <= 3'd0;
            verify_h_start <= 8'd0;
            verify_h_end <= 8'd0;
            verify_h_pos <= 8'd0;
            hint_count <= 8'd0;
            // Do not bulk-clear the byte-store arrays on reset.
            // DC expands these loops during elaboration and the fused frontdoor
            // becomes unnecessarily hard to compile. The ACVP/KAT benches write
            // the active byte ranges explicitly before start.
        end else begin
            done <= 1'b0;
            core_start <= 1'b0;
            core_load_seed_we <= 1'b0;
            core_load_expanded_seed_we <= 1'b0;
            core_load_ctx_we <= 1'b0;
            core_load_sig_hash_we <= 1'b0;
            core_load_poly_we <= 1'b0;
            core_host_rd_en <= 1'b0;
            sig_store_wr_en <= 1'b0;
            busy <= core_busy || (fstate != F_IDLE);

            if (byte_rd_pending_q) begin
                byte_rd_pending_q <= 1'b0;
                unique case (byte_rd_region_q)
                    REGION_PK:   byte_rdata <= (byte_rd_addr_q < MLDSA_MAX_PK_BYTES) ? pk_rd_data : 8'd0;
                    REGION_SK:   byte_rdata <= (byte_rd_addr_q < MLDSA_MAX_SK_BYTES) ? sk_rd_data : 8'd0;
                    REGION_MSG:  byte_rdata <= (byte_rd_addr_q < MLDSA65_MSG_MAX_BYTES) ? msg_rd_data : 8'd0;
                    REGION_SIG:  byte_rdata <= (byte_rd_addr_q < MLDSA_MAX_SIG_BYTES) ? sig_rd_data : 8'd0;
                    REGION_CTX:  byte_rdata <= (byte_rd_addr_q < MLDSA65_CTX_MAX_BYTES) ? ctx_rd_data : 8'd0;
                    REGION_SEED: byte_rdata <= (byte_rd_addr_q < MLDSA65_SEED_BYTES) ? seed_rd_data : 8'd0;
                    REGION_AUX:  byte_rdata <= (byte_rd_addr_q < MLDSA65_AUX_BYTES) ? aux_rd_data : 8'd0;
                    default:     byte_rdata <= 8'd0;
                endcase
            end

            if (len_we && !core_busy && fstate == F_IDLE) begin
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

            if (byte_we && !core_busy && fstate == F_IDLE) begin
                unique case (byte_region)
                    REGION_CTX:  if (byte_addr < MLDSA65_CTX_MAX_BYTES) begin
                                    core_load_ctx_we <= 1'b1;
                                    core_load_ctx_sel <= 2'd1;
                                    core_load_ctx_addr <= byte_addr + 13'd2;
                                    core_load_ctx_data <= byte_wdata;
                                end
                    REGION_MSG:  if (byte_addr < MLDSA65_MSG_MAX_BYTES) begin
                                    core_load_ctx_we <= 1'b1;
                                    core_load_ctx_sel <= 2'd1;
                                    core_load_ctx_addr <= byte_addr + {5'd0, ctx_len_q[7:0]} + 13'd2;
                                    core_load_ctx_data <= byte_wdata;
                                end
                    REGION_SK:   begin
                                    if (sk_rho_byte(byte_addr)) begin
                                        core_load_expanded_seed_we <= 1'b1;
                                        core_load_expanded_seed_sel <= 2'd0;
                                        core_load_expanded_seed_addr <= {1'b0, byte_addr[4:0]};
                                        core_load_expanded_seed_data <= byte_wdata;
                                    end else if (sk_k_byte(byte_addr)) begin
                                        core_load_expanded_seed_we <= 1'b1;
                                        core_load_expanded_seed_sel <= 2'd2;
                                        core_load_expanded_seed_addr <= {1'b0, byte_addr[4:0]};
                                        core_load_expanded_seed_data <= byte_wdata;
                                    end else if (sk_tr_byte(byte_addr)) begin
                                        core_load_ctx_we <= 1'b1;
                                        core_load_ctx_sel <= 2'd0;
                                        core_load_ctx_addr <= byte_addr - 13'd64;
                                        core_load_ctx_data <= byte_wdata;
                                    end
                                end
                    REGION_SEED: if (byte_addr < MLDSA65_SEED_BYTES) begin
                                    core_load_seed_we <= 1'b1;
                                    core_load_seed_addr <= byte_addr[4:0];
                                    core_load_seed_data <= byte_wdata;
                                 end
                    default: ;
                endcase
            end

            if (byte_rd_en && !core_busy && fstate == F_IDLE) begin
                byte_rd_pending_q <= 1'b1;
                byte_rd_region_q <= byte_region;
                byte_rd_addr_q <= byte_addr;
            end

            core_message_len <= msg_len_q[12:0] + {5'd0, ctx_len_q[7:0]} + 13'd2;
            core_level_sel <= level_sel;
            core_op_mode <= op_mode;

            if (start && !core_busy && fstate == F_IDLE) begin
                if (op_mode == MLDSA_OP_KEYGEN) begin
                    fstate <= F_START_CORE;
                end else if (op_mode == MLDSA_OP_SIGN) begin
                    pre_poly <= 3'd0;
                    pre_byte_idx <= 8'd0;
                    pre_coeff_idx <= 8'd0;
                    fstate <= F_LOAD_MU_PREFIX0;
                end else if (op_mode == MLDSA_OP_VERIFY) begin
                    pre_poly <= 3'd0;
                    pre_byte_idx <= 8'd0;
                    pre_coeff_idx <= 8'd0;
                    pre_t0_byte_idx <= 4'd0;
                    verify_group_idx <= 10'd0;
                    verify_phase <= 3'd0;
                    verify_h_start <= 8'd0;
                    verify_h_pos <= 8'd0;
                    fstate <= F_VER_MU_PREFIX0;
                end else begin
                    fstate <= F_START_CORE;
                end
            end

            unique case (fstate)
                F_IDLE: begin
                end
                F_LOAD_MU_PREFIX0: begin
                    core_load_ctx_we <= 1'b1;
                    core_load_ctx_sel <= 2'd1;
                    core_load_ctx_addr <= 13'd0;
                    core_load_ctx_data <= 8'd0;
                    fstate <= F_LOAD_MU_PREFIX1;
                end
                F_LOAD_MU_PREFIX1: begin
                    core_load_ctx_we <= 1'b1;
                    core_load_ctx_sel <= 2'd1;
                    core_load_ctx_addr <= 13'd1;
                    core_load_ctx_data <= ctx_len_q[7:0];
                    fstate <= F_PRE_S1_RD;
                end
                F_PRE_S1_RD: begin
                    if (FD_ETA == 2) begin
                        pre_t0_byte_idx <= 4'd0;
                        fstate <= F_PRE_S1_ETA2_RD;
                    end else begin
                        fstate <= F_PRE_S1_LO;
                    end
                end
                F_PRE_S1_LO: begin
                    pre_eta_byte <= sk_rd_data;
                    core_load_poly_we <= 1'b1;
                    core_load_poly_bank <= FD_B_S1_BASE + pre_poly;
                    core_load_poly_addr <= pre_coeff_idx;
                    core_load_poly_data <= decode_eta4_low(sk_rd_data);
                    fstate <= F_PRE_S1_HI;
                end
                F_PRE_S1_HI: begin
                    core_load_poly_we <= 1'b1;
                    core_load_poly_bank <= FD_B_S1_BASE + pre_poly;
                    core_load_poly_addr <= pre_coeff_idx + 8'd1;
                    core_load_poly_data <= decode_eta4_high(pre_eta_byte);
                    if (pre_byte_idx == 8'd127) begin
                        pre_byte_idx <= 8'd0;
                        pre_coeff_idx <= 8'd0;
                        if (pre_poly == FD_L - 1) begin
                            pre_poly <= 3'd0;
                            fstate <= F_PRE_S2_RD;
                        end else begin
                            pre_poly <= pre_poly + 3'd1;
                            fstate <= F_PRE_S1_RD;
                        end
                    end else begin
                        pre_byte_idx <= pre_byte_idx + 8'd1;
                        pre_coeff_idx <= pre_coeff_idx + 8'd2;
                        fstate <= F_PRE_S1_RD;
                    end
                end
                F_PRE_S1_ETA2_RD: begin
                    fstate <= F_PRE_S1_ETA2_CAP;
                end
                F_PRE_S1_ETA2_CAP: begin
                    pre_t0_buf[pre_t0_byte_idx] <= sk_rd_data;
                    if (pre_t0_byte_idx == 4'd2) begin
                        pre_t0_byte_idx <= 4'd0;
                        pre_t0_lane <= 3'd0;
                        fstate <= F_PRE_S1_ETA2_EMIT;
                    end else begin
                        pre_t0_byte_idx <= pre_t0_byte_idx + 4'd1;
                        fstate <= F_PRE_S1_ETA2_RD;
                    end
                end
                F_PRE_S1_ETA2_EMIT: begin
                    core_load_poly_we <= 1'b1;
                    core_load_poly_bank <= FD_B_S1_BASE + pre_poly;
                    core_load_poly_addr <= pre_coeff_idx + {5'd0, pre_t0_lane};
                    core_load_poly_data <= decode_eta2(pre_t0_lane);
                    if (pre_t0_lane == 3'd7) begin
                        if (pre_byte_idx == FD_ETA_BYTES_PER_POLY - 3) begin
                            pre_byte_idx <= 8'd0;
                            pre_coeff_idx <= 8'd0;
                            if (pre_poly == FD_L - 1) begin
                                pre_poly <= 3'd0;
                                fstate <= F_PRE_S2_RD;
                            end else begin
                                pre_poly <= pre_poly + 3'd1;
                                fstate <= F_PRE_S1_RD;
                            end
                        end else begin
                            pre_byte_idx <= pre_byte_idx + 8'd3;
                            pre_coeff_idx <= pre_coeff_idx + 8'd8;
                            fstate <= F_PRE_S1_RD;
                        end
                    end else begin
                        pre_t0_lane <= pre_t0_lane + 3'd1;
                    end
                end
                F_PRE_S2_RD: begin
                    if (FD_ETA == 2) begin
                        pre_t0_byte_idx <= 4'd0;
                        fstate <= F_PRE_S2_ETA2_RD;
                    end else begin
                        fstate <= F_PRE_S2_LO;
                    end
                end
                F_PRE_S2_LO: begin
                    pre_eta_byte <= sk_rd_data;
                    core_load_poly_we <= 1'b1;
                    core_load_poly_bank <= FD_B_S2_BASE + pre_poly;
                    core_load_poly_addr <= pre_coeff_idx;
                    core_load_poly_data <= decode_eta4_low(sk_rd_data);
                    fstate <= F_PRE_S2_HI;
                end
                F_PRE_S2_HI: begin
                    core_load_poly_we <= 1'b1;
                    core_load_poly_bank <= FD_B_S2_BASE + pre_poly;
                    core_load_poly_addr <= pre_coeff_idx + 8'd1;
                    core_load_poly_data <= decode_eta4_high(pre_eta_byte);
                    if (pre_byte_idx == 8'd127) begin
                        pre_byte_idx <= 8'd0;
                        pre_coeff_idx <= 8'd0;
                        if (pre_poly == FD_K - 1) begin
                            pre_poly <= 3'd0;
                            pre_t0_byte_idx <= 4'd0;
                            fstate <= F_PRE_T0_RD;
                        end else begin
                            pre_poly <= pre_poly + 3'd1;
                            fstate <= F_PRE_S2_RD;
                        end
                    end else begin
                        pre_byte_idx <= pre_byte_idx + 8'd1;
                        pre_coeff_idx <= pre_coeff_idx + 8'd2;
                        fstate <= F_PRE_S2_RD;
                    end
                end
                F_PRE_S2_ETA2_RD: begin
                    fstate <= F_PRE_S2_ETA2_CAP;
                end
                F_PRE_S2_ETA2_CAP: begin
                    pre_t0_buf[pre_t0_byte_idx] <= sk_rd_data;
                    if (pre_t0_byte_idx == 4'd2) begin
                        pre_t0_byte_idx <= 4'd0;
                        pre_t0_lane <= 3'd0;
                        fstate <= F_PRE_S2_ETA2_EMIT;
                    end else begin
                        pre_t0_byte_idx <= pre_t0_byte_idx + 4'd1;
                        fstate <= F_PRE_S2_ETA2_RD;
                    end
                end
                F_PRE_S2_ETA2_EMIT: begin
                    core_load_poly_we <= 1'b1;
                    core_load_poly_bank <= FD_B_S2_BASE + pre_poly;
                    core_load_poly_addr <= pre_coeff_idx + {5'd0, pre_t0_lane};
                    core_load_poly_data <= decode_eta2(pre_t0_lane);
                    if (pre_t0_lane == 3'd7) begin
                        if (pre_byte_idx == FD_ETA_BYTES_PER_POLY - 3) begin
                            pre_byte_idx <= 8'd0;
                            pre_coeff_idx <= 8'd0;
                            if (pre_poly == FD_K - 1) begin
                                pre_poly <= 3'd0;
                                pre_t0_byte_idx <= 4'd0;
                                fstate <= F_PRE_T0_RD;
                            end else begin
                                pre_poly <= pre_poly + 3'd1;
                                fstate <= F_PRE_S2_RD;
                            end
                        end else begin
                            pre_byte_idx <= pre_byte_idx + 8'd3;
                            pre_coeff_idx <= pre_coeff_idx + 8'd8;
                            fstate <= F_PRE_S2_RD;
                        end
                    end else begin
                        pre_t0_lane <= pre_t0_lane + 3'd1;
                    end
                end
                F_PRE_T0_RD: begin
                    fstate <= F_PRE_T0_CAP;
                end
                F_PRE_T0_CAP: begin
                    pre_t0_buf[pre_t0_byte_idx] <= sk_rd_data;
                    if (pre_t0_byte_idx == 4'd12) begin
                        pre_t0_byte_idx <= 4'd0;
                        pre_t0_lane <= 3'd0;
                        fstate <= F_PRE_T0_EMIT;
                    end else begin
                        pre_t0_byte_idx <= pre_t0_byte_idx + 4'd1;
                        fstate <= F_PRE_T0_RD;
                    end
                end
                F_PRE_T0_EMIT: begin
                    core_load_poly_we <= 1'b1;
                    core_load_poly_bank <= FD_B_T0_TMP_BASE + pre_poly;
                    core_load_poly_addr <= pre_coeff_idx + {5'd0, pre_t0_lane};
                    core_load_poly_data <= decode_t0_65(pre_t0_lane);
                    if (pre_t0_lane == 3'd7) begin
                        if (pre_byte_idx == 8'd31) begin
                            pre_byte_idx <= 8'd0;
                            pre_coeff_idx <= 8'd0;
                            if (pre_poly == FD_K - 1) begin
                                fstate <= F_START_CORE;
                            end else begin
                                pre_poly <= pre_poly + 3'd1;
                                fstate <= F_PRE_T0_RD;
                            end
                        end else begin
                            pre_byte_idx <= pre_byte_idx + 8'd1;
                            pre_coeff_idx <= pre_coeff_idx + 8'd8;
                            fstate <= F_PRE_T0_RD;
                        end
                    end else begin
                        pre_t0_lane <= pre_t0_lane + 3'd1;
                    end
                end
                F_VER_MU_PREFIX0: begin
                    core_load_ctx_we <= 1'b1;
                    core_load_ctx_sel <= 2'd1;
                    core_load_ctx_addr <= 13'd0;
                    core_load_ctx_data <= 8'd0;
                    fstate <= F_VER_MU_PREFIX1;
                end
                F_VER_MU_PREFIX1: begin
                    core_load_ctx_we <= 1'b1;
                    core_load_ctx_sel <= 2'd1;
                    core_load_ctx_addr <= 13'd1;
                    core_load_ctx_data <= ctx_len_q[7:0];
                    pre_byte_idx <= 8'd0;
                    fstate <= F_VER_RHO_RD;
                end
                F_VER_RHO_RD: begin
                    fstate <= F_VER_RHO_CAP;
                end
                F_VER_RHO_CAP: begin
                    core_load_expanded_seed_we <= 1'b1;
                    core_load_expanded_seed_sel <= 2'd0;
                    core_load_expanded_seed_addr <= {1'b0, pre_byte_idx[4:0]};
                    core_load_expanded_seed_data <= pk_rd_data;
                    if (pre_byte_idx == 8'd31) begin
                        pre_byte_idx <= 8'd0;
                        fstate <= F_VER_C_RD;
                    end else begin
                        pre_byte_idx <= pre_byte_idx + 8'd1;
                        fstate <= F_VER_RHO_RD;
                    end
                end
                F_VER_C_RD: begin
                    fstate <= F_VER_C_CAP;
                end
                F_VER_C_CAP: begin
                    core_load_sig_hash_we <= 1'b1;
                    core_load_sig_hash_addr <= pre_byte_idx[5:0];
                    core_load_sig_hash_data <= sig_rd_data;
                    if (pre_byte_idx == FD_CTILDE_BYTES - 1) begin
                        pre_poly <= 3'd0;
                        pre_coeff_idx <= 8'd0;
                        fstate <= F_VER_HINT_CLEAR;
                    end else begin
                        pre_byte_idx <= pre_byte_idx + 8'd1;
                        fstate <= F_VER_C_RD;
                    end
                end
                F_VER_HINT_CLEAR: begin
                    core_load_poly_we <= 1'b1;
                    core_load_poly_bank <= FD_B_HINT_BASE + pre_poly;
                    core_load_poly_addr <= pre_coeff_idx;
                    core_load_poly_data <= 24'd0;
                    if (pre_coeff_idx == 8'd255) begin
                        pre_coeff_idx <= 8'd0;
                        if (pre_poly == FD_K - 1) begin
                            verify_group_idx <= 10'd0;
                            verify_phase <= 3'd0;
                            fstate <= F_VER_Z_RD;
                        end else begin
                            pre_poly <= pre_poly + 3'd1;
                        end
                    end else begin
                        pre_coeff_idx <= pre_coeff_idx + 8'd1;
                    end
                end
                F_VER_Z_RD: begin
                    fstate <= F_VER_Z_CAP;
                end
                F_VER_Z_CAP: begin
                    pre_t0_buf[verify_phase] <= sig_rd_data;
                    if (verify_phase == FD_Z_GROUP_BYTES - 1) begin
                        verify_phase <= 3'd0;
                        fstate <= F_VER_Z_WR0;
                    end else begin
                        verify_phase <= verify_phase + 3'd1;
                        fstate <= F_VER_Z_RD;
                    end
                end
                F_VER_Z_WR0: begin
                    core_load_poly_we <= 1'b1;
                    core_load_poly_bank <= FD_B_Z_BASE + ((FD_Z_BITS == 18) ? {1'b0, verify_group_idx[8:6]} : verify_group_idx[9:7]);
                    core_load_poly_addr <= (FD_Z_BITS == 18) ? {verify_group_idx[5:0], 2'd0} : {verify_group_idx[6:0], 1'b0};
                    core_load_poly_data <= (FD_Z_BITS == 18) ? decode_z_modq({2'd0, sig_z18_0_from_buf()}) : decode_z_modq(sig_z0_from_buf());
                    fstate <= F_VER_Z_WR1;
                end
                F_VER_Z_WR1: begin
                    core_load_poly_we <= 1'b1;
                    core_load_poly_bank <= FD_B_Z_BASE + ((FD_Z_BITS == 18) ? {1'b0, verify_group_idx[8:6]} : verify_group_idx[9:7]);
                    core_load_poly_addr <= (FD_Z_BITS == 18) ? {verify_group_idx[5:0], 2'd1} : {verify_group_idx[6:0], 1'b1};
                    core_load_poly_data <= (FD_Z_BITS == 18) ? decode_z_modq({2'd0, sig_z18_1_from_buf()}) : decode_z_modq(sig_z1_from_buf());
                    if (FD_Z_BITS == 18) begin
                        fstate <= F_VER_Z_WR2;
                    end else if (verify_group_idx == FD_Z_GROUP_COUNT - 1) begin
                        verify_group_idx <= 10'd0;
                        verify_phase <= 3'd0;
                        fstate <= F_VER_T1_RD;
                    end else begin
                        verify_group_idx <= verify_group_idx + 10'd1;
                        fstate <= F_VER_Z_RD;
                    end
                end
                F_VER_Z_WR2: begin
                    core_load_poly_we <= 1'b1;
                    core_load_poly_bank <= FD_B_Z_BASE + {1'b0, verify_group_idx[8:6]};
                    core_load_poly_addr <= {verify_group_idx[5:0], 2'd2};
                    core_load_poly_data <= decode_z_modq({2'd0, sig_z18_2_from_buf()});
                    fstate <= F_VER_Z_WR3;
                end
                F_VER_Z_WR3: begin
                    core_load_poly_we <= 1'b1;
                    core_load_poly_bank <= FD_B_Z_BASE + {1'b0, verify_group_idx[8:6]};
                    core_load_poly_addr <= {verify_group_idx[5:0], 2'd3};
                    core_load_poly_data <= decode_z_modq({2'd0, sig_z18_3_from_buf()});
                    if (verify_group_idx == FD_Z_GROUP_COUNT - 1) begin
                        verify_group_idx <= 10'd0;
                        verify_phase <= 3'd0;
                        fstate <= F_VER_T1_RD;
                    end else begin
                        verify_group_idx <= verify_group_idx + 10'd1;
                        fstate <= F_VER_Z_RD;
                    end
                end
                F_VER_T1_RD: begin
                    fstate <= F_VER_T1_CAP;
                end
                F_VER_T1_CAP: begin
                    pre_t0_buf[verify_phase] <= pk_rd_data;
                    if (verify_phase == 3'd4) begin
                        verify_phase <= 3'd0;
                        fstate <= F_VER_T1_WR0;
                    end else begin
                        verify_phase <= verify_phase + 3'd1;
                        fstate <= F_VER_T1_RD;
                    end
                end
                F_VER_T1_WR0: begin
                    core_load_poly_we <= 1'b1;
                    core_load_poly_bank <= FD_B_T1N_BASE + verify_group_idx[8:6];
                    core_load_poly_addr <= {verify_group_idx[5:0], 2'd0};
                    core_load_poly_data <= {14'd0, pk_t1_0_from_buf()};
                    fstate <= F_VER_T1_WR1;
                end
                F_VER_T1_WR1: begin
                    core_load_poly_we <= 1'b1;
                    core_load_poly_bank <= FD_B_T1N_BASE + verify_group_idx[8:6];
                    core_load_poly_addr <= {verify_group_idx[5:0], 2'd1};
                    core_load_poly_data <= {14'd0, pk_t1_1_from_buf()};
                    fstate <= F_VER_T1_WR2;
                end
                F_VER_T1_WR2: begin
                    core_load_poly_we <= 1'b1;
                    core_load_poly_bank <= FD_B_T1N_BASE + verify_group_idx[8:6];
                    core_load_poly_addr <= {verify_group_idx[5:0], 2'd2};
                    core_load_poly_data <= {14'd0, pk_t1_2_from_buf()};
                    fstate <= F_VER_T1_WR3;
                end
                F_VER_T1_WR3: begin
                    core_load_poly_we <= 1'b1;
                    core_load_poly_bank <= FD_B_T1N_BASE + verify_group_idx[8:6];
                    core_load_poly_addr <= {verify_group_idx[5:0], 2'd3};
                    core_load_poly_data <= {14'd0, pk_t1_3_from_buf()};
                    if (verify_group_idx == (FD_K * 64) - 1) begin
                        pre_poly <= 3'd0;
                        verify_h_start <= 8'd0;
                        fstate <= F_VER_H_MARK_RD;
                    end else begin
                        verify_group_idx <= verify_group_idx + 10'd1;
                        fstate <= F_VER_T1_RD;
                    end
                end
                F_VER_H_MARK_RD: begin
                    fstate <= F_VER_H_MARK_CAP;
                end
                F_VER_H_MARK_CAP: begin
                    verify_h_end <= sig_rd_data;
                    if (sig_rd_data == verify_h_start) begin
                        if (pre_poly == FD_K - 1) begin
                            verify_h_pos <= verify_h_start;
                            fstate <= F_VER_H_UNUSED_RD;
                        end else begin
                            pre_poly <= pre_poly + 3'd1;
                            fstate <= F_VER_H_MARK_RD;
                        end
                    end else begin
                        verify_h_pos <= verify_h_start;
                        fstate <= F_VER_H_POS_RD;
                    end
                end
                F_VER_H_POS_RD: begin
                    fstate <= F_VER_H_POS_CAP;
                end
                F_VER_H_POS_CAP: begin
                    core_load_poly_we <= 1'b1;
                    core_load_poly_bank <= FD_B_HINT_BASE + pre_poly;
                    core_load_poly_addr <= sig_rd_data;
                    core_load_poly_data <= 24'd1;
                    if (verify_h_pos + 8'd1 == verify_h_end) begin
                        verify_h_start <= verify_h_end;
                        if (pre_poly == FD_K - 1) begin
                            verify_h_pos <= verify_h_end;
                            fstate <= F_VER_H_UNUSED_RD;
                        end else begin
                            pre_poly <= pre_poly + 3'd1;
                            fstate <= F_VER_H_MARK_RD;
                        end
                    end else begin
                        verify_h_pos <= verify_h_pos + 8'd1;
                        fstate <= F_VER_H_POS_RD;
                    end
                end
                F_VER_H_UNUSED_RD: begin
                    fstate <= F_VER_H_UNUSED_CAP;
                end
                F_VER_H_UNUSED_CAP: begin
                    if (verify_h_pos >= FD_OMEGA) begin
                        fstate <= F_START_CORE;
                    end else if (sig_rd_data != 8'd0) begin
                        done <= 1'b1;
                        pass <= 1'b0;
                        error <= 1'b1;
                        err_code <= 8'h15;
                        fstate <= F_IDLE;
                    end else begin
                        verify_h_pos <= verify_h_pos + 8'd1;
                        fstate <= F_VER_H_UNUSED_RD;
                    end
                end
                F_START_CORE: begin
                    core_start <= 1'b1;
                    fstate <= F_WAIT_CORE;
                end
                F_WAIT_CORE: begin
                    if (core_done) begin
                        pass <= core_pass;
                        error <= core_error;
                        err_code <= core_err_code;
                        if (core_pass && !core_error && core_op_mode == MLDSA_OP_SIGN) begin
                            pack_sig_addr <= 13'd0;
                            core_sig_hash_rd_addr <= 6'd0;
                            fstate <= F_PACK_C_RD;
                        end else begin
                            fstate <= F_REPORT_DONE;
                        end
                    end
                end
                F_PACK_C_RD: begin
                    core_sig_hash_rd_addr <= pack_sig_addr[5:0];
                    fstate <= F_PACK_C_WAIT;
                end
                F_PACK_C_WAIT: begin
                    fstate <= F_PACK_C_CAP;
                end
                F_PACK_C_CAP: begin
                    sig_store_wr_en <= 1'b1;
                    sig_store_wr_addr <= pack_sig_addr;
                    sig_store_wr_data <= core_sig_hash_rd_data;
`ifdef MLDSA_DEBUG_DISPLAY
                    if (pack_sig_addr < 13'd8) begin
                        $display("FRONT_PACK_C addr=%0d data=%02x", pack_sig_addr, core_sig_hash_rd_data);
                    end
`endif
                    if (pack_sig_addr == FD_CTILDE_BYTES - 1) begin
                        pack_sig_addr <= FD_CTILDE_BYTES;
                        pack_poly <= 3'd0;
                        pack_coeff <= 8'd0;
                        fstate <= F_PACK_Z_RD0;
                    end else begin
                        pack_sig_addr <= pack_sig_addr + 13'd1;
                        fstate <= F_PACK_C_RD;
                    end
                end
                F_PACK_Z_RD0: begin
                    core_host_rd_en <= 1'b1;
                    core_host_rd_bank <= FD_B_Z_BASE + pack_poly;
                    core_host_rd_addr <= pack_coeff;
                    fstate <= F_PACK_Z_WAIT0;
                end
                F_PACK_Z_WAIT0: begin
                    fstate <= F_PACK_Z_CAP0;
                end
                F_PACK_Z_CAP0: begin
                    z_coeff0 <= core_host_rd_data;
                    core_host_rd_en <= 1'b1;
                    core_host_rd_bank <= FD_B_Z_BASE + pack_poly;
                    core_host_rd_addr <= pack_coeff + 8'd1;
                    fstate <= F_PACK_Z_WAIT1;
                end
                F_PACK_Z_WAIT1: begin
                    fstate <= F_PACK_Z_CAP1;
                end
                F_PACK_Z_CAP1: begin
                    z_coeff1 <= core_host_rd_data;
                    if (FD_Z_BITS == 18) begin
                        core_host_rd_en <= 1'b1;
                        core_host_rd_bank <= FD_B_Z_BASE + pack_poly;
                        core_host_rd_addr <= pack_coeff + 8'd2;
                        fstate <= F_PACK_Z_RD2;
                    end else begin
                        pack_phase <= 4'd0;
                        fstate <= F_PACK_Z_WR;
                    end
                end
                F_PACK_Z_RD2: begin
                    fstate <= F_PACK_Z_WAIT2;
                end
                F_PACK_Z_WAIT2: begin
                    fstate <= F_PACK_Z_CAP2;
                end
                F_PACK_Z_CAP2: begin
                    z_coeff2 <= core_host_rd_data;
                    core_host_rd_en <= 1'b1;
                    core_host_rd_bank <= FD_B_Z_BASE + pack_poly;
                    core_host_rd_addr <= pack_coeff + 8'd3;
                    fstate <= F_PACK_Z_WAIT3;
                end
                F_PACK_Z_WAIT3: begin
                    fstate <= F_PACK_Z_CAP3;
                end
                F_PACK_Z_CAP3: begin
                    z_coeff3 <= core_host_rd_data;
                    pack_phase <= 4'd0;
                    fstate <= F_PACK_Z_WR;
                end
                F_PACK_Z_WR: begin
                    sig_store_wr_en <= 1'b1;
                    sig_store_wr_addr <= pack_sig_addr;
                    unique case (pack_phase)
                        4'd0: sig_store_wr_data <= z_enc0[7:0];
                        4'd1: sig_store_wr_data <= z_enc0[15:8];
                        4'd2: sig_store_wr_data <= (FD_Z_BITS == 18) ? {z_enc1[5:0], z_enc0[17:16]} : {z_enc1[3:0], z_enc0[19:16]};
                        4'd3: sig_store_wr_data <= (FD_Z_BITS == 18) ? z_enc1[13:6] : z_enc1[11:4];
                        4'd4: sig_store_wr_data <= (FD_Z_BITS == 18) ? {z_enc2[3:0], z_enc1[17:14]} : z_enc1[19:12];
                        4'd5: sig_store_wr_data <= z_enc2[11:4];
                        4'd6: sig_store_wr_data <= {z_enc3[1:0], z_enc2[17:12]};
                        4'd7: sig_store_wr_data <= z_enc3[9:2];
                        default: sig_store_wr_data <= z_enc3[17:10];
                    endcase
`ifdef MLDSA_DEBUG_DISPLAY
                    if (pack_poly == 3'd0 && pack_coeff < 8'd4) begin
                        $display("FRONT_PACK_Z poly=%0d coeff=%0d phase=%0d addr=%0d data=%02x z0=%0d z1=%0d",
                                 pack_poly, pack_coeff, pack_phase, pack_sig_addr, sig_store_wr_data, z_coeff0, z_coeff1);
                    end
`endif
                    pack_sig_addr <= pack_sig_addr + 13'd1;
                    if (pack_phase == FD_Z_GROUP_BYTES - 1) begin
                        pack_phase <= 4'd0;
                        if (pack_coeff == ((FD_Z_BITS == 18) ? 8'd252 : 8'd254)) begin
                            pack_coeff <= 8'd0;
                            if (pack_poly == FD_L - 1) begin
                                hint_count <= 8'd0;
                                pack_poly <= 3'd0;
                                pack_coeff <= 8'd0;
                                fstate <= F_PACK_H_CLEAR;
                            end else begin
                                pack_poly <= pack_poly + 3'd1;
                                fstate <= F_PACK_Z_RD0;
                            end
                        end else begin
                            pack_coeff <= pack_coeff + ((FD_Z_BITS == 18) ? 8'd4 : 8'd2);
                            fstate <= F_PACK_Z_RD0;
                        end
                    end else begin
                        pack_phase <= pack_phase + 4'd1;
                    end
                end
                F_PACK_H_CLEAR: begin
                    sig_store_wr_en <= 1'b1;
                    sig_store_wr_addr <= FD_CTILDE_BYTES + FD_Z_BYTES + pack_coeff;
                    sig_store_wr_data <= 8'd0;
                    if (pack_coeff == FD_OMEGA + FD_K - 1) begin
                        pack_coeff <= 8'd0;
                        pack_poly <= 3'd0;
                        fstate <= F_PACK_H_RD;
                    end else begin
                        pack_coeff <= pack_coeff + 8'd1;
                    end
                end
                F_PACK_H_RD: begin
                    core_host_rd_en <= 1'b1;
                    core_host_rd_bank <= FD_B_HINT_RESULT_BASE + pack_poly;
                    core_host_rd_addr <= pack_coeff;
                    fstate <= F_PACK_H_WAIT;
                end
                F_PACK_H_WAIT: begin
                    fstate <= F_PACK_H_CAP;
                end
                F_PACK_H_CAP: begin
                    if (core_host_rd_data[0]) begin
                        sig_store_wr_en <= 1'b1;
                        sig_store_wr_addr <= FD_CTILDE_BYTES + FD_Z_BYTES + hint_count;
                        sig_store_wr_data <= pack_coeff;
`ifdef MLDSA_DEBUG_DISPLAY
                        if (hint_count < 8'd80) begin
                            $display("FRONT_PACK_H poly=%0d coeff=%0d out_idx=%0d data=%0d",
                                     pack_poly, pack_coeff, hint_count, core_host_rd_data);
                        end
`endif
                        hint_count <= hint_count + 8'd1;
                    end
                    if (pack_coeff == 8'd255) begin
                        fstate <= F_PACK_H_MARK;
                    end else begin
                        pack_coeff <= pack_coeff + 8'd1;
                        fstate <= F_PACK_H_RD;
                    end
                end
                F_PACK_H_MARK: begin
                    sig_store_wr_en <= 1'b1;
                    sig_store_wr_addr <= FD_CTILDE_BYTES + FD_Z_BYTES + FD_OMEGA + pack_poly;
                    sig_store_wr_data <= hint_count;
`ifdef MLDSA_DEBUG_DISPLAY
                    $display("FRONT_PACK_H_MARK poly=%0d cumulative=%0d", pack_poly, hint_count);
`endif
                    if (pack_poly == FD_K - 1) begin
                        fstate <= F_REPORT_DONE;
                    end else begin
                        pack_poly <= pack_poly + 3'd1;
                        pack_coeff <= 8'd0;
                        fstate <= F_PACK_H_RD;
                    end
                end
                F_REPORT_DONE: begin
                    done <= 1'b1;
                    fstate <= F_IDLE;
                end
                default: fstate <= F_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
