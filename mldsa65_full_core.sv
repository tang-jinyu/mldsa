`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

module mldsa65_full_core #(
    parameter int BANKS = 65
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         load_poly_we,
    input  wire [$clog2(BANKS)-1:0]     load_poly_bank,
    input  wire [7:0]                   load_poly_addr,
    input  wire [MLDSA_COEFF_W-1:0]     load_poly_data,
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
    output logic                        busy,
    output logic                        done,
    output logic                        pass,
    output logic                        error,
    output logic [7:0]                  err_code,
    output logic [7:0]                  uop_dbg,
    output logic [7:0]                  state_dbg,
    output wire [MLDSA_COEFF_W-1:0]     host_rd_data
);

    localparam int W1_BYTES_PER_POLY   = MLDSA_N * 3;
    localparam int HINT_BYTES_PER_POLY = MLDSA_N / 8;
    localparam int W1_PACK_BYTES_MAX   = MLDSA_K_MAX * W1_BYTES_PER_POLY;
    localparam int HINT_PACK_BYTES_MAX = MLDSA_K_MAX * HINT_BYTES_PER_POLY;
    localparam int HASH_BUF_BYTES      = 4096;
    localparam int W1_MEM_ADDR_W       = (W1_PACK_BYTES_MAX <= 1) ? 1 : $clog2(W1_PACK_BYTES_MAX);
    localparam int HASH_MEM_ADDR_W     = (HASH_BUF_BYTES <= 1) ? 1 : $clog2(HASH_BUF_BYTES);
    localparam int Z_TOTAL_COEFFS_MAX  = MLDSA_L_MAX * MLDSA_N;
    localparam logic [7:0] SIGN_RETRY_MAX = 8'hff;

    // Two layouts are supported:
    // 1) Resident-A prototype layout (BANKS >= 120) keeps the full A matrix in
    //    poly RAM for legacy bring-up benches.
    // 2) Compact standard layout (BANKS < 120) stores only mutable polys and
    //    regenerates A(i,j) into a single temporary bank on demand.
    localparam bit USE_RESIDENT_A = (BANKS >= 120);
    localparam int A_STORAGE_BANKS = MLDSA_K_MAX * MLDSA_L_MAX;

    // Compact 40nm-oriented bank map:
    // - S2N reuses T1N
    // - Z reuses Y
    // - C_VERIFY reuses Y[0] (verify mode never uses Y)
    // - Resident A banks disappear in compact mode; one temporary A bank is
    //   filled by ExpandA immediately before each MAC.
    localparam int B_A_BASE    = 0;
    localparam int B_S1_BASE   = USE_RESIDENT_A ? (B_A_BASE + A_STORAGE_BANKS) : 0;
    localparam int B_S1N_BASE  = B_S1_BASE + MLDSA_L_MAX;
    localparam int B_T1N_BASE  = B_S1N_BASE + MLDSA_L_MAX;
    localparam int B_Y_BASE    = B_T1N_BASE + MLDSA_K_MAX;
    localparam int B_ACC_BASE  = B_Y_BASE + MLDSA_L_MAX;
    localparam int B_W_BASE    = B_ACC_BASE + MLDSA_K_MAX;
    localparam int B_W1_BASE   = B_W_BASE + MLDSA_K_MAX;
    localparam int B_C_BASE    = B_W1_BASE + MLDSA_K_MAX;
    localparam int B_CN_BASE   = B_C_BASE + 1;
    localparam int B_TMP_BASE  = B_CN_BASE + 1;
    localparam int B_Z_BASE    = B_Y_BASE;
    localparam int B_HINT_BASE = B_TMP_BASE + 1;
    localparam int B_A_TMP_BASE = B_HINT_BASE + MLDSA_K_MAX;
    localparam int B_CV_BASE   = B_Y_BASE;
    localparam int BANKS_MIN   = USE_RESIDENT_A ? (B_HINT_BASE + MLDSA_K_MAX)
                                                : (B_A_TMP_BASE + 1);

    initial begin
        if (BANKS < BANKS_MIN) begin
            $error("mldsa65_full_core requires BANKS >= %0d, got %0d", BANKS_MIN, BANKS);
        end
    end

    typedef enum logic [7:0] {
        S_WAIT_UOP,
        S_SEED_START,
        S_SEED_WAIT,
        S_EXPANDA_RHO_LOAD,
        S_EXPANDA_START,
        S_EXPANDA_FILL,
        S_T1_A_START,
        S_T1_A_FILL,
        S_STD_HASH_LOAD_A,
        S_STD_HASH_LOAD_B,
        S_STD_HASH_START,
        S_STD_HASH_WAIT,
        S_STD_HASH_COPY,
        S_T1_CLR,
        S_T1_NTT_START,
        S_T1_NTT_WAIT,
        S_T1_MAC_START,
        S_T1_MAC_WAIT,
        S_Y_XOF_START,
        S_Y_ABSORB,
        S_Y_SAMPLER_START,
        S_Y_SQUEEZE,
        S_Y_STOP,
        S_MV_CLR,
        S_MV_NTT_START,
        S_MV_NTT_WAIT,
        S_MV_A_START,
        S_MV_A_FILL,
        S_MV_MAC_START,
        S_MV_MAC_WAIT,
        S_MV_INV_START,
        S_MV_INV_WAIT,
        S_W1_DEC_READ,
        S_W1_DEC_START,
        S_W1_DEC_WAIT,
        S_W1_DEC_WRITE,
        S_PACKW_START,
        S_PACKW_READ,
        S_PACKW_PUSH,
        S_PACKW_WAIT,
        S_HASH_START,
        S_HASH_ABSORB,
        S_HASH_SQUEEZE,
        S_HASH_STOP,
        S_CHAL_START,
        S_CHAL_FEED,
        S_CHAL_NTT_START,
        S_CHAL_NTT_WAIT,
        S_Z_MUL_START,
        S_Z_MUL_WAIT,
        S_Z_INV_START,
        S_Z_INV_WAIT,
        S_Z_ADD_START,
        S_Z_ADD_WAIT,
        S_ZCHK_START,
        S_ZCHK_RDWAIT,
        S_ZCHK_FEED,
        S_ZCHK_WAIT,
        S_HINT_GEN_START,
        S_HINT_CS2_MUL_START,
        S_HINT_CS2_MUL_WAIT,
        S_HINT_CS2_INV_START,
        S_HINT_CS2_INV_WAIT,
        S_HINT_WMINUS_START,
        S_HINT_WMINUS_WAIT,
        S_HINT_CT0_MUL_START,
        S_HINT_CT0_MUL_WAIT,
        S_HINT_CT0_INV_START,
        S_HINT_CT0_INV_WAIT,
        S_HINT_GEN_READ,
        S_HINT_GEN_WAIT,
        S_HINT_GEN_WRITE,
        S_HINTCNT_START,
        S_HINTCNT_RDWAIT,
        S_HINTCNT_FEED,
        S_HINTCNT_WAIT,
        S_PACKH_START,
        S_PACKH_READ,
        S_PACKH_PUSH,
        S_PACKH_WAIT,
        S_UNPACKH_START,
        S_UNPACKH_FEED,
        S_UNPACKH_WAIT,
        S_USEH_READ,
        S_USEH_START,
        S_USEH_WAIT,
        S_USEH_WRITE,
        S_CMPC_START,
        S_CMPC_RDWAIT,
        S_CMPC_FEED,
        S_CMPC_WAIT,
        S_UOP_DONE,
        S_UOP_FAIL
    } state_e;

    state_e st;
    logic [7:0] seed_mem [0:31];
    logic [7:0] tr_rd_data;
    logic [7:0] message_rd_data;
    logic [7:0] mu_rd_data;
    logic       mu_wr_en;
    logic [5:0] mu_wr_addr;
    logic [7:0] mu_wr_data;
    logic [7:0] hint_bytes [0:HINT_PACK_BYTES_MAX-1];
    logic [7:0] sign_w1_rd_data;
    logic [7:0] verify_w1_rd_data;
    logic [7:0] hash_buf_rd_data;
    logic [7:0] sign_hash_buf_rd_data_unused;
    logic sign_w1_wr_en;
    logic [W1_MEM_ADDR_W-1:0] sign_w1_wr_addr;
    logic [7:0] sign_w1_wr_data;
    logic [W1_MEM_ADDR_W-1:0] sign_w1_rd_addr;
    logic verify_w1_wr_en;
    logic [W1_MEM_ADDR_W-1:0] verify_w1_wr_addr;
    logic [7:0] verify_w1_wr_data;
    logic [W1_MEM_ADDR_W-1:0] verify_w1_rd_addr;
    logic hash_buf_wr_en;
    logic [HASH_MEM_ADDR_W-1:0] hash_buf_wr_addr;
    logic [7:0] hash_buf_wr_data;
    logic [HASH_MEM_ADDR_W-1:0] hash_buf_rd_addr;
    logic sign_hash_buf_wr_en;
    logic [HASH_MEM_ADDR_W-1:0] sign_hash_buf_wr_addr;
    logic [7:0] sign_hash_buf_wr_data;

    logic [3:0] cfg_k_q;
    logic [3:0] cfg_l_q;
    logic [2:0] cfg_eta_q;
    logic [5:0] cfg_tau_q;
    logic [19:0] cfg_gamma1_q;
    logic [19:0] cfg_gamma2_q;
    logic [8:0] cfg_beta_q;
    logic [7:0] cfg_omega_q;
    logic [2:0] cfg_k_last_q;
    logic [2:0] cfg_l_last_q;
    logic [12:0] cfg_w1_pack_bytes_q;
    logic [12:0] cfg_hint_pack_bytes_q;
    logic [15:0] cfg_z_total_coeffs_q;
    logic [MLDSA_COEFF_W-1:0] cfg_z_bound_q;
    logic cfg_gamma1_is_2p19_q;

    logic [3:0] level_k;
    logic [3:0] level_l;
    logic [2:0] level_eta;
    logic [5:0] level_tau;
    logic [19:0] level_gamma1;
    logic [19:0] level_gamma2;
    logic [8:0] level_beta;
    logic [7:0] level_omega;
    logic [15:0] level_pk_bytes;
    logic [15:0] level_sk_bytes;
    logic [15:0] level_sig_bytes;
    logic level_valid;

    logic ucode_busy;
    logic ucode_done;
    logic ucode_error;
    logic [7:0] ucode_pc;
    logic [7:0] uop_code;
    logic uop_step_done;
    logic uop_step_retry;
    logic uop_step_fail;

    logic ram_rd0_en;
    logic [$clog2(BANKS)-1:0] ram_rd0_bank;
    logic [7:0] ram_rd0_addr;
    logic [MLDSA_COEFF_W-1:0] ram_rd0_data;
    logic [MLDSA_COEFF_W-1:0] ram_rd0_data_pipe;
    logic                     ram_rd0_valid;
    logic ram_pair_rd0_en;
    logic [$clog2(BANKS)-1:0] ram_pair_rd0_bank;
    logic [6:0] ram_pair_rd0_addr;
    logic [47:0] ram_pair_rd0_data;
    logic [47:0] ram_pair_rd0_data_pipe;
    logic        ram_pair_rd0_valid;
    logic ram_rd1_en;
    logic [$clog2(BANKS)-1:0] ram_rd1_bank;
    logic [7:0] ram_rd1_addr;
    logic [MLDSA_COEFF_W-1:0] ram_rd1_data;
    logic [MLDSA_COEFF_W-1:0] ram_rd1_data_pipe;
    logic                     ram_rd1_valid;
    logic ram_pair_rd1_en;
    logic [$clog2(BANKS)-1:0] ram_pair_rd1_bank;
    logic [6:0] ram_pair_rd1_addr;
    logic [47:0] ram_pair_rd1_data;
    logic [47:0] ram_pair_rd1_data_pipe;
    logic        ram_pair_rd1_valid;
    logic ram_wr_en;
    logic [$clog2(BANKS)-1:0] ram_wr_bank;
    logic [7:0] ram_wr_addr;
    logic [MLDSA_COEFF_W-1:0] ram_wr_data;
    logic ram_pair_wr_en;
    logic [$clog2(BANKS)-1:0] ram_pair_wr_bank;
    logic [6:0] ram_pair_wr_addr;
    logic [47:0] ram_pair_wr_data;

    logic ctrl_rd0_en;
    logic [$clog2(BANKS)-1:0] ctrl_rd0_bank;
    logic [7:0] ctrl_rd0_addr;
    logic ctrl_rd1_en;
    logic [$clog2(BANKS)-1:0] ctrl_rd1_bank;
    logic [7:0] ctrl_rd1_addr;
    logic ctrl_wr_en;
    logic [$clog2(BANKS)-1:0] ctrl_wr_bank;
    logic [7:0] ctrl_wr_addr;
    logic [MLDSA_COEFF_W-1:0] ctrl_wr_data;

    logic ntt_start;
    logic ntt_inverse;
    logic [$clog2(BANKS)-1:0] ntt_src_bank;
    logic [$clog2(BANKS)-1:0] ntt_dst_bank;
    logic ntt_rd_en;
    logic [$clog2(BANKS)-1:0] ntt_rd_bank;
    logic [7:0] ntt_rd_addr;
    logic ntt_wr_en;
    logic [$clog2(BANKS)-1:0] ntt_wr_bank;
    logic [7:0] ntt_wr_addr;
    logic [MLDSA_COEFF_W-1:0] ntt_wr_data;
    logic ntt_busy;
    logic ntt_done;

    logic poly_start;
    logic [2:0] poly_op;
    logic [$clog2(BANKS)-1:0] poly_src0_bank;
    logic [$clog2(BANKS)-1:0] poly_src1_bank;
    logic [$clog2(BANKS)-1:0] poly_dst_bank;
    logic poly_rd0_en;
    logic [$clog2(BANKS)-1:0] poly_rd0_bank;
    logic [7:0] poly_rd0_addr;
    logic poly_pair_rd0_en;
    logic [$clog2(BANKS)-1:0] poly_pair_rd0_bank;
    logic [6:0] poly_pair_rd0_addr;
    logic poly_rd1_en;
    logic [$clog2(BANKS)-1:0] poly_rd1_bank;
    logic [7:0] poly_rd1_addr;
    logic poly_pair_rd1_en;
    logic [$clog2(BANKS)-1:0] poly_pair_rd1_bank;
    logic [6:0] poly_pair_rd1_addr;
    logic poly_wr_en;
    logic [$clog2(BANKS)-1:0] poly_wr_bank;
    logic [7:0] poly_wr_addr;
    logic [MLDSA_COEFF_W-1:0] poly_wr_data;
    logic poly_pair_wr_en;
    logic [$clog2(BANKS)-1:0] poly_pair_wr_bank;
    logic [6:0] poly_pair_wr_addr;
    logic [47:0] poly_pair_wr_data;
    logic poly_busy;
    logic poly_done;

    logic dh_start;
    logic [1:0] dh_op;
    logic [MLDSA_COEFF_W-1:0] dh_coeff_in;
    logic [MLDSA_COEFF_W-1:0] dh_aux_in;
    logic [19:0] dh_gamma2;
    logic [MLDSA_COEFF_W-1:0] dh_high_out;
    logic [MLDSA_COEFF_W-1:0] dh_low_out;
    logic dh_hint_out;
    logic dh_done;

    logic check_start;
    logic [1:0] check_mode;
    logic [15:0] check_item_count;
    logic [MLDSA_COEFF_W-1:0] check_limit;
    logic [MLDSA_COEFF_W-1:0] check_data_a;
    logic [MLDSA_COEFF_W-1:0] check_data_b;
    logic check_data_valid;
    logic check_data_ready;
    logic check_pass;
    logic [15:0] check_fail_index;
    logic [15:0] check_accum_value;
    logic check_done;
    logic [MLDSA_COEFF_W-1:0] check_pipe_a_q;
    logic [MLDSA_COEFF_W-1:0] check_pipe_b_q;
    logic                     check_pipe_valid_q;
    logic                     check_pipe_issue_q;
    logic                     check_pipe_dual_q;
    logic                     check_pipe_issue;

    logic pack_start;
    logic [2:0] pack_mode;
    logic [15:0] pack_item_count;
    logic [MLDSA_COEFF_W-1:0] pack_coeff_in;
    logic pack_coeff_in_valid;
    logic pack_coeff_in_ready;
    logic [7:0] pack_byte_out;
    logic pack_byte_out_valid;
    logic pack_byte_out_ready;
    logic [7:0] pack_byte_in;
    logic pack_byte_in_valid;
    logic pack_byte_in_ready;
    logic [MLDSA_COEFF_W-1:0] pack_coeff_out;
    logic pack_coeff_out_valid;
    logic pack_coeff_out_ready;
    logic pack_done;
    logic pack_error;

    logic xof_start;
    logic xof_stop;
    logic [7:0] xof_abs_byte_data;
    logic xof_abs_byte_valid;
    logic xof_abs_byte_last;
    logic xof_abs_ready;
    logic [7:0] xof_sq_byte;
    logic xof_sq_valid;
    logic xof_sq_byte_ready;
    logic xof_busy;
    logic shared_xof_owner_local;
    logic shared_xof_owner_seed;
    logic shared_xof_owner_expanda;
    logic shared_xof_owner_std_hash;
    logic shared_xof_start_mux;
    logic shared_xof_stop_mux;
    logic shared_xof_mode_shake256_mux;
    logic [7:0] shared_xof_abs_byte_data_mux;
    logic shared_xof_abs_byte_valid_mux;
    logic shared_xof_abs_byte_last_mux;
    logic shared_xof_abs_ready_mux;
    logic [7:0] shared_xof_sq_byte_mux;
    logic shared_xof_sq_valid_mux;
    logic shared_xof_sq_byte_ready_mux;
    logic shared_xof_state_squeezing_mux;

    logic sampler_start;
    logic [1:0] sampler_mode;
    logic [2:0] sampler_eta;
    logic sampler_gamma1_2p19;
    logic [5:0] sampler_tau;
    logic [7:0] sampler_byte_data;
    logic sampler_byte_valid;
    logic sampler_byte_ready;
    logic [7:0] sampler_coeff_index;
    logic [MLDSA_COEFF_W-1:0] sampler_coeff_data;
    logic sampler_coeff_valid;
    logic sampler_done;
    logic sampler_error;

    logic [2:0] poly_idx;
    logic [2:0] vec_i;
    logic [2:0] vec_j;
    logic [7:0] coeff_idx;
    logic [6:0] seed_idx;
    logic [12:0] byte_idx;
    logic [12:0] byte_base;
    logic [10:0] check_count;
    logic op_pass_q;
    logic [7:0] sign_retry_count;
    logic uop_issued_q;
    logic [7:0] uop_issued_code_q;
    logic [MLDSA_COEFF_W-1:0] coeff_hold0;
    logic [MLDSA_COEFF_W-1:0] coeff_hold1;
    logic standard_mode_q;
    logic [12:0] message_len_q;

    logic seed_expand_start;
    logic seed_expand_busy;
    logic seed_expand_done;
    logic seed_expand_error;
    logic [4:0] seed_rho_rd_addr;
    logic [7:0] seed_rho_rd_data;
    logic [5:0] seed_rho_prime_rd_addr;
    logic [7:0] seed_rho_prime_rd_data;
    logic [4:0] seed_k_rd_addr;
    logic [7:0] seed_k_rd_data;
    logic seed_xof_start;
    logic seed_xof_stop;
    logic seed_xof_mode_shake256;
    logic [7:0] seed_xof_abs_byte_data;
    logic seed_xof_abs_byte_valid;
    logic seed_xof_abs_byte_last;
    logic seed_xof_abs_byte_ready;
    logic [7:0] seed_xof_sq_byte;
    logic seed_xof_sq_valid;
    logic seed_xof_sq_byte_ready;
    logic seed_xof_state_squeezing;

    logic expanda_rho_we;
    logic [4:0] expanda_rho_addr;
    logic [7:0] expanda_rho_wdata;
    logic expanda_start;
    logic [7:0] expanda_coeff_index;
    logic [MLDSA_COEFF_W-1:0] expanda_coeff_data;
    logic expanda_coeff_valid;
    logic expanda_busy;
    logic expanda_done;
    logic expanda_error;
    logic expanda_xof_start;
    logic expanda_xof_stop;
    logic expanda_xof_mode_shake256;
    logic [7:0] expanda_xof_abs_byte_data;
    logic expanda_xof_abs_byte_valid;
    logic expanda_xof_abs_byte_last;
    logic expanda_xof_abs_byte_ready;
    logic [7:0] expanda_xof_sq_byte;
    logic expanda_xof_sq_valid;
    logic expanda_xof_sq_byte_ready;
    logic expanda_xof_state_squeezing;

    logic std_hash_a_we;
    logic [5:0] std_hash_a_wr_addr;
    logic [7:0] std_hash_a_wr_data;
    logic std_hash_b_we;
    logic [W1_MEM_ADDR_W-1:0] std_hash_b_wr_addr;
    logic [7:0] std_hash_b_wr_data;
    logic std_hash_start;
    logic [15:0] std_hash_a_len;
    logic [15:0] std_hash_b_len;
    logic [15:0] std_hash_out_len;
    logic [HASH_MEM_ADDR_W-1:0] std_hash_out_rd_addr;
    logic [7:0] std_hash_out_rd_data;
    logic std_hash_busy;
    logic std_hash_done;
    logic std_hash_error;
    logic std_hash_xof_start;
    logic std_hash_xof_stop;
    logic std_hash_xof_mode_shake256;
    logic [7:0] std_hash_xof_abs_byte_data;
    logic std_hash_xof_abs_byte_valid;
    logic std_hash_xof_abs_byte_last;
    logic std_hash_xof_abs_byte_ready;
    logic [7:0] std_hash_xof_sq_byte;
    logic std_hash_xof_sq_valid;
    logic std_hash_xof_sq_byte_ready;
    logic std_hash_xof_state_squeezing;

    // These byte stores keep the existing zero-latency read behavior used by
    // the current state machine. They are now isolated so we can later retime
    // the affected states and swap in compiler SRAMs with minimal collateral.
    mldsa_byte_store_async #(
        .DEPTH (W1_PACK_BYTES_MAX)
    ) u_sign_w1_bytes (
        .clk    (clk),
        .wr_en  (sign_w1_wr_en),
        .wr_addr(sign_w1_wr_addr),
        .wr_data(sign_w1_wr_data),
        .rd_addr(sign_w1_rd_addr),
        .rd_data(sign_w1_rd_data)
    );

    mldsa_byte_store_async #(
        .DEPTH (W1_PACK_BYTES_MAX)
    ) u_verify_w1_bytes (
        .clk    (clk),
        .wr_en  (verify_w1_wr_en),
        .wr_addr(verify_w1_wr_addr),
        .wr_data(verify_w1_wr_data),
        .rd_addr(verify_w1_rd_addr),
        .rd_data(verify_w1_rd_data)
    );

    mldsa_byte_store_async #(
        .DEPTH (HASH_BUF_BYTES)
    ) u_hash_buf (
        .clk    (clk),
        .wr_en  (hash_buf_wr_en),
        .wr_addr(hash_buf_wr_addr),
        .wr_data(hash_buf_wr_data),
        .rd_addr(hash_buf_rd_addr),
        .rd_data(hash_buf_rd_data)
    );

    mldsa_byte_store_async #(
        .DEPTH (HASH_BUF_BYTES)
    ) u_sign_hash_buf (
        .clk    (clk),
        .wr_en  (sign_hash_buf_wr_en),
        .wr_addr(sign_hash_buf_wr_addr),
        .wr_data(sign_hash_buf_wr_data),
        .rd_addr('0),
        .rd_data(sign_hash_buf_rd_data_unused)
    );

    mldsa_byte_store_async #(.DEPTH(64), .ADDR_W(6)) u_tr_store (
        .clk    (clk),
        .wr_en  (load_ctx_we && !ucode_busy && load_ctx_sel == 2'd0 && load_ctx_addr < 13'd64),
        .wr_addr(load_ctx_addr[5:0]),
        .wr_data(load_ctx_data),
        .rd_addr(byte_idx[5:0]),
        .rd_data(tr_rd_data)
    );

    mldsa_byte_store_async #(.DEPTH(HASH_BUF_BYTES), .ADDR_W(HASH_MEM_ADDR_W)) u_message_store (
        .clk    (clk),
        .wr_en  (load_ctx_we && !ucode_busy && load_ctx_sel == 2'd1 && load_ctx_addr < HASH_BUF_BYTES),
        .wr_addr(load_ctx_addr[HASH_MEM_ADDR_W-1:0]),
        .wr_data(load_ctx_data),
        .rd_addr(byte_idx[HASH_MEM_ADDR_W-1:0]),
        .rd_data(message_rd_data)
    );

    mldsa_byte_store_async #(.DEPTH(64), .ADDR_W(6)) u_mu_store (
        .clk    (clk),
        .wr_en  (mu_wr_en),
        .wr_addr(mu_wr_addr),
        .wr_data(mu_wr_data),
        .rd_addr(byte_idx[5:0]),
        .rd_data(mu_rd_data)
    );

    mldsa_seed_expand u_seed_expand (
        .clk              (clk),
        .rst_n            (rst_n),
        .seed_we          (load_seed_we && !ucode_busy),
        .seed_addr        (load_seed_addr),
        .seed_data        (load_seed_data),
        .start            (seed_expand_start),
        .busy             (seed_expand_busy),
        .done             (seed_expand_done),
        .error            (seed_expand_error),
        .rho_rd_addr      (seed_rho_rd_addr),
        .rho_rd_data      (seed_rho_rd_data),
        .rho_prime_rd_addr(seed_rho_prime_rd_addr),
        .rho_prime_rd_data(seed_rho_prime_rd_data),
        .k_rd_addr        (seed_k_rd_addr),
        .k_rd_data        (seed_k_rd_data),
        .xof_start_o      (seed_xof_start),
        .xof_stop_o       (seed_xof_stop),
        .xof_mode_shake256_o(seed_xof_mode_shake256),
        .xof_abs_byte_data_o(seed_xof_abs_byte_data),
        .xof_abs_byte_valid_o(seed_xof_abs_byte_valid),
        .xof_abs_byte_last_o(seed_xof_abs_byte_last),
        .xof_abs_byte_ready_i(seed_xof_abs_byte_ready),
        .xof_sq_byte_i    (seed_xof_sq_byte),
        .xof_sq_valid_i   (seed_xof_sq_valid),
        .xof_sq_byte_ready_o(seed_xof_sq_byte_ready),
        .xof_state_squeezing_i(seed_xof_state_squeezing)
    );

    mldsa_expanda_poly u_expanda (
        .clk        (clk),
        .rst_n      (rst_n),
        .rho_we     (expanda_rho_we),
        .rho_addr   (expanda_rho_addr),
        .rho_wdata  (expanda_rho_wdata),
        .start      (expanda_start),
        .row_i      (vec_i),
        .col_j      (vec_j),
        .coeff_index(expanda_coeff_index),
        .coeff_data (expanda_coeff_data),
        .coeff_valid(expanda_coeff_valid),
        .coeff_ready(1'b1),
        .busy       (expanda_busy),
        .done       (expanda_done),
        .error      (expanda_error),
        .xof_start_o(expanda_xof_start),
        .xof_stop_o (expanda_xof_stop),
        .xof_mode_shake256_o(expanda_xof_mode_shake256),
        .xof_abs_byte_data_o(expanda_xof_abs_byte_data),
        .xof_abs_byte_valid_o(expanda_xof_abs_byte_valid),
        .xof_abs_byte_last_o(expanda_xof_abs_byte_last),
        .xof_abs_byte_ready_i(expanda_xof_abs_byte_ready),
        .xof_sq_byte_i(expanda_xof_sq_byte),
        .xof_sq_valid_i(expanda_xof_sq_valid),
        .xof_sq_byte_ready_o(expanda_xof_sq_byte_ready),
        .xof_state_squeezing_i(expanda_xof_state_squeezing)
    );

    mldsa_hash_concat2 #(
        .A_BYTES  (64),
        .B_BYTES  (W1_PACK_BYTES_MAX),
        .OUT_BYTES(HASH_BUF_BYTES)
    ) u_std_hash (
        .clk        (clk),
        .rst_n      (rst_n),
        .a_we       (std_hash_a_we),
        .a_wr_addr  (std_hash_a_wr_addr),
        .a_wr_data  (std_hash_a_wr_data),
        .b_we       (std_hash_b_we),
        .b_wr_addr  (std_hash_b_wr_addr),
        .b_wr_data  (std_hash_b_wr_data),
        .start      (std_hash_start),
        .a_len      (std_hash_a_len),
        .b_len      (std_hash_b_len),
        .out_len    (std_hash_out_len),
        .out_rd_addr(std_hash_out_rd_addr),
        .out_rd_data(std_hash_out_rd_data),
        .busy       (std_hash_busy),
        .done       (std_hash_done),
        .error      (std_hash_error),
        .xof_start_o(std_hash_xof_start),
        .xof_stop_o (std_hash_xof_stop),
        .xof_mode_shake256_o(std_hash_xof_mode_shake256),
        .xof_abs_byte_data_o(std_hash_xof_abs_byte_data),
        .xof_abs_byte_valid_o(std_hash_xof_abs_byte_valid),
        .xof_abs_byte_last_o(std_hash_xof_abs_byte_last),
        .xof_abs_byte_ready_i(std_hash_xof_abs_byte_ready),
        .xof_sq_byte_i(std_hash_xof_sq_byte),
        .xof_sq_valid_i(std_hash_xof_sq_valid),
        .xof_sq_byte_ready_o(std_hash_xof_sq_byte_ready),
        .xof_state_squeezing_i(std_hash_xof_state_squeezing)
    );

    function automatic logic [$clog2(BANKS)-1:0] bank_a(input logic [2:0] i, input logic [2:0] j);
        integer idx;
        begin
            if (USE_RESIDENT_A) idx = B_A_BASE + (i * MLDSA_L_MAX) + j;
            else idx = B_A_TMP_BASE;
            bank_a = idx[$clog2(BANKS)-1:0];
        end
    endfunction

    function automatic logic [$clog2(BANKS)-1:0] bank_s1(input logic [2:0] j);
        integer idx;
        begin
            idx = B_S1_BASE + j;
            bank_s1 = idx[$clog2(BANKS)-1:0];
        end
    endfunction

    function automatic logic [$clog2(BANKS)-1:0] bank_s1n(input logic [2:0] j);
        integer idx;
        begin
            idx = B_S1N_BASE + j;
            bank_s1n = idx[$clog2(BANKS)-1:0];
        end
    endfunction

    function automatic logic [$clog2(BANKS)-1:0] bank_t1n(input logic [2:0] i);
        integer idx;
        begin
            idx = B_T1N_BASE + i;
            bank_t1n = idx[$clog2(BANKS)-1:0];
        end
    endfunction

    function automatic logic [$clog2(BANKS)-1:0] bank_s2n(input logic [2:0] i);
        begin
            bank_s2n = bank_t1n(i);
        end
    endfunction

    function automatic logic [$clog2(BANKS)-1:0] bank_y(input logic [2:0] j);
        integer idx;
        begin
            idx = B_Y_BASE + j;
            bank_y = idx[$clog2(BANKS)-1:0];
        end
    endfunction

    function automatic logic [$clog2(BANKS)-1:0] bank_acc(input logic [2:0] i);
        integer idx;
        begin
            idx = B_ACC_BASE + i;
            bank_acc = idx[$clog2(BANKS)-1:0];
        end
    endfunction

    function automatic logic [$clog2(BANKS)-1:0] bank_w(input logic [2:0] i);
        integer idx;
        begin
            idx = B_W_BASE + i;
            bank_w = idx[$clog2(BANKS)-1:0];
        end
    endfunction

    function automatic logic [$clog2(BANKS)-1:0] bank_w1(input logic [2:0] i);
        integer idx;
        begin
            idx = B_W1_BASE + i;
            bank_w1 = idx[$clog2(BANKS)-1:0];
        end
    endfunction

    function automatic logic [$clog2(BANKS)-1:0] bank_z(input logic [2:0] j);
        integer idx;
        begin
            idx = B_Z_BASE + j;
            bank_z = idx[$clog2(BANKS)-1:0];
        end
    endfunction

    function automatic logic [$clog2(BANKS)-1:0] bank_hint(input logic [2:0] i);
        integer idx;
        begin
            idx = B_HINT_BASE + i;
            bank_hint = idx[$clog2(BANKS)-1:0];
        end
    endfunction

    function automatic logic [$clog2(BANKS)-1:0] bank_t0n(input logic [2:0] i);
        begin
            bank_t0n = bank_hint(i);
        end
    endfunction

    function automatic logic [$clog2(BANKS)-1:0] bank_hint_result(input logic [2:0] i);
        begin
            if (standard_mode_q && op_mode == MLDSA_OP_SIGN) bank_hint_result = bank_w1(i);
            else bank_hint_result = bank_hint(i);
        end
    endfunction

    function automatic logic [$clog2(BANKS)-1:0] bank_c_verify();
        integer idx;
        begin
            idx = B_CV_BASE;
            bank_c_verify = idx[$clog2(BANKS)-1:0];
        end
    endfunction

    mldsa_param_rom u_param_rom (
        .level_sel  (level_sel),
        .k          (level_k),
        .l          (level_l),
        .eta        (level_eta),
        .tau        (level_tau),
        .gamma1     (level_gamma1),
        .gamma2     (level_gamma2),
        .beta       (level_beta),
        .omega      (level_omega),
        .pk_bytes   (level_pk_bytes),
        .sk_bytes   (level_sk_bytes),
        .sig_bytes  (level_sig_bytes),
        .level_valid(level_valid)
    );

    mldsa_ucode_ctrl u_ucode (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (start && level_valid),
        .op_mode   (op_mode),
        .step_done (uop_step_done),
        .step_retry(uop_step_retry),
        .step_fail (uop_step_fail),
        .uop_valid (),
        .uop_code  (uop_code),
        .busy      (ucode_busy),
        .done      (ucode_done),
        .error     (ucode_error),
        .pc_dbg    (ucode_pc)
    );

    mldsa_poly_ram #(.BANKS(BANKS)) u_ram (
        .clk      (clk),
        .rd0_en   (ram_rd0_en),
        .rd0_bank (ram_rd0_bank),
        .rd0_addr (ram_rd0_addr),
        .rd0_data (ram_rd0_data),
        .rd0_data_pipe (ram_rd0_data_pipe),
        .rd0_valid (ram_rd0_valid),
        .pair_rd0_en   (ram_pair_rd0_en),
        .pair_rd0_bank (ram_pair_rd0_bank),
        .pair_rd0_addr (ram_pair_rd0_addr),
        .pair_rd0_data (ram_pair_rd0_data),
        .pair_rd0_data_pipe (ram_pair_rd0_data_pipe),
        .pair_rd0_valid (ram_pair_rd0_valid),
        .rd1_en   (ram_rd1_en),
        .rd1_bank (ram_rd1_bank),
        .rd1_addr (ram_rd1_addr),
        .rd1_data (ram_rd1_data),
        .rd1_data_pipe (ram_rd1_data_pipe),
        .rd1_valid (ram_rd1_valid),
        .pair_rd1_en   (ram_pair_rd1_en),
        .pair_rd1_bank (ram_pair_rd1_bank),
        .pair_rd1_addr (ram_pair_rd1_addr),
        .pair_rd1_data (ram_pair_rd1_data),
        .pair_rd1_data_pipe (ram_pair_rd1_data_pipe),
        .pair_rd1_valid (ram_pair_rd1_valid),
        .wr_en    (ram_wr_en),
        .wr_bank  (ram_wr_bank),
        .wr_addr  (ram_wr_addr),
        .wr_data  (ram_wr_data),
        .pair_wr_en   (ram_pair_wr_en),
        .pair_wr_bank (ram_pair_wr_bank),
        .pair_wr_addr (ram_pair_wr_addr),
        .pair_wr_data (ram_pair_wr_data)
    );

    mldsa_ntt_bank_engine #(.BANKS(BANKS)) u_ntt_bank (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (ntt_start),
        .inverse  (ntt_inverse),
        .src_bank (ntt_src_bank),
        .dst_bank (ntt_dst_bank),
        .rd_en    (ntt_rd_en),
        .rd_bank  (ntt_rd_bank),
        .rd_addr  (ntt_rd_addr),
        .rd_data  (ram_rd0_data),
        .wr_en    (ntt_wr_en),
        .wr_bank  (ntt_wr_bank),
        .wr_addr  (ntt_wr_addr),
        .wr_data  (ntt_wr_data),
        .busy     (ntt_busy),
        .done     (ntt_done),
        .state_dbg()
    );

    mldsa_poly_alu #(.BANKS(BANKS)) u_poly (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (poly_start),
        .op_code   (poly_op),
        .src0_bank (poly_src0_bank),
        .src1_bank (poly_src1_bank),
        .dst_bank  (poly_dst_bank),
        .rd0_en    (poly_rd0_en),
        .rd0_bank  (poly_rd0_bank),
        .rd0_addr  (poly_rd0_addr),
        .rd0_data  (ram_rd0_data_pipe),
        .rd0_valid (ram_rd0_valid),
        .pair_rd0_en  (poly_pair_rd0_en),
        .pair_rd0_bank(poly_pair_rd0_bank),
        .pair_rd0_addr(poly_pair_rd0_addr),
        .pair_rd0_data(ram_pair_rd0_data_pipe),
        .pair_rd0_valid(ram_pair_rd0_valid),
        .rd1_en    (poly_rd1_en),
        .rd1_bank  (poly_rd1_bank),
        .rd1_addr  (poly_rd1_addr),
        .rd1_data  (ram_rd1_data_pipe),
        .rd1_valid (ram_rd1_valid),
        .pair_rd1_en  (poly_pair_rd1_en),
        .pair_rd1_bank(poly_pair_rd1_bank),
        .pair_rd1_addr(poly_pair_rd1_addr),
        .pair_rd1_data(ram_pair_rd1_data_pipe),
        .pair_rd1_valid(ram_pair_rd1_valid),
        .wr_en     (poly_wr_en),
        .wr_bank   (poly_wr_bank),
        .wr_addr   (poly_wr_addr),
        .wr_data   (poly_wr_data),
        .pair_wr_en  (poly_pair_wr_en),
        .pair_wr_bank(poly_pair_wr_bank),
        .pair_wr_addr(poly_pair_wr_addr),
        .pair_wr_data(poly_pair_wr_data),
        .busy      (poly_busy),
        .done      (poly_done),
        .state_dbg ()
    );

    mldsa_decompose_hint u_dh (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (dh_start),
        .op_code   (dh_op),
        .coeff_in  (dh_coeff_in),
        .aux_in    (dh_aux_in),
        .gamma2    (dh_gamma2),
        .high_out  (dh_high_out),
        .low_out   (dh_low_out),
        .hint_out  (dh_hint_out),
        .busy      (),
        .done      (dh_done),
        .state_dbg ()
    );

    mldsa_check u_check (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (check_start),
        .mode        (check_mode),
        .item_count  (check_item_count),
        .limit_value (check_limit),
        .data_a      (check_data_a),
        .data_b      (check_data_b),
        .data_valid  (check_data_valid),
        .data_ready  (check_data_ready),
        .pass        (check_pass),
        .fail_index  (check_fail_index),
        .accum_value (check_accum_value),
        .busy        (),
        .done        (check_done),
        .state_dbg   ()
    );

    mldsa_pack_unpack u_pack (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (pack_start),
        .mode           (pack_mode),
        .item_count     (pack_item_count),
        .coeff_in       (pack_coeff_in),
        .coeff_in_valid (pack_coeff_in_valid),
        .coeff_in_ready (pack_coeff_in_ready),
        .byte_out       (pack_byte_out),
        .byte_out_valid (pack_byte_out_valid),
        .byte_out_ready (pack_byte_out_ready),
        .byte_in        (pack_byte_in),
        .byte_in_valid  (pack_byte_in_valid),
        .byte_in_ready  (pack_byte_in_ready),
        .coeff_out      (pack_coeff_out),
        .coeff_out_valid(pack_coeff_out_valid),
        .coeff_out_ready(pack_coeff_out_ready),
        .busy           (),
        .done           (pack_done),
        .error          (pack_error),
        .state_dbg      ()
    );

    always_comb begin
        shared_xof_owner_local =
            (st == S_Y_XOF_START) ||
            (st == S_Y_ABSORB) ||
            (st == S_Y_SAMPLER_START) ||
            (st == S_Y_SQUEEZE) ||
            (st == S_Y_STOP) ||
            (st == S_HASH_START) ||
            (st == S_HASH_ABSORB) ||
            (st == S_HASH_SQUEEZE) ||
            (st == S_HASH_STOP);
        shared_xof_owner_seed =
            (st == S_SEED_START) ||
            (st == S_SEED_WAIT);
        shared_xof_owner_expanda =
            (st == S_EXPANDA_START) ||
            (st == S_EXPANDA_FILL) ||
            (st == S_T1_A_START) ||
            (st == S_T1_A_FILL) ||
            (st == S_MV_A_START) ||
            (st == S_MV_A_FILL);
        shared_xof_owner_std_hash =
            (st == S_STD_HASH_START) ||
            (st == S_STD_HASH_WAIT);

        shared_xof_start_mux          = 1'b0;
        shared_xof_stop_mux           = 1'b0;
        shared_xof_mode_shake256_mux  = 1'b1;
        shared_xof_abs_byte_data_mux  = 8'd0;
        shared_xof_abs_byte_valid_mux = 1'b0;
        shared_xof_abs_byte_last_mux  = 1'b0;
        shared_xof_sq_byte_ready_mux  = 1'b0;

        if (shared_xof_owner_local) begin
            shared_xof_start_mux          = xof_start;
            shared_xof_stop_mux           = xof_stop;
            shared_xof_mode_shake256_mux  = 1'b1;
            shared_xof_abs_byte_data_mux  = xof_abs_byte_data;
            shared_xof_abs_byte_valid_mux = xof_abs_byte_valid;
            shared_xof_abs_byte_last_mux  = xof_abs_byte_last;
            shared_xof_sq_byte_ready_mux  = xof_sq_byte_ready;
        end else if (shared_xof_owner_seed) begin
            shared_xof_start_mux          = seed_xof_start;
            shared_xof_stop_mux           = seed_xof_stop;
            shared_xof_mode_shake256_mux  = seed_xof_mode_shake256;
            shared_xof_abs_byte_data_mux  = seed_xof_abs_byte_data;
            shared_xof_abs_byte_valid_mux = seed_xof_abs_byte_valid;
            shared_xof_abs_byte_last_mux  = seed_xof_abs_byte_last;
            shared_xof_sq_byte_ready_mux  = seed_xof_sq_byte_ready;
        end else if (shared_xof_owner_expanda) begin
            shared_xof_start_mux          = expanda_xof_start;
            shared_xof_stop_mux           = expanda_xof_stop;
            shared_xof_mode_shake256_mux  = expanda_xof_mode_shake256;
            shared_xof_abs_byte_data_mux  = expanda_xof_abs_byte_data;
            shared_xof_abs_byte_valid_mux = expanda_xof_abs_byte_valid;
            shared_xof_abs_byte_last_mux  = expanda_xof_abs_byte_last;
            shared_xof_sq_byte_ready_mux  = expanda_xof_sq_byte_ready;
        end else if (shared_xof_owner_std_hash) begin
            shared_xof_start_mux          = std_hash_xof_start;
            shared_xof_stop_mux           = std_hash_xof_stop;
            shared_xof_mode_shake256_mux  = std_hash_xof_mode_shake256;
            shared_xof_abs_byte_data_mux  = std_hash_xof_abs_byte_data;
            shared_xof_abs_byte_valid_mux = std_hash_xof_abs_byte_valid;
            shared_xof_abs_byte_last_mux  = std_hash_xof_abs_byte_last;
            shared_xof_sq_byte_ready_mux  = std_hash_xof_sq_byte_ready;
        end
    end

    assign xof_abs_ready = shared_xof_owner_local ? shared_xof_abs_ready_mux : 1'b0;
    assign xof_sq_byte = shared_xof_sq_byte_mux;
    assign xof_sq_valid = shared_xof_owner_local ? shared_xof_sq_valid_mux : 1'b0;
    assign seed_xof_abs_byte_ready = shared_xof_owner_seed ? shared_xof_abs_ready_mux : 1'b0;
    assign seed_xof_sq_byte = shared_xof_sq_byte_mux;
    assign seed_xof_sq_valid = shared_xof_owner_seed ? shared_xof_sq_valid_mux : 1'b0;
    assign seed_xof_state_squeezing = shared_xof_owner_seed ? shared_xof_state_squeezing_mux : 1'b0;
    assign expanda_xof_abs_byte_ready = shared_xof_owner_expanda ? shared_xof_abs_ready_mux : 1'b0;
    assign expanda_xof_sq_byte = shared_xof_sq_byte_mux;
    assign expanda_xof_sq_valid = shared_xof_owner_expanda ? shared_xof_sq_valid_mux : 1'b0;
    assign expanda_xof_state_squeezing = shared_xof_owner_expanda ? shared_xof_state_squeezing_mux : 1'b0;
    assign std_hash_xof_abs_byte_ready = shared_xof_owner_std_hash ? shared_xof_abs_ready_mux : 1'b0;
    assign std_hash_xof_sq_byte = shared_xof_sq_byte_mux;
    assign std_hash_xof_sq_valid = shared_xof_owner_std_hash ? shared_xof_sq_valid_mux : 1'b0;
    assign std_hash_xof_state_squeezing = shared_xof_owner_std_hash ? shared_xof_state_squeezing_mux : 1'b0;

    mldsa_xof_byte_engine u_xof_bytes (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (shared_xof_start_mux),
        .stop           (shared_xof_stop_mux),
        .mode_shake256  (shared_xof_mode_shake256_mux),
        .abs_byte_data  (shared_xof_abs_byte_data_mux),
        .abs_byte_valid (shared_xof_abs_byte_valid_mux),
        .abs_byte_last  (shared_xof_abs_byte_last_mux),
        .abs_byte_ready (shared_xof_abs_ready_mux),
        .sq_byte_data   (shared_xof_sq_byte_mux),
        .sq_byte_valid  (shared_xof_sq_valid_mux),
        .sq_byte_ready  (shared_xof_sq_byte_ready_mux),
        .busy           (xof_busy),
        .state_absorbing(),
        .state_squeezing(shared_xof_state_squeezing_mux)
    );

    mldsa_sampler u_sampler (
        .clk             (clk),
        .rst_n           (rst_n),
        .start           (sampler_start),
        .sample_mode     (sampler_mode),
        .eta             (sampler_eta),
        .gamma1_is_2p19  (sampler_gamma1_2p19),
        .tau             (sampler_tau),
        .byte_data       (sampler_byte_data),
        .byte_valid      (sampler_byte_valid),
        .byte_ready      (sampler_byte_ready),
        .coeff_index     (sampler_coeff_index),
        .coeff_data      (sampler_coeff_data),
        .coeff_valid     (sampler_coeff_valid),
        .coeff_ready     (1'b1),
        .busy            (),
        .done            (sampler_done),
        .error           (sampler_error),
        .state_dbg       ()
    );

    always_comb begin
        ctrl_rd0_en   = 1'b0;
        ctrl_rd0_bank = '0;
        ctrl_rd0_addr = 8'd0;
        ctrl_rd1_en   = 1'b0;
        ctrl_rd1_bank = '0;
        ctrl_rd1_addr = 8'd0;
        ctrl_wr_en    = 1'b0;
        ctrl_wr_bank  = '0;
        ctrl_wr_addr  = 8'd0;
        ctrl_wr_data  = '0;

        ntt_start    = 1'b0;
        ntt_inverse  = 1'b0;
        ntt_src_bank = B_TMP_BASE;
        ntt_dst_bank = B_TMP_BASE;

        poly_start     = 1'b0;
        poly_op        = MLDSA_POLY_COPY;
        poly_src0_bank = B_TMP_BASE;
        poly_src1_bank = B_TMP_BASE;
        poly_dst_bank  = B_TMP_BASE;

        dh_start    = 1'b0;
        dh_op       = MLDSA_DH_DECOMPOSE;
        dh_coeff_in = coeff_hold0;
        dh_aux_in   = coeff_hold1;
        dh_gamma2   = cfg_gamma2_q;

        check_start      = 1'b0;
        check_mode       = MLDSA_CHECK_NORM;
        check_item_count = 16'd0;
        check_limit      = 24'd0;
        check_data_a     = check_pipe_a_q;
        check_data_b     = check_pipe_b_q;
        check_data_valid = 1'b0;
        check_pipe_issue = 1'b0;

        pack_start           = 1'b0;
        pack_mode            = MLDSA_PACK_COEFF24_PACK;
        pack_item_count      = 16'd256;
        pack_coeff_in        = ram_rd0_data;
        pack_coeff_in_valid  = 1'b0;
        pack_byte_out_ready  = 1'b1;
        pack_byte_in         = 8'd0;
        pack_byte_in_valid   = 1'b0;
        pack_coeff_out_ready = 1'b1;

        xof_start = 1'b0;
        xof_stop  = 1'b0;
        xof_abs_byte_data  = 8'd0;
        xof_abs_byte_valid = 1'b0;
        xof_abs_byte_last  = 1'b0;
        xof_sq_byte_ready  = 1'b0;

        sampler_start       = 1'b0;
        sampler_mode        = MLDSA_SAMPLE_GAMMA1;
        sampler_eta         = cfg_eta_q;
        sampler_gamma1_2p19 = cfg_gamma1_is_2p19_q;
        sampler_tau         = cfg_tau_q;
        sampler_byte_data   = xof_sq_byte;
        sampler_byte_valid  = 1'b0;

        seed_expand_start = 1'b0;
        seed_rho_rd_addr = byte_idx[4:0];
        seed_rho_prime_rd_addr = seed_idx[5:0];
        seed_k_rd_addr = byte_idx[4:0];

        expanda_rho_we = 1'b0;
        expanda_rho_addr = byte_idx[4:0];
        expanda_rho_wdata = seed_rho_rd_data;
        expanda_start = 1'b0;

        std_hash_a_we = 1'b0;
        std_hash_a_wr_addr = byte_idx[5:0];
        std_hash_a_wr_data = 8'd0;
        std_hash_b_we = 1'b0;
        std_hash_b_wr_addr = byte_idx[W1_MEM_ADDR_W-1:0];
        std_hash_b_wr_data = 8'd0;
        std_hash_start = 1'b0;
        std_hash_a_len = 16'd64;
        std_hash_b_len = 16'd0;
        std_hash_out_len = 16'd64;
        std_hash_out_rd_addr = byte_idx[HASH_MEM_ADDR_W-1:0];
        mu_wr_en = 1'b0;
        mu_wr_addr = byte_idx[5:0];
        mu_wr_data = std_hash_out_rd_data;

        sign_w1_wr_en   = 1'b0;
        sign_w1_wr_addr = '0;
        sign_w1_wr_data = 8'd0;
        sign_w1_rd_addr = byte_idx[W1_MEM_ADDR_W-1:0];
        verify_w1_wr_en   = 1'b0;
        verify_w1_wr_addr = '0;
        verify_w1_wr_data = 8'd0;
        verify_w1_rd_addr = byte_idx[W1_MEM_ADDR_W-1:0];
        hash_buf_wr_en   = 1'b0;
        hash_buf_wr_addr = '0;
        hash_buf_wr_data = 8'd0;
        hash_buf_rd_addr = byte_idx[HASH_MEM_ADDR_W-1:0];
        sign_hash_buf_wr_en   = 1'b0;
        sign_hash_buf_wr_addr = '0;
        sign_hash_buf_wr_data = 8'd0;

        unique case (st)
            S_SEED_START: begin
                seed_expand_start = 1'b1;
            end
            S_EXPANDA_RHO_LOAD: begin
                expanda_rho_we = 1'b1;
                expanda_rho_addr = byte_idx[4:0];
                expanda_rho_wdata = seed_rho_rd_data;
            end
            S_EXPANDA_START: begin
                expanda_start = 1'b1;
            end
            S_EXPANDA_FILL: begin
                if (expanda_coeff_valid) begin
                    ctrl_wr_en   = 1'b1;
                    ctrl_wr_bank = bank_a(vec_i, vec_j);
                    ctrl_wr_addr = expanda_coeff_index;
                    ctrl_wr_data = expanda_coeff_data;
                end
            end
            S_T1_A_START,
            S_MV_A_START: begin
                expanda_start = 1'b1;
            end
            S_T1_A_FILL,
            S_MV_A_FILL: begin
                if (expanda_coeff_valid) begin
                    ctrl_wr_en   = 1'b1;
                    ctrl_wr_bank = bank_a(vec_i, vec_j);
                    ctrl_wr_addr = expanda_coeff_index;
                    ctrl_wr_data = expanda_coeff_data;
                end
            end
            S_STD_HASH_LOAD_A: begin
                std_hash_a_we = 1'b1;
                std_hash_a_wr_addr = byte_idx[5:0];
                if (uop_code == MLDSA_UOP_HASH_MU) std_hash_a_wr_data = tr_rd_data;
                else std_hash_a_wr_data = mu_rd_data;
            end
            S_STD_HASH_LOAD_B: begin
                std_hash_b_we = 1'b1;
                std_hash_b_wr_addr = byte_idx[W1_MEM_ADDR_W-1:0];
                if (uop_code == MLDSA_UOP_HASH_MU) begin
                    std_hash_b_wr_data = message_rd_data;
                end else if (uop_code == MLDSA_UOP_HASH_SIGN_W1) begin
                    std_hash_b_wr_data = sign_w1_rd_data;
                end else begin
                    std_hash_b_wr_data = verify_w1_rd_data;
                end
            end
            S_STD_HASH_START: begin
                std_hash_start = 1'b1;
                std_hash_a_len = 16'd64;
                if (uop_code == MLDSA_UOP_HASH_MU) begin
                    std_hash_b_len = {3'd0, message_len_q};
                    std_hash_out_len = 16'd64;
                end else begin
                    std_hash_b_len = {3'd0, cfg_w1_pack_bytes_q};
                    std_hash_out_len = HASH_BUF_BYTES;
                end
            end
            S_STD_HASH_COPY: begin
                std_hash_out_rd_addr = byte_idx[HASH_MEM_ADDR_W-1:0];
                if (uop_code == MLDSA_UOP_HASH_MU) begin
                    mu_wr_en   = 1'b1;
                    mu_wr_addr = byte_idx[5:0];
                    mu_wr_data = std_hash_out_rd_data;
                end else begin
                    hash_buf_wr_en   = 1'b1;
                    hash_buf_wr_addr = byte_idx[HASH_MEM_ADDR_W-1:0];
                    hash_buf_wr_data = std_hash_out_rd_data;
                    if (uop_code == MLDSA_UOP_HASH_SIGN_W1) begin
                        sign_hash_buf_wr_en   = 1'b1;
                        sign_hash_buf_wr_addr = byte_idx[HASH_MEM_ADDR_W-1:0];
                        sign_hash_buf_wr_data = std_hash_out_rd_data;
                    end
                end
            end

            S_T1_CLR,
            S_MV_CLR: begin
                ctrl_wr_en   = 1'b1;
                if (st == S_T1_CLR) ctrl_wr_bank = bank_t1n(vec_i);
                else ctrl_wr_bank = bank_acc(vec_i);
                ctrl_wr_addr = coeff_idx;
                ctrl_wr_data = 24'd0;
            end

            S_T1_NTT_START: begin
                ntt_start    = 1'b1;
                ntt_inverse  = 1'b0;
                ntt_src_bank = bank_s1(vec_j);
                ntt_dst_bank = bank_s1n(vec_j);
            end
            S_T1_MAC_START: begin
                poly_start     = 1'b1;
                poly_op        = (vec_j == 3'd0) ? MLDSA_POLY_MUL : MLDSA_POLY_MAC;
                poly_src0_bank = bank_a(vec_i, vec_j);
                poly_src1_bank = bank_s1n(vec_j);
                poly_dst_bank  = bank_t1n(vec_i);
            end

            S_Y_XOF_START: begin
                xof_start = 1'b1;
            end
            S_Y_ABSORB: begin
                xof_abs_byte_valid = 1'b1;
                xof_abs_byte_last  = standard_mode_q ? (seed_idx == 7'd65) : (seed_idx == 7'd33);
                if (standard_mode_q) begin
                    if (seed_idx == 7'd64) xof_abs_byte_data = sign_retry_count;
                    else if (seed_idx == 7'd65) xof_abs_byte_data = {5'd0, poly_idx};
                    else xof_abs_byte_data = seed_rho_prime_rd_data;
                end else begin
                    if (seed_idx == 7'd32) xof_abs_byte_data = sign_retry_count;
                    else if (seed_idx == 7'd33) xof_abs_byte_data = {5'd0, poly_idx};
                    else xof_abs_byte_data = seed_mem[seed_idx[4:0]];
                end
            end
            S_Y_SAMPLER_START: begin
                sampler_start = 1'b1;
                sampler_mode  = MLDSA_SAMPLE_GAMMA1;
            end
            S_Y_SQUEEZE: begin
                sampler_mode       = MLDSA_SAMPLE_GAMMA1;
                sampler_byte_data  = xof_sq_byte;
                sampler_byte_valid = xof_sq_valid;
                xof_sq_byte_ready  = sampler_byte_ready;
                if (sampler_coeff_valid) begin
                    ctrl_wr_en   = 1'b1;
                    ctrl_wr_bank = bank_y(poly_idx);
                    ctrl_wr_addr = sampler_coeff_index;
                    ctrl_wr_data = sampler_coeff_data;
                end
            end
            S_Y_STOP: begin
                xof_stop = 1'b1;
            end

            S_MV_NTT_START: begin
                ntt_start    = 1'b1;
                ntt_inverse  = 1'b0;
                if (uop_code == MLDSA_UOP_MATVEC_AY) ntt_src_bank = bank_y(vec_j);
                else ntt_src_bank = bank_z(vec_j);
                ntt_dst_bank = B_TMP_BASE;
            end
            S_MV_MAC_START: begin
                poly_start     = 1'b1;
                poly_op        = (vec_j == 3'd0) ? MLDSA_POLY_MUL : MLDSA_POLY_MAC;
                poly_src0_bank = bank_a(vec_i, vec_j);
                poly_src1_bank = B_TMP_BASE;
                poly_dst_bank  = bank_acc(vec_i);
            end
            S_MV_INV_START: begin
                if (uop_code == MLDSA_UOP_MATVEC_AY) begin
                    ntt_start    = 1'b1;
                    ntt_inverse  = 1'b1;
                    ntt_src_bank = bank_acc(vec_i);
                    ntt_dst_bank = bank_w(vec_i);
                end else begin
                    if (vec_j == 3'd0) begin
                        poly_start     = 1'b1;
                        poly_op        = MLDSA_POLY_MUL;
                        poly_src0_bank = B_CN_BASE;
                        poly_src1_bank = bank_t1n(vec_i);
                        poly_dst_bank  = B_TMP_BASE;
                    end else if (vec_j == 3'd1) begin
                        poly_start     = 1'b1;
                        poly_op        = MLDSA_POLY_SUB;
                        poly_src0_bank = bank_acc(vec_i);
                        poly_src1_bank = B_TMP_BASE;
                        poly_dst_bank  = bank_acc(vec_i);
                    end else begin
                        ntt_start    = 1'b1;
                        ntt_inverse  = 1'b1;
                        ntt_src_bank = bank_acc(vec_i);
                        ntt_dst_bank = bank_w(vec_i);
                    end
                end
            end

            S_W1_DEC_READ: begin
                ctrl_rd0_en   = 1'b1;
                ctrl_rd0_bank = bank_w(poly_idx);
                ctrl_rd0_addr = coeff_idx;
            end
            S_W1_DEC_START: begin
                dh_start    = 1'b1;
                dh_op       = MLDSA_DH_DECOMPOSE;
                dh_coeff_in = ram_rd0_data;
            end
            S_W1_DEC_WRITE: begin
                ctrl_wr_en   = 1'b1;
                ctrl_wr_bank = bank_w1(poly_idx);
                ctrl_wr_addr = coeff_idx;
                ctrl_wr_data = dh_high_out;
            end
            S_PACKW_START: begin
                pack_start      = 1'b1;
                pack_mode       = MLDSA_PACK_COEFF24_PACK;
                pack_item_count = 16'd256;
            end
            S_PACKW_READ: begin
                ctrl_rd0_en   = 1'b1;
                ctrl_rd0_bank = bank_w1(poly_idx);
                ctrl_rd0_addr = coeff_idx;
            end
            S_PACKW_PUSH: begin
                pack_mode           = MLDSA_PACK_COEFF24_PACK;
                pack_item_count     = 16'd256;
                pack_coeff_in       = ram_rd0_data;
                pack_coeff_in_valid = 1'b1;
            end

            S_HASH_START: begin
                xof_start = 1'b1;
            end
            S_HASH_ABSORB: begin
                xof_abs_byte_valid = 1'b1;
                xof_abs_byte_last  = (byte_idx == cfg_w1_pack_bytes_q - 13'd1);
                if (uop_code == MLDSA_UOP_HASH_SIGN_W1) xof_abs_byte_data = sign_w1_rd_data;
                else xof_abs_byte_data = verify_w1_rd_data;
            end
            S_HASH_SQUEEZE: begin
                xof_sq_byte_ready = 1'b1;
            end
            S_HASH_STOP: begin
                xof_stop = 1'b1;
            end

            S_CHAL_START: begin
                sampler_start = 1'b1;
                sampler_mode  = MLDSA_SAMPLE_CHALLENGE;
            end
            S_CHAL_FEED: begin
                sampler_mode       = MLDSA_SAMPLE_CHALLENGE;
                sampler_byte_data  = hash_buf_rd_data;
                sampler_byte_valid = sampler_byte_ready && (byte_idx < HASH_BUF_BYTES);
                if (sampler_coeff_valid) begin
                    ctrl_wr_bank = (uop_code == MLDSA_UOP_SAMPLE_C_SIGN) ? B_C_BASE : bank_c_verify();
                    ctrl_wr_en   = 1'b1;
                    ctrl_wr_addr = sampler_coeff_index;
                    ctrl_wr_data = sampler_coeff_data;
                end
            end
            S_CHAL_NTT_START: begin
                if (uop_code == MLDSA_UOP_SAMPLE_C_SIGN) begin
                    ntt_start    = 1'b1;
                    ntt_inverse  = 1'b0;
                    ntt_src_bank = B_C_BASE;
                    ntt_dst_bank = B_CN_BASE;
                end
            end

            S_Z_MUL_START: begin
                if (vec_j == 3'd0) begin
                    poly_start     = 1'b1;
                    poly_op        = MLDSA_POLY_MUL;
                    poly_src0_bank = B_CN_BASE;
                    poly_src1_bank = bank_s1n(poly_idx);
                    poly_dst_bank  = B_TMP_BASE;
                end else if (vec_j == 3'd1) begin
                    ntt_start    = 1'b1;
                    ntt_inverse  = 1'b1;
                    ntt_src_bank = B_TMP_BASE;
                    ntt_dst_bank = B_TMP_BASE;
                end else begin
                    poly_start     = 1'b1;
                    poly_op        = MLDSA_POLY_ADD;
                    poly_src0_bank = bank_y(poly_idx);
                    poly_src1_bank = B_TMP_BASE;
                    poly_dst_bank  = bank_z(poly_idx);
                end
            end

            S_ZCHK_START: begin
                check_start      = 1'b1;
                check_mode       = MLDSA_CHECK_NORM;
                check_item_count = cfg_z_total_coeffs_q;
                check_limit      = cfg_z_bound_q;
                ctrl_rd0_en      = 1'b1;
                ctrl_rd0_bank    = bank_z(3'd0);
                ctrl_rd0_addr    = 8'd0;
                check_pipe_issue = 1'b1;
            end
            S_ZCHK_RDWAIT: begin
                check_mode       = MLDSA_CHECK_NORM;
                check_item_count = cfg_z_total_coeffs_q;
                check_limit      = cfg_z_bound_q;
            end
            S_ZCHK_FEED: begin
                check_mode       = MLDSA_CHECK_NORM;
                check_item_count = cfg_z_total_coeffs_q;
                check_limit      = cfg_z_bound_q;
                check_data_valid = check_pipe_valid_q;
                if (poly_idx != cfg_l_last_q || coeff_idx != 8'd255) begin
                    ctrl_rd0_en   = 1'b1;
                    ctrl_rd0_bank = (coeff_idx == 8'd255) ? bank_z(poly_idx + 3'd1) : bank_z(poly_idx);
                    ctrl_rd0_addr = (coeff_idx == 8'd255) ? 8'd0 : (coeff_idx + 8'd1);
                    check_pipe_issue = check_pipe_valid_q;
                end
            end

            S_HINT_CS2_MUL_START: begin
                poly_start     = 1'b1;
                poly_op        = MLDSA_POLY_MUL;
                poly_src0_bank = B_CN_BASE;
                poly_src1_bank = bank_s2n(poly_idx);
                poly_dst_bank  = B_TMP_BASE;
            end
            S_HINT_CS2_INV_START: begin
                ntt_start    = 1'b1;
                ntt_inverse  = 1'b1;
                ntt_src_bank = B_TMP_BASE;
                ntt_dst_bank = bank_acc(poly_idx);
            end
            S_HINT_WMINUS_START: begin
                poly_start     = 1'b1;
                poly_op        = MLDSA_POLY_SUB;
                poly_src0_bank = bank_w(poly_idx);
                poly_src1_bank = bank_acc(poly_idx);
                poly_dst_bank  = bank_w(poly_idx);
            end
            S_HINT_CT0_MUL_START: begin
                poly_start     = 1'b1;
                poly_op        = MLDSA_POLY_MUL;
                poly_src0_bank = B_CN_BASE;
                poly_src1_bank = bank_t0n(poly_idx);
                poly_dst_bank  = B_TMP_BASE;
            end
            S_HINT_CT0_INV_START: begin
                ntt_start    = 1'b1;
                ntt_inverse  = 1'b1;
                ntt_src_bank = B_TMP_BASE;
                ntt_dst_bank = bank_acc(poly_idx);
            end
            S_HINT_GEN_READ: begin
                ctrl_rd0_en   = 1'b1;
                ctrl_rd0_bank = bank_w(poly_idx);
                ctrl_rd0_addr = coeff_idx;
                ctrl_rd1_en   = 1'b1;
                ctrl_rd1_bank = bank_acc(poly_idx);
                ctrl_rd1_addr = coeff_idx;
            end
            S_HINT_GEN_START: begin
                dh_start    = 1'b1;
                dh_op       = MLDSA_DH_MAKE_HINT;
                if (standard_mode_q) begin
                    dh_coeff_in = modq_add(ram_rd0_data, ram_rd1_data);
                    dh_aux_in   = encode_s24(-center_modq(ram_rd1_data));
                end else begin
                    dh_coeff_in = 24'd0;
                    dh_aux_in   = 24'd0;
                end
            end
            S_HINT_GEN_WRITE: begin
                ctrl_wr_en   = 1'b1;
                ctrl_wr_bank = bank_hint_result(poly_idx);
                ctrl_wr_addr = coeff_idx;
                ctrl_wr_data = {23'd0, dh_hint_out};
            end
            S_HINTCNT_START: begin
                check_start      = 1'b1;
                check_mode       = MLDSA_CHECK_HINTCNT;
                check_item_count = {4'd0, cfg_k_q, 8'd0};
                check_limit      = {{16{1'b0}}, cfg_omega_q};
                ctrl_rd0_en      = 1'b1;
                ctrl_rd0_bank    = bank_hint_result(3'd0);
                ctrl_rd0_addr    = 8'd0;
                check_pipe_issue = 1'b1;
            end
            S_HINTCNT_RDWAIT: begin
                check_mode       = MLDSA_CHECK_HINTCNT;
                check_item_count = {4'd0, cfg_k_q, 8'd0};
                check_limit      = {{16{1'b0}}, cfg_omega_q};
            end
            S_HINTCNT_FEED: begin
                check_mode       = MLDSA_CHECK_HINTCNT;
                check_item_count = {4'd0, cfg_k_q, 8'd0};
                check_limit      = {{16{1'b0}}, cfg_omega_q};
                check_data_valid = check_pipe_valid_q;
                if (poly_idx != cfg_k_last_q || coeff_idx != 8'd255) begin
                    ctrl_rd0_en   = 1'b1;
                    ctrl_rd0_bank = (coeff_idx == 8'd255) ? bank_hint_result(poly_idx + 3'd1) : bank_hint_result(poly_idx);
                    ctrl_rd0_addr = (coeff_idx == 8'd255) ? 8'd0 : (coeff_idx + 8'd1);
                    check_pipe_issue = check_pipe_valid_q;
                end
            end
            S_PACKH_START: begin
                pack_start      = 1'b1;
                pack_mode       = MLDSA_PACK_BIT1_PACK;
                pack_item_count = 16'd256;
            end
            S_PACKH_READ: begin
                ctrl_rd0_en   = 1'b1;
                ctrl_rd0_bank = bank_hint_result(poly_idx);
                ctrl_rd0_addr = coeff_idx;
            end
            S_PACKH_PUSH: begin
                pack_mode           = MLDSA_PACK_BIT1_PACK;
                pack_item_count     = 16'd256;
                pack_coeff_in       = ram_rd0_data;
                pack_coeff_in_valid = 1'b1;
            end

            S_UNPACKH_START: begin
                pack_start      = 1'b1;
                pack_mode       = MLDSA_PACK_BIT1_UNPACK;
                pack_item_count = 16'd256;
            end
            S_UNPACKH_FEED: begin
                pack_mode         = MLDSA_PACK_BIT1_UNPACK;
                pack_item_count   = 16'd256;
                pack_byte_in      = hint_bytes[byte_base + byte_idx];
                pack_byte_in_valid= pack_byte_in_ready;
                if (pack_coeff_out_valid) begin
                    ctrl_wr_en   = 1'b1;
                    ctrl_wr_bank = bank_hint(poly_idx);
                    ctrl_wr_addr = coeff_idx;
                    ctrl_wr_data = pack_coeff_out;
                end
            end
            S_UNPACKH_WAIT: begin
                if (pack_coeff_out_valid) begin
                    ctrl_wr_en   = 1'b1;
                    ctrl_wr_bank = bank_hint(poly_idx);
                    ctrl_wr_addr = coeff_idx;
                    ctrl_wr_data = pack_coeff_out;
                end
            end

            S_USEH_READ: begin
                ctrl_rd0_en   = 1'b1;
                ctrl_rd0_bank = bank_w(poly_idx);
                ctrl_rd0_addr = coeff_idx;
                ctrl_rd1_en   = 1'b1;
                ctrl_rd1_bank = bank_hint(poly_idx);
                ctrl_rd1_addr = coeff_idx;
            end
            S_USEH_START: begin
                dh_start    = 1'b1;
                dh_op       = MLDSA_DH_USE_HINT;
                dh_coeff_in = ram_rd0_data;
                dh_aux_in   = ram_rd1_data;
            end
            S_USEH_WRITE: begin
                ctrl_wr_en   = 1'b1;
                ctrl_wr_bank = bank_w1(poly_idx);
                ctrl_wr_addr = coeff_idx;
                ctrl_wr_data = dh_high_out;
            end

            S_CMPC_START: begin
                check_start      = 1'b1;
                check_mode       = MLDSA_CHECK_COEFFEQ;
                check_item_count = 16'd256;
                check_limit      = 24'd0;
                ctrl_rd0_en      = 1'b1;
                ctrl_rd0_bank    = B_C_BASE;
                ctrl_rd0_addr    = 8'd0;
                ctrl_rd1_en      = 1'b1;
                ctrl_rd1_bank    = bank_c_verify();
                ctrl_rd1_addr    = 8'd0;
                check_pipe_issue = 1'b1;
            end
            S_CMPC_RDWAIT: begin
                check_mode       = MLDSA_CHECK_COEFFEQ;
                check_item_count = 16'd256;
            end
            S_CMPC_FEED: begin
                check_mode       = MLDSA_CHECK_COEFFEQ;
                check_item_count = 16'd256;
                check_data_valid = check_pipe_valid_q;
                if (coeff_idx != 8'd255) begin
                    ctrl_rd0_en   = 1'b1;
                    ctrl_rd0_bank = B_C_BASE;
                    ctrl_rd0_addr = coeff_idx + 8'd1;
                    ctrl_rd1_en   = 1'b1;
                    ctrl_rd1_bank = bank_c_verify();
                    ctrl_rd1_addr = coeff_idx + 8'd1;
                    check_pipe_issue = check_pipe_valid_q;
                end
            end
            default: ;
        endcase

        if ((st == S_PACKW_PUSH || st == S_PACKW_WAIT) && pack_byte_out_valid) begin
            if (uop_code == MLDSA_UOP_PACK_W1_SIGN) begin
                sign_w1_wr_en   = 1'b1;
                sign_w1_wr_addr = (poly_idx * W1_BYTES_PER_POLY) + byte_idx;
                sign_w1_wr_data = pack_byte_out;
            end else begin
                verify_w1_wr_en   = 1'b1;
                verify_w1_wr_addr = (poly_idx * W1_BYTES_PER_POLY) + byte_idx;
                verify_w1_wr_data = pack_byte_out;
            end
        end

        if (st == S_HASH_SQUEEZE && xof_sq_valid) begin
            hash_buf_wr_en   = 1'b1;
            hash_buf_wr_addr = byte_idx[HASH_MEM_ADDR_W-1:0];
            hash_buf_wr_data = xof_sq_byte;
            if (uop_code == MLDSA_UOP_HASH_SIGN_W1) begin
                sign_hash_buf_wr_en   = 1'b1;
                sign_hash_buf_wr_addr = byte_idx[HASH_MEM_ADDR_W-1:0];
                sign_hash_buf_wr_data = xof_sq_byte;
            end
        end
    end

    always_comb begin
        if (ntt_busy) begin
            ram_rd0_en   = ntt_rd_en;
            ram_rd0_bank = ntt_rd_bank;
            ram_rd0_addr = ntt_rd_addr;
            ram_pair_rd0_en   = 1'b0;
            ram_pair_rd0_bank = '0;
            ram_pair_rd0_addr = 7'd0;
            ram_rd1_en   = 1'b0;
            ram_rd1_bank = '0;
            ram_rd1_addr = '0;
            ram_pair_rd1_en   = 1'b0;
            ram_pair_rd1_bank = '0;
            ram_pair_rd1_addr = 7'd0;
            ram_wr_en    = ntt_wr_en;
            ram_wr_bank  = ntt_wr_bank;
            ram_wr_addr  = ntt_wr_addr;
            ram_wr_data  = ntt_wr_data;
            ram_pair_wr_en   = 1'b0;
            ram_pair_wr_bank = '0;
            ram_pair_wr_addr = 7'd0;
            ram_pair_wr_data = 48'd0;
        end else if (poly_busy) begin
            ram_rd0_en   = poly_rd0_en;
            ram_rd0_bank = poly_rd0_bank;
            ram_rd0_addr = poly_rd0_addr;
            ram_pair_rd0_en   = poly_pair_rd0_en;
            ram_pair_rd0_bank = poly_pair_rd0_bank;
            ram_pair_rd0_addr = poly_pair_rd0_addr;
            ram_rd1_en   = poly_rd1_en;
            ram_rd1_bank = poly_rd1_bank;
            ram_rd1_addr = poly_rd1_addr;
            ram_pair_rd1_en   = poly_pair_rd1_en;
            ram_pair_rd1_bank = poly_pair_rd1_bank;
            ram_pair_rd1_addr = poly_pair_rd1_addr;
            ram_wr_en    = poly_wr_en;
            ram_wr_bank  = poly_wr_bank;
            ram_wr_addr  = poly_wr_addr;
            ram_wr_data  = poly_wr_data;
            ram_pair_wr_en   = poly_pair_wr_en;
            ram_pair_wr_bank = poly_pair_wr_bank;
            ram_pair_wr_addr = poly_pair_wr_addr;
            ram_pair_wr_data = poly_pair_wr_data;
        end else if (ucode_busy) begin
            ram_rd0_en   = ctrl_rd0_en;
            ram_rd0_bank = ctrl_rd0_bank;
            ram_rd0_addr = ctrl_rd0_addr;
            ram_pair_rd0_en   = 1'b0;
            ram_pair_rd0_bank = '0;
            ram_pair_rd0_addr = 7'd0;
            ram_rd1_en   = ctrl_rd1_en;
            ram_rd1_bank = ctrl_rd1_bank;
            ram_rd1_addr = ctrl_rd1_addr;
            ram_pair_rd1_en   = 1'b0;
            ram_pair_rd1_bank = '0;
            ram_pair_rd1_addr = 7'd0;
            ram_wr_en    = ctrl_wr_en;
            ram_wr_bank  = ctrl_wr_bank;
            ram_wr_addr  = ctrl_wr_addr;
            ram_wr_data  = ctrl_wr_data;
            ram_pair_wr_en   = 1'b0;
            ram_pair_wr_bank = '0;
            ram_pair_wr_addr = 7'd0;
            ram_pair_wr_data = 48'd0;
        end else begin
            ram_rd0_en   = host_rd_en;
            ram_rd0_bank = host_rd_bank;
            ram_rd0_addr = host_rd_addr;
            ram_pair_rd0_en   = 1'b0;
            ram_pair_rd0_bank = '0;
            ram_pair_rd0_addr = 7'd0;
            ram_rd1_en   = 1'b0;
            ram_rd1_bank = '0;
            ram_rd1_addr = '0;
            ram_pair_rd1_en   = 1'b0;
            ram_pair_rd1_bank = '0;
            ram_pair_rd1_addr = 7'd0;
            ram_wr_en    = load_poly_we;
            ram_wr_bank  = load_poly_bank;
            ram_wr_addr  = load_poly_addr;
            ram_wr_data  = load_poly_data;
            ram_pair_wr_en   = 1'b0;
            ram_pair_wr_bank = '0;
            ram_pair_wr_addr = 7'd0;
            ram_pair_wr_data = 48'd0;
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            st            <= S_WAIT_UOP;
            poly_idx      <= 3'd0;
            vec_i         <= 3'd0;
            vec_j         <= 3'd0;
            coeff_idx     <= 8'd0;
            seed_idx      <= 7'd0;
            byte_idx      <= 13'd0;
            byte_base     <= 13'd0;
            check_count   <= 11'd0;
            coeff_hold0   <= '0;
            coeff_hold1   <= '0;
            uop_step_done <= 1'b0;
            uop_step_retry<= 1'b0;
            uop_step_fail <= 1'b0;
            done          <= 1'b0;
            pass          <= 1'b0;
            error         <= 1'b0;
            err_code      <= 8'd0;
            busy          <= 1'b0;
            uop_dbg       <= 8'd0;
            state_dbg     <= 8'd0;
            op_pass_q     <= 1'b0;
            sign_retry_count <= 8'd0;
            uop_issued_q  <= 1'b0;
            uop_issued_code_q <= 8'd0;
            cfg_k_q       <= 4'd6;
            cfg_l_q       <= 4'd5;
            cfg_eta_q     <= 3'd4;
            cfg_tau_q     <= 6'd49;
            cfg_gamma1_q  <= 20'd524288;
            cfg_gamma2_q  <= 20'd261888;
            cfg_beta_q    <= 9'd196;
            cfg_omega_q   <= 8'd55;
            cfg_k_last_q  <= 3'd5;
            cfg_l_last_q  <= 3'd4;
            cfg_w1_pack_bytes_q <= 13'd4608;
            cfg_hint_pack_bytes_q <= 13'd192;
            cfg_z_total_coeffs_q <= 16'd1280;
            cfg_z_bound_q <= 24'd524092;
            cfg_gamma1_is_2p19_q <= 1'b1;
            standard_mode_q <= 1'b0;
            message_len_q <= 13'd0;
            check_pipe_a_q <= '0;
            check_pipe_b_q <= '0;
            check_pipe_valid_q <= 1'b0;
            check_pipe_issue_q <= 1'b0;
            check_pipe_dual_q <= 1'b0;
        end else begin
            uop_step_done <= 1'b0;
            uop_step_retry<= 1'b0;
            uop_step_fail <= 1'b0;
            done          <= 1'b0;
            error         <= 1'b0;
            state_dbg     <= st;
            uop_dbg       <= uop_code;
            busy          <= ucode_busy;
            check_pipe_valid_q <= 1'b0;
            check_pipe_issue_q <= check_pipe_issue;
            check_pipe_dual_q <= check_pipe_issue && ctrl_rd1_en;

            if (check_pipe_issue_q) begin
                check_pipe_a_q <= ram_rd0_data;
                check_pipe_b_q <= check_pipe_dual_q ? ram_rd1_data : '0;
                check_pipe_valid_q <= 1'b1;
            end

            if (ucode_done) begin
                done <= 1'b1;
                pass <= op_pass_q;
            end
            if (ucode_error) begin
                error <= 1'b1;
            end
            if (start && !ucode_busy) begin
                standard_mode_q <= standard_mode;
                message_len_q <= message_len;
                cfg_k_q       <= level_k;
                cfg_l_q       <= level_l;
                cfg_eta_q     <= level_eta;
                cfg_tau_q     <= level_tau;
                cfg_gamma1_q  <= level_gamma1;
                cfg_gamma2_q  <= level_gamma2;
                cfg_beta_q    <= level_beta;
                cfg_omega_q   <= level_omega;
                cfg_k_last_q  <= level_k[2:0] - 3'd1;
                cfg_l_last_q  <= level_l[2:0] - 3'd1;
                cfg_w1_pack_bytes_q <= level_k * W1_BYTES_PER_POLY;
                cfg_hint_pack_bytes_q <= level_k * HINT_BYTES_PER_POLY;
                cfg_z_total_coeffs_q <= level_l * MLDSA_N;
                cfg_z_bound_q <= {{4{1'b0}}, level_gamma1} - {{15{1'b0}}, level_beta};
                cfg_gamma1_is_2p19_q <= (level_gamma1 == 20'd524288);
                if (!level_valid) begin
                    err_code <= 8'h41;
                    done <= 1'b1;
                    pass <= 1'b0;
                    error <= 1'b1;
                end
            end
            if (!ucode_busy) begin
                uop_issued_q <= 1'b0;
            end else if (uop_code != uop_issued_code_q) begin
                uop_issued_q <= 1'b0;
            end

            if (load_seed_we && !ucode_busy) begin
                seed_mem[load_seed_addr] <= load_seed_data;
            end
            unique case (st)
                S_WAIT_UOP: begin
                    if (ucode_busy && (!uop_issued_q || (uop_code != uop_issued_code_q))) begin
                        uop_issued_q <= 1'b1;
                        uop_issued_code_q <= uop_code;
                        unique case (uop_code)
                            MLDSA_UOP_SEED_EXPAND: begin
                                if (standard_mode_q) st <= S_SEED_START;
                                else begin
                                    uop_step_done <= 1'b1;
                                    st <= S_WAIT_UOP;
                                end
                            end
                            MLDSA_UOP_EXPAND_A: begin
                                if (standard_mode_q) begin
                                    byte_idx <= 13'd0;
                                    vec_i <= 3'd0;
                                    vec_j <= 3'd0;
                                    st <= S_EXPANDA_RHO_LOAD;
                                end else begin
                                    uop_step_done <= 1'b1;
                                    st <= S_WAIT_UOP;
                                end
                            end
                            MLDSA_UOP_HASH_MU: begin
                                if (standard_mode_q && op_mode != MLDSA_OP_KEYGEN) begin
                                    byte_idx <= 13'd0;
                                    st <= S_STD_HASH_LOAD_A;
                                end else begin
                                    uop_step_done <= 1'b1;
                                    st <= S_WAIT_UOP;
                                end
                            end
                            MLDSA_UOP_PREP_T1: begin
                                sign_retry_count <= 8'd0;
                                op_pass_q <= 1'b0;
                                if (standard_mode_q && op_mode != MLDSA_OP_KEYGEN) begin
                                    uop_step_done <= 1'b1;
                                    st <= S_WAIT_UOP;
                                end else begin
                                    vec_i <= 3'd0;
                                    coeff_idx <= 8'd0;
                                    st <= S_T1_CLR;
                                end
                            end
                            MLDSA_UOP_SAMPLE_Y: begin
                                poly_idx <= 3'd0;
                                seed_idx <= 7'd0;
                                st <= S_Y_XOF_START;
                            end
                            MLDSA_UOP_MATVEC_AY,
                            MLDSA_UOP_MATVEC_AZ: begin
                                vec_i <= 3'd0;
                                coeff_idx <= 8'd0;
                                if (uop_code == MLDSA_UOP_MATVEC_AZ) op_pass_q <= 1'b0;
                                st <= S_MV_CLR;
                            end
                            MLDSA_UOP_PACK_W1_SIGN: begin
                                poly_idx <= 3'd0;
                                coeff_idx <= 8'd0;
                                st <= S_W1_DEC_READ;
                            end
                            MLDSA_UOP_PACK_W1_VERIFY: begin
                                poly_idx <= 3'd0;
                                coeff_idx <= 8'd0;
                                byte_idx <= 13'd0;
                                st <= S_PACKW_START;
                            end
                            MLDSA_UOP_HASH_SIGN_W1,
                            MLDSA_UOP_HASH_VERIFY_W1: begin
                                byte_idx <= 13'd0;
                                if (standard_mode_q) st <= S_STD_HASH_LOAD_A;
                                else st <= S_HASH_START;
                            end
                            MLDSA_UOP_SAMPLE_C_SIGN,
                            MLDSA_UOP_SAMPLE_C_VERIFY: begin
                                byte_idx <= 13'd0;
                                st <= S_CHAL_START;
                            end
                            MLDSA_UOP_MUL_CS1_ADD_Z: begin
                                poly_idx <= 3'd0;
                                vec_j <= 3'd0;
                                st <= S_Z_MUL_START;
                            end
                            MLDSA_UOP_CHECK_Z: begin
                                poly_idx <= 3'd0;
                                coeff_idx <= 8'd0;
                                st <= S_ZCHK_START;
                            end
                            MLDSA_UOP_MAKE_HINT: begin
                                poly_idx <= 3'd0;
                                coeff_idx <= 8'd0;
                                if (standard_mode_q) st <= S_HINT_CS2_MUL_START;
                                else st <= S_HINT_GEN_START;
                            end
                            MLDSA_UOP_PACK_HINT: begin
                                poly_idx <= 3'd0;
                                coeff_idx <= 8'd0;
                                byte_idx <= 13'd0;
                                st <= S_PACKH_START;
                            end
                            MLDSA_UOP_USE_HINT: begin
                                poly_idx <= 3'd0;
                                byte_idx <= 13'd0;
                                coeff_idx <= 8'd0;
                                st <= S_UNPACKH_START;
                            end
                            MLDSA_UOP_COMPARE_C: begin
                                coeff_idx <= 8'd0;
                                st <= S_CMPC_START;
                            end
                            default: begin
                                st <= S_WAIT_UOP;
                            end
                        endcase
                    end
                end

                S_SEED_START: begin
                    st <= S_SEED_WAIT;
                end
                S_SEED_WAIT: begin
                    if (seed_expand_error) begin
                        err_code <= 8'h42;
                        st <= S_UOP_FAIL;
                    end else if (seed_expand_done) begin
                        uop_step_done <= 1'b1;
                        st <= S_WAIT_UOP;
                    end
                end

                S_EXPANDA_RHO_LOAD: begin
                    if (byte_idx == 13'd31) begin
                        if (USE_RESIDENT_A) begin
                            byte_idx <= 13'd0;
                            st <= S_EXPANDA_START;
                        end else begin
                            uop_step_done <= 1'b1;
                            st <= S_WAIT_UOP;
                        end
                    end else begin
                        byte_idx <= byte_idx + 13'd1;
                    end
                end
                S_EXPANDA_START: begin
                    st <= S_EXPANDA_FILL;
                end
                S_EXPANDA_FILL: begin
                    if (expanda_error) begin
                        err_code <= 8'h43;
                        st <= S_UOP_FAIL;
                    end else if (expanda_done) begin
                        if (vec_j == cfg_l_last_q) begin
                            if (vec_i == cfg_k_last_q) begin
                                uop_step_done <= 1'b1;
                                st <= S_WAIT_UOP;
                            end else begin
                                vec_i <= vec_i + 3'd1;
                                vec_j <= 3'd0;
                                st <= S_EXPANDA_START;
                            end
                        end else begin
                            vec_j <= vec_j + 3'd1;
                            st <= S_EXPANDA_START;
                        end
                    end
                end
                S_T1_A_START: begin
                    st <= S_T1_A_FILL;
                end
                S_T1_A_FILL: begin
                    if (expanda_error) begin
                        err_code <= 8'h43;
                        st <= S_UOP_FAIL;
                    end else if (expanda_done) begin
                        st <= S_T1_MAC_START;
                    end
                end

                S_STD_HASH_LOAD_A: begin
                    if (byte_idx == 13'd63) begin
                        byte_idx <= 13'd0;
                        if (uop_code == MLDSA_UOP_HASH_MU && message_len_q == 13'd0) begin
                            st <= S_STD_HASH_START;
                        end else begin
                            st <= S_STD_HASH_LOAD_B;
                        end
                    end else begin
                        byte_idx <= byte_idx + 13'd1;
                    end
                end
                S_STD_HASH_LOAD_B: begin
                    if ((uop_code == MLDSA_UOP_HASH_MU && byte_idx == message_len_q - 13'd1) ||
                        (uop_code != MLDSA_UOP_HASH_MU && byte_idx == cfg_w1_pack_bytes_q - 13'd1)) begin
                        byte_idx <= 13'd0;
                        st <= S_STD_HASH_START;
                    end else begin
                        byte_idx <= byte_idx + 13'd1;
                    end
                end
                S_STD_HASH_START: begin
                    st <= S_STD_HASH_WAIT;
                end
                S_STD_HASH_WAIT: begin
                    if (std_hash_error) begin
                        err_code <= 8'h44;
                        st <= S_UOP_FAIL;
                    end else if (std_hash_done) begin
                        byte_idx <= 13'd0;
                        st <= S_STD_HASH_COPY;
                    end
                end
                S_STD_HASH_COPY: begin
                    if (uop_code == MLDSA_UOP_HASH_MU) begin
                        if (byte_idx == 13'd63) begin
                            uop_step_done <= 1'b1;
                            st <= S_WAIT_UOP;
                        end else begin
                            byte_idx <= byte_idx + 13'd1;
                        end
                    end else if (byte_idx == HASH_BUF_BYTES - 1) begin
                        uop_step_done <= 1'b1;
                        st <= S_WAIT_UOP;
                    end else begin
                        byte_idx <= byte_idx + 13'd1;
                    end
                end

                S_T1_CLR: begin
                    if (coeff_idx == 8'd255) begin
                        coeff_idx <= 8'd0;
                        if (vec_i == cfg_k_last_q) begin
                            vec_j <= 3'd0;
                            st <= S_T1_NTT_START;
                        end else begin
                            vec_i <= vec_i + 3'd1;
                        end
                    end else begin
                        coeff_idx <= coeff_idx + 8'd1;
                    end
                end
                S_T1_NTT_START: begin
                    st <= S_T1_NTT_WAIT;
                end
                S_T1_NTT_WAIT: begin
                    if (ntt_done) begin
                        if (vec_j == cfg_l_last_q) begin
                            vec_i <= 3'd0;
                            vec_j <= 3'd0;
                            st <= USE_RESIDENT_A ? S_T1_MAC_START : S_T1_A_START;
                        end else begin
                            vec_j <= vec_j + 3'd1;
                            st <= S_T1_NTT_START;
                        end
                    end
                end
                S_T1_MAC_START: begin
                    st <= S_T1_MAC_WAIT;
                end
                S_T1_MAC_WAIT: begin
                    if (poly_done) begin
                        if (vec_j == cfg_l_last_q) begin
                            if (vec_i == cfg_k_last_q) begin
                                if (op_mode == MLDSA_OP_KEYGEN) op_pass_q <= 1'b1;
                                uop_step_done <= 1'b1;
                                st <= S_WAIT_UOP;
                            end else begin
                                vec_i <= vec_i + 3'd1;
                                vec_j <= 3'd0;
                                st <= USE_RESIDENT_A ? S_T1_MAC_START : S_T1_A_START;
                            end
                        end else begin
                            vec_j <= vec_j + 3'd1;
                            st <= USE_RESIDENT_A ? S_T1_MAC_START : S_T1_A_START;
                        end
                    end
                end

                S_Y_XOF_START: begin
                    seed_idx <= 7'd0;
                    st <= S_Y_ABSORB;
                end
                S_Y_ABSORB: begin
                    if (xof_abs_ready) begin
                        if ((!standard_mode_q && seed_idx == 7'd33) ||
                            (standard_mode_q && seed_idx == 7'd65)) begin
                            st <= S_Y_SAMPLER_START;
                        end else begin
                            seed_idx <= seed_idx + 7'd1;
                        end
                    end
                end
                S_Y_SAMPLER_START: begin
                    st <= S_Y_SQUEEZE;
                end
                S_Y_SQUEEZE: begin
                    if (sampler_error) begin
                        err_code <= 8'h31;
                        st <= S_UOP_FAIL;
                    end else if (sampler_done) begin
                        st <= S_Y_STOP;
                    end
                end
                S_Y_STOP: begin
                    if (poly_idx == cfg_l_last_q) begin
                        uop_step_done <= 1'b1;
                        st <= S_WAIT_UOP;
                    end else begin
                        poly_idx <= poly_idx + 3'd1;
                        seed_idx <= 7'd0;
                        st <= S_Y_XOF_START;
                    end
                end

                S_MV_CLR: begin
                    if (coeff_idx == 8'd255) begin
                        coeff_idx <= 8'd0;
                        if (vec_i == cfg_k_last_q) begin
                            vec_j <= 3'd0;
                            st <= S_MV_NTT_START;
                        end else begin
                            vec_i <= vec_i + 3'd1;
                        end
                    end else begin
                        coeff_idx <= coeff_idx + 8'd1;
                    end
                end
                S_MV_NTT_START: begin
                    st <= S_MV_NTT_WAIT;
                end
                S_MV_NTT_WAIT: begin
                    if (ntt_done) begin
                        vec_i <= 3'd0;
                        st <= USE_RESIDENT_A ? S_MV_MAC_START : S_MV_A_START;
                    end
                end
                S_MV_A_START: begin
                    st <= S_MV_A_FILL;
                end
                S_MV_A_FILL: begin
                    if (expanda_error) begin
                        err_code <= 8'h43;
                        st <= S_UOP_FAIL;
                    end else if (expanda_done) begin
                        st <= S_MV_MAC_START;
                    end
                end
                S_MV_MAC_START: begin
                    st <= S_MV_MAC_WAIT;
                end
                S_MV_MAC_WAIT: begin
                    if (poly_done) begin
                        if (vec_i == cfg_k_last_q) begin
                            if (vec_j == cfg_l_last_q) begin
                                vec_i <= 3'd0;
                                if (uop_code == MLDSA_UOP_MATVEC_AY) begin
                                    st <= S_MV_INV_START;
                                end else begin
                                    vec_j <= 3'd0;
                                    st <= S_MV_INV_START;
                                end
                            end else begin
                                vec_j <= vec_j + 3'd1;
                                st <= S_MV_NTT_START;
                            end
                        end else begin
                            vec_i <= vec_i + 3'd1;
                            st <= USE_RESIDENT_A ? S_MV_MAC_START : S_MV_A_START;
                        end
                    end
                end
                S_MV_INV_START: begin
                    st <= S_MV_INV_WAIT;
                end
                S_MV_INV_WAIT: begin
                    if (uop_code == MLDSA_UOP_MATVEC_AY) begin
                        if (ntt_done) begin
                            if (vec_i == cfg_k_last_q) begin
                                uop_step_done <= 1'b1;
                                st <= S_WAIT_UOP;
                            end else begin
                                vec_i <= vec_i + 3'd1;
                                st <= S_MV_INV_START;
                            end
                        end
                    end else begin
                        if (vec_j == 3'd0 && poly_done) begin
                            vec_j <= 3'd1;
                            st <= S_MV_INV_START;
                        end else if (vec_j == 3'd1 && poly_done) begin
                            vec_j <= 3'd2;
                            st <= S_MV_INV_START;
                        end else if (vec_j == 3'd2 && ntt_done) begin
                            if (vec_i == cfg_k_last_q) begin
                                uop_step_done <= 1'b1;
                                st <= S_WAIT_UOP;
                            end else begin
                                vec_i <= vec_i + 3'd1;
                                vec_j <= 3'd0;
                                st <= S_MV_INV_START;
                            end
                        end
                    end
                end

                S_W1_DEC_READ: begin
                    st <= S_W1_DEC_START;
                end
                S_W1_DEC_START: begin
                    st <= S_W1_DEC_WAIT;
                end
                S_W1_DEC_WAIT: begin
                    if (dh_done) begin
                        st <= S_W1_DEC_WRITE;
                    end
                end
                S_W1_DEC_WRITE: begin
                    if (coeff_idx == 8'd255) begin
                        coeff_idx <= 8'd0;
                        byte_idx <= 13'd0;
                        st <= S_PACKW_START;
                    end else begin
                        coeff_idx <= coeff_idx + 8'd1;
                        st <= S_W1_DEC_READ;
                    end
                end
                S_PACKW_START: begin
                    st <= S_PACKW_READ;
                end
                S_PACKW_READ: begin
                    st <= S_PACKW_PUSH;
                end
                S_PACKW_PUSH: begin
                    if (pack_byte_out_valid) begin
                        byte_idx <= byte_idx + 13'd1;
                    end
                    if (pack_coeff_in_ready) begin
                        if (coeff_idx == 8'd255) st <= S_PACKW_WAIT;
                        else begin
                            coeff_idx <= coeff_idx + 8'd1;
                            st <= S_PACKW_READ;
                        end
                    end
                end
                S_PACKW_WAIT: begin
                    if (pack_byte_out_valid) begin
                        byte_idx <= byte_idx + 13'd1;
                    end
                    if (pack_done) begin
                        if (poly_idx == cfg_k_last_q) begin
                            uop_step_done <= 1'b1;
                            st <= S_WAIT_UOP;
                        end else begin
                            poly_idx <= poly_idx + 3'd1;
                            coeff_idx <= 8'd0;
                            byte_idx <= 13'd0;
                            st <= S_W1_DEC_READ;
                        end
                    end else if (pack_error) begin
                        err_code <= 8'h32;
                        st <= S_UOP_FAIL;
                    end
                end

                S_HASH_START: begin
                    byte_idx <= 13'd0;
                    st <= S_HASH_ABSORB;
                end
                S_HASH_ABSORB: begin
                    if (xof_abs_ready) begin
                        if (byte_idx == cfg_w1_pack_bytes_q - 13'd1) begin
                            byte_idx <= 13'd0;
                            st <= S_HASH_SQUEEZE;
                        end else begin
                            byte_idx <= byte_idx + 13'd1;
                        end
                    end
                end
                S_HASH_SQUEEZE: begin
                    if (xof_sq_valid) begin
                        if (byte_idx == HASH_BUF_BYTES - 1) begin
                            st <= S_HASH_STOP;
                        end else begin
                            byte_idx <= byte_idx + 13'd1;
                        end
                    end
                end
                S_HASH_STOP: begin
                    uop_step_done <= 1'b1;
                    st <= S_WAIT_UOP;
                end

                S_CHAL_START: begin
                    byte_idx <= 13'd0;
                    st <= S_CHAL_FEED;
                end
                S_CHAL_FEED: begin
                    if (sampler_coeff_valid && sampler_coeff_index == 8'd255) begin
                        if (uop_code == MLDSA_UOP_SAMPLE_C_SIGN) begin
                            st <= S_CHAL_NTT_START;
                        end else begin
                            uop_step_done <= 1'b1;
                            st <= S_WAIT_UOP;
                        end
                    end else if (sampler_error || byte_idx == HASH_BUF_BYTES - 1) begin
                        if (!sampler_done) begin
                            err_code <= 8'h33;
                            st <= S_UOP_FAIL;
                        end
                    end else if (sampler_byte_ready) begin
                        byte_idx <= byte_idx + 13'd1;
                    end
                end
                S_CHAL_NTT_START: begin
                    st <= S_CHAL_NTT_WAIT;
                end
                S_CHAL_NTT_WAIT: begin
                    if (ntt_done) begin
                        uop_step_done <= 1'b1;
                        st <= S_WAIT_UOP;
                    end
                end

                S_Z_MUL_START: begin
                    st <= S_Z_MUL_WAIT;
                end
                S_Z_MUL_WAIT: begin
                    if ((vec_j == 3'd0 && poly_done) || (vec_j == 3'd1 && ntt_done) || (vec_j == 3'd2 && poly_done)) begin
                        if (vec_j == 3'd2) begin
                            if (poly_idx == cfg_l_last_q) begin
                                uop_step_done <= 1'b1;
                                st <= S_WAIT_UOP;
                            end else begin
                                poly_idx <= poly_idx + 3'd1;
                                vec_j <= 3'd0;
                                st <= S_Z_MUL_START;
                            end
                        end else begin
                            vec_j <= vec_j + 3'd1;
                            st <= S_Z_MUL_START;
                        end
                    end
                end

                S_ZCHK_START: begin
                    st <= S_ZCHK_RDWAIT;
                end
                S_ZCHK_RDWAIT: begin
                    if (check_pipe_valid_q) begin
                        st <= S_ZCHK_FEED;
                    end
                end
                S_ZCHK_FEED: begin
                    if (check_pipe_valid_q) begin
                        if (poly_idx == cfg_l_last_q && coeff_idx == 8'd255) begin
                            st <= S_ZCHK_WAIT;
                        end else if (coeff_idx == 8'd255) begin
                            poly_idx <= poly_idx + 3'd1;
                            coeff_idx <= 8'd0;
                            st <= S_ZCHK_RDWAIT;
                        end else begin
                            coeff_idx <= coeff_idx + 8'd1;
                            st <= S_ZCHK_RDWAIT;
                        end
                    end
                end
                S_ZCHK_WAIT: begin
                    if (check_done) begin
                        if (!check_pass) begin
                            if (sign_retry_count == SIGN_RETRY_MAX) begin
                                err_code <= 8'h38;
                                st <= S_UOP_FAIL;
                            end else begin
                                sign_retry_count <= sign_retry_count + 8'd1;
                                uop_step_retry <= 1'b1;
                                st <= S_WAIT_UOP;
                            end
                        end else begin
                            op_pass_q <= 1'b1;
                            uop_step_done <= 1'b1;
                            st <= S_WAIT_UOP;
                        end
                    end
                end

                S_HINT_CS2_MUL_START: begin
                    st <= S_HINT_CS2_MUL_WAIT;
                end
                S_HINT_CS2_MUL_WAIT: begin
                    if (poly_done) st <= S_HINT_CS2_INV_START;
                end
                S_HINT_CS2_INV_START: begin
                    st <= S_HINT_CS2_INV_WAIT;
                end
                S_HINT_CS2_INV_WAIT: begin
                    if (ntt_done) st <= S_HINT_WMINUS_START;
                end
                S_HINT_WMINUS_START: begin
                    st <= S_HINT_WMINUS_WAIT;
                end
                S_HINT_WMINUS_WAIT: begin
                    if (poly_done) st <= S_HINT_CT0_MUL_START;
                end
                S_HINT_CT0_MUL_START: begin
                    st <= S_HINT_CT0_MUL_WAIT;
                end
                S_HINT_CT0_MUL_WAIT: begin
                    if (poly_done) st <= S_HINT_CT0_INV_START;
                end
                S_HINT_CT0_INV_START: begin
                    st <= S_HINT_CT0_INV_WAIT;
                end
                S_HINT_CT0_INV_WAIT: begin
                    if (ntt_done) begin
                        coeff_idx <= 8'd0;
                        st <= S_HINT_GEN_READ;
                    end
                end
                S_HINT_GEN_READ: begin
                    st <= S_HINT_GEN_START;
                end
                S_HINT_GEN_START: begin
                    st <= S_HINT_GEN_WAIT;
                end
                S_HINT_GEN_WAIT: begin
                    if (dh_done) st <= S_HINT_GEN_WRITE;
                end
                S_HINT_GEN_WRITE: begin
                    if (coeff_idx == 8'd255) begin
                        coeff_idx <= 8'd0;
                        if (poly_idx == cfg_k_last_q) begin
                            poly_idx <= 3'd0;
                            st <= S_HINTCNT_START;
                        end else begin
                            poly_idx <= poly_idx + 3'd1;
                            if (standard_mode_q) st <= S_HINT_CS2_MUL_START;
                            else st <= S_HINT_GEN_START;
                        end
                    end else begin
                        coeff_idx <= coeff_idx + 8'd1;
                        if (standard_mode_q) st <= S_HINT_GEN_READ;
                        else st <= S_HINT_GEN_START;
                    end
                end

                S_HINTCNT_START: begin
                    poly_idx <= 3'd0;
                    coeff_idx <= 8'd0;
                    st <= S_HINTCNT_RDWAIT;
                end
                S_HINTCNT_RDWAIT: begin
                    if (check_pipe_valid_q) begin
                        st <= S_HINTCNT_FEED;
                    end
                end
                S_HINTCNT_FEED: begin
                    if (check_pipe_valid_q) begin
                        if (poly_idx == cfg_k_last_q && coeff_idx == 8'd255) begin
                            st <= S_HINTCNT_WAIT;
                        end else if (coeff_idx == 8'd255) begin
                            poly_idx <= poly_idx + 3'd1;
                            coeff_idx <= 8'd0;
                            st <= S_HINTCNT_RDWAIT;
                        end else begin
                            coeff_idx <= coeff_idx + 8'd1;
                            st <= S_HINTCNT_RDWAIT;
                        end
                    end
                end
                S_HINTCNT_WAIT: begin
                    if (check_done) begin
                        if (!check_pass) begin
                            if (sign_retry_count == SIGN_RETRY_MAX) begin
                                err_code <= 8'h39;
                                st <= S_UOP_FAIL;
                            end else begin
                                sign_retry_count <= sign_retry_count + 8'd1;
                                uop_step_retry <= 1'b1;
                                st <= S_WAIT_UOP;
                            end
                        end else begin
                            uop_step_done <= 1'b1;
                            st <= S_WAIT_UOP;
                        end
                    end
                end

                S_PACKH_START: begin
                    st <= S_PACKH_READ;
                end
                S_PACKH_READ: begin
                    st <= S_PACKH_PUSH;
                end
                S_PACKH_PUSH: begin
                    if (pack_byte_out_valid) begin
                        hint_bytes[(poly_idx * HINT_BYTES_PER_POLY) + byte_idx] <= pack_byte_out;
                        byte_idx <= byte_idx + 13'd1;
                    end
                    if (pack_coeff_in_ready) begin
                        if (coeff_idx == 8'd255) st <= S_PACKH_WAIT;
                        else begin
                            coeff_idx <= coeff_idx + 8'd1;
                            st <= S_PACKH_READ;
                        end
                    end
                end
                S_PACKH_WAIT: begin
                    if (pack_byte_out_valid) begin
                        hint_bytes[(poly_idx * HINT_BYTES_PER_POLY) + byte_idx] <= pack_byte_out;
                        byte_idx <= byte_idx + 13'd1;
                    end
                    if (pack_done) begin
                        if (poly_idx == cfg_k_last_q) begin
                            uop_step_done <= 1'b1;
                            st <= S_WAIT_UOP;
                        end else begin
                            poly_idx <= poly_idx + 3'd1;
                            coeff_idx <= 8'd0;
                            byte_idx <= 13'd0;
                            st <= S_PACKH_START;
                        end
                    end else if (pack_error) begin
                        err_code <= 8'h35;
                        st <= S_UOP_FAIL;
                    end
                end

                S_UNPACKH_START: begin
                    byte_base <= poly_idx * HINT_BYTES_PER_POLY;
                    byte_idx <= 13'd0;
                    coeff_idx <= 8'd0;
                    st <= S_UNPACKH_FEED;
                end
                S_UNPACKH_FEED: begin
                    if (pack_coeff_out_valid) begin
                        coeff_idx <= coeff_idx + 8'd1;
                    end
                    if (pack_byte_in_ready) begin
                        if (byte_idx == HINT_BYTES_PER_POLY - 1) st <= S_UNPACKH_WAIT;
                        else byte_idx <= byte_idx + 13'd1;
                    end
                end
                S_UNPACKH_WAIT: begin
                    if (pack_coeff_out_valid) begin
                        coeff_idx <= coeff_idx + 8'd1;
                    end
                    if (pack_done) begin
                        if (poly_idx == cfg_k_last_q) begin
                            poly_idx <= 3'd0;
                            coeff_idx <= 8'd0;
                            st <= S_USEH_READ;
                        end else begin
                            poly_idx <= poly_idx + 3'd1;
                            st <= S_UNPACKH_START;
                        end
                    end else if (pack_error) begin
                        err_code <= 8'h36;
                        st <= S_UOP_FAIL;
                    end
                end

                S_USEH_READ: begin
                    st <= S_USEH_START;
                end
                S_USEH_START: begin
                    st <= S_USEH_WAIT;
                end
                S_USEH_WAIT: begin
                    if (dh_done) st <= S_USEH_WRITE;
                end
                S_USEH_WRITE: begin
                    if (coeff_idx == 8'd255) begin
                        coeff_idx <= 8'd0;
                        if (poly_idx == cfg_k_last_q) begin
                            uop_step_done <= 1'b1;
                            st <= S_WAIT_UOP;
                        end else begin
                            poly_idx <= poly_idx + 3'd1;
                            st <= S_USEH_READ;
                        end
                    end else begin
                        coeff_idx <= coeff_idx + 8'd1;
                        st <= S_USEH_READ;
                    end
                end

                S_CMPC_START: begin
                    coeff_idx <= 8'd0;
                    st <= S_CMPC_RDWAIT;
                end
                S_CMPC_RDWAIT: begin
                    if (check_pipe_valid_q) begin
                        st <= S_CMPC_FEED;
                    end
                end
                S_CMPC_FEED: begin
                    if (check_pipe_valid_q) begin
                        if (coeff_idx == 8'd255) begin
                            st <= S_CMPC_WAIT;
                        end else begin
                            coeff_idx <= coeff_idx + 8'd1;
                            st <= S_CMPC_RDWAIT;
                        end
                    end
                end
                S_CMPC_WAIT: begin
                    if (check_done) begin
                        if (!check_pass) begin
                            op_pass_q <= 1'b0;
                            err_code <= 8'h37;
                        end else begin
                            op_pass_q <= 1'b1;
                        end
                        uop_step_done <= 1'b1;
                        st <= S_WAIT_UOP;
                    end
                end

                S_UOP_FAIL: begin
                    uop_step_fail <= 1'b1;
                    op_pass_q <= 1'b0;
                    st <= S_WAIT_UOP;
                end

                default: st <= S_WAIT_UOP;
            endcase
        end
    end

    assign host_rd_data = ram_rd0_data;
endmodule

`default_nettype wire
