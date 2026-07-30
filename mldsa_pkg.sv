/*
作者：唐金钰
时间：2026/7/28
概述：参数包
*/

`timescale 1ns/1ps
`default_nettype none

package mldsa_pkg;
    localparam int MLDSA_LEVEL_NUM = 3;
    localparam int MLDSA_N         = 256;
    localparam int MLDSA_Q         = 8380417;
    localparam int MLDSA_D         = 13; // Power2Round 丢弃的低位数
    localparam int MLDSA_COEFF_W   = 24;
    localparam int MLDSA_K_MAX     = 8;
    localparam int MLDSA_L_MAX     = 7;
    localparam int MLDSA_MUL_W     = 2 * MLDSA_COEFF_W;
    localparam int MLDSA_GAMMA_W   = 20;
    localparam int MLDSA_BETA_W    = 9;
    localparam int MLDSA_OMEGA_W   = 8;
    localparam int MLDSA_TAU_W     = 6;
    localparam int MLDSA_ETA_W     = 3;

    typedef logic [MLDSA_COEFF_W-1:0] mldsa_coeff_t; // 无符号：规范模q系数，值域[0, q)
    typedef logic signed [MLDSA_COEFF_W-1:0] mldsa_scoeff_t; // 有符号：二补码sideband
    //规范系数活在模 q 环上，sideband 量活在整数意义上（要比较绝对值大小做范数检查）
    typedef logic [MLDSA_MUL_W-1:0]   mldsa_prod_t;
    typedef mldsa_coeff_t             mldsa_poly_t [0:MLDSA_N-1];
    typedef mldsa_poly_t              mldsa_vec_kmax_t [0:MLDSA_K_MAX-1];
    typedef mldsa_poly_t              mldsa_vec_lmax_t [0:MLDSA_L_MAX-1];
    typedef mldsa_poly_t              mldsa_mat_kxlmax_t [0:MLDSA_K_MAX-1][0:MLDSA_L_MAX-1];

    localparam mldsa_coeff_t MLDSA_Q_COEFF = mldsa_coeff_t'(MLDSA_Q);

    localparam logic [1:0] MLDSA_LEVEL_44 = 2'd0;
    localparam logic [1:0] MLDSA_LEVEL_65 = 2'd1;
    localparam logic [1:0] MLDSA_LEVEL_87 = 2'd2;

    localparam int MLDSA_LEVEL_K_TAB        [0:MLDSA_LEVEL_NUM-1] = '{4, 6, 8};
    localparam int MLDSA_LEVEL_L_TAB        [0:MLDSA_LEVEL_NUM-1] = '{4, 5, 7};
    localparam int MLDSA_LEVEL_ETA_TAB      [0:MLDSA_LEVEL_NUM-1] = '{2, 4, 2};
    localparam int MLDSA_LEVEL_TAU_TAB      [0:MLDSA_LEVEL_NUM-1] = '{39, 49, 60};
    localparam int MLDSA_LEVEL_GAMMA1_TAB   [0:MLDSA_LEVEL_NUM-1] = '{131072, 524288, 524288};
    localparam int MLDSA_LEVEL_GAMMA2_TAB   [0:MLDSA_LEVEL_NUM-1] = '{95232, 261888, 261888};
    localparam int MLDSA_LEVEL_BETA_TAB     [0:MLDSA_LEVEL_NUM-1] = '{78, 196, 120};
    localparam int MLDSA_LEVEL_OMEGA_TAB    [0:MLDSA_LEVEL_NUM-1] = '{80, 55, 75};
    localparam int MLDSA_LEVEL_PK_BYTES_TAB [0:MLDSA_LEVEL_NUM-1] = '{1312, 1952, 2592};
    localparam int MLDSA_LEVEL_SK_BYTES_TAB [0:MLDSA_LEVEL_NUM-1] = '{2560, 4032, 4896};
    localparam int MLDSA_LEVEL_SIG_BYTES_TAB[0:MLDSA_LEVEL_NUM-1] = '{2420, 3309, 4627};

    localparam logic [1:0] MLDSA_OP_KEYGEN = 2'd0;
    localparam logic [1:0] MLDSA_OP_SIGN   = 2'd1;
    localparam logic [1:0] MLDSA_OP_VERIFY = 2'd2;

//采样子操作，mldsa_sampler的mode输入
    localparam logic [1:0] MLDSA_SAMPLE_UNIFORM   = 2'd0;//均匀
    localparam logic [1:0] MLDSA_SAMPLE_ETA       = 2'd1;//η分布
    localparam logic [1:0] MLDSA_SAMPLE_GAMMA1    = 2'd2;//掩码y
    localparam logic [1:0] MLDSA_SAMPLE_CHALLENGE = 2'd3;//挑战c

//多项式ALU操作
    localparam logic [2:0] MLDSA_POLY_COPY = 3'd0;
    localparam logic [2:0] MLDSA_POLY_ADD  = 3'd1;
    localparam logic [2:0] MLDSA_POLY_SUB  = 3'd2;
    localparam logic [2:0] MLDSA_POLY_MUL  = 3'd3;
    localparam logic [2:0] MLDSA_POLY_MAC  = 3'd4;//乘加

//分解、提示操作
    localparam logic [1:0] MLDSA_DH_POWER2ROUND = 2'd0;
    localparam logic [1:0] MLDSA_DH_DECOMPOSE   = 2'd1;
    localparam logic [1:0] MLDSA_DH_MAKE_HINT   = 2'd2;
    localparam logic [1:0] MLDSA_DH_USE_HINT    = 2'd3;

//检查操作
    localparam logic [1:0] MLDSA_CHECK_NORM     = 2'd0;//范数
    localparam logic [1:0] MLDSA_CHECK_HINTCNT  = 2'd1;//Hint计数
    localparam logic [1:0] MLDSA_CHECK_BYTEEQ   = 2'd2;//字节比较
    localparam logic [1:0] MLDSA_CHECK_COEFFEQ  = 2'd3;//系数比较

    localparam logic [2:0] MLDSA_PACK_COEFF24_PACK   = 3'd0;
    localparam logic [2:0] MLDSA_PACK_COEFF24_UNPACK = 3'd1;
    localparam logic [2:0] MLDSA_PACK_BIT1_PACK      = 3'd2;
    localparam logic [2:0] MLDSA_PACK_BIT1_UNPACK    = 3'd3;
    localparam logic [2:0] MLDSA_PACK_NIBBLE4_PACK   = 3'd4;
    localparam logic [2:0] MLDSA_PACK_NIBBLE4_UNPACK = 3'd5;
    localparam logic [2:0] MLDSA_PACK_BITS10_PACK    = 3'd6;
    localparam logic [2:0] MLDSA_PACK_BITS10_UNPACK  = 3'd7;

    localparam logic [2:0] MLDSA_FIPS_PACK_T1  = 3'd0;
    localparam logic [2:0] MLDSA_FIPS_PACK_T0  = 3'd1;
    localparam logic [2:0] MLDSA_FIPS_PACK_ETA = 3'd2;
    localparam logic [2:0] MLDSA_FIPS_PACK_Z   = 3'd3;

//UOP表格
/*映射到算法操作：
KeyGen:  SEED_EXPAND(14) → EXPAND_A(15) → PREP_T1(01) → KEYGEN_DONE(13)
Sign:    HASH_MU(16) → SAMPLE_Y(02) → MATVEC_AY(03) → PACK_W1_SIGN(04)
         → HASH_SIGN_W1(05) → SAMPLE_C_SIGN(06) → MUL_CS1_ADD_Z(07)
         → CHECK_Z(08) → MAKE_HINT(09) → PACK_HINT(0A) → SIGN_DONE(0B)
Verify:  EXPAND_A(15) → MATVEC_AZ(0C) → USE_HINT(0D) → PACK_W1_VERIFY(0E)
         → HASH_VERIFY_W1(0F) → SAMPLE_C_VERIFY(10) → COMPARE_C(11) → VERIFY_DONE(12)*/
    localparam logic [7:0] MLDSA_UOP_NOP             = 8'h00;
    localparam logic [7:0] MLDSA_UOP_PREP_T1         = 8'h01;
    localparam logic [7:0] MLDSA_UOP_SAMPLE_Y        = 8'h02;
    localparam logic [7:0] MLDSA_UOP_MATVEC_AY       = 8'h03;
    localparam logic [7:0] MLDSA_UOP_PACK_W1_SIGN    = 8'h04;
    localparam logic [7:0] MLDSA_UOP_HASH_SIGN_W1    = 8'h05;
    localparam logic [7:0] MLDSA_UOP_SAMPLE_C_SIGN   = 8'h06;
    localparam logic [7:0] MLDSA_UOP_MUL_CS1_ADD_Z   = 8'h07;
    localparam logic [7:0] MLDSA_UOP_CHECK_Z         = 8'h08;
    localparam logic [7:0] MLDSA_UOP_MAKE_HINT       = 8'h09;
    localparam logic [7:0] MLDSA_UOP_PACK_HINT       = 8'h0A;
    localparam logic [7:0] MLDSA_UOP_SIGN_DONE       = 8'h0B;
    localparam logic [7:0] MLDSA_UOP_MATVEC_AZ       = 8'h0C;
    localparam logic [7:0] MLDSA_UOP_USE_HINT        = 8'h0D;
    localparam logic [7:0] MLDSA_UOP_PACK_W1_VERIFY  = 8'h0E;
    localparam logic [7:0] MLDSA_UOP_HASH_VERIFY_W1  = 8'h0F;
    localparam logic [7:0] MLDSA_UOP_SAMPLE_C_VERIFY = 8'h10;
    localparam logic [7:0] MLDSA_UOP_COMPARE_C       = 8'h11;
    localparam logic [7:0] MLDSA_UOP_VERIFY_DONE     = 8'h12;
    localparam logic [7:0] MLDSA_UOP_KEYGEN_DONE     = 8'h13;
    localparam logic [7:0] MLDSA_UOP_SEED_EXPAND     = 8'h14;
    localparam logic [7:0] MLDSA_UOP_EXPAND_A        = 8'h15;
    localparam logic [7:0] MLDSA_UOP_HASH_MU         = 8'h16;

    function automatic int mldsa_level_index(input logic [1:0] level_sel);
        begin
            unique case (level_sel)
                MLDSA_LEVEL_44: mldsa_level_index = 0;
                MLDSA_LEVEL_65: mldsa_level_index = 1;
                MLDSA_LEVEL_87: mldsa_level_index = 2;
                default:        mldsa_level_index = 0;
            endcase
        end
    endfunction

    function automatic logic mldsa_level_valid(input logic [1:0] level_sel);
        begin
            unique case (level_sel)
                MLDSA_LEVEL_44,
                MLDSA_LEVEL_65,
                MLDSA_LEVEL_87: mldsa_level_valid = 1'b1;
                default:        mldsa_level_valid = 1'b0;
            endcase
        end
    endfunction

    function automatic int mldsa_level_k(input logic [1:0] level_sel);
        mldsa_level_k = MLDSA_LEVEL_K_TAB[mldsa_level_index(level_sel)];
    endfunction

    function automatic int mldsa_level_l(input logic [1:0] level_sel);
        mldsa_level_l = MLDSA_LEVEL_L_TAB[mldsa_level_index(level_sel)];
    endfunction

    function automatic int mldsa_level_eta(input logic [1:0] level_sel);
        mldsa_level_eta = MLDSA_LEVEL_ETA_TAB[mldsa_level_index(level_sel)];
    endfunction

    function automatic int mldsa_level_tau(input logic [1:0] level_sel);
        mldsa_level_tau = MLDSA_LEVEL_TAU_TAB[mldsa_level_index(level_sel)];
    endfunction

    function automatic int mldsa_level_gamma1(input logic [1:0] level_sel);
        mldsa_level_gamma1 = MLDSA_LEVEL_GAMMA1_TAB[mldsa_level_index(level_sel)];
    endfunction

    function automatic int mldsa_level_gamma2(input logic [1:0] level_sel);
        mldsa_level_gamma2 = MLDSA_LEVEL_GAMMA2_TAB[mldsa_level_index(level_sel)];
    endfunction

    function automatic int mldsa_level_beta(input logic [1:0] level_sel);
        mldsa_level_beta = MLDSA_LEVEL_BETA_TAB[mldsa_level_index(level_sel)];
    endfunction

    function automatic int mldsa_level_omega(input logic [1:0] level_sel);
        mldsa_level_omega = MLDSA_LEVEL_OMEGA_TAB[mldsa_level_index(level_sel)];
    endfunction

    function automatic int mldsa_level_pk_bytes(input logic [1:0] level_sel);
        mldsa_level_pk_bytes = MLDSA_LEVEL_PK_BYTES_TAB[mldsa_level_index(level_sel)];
    endfunction

    function automatic int mldsa_level_sk_bytes(input logic [1:0] level_sel);
        mldsa_level_sk_bytes = MLDSA_LEVEL_SK_BYTES_TAB[mldsa_level_index(level_sel)];
    endfunction

    function automatic int mldsa_level_sig_bytes(input logic [1:0] level_sel);
        mldsa_level_sig_bytes = MLDSA_LEVEL_SIG_BYTES_TAB[mldsa_level_index(level_sel)];
    endfunction

//模加
    function automatic mldsa_coeff_t modq_add(
        input mldsa_coeff_t a,
        input mldsa_coeff_t b
    );
        logic [MLDSA_COEFF_W:0] sum;
        begin
            sum = {1'b0, a} + {1'b0, b};
            if (sum >= {1'b0, MLDSA_Q_COEFF}) sum = sum - {1'b0, MLDSA_Q_COEFF};
            modq_add = sum[MLDSA_COEFF_W-1:0];
        end
    endfunction

//模减
    function automatic mldsa_coeff_t modq_sub(
        input mldsa_coeff_t a,
        input mldsa_coeff_t b
    );
        logic [MLDSA_COEFF_W:0] diff;
        begin
            if (a >= b) diff = {1'b0, a} - {1'b0, b};
            else diff = {1'b0, a} + {1'b0, MLDSA_Q_COEFF} - {1'b0, b};
            modq_sub = diff[MLDSA_COEFF_W-1:0];
        end
    endfunction

    function automatic mldsa_coeff_t modq_neg(input mldsa_coeff_t a);
        begin
            modq_neg = (a == '0) ? '0 : MLDSA_Q_COEFF - a;
        end
    endfunction

//在最多 ±4q 范围内收拢
    function automatic mldsa_coeff_t normalize_modq_small(
        input integer signed value
    );
        integer signed reduced;
        begin
            reduced = value;
            if (reduced < 0) reduced = reduced + MLDSA_Q;
            if (reduced < 0) reduced = reduced + MLDSA_Q;
            if (reduced < 0) reduced = reduced + MLDSA_Q;
            if (reduced < 0) reduced = reduced + MLDSA_Q;
            if (reduced >= MLDSA_Q) reduced = reduced - MLDSA_Q;
            if (reduced >= MLDSA_Q) reduced = reduced - MLDSA_Q;
            if (reduced >= MLDSA_Q) reduced = reduced - MLDSA_Q;
            if (reduced >= MLDSA_Q) reduced = reduced - MLDSA_Q;
            normalize_modq_small = mldsa_coeff_t'(reduced[MLDSA_COEFF_W-1:0]);
        end
    endfunction

//把 [0,q) 映射到 (−q/2, q/2] 的居中表示。范数检查的前置步骤——‖z‖∞ 比较的是 |居中值|，不是 [0,q) 里的原值
    function automatic integer signed center_modq(input mldsa_coeff_t a);
        integer signed centered;
        begin
            centered = $signed({8'd0, a});
            if (centered > (MLDSA_Q / 2)) centered = centered - MLDSA_Q;
            center_modq = centered;
        end
    endfunction

    function automatic integer signed center_small_modq_from_signed(
        input integer signed value
    );
        begin
            center_small_modq_from_signed = center_modq(normalize_modq_small(value));
        end
    endfunction

    // 仅用于 two's-complement sideband 辅助量，如 t0/w0/a0。
    // 规范模 q 系数始终使用 [0, q) 编码，不应走这组接口。
    function automatic integer signed decode_aux_s24(input mldsa_coeff_t a);
        begin
            decode_aux_s24 = $signed(a);
        end
    endfunction

    function automatic mldsa_coeff_t encode_aux_s24(input integer signed a);
        begin
            encode_aux_s24 = mldsa_coeff_t'(a[MLDSA_COEFF_W-1:0]);
        end
    endfunction

    function automatic integer signed decode_s24(input mldsa_coeff_t a);
        begin
            decode_s24 = decode_aux_s24(a);
        end
    endfunction

    function automatic mldsa_coeff_t encode_s24(input integer signed a);
        begin
            encode_s24 = encode_aux_s24(a);
        end
    endfunction
endpackage

`default_nettype wire
