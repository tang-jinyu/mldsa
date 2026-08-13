`timescale 1ns/1ps
`default_nettype none

package mldsa_fused_bank_map_pkg;
    localparam int MLDSA_FUSED_BANKS_44 = 40;
    localparam int MLDSA_FUSED_BANKS_65 = 65;
    localparam int MLDSA_FUSED_BANKS_87 = 73;

    localparam int MLDSA_FUSED_ADDR_W = 8;  // bank内地址8位 → 深度256 = N
    localparam int MLDSA_FUSED_COEFF_W = 24;// 系数24bit → 覆盖 q = 8,380,417 < 2^23

//存储架构设计：4个生命周期槽位，每个槽8个bank，任何向量最多8个多项式
/*
         KeyGen         Sign           Verify
槽0 S1:  s1          →  y            →  z
槽1 S2:  s2          →  (t0解包用)
槽2 T1:  t1输出      →  w=A·y → w1   →  w'=A·z-c·t1 → w1
槽3 ACC: 累加器      →  累加器/tmp   →  累加器/tmp
*/
    typedef enum logic [2:0] {
        FUSED_SLOT_S1      = 3'd0,// 槽0：s1 / y / z
        FUSED_SLOT_S2_T0   = 3'd1, // 槽1：s2 或 t0
        FUSED_SLOT_T1_W_W1 = 3'd2,  // 槽2：t1 / w / w1
        FUSED_SLOT_ACC_TMP = 3'd3 // 槽3：累加器/临时
    } fused_slot_e;


//四个 base 函数全部返回与等级无关的常数：槽基址恒为 0 / 8 / 16 / 24
//槽基地址不随等级收缩，因为物理上综合最大就是需要87挡位的40bank，44档空着的bank并不会消失
//基址恒定只变总数：地址生成逻辑0成本，换来小等级下槽内部分bank限制，是用存储利用率换控制简洁
    function automatic logic [5:0] fused_bank_count(input logic [1:0] level_sel);
        begin
            unique case (level_sel)
                2'd0: fused_bank_count = MLDSA_FUSED_BANKS_44[5:0];
                2'd1: fused_bank_count = MLDSA_FUSED_BANKS_65[5:0];
                2'd2: fused_bank_count = MLDSA_FUSED_BANKS_87[5:0];
                default: fused_bank_count = MLDSA_FUSED_BANKS_65[5:0];
            endcase
        end
    endfunction

    function automatic logic [5:0] fused_s1_base(input logic [1:0] level_sel);
        begin
            case (level_sel)
                2'd0: fused_s1_base = 6'd0;
                2'd1: fused_s1_base = 6'd0;
                2'd2: fused_s1_base = 6'd0;
                default: fused_s1_base = 6'd0;
            endcase
        end
    endfunction

    function automatic logic [5:0] fused_s2_t0_base(input logic [1:0] level_sel);
        begin
            case (level_sel)
                2'd0: fused_s2_t0_base = 6'd8;
                2'd1: fused_s2_t0_base = 6'd8;
                2'd2: fused_s2_t0_base = 6'd8;
                default: fused_s2_t0_base = 6'd8;
            endcase
        end
    endfunction

    function automatic logic [5:0] fused_t1_w_w1_base(input logic [1:0] level_sel);
        begin
            case (level_sel)
                2'd0: fused_t1_w_w1_base = 6'd16;
                2'd1: fused_t1_w_w1_base = 6'd16;
                2'd2: fused_t1_w_w1_base = 6'd16;
                default: fused_t1_w_w1_base = 6'd16;
            endcase
        end
    endfunction

    function automatic logic [5:0] fused_acc_tmp_base(input logic [1:0] level_sel);
        begin
            case (level_sel)
                2'd0: fused_acc_tmp_base = 6'd24;
                2'd1: fused_acc_tmp_base = 6'd24;
                2'd2: fused_acc_tmp_base = 6'd24;
                default: fused_acc_tmp_base = 6'd24;
            endcase
        end
    endfunction
endpackage

`default_nettype wire
