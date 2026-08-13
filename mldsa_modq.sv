`timescale 1ns/1ps
`default_nettype none

/*
作者：mnt / Codex
功能：计算普通域乘法 (a*b) mod q, q = 8380417 = 2^23 - 2^13 + 1
实现：24x24 乘法 + 固定级 Solinas 折叠约减

说明：
- 原实现采用 48-cycle 串行移位减法，是 NTT/BFU 的主要周期瓶颈。
- 这里利用 2^23 == 2^13 - 1 (mod q)，将 48-bit 普通乘积固定折叠回 q 域。
- 这不是 Montgomery 域乘法；输入/输出均为普通 q 域系数。
- 全路径无除法、无取模运算、无数据相关循环，适合 ASIC 综合。
*/
import mldsa_pkg::*;

module mldsa_mul24x24 (
    input  wire [MLDSA_COEFF_W-1:0] a,
    input  wire [MLDSA_COEFF_W-1:0] b,
    output wire [47:0]              p
);
`ifdef MLDSA_TARGET_FPGA
    (* use_dsp = "yes" *) wire [47:0] product_dsp = a * b;
    assign p = product_dsp;
`else
    assign p = a * b;
`endif
endmodule

module mldsa_reduce_q_solinas (
    input  wire [47:0]                  x,
    output logic [MLDSA_COEFF_W-1:0]    r
);
    logic [37:0] fold1;
    logic [28:0] fold2;
    logic [24:0] fold3;
    logic [24:0] fold4;
    logic [24:0] corr0;

    function automatic logic [37:0] mul_8191_25(input logic [24:0] h);
        logic [37:0] hh;
        begin
            hh = {13'd0, h};
            mul_8191_25 = (hh << 13) - hh;
        end
    endfunction

    function automatic logic [28:0] mul_8191_15(input logic [14:0] h);
        logic [28:0] hh;
        begin
            hh = {14'd0, h};
            mul_8191_15 = (hh << 13) - hh;
        end
    endfunction

    function automatic logic [24:0] mul_8191_6(input logic [5:0] h);
        logic [24:0] hh;
        begin
            hh = {19'd0, h};
            mul_8191_6 = (hh << 13) - hh;
        end
    endfunction

    function automatic logic [24:0] mul_8191_2(input logic [1:0] h);
        logic [24:0] hh;
        begin
            hh = {23'd0, h};
            mul_8191_2 = (hh << 13) - hh;
        end
    endfunction

    always_comb begin
        fold1 = {15'd0, x[22:0]}     + mul_8191_25(x[47:23]);
        fold2 = {6'd0,  fold1[22:0]} + mul_8191_15(fold1[37:23]);
        fold3 = {2'd0,  fold2[22:0]} + mul_8191_6(fold2[28:23]);
        fold4 = {2'd0,  fold3[22:0]} + mul_8191_2(fold3[24:23]);

        // Four folds bound any 48-bit input below 2q, so one correction is enough.
        corr0 = (fold4 >= {1'b0, MLDSA_Q_COEFF}) ? (fold4 - {1'b0, MLDSA_Q_COEFF}) : fold4;
        r     = corr0[MLDSA_COEFF_W-1:0];
    end
endmodule

module mldsa_modq_mul (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         start,
    input  wire [MLDSA_COEFF_W-1:0]     a,
    input  wire [MLDSA_COEFF_W-1:0]     b,
    output logic [MLDSA_COEFF_W-1:0]    result,
    output logic                        busy,
    output logic                        done
);

    typedef enum logic [1:0] {
        S_IDLE,
        S_MUL,
        S_REDUCE,
        S_DONE
    } state_e;

    state_e st;

    logic [MLDSA_COEFF_W-1:0] a_q;
    logic [MLDSA_COEFF_W-1:0] b_q;
    logic [47:0]             product_comb;
    logic [47:0]             product_q;
    logic [MLDSA_COEFF_W-1:0] reduced_comb;

    mldsa_mul24x24 u_mul24x24 (
        .a(a_q),
        .b(b_q),
        .p(product_comb)
    );

    mldsa_reduce_q_solinas u_reduce (
        .x(product_q),
        .r(reduced_comb)
    );

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            st        <= S_IDLE;
            a_q       <= '0;
            b_q       <= '0;
            product_q <= '0;
            result    <= '0;
            busy      <= 1'b0;
            done      <= 1'b0;
        end else begin
            done <= 1'b0;
            unique case (st)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        a_q  <= a;
                        b_q  <= b;
                        busy <= 1'b1;
                        st   <= S_MUL;
                    end
                end
                S_MUL: begin
                    product_q <= product_comb;
                    st        <= S_REDUCE;
                end
                S_REDUCE: begin
                    result <= reduced_comb;
                    st     <= S_DONE;
                end
                S_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    st   <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule

module mldsa_modq_mul_pipe (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         in_valid,
    input  wire [MLDSA_COEFF_W-1:0]     a,
    input  wire [MLDSA_COEFF_W-1:0]     b,
    output wire                         in_ready,
    output logic                        out_valid,
    output logic [MLDSA_COEFF_W-1:0]    result
);

    logic [MLDSA_COEFF_W-1:0] a_q;
    logic [MLDSA_COEFF_W-1:0] b_q;
    logic [47:0]              product_comb;
    logic [47:0]              product_q;
`ifdef MLDSA_FPGA_REDUCE_PIPE2
    logic [37:0]              reduce_fold1_comb;
    logic [28:0]              reduce_fold2_comb;
    logic [28:0]              reduce_fold2_q;
    logic [24:0]              reduce_fold3_comb;
    logic [24:0]              reduce_fold4_comb;
    logic [24:0]              reduce_corr_comb;
    logic                     valid_fold_q;
`else
    logic [MLDSA_COEFF_W-1:0] reduced_comb;
`endif
    (* shreg_extract = "no", keep = "true" *) logic valid_mul_q;
    (* shreg_extract = "no", keep = "true" *) logic valid_red_q;
`ifdef MLDSA_TARGET_FPGA
    (* shreg_extract = "no", keep = "true" *) logic [MLDSA_COEFF_W-1:0] reduced_q;
    (* shreg_extract = "no", keep = "true" *) logic valid_out_q;
`endif
`ifdef MLDSA_FPGA_REDUCE_PIPE2

    function automatic logic [37:0] mul_8191_25(input logic [24:0] h);
        logic [37:0] hh;
        begin
            hh = {13'd0, h};
            mul_8191_25 = (hh << 13) - hh;
        end
    endfunction

    function automatic logic [28:0] mul_8191_15(input logic [14:0] h);
        logic [28:0] hh;
        begin
            hh = {14'd0, h};
            mul_8191_15 = (hh << 13) - hh;
        end
    endfunction

    function automatic logic [24:0] mul_8191_6(input logic [5:0] h);
        logic [24:0] hh;
        begin
            hh = {19'd0, h};
            mul_8191_6 = (hh << 13) - hh;
        end
    endfunction

    function automatic logic [24:0] mul_8191_2(input logic [1:0] h);
        logic [24:0] hh;
        begin
            hh = {23'd0, h};
            mul_8191_2 = (hh << 13) - hh;
        end
    endfunction
`endif

    assign in_ready = 1'b1;

    mldsa_mul24x24 u_mul24x24 (
        .a(a_q),
        .b(b_q),
        .p(product_comb)
    );

`ifdef MLDSA_FPGA_REDUCE_PIPE2
    always_comb begin
        reduce_fold1_comb = {15'd0, product_q[22:0]} + mul_8191_25(product_q[47:23]);
        reduce_fold2_comb = {6'd0, reduce_fold1_comb[22:0]} +
                            mul_8191_15(reduce_fold1_comb[37:23]);
        reduce_fold3_comb = {2'd0, reduce_fold2_q[22:0]} +
                            mul_8191_6(reduce_fold2_q[28:23]);
        reduce_fold4_comb = {2'd0, reduce_fold3_comb[22:0]} +
                            mul_8191_2(reduce_fold3_comb[24:23]);
        reduce_corr_comb  = (reduce_fold4_comb >= {1'b0, MLDSA_Q_COEFF}) ?
                            (reduce_fold4_comb - {1'b0, MLDSA_Q_COEFF}) :
                            reduce_fold4_comb;
    end
`else
    mldsa_reduce_q_solinas u_reduce (
        .x(product_q),
        .r(reduced_comb)
    );
`endif

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            a_q         <= '0;
            b_q         <= '0;
            product_q   <= '0;
            result      <= '0;
            valid_mul_q <= 1'b0;
            valid_red_q <= 1'b0;
`ifdef MLDSA_FPGA_REDUCE_PIPE2
            reduce_fold2_q <= '0;
            valid_fold_q   <= 1'b0;
`endif
`ifdef MLDSA_TARGET_FPGA
            reduced_q   <= '0;
            valid_out_q <= 1'b0;
`endif
            out_valid   <= 1'b0;
        end else begin
            if (in_valid) begin
                a_q <= a;
                b_q <= b;
            end

            product_q   <= product_comb;
            valid_mul_q <= in_valid;
`ifdef MLDSA_FPGA_REDUCE_PIPE2
            reduce_fold2_q <= reduce_fold2_comb;
            valid_fold_q   <= valid_mul_q;
            reduced_q      <= reduce_corr_comb[MLDSA_COEFF_W-1:0];
            valid_red_q    <= valid_fold_q;
            result      <= reduced_q;
            valid_out_q <= valid_red_q;
            out_valid   <= valid_out_q;
`elsif MLDSA_TARGET_FPGA
            valid_red_q <= valid_mul_q;
            reduced_q   <= reduced_comb;
            result      <= reduced_q;
            valid_out_q <= valid_red_q;
            out_valid   <= valid_out_q;
`else
            valid_red_q <= valid_mul_q;
            result      <= reduced_comb;
            out_valid   <= valid_red_q;
`endif
        end
    end
endmodule

`default_nettype wire
