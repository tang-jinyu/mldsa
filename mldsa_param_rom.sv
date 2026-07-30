/*
作者：唐金钰
时间：2026/7/28
概述：安全等级参数译码器
*/


`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

module mldsa_param_rom (
    input  wire [1:0]  level_sel,
    output logic [3:0] k,
    output logic [3:0] l,
    output logic [2:0] eta,// 私钥小系数范围 η
    output logic [5:0] tau,// 挑战多项式c的非零系数个数 τ
    output logic [19:0] gamma1,// 掩码向量y的采样范围 γ1
    output logic [19:0] gamma2, // 高低位分解粒度 γ2
    output logic [8:0] beta,// 范数安全边界 β = τ·η
    output logic [7:0] omega, // hint数量上限 ω
    output logic [15:0] pk_bytes,
    output logic [15:0] sk_bytes,
    output logic [15:0] sig_bytes,
    output logic        level_valid // level_sel合法性
);

    always_comb begin
        level_valid = mldsa_level_valid(level_sel);
        k           = 4'(mldsa_level_k(level_sel));
        l           = 4'(mldsa_level_l(level_sel));
        eta         = 3'(mldsa_level_eta(level_sel));
        tau         = 6'(mldsa_level_tau(level_sel));
        gamma1      = 20'(mldsa_level_gamma1(level_sel));
        gamma2      = 20'(mldsa_level_gamma2(level_sel));
        beta        = 9'(mldsa_level_beta(level_sel));
        omega       = 8'(mldsa_level_omega(level_sel));
        pk_bytes    = 16'(mldsa_level_pk_bytes(level_sel));
        sk_bytes    = 16'(mldsa_level_sk_bytes(level_sel));
        sig_bytes   = 16'(mldsa_level_sig_bytes(level_sel));
    end
endmodule
//4'(...) 是 SystemVerilog 的尺寸cast（size cast），把函数返回值强制截断/扩展到指定位宽。
`default_nettype wire
/*为什么参数不同但硬件可以共核？——这是你这个模块存在的根本理由，也是论文里的关键论证：
注意看上表，三个等级的算法流程完全一样，差异只体现在四类东西上：①循环边界（k,l,τ）；②位宽/打包格式（η,γ1,γ2）；③判定阈值（β,ω）；④缓冲区尺寸（pk/sk/sig_bytes）。所以正确的架构选择不是复制三套控制器，而是一套执行骨架 + 一份参数表——param_rom 就是这份参数表的硬件形态。等级切换时，ucode 微码序列不用变，只是下级模块的计数器边界、打包格式选择、比较阈值从 cfg 寄存器取不同值。"把安全等级差异从控制流差异降级为数据差异"，这句话可以直接写进论文。
再补一个深层问题：为什么 γ2 取 (q-1)/88 和 (q-1)/32 这种奇怪的数？ 因为 w1 要能被均匀分解：w1 的取值个数是 (q-1)/2γ2，44档=44种（每系数约5.46bit→实际按6bit打包），65/87档=16种（正好4bit）。γ2 必须整除 (q-1)/2，且要平衡"签名的z范数约束"和"验证能恢复w1"这两头——这是 Dilithium 设计时反复调过的参数，硬件上你只需要知道：γ2 档位一变，decompose_hint 和 pack_unpack 的分支就要跟着变。
*/