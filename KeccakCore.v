`timescale 1ns/1ps

/*
    --------------------模块功能--------------------
    Keccak核心运算
    --------------------基本信息--------------------
    日期: 2026-01-12
    作者: ZJX
    --------------------端口描述--------------------
    i_clk   : 时钟
    i_rst_n : 低电平复位
    
    i_data  : 数据输入
    i_valid : 输入有效 {0: r不变, 1: r被赋值为i_data}
    
    o_data  : 数据输出
    o_valid : 输出有效
    
    i_start : 开始
    
    o_done  : 结束
    o_idle  : 空闲
*/

module KeccakCore(
    // 门控信号
    input  wire         i_clk   ,
    input  wire         i_rst_n ,
    // 数据输入
    input  wire[1343:0] i_data  ,
    input  wire         i_valid ,
    // 数据输出
    output wire[1343:0] o_data  ,
    output wire         o_valid ,
    // 控制信号
    input  wire         i_start ,
    // 状态信号
    output wire         o_done  ,
    output wire         o_idle   
);
    /*--------------- 参数定义 ---------------*/
    parameter COMPUTE_TIMES = 24;
    reg cur_state, nxt_state;
    parameter IDLE = 1'b1,
              WORK = 1'b0;
    /*--------------- 变量定义 ---------------*/
    reg[1343:0] r;
    reg[255:0] c;
    reg[4:0] cnt;
    wire[4:0] cnt_wire;
    wire signal_1;
    // ReadRC
    wire[4:0] i_r_ReadRC  ;
    wire[6:0] o_rc_ReadRC ;
    // KeccakF
    wire[1599:0] i_state_KeccakF ;
    wire[6:0]    i_rc_KeccakF    ;
    wire[1599:0] o_state_KeccakF ;
    /*--------------- 打拍 ---------------*/
    reg idle_Q;
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) idle_Q <= IDLE;
        else idle_Q <= o_idle;
    end
    /*--------------- 逻辑控制 ---------------*/
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) cur_state <= IDLE;
        else cur_state <= nxt_state;
    end
    always @(*)begin
        case(cur_state)
            IDLE    : nxt_state = i_start  ? WORK : IDLE ;
            WORK    : nxt_state = signal_1 ? IDLE : WORK ;
            default : nxt_state =            IDLE        ;
        endcase
    end
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            r <= 'd0;
            c <= 'd0;
            cnt <= COMPUTE_TIMES;
        end else begin
            case (cur_state)
                IDLE : begin
                    if (i_valid) r <= i_data;
                    cnt <= COMPUTE_TIMES;
                end
                WORK : begin
                    cnt <= cnt_wire;
                    {c, r} <= o_state_KeccakF;
                end
            endcase
        end
    end
    assign cnt_wire = cnt-1;
    assign signal_1 = cnt=='d1;
    // ReadRC
    assign i_r_ReadRC = cnt;
    // KeccakF
    assign i_state_KeccakF = {c, r};
    assign i_rc_KeccakF = o_rc_ReadRC;
    // Output
    assign o_data = r;
    assign o_valid = o_done;
    assign o_done = o_idle&&(!idle_Q);
    assign o_idle = cur_state;
    /*--------------- 模块例化 ---------------*/
    // ReadRC
    ReadRC ReadRC(
        .i_r  (i_r_ReadRC  ),
        .o_rc (o_rc_ReadRC ) 
    );
    // KeccakF
    KeccakF KeccakF(
        .i_state (i_state_KeccakF ),
        .i_rc    (i_rc_KeccakF    ),
        .o_state (o_state_KeccakF ) 
    );
endmodule
