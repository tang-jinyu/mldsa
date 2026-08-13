`timescale 1ns/1ps

/*
    --------------------模块功能--------------------
    SHA3运算模块
    <Pseudorandom number generator/PRNG>
    流程 : INIT初始化 -> ABSORB吸收阶段 -> SQUEEZ挤压阶段[可扩展输出]
    --------------------基本信息--------------------
    日期: 2025-07-31
    作者: ZJX
    --------------------端口描述--------------------
    i_clk        : 时钟
    i_rst_n      : 低电平复位
    
    i_ready      : 外部输入, 表示可以输入数据
    o_need       : 表示内部需要输入数据
    i_data       : 输入数据
    i_valid      : 输入有效
    
    o_ready      : 内部输出, 表示可以输出数据
    i_need       : 表示外部需要输出数据
    o_data       : 输出数据
    o_valid      : 输出有效
    
    i_last_pos   : 最后数据输入有效表示{00_0001, 00_0011, 00_0111, 00_1111, ...}
    i_last       : 表示最后消息块输入

    i_mode       : 模式 {0: SHA3-224, 1: SHA3-256, 2: SHA3-384, 3: SHA3-512, 4: SHAKE128, 5: SHAKE256}
    i_start      : 开始
    
    o_done       : 结束
    o_idle       : 空闲
*/

module PRNG #( // 5462, 3067
    parameter DATA_WIDTH = 64 // {64, 32, 16, 8}
)(
    // 门控信号
    input  wire                     i_clk      ,
    input  wire                     i_rst_n    ,
    // 数据输入
    input  wire                     i_ready    ,
    output wire                     o_need     ,
    input  wire[DATA_WIDTH-1:0]     i_data     ,
    input  wire                     i_valid    ,
    // 数据输出
    output wire                     o_ready    ,
    input  wire                     i_need     ,
    output reg[DATA_WIDTH-1:0]      o_data     ,
    output reg                      o_valid    ,
    // 数据输入结束控制
    input  wire[(DATA_WIDTH/8)-1:0] i_last_pos ,
    input  wire                     i_last     ,
    // 控制信号
    input  wire[2:0]                i_mode     ,
    input  wire                     i_start    ,
    // 状态信号
    output wire                     o_done     ,
    output wire                     o_idle      
);
    /*--------------- 参数定义 ---------------*/
    genvar i;
    integer j;
    parameter DEPTH = 1344/DATA_WIDTH , // 存储器深度
              ADDR  = $clog2(DEPTH)   ; // 存储器深度所用位数
    parameter SHA3_224 = 'd0 ,
              SHA3_256 = 'd1 ,
              SHA3_384 = 'd2 ,
              SHA3_512 = 'd3 ,
              SHAKE128 = 'd4 ,
              SHAKE256 = 'd5 ;
    reg[$clog2(6)-1:0] mode;
    reg[ADDR-1:0] absorb_times;
    always @(*) begin
        case (mode)
            SHA3_224 : absorb_times = 1152/DATA_WIDTH;
            SHA3_256 : absorb_times = 1088/DATA_WIDTH;
            SHA3_384 : absorb_times = 832 /DATA_WIDTH;
            SHA3_512 : absorb_times = 576 /DATA_WIDTH;
            SHAKE128 : absorb_times = 1344/DATA_WIDTH;
            SHAKE256 : absorb_times = 1088/DATA_WIDTH;
            default  : absorb_times = 1088/DATA_WIDTH;
        endcase
    end
    reg[ADDR-1:0] squeez_times;
    always @(*) begin
        case (mode)
            SHA3_224 : squeez_times = ceil_div(224 ,DATA_WIDTH); // 没法被64整除, 即以64吞吐的话, 最后一次输出只能取低32位
            SHA3_256 : squeez_times = ceil_div(256 ,DATA_WIDTH);
            SHA3_384 : squeez_times = ceil_div(384 ,DATA_WIDTH);
            SHA3_512 : squeez_times = ceil_div(512 ,DATA_WIDTH);
            SHAKE128 : squeez_times = ceil_div(1344,DATA_WIDTH);
            SHAKE256 : squeez_times = ceil_div(1088,DATA_WIDTH);
            default  : squeez_times = ceil_div(1088,DATA_WIDTH);
        endcase
    end
    wire[ADDR-1:0] r_addr_final;
    assign r_addr_final = absorb_times-1;
    reg[DATA_WIDTH-1:0] first_data;
    always @(*) begin
        case (mode)
            SHA3_224 : first_data = 'b110;
            SHA3_256 : first_data = 'b110;
            SHA3_384 : first_data = 'b110;
            SHA3_512 : first_data = 'b110;
            SHAKE128 : first_data = 'b1_1111;
            SHAKE256 : first_data = 'b1_1111;
            default  : first_data = 'b110;
        endcase
    end
    reg[1:0] cur_state, nxt_state;
    parameter IDLE   = 2'b00 ,
              ABSORB = 2'b01 ,
              SQUEEZ = 2'b11 ;
    reg[1:0] cur_state_child, nxt_state_child;
    parameter ABSORB_LOAD  = 2'b00 ,
              ABSORB_WAIT  = 2'b01 ,
              ABSORB_FINAL = 2'b11 ;
    /*--------------- 变量定义 ---------------*/
    reg[DATA_WIDTH-1:0] r [0:DEPTH-1];
    wire[1343:0] r_wire;
    reg[ADDR-1:0] r_addr, cnt;
    wire[ADDR-1:0] r_addr_wire, cnt_wire;
    reg absorb_end, more_1round;
    wire signal_0, signal_1, signal_2, absorb_need_cal, absorb_term_end, absorb_full_end, signal_wait;
    reg output_valid, core_valid;
    // DataPad
    wire[DATA_WIDTH-1:0]     i_data_DataPad     ;
    wire[DATA_WIDTH-1:0]     o_data_DataPad     ;
    wire[$clog2(6)-1:0]      i_mode_DataPad     ;
    wire[(DATA_WIDTH/8)-1:0] i_last_pos_DataPad ;
    wire                     i_term_end_DataPad ;
    wire                     i_full_end_DataPad ;
    // KeccakCore
    wire[1599:0] i_data_KeccakCore  ;
    reg          i_valid_KeccakCore ;
    wire[1599:0] o_data_KeccakCore  ;
    reg [1599:0] keccak_chain_state ;
    wire         o_valid_KeccakCore ;
    reg          i_start_KeccakCore ;
    wire         o_done_KeccakCore  ;
    wire         o_idle_KeccakCore  ;
    reg          absorb_perm_started;
    /*--------------- 打拍 ---------------*/
    reg idle_Q;
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) idle_Q <= 1'b1;
        else idle_Q <= o_idle;
    end
    /*--------------- 逻辑控制 ---------------*/
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) cur_state <= IDLE;
        else cur_state <= nxt_state;
    end
    always @(*)begin
        case(cur_state)
            IDLE    : nxt_state = i_start                                          ? ABSORB : IDLE   ;
            ABSORB  : nxt_state = ((!more_1round)&&absorb_end&&i_start_KeccakCore) ? SQUEEZ : ABSORB ;
            SQUEEZ  : nxt_state =                                                    SQUEEZ          ;
            default : nxt_state =                                                    IDLE            ;
        endcase
    end
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) cur_state_child <= ABSORB_LOAD;
        else cur_state_child <= nxt_state_child;
    end
    always @(*)begin
        case(cur_state_child)
            ABSORB_LOAD  : nxt_state_child = signal_wait       ? ABSORB_WAIT                            : ABSORB_LOAD ;
            ABSORB_WAIT  : nxt_state_child = o_done_KeccakCore ? (more_1round?ABSORB_FINAL:ABSORB_LOAD) : ABSORB_WAIT ;
            ABSORB_FINAL : nxt_state_child =                     ABSORB_WAIT                                          ;
            default      : nxt_state_child =                     ABSORB_LOAD                                          ;
        endcase
    end
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            i_valid_KeccakCore <= 'd0;
            i_start_KeccakCore <= 'd0;
            o_data <= 'd0;
            o_valid <= 'd0;
            for (j=0; j<DEPTH; j=j+1) r[j] <= 'd0;
            keccak_chain_state <= 'd0;
            r_addr <= 'd0;
            mode <= SHA3_512;
            cnt <= 576/DATA_WIDTH;
            absorb_end <= 'd0;
            more_1round <= 'd0;
            output_valid <= 'd0;
            core_valid <= 'd0;
            absorb_perm_started <= 'd0;
        end else begin
            i_valid_KeccakCore <= 'd0;
            i_start_KeccakCore <= 'd0;
            o_valid <= 'd0;
            case (cur_state)
                IDLE : begin
                    for (j=0; j<DEPTH; j=j+1) r[j] <= 'd0;
                    keccak_chain_state <= 'd0;
                    r_addr <= 'd0;
                    mode <= i_mode;
                    case (i_mode)
                        SHA3_224 : cnt <= 1152/DATA_WIDTH;
                        SHA3_256 : cnt <= 1088/DATA_WIDTH;
                        SHA3_384 : cnt <= 832 /DATA_WIDTH;
                        SHA3_512 : cnt <= 576 /DATA_WIDTH;
                        SHAKE128 : cnt <= 1344/DATA_WIDTH;
                        SHAKE256 : cnt <= 1088/DATA_WIDTH;
                        default  : cnt <= 1088/DATA_WIDTH;
                    endcase
                    absorb_end <= 'd0;
                    more_1round <= 'd0;
                    output_valid <= 'd0;
                    core_valid <= 'd0;
                    absorb_perm_started <= 'd0;
                end
                ABSORB : begin
                    case (cur_state_child)
                        ABSORB_LOAD : begin
                            if (i_valid) begin
                                r[r_addr] <= o_data_DataPad;
                                if (absorb_need_cal) begin
                                    cnt <= absorb_times;
                                    r_addr <= 'd0;
                                    absorb_perm_started <= 'd0;
                                end else if (absorb_term_end) begin
                                    cnt <= squeez_times;
                                    r_addr <= 'd0;
                                    r[r_addr_final] <= 1'b1<<(DATA_WIDTH-1);
                                    absorb_end <= 'd1;
                                    absorb_perm_started <= 'd0;
                                end else if (absorb_full_end) begin
                                    cnt <= squeez_times;
                                    r_addr <= 'd0;
                                    more_1round <= (i_last_pos == {DATA_WIDTH/8{1'b1}});
                                    absorb_end <= 'd1;
                                    absorb_perm_started <= 'd0;
                                end else begin
                                    cnt <= cnt_wire;
                                    r_addr <= r_addr_wire;
                                end
                            end
                        end
                        ABSORB_WAIT : begin
                            if ((!absorb_perm_started) && o_idle_KeccakCore) begin
                                i_valid_KeccakCore <= 'd1;
                                i_start_KeccakCore <= 'd1;
                                absorb_perm_started <= 'd1;
                            end
                        end
                        ABSORB_FINAL : begin
                            r[0] <= first_data;
                            r[r_addr_final] <= 1'b1<<(DATA_WIDTH-1);
                            more_1round <= 'd0;
                            absorb_perm_started <= 'd0;
                        end
                    endcase
                    if (cur_state_child == ABSORB_WAIT && o_done_KeccakCore) begin
                        keccak_chain_state <= o_data_KeccakCore;
                        absorb_perm_started <= 'd0;
                    end
                    if (i_start_KeccakCore) for (j=0; j<DEPTH; j=j+1) r[j] <= 'd0;
                end
                SQUEEZ : begin
                    case ({output_valid, core_valid})
                        2'b00 : begin
                            if (o_done_KeccakCore) begin
                                output_valid <= 'd1;
                                keccak_chain_state <= o_data_KeccakCore;
                                for (j=0; j<DEPTH; j=j+1) r[j] <= o_data_KeccakCore[DATA_WIDTH*j+:DATA_WIDTH];
                                r_addr <= 'd0;
                                cnt <= squeez_times;
                                i_start_KeccakCore <= 'd1;
                            end
                        end
                        2'b01 : begin
                            output_valid <= 'd1;
                            core_valid <= 'd1;
                            keccak_chain_state <= o_data_KeccakCore;
                            for (j=0; j<DEPTH; j=j+1) r[j] <= o_data_KeccakCore[DATA_WIDTH*j+:DATA_WIDTH];
                            r_addr <= 'd0;
                            cnt <= squeez_times;
                            i_start_KeccakCore <= 'd1;
                        end
                        2'b10 : begin
                            if (i_need) begin
                                o_data <= r[r_addr];
                                o_valid <= 'd1;
                                r_addr <= r_addr_wire;
                                cnt <= cnt_wire;
                                if (signal_1) begin
                                    output_valid <= 'd0;
                                    r_addr <= 'd0;
                                    cnt <= squeez_times;
                                end
                            end
                            if (o_done_KeccakCore) begin
                                core_valid <= 'd1;
                                keccak_chain_state <= o_data_KeccakCore;
                            end
                        end
                        2'b11 : begin
                            if (i_need) begin
                                o_data <= r[r_addr];
                                o_valid <= 'd1;
                                r_addr <= r_addr_wire;
                                cnt <= cnt_wire;
                                if (signal_1) begin
                                    output_valid <= 'd0;
                                    r_addr <= 'd0;
                                    cnt <= squeez_times;
                                end
                            end
                        end
                    endcase 
                end
            endcase
        end
    end
    generate
        for (i=0; i<DEPTH; i=i+1) assign r_wire[DATA_WIDTH*i+:DATA_WIDTH] = r[i];
    endgenerate
    assign r_addr_wire = r_addr+1;
    assign cnt_wire = cnt-1;
    assign signal_0 = cnt=='d0;
    assign signal_1 = cnt=='d1;
    assign signal_2 = cnt=='d2;
    assign absorb_need_cal =   signal_1 &&i_valid&&(!i_last);
    assign absorb_term_end = (!signal_1)&&i_valid&&  i_last ;
    assign absorb_full_end =   signal_1 &&i_valid&&  i_last ;
    assign signal_wait = absorb_need_cal||absorb_term_end||absorb_full_end;
    // DataPad
    assign i_data_DataPad = i_data;
    assign i_mode_DataPad = mode;
    assign i_last_pos_DataPad = i_last_pos;
    assign i_term_end_DataPad = absorb_term_end;
    assign i_full_end_DataPad = absorb_full_end;
    // KeccakCore
    assign i_data_KeccakCore = {256'd0, r_wire} ^ keccak_chain_state;
    // Output
    assign o_need = (cur_state==ABSORB)&&(cur_state_child==ABSORB_LOAD)&&(!absorb_end);
    assign o_ready = (cur_state==SQUEEZ)&&output_valid;
    assign o_done = o_idle&&(!idle_Q);
    assign o_idle = !(|cur_state);
    /*--------------- 模块例化 ---------------*/
    // DataPad
    DataPad #(
        .DATA_WIDTH (DATA_WIDTH )
    )DataPad(
        .i_data     (i_data_DataPad     ),
        
        .o_data     (o_data_DataPad     ),
        
        .i_mode     (i_mode_DataPad     ),
        .i_last_pos (i_last_pos_DataPad ),
        .i_term_end (i_term_end_DataPad ),
        .i_full_end (i_full_end_DataPad ) 
    );
    // KeccakCore
    KeccakCore KeccakCore(
        .i_clk   (i_clk              ),
        .i_rst_n (i_rst_n            ),
        
        .i_data  (i_data_KeccakCore  ),
        .i_valid (i_valid_KeccakCore ),
        
        .o_data  (o_data_KeccakCore  ),
        .o_valid (o_valid_KeccakCore ),
        
        .i_start (i_start_KeccakCore ),
        
        .o_done  (o_done_KeccakCore  ),
        .o_idle  (o_idle_KeccakCore  ) 
    );
    /*--------------- [function] ---------------*/
    function [DATA_WIDTH-1:0] ceil_div; // 计算 : dividend / divisor, 结果向上取整
        input [DATA_WIDTH-1:0] dividend;
        input [DATA_WIDTH-1:0] divisor;
        ceil_div = (dividend+divisor-1)/divisor;
    endfunction
endmodule
