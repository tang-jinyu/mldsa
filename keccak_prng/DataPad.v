`timescale 1ns/1ps

/*
    --------------------模块功能--------------------
    数据填充
    组合逻辑, 外部需要打拍
    --------------------基本信息--------------------
    日期: 2021-01-13
    作者: ZJX
    --------------------端口描述--------------------
    i_data     : 数据输入
    
    o_data     : 数据输出
    
    i_mode     : 模式 {0: SHA3-224, 1: SHA3-256, 2: SHA3-384, 3: SHA3-512, 4: SHAKE128, 5: SHAKE256}
    i_last_pos : 最后数据输入有效位置表示{00_0001, 00_0011, 00_0111, 00_1111, ...}
    i_term_end : 吸收阶段, 未完整吸收完一轮就结束标志
    i_full_end : 吸收阶段, 已完整吸收完一轮就结束标志
*/

module DataPad #(
    parameter DATA_WIDTH = 64
)(
    // 数据输入
    input  wire[DATA_WIDTH-1:0]     i_data     ,
    // 数据输出
    output wire[DATA_WIDTH-1:0]     o_data     ,
    // 控制信号
    input  wire[$clog2(6)-1:0]      i_mode     ,
    input  wire[(DATA_WIDTH/8)-1:0] i_last_pos ,
    input  wire                     i_term_end ,
    input  wire                     i_full_end  
);
    /*--------------- 参数定义 ---------------*/
    genvar i;
    integer j;
    parameter SHA3_224 = 'd0 ,
              SHA3_256 = 'd1 ,
              SHA3_384 = 'd2 ,
              SHA3_512 = 'd3 ,
              SHAKE128 = 'd4 ,
              SHAKE256 = 'd5 ;
    parameter LAST = 8'b1000_0000;
    reg[7:0] first, first_and_last;
    always @(*) begin
        case (i_mode)
            SHA3_224, SHA3_256, SHA3_384, SHA3_512 : begin
                first = 8'b0000_0110;
                first_and_last = 8'b1000_0110;
            end
            default: begin
                first = 8'b0001_1111;
                first_and_last = 8'b1001_1111;
            end
        endcase
    end
    /*--------------- 变量定义 ---------------*/
    reg[DATA_WIDTH-1:0] data;
    /*--------------- 打拍 ---------------*/
    /*--------------- 逻辑控制 ---------------*/
    generate
        if (DATA_WIDTH == 64) begin : gen_64bits_pad
            always @(*) begin
                case ({i_term_end, i_full_end})
                    2'b01   : begin
                        case (i_last_pos)
                            'd0     : data = {LAST, 48'b0, first};
                            'd1     : data = {LAST, 40'b0, first, i_data[0+:1*8]};
                            'd3     : data = {LAST, 32'b0, first, i_data[0+:2*8]};
                            'd7     : data = {LAST, 24'b0, first, i_data[0+:3*8]};
                            'd15    : data = {LAST, 16'b0, first, i_data[0+:4*8]};
                            'd31    : data = {LAST,  8'b0, first, i_data[0+:5*8]};
                            'd63    : data = {LAST,        first, i_data[0+:6*8]};
                            'd127   : data = {first_and_last    , i_data[0+:7*8]};
                            default : data = i_data;
                        endcase
                    end
                    2'b10   : begin
                        case (i_last_pos)
                            'd0     : data = {56'b0, first};
                            'd1     : data = {48'b0, first, i_data[0+:1*8]};
                            'd3     : data = {40'b0, first, i_data[0+:2*8]};
                            'd7     : data = {32'b0, first, i_data[0+:3*8]};
                            'd15    : data = {24'b0, first, i_data[0+:4*8]};
                            'd31    : data = {16'b0, first, i_data[0+:5*8]};
                            'd63    : data = { 8'b0, first, i_data[0+:6*8]};
                            'd127   : data = {       first, i_data[0+:7*8]};
                            default : data = i_data;
                        endcase
                    end
                    default : data = i_data; 
                endcase
            end
        end else if (DATA_WIDTH == 32) begin : gen_32bits_pad
            always @(*) begin
                case ({i_term_end, i_full_end})
                    2'b01   : begin
                        case (i_last_pos)
                            'd0     : data = {LAST, 16'b0, first};
                            'd1     : data = {LAST, 8'b0, first, i_data[0+:1*8]};
                            'd3     : data = {LAST,       first, i_data[0+:2*8]};
                            'd7     : data = {first_and_last   , i_data[0+:3*8]};
                            default : data = i_data;
                        endcase
                    end
                    2'b10   : begin
                        case (i_last_pos)
                            'd0     : data = {24'b0, first};
                            'd1     : data = {16'b0, first, i_data[0+:1*8]};
                            'd3     : data = {8'b0 , first, i_data[0+:2*8]};
                            'd7     : data = {       first, i_data[0+:3*8]};
                            default : data = i_data;
                        endcase
                    end
                    default : data = i_data; 
                endcase
            end
        end else if (DATA_WIDTH == 16) begin : gen_16bits_pad
            always @(*) begin
                case ({i_term_end, i_full_end})
                    2'b01   : begin
                        case (i_last_pos)
                            'd0     : data = {LAST, 8'b0, first};
                            'd1     : data = {first_and_last, i_data[0+:1*8]};
                            default : data = i_data;
                        endcase
                    end
                    2'b10   : begin
                        case (i_last_pos)
                            'd0     : data = {8'b0, first};
                            'd1     : data = {first, i_data[0+:1*8]};
                            default : data = i_data;
                        endcase
                    end
                    default : data = i_data; 
                endcase
            end
        end else if (DATA_WIDTH == 8) begin : gen_8bits_pad
            always @(*) begin
                if (i_term_end||i_full_end) data = first_and_last;
                else data = i_data;
            end
        end
    endgenerate
    // Output
    assign o_data = data;
    /*--------------- 模块例化 ---------------*/
endmodule
