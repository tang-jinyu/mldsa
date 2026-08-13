`timescale 1ns/1ps

/*
    --------------------模块功能--------------------
    Keccak模块的核心函数F
    --------------------基本信息--------------------
    日期: 2025-07-31
    作者: ZJX
    --------------------端口描述--------------------
    i_state : 轮状态
    i_rc    : 轮常数
    o_state : 轮状态
*/

module KeccakF(
    input  wire[1599:0] i_state ,
    input  wire[6:0]    i_rc    ,
    output wire[1599:0] o_state  
);
    wire[63:0] A_00, A_10, A_20, A_30, A_40 ;
    wire[63:0] A_01, A_11, A_21, A_31, A_41 ;
    wire[63:0] A_02, A_12, A_22, A_32, A_42 ;
    wire[63:0] A_03, A_13, A_23, A_33, A_43 ;
    wire[63:0] A_04, A_14, A_24, A_34, A_44 ;

    wire[63:0] A_00_wire, A_10_wire, A_20_wire, A_30_wire, A_40_wire ;
    wire[63:0] A_01_wire, A_11_wire, A_21_wire, A_31_wire, A_41_wire ;
    wire[63:0] A_02_wire, A_12_wire, A_22_wire, A_32_wire, A_42_wire ;
    wire[63:0] A_03_wire, A_13_wire, A_23_wire, A_33_wire, A_43_wire ;
    wire[63:0] A_04_wire, A_14_wire, A_24_wire, A_34_wire, A_44_wire ;

    wire[63:0] B_00, B_10, B_20, B_30, B_40 ;
    wire[63:0] B_01, B_11, B_21, B_31, B_41 ;
    wire[63:0] B_02, B_12, B_22, B_32, B_42 ;
    wire[63:0] B_03, B_13, B_23, B_33, B_43 ;
    wire[63:0] B_04, B_14, B_24, B_34, B_44 ;

    wire[63:0] C_0, C_1, C_2, C_3, C_4 ;
    wire[63:0] D_0, D_1, D_2, D_3, D_4 ;

    wire[63:0] E_00, E_10, E_20, E_30, E_40 ;
    wire[63:0] E_01, E_11, E_21, E_31, E_41 ;
    wire[63:0] E_02, E_12, E_22, E_32, E_42 ;
    wire[63:0] E_03, E_13, E_23, E_33, E_43 ;
    wire[63:0] E_04, E_14, E_24, E_34, E_44 ;
    wire[63:0] E_00_wire                    ;

	// 将数据存储到状态数组A中
    assign A_44	= i_state[1599:1536];
    assign A_34	= i_state[1535:1472];
    assign A_24	= i_state[1471:1408];
    assign A_14	= i_state[1407:1344];
    assign A_04	= i_state[1343:1280];

    assign A_43	= i_state[1279:1216];
    assign A_33	= i_state[1215:1152];
    assign A_23	= i_state[1151:1088];
    assign A_13	= i_state[1087:1024];
    assign A_03	= i_state[1023:960 ];

    assign A_42	= i_state[959 :896 ];
    assign A_32	= i_state[895 :832 ];
    assign A_22	= i_state[831 :768 ];
    assign A_12	= i_state[767 :704 ];
    assign A_02	= i_state[703 :640 ];

    assign A_41	= i_state[639 :576 ];
    assign A_31	= i_state[575 :512 ];
    assign A_21	= i_state[511 :448 ];
    assign A_11	= i_state[447 :384 ];
    assign A_01	= i_state[383 :320 ];

    assign A_40	= i_state[319 :256 ];
    assign A_30	= i_state[255 :192 ];
    assign A_20	= i_state[191 :128 ];
    assign A_10	= i_state[127 :64  ];
    assign A_00	= i_state[63  :0   ];

    /*
        Theta[θ]
    */
    // C[x] = A[x,0]^A[x,1]^A[x,2]^A[x,3]^A[x,4]
    assign C_0 = A_00^A_01^A_02^A_03^A_04;
    assign C_1 = A_10^A_11^A_12^A_13^A_14;
    assign C_2 = A_20^A_21^A_22^A_23^A_24;
    assign C_3 = A_30^A_31^A_32^A_33^A_34;
    assign C_4 = A_40^A_41^A_42^A_43^A_44;

    // D[x] = C[x-1] ^ rot[C[x+1],1]
    assign D_0 = C_4^rol(C_1,1);
    assign D_1 = C_0^rol(C_2,1);
    assign D_2 = C_1^rol(C_3,1);
    assign D_3 = C_2^rol(C_4,1);
    assign D_4 = C_3^rol(C_0,1);

    // A[x,y] = A[x,y] ^ D[x]
    assign A_00_wire = A_00^D_0;
    assign A_10_wire = A_10^D_1;
    assign A_20_wire = A_20^D_2;
    assign A_30_wire = A_30^D_3;
    assign A_40_wire = A_40^D_4;

    assign A_01_wire = A_01^D_0;
    assign A_11_wire = A_11^D_1;
    assign A_21_wire = A_21^D_2;
    assign A_31_wire = A_31^D_3;
    assign A_41_wire = A_41^D_4;

    assign A_02_wire = A_02^D_0;
    assign A_12_wire = A_12^D_1;
    assign A_22_wire = A_22^D_2;
    assign A_32_wire = A_32^D_3;
    assign A_42_wire = A_42^D_4;

    assign A_03_wire = A_03^D_0;
    assign A_13_wire = A_13^D_1;
    assign A_23_wire = A_23^D_2;
    assign A_33_wire = A_33^D_3;
    assign A_43_wire = A_43^D_4;

    assign A_04_wire = A_04^D_0;
    assign A_14_wire = A_14^D_1;
    assign A_24_wire = A_24^D_2;
    assign A_34_wire = A_34^D_3;
    assign A_44_wire = A_44^D_4;

    /*
        Rho[ρ] and Pi[π]
    */
    // B[y,[2x+3y] mod 5] = rot[A[x,y], r[x,y]]
    assign B_00 =     A_00_wire     ;
    assign B_02 = rol(A_10_wire, 1 );
    assign B_04 = rol(A_20_wire, 62);
    assign B_01 = rol(A_30_wire, 28);
    assign B_03 = rol(A_40_wire, 27);

    assign B_13 = rol(A_01_wire, 36);
    assign B_10 = rol(A_11_wire, 44);
    assign B_12 = rol(A_21_wire, 6 );
    assign B_14 = rol(A_31_wire, 55);
    assign B_11 = rol(A_41_wire, 20);

    assign B_21 = rol(A_02_wire, 3 );
    assign B_23 = rol(A_12_wire, 10);
    assign B_20 = rol(A_22_wire, 43);
    assign B_22 = rol(A_32_wire, 25);
    assign B_24 = rol(A_42_wire, 39);

    assign B_34 = rol(A_03_wire, 41);
    assign B_31 = rol(A_13_wire, 45);
    assign B_33 = rol(A_23_wire, 15);
    assign B_30 = rol(A_33_wire, 21);
    assign B_32 = rol(A_43_wire, 8 );

    assign B_42 = rol(A_04_wire, 18);
    assign B_44 = rol(A_14_wire, 2 );
    assign B_41 = rol(A_24_wire, 61);
    assign B_43 = rol(A_34_wire, 56);
    assign B_40 = rol(A_44_wire, 14);

    /*
        Chi[χ]
    */
    // assign E_00      = {i_rc[6], 31'b0, i_rc[5], 15'b0, i_rc[4], 7'b0, i_rc[3], 3'b0, i_rc[2], 1'b0, i_rc[1], i_rc[0]} ^ B_00 ^ (~ B_10 & B_20);
    assign E_00_wire = B_00 ^ (~ B_10 & B_20);
    assign E_10      = B_10 ^ (~ B_20 & B_30);
    assign E_20      = B_20 ^ (~ B_30 & B_40);
    assign E_30      = B_30 ^ (~ B_40 & B_00);
    assign E_40      = B_40 ^ (~ B_00 & B_10);

    assign E_01      = B_01 ^ (~ B_11 & B_21);
    assign E_11      = B_11 ^ (~ B_21 & B_31);
    assign E_21      = B_21 ^ (~ B_31 & B_41);
    assign E_31      = B_31 ^ (~ B_41 & B_01);
    assign E_41      = B_41 ^ (~ B_01 & B_11);

    assign E_02      = B_02 ^ (~ B_12 & B_22);
    assign E_12      = B_12 ^ (~ B_22 & B_32);
    assign E_22      = B_22 ^ (~ B_32 & B_42);
    assign E_32      = B_32 ^ (~ B_42 & B_02);
    assign E_42      = B_42 ^ (~ B_02 & B_12);

    assign E_03      = B_03 ^ (~ B_13 & B_23);
    assign E_13      = B_13 ^ (~ B_23 & B_33);
    assign E_23      = B_23 ^ (~ B_33 & B_43);
    assign E_33      = B_33 ^ (~ B_43 & B_03);
    assign E_43      = B_43 ^ (~ B_03 & B_13);

    assign E_04      = B_04 ^ (~ B_14 & B_24);
    assign E_14      = B_14 ^ (~ B_24 & B_34);
    assign E_24      = B_24 ^ (~ B_34 & B_44);
    assign E_34      = B_34 ^ (~ B_44 & B_04);
    assign E_44      = B_44 ^ (~ B_04 & B_14);

    /*
        Iota[ι]
    */
    assign E_00 = {E_00_wire[63]^i_rc[6] , E_00_wire[62:32]      , E_00_wire[31]^i_rc[5] ,
                   E_00_wire[30:16]      , E_00_wire[15]^i_rc[4] , E_00_wire[14:8]       ,
                   E_00_wire[7]^i_rc[3]  , E_00_wire[6:4]        , E_00_wire[3]^i_rc[2]  ,
                   E_00_wire[2]          , E_00_wire[1]^i_rc[1]  , E_00_wire[0]^i_rc[0]} ;

    assign o_state[1599:1536] = E_44;
    assign o_state[1535:1472] = E_34;
    assign o_state[1471:1408] = E_24;
    assign o_state[1407:1344] = E_14;
    assign o_state[1343:1280] = E_04;

    assign o_state[1279:1216] = E_43;
    assign o_state[1215:1152] = E_33;
    assign o_state[1151:1088] = E_23;
    assign o_state[1087:1024] = E_13;
    assign o_state[1023:960 ] = E_03;

    assign o_state[959 :896 ] = E_42;
    assign o_state[895 :832 ] = E_32;
    assign o_state[831 :768 ] = E_22;
    assign o_state[767 :704 ] = E_12;
    assign o_state[703 :640 ] = E_02;

    assign o_state[639 :576 ] = E_41;
    assign o_state[575 :512 ] = E_31;
    assign o_state[511 :448 ] = E_21;
    assign o_state[447 :384 ] = E_11;
    assign o_state[383 :320 ] = E_01;

    assign o_state[319 :256 ] = E_40;
    assign o_state[255 :192 ] = E_30;
    assign o_state[191 :128 ] = E_20;
    assign o_state[127 :64  ] = E_10;
    assign o_state[63  :0   ] = E_00;
    /*-------------------- 循环左移 --------------------*/
    function[63:0] rol;
        input [63:0] x;
        input integer y;
        integer i;
        begin
            rol = x;
            for(i=0; i<y; i=i+1) rol = {rol[62:0], rol[63]};
        end
    endfunction
endmodule