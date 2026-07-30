`timescale 1ns/1ps
`default_nettype none

// Shared Keccak/SHA3/SHAKE service for the ASAP7 top-level integration.
//
// IMPORTANT ASSUMPTION:
// This design assumes ECDSA and ML-DSA never issue Keccak requests at the same
// time. It only implements simple mutual exclusion. If a future system allows
// concurrent ECDSA and ML-DSA requests, this block must be replaced by a real
// arbiter with request buffering/ordering; concurrent correctness is not
// guaranteed by this implementation.
//
// Mode encoding follows the existing PRNG core:
//   3'd3: SHA3-512
//   3'd4: SHAKE128
//   3'd5: SHAKE256
module shared_keccak_service #(
    parameter int DATA_WIDTH = 64
) (
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire                     ecdsa_req,
    input  wire                     ecdsa_release,
    output wire                     ecdsa_grant,
    input  wire                     ecdsa_i_ready,
    output wire                     ecdsa_o_need,
    input  wire [DATA_WIDTH-1:0]    ecdsa_i_data,
    input  wire                     ecdsa_i_valid,
    output wire                     ecdsa_o_ready,
    input  wire                     ecdsa_i_need,
    output wire [DATA_WIDTH-1:0]    ecdsa_o_data,
    output wire                     ecdsa_o_valid,
    input  wire [(DATA_WIDTH/8)-1:0] ecdsa_i_last_pos,
    input  wire                     ecdsa_i_last,
    input  wire [2:0]               ecdsa_i_mode,
    input  wire                     ecdsa_i_start,
    output wire                     ecdsa_o_done,
    output wire                     ecdsa_o_idle,

    input  wire                     mldsa_req,
    input  wire                     mldsa_release,
    output wire                     mldsa_grant,
    input  wire                     mldsa_i_ready,
    output wire                     mldsa_o_need,
    input  wire [DATA_WIDTH-1:0]    mldsa_i_data,
    input  wire                     mldsa_i_valid,
    output wire                     mldsa_o_ready,
    input  wire                     mldsa_i_need,
    output wire [DATA_WIDTH-1:0]    mldsa_o_data,
    output wire                     mldsa_o_valid,
    input  wire [(DATA_WIDTH/8)-1:0] mldsa_i_last_pos,
    input  wire                     mldsa_i_last,
    input  wire [2:0]               mldsa_i_mode,
    input  wire                     mldsa_i_start,
    output wire                     mldsa_o_done,
    output wire                     mldsa_o_idle,

    output wire                     busy
);
    localparam logic [1:0] OWNER_NONE  = 2'd0;
    localparam logic [1:0] OWNER_ECDSA = 2'd1;
    localparam logic [1:0] OWNER_MLDSA = 2'd2;

    logic [1:0] owner_q, owner_d;
    logic       prng_rst_n;
    logic       prng_i_ready;
    logic       prng_o_need;
    logic [DATA_WIDTH-1:0] prng_i_data;
    logic       prng_i_valid;
    logic       prng_o_ready;
    logic       prng_i_need;
    logic [DATA_WIDTH-1:0] prng_o_data;
    logic       prng_o_valid;
    logic [(DATA_WIDTH/8)-1:0] prng_i_last_pos;
    logic       prng_i_last;
    logic [2:0] prng_i_mode;
    logic       prng_i_start;
    logic       prng_o_done;
    logic       prng_o_idle;
    logic [1:0] prng_reset_cnt_q;
    logic       prng_live;

    always_comb begin
        owner_d = owner_q;
        unique case (owner_q)
            OWNER_NONE: begin
                if (ecdsa_req) owner_d = OWNER_ECDSA;
                else if (mldsa_req) owner_d = OWNER_MLDSA;
            end
            OWNER_ECDSA: begin
                if (ecdsa_release) owner_d = OWNER_NONE;
            end
            OWNER_MLDSA: begin
                if (mldsa_release) owner_d = OWNER_NONE;
            end
            default: owner_d = OWNER_NONE;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) owner_q <= OWNER_NONE;
        else owner_q <= owner_d;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prng_reset_cnt_q <= 2'd0;
        end else if (owner_q == OWNER_NONE) begin
            prng_reset_cnt_q <= 2'd0;
        end else if (prng_reset_cnt_q != 2'd2) begin
            prng_reset_cnt_q <= prng_reset_cnt_q + 2'd1;
        end
    end

    assign prng_live = (owner_q != OWNER_NONE) && (prng_reset_cnt_q == 2'd2);

    assign ecdsa_grant = (owner_q == OWNER_ECDSA) && prng_live;
    assign mldsa_grant = (owner_q == OWNER_MLDSA) && prng_live;
    assign busy = (owner_q != OWNER_NONE);

    always_comb begin
        prng_i_ready   = 1'b0;
        prng_i_data    = '0;
        prng_i_valid   = 1'b0;
        prng_i_need    = 1'b0;
        prng_i_last_pos= '0;
        prng_i_last    = 1'b0;
        prng_i_mode    = 3'd3;
        prng_i_start   = 1'b0;

        unique case (owner_q)
            OWNER_ECDSA: begin
                prng_i_ready    = ecdsa_i_ready;
                prng_i_data     = ecdsa_i_data;
                prng_i_valid    = ecdsa_i_valid;
                prng_i_need     = ecdsa_i_need;
                prng_i_last_pos = ecdsa_i_last_pos;
                prng_i_last     = ecdsa_i_last;
                prng_i_mode     = ecdsa_i_mode;
                prng_i_start    = ecdsa_i_start;
            end
            OWNER_MLDSA: begin
                prng_i_ready    = mldsa_i_ready;
                prng_i_data     = mldsa_i_data;
                prng_i_valid    = mldsa_i_valid;
                prng_i_need     = mldsa_i_need;
                prng_i_last_pos = mldsa_i_last_pos;
                prng_i_last     = mldsa_i_last;
                prng_i_mode     = mldsa_i_mode;
                prng_i_start    = mldsa_i_start;
            end
            default: ;
        endcase
    end

    // Keep the PRNG/Keccak engine in reset while unowned, and for two cycles
    // after ownership changes, so the first granted cycle always starts from a
    // known sponge state.
    assign prng_rst_n = rst_n && prng_live;

    PRNG #(.DATA_WIDTH(DATA_WIDTH)) u_prng (
        .i_clk      (clk),
        .i_rst_n    (prng_rst_n),
        .i_ready    (prng_i_ready),
        .o_need     (prng_o_need),
        .i_data     (prng_i_data),
        .i_valid    (prng_i_valid),
        .o_ready    (prng_o_ready),
        .i_need     (prng_i_need),
        .o_data     (prng_o_data),
        .o_valid    (prng_o_valid),
        .i_last_pos (prng_i_last_pos),
        .i_last     (prng_i_last),
        .i_mode     (prng_i_mode),
        .i_start    (prng_i_start),
        .o_done     (prng_o_done),
        .o_idle     (prng_o_idle)
    );

    assign ecdsa_o_need  = ecdsa_grant ? prng_o_need  : 1'b0;
    assign ecdsa_o_ready = ecdsa_grant ? prng_o_ready : 1'b0;
    assign ecdsa_o_data  = ecdsa_grant ? prng_o_data  : '0;
    assign ecdsa_o_valid = ecdsa_grant ? prng_o_valid : 1'b0;
    assign ecdsa_o_done  = ecdsa_grant ? prng_o_done  : 1'b0;
    assign ecdsa_o_idle  = ecdsa_grant ? prng_o_idle  : 1'b1;

    assign mldsa_o_need  = mldsa_grant ? prng_o_need  : 1'b0;
    assign mldsa_o_ready = mldsa_grant ? prng_o_ready : 1'b0;
    assign mldsa_o_data  = mldsa_grant ? prng_o_data  : '0;
    assign mldsa_o_valid = mldsa_grant ? prng_o_valid : 1'b0;
    assign mldsa_o_done  = mldsa_grant ? prng_o_done  : 1'b0;
    assign mldsa_o_idle  = mldsa_grant ? prng_o_idle  : 1'b1;
endmodule

`default_nettype wire
