/*
作者：唐金钰
时间：2026/7/29
概述：是共享服务的客户端代理

*/

`timescale 1ns/1ps
`default_nettype none

module mldsa_keccak_xof_adapter (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire        mode_shake256,
    input  wire [63:0] in_data,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire        in_last,
    input  wire [7:0]  in_last_pos,
    output wire [63:0] out_data,
    output wire        out_valid,
    input  wire        out_ready,
    input  wire        squeeze_done,
    output wire        squeeze_ready,
    output logic       absorb_done,
    output wire        busy,
    output logic       done
);

    localparam logic [2:0] PRNG_SHAKE128 = 3'd4;
    localparam logic [2:0] PRNG_SHAKE256 = 3'd5;
//差别只是 rate（吸收区大小）：SHAKE128 rate=1344bit=21个64bit字，SHAKE256 rate=1088bit=17个字
//expandA用SHAKE128（快），μ/c̃/种子扩展用 SHAKE256（安全性要求高）
    typedef enum logic [2:0] {
        S_IDLE,
        S_KICK,
        S_START,
        S_ABSORB,
        S_SQUEEZE,
        S_DONE
    } state_e;

    state_e st, st_n;

    logic        prng_i_ready;
    logic        prng_o_need;
    logic [63:0] prng_i_data;
    logic        prng_i_valid;
    logic        prng_o_ready;
    logic        prng_i_need;
    logic [63:0] prng_o_data;
    logic        prng_o_valid;
    logic [7:0]  prng_i_last_pos;
    logic        prng_i_last;
    logic [2:0]  prng_i_mode;
    logic        prng_i_start;
    logic        prng_o_done;
    logic        prng_o_idle;
    logic        mode_q;
    logic        svc_req;
    logic        svc_release;
    logic        svc_grant;
    logic        buf_valid;
    logic [63:0] buf_data;
    logic        buf_last;
    logic [7:0]  buf_last_pos;
    logic        prng_accept;

    // Step-3 integration: ML-DSA uses the shared Keccak service through the
    // ML-DSA client port. The ECDSA client side is tied off in this local
    // adapter-level integration step.
    // This simple mutual-exclusion service assumes ECDSA and ML-DSA do not
    // issue concurrent Keccak requests; concurrent correctness is not
    // guaranteed without a real arbiter.
    shared_keccak_service u_shared_keccak (
        .clk(clk),
        .rst_n(rst_n),
        .ecdsa_req(1'b0),
        .ecdsa_release(1'b0),
        .ecdsa_grant(),
        .ecdsa_i_ready(1'b0),
        .ecdsa_o_need(),
        .ecdsa_i_data(64'd0),
        .ecdsa_i_valid(1'b0),
        .ecdsa_o_ready(),
        .ecdsa_i_need(1'b0),
        .ecdsa_o_data(),
        .ecdsa_o_valid(),
        .ecdsa_i_last_pos(8'd0),
        .ecdsa_i_last(1'b0),
        .ecdsa_i_mode(3'd3),
        .ecdsa_i_start(1'b0),
        .ecdsa_o_done(),
        .ecdsa_o_idle(),
        .mldsa_req(svc_req),
        .mldsa_release(svc_release),
        .mldsa_grant(svc_grant),
        .mldsa_i_ready(prng_i_ready),
        .mldsa_o_need(prng_o_need),
        .mldsa_i_data(prng_i_data),
        .mldsa_i_valid(prng_i_valid),
        .mldsa_o_ready(prng_o_ready),
        .mldsa_i_need(prng_i_need),
        .mldsa_o_data(prng_o_data),
        .mldsa_o_valid(prng_o_valid),
        .mldsa_i_last_pos(prng_i_last_pos),
        .mldsa_i_last(prng_i_last),
        .mldsa_i_mode(prng_i_mode),
        .mldsa_i_start(prng_i_start),
        .mldsa_o_done(prng_o_done),
        .mldsa_o_idle(prng_o_idle),
        .busy()
    );

    always_comb begin
        st_n = st;
        unique case (st)
            S_IDLE:    if (start) st_n = S_KICK;
            S_KICK:    if (svc_grant) st_n = S_START;
            S_START:   st_n = S_ABSORB;
            S_ABSORB:  if (prng_accept && buf_last) st_n = S_SQUEEZE;
            S_SQUEEZE: if (squeeze_done) st_n = S_DONE;
            S_DONE:    st_n = S_IDLE;
            default:   st_n = S_IDLE;
        endcase
    end

    always_comb begin
        prng_i_ready    = 1'b1;
        prng_i_data     = buf_data;
        prng_i_valid    = 1'b0;
        prng_i_need     = 1'b0;
        prng_i_last_pos = buf_last_pos;
        prng_i_last     = 1'b0;
        prng_i_mode     = mode_q ? PRNG_SHAKE256 : PRNG_SHAKE128;
        prng_i_start    = 1'b0;

        unique case (st)
            S_START: begin
                prng_i_start = 1'b1;
                prng_i_mode  = mode_q ? PRNG_SHAKE256 : PRNG_SHAKE128;
            end
            S_ABSORB: begin
                prng_i_valid = buf_valid && prng_o_need;
                prng_i_last  = buf_last && buf_valid && prng_o_need;
            end
            S_SQUEEZE: begin
                prng_i_need = out_ready && prng_o_ready;
            end
            default: ;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            st     <= S_IDLE;
            mode_q <= 1'b0;
            done   <= 1'b0;
            absorb_done <= 1'b0;
            buf_valid <= 1'b0;
            buf_data <= 64'd0;
            buf_last <= 1'b0;
            buf_last_pos <= 8'hff;
        end else begin
            st   <= st_n;
            done <= 1'b0;
            absorb_done <= 1'b0;
            if (st == S_IDLE && start) mode_q <= mode_shake256;
            if (st == S_IDLE && start) begin
                buf_valid <= 1'b0;
                buf_data <= 64'd0;
                buf_last <= 1'b0;
                buf_last_pos <= 8'hff;
            end
            if (prng_accept && in_ready && in_valid) begin
`ifdef MLDSA_DEBUG_DISPLAY
                $display("XOF_ACCEPT data=%016x last=%0b last_pos=%02x mode=%0d",
                         buf_data, buf_last, buf_last_pos, prng_i_mode);
`endif
                buf_valid <= 1'b1;
                buf_data <= in_data;
                buf_last <= in_last;
                buf_last_pos <= in_last_pos;
            end else if (prng_accept) begin
`ifdef MLDSA_DEBUG_DISPLAY
                $display("XOF_ACCEPT data=%016x last=%0b last_pos=%02x mode=%0d",
                         buf_data, buf_last, buf_last_pos, prng_i_mode);
`endif
                buf_valid <= 1'b0;
            end else if (in_ready && in_valid) begin
                buf_valid <= 1'b1;
                buf_data <= in_data;
                buf_last <= in_last;
                buf_last_pos <= in_last_pos;
            end
            if (prng_accept && buf_last) absorb_done <= 1'b1;
            if (st == S_DONE) done <= 1'b1;
        end
    end

    assign svc_req = (st != S_IDLE);
    assign svc_release = (st == S_DONE);
    assign prng_accept   = (st == S_ABSORB) && buf_valid && prng_o_need;
    assign in_ready      = (st == S_ABSORB) && (!buf_valid || prng_accept);
    assign out_valid     = prng_o_valid;
    assign out_data      = out_valid ? prng_o_data : 64'd0;
    assign squeeze_ready = (st == S_SQUEEZE) && prng_o_ready;
    assign busy          = (st != S_IDLE) && (st != S_DONE);
endmodule

`default_nettype wire
