/*
作者：唐金钰
时间：2026/7/29
概述：64bit和8bit的转换器，因为mldsa都是用8bit但是FIP204定义的输入输出都是64bit字节流，本模块
负责把64bit输入的字节流转换为8bit，再把8bit打包为64bit输出

*/

`timescale 1ns/1ps
`default_nettype none

module mldsa_xof_byte_engine (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire        stop,
    input  wire        mode_shake256,
    input  wire [7:0]  abs_byte_data,
    input  wire        abs_byte_valid,
    input  wire        abs_byte_last,
    output logic       abs_byte_ready,
    output logic [7:0] sq_byte_data,
    output logic       sq_byte_valid,
    input  wire        sq_byte_ready,
    output logic [63:0] sq_word_data,
    output logic        sq_word_valid,
    input  wire         sq_word_ready,
    output wire        busy,
    output logic       state_absorbing,
    output logic       state_squeezing
);
    typedef enum logic [2:0] {
        S_IDLE,
        S_COLLECT,
        S_SEND,
        S_SQUEEZE_REQ,
        S_SQUEEZE_WAIT,
        S_SQUEEZE_SHIFT
    } state_e;

    state_e st;
    logic [63:0] word_buf;
    logic [2:0]  byte_count;
    logic [2:0]  shift_idx;
    logic [63:0] sq_word;
    logic [63:0] prefetch_word;
    logic        prefetch_valid;
    logic        prefetch_pending;
    logic        send_last;
    logic [7:0]  send_last_pos;
    logic        pending_empty_last;
    logic        core_in_valid;
    logic        core_start;
    logic        core_in_ready;
    logic [63:0] core_in_data;
    logic        core_in_last;
    logic [7:0]  core_in_last_pos;
    logic [63:0] core_out_data;
    logic        core_out_valid;
    logic        core_out_ready;
    logic        core_squeeze_ready;
    logic        core_busy;
    logic        core_done;
    logic        start_pending_q;
    logic        prefetch_request_fire;

    mldsa_keccak_xof_adapter u_xof (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (core_start),
        .mode_shake256 (mode_shake256),
        .in_data       (core_in_data),
        .in_valid      (core_in_valid),
        .in_ready      (core_in_ready),
        .in_last       (core_in_last),
        .in_last_pos   (core_in_last_pos),
        .out_data      (core_out_data),
        .out_valid     (core_out_valid),
        .out_ready     (core_out_ready),
        .squeeze_done  (stop),
        .squeeze_ready (core_squeeze_ready),
        .absorb_done   (),
        .busy          (core_busy),
        .done          (core_done)
    );

    function automatic [7:0] last_pos_mask(input logic [2:0] count_value);
        begin
            unique case (count_value)
                3'd0: last_pos_mask = 8'h01;
                3'd1: last_pos_mask = 8'h03;
                3'd2: last_pos_mask = 8'h07;
                3'd3: last_pos_mask = 8'h0F;
                3'd4: last_pos_mask = 8'h1F;
                3'd5: last_pos_mask = 8'h3F;
                3'd6: last_pos_mask = 8'h7F;
                default: last_pos_mask = 8'hFF;
            endcase
        end
    endfunction

    always_comb begin
        abs_byte_ready  = 1'b0;
        sq_byte_valid   = 1'b0;
        sq_byte_data    = sq_word[shift_idx*8 +: 8];
        sq_word_valid   = 1'b0;
        sq_word_data    = sq_word;
        core_in_valid   = 1'b0;
        core_start      = 1'b0;
        core_in_data    = word_buf;
        core_in_last    = send_last;
        core_in_last_pos= send_last_pos;
        core_out_ready  = 1'b0;
        state_absorbing = 1'b0;
        state_squeezing = 1'b0;

        unique case (st)
            S_COLLECT: begin
                abs_byte_ready  = 1'b1;
                state_absorbing = 1'b1;
            end
            S_IDLE: begin
                state_absorbing = 1'b0;
            end
            S_SEND: begin
                core_in_valid   = 1'b1;
                core_start      = start_pending_q;
                state_absorbing = 1'b1;
            end
            S_SQUEEZE_REQ: begin
                core_out_ready  = core_squeeze_ready;
                state_squeezing = 1'b1;
            end
            S_SQUEEZE_WAIT: begin
                state_squeezing = 1'b1;
            end
            S_SQUEEZE_SHIFT: begin
                sq_word_valid   = (shift_idx == 3'd0);
                sq_byte_valid   = !((shift_idx == 3'd0) && sq_word_ready);
                core_out_ready  = core_squeeze_ready && !prefetch_valid && !prefetch_pending;
                state_squeezing = 1'b1;
            end
            default: ;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            st           <= S_IDLE;
            word_buf     <= 64'd0;
            byte_count   <= 3'd0;
            shift_idx    <= 3'd0;
            sq_word      <= 64'd0;
            prefetch_word <= 64'd0;
            prefetch_valid <= 1'b0;
            prefetch_pending <= 1'b0;
            send_last    <= 1'b0;
            send_last_pos<= 8'hFF;
            pending_empty_last <= 1'b0;
            start_pending_q <= 1'b0;
        end else begin
            unique case (st)
                S_IDLE: begin
                    word_buf      <= 64'd0;
                    byte_count    <= 3'd0;
                    shift_idx     <= 3'd0;
                    sq_word       <= 64'd0;
                    prefetch_word <= 64'd0;
                    prefetch_valid <= 1'b0;
                    prefetch_pending <= 1'b0;
                    send_last     <= 1'b0;
                    send_last_pos <= 8'hFF;
                    pending_empty_last <= 1'b0;
                    if (start) begin
                        start_pending_q <= 1'b1;
                        st <= S_COLLECT;
                    end
                end
                S_COLLECT: begin
                    if (abs_byte_valid && abs_byte_ready) begin
                        word_buf[byte_count*8 +: 8] <= abs_byte_data;
                        if (abs_byte_last || byte_count == 3'd7) begin
                            send_last     <= abs_byte_last && (byte_count != 3'd7);
                            send_last_pos <= (abs_byte_last && (byte_count != 3'd7)) ? last_pos_mask(byte_count) : 8'hFF;
                            pending_empty_last <= abs_byte_last && (byte_count == 3'd7);
                            st <= S_SEND;
                        end else begin
                            byte_count <= byte_count + 3'd1;
                        end
                    end
                end
                S_SEND: begin
`ifdef MLDSA_DEBUG_DISPLAY
                    if (send_last || pending_empty_last) begin
                        $display("XOF_BYTE_SEND ready=%0b data=%016x send_last=%0b last_pos=%02x pending_empty=%0b byte_count=%0d",
                                 core_in_ready, word_buf, send_last, send_last_pos, pending_empty_last, byte_count);
                    end
`endif
                    if (core_in_ready) begin
                        word_buf      <= 64'd0;
                        byte_count    <= 3'd0;
                        start_pending_q <= 1'b0;
                        if (send_last) begin
                            st <= S_SQUEEZE_REQ;
                        end else if (pending_empty_last) begin
                            send_last <= 1'b1;
                            send_last_pos <= 8'd0;
                            pending_empty_last <= 1'b0;
                            st <= S_SEND;
                        end else begin
                            st <= S_COLLECT;
                        end
                    end
                end
                S_SQUEEZE_REQ: begin
                    if (stop) begin
                        start_pending_q <= 1'b0;
                        prefetch_valid <= 1'b0;
                        prefetch_pending <= 1'b0;
                        st <= S_IDLE;
                    end else if (core_squeeze_ready) begin
                        st <= S_SQUEEZE_WAIT;
                    end
                end
                S_SQUEEZE_WAIT: begin
                    if (stop) begin
                        start_pending_q <= 1'b0;
                        prefetch_valid <= 1'b0;
                        prefetch_pending <= 1'b0;
                        st <= S_IDLE;
                    end else if (core_out_valid) begin
                        sq_word   <= core_out_data;
                        shift_idx <= 3'd0;
                        prefetch_pending <= 1'b0;
                        st        <= S_SQUEEZE_SHIFT;
                    end
                end
                S_SQUEEZE_SHIFT: begin
                    if (stop) begin
                        start_pending_q <= 1'b0;
                        prefetch_valid <= 1'b0;
                        prefetch_pending <= 1'b0;
                        st <= S_IDLE;
                    end else begin
                        if (prefetch_request_fire) begin
                            prefetch_pending <= 1'b1;
                        end
                        if (core_out_valid && prefetch_pending) begin
                            prefetch_word    <= core_out_data;
                            prefetch_valid   <= 1'b1;
                            prefetch_pending <= 1'b0;
                        end
                        if (sq_word_valid && sq_word_ready) begin
                            if (core_out_valid && prefetch_pending) begin
                                sq_word          <= core_out_data;
                                shift_idx        <= 3'd0;
                                prefetch_valid   <= 1'b0;
                                prefetch_pending <= 1'b0;
                            end else if (prefetch_valid) begin
                                sq_word        <= prefetch_word;
                                shift_idx      <= 3'd0;
                                prefetch_valid <= 1'b0;
                            end else if (prefetch_pending || prefetch_request_fire) begin
                                st <= S_SQUEEZE_WAIT;
                            end else begin
                                st <= S_SQUEEZE_REQ;
                            end
                        end else if (sq_byte_valid && sq_byte_ready) begin
                            if (shift_idx == 3'd7) begin
                                if (core_out_valid && prefetch_pending) begin
                                    sq_word          <= core_out_data;
                                    shift_idx        <= 3'd0;
                                    prefetch_valid   <= 1'b0;
                                    prefetch_pending <= 1'b0;
                                end else if (prefetch_valid) begin
                                    sq_word        <= prefetch_word;
                                    shift_idx      <= 3'd0;
                                    prefetch_valid <= 1'b0;
                                end else if (prefetch_pending || prefetch_request_fire) begin
                                    st <= S_SQUEEZE_WAIT;
                                end else begin
                                    st <= S_SQUEEZE_REQ;
                                end
                            end else begin
                                shift_idx <= shift_idx + 3'd1;
                            end
                        end
                    end
                end
                default: st <= S_IDLE;
            endcase
        end
    end

    assign prefetch_request_fire = (st == S_SQUEEZE_SHIFT) && core_out_ready && core_squeeze_ready;
    assign busy = core_busy || (st != S_IDLE);
endmodule

`default_nettype wire
