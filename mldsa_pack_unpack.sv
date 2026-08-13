`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

module mldsa_pack_unpack (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         start,
    input  wire [3:0]                   mode,
    input  wire [15:0]                  item_count,
    input  wire [MLDSA_COEFF_W-1:0]     coeff_in,
    input  wire                         coeff_in_valid,
    output logic                        coeff_in_ready,
    output logic [7:0]                  byte_out,
    output logic                        byte_out_valid,
    input  wire                         byte_out_ready,
    input  wire [7:0]                   byte_in,
    input  wire                         byte_in_valid,
    output logic                        byte_in_ready,
    output logic [MLDSA_COEFF_W-1:0]    coeff_out,
    output logic                        coeff_out_valid,
    input  wire                         coeff_out_ready,
    output logic                        busy,
    output logic                        done,
    output logic                        error,
    output logic [4:0]                  state_dbg
);

    typedef enum logic [4:0] {
        S_IDLE,
        S_PACK_WAIT,
        S_PACK_EMIT0,
        S_PACK_EMIT1,
        S_PACK_EMIT2,
        S_UNPACK_WAIT0,
        S_UNPACK_WAIT1,
        S_UNPACK_WAIT2,
        S_UNPACK_EMIT,
        S_BITPACK_WAIT,
        S_BITPACK_EMIT,
        S_BITUNPACK_WAIT,
        S_BITUNPACK_EMIT,
        S_VARPACK_WAIT,
        S_VARPACK_ACCUM,
        S_VARPACK_EMIT,
        S_VARPACK_FLUSH,
        S_VARUNPACK_WAIT,
        S_VARUNPACK_EMIT,
        S_DONE,
        S_ERROR
    } state_e;

    state_e st;
    logic [3:0] mode_q;
    logic [15:0] item_count_q;
    logic [15:0] item_idx;
    logic [MLDSA_COEFF_W-1:0] coeff_q;
    logic [7:0] byte_q;
    logic [7:0] bit_accum;
    logic [2:0] bit_idx;
    logic [1:0] coeff_byte_idx;
    logic [31:0] var_bitbuf;
    logic [5:0]  var_bitcount;
    logic [4:0]  var_word_bits;
    logic        var_last_item;
    logic [MLDSA_COEFF_W-1:0] var_word_mask;
    logic [MLDSA_COEFF_W:0] var_word_mask_ext;
    logic [31:0] var_coeff_bits;

    always_comb begin
        if (var_word_bits == 5'd0) var_word_mask_ext = '0;
        else var_word_mask_ext = ({{MLDSA_COEFF_W{1'b0}}, 1'b1} << var_word_bits) - {{MLDSA_COEFF_W{1'b0}}, 1'b1};
        var_word_mask = var_word_mask_ext[MLDSA_COEFF_W-1:0];
        var_coeff_bits = {8'd0, (coeff_in & var_word_mask)};
    end

    function automatic logic [4:0] packed_word_bits(input logic [3:0] pack_mode);
        begin
            unique case (pack_mode)
                MLDSA_PACK_NIBBLE4_PACK,
                MLDSA_PACK_NIBBLE4_UNPACK: packed_word_bits = 5'd4;
                MLDSA_PACK_BITS6_PACK,
                MLDSA_PACK_BITS6_UNPACK:    packed_word_bits = 5'd6;
                MLDSA_PACK_BITS10_PACK,
                MLDSA_PACK_BITS10_UNPACK:  packed_word_bits = 5'd10;
                default:                   packed_word_bits = 5'd0;
            endcase
        end
    endfunction

    always_comb begin
        coeff_in_ready = 1'b0;
        byte_in_ready  = 1'b0;
        busy           = (st != S_IDLE) && (st != S_DONE) && (st != S_ERROR);
        state_dbg      = st;
        unique case (st)
            S_PACK_WAIT,
            S_BITPACK_WAIT,
            S_VARPACK_WAIT: coeff_in_ready = !byte_out_valid && !coeff_out_valid;
            S_UNPACK_WAIT0,
            S_UNPACK_WAIT1,
            S_UNPACK_WAIT2,
            S_BITUNPACK_WAIT: byte_in_ready = !coeff_out_valid && !byte_out_valid;
            S_VARUNPACK_WAIT: byte_in_ready = !coeff_out_valid && !byte_out_valid &&
                                             (var_bitcount < var_word_bits);
            default: ;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st            <= S_IDLE;
            mode_q        <= MLDSA_PACK_COEFF24_PACK;
            item_count_q  <= 16'd0;
            item_idx      <= 16'd0;
            coeff_q       <= '0;
            byte_q        <= 8'd0;
            bit_accum     <= 8'd0;
            bit_idx       <= 3'd0;
            coeff_byte_idx <= 2'd0;
            var_bitbuf    <= 32'd0;
            var_bitcount  <= 6'd0;
            var_word_bits <= 5'd0;
            var_last_item <= 1'b0;
            byte_out      <= 8'd0;
            byte_out_valid <= 1'b0;
            coeff_out     <= '0;
            coeff_out_valid <= 1'b0;
            done          <= 1'b0;
            error         <= 1'b0;
        end else begin
            done <= 1'b0;

            if (byte_out_valid && byte_out_ready) byte_out_valid <= 1'b0;
            if (coeff_out_valid && coeff_out_ready) coeff_out_valid <= 1'b0;

            unique case (st)
                S_IDLE: begin
                    error <= 1'b0;
                    if (start) begin
                        mode_q       <= mode;
                        item_count_q <= item_count;
                        item_idx     <= 16'd0;
                        bit_accum    <= 8'd0;
                        bit_idx      <= 3'd0;
                        var_bitbuf   <= 32'd0;
                        var_bitcount <= 6'd0;
                        var_word_bits <= packed_word_bits(mode);
                        var_last_item <= 1'b0;
                        if (item_count == 16'd0) begin
                            st <= S_DONE;
                        end else if (mode == MLDSA_PACK_COEFF24_PACK) st <= S_PACK_WAIT;
                        else if (mode == MLDSA_PACK_COEFF24_UNPACK) st <= S_UNPACK_WAIT0;
                        else if (mode == MLDSA_PACK_BIT1_PACK) st <= S_BITPACK_WAIT;
                        else if (mode == MLDSA_PACK_BIT1_UNPACK) st <= S_BITUNPACK_WAIT;
                        else if (mode == MLDSA_PACK_NIBBLE4_PACK || mode == MLDSA_PACK_BITS6_PACK || mode == MLDSA_PACK_BITS10_PACK) st <= S_VARPACK_WAIT;
                        else if (mode == MLDSA_PACK_NIBBLE4_UNPACK || mode == MLDSA_PACK_BITS6_UNPACK || mode == MLDSA_PACK_BITS10_UNPACK) st <= S_VARUNPACK_WAIT;
                        else st <= S_ERROR;
                    end
                end
                S_PACK_WAIT: begin
                    if (coeff_in_valid && coeff_in_ready) begin
                        coeff_q <= coeff_in;
                        st <= S_PACK_EMIT0;
                    end
                end
                S_PACK_EMIT0: begin
                    if (!byte_out_valid) begin
                        byte_out <= coeff_q[7:0];
                        byte_out_valid <= 1'b1;
                        st <= S_PACK_EMIT1;
                    end
                end
                S_PACK_EMIT1: begin
                    if (!byte_out_valid) begin
                        byte_out <= coeff_q[15:8];
                        byte_out_valid <= 1'b1;
                        st <= S_PACK_EMIT2;
                    end
                end
                S_PACK_EMIT2: begin
                    if (!byte_out_valid) begin
                        byte_out <= coeff_q[23:16];
                        byte_out_valid <= 1'b1;
                        if (item_idx == item_count_q - 16'd1) st <= S_DONE;
                        else begin
                            item_idx <= item_idx + 16'd1;
                            st <= S_PACK_WAIT;
                        end
                    end
                end
                S_UNPACK_WAIT0: begin
                    if (byte_in_valid && byte_in_ready) begin
                        coeff_q[7:0] <= byte_in;
                        st <= S_UNPACK_WAIT1;
                    end
                end
                S_UNPACK_WAIT1: begin
                    if (byte_in_valid && byte_in_ready) begin
                        coeff_q[15:8] <= byte_in;
                        st <= S_UNPACK_WAIT2;
                    end
                end
                S_UNPACK_WAIT2: begin
                    if (byte_in_valid && byte_in_ready) begin
                        coeff_q[23:16] <= byte_in;
                        st <= S_UNPACK_EMIT;
                    end
                end
                S_UNPACK_EMIT: begin
                    if (!coeff_out_valid) begin
                        coeff_out <= coeff_q;
                        coeff_out_valid <= 1'b1;
                        if (item_idx == item_count_q - 16'd1) st <= S_DONE;
                        else begin
                            item_idx <= item_idx + 16'd1;
                            st <= S_UNPACK_WAIT0;
                        end
                    end
                end
                S_BITPACK_WAIT: begin
                    if (coeff_in_valid && coeff_in_ready) begin
                        bit_accum[bit_idx] <= coeff_in[0];
                        if (bit_idx == 3'd7 || item_idx == item_count_q - 16'd1) begin
                            st <= S_BITPACK_EMIT;
                        end else begin
                            bit_idx <= bit_idx + 3'd1;
                            item_idx <= item_idx + 16'd1;
                        end
                    end
                end
                S_BITPACK_EMIT: begin
                    if (!byte_out_valid) begin
                        byte_out <= bit_accum;
                        byte_out_valid <= 1'b1;
                        if (item_idx == item_count_q - 16'd1) begin
                            st <= S_DONE;
                        end else begin
                            item_idx <= item_idx + 16'd1;
                            bit_idx <= 3'd0;
                            bit_accum <= 8'd0;
                            st <= S_BITPACK_WAIT;
                        end
                    end
                end
                S_BITUNPACK_WAIT: begin
                    if (byte_in_valid && byte_in_ready) begin
                        byte_q <= byte_in;
                        bit_idx <= 3'd0;
                        st <= S_BITUNPACK_EMIT;
                    end
                end
                S_BITUNPACK_EMIT: begin
                    if (!coeff_out_valid) begin
                        coeff_out <= {23'd0, byte_q[bit_idx]};
                        coeff_out_valid <= 1'b1;
                        if (item_idx == item_count_q - 16'd1) begin
                            st <= S_DONE;
                        end else begin
                            item_idx <= item_idx + 16'd1;
                            if (bit_idx == 3'd7) st <= S_BITUNPACK_WAIT;
                            else bit_idx <= bit_idx + 3'd1;
                        end
                    end
                end
                S_VARPACK_WAIT: begin
                    if (coeff_in_valid && coeff_in_ready) begin
                        coeff_q       <= coeff_in & var_word_mask;
`ifdef MLDSA_TARGET_FPGA
                        var_last_item <= (item_idx == item_count_q - 16'd1);
                        st            <= S_VARPACK_ACCUM;
`else
                        var_bitbuf    <= var_bitbuf | (var_coeff_bits << var_bitcount);
                        var_bitcount  <= var_bitcount + {1'b0, var_word_bits};
                        var_last_item <= (item_idx == item_count_q - 16'd1);
                        st            <= S_VARPACK_EMIT;
`endif
                    end
                end
                S_VARPACK_ACCUM: begin
                    var_bitbuf   <= var_bitbuf | ({8'd0, coeff_q} << var_bitcount);
                    var_bitcount <= var_bitcount + {1'b0, var_word_bits};
                    st           <= S_VARPACK_EMIT;
                end
                S_VARPACK_EMIT: begin
                    if ((var_bitcount >= 6'd8) && !byte_out_valid) begin
                        byte_out       <= var_bitbuf[7:0];
                        byte_out_valid <= 1'b1;
                        var_bitbuf     <= var_bitbuf >> 8;
                        var_bitcount   <= var_bitcount - 6'd8;
                        if ((var_bitcount - 6'd8) >= 6'd8) begin
                            st <= S_VARPACK_EMIT;
                        end else if (var_last_item) begin
                            if ((var_bitcount - 6'd8) == 6'd0) st <= S_DONE;
                            else st <= S_VARPACK_FLUSH;
                        end else begin
                            item_idx <= item_idx + 16'd1;
                            st <= S_VARPACK_WAIT;
                        end
                    end else if (var_bitcount < 6'd8) begin
                        if (var_last_item) st <= S_VARPACK_FLUSH;
                        else begin
                            item_idx <= item_idx + 16'd1;
                            st <= S_VARPACK_WAIT;
                        end
                    end
                end
                S_VARPACK_FLUSH: begin
                    if (!byte_out_valid) begin
                        if (var_bitcount == 6'd0) begin
                            st <= S_DONE;
                        end else begin
                            byte_out       <= var_bitbuf[7:0];
                            byte_out_valid <= 1'b1;
                            if (var_bitcount <= 6'd8) begin
                                var_bitbuf   <= 32'd0;
                                var_bitcount <= 6'd0;
                                st           <= S_DONE;
                            end else begin
                                var_bitbuf   <= var_bitbuf >> 8;
                                var_bitcount <= var_bitcount - 6'd8;
                            end
                        end
                    end
                end
                S_VARUNPACK_WAIT: begin
                    if ((var_bitcount < var_word_bits) && byte_in_valid && byte_in_ready) begin
                        var_bitbuf   <= var_bitbuf | ({24'd0, byte_in} << var_bitcount);
                        var_bitcount <= var_bitcount + 6'd8;
                        if ((var_bitcount + 6'd8) >= {1'b0, var_word_bits}) begin
                            st <= S_VARUNPACK_EMIT;
                        end
                    end
                    if (var_bitcount >= {1'b0, var_word_bits}) begin
                        st <= S_VARUNPACK_EMIT;
                    end
                end
                S_VARUNPACK_EMIT: begin
                    if (!coeff_out_valid) begin
                        coeff_out       <= var_bitbuf[MLDSA_COEFF_W-1:0] & var_word_mask;
                        coeff_out_valid <= 1'b1;
                        var_bitbuf      <= var_bitbuf >> var_word_bits;
                        var_bitcount    <= var_bitcount - {1'b0, var_word_bits};
                        if (item_idx == item_count_q - 16'd1) begin
                            st <= S_DONE;
                        end else begin
                            item_idx <= item_idx + 16'd1;
                            st <= S_VARUNPACK_WAIT;
                        end
                    end
                end
                S_DONE: begin
                    done <= 1'b1;
                    st   <= S_IDLE;
                end
                S_ERROR: begin
                    error <= 1'b1;
                    st    <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
