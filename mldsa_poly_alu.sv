`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

module mldsa_poly_alu #(
    parameter int BANKS = 16
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         start,
    input  wire [2:0]                   op_code,
    input  wire [$clog2(BANKS)-1:0]     src0_bank,
    input  wire [$clog2(BANKS)-1:0]     src1_bank,
    input  wire [$clog2(BANKS)-1:0]     dst_bank,
    output logic                        rd0_en,
    output logic [$clog2(BANKS)-1:0]    rd0_bank,
    output logic [7:0]                  rd0_addr,
    input  wire [MLDSA_COEFF_W-1:0]     rd0_data,
    input  wire                         rd0_valid,
    output logic                        pair_rd0_en,
    output logic [$clog2(BANKS)-1:0]    pair_rd0_bank,
    output logic [6:0]                  pair_rd0_addr,
    input  wire [47:0]                  pair_rd0_data,
    input  wire                         pair_rd0_valid,
    output logic                        rd1_en,
    output logic [$clog2(BANKS)-1:0]    rd1_bank,
    output logic [7:0]                  rd1_addr,
    input  wire [MLDSA_COEFF_W-1:0]     rd1_data,
    input  wire                         rd1_valid,
    output logic                        pair_rd1_en,
    output logic [$clog2(BANKS)-1:0]    pair_rd1_bank,
    output logic [6:0]                  pair_rd1_addr,
    input  wire [47:0]                  pair_rd1_data,
    input  wire                         pair_rd1_valid,
    output logic                        wr_en,
    output logic [$clog2(BANKS)-1:0]    wr_bank,
    output logic [7:0]                  wr_addr,
    output logic [MLDSA_COEFF_W-1:0]    wr_data,
    output logic                        pair_wr_en,
    output logic [$clog2(BANKS)-1:0]    pair_wr_bank,
    output logic [6:0]                  pair_wr_addr,
    output logic [47:0]                 pair_wr_data,
    output logic                        busy,
    output logic                        done,
    output logic [3:0]                  state_dbg
);

    typedef enum logic [4:0] {
        S_IDLE,
        S_READ_AB,
        S_EXEC,
        S_READ_ACC,
        S_ACC_EXEC,
        S_PIPE_READ,
        S_PIPE_ISSUE,
        S_PIPE_DRAIN,
        S_PIPE_FLUSH,
        S_MAC_LOAD_READ,
        S_MAC_LOAD_CAP,
        S_MAC_PIPE_READ,
        S_MAC_PIPE_ISSUE,
        S_MAC_PIPE_DRAIN,
        S_MAC_WRITE,
        S_WRITE,
        S_NEXT,
        S_DONE
    } state_e;

    state_e st;
    logic [2:0] op_q;
    logic [$clog2(BANKS)-1:0] src0_bank_q;
    logic [$clog2(BANKS)-1:0] src1_bank_q;
    logic [$clog2(BANKS)-1:0] dst_bank_q;
    logic [7:0] coeff_idx;
    logic [MLDSA_COEFF_W-1:0] a_reg;
    logic [MLDSA_COEFF_W-1:0] b_reg;
    logic [MLDSA_COEFF_W-1:0] result_reg;
    logic pipe_in_valid;
    logic pipe_in_ready;
    logic pipe_odd_in_ready;
    logic [MLDSA_COEFF_W-1:0] pipe_a;
    logic [MLDSA_COEFF_W-1:0] pipe_b;
    logic [MLDSA_COEFF_W-1:0] pipe_odd_a;
    logic [MLDSA_COEFF_W-1:0] pipe_odd_b;
    logic [MLDSA_COEFF_W-1:0] pipe_result;
    logic [MLDSA_COEFF_W-1:0] pipe_odd_result;
    logic pipe_out_valid;
    logic pipe_odd_out_valid;
    logic [7:0] pipe_addr_d0;
    logic [7:0] pipe_addr_d1;
    logic [7:0] pipe_addr_d2;
    logic [7:0] pipe_addr_d3;
    logic [9:0] pipe_pending_q;
    logic pipe_wr_valid_q;
    logic pipe_fire;
    logic [7:0] pipe_wr_addr_q;
    logic [MLDSA_COEFF_W-1:0] pipe_wr_data_q;
    logic [8:0] coeff_idx_next;
    logic [MLDSA_COEFF_W-1:0] mac_acc_rf [0:MLDSA_N-1];
    logic mac_acc_wr_valid_q;
    logic [7:0] mac_acc_wr_addr_q;
    logic [MLDSA_COEFF_W-1:0] mac_acc_wr_data_q;

    mldsa_modq_mul_pipe u_mul_pipe (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (pipe_in_valid),
        .a         (pipe_a),
        .b         (pipe_b),
        .in_ready  (pipe_in_ready),
        .out_valid (pipe_out_valid),
        .result    (pipe_result)
    );

    mldsa_modq_mul_pipe u_mul_pipe_odd (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (pipe_in_valid && (op_q == MLDSA_POLY_MAC)),
        .a         (pipe_odd_a),
        .b         (pipe_odd_b),
        .in_ready  (pipe_odd_in_ready),
        .out_valid (pipe_odd_out_valid),
        .result    (pipe_odd_result)
    );

    assign pipe_fire = pipe_in_valid && pipe_in_ready && ((op_q != MLDSA_POLY_MAC) || pipe_odd_in_ready);

    always_comb begin
        rd0_en    = 1'b0;
        rd0_bank  = src0_bank_q;
        rd0_addr  = coeff_idx;
        pair_rd0_en   = 1'b0;
        pair_rd0_bank = src0_bank_q;
        pair_rd0_addr = coeff_idx[6:0];
        rd1_en    = 1'b0;
        rd1_bank  = src1_bank_q;
        rd1_addr  = coeff_idx;
        pair_rd1_en   = 1'b0;
        pair_rd1_bank = src1_bank_q;
        pair_rd1_addr = coeff_idx[6:0];
        wr_en     = 1'b0;
        wr_bank   = dst_bank_q;
        wr_addr   = coeff_idx;
        wr_data   = result_reg;
        pair_wr_en   = 1'b0;
        pair_wr_bank = dst_bank_q;
        pair_wr_addr = coeff_idx[6:0];
        pair_wr_data = {
            mac_acc_rf[{coeff_idx[6:0], 1'b1}],
            mac_acc_rf[{coeff_idx[6:0], 1'b0}]
        };
        pipe_in_valid = 1'b0;
        pipe_a        = a_reg;
        pipe_b        = b_reg;
        pipe_odd_a    = 24'd0;
        pipe_odd_b    = 24'd0;
        coeff_idx_next = {1'b0, coeff_idx} + 9'd1;
        busy      = (st != S_IDLE) && (st != S_DONE);
        state_dbg = st[3:0];

        unique case (st)
            S_READ_AB: begin
                rd0_en = 1'b1;
                rd1_en = (op_q != MLDSA_POLY_COPY);
            end
            S_READ_ACC: begin
                rd0_en   = 1'b1;
                rd0_bank = dst_bank_q;
            end
            S_PIPE_READ: begin
                rd0_en = 1'b1;
                rd1_en = 1'b1;
            end
            S_PIPE_ISSUE: begin
            if (rd0_valid && rd1_valid) begin
                pipe_in_valid = 1'b1;
                pipe_a        = rd0_data;
                pipe_b        = rd1_data;
                if (coeff_idx != 8'd255) begin
                    rd0_en   = 1'b1;
                    rd1_en   = 1'b1;
                    rd0_addr = coeff_idx_next[7:0];
                    rd1_addr = coeff_idx_next[7:0];
                end
            end
        end
            S_WRITE: begin
                wr_en = 1'b1;
            end
            S_MAC_PIPE_READ: begin
                pair_rd0_en = 1'b1;
                pair_rd1_en = 1'b1;
            end
            S_MAC_PIPE_ISSUE: begin
            if (pair_rd0_valid && pair_rd1_valid) begin
                pipe_in_valid = 1'b1;
                pipe_a        = pair_rd0_data[23:0];
                pipe_b        = pair_rd1_data[23:0];
                pipe_odd_a    = pair_rd0_data[47:24];
                pipe_odd_b    = pair_rd1_data[47:24];
                if (coeff_idx != 8'd127) begin
                    pair_rd0_en   = 1'b1;
                    pair_rd1_en   = 1'b1;
                    pair_rd0_addr = coeff_idx_next[6:0];
                    pair_rd1_addr = coeff_idx_next[6:0];
                end
            end
        end
            S_MAC_LOAD_READ: begin
                pair_rd0_en   = 1'b1;
                pair_rd0_bank = dst_bank_q;
            end
            S_MAC_LOAD_CAP: begin
                pair_rd0_bank = dst_bank_q;
                if (coeff_idx != 8'd127) begin
                    pair_rd0_en   = 1'b1;
                    pair_rd0_addr = coeff_idx_next[6:0];
                end
            end
            S_MAC_WRITE: begin
                pair_wr_en   = 1'b1;
                pair_wr_bank = dst_bank_q;
                pair_wr_addr = coeff_idx[6:0];
            end
            default: ;
        endcase

        if (pipe_wr_valid_q) begin
            wr_en   = 1'b1;
            wr_bank = dst_bank_q;
            wr_addr = pipe_wr_addr_q;
            wr_data = pipe_wr_data_q;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st          <= S_IDLE;
            op_q        <= MLDSA_POLY_COPY;
            src0_bank_q <= '0;
            src1_bank_q <= '0;
            dst_bank_q  <= '0;
            coeff_idx   <= 8'd0;
            a_reg       <= 24'd0;
            b_reg       <= 24'd0;
            result_reg  <= 24'd0;
            pipe_addr_d0 <= 8'd0;
            pipe_addr_d1 <= 8'd0;
            pipe_addr_d2 <= 8'd0;
            pipe_addr_d3 <= 8'd0;
            pipe_pending_q <= 10'd0;
            pipe_wr_valid_q <= 1'b0;
            pipe_wr_addr_q  <= 8'd0;
            pipe_wr_data_q  <= 24'd0;
            mac_acc_wr_valid_q <= 1'b0;
            mac_acc_wr_addr_q  <= 8'd0;
            mac_acc_wr_data_q  <= 24'd0;
            done        <= 1'b0;
        end else begin
            done <= 1'b0;
            pipe_wr_valid_q <= 1'b0;
            mac_acc_wr_valid_q <= 1'b0;
            pipe_addr_d1 <= pipe_addr_d0;
            pipe_addr_d2 <= pipe_addr_d1;
            pipe_addr_d3 <= pipe_addr_d2;
            pipe_pending_q <= pipe_pending_q
                              + (pipe_fire ? 10'd1 : 10'd0)
                              - (pipe_out_valid ? 10'd1 : 10'd0);
            if (pipe_out_valid && op_q == MLDSA_POLY_MUL) begin
                pipe_wr_valid_q <= 1'b1;
                pipe_wr_addr_q  <= pipe_addr_d2;
                pipe_wr_data_q  <= pipe_result;
`ifdef MLDSA_POLY_DEBUG
                if (pipe_addr_d2 < 8'd4) begin
                    $display("[POLY_DBG] wrcap t=%0t addr=%0d data=%0d", $time, pipe_addr_d2, pipe_result);
                end
`endif
            end
            if (pipe_out_valid && op_q == MLDSA_POLY_MAC) begin
                mac_acc_rf[{pipe_addr_d2[6:0], 1'b0}] <= modq_add(mac_acc_rf[{pipe_addr_d2[6:0], 1'b0}], pipe_result);
                mac_acc_rf[{pipe_addr_d2[6:0], 1'b1}] <= modq_add(mac_acc_rf[{pipe_addr_d2[6:0], 1'b1}], pipe_odd_result);
            end
            unique case (st)
                S_IDLE: begin
                    if (start) begin
                        op_q        <= op_code;
                        src0_bank_q <= src0_bank;
                        src1_bank_q <= src1_bank;
                        dst_bank_q  <= dst_bank;
                        coeff_idx   <= 8'd0;
                        pipe_pending_q <= 10'd0;
                        pipe_wr_valid_q <= 1'b0;
                        mac_acc_wr_valid_q <= 1'b0;
                        st          <= (op_code == MLDSA_POLY_MUL) ? S_PIPE_READ :
                                       (op_code == MLDSA_POLY_MAC) ? S_MAC_LOAD_READ : S_READ_AB;
                    end
                end
                S_READ_AB: begin
                    st <= S_EXEC;
                end
                S_EXEC: begin
                    if (rd0_valid && (op_q == MLDSA_POLY_COPY || rd1_valid)) begin
                    a_reg <= rd0_data;
                    b_reg <= rd1_data;
                    unique case (op_q)
                        MLDSA_POLY_COPY: begin
                            result_reg <= rd0_data;
                            st         <= S_WRITE;
                        end
                        MLDSA_POLY_ADD: begin
                            result_reg <= modq_add(rd0_data, rd1_data);
                            st         <= S_WRITE;
                        end
                        MLDSA_POLY_SUB: begin
                            result_reg <= modq_sub(rd0_data, rd1_data);
                            st         <= S_WRITE;
                        end
                        default: begin
                            result_reg <= 24'd0;
                            st         <= S_WRITE;
                        end
                    endcase
                    end
                end
                S_READ_ACC: begin
                    st <= S_ACC_EXEC;
                end
                S_ACC_EXEC: begin
                    if (rd0_valid) begin
                        result_reg <= modq_add(rd0_data, result_reg);
                        st <= S_WRITE;
                    end
                end
                S_PIPE_READ: begin
                    st <= S_PIPE_ISSUE;
                end
                S_PIPE_ISSUE: begin
                    if (pipe_fire) begin
                    pipe_addr_d0 <= coeff_idx;
`ifdef MLDSA_POLY_DEBUG
                    if (coeff_idx < 8'd4) begin
                        $display("[POLY_DBG] issue t=%0t idx=%0d a=%0d b=%0d", $time, coeff_idx, rd0_data, rd1_data);
                    end
`endif
                    if (coeff_idx == 8'd255) begin
                        st <= S_PIPE_DRAIN;
                    end else begin
                        coeff_idx <= coeff_idx + 8'd1;
                        st <= S_PIPE_ISSUE;
                    end
                    end
                end
                S_PIPE_DRAIN: begin
                    if (pipe_pending_q == (pipe_out_valid ? 10'd1 : 10'd0)) begin
                        st <= S_PIPE_FLUSH;
                    end
                end
                S_PIPE_FLUSH: begin
                    st <= S_DONE;
                end
                S_MAC_LOAD_READ: begin
                    st <= S_MAC_LOAD_CAP;
                end
                S_MAC_LOAD_CAP: begin
                    if (pair_rd0_valid) begin
                    mac_acc_rf[{coeff_idx[6:0], 1'b0}] <= pair_rd0_data[23:0];
                    mac_acc_rf[{coeff_idx[6:0], 1'b1}] <= pair_rd0_data[47:24];
                    if (coeff_idx == 8'd127) begin
                        coeff_idx <= 8'd0;
                        st        <= S_MAC_PIPE_READ;
                    end else begin
                        coeff_idx <= coeff_idx + 8'd1;
                        st        <= S_MAC_LOAD_CAP;
                    end
                    end
                end
                S_MAC_PIPE_READ: begin
                    pipe_pending_q <= 10'd0;
                    st <= S_MAC_PIPE_ISSUE;
                end
                S_MAC_PIPE_ISSUE: begin
                    if (pipe_fire) begin
                    pipe_addr_d0 <= coeff_idx;
                    if (coeff_idx == 8'd127) begin
                        st <= S_MAC_PIPE_DRAIN;
                    end else begin
                        coeff_idx <= coeff_idx + 8'd1;
                        st <= S_MAC_PIPE_ISSUE;
                    end
                    end
                end
                S_MAC_PIPE_DRAIN: begin
                    if (pipe_pending_q == (pipe_out_valid ? 10'd1 : 10'd0)) begin
                        coeff_idx <= 8'd0;
                        st        <= S_MAC_WRITE;
                    end
                end
                S_MAC_WRITE: begin
                    if (coeff_idx == 8'd127) begin
                        st <= S_DONE;
                    end else begin
                        coeff_idx <= coeff_idx + 8'd1;
                        st        <= S_MAC_WRITE;
                    end
                end
                S_WRITE: begin
                    st <= S_NEXT;
                end
                S_NEXT: begin
                    if (coeff_idx == 8'd255) begin
                        st <= S_DONE;
                    end else begin
                        coeff_idx <= coeff_idx + 8'd1;
                        if (op_q == MLDSA_POLY_MAC) st <= S_MAC_PIPE_READ;
                        else                         st <= S_READ_AB;
                    end
                end
                S_DONE: begin
                    done <= 1'b1;
                    st   <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
