`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

module mldsa_ntt_core #(
    parameter int BFU_LAT = 4
) (
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire                      start,
    input  wire                      inverse,
    input  wire                      load_we,
    input  wire [7:0]                load_addr,
    input  wire [MLDSA_COEFF_W-1:0]  load_data,
    input  wire [7:0]                rd_addr,
    output wire [MLDSA_COEFF_W-1:0]  rd_data,
    output logic                     busy,
    output logic                     done,
    output logic [3:0]               state_dbg
);

    localparam logic [MLDSA_COEFF_W-1:0] N_INV = 24'd8347681;
    localparam int NTT_STAGES = 8;

    typedef enum logic [3:0] {
        S_IDLE,
        S_SETUP,
        S_STAGE_ISSUE,
        S_STAGE_DRAIN,
        S_STAGE_SWAP,
        S_SCALE_ISSUE,
        S_SCALE_DRAIN,
        S_SCALE_SWAP,
        S_DONE
    } state_e;

    state_e st;

    logic                     op_inv_q;
    logic                     active_bank_q;
    logic [2:0]               stage_idx_q;
    logic [7:0]               pair_issue_idx_q;
    logic [8:0]               scale_issue_idx_q;

    logic [8:0]               len;
    logic [8:0]               block_start;
    logic [8:0]               j_idx;
    logic [8:0]               k_idx;
    logic [9:0]               pending_q;

    logic                     mem_src_rd_req;
    logic                     mem_src_rd_dual;
    logic [7:0]               mem_src_rd0_addr;
    logic [7:0]               mem_src_rd1_addr;
    logic                     mem_src_rd_valid;
    logic [MLDSA_COEFF_W-1:0] mem_src_rd0_data;
    logic [MLDSA_COEFF_W-1:0] mem_src_rd1_data;
    logic                     mem_dst_wr0_en;
    logic [7:0]               mem_dst_wr0_addr;
    logic [MLDSA_COEFF_W-1:0] mem_dst_wr0_data;
    logic                     mem_dst_wr1_en;
    logic [7:0]               mem_dst_wr1_addr;
    logic [MLDSA_COEFF_W-1:0] mem_dst_wr1_data;

    logic                     req_valid_q;
    logic                     req_scale_q;
    logic                     req_inv_q;
    logic [7:0]               req_dst0_q;
    logic [7:0]               req_dst1_q;
    logic [MLDSA_COEFF_W-1:0] req_zeta_q;

    logic                     hold_valid_q;
    logic                     hold_scale_q;
    logic                     hold_inv_q;
    logic [7:0]               hold_dst0_q;
    logic [7:0]               hold_dst1_q;
    logic [MLDSA_COEFF_W-1:0] hold_a_q;
    logic [MLDSA_COEFF_W-1:0] hold_b_q;
    logic [MLDSA_COEFF_W-1:0] hold_zeta_q;

    logic                     src_data_valid;
    logic                     src_scale;
    logic                     src_inv;
    logic [7:0]               src_dst0;
    logic [7:0]               src_dst1;
    logic [MLDSA_COEFF_W-1:0] src_a;
    logic [MLDSA_COEFF_W-1:0] src_b;
    logic [MLDSA_COEFF_W-1:0] src_zeta;

    logic                     bfu_in_valid;
    logic                     bfu_in_ready;
    logic [MLDSA_COEFF_W-1:0] bfu_a_in;
    logic [MLDSA_COEFF_W-1:0] bfu_b_in;
    logic [MLDSA_COEFF_W-1:0] bfu_zeta;
    logic                     bfu_inverse;
    logic                     issue_fire;
    logic [MLDSA_COEFF_W-1:0] bfu_a_out;
    logic [MLDSA_COEFF_W-1:0] bfu_b_out;
    logic                     bfu_out_valid;

    logic [BFU_LAT-1:0]                 wr_valid_pipe_q;
    logic [BFU_LAT-1:0]                 wr_scale_pipe_q;
    logic [7:0]                         wr_addr0_pipe_q [0:BFU_LAT-1];
    logic [7:0]                         wr_addr1_pipe_q [0:BFU_LAT-1];

    logic [7:0]               fwd_rd0_addr;
    logic [7:0]               fwd_rd1_addr;
    logic [7:0]               fwd_dst0_addr;
    logic [7:0]               fwd_dst1_addr;
    logic [7:0]               inv_rd0_addr;
    logic [7:0]               inv_rd1_addr;
    logic [7:0]               inv_dst0_addr;
    logic [7:0]               inv_dst1_addr;
    logic [8:0]               block_idx_calc;
    logic [8:0]               block_idx_calc_inv;
    logic [8:0]               len_calc_fwd;
    logic [8:0]               len_calc_inv;
    logic [8:0]               num_blocks_inv;
    logic [7:0]               zeta_idx_fwd;
    logic [7:0]               zeta_idx_inv;
    logic [MLDSA_COEFF_W-1:0] zeta_lookup;

    integer pipe_i;

    mldsa_bfu_pipe u_bfu (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (bfu_in_valid),
        .inverse   (bfu_inverse),
        .a_in      (bfu_a_in),
        .b_in      (bfu_b_in),
        .zeta      (bfu_zeta),
        .in_ready  (bfu_in_ready),
        .out_valid (bfu_out_valid),
        .a_out     (bfu_a_out),
        .b_out     (bfu_b_out)
    );

    mldsa_ntt_zeta_rom u_zeta_rom (
        .clk (clk),
        .addr(op_inv_q ? zeta_idx_inv : zeta_idx_fwd),
        .data(zeta_lookup)
    );

    mldsa_ntt_poly_mem_compat u_poly_mem (
        .clk         (clk),
        .load_we     (load_we && (st == S_IDLE)),
        .load_addr   (load_addr),
        .load_data   (load_data),
        .src_bank_sel(active_bank_q),
        .src_rd_req  (mem_src_rd_req),
        .src_rd_dual (mem_src_rd_dual),
        .src_rd0_addr(mem_src_rd0_addr),
        .src_rd1_addr(mem_src_rd1_addr),
        .src_rd_valid(mem_src_rd_valid),
        .src_rd0_data(mem_src_rd0_data),
        .src_rd1_data(mem_src_rd1_data),
        .dst_bank_sel(~active_bank_q),
        .dst_wr0_en  (mem_dst_wr0_en),
        .dst_wr0_addr(mem_dst_wr0_addr),
        .dst_wr0_data(mem_dst_wr0_data),
        .dst_wr1_en  (mem_dst_wr1_en),
        .dst_wr1_addr(mem_dst_wr1_addr),
        .dst_wr1_data(mem_dst_wr1_data),
        .host_bank_sel(active_bank_q),
        .host_rd_addr(rd_addr),
        .host_rd_data(rd_data)
    );

    always_comb begin
        len_calc_fwd = 9'd128 >> stage_idx_q;
        len_calc_inv = 9'd1 << stage_idx_q;
        block_idx_calc = pair_issue_idx_q / len_calc_fwd;
        block_idx_calc_inv = pair_issue_idx_q / len_calc_inv;
        num_blocks_inv = 9'd128 >> stage_idx_q;

        fwd_rd0_addr  = {pair_issue_idx_q[6:0], 1'b0};
        fwd_rd1_addr  = {pair_issue_idx_q[6:0], 1'b1};
        fwd_dst0_addr = pair_issue_idx_q;
        fwd_dst1_addr = pair_issue_idx_q + 8'd128;
        zeta_idx_fwd  = (8'd1 << stage_idx_q) + block_idx_calc[7:0];

        inv_rd0_addr  = pair_issue_idx_q;
        inv_rd1_addr  = pair_issue_idx_q + 8'd128;
        inv_dst0_addr = {pair_issue_idx_q[6:0], 1'b0};
        inv_dst1_addr = {pair_issue_idx_q[6:0], 1'b1};
        zeta_idx_inv  = ((num_blocks_inv[7:0] << 1) - 8'd1) - block_idx_calc_inv[7:0];

        if (!op_inv_q) begin
            len         = len_calc_fwd;
            block_start = block_idx_calc * len_calc_fwd;
            j_idx       = block_start + (pair_issue_idx_q % len_calc_fwd);
            k_idx       = zeta_idx_fwd;
        end else begin
            len         = len_calc_inv;
            block_start = block_idx_calc_inv * len_calc_inv;
            j_idx       = block_start + (pair_issue_idx_q % len_calc_inv);
            k_idx       = zeta_idx_inv;
        end
    end

    always_comb begin
        src_data_valid = hold_valid_q || mem_src_rd_valid;
        src_scale      = hold_valid_q ? hold_scale_q : req_scale_q;
        src_inv        = hold_valid_q ? hold_inv_q   : req_inv_q;
        src_dst0       = hold_valid_q ? hold_dst0_q  : req_dst0_q;
        src_dst1       = hold_valid_q ? hold_dst1_q  : req_dst1_q;
        src_a          = hold_valid_q ? hold_a_q     : mem_src_rd0_data;
        src_b          = hold_valid_q ? hold_b_q     : mem_src_rd1_data;
        src_zeta       = hold_valid_q ? hold_zeta_q  : req_zeta_q;

        bfu_in_valid = src_data_valid;
        bfu_inverse  = src_inv;
        bfu_a_in     = src_a;
        bfu_b_in     = src_b;
        bfu_zeta     = src_zeta;
        issue_fire   = bfu_in_valid && bfu_in_ready;

        mem_src_rd_req   = 1'b0;
        mem_src_rd_dual  = 1'b0;
        mem_src_rd0_addr = 8'd0;
        mem_src_rd1_addr = 8'd0;

        if ((st == S_STAGE_ISSUE) && (pair_issue_idx_q < 8'd128) && !hold_valid_q && !(mem_src_rd_valid && !bfu_in_ready)) begin
            mem_src_rd_req   = 1'b1;
            mem_src_rd_dual  = 1'b1;
            mem_src_rd0_addr = op_inv_q ? inv_rd0_addr : fwd_rd0_addr;
            mem_src_rd1_addr = op_inv_q ? inv_rd1_addr : fwd_rd1_addr;
        end else if ((st == S_SCALE_ISSUE) && (scale_issue_idx_q < 9'd256) &&
                     !hold_valid_q && !(mem_src_rd_valid && !bfu_in_ready)) begin
            mem_src_rd_req   = 1'b1;
            mem_src_rd_dual  = 1'b0;
            mem_src_rd0_addr = scale_issue_idx_q[7:0];
            mem_src_rd1_addr = 8'd0;
        end

        mem_dst_wr0_en   = bfu_out_valid;
        mem_dst_wr0_addr = wr_addr0_pipe_q[BFU_LAT-1];
        mem_dst_wr0_data = wr_scale_pipe_q[BFU_LAT-1] ? bfu_b_out : bfu_a_out;
        mem_dst_wr1_en   = bfu_out_valid && !wr_scale_pipe_q[BFU_LAT-1];
        mem_dst_wr1_addr = wr_addr1_pipe_q[BFU_LAT-1];
        mem_dst_wr1_data = bfu_b_out;

        busy      = (st != S_IDLE) && (st != S_DONE);
        state_dbg = st;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            st               <= S_IDLE;
            op_inv_q         <= 1'b0;
            active_bank_q    <= 1'b0;
            stage_idx_q      <= 3'd0;
            pair_issue_idx_q <= 8'd0;
            scale_issue_idx_q<= 9'd0;
            pending_q        <= 10'd0;
            req_valid_q      <= 1'b0;
            req_scale_q      <= 1'b0;
            req_inv_q        <= 1'b0;
            req_dst0_q       <= 8'd0;
            req_dst1_q       <= 8'd0;
            req_zeta_q       <= '0;
            hold_valid_q     <= 1'b0;
            hold_scale_q     <= 1'b0;
            hold_inv_q       <= 1'b0;
            hold_dst0_q      <= 8'd0;
            hold_dst1_q      <= 8'd0;
            hold_a_q         <= '0;
            hold_b_q         <= '0;
            hold_zeta_q      <= '0;
            for (pipe_i = 0; pipe_i < BFU_LAT; pipe_i = pipe_i + 1) begin
                wr_valid_pipe_q[pipe_i] <= 1'b0;
                wr_scale_pipe_q[pipe_i] <= 1'b0;
                wr_addr0_pipe_q[pipe_i] <= 8'd0;
                wr_addr1_pipe_q[pipe_i] <= 8'd0;
            end
            done <= 1'b0;
        end else begin
            done <= 1'b0;

            req_valid_q <= mem_src_rd_req;
            if (mem_src_rd_req) begin
                req_scale_q <= (st == S_SCALE_ISSUE);
                req_inv_q   <= (st == S_SCALE_ISSUE) ? 1'b1 : op_inv_q;
                req_dst0_q  <= (st == S_SCALE_ISSUE) ? scale_issue_idx_q[7:0] :
                               (op_inv_q ? inv_dst0_addr : fwd_dst0_addr);
                req_dst1_q  <= (st == S_SCALE_ISSUE) ? scale_issue_idx_q[7:0] :
                               (op_inv_q ? inv_dst1_addr : fwd_dst1_addr);
                req_zeta_q  <= (st == S_SCALE_ISSUE) ? N_INV :
                               (op_inv_q ? modq_neg(zeta_lookup) : zeta_lookup);
            end

            if (hold_valid_q && issue_fire) begin
                hold_valid_q <= 1'b0;
            end
            if (mem_src_rd_valid && !issue_fire && !hold_valid_q) begin
                hold_valid_q <= 1'b1;
                hold_scale_q <= req_scale_q;
                hold_inv_q   <= req_inv_q;
                hold_dst0_q  <= req_dst0_q;
                hold_dst1_q  <= req_dst1_q;
                hold_a_q     <= mem_src_rd0_data;
                hold_b_q     <= mem_src_rd1_data;
                hold_zeta_q  <= req_zeta_q;
            end

            for (pipe_i = BFU_LAT-1; pipe_i > 0; pipe_i = pipe_i - 1) begin
                wr_valid_pipe_q[pipe_i] <= wr_valid_pipe_q[pipe_i-1];
                wr_scale_pipe_q[pipe_i] <= wr_scale_pipe_q[pipe_i-1];
                wr_addr0_pipe_q[pipe_i] <= wr_addr0_pipe_q[pipe_i-1];
                wr_addr1_pipe_q[pipe_i] <= wr_addr1_pipe_q[pipe_i-1];
            end
            wr_valid_pipe_q[0] <= issue_fire;
            wr_scale_pipe_q[0] <= src_scale;
            wr_addr0_pipe_q[0] <= src_dst0;
            wr_addr1_pipe_q[0] <= src_dst1;

            pending_q <= pending_q
                       + (issue_fire ? 10'd1 : 10'd0)
                       - (bfu_out_valid ? 10'd1 : 10'd0);

            unique case (st)
                S_IDLE: begin
                    pending_q        <= 10'd0;
                    req_valid_q      <= 1'b0;
                    hold_valid_q     <= 1'b0;
                    stage_idx_q      <= 3'd0;
                    pair_issue_idx_q <= 8'd0;
                    scale_issue_idx_q<= 9'd0;
                    if (start) begin
                        op_inv_q      <= inverse;
                        active_bank_q <= 1'b0;
                        st            <= S_SETUP;
                    end
                end

                S_SETUP: begin
                    pending_q        <= 10'd0;
                    req_valid_q      <= 1'b0;
                    hold_valid_q     <= 1'b0;
                    stage_idx_q      <= 3'd0;
                    pair_issue_idx_q <= 8'd0;
                    scale_issue_idx_q<= 9'd0;
                    st               <= S_STAGE_ISSUE;
                end

                S_STAGE_ISSUE: begin
                    if (mem_src_rd_req) begin
                        pair_issue_idx_q <= pair_issue_idx_q + 8'd1;
                    end
                    if (pair_issue_idx_q == 8'd128 && !mem_src_rd_req) begin
                        st <= S_STAGE_DRAIN;
                    end
                end

                S_STAGE_DRAIN: begin
                    if (!src_data_valid && (pending_q == (bfu_out_valid ? 10'd1 : 10'd0))) begin
                        st <= S_STAGE_SWAP;
                    end
                end

                S_STAGE_SWAP: begin
                    active_bank_q    <= ~active_bank_q;
                    pair_issue_idx_q <= 8'd0;
                    pending_q        <= 10'd0;
                    req_valid_q      <= 1'b0;
                    hold_valid_q     <= 1'b0;
                    if (stage_idx_q == (NTT_STAGES-1)) begin
                        if (op_inv_q) begin
`ifdef MLDSA_NTT_FOLD_INTT_HALF
                            st <= S_DONE;
`else
                            scale_issue_idx_q <= 9'd0;
                            st <= S_SCALE_ISSUE;
`endif
                        end else begin
                            st <= S_DONE;
                        end
                    end else begin
                        stage_idx_q <= stage_idx_q + 3'd1;
                        st <= S_STAGE_ISSUE;
                    end
                end

                S_SCALE_ISSUE: begin
                    if (mem_src_rd_req) begin
                        if (scale_issue_idx_q == 8'd255) begin
                            scale_issue_idx_q <= 9'd256;
                        end else begin
                            scale_issue_idx_q <= scale_issue_idx_q + 9'd1;
                        end
                    end
                    if (scale_issue_idx_q == 9'd256) begin
                        st <= S_SCALE_DRAIN;
                    end
                end

                S_SCALE_DRAIN: begin
                    if (!src_data_valid && (pending_q == (bfu_out_valid ? 10'd1 : 10'd0))) begin
                        st <= S_SCALE_SWAP;
                    end
                end

                S_SCALE_SWAP: begin
                    active_bank_q <= ~active_bank_q;
                    pending_q     <= 10'd0;
                    req_valid_q   <= 1'b0;
                    hold_valid_q  <= 1'b0;
                    st            <= S_DONE;
                end

                S_DONE: begin
                    done <= 1'b1;
                    st   <= S_IDLE;
                end

                default: st <= S_IDLE;
            endcase
        end
    end

`ifndef SYNTHESIS
    property p_issue_valid_ready;
        @(posedge clk) disable iff (!rst_n)
            bfu_in_valid |-> bfu_in_ready;
    endproperty
    assert property (p_issue_valid_ready);

    property p_output_matches_pipeline;
        @(posedge clk) disable iff (!rst_n)
            bfu_out_valid |-> wr_valid_pipe_q[BFU_LAT-1];
    endproperty
    assert property (p_output_matches_pipeline);
`endif
endmodule

`default_nettype wire
