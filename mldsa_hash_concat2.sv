`timescale 1ns/1ps
`default_nettype none

module mldsa_hash_concat2 #(
    parameter int A_BYTES   = 64,
    parameter int B_BYTES   = 4096,
    parameter int OUT_BYTES = 4096,
    parameter int A_ADDR_W  = (A_BYTES <= 1) ? 1 : $clog2(A_BYTES),
    parameter int B_ADDR_W  = (B_BYTES <= 1) ? 1 : $clog2(B_BYTES),
    parameter int O_ADDR_W  = (OUT_BYTES <= 1) ? 1 : $clog2(OUT_BYTES)
) (
    input  wire                clk,
    input  wire                rst_n,
    input  wire                a_we,
    input  wire [A_ADDR_W-1:0] a_wr_addr,
    input  wire [7:0]          a_wr_data,
    input  wire                b_we,
    input  wire [B_ADDR_W-1:0] b_wr_addr,
    input  wire [7:0]          b_wr_data,
    input  wire                start,
    input  wire [15:0]         a_len,
    input  wire [15:0]         b_len,
    input  wire [15:0]         out_len,
    input  wire [O_ADDR_W-1:0] out_rd_addr,
    output logic [7:0]         out_rd_data,
    output logic               busy,
    output logic               done,
    output logic               error,
    output logic               xof_start_o,
    output logic               xof_stop_o,
    output logic               xof_mode_shake256_o,
    output logic [7:0]         xof_abs_byte_data_o,
    output logic               xof_abs_byte_valid_o,
    output logic               xof_abs_byte_last_o,
    input  wire                xof_abs_byte_ready_i,
    input  wire [7:0]          xof_sq_byte_i,
    input  wire                xof_sq_valid_i,
    output logic               xof_sq_byte_ready_o,
    input  wire                xof_state_squeezing_i
);

    typedef enum logic [3:0] {
        S_IDLE,
        S_ABSORB_PRIME,
        S_ABSORB_WAIT,
        S_ABSORB_LOAD,
        S_ABSORB,
        S_WAIT_SQUEEZE,
        S_SQUEEZE,
        S_STOP,
        S_DONE,
        S_ERROR
    } state_e;

    state_e st;

    logic [7:0] a_rd_data;
    logic [7:0] b_rd_data;
    logic [15:0] abs_idx;
    logic [15:0] abs_rd_idx;
    logic [15:0] out_idx;
    logic [15:0] a_len_q;
    logic [15:0] b_len_q;
    logic [15:0] out_len_q;
    logic [15:0] total_len_q;
    logic [15:0] b_abs_idx;
    logic [7:0]  abs_data_q;
    logic [A_ADDR_W-1:0] a_rd_addr;
    logic [B_ADDR_W-1:0] b_rd_addr;
    logic [O_ADDR_W-1:0] out_wr_addr;
    logic out_wr_en;

    mldsa_byte_store_async #(.DEPTH(A_BYTES), .ADDR_W(A_ADDR_W)) u_a_store (
        .clk    (clk),
        .wr_en  (a_we && !busy),
        .wr_addr(a_wr_addr),
        .wr_data(a_wr_data),
        .rd_addr(a_rd_addr),
        .rd_data(a_rd_data)
    );

    mldsa_byte_store_async #(.DEPTH(B_BYTES), .ADDR_W(B_ADDR_W)) u_b_store (
        .clk    (clk),
        .wr_en  (b_we && !busy),
        .wr_addr(b_wr_addr),
        .wr_data(b_wr_data),
        .rd_addr(b_rd_addr),
        .rd_data(b_rd_data)
    );

    mldsa_byte_store_async #(.DEPTH(OUT_BYTES), .ADDR_W(O_ADDR_W)) u_out_store (
        .clk    (clk),
        .wr_en  (out_wr_en),
        .wr_addr(out_wr_addr),
        .wr_data(xof_sq_byte_i),
        .rd_addr(out_rd_addr),
        .rd_data(out_rd_data)
    );

    always_comb begin
        xof_start_o = (st == S_IDLE) && start && (a_len != 16'd0 || b_len != 16'd0) && (out_len != 16'd0);
        xof_stop_o  = (st == S_STOP);
        xof_mode_shake256_o = 1'b1;
        xof_abs_byte_valid_o = (st == S_ABSORB);
        xof_abs_byte_last_o  = (abs_idx == total_len_q - 16'd1);
        b_abs_idx = abs_rd_idx - a_len_q;
        a_rd_addr = abs_rd_idx[A_ADDR_W-1:0];
        b_rd_addr = b_abs_idx[B_ADDR_W-1:0];
        out_wr_addr = out_idx[O_ADDR_W-1:0];
        xof_abs_byte_data_o = abs_data_q;
        xof_sq_byte_ready_o = (st == S_SQUEEZE);
        out_wr_en = (st == S_SQUEEZE) && xof_sq_valid_i;

        busy = (st != S_IDLE) && (st != S_DONE) && (st != S_ERROR);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st          <= S_IDLE;
            abs_idx     <= 16'd0;
            abs_rd_idx  <= 16'd0;
            out_idx     <= 16'd0;
            a_len_q     <= 16'd0;
            b_len_q     <= 16'd0;
            out_len_q   <= 16'd0;
            total_len_q <= 16'd0;
            abs_data_q  <= 8'd0;
            done        <= 1'b0;
            error       <= 1'b0;
        end else begin
            done  <= 1'b0;
            error <= 1'b0;

            unique case (st)
                S_IDLE: begin
                    abs_idx    <= 16'd0;
                    abs_rd_idx <= 16'd0;
                    out_idx    <= 16'd0;
                    abs_data_q <= 8'd0;
                    if (start) begin
                        if ((a_len == 16'd0 && b_len == 16'd0) || out_len == 16'd0 ||
                            a_len > A_BYTES || b_len > B_BYTES || out_len > OUT_BYTES) begin
                            st <= S_ERROR;
                        end else begin
                            a_len_q     <= a_len;
                            b_len_q     <= b_len;
                            out_len_q   <= out_len;
                            total_len_q <= a_len + b_len;
                            st          <= S_ABSORB_PRIME;
                        end
                    end
                end
                S_ABSORB_PRIME: begin
                    abs_rd_idx <= 16'd0;
                    st         <= S_ABSORB_WAIT;
                end
                S_ABSORB_WAIT: begin
                    st <= S_ABSORB_LOAD;
                end
                S_ABSORB_LOAD: begin
                    abs_data_q <= (abs_idx < a_len_q) ? a_rd_data : b_rd_data;
                    if (abs_idx != total_len_q - 16'd1) begin
                        abs_rd_idx <= abs_idx + 16'd1;
                    end
                    st <= S_ABSORB;
                end
                S_ABSORB: begin
                    if (xof_abs_byte_valid_o && xof_abs_byte_ready_i) begin
`ifdef MLDSA_DEBUG_DISPLAY
                        if (abs_idx < 16'd96 || abs_idx >= total_len_q - 16'd24) begin
                            $display("HASHC_ABS idx=%0d data=%02x last=%0b rd=%0d a_len=%0d b_len=%0d",
                                     abs_idx, xof_abs_byte_data_o, xof_abs_byte_last_o, abs_rd_idx, a_len_q, b_len_q);
                        end
`endif
                        if (abs_idx == total_len_q - 16'd1) st <= S_WAIT_SQUEEZE;
                        else begin
                            abs_idx <= abs_idx + 16'd1;
                            st      <= S_ABSORB_LOAD;
                        end
                    end
                end
                S_WAIT_SQUEEZE: begin
                    if (xof_state_squeezing_i) st <= S_SQUEEZE;
                end
                S_SQUEEZE: begin
                    if (xof_sq_valid_i) begin
`ifdef MLDSA_DEBUG_DISPLAY
                        if (out_idx < 16'd16) begin
                            $display("HASHC_SQ idx=%0d data=%02x", out_idx, xof_sq_byte_i);
                        end
`endif
                        if (out_idx == out_len_q - 16'd1) st <= S_STOP;
                        else out_idx <= out_idx + 16'd1;
                    end
                end
                S_STOP: begin
                    st <= S_DONE;
                end
                S_DONE: begin
                    done <= 1'b1;
                    st   <= S_IDLE;
                end
                S_ERROR: begin
                    error <= 1'b1;
                    st    <= S_IDLE;
                end
                default: st <= S_ERROR;
            endcase
        end
    end
endmodule

`default_nettype wire
