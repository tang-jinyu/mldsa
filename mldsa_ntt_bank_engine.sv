`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

module mldsa_ntt_bank_engine #(
    parameter int BANKS = 65
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         start,
    input  wire                         inverse,
    input  wire [$clog2(BANKS)-1:0]     src_bank,
    input  wire [$clog2(BANKS)-1:0]     dst_bank,
    output logic                        rd_en,
    output logic [$clog2(BANKS)-1:0]    rd_bank,
    output logic [7:0]                  rd_addr,
    input  wire [MLDSA_COEFF_W-1:0]     rd_data,
    output logic                        wr_en,
    output logic [$clog2(BANKS)-1:0]    wr_bank,
    output logic [7:0]                  wr_addr,
    output logic [MLDSA_COEFF_W-1:0]    wr_data,
    output logic                        busy,
    output logic                        done,
    output logic [4:0]                  state_dbg
);

    typedef enum logic [4:0] {
        S_IDLE,
        S_LOAD_REQ,
        S_LOAD_PUSH,
        S_CORE_START,
        S_CORE_WAIT,
        S_STORE_WRITE,
        S_DONE
    } state_e;

    state_e st;
    logic inv_q;
    logic [$clog2(BANKS)-1:0] src_bank_q;
    logic [$clog2(BANKS)-1:0] dst_bank_q;
    logic [7:0] coeff_idx;
    logic ntt_start;
    logic ntt_load_we;
    logic [7:0] ntt_load_addr;
    logic [MLDSA_COEFF_W-1:0] ntt_load_data;
    logic [7:0] ntt_rd_addr;
    logic [MLDSA_COEFF_W-1:0] ntt_rd_data;
    logic ntt_busy;
    logic ntt_done;

    mldsa_ntt_core u_ntt (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (ntt_start),
        .inverse   (inv_q),
        .load_we   (ntt_load_we),
        .load_addr (ntt_load_addr),
        .load_data (ntt_load_data),
        .rd_addr   (ntt_rd_addr),
        .rd_data   (ntt_rd_data),
        .busy      (ntt_busy),
        .done      (ntt_done),
        .state_dbg ()
    );

    always_comb begin
        rd_en         = 1'b0;
        rd_bank       = src_bank_q;
        rd_addr       = coeff_idx;
        wr_en         = 1'b0;
        wr_bank       = dst_bank_q;
        wr_addr       = coeff_idx;
        wr_data       = ntt_rd_data;
        ntt_start     = 1'b0;
        ntt_load_we   = 1'b0;
        ntt_load_addr = coeff_idx;
        ntt_load_data = rd_data;
        ntt_rd_addr   = coeff_idx;
        busy          = (st != S_IDLE) && (st != S_DONE);
        state_dbg     = st;

        unique case (st)
            S_LOAD_REQ: begin
                rd_en   = 1'b1;
                rd_bank = src_bank_q;
                rd_addr = coeff_idx;
            end
            S_LOAD_PUSH: begin
                ntt_load_we   = 1'b1;
                ntt_load_addr = coeff_idx;
                ntt_load_data = rd_data;
            end
            S_CORE_START: begin
                ntt_start = 1'b1;
            end
            S_STORE_WRITE: begin
                wr_en   = 1'b1;
                wr_bank = dst_bank_q;
                wr_addr = coeff_idx;
                wr_data = ntt_rd_data;
            end
            default: ;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            st         <= S_IDLE;
            inv_q      <= 1'b0;
            src_bank_q <= '0;
            dst_bank_q <= '0;
            coeff_idx  <= 8'd0;
            done       <= 1'b0;
        end else begin
            done <= 1'b0;
            unique case (st)
                S_IDLE: begin
                    if (start) begin
                        inv_q      <= inverse;
                        src_bank_q <= src_bank;
                        dst_bank_q <= dst_bank;
                        coeff_idx  <= 8'd0;
                        st         <= S_LOAD_REQ;
                    end
                end
                S_LOAD_REQ: begin
                    st <= S_LOAD_PUSH;
                end
                S_LOAD_PUSH: begin
                    if (coeff_idx == 8'd255) begin
                        coeff_idx <= 8'd0;
                        st <= S_CORE_START;
                    end else begin
                        coeff_idx <= coeff_idx + 8'd1;
                        st <= S_LOAD_REQ;
                    end
                end
                S_CORE_START: begin
                    st <= S_CORE_WAIT;
                end
                S_CORE_WAIT: begin
                    if (ntt_done) begin
                        coeff_idx <= 8'd0;
                        st <= S_STORE_WRITE;
                    end
                end
                S_STORE_WRITE: begin
                    if (coeff_idx == 8'd255) begin
                        st <= S_DONE;
                    end else begin
                        coeff_idx <= coeff_idx + 8'd1;
                        st <= S_STORE_WRITE;
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
