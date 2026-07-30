`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

module mldsa_mem_arb #(
    parameter int BANKS = 16,
    parameter int MASTERS = 4,
    parameter int READ_LAT = 1,
    parameter int BANK_W = (BANKS <= 1) ? 1 : $clog2(BANKS),
    parameter int MASTER_W = (MASTERS <= 1) ? 1 : $clog2(MASTERS)
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire [MASTERS-1:0]           req_valid,
    output logic [MASTERS-1:0]          req_ready,
    input  wire [MASTERS-1:0]           req_write,
    input  wire [MASTERS*BANK_W-1:0]    req_bank_flat,
    input  wire [MASTERS*8-1:0]         req_addr_flat,
    input  wire [MASTERS*MLDSA_COEFF_W-1:0] req_wdata_flat,
    output logic                        mem_valid,
    input  wire                         mem_ready,
    output logic                        mem_write,
    output logic [BANK_W-1:0]           mem_bank,
    output logic [7:0]                  mem_addr,
    output logic [MLDSA_COEFF_W-1:0]    mem_wdata,
    input  wire                         mem_rvalid,
    input  wire [MLDSA_COEFF_W-1:0]     mem_rdata,
    output logic [MASTERS-1:0]          resp_valid,
    output logic [MASTERS*MLDSA_COEFF_W-1:0] resp_rdata_flat,
    output logic [MASTER_W-1:0]         grant_id
);

    logic [MASTER_W-1:0] rr_ptr_q;
    logic [MASTER_W-1:0] grant_comb;
    logic                grant_found;
    logic                accept_fire;
    logic                accept_read;

    integer i;
    integer offset;
    integer cand;

    always_comb begin
        grant_found = 1'b0;
        grant_comb  = '0;
        for (offset = 0; offset < MASTERS; offset = offset + 1) begin
            cand = rr_ptr_q + offset;
            if (cand >= MASTERS) cand = cand - MASTERS;
            if (!grant_found && req_valid[cand]) begin
                grant_found = 1'b1;
                grant_comb  = cand[MASTER_W-1:0];
            end
        end
    end

    always_comb begin
        req_ready = '0;
        mem_valid = grant_found;
        mem_write = 1'b0;
        mem_bank  = '0;
        mem_addr  = 8'd0;
        mem_wdata = '0;
        grant_id  = grant_comb;

        if (grant_found) begin
            mem_write = req_write[grant_comb];
            mem_bank  = req_bank_flat[grant_comb*BANK_W +: BANK_W];
            mem_addr  = req_addr_flat[grant_comb*8 +: 8];
            mem_wdata = req_wdata_flat[grant_comb*MLDSA_COEFF_W +: MLDSA_COEFF_W];
            if (mem_ready) req_ready[grant_comb] = 1'b1;
        end
    end

    assign accept_fire = grant_found && mem_ready;
    assign accept_read = accept_fire && !req_write[grant_comb];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rr_ptr_q <= '0;
        end else if (accept_fire) begin
            if (grant_comb == MASTERS-1) rr_ptr_q <= '0;
            else rr_ptr_q <= grant_comb + {{(MASTER_W-1){1'b0}}, 1'b1};
        end
    end

    generate
        if (READ_LAT == 0) begin : g_read_lat0
            always_comb begin
                resp_valid = '0;
                resp_rdata_flat = '0;
                if (mem_rvalid && accept_read) begin
                    resp_valid[grant_comb] = 1'b1;
                    resp_rdata_flat[grant_comb*MLDSA_COEFF_W +: MLDSA_COEFF_W] = mem_rdata;
                end
            end
        end else begin : g_read_latn
            logic [READ_LAT-1:0] read_valid_pipe;
            logic [MASTER_W-1:0] read_id_pipe [0:READ_LAT-1];

            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    read_valid_pipe <= '0;
                    for (i = 0; i < READ_LAT; i = i + 1) begin
                        read_id_pipe[i] <= '0;
                    end
                end else begin
                    read_valid_pipe[0] <= accept_read;
                    read_id_pipe[0] <= grant_comb;
                    for (i = 1; i < READ_LAT; i = i + 1) begin
                        read_valid_pipe[i] <= read_valid_pipe[i-1];
                        read_id_pipe[i] <= read_id_pipe[i-1];
                    end
                end
            end

            always_comb begin
                resp_valid = '0;
                resp_rdata_flat = '0;
                if (mem_rvalid && read_valid_pipe[READ_LAT-1]) begin
                    resp_valid[read_id_pipe[READ_LAT-1]] = 1'b1;
                    resp_rdata_flat[read_id_pipe[READ_LAT-1]*MLDSA_COEFF_W +: MLDSA_COEFF_W] = mem_rdata;
                end
            end
        end
    endgenerate

`ifndef SYNTHESIS
    always_ff @(posedge clk) begin
        if (rst_n) begin
            if (grant_found) assert (req_valid[grant_comb]);
            assert ($onehot0(req_ready));
            assert ($onehot0(resp_valid));
        end
    end
`endif
endmodule

`default_nettype wire
