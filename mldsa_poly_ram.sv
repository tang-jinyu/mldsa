`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

`ifdef SYNTHESIS
module mldsa_poly_bank_asap7_sram (
    input  wire                      clk,
    input  wire                      rd0_en,
    input  wire [7:0]                rd0_addr,
    output logic [MLDSA_COEFF_W-1:0] rd0_data,
    input  wire                      pair_rd0_en,
    input  wire [6:0]                pair_rd0_addr,
    output logic [47:0]              pair_rd0_data,
    input  wire                      rd1_en,
    input  wire [7:0]                rd1_addr,
    output logic [MLDSA_COEFF_W-1:0] rd1_data,
    input  wire                      pair_rd1_en,
    input  wire [6:0]                pair_rd1_addr,
    output logic [47:0]              pair_rd1_data,
    input  wire                      wr_en,
    input  wire [7:0]                wr_addr,
    input  wire [MLDSA_COEFF_W-1:0]  wr_data,
    input  wire                      pair_wr_en,
    input  wire [6:0]                pair_wr_addr,
    input  wire [47:0]               pair_wr_data
);
    logic        rd0_half_q;
    logic        rd1_half_q;
    logic        rd0_pair_q;
    logic        rd1_pair_q;
    logic [8:0]  macro_rd0_addr;
    logic [8:0]  macro_rd1_addr;
    logic [8:0]  macro_wr_addr;
    logic [47:0] wr_word_data;
    logic [47:0] rd0_word;
    logic [47:0] rd1_word;

    always_comb begin
        macro_rd0_addr = pair_rd0_en ? {2'b00, pair_rd0_addr} : {1'b0, rd0_addr[7:1]};
        macro_rd1_addr = pair_rd1_en ? {2'b00, pair_rd1_addr} : {1'b0, rd1_addr[7:1]};
        macro_wr_addr  = pair_wr_en  ? {2'b00, pair_wr_addr}  : {1'b0, wr_addr[7:1]};
        wr_word_data   = 48'd0;
        if (pair_wr_en) begin
            wr_word_data = pair_wr_data;
        end else if (wr_en) begin
            if (wr_addr[0]) begin
                wr_word_data = {wr_data, 24'd0};
            end else begin
                wr_word_data = {24'd0, wr_data};
            end
        end
    end

    always_ff @(posedge clk) begin
        if (rd0_en || pair_rd0_en) begin
            rd0_half_q <= rd0_addr[0];
            rd0_pair_q <= pair_rd0_en;
        end
        if (rd1_en || pair_rd1_en) begin
            rd1_half_q <= rd1_addr[0];
            rd1_pair_q <= pair_rd1_en;
        end
    end

    always_comb begin
        rd0_data      = rd0_half_q ? rd0_word[47:24] : rd0_word[23:0];
        rd1_data      = rd1_half_q ? rd1_word[47:24] : rd1_word[23:0];
        pair_rd0_data = rd0_pair_q ? rd0_word : 48'd0;
        pair_rd1_data = rd1_pair_q ? rd1_word : 48'd0;
    end

    srambank_128x4x48_6t122 u_sram_rd0 (
        .clk     (clk),
        .ADDRESS ((wr_en || pair_wr_en) ? macro_wr_addr : macro_rd0_addr),
        .wd      (wr_word_data),
        .banksel ((wr_en || pair_wr_en) || rd0_en || pair_rd0_en),
        .read    (!(wr_en || pair_wr_en) && (rd0_en || pair_rd0_en)),
        .write   (wr_en || pair_wr_en),
        .dataout (rd0_word)
    );

    srambank_128x4x48_6t122 u_sram_rd1 (
        .clk     (clk),
        .ADDRESS ((wr_en || pair_wr_en) ? macro_wr_addr : macro_rd1_addr),
        .wd      (wr_word_data),
        .banksel ((wr_en || pair_wr_en) || rd1_en || pair_rd1_en),
        .read    (!(wr_en || pair_wr_en) && (rd1_en || pair_rd1_en)),
        .write   (wr_en || pair_wr_en),
        .dataout (rd1_word)
    );
endmodule

module mldsa_poly_ram_group8 (
    input  wire                      clk,
    input  wire                      rd0_en,
    input  wire [2:0]                rd0_bank,
    input  wire [7:0]                rd0_addr,
    output logic [MLDSA_COEFF_W-1:0] rd0_data,
    input  wire                      pair_rd0_en,
    input  wire [2:0]                pair_rd0_bank,
    input  wire [6:0]                pair_rd0_addr,
    output logic [47:0]              pair_rd0_data,
    input  wire                      rd1_en,
    input  wire [2:0]                rd1_bank,
    input  wire [7:0]                rd1_addr,
    output logic [MLDSA_COEFF_W-1:0] rd1_data,
    input  wire                      pair_rd1_en,
    input  wire [2:0]                pair_rd1_bank,
    input  wire [6:0]                pair_rd1_addr,
    output logic [47:0]              pair_rd1_data,
    input  wire                      wr_en,
    input  wire [2:0]                wr_bank,
    input  wire [7:0]                wr_addr,
    input  wire [MLDSA_COEFF_W-1:0]  wr_data,
    input  wire                      pair_wr_en,
    input  wire [2:0]                pair_wr_bank,
    input  wire [6:0]                pair_wr_addr,
    input  wire [47:0]               pair_wr_data
);
    logic [2:0] rd0_bank_q;
    logic [2:0] rd1_bank_q;
    logic [2:0] pair_rd0_bank_q;
    logic [2:0] pair_rd1_bank_q;
    logic [MLDSA_COEFF_W-1:0] bank_rd0_data [0:7];
    logic [MLDSA_COEFF_W-1:0] bank_rd1_data [0:7];
    logic [47:0] bank_pair_rd0_data [0:7];
    logic [47:0] bank_pair_rd1_data [0:7];

    genvar bank_idx;
    generate
        for (bank_idx = 0; bank_idx < 8; bank_idx = bank_idx + 1) begin : g_bank
            localparam logic [2:0] BANK_ID = bank_idx[2:0];
            wire bank_rd0_en = rd0_en && (rd0_bank == BANK_ID);
            wire bank_rd1_en = rd1_en && (rd1_bank == BANK_ID);
            wire bank_pair_rd0_en = pair_rd0_en && (pair_rd0_bank == BANK_ID);
            wire bank_pair_rd1_en = pair_rd1_en && (pair_rd1_bank == BANK_ID);
            wire bank_wr_en = wr_en && (wr_bank == BANK_ID);
            wire bank_pair_wr_en = pair_wr_en && (pair_wr_bank == BANK_ID);

            mldsa_poly_bank_asap7_sram u_bank (
                .clk          (clk),
                .rd0_en       (bank_rd0_en),
                .rd0_addr     (rd0_addr),
                .rd0_data     (bank_rd0_data[bank_idx]),
                .pair_rd0_en  (bank_pair_rd0_en),
                .pair_rd0_addr(pair_rd0_addr),
                .pair_rd0_data(bank_pair_rd0_data[bank_idx]),
                .rd1_en       (bank_rd1_en),
                .rd1_addr     (rd1_addr),
                .rd1_data     (bank_rd1_data[bank_idx]),
                .pair_rd1_en  (bank_pair_rd1_en),
                .pair_rd1_addr(pair_rd1_addr),
                .pair_rd1_data(bank_pair_rd1_data[bank_idx]),
                .wr_en        (bank_wr_en),
                .wr_addr      (wr_addr),
                .wr_data      (wr_data),
                .pair_wr_en   (bank_pair_wr_en),
                .pair_wr_addr (pair_wr_addr),
                .pair_wr_data (pair_wr_data)
            );
        end
    endgenerate

    always_ff @(posedge clk) begin
        if (rd0_en) rd0_bank_q <= rd0_bank;
        if (rd1_en) rd1_bank_q <= rd1_bank;
        if (pair_rd0_en) pair_rd0_bank_q <= pair_rd0_bank;
        if (pair_rd1_en) pair_rd1_bank_q <= pair_rd1_bank;
    end

    always_comb begin
        rd0_data      = bank_rd0_data[rd0_bank_q];
        rd1_data      = bank_rd1_data[rd1_bank_q];
        pair_rd0_data = bank_pair_rd0_data[pair_rd0_bank_q];
        pair_rd1_data = bank_pair_rd1_data[pair_rd1_bank_q];
    end
endmodule

module mldsa_poly_ram_pool4_backend #(
    parameter int BANKS = 65,
    parameter int BANK_W = (BANKS <= 1) ? 1 : $clog2(BANKS),
    parameter int POOLS = (BANKS + 3) / 4,
    parameter int POOL_W = (POOLS <= 1) ? 1 : $clog2(POOLS)
) (
    input  wire                         clk,
    input  wire                         rd0_en,
    input  wire [BANK_W-1:0]            rd0_bank,
    input  wire [7:0]                   rd0_addr,
    output logic [MLDSA_COEFF_W-1:0]    rd0_data,
    input  wire                         pair_rd0_en,
    input  wire [BANK_W-1:0]            pair_rd0_bank,
    input  wire [6:0]                   pair_rd0_addr,
    output logic [47:0]                 pair_rd0_data,
    input  wire                         rd1_en,
    input  wire [BANK_W-1:0]            rd1_bank,
    input  wire [7:0]                   rd1_addr,
    output logic [MLDSA_COEFF_W-1:0]    rd1_data,
    input  wire                         pair_rd1_en,
    input  wire [BANK_W-1:0]            pair_rd1_bank,
    input  wire [6:0]                   pair_rd1_addr,
    output logic [47:0]                 pair_rd1_data,
    input  wire                         wr_en,
    input  wire [BANK_W-1:0]            wr_bank,
    input  wire [7:0]                   wr_addr,
    input  wire [MLDSA_COEFF_W-1:0]     wr_data,
    input  wire                         pair_wr_en,
    input  wire [BANK_W-1:0]            pair_wr_bank,
    input  wire [6:0]                   pair_wr_addr,
    input  wire [47:0]                  pair_wr_data
);
    function automatic logic [POOL_W-1:0] pool_sel(input logic [BANK_W-1:0] bank);
        begin
            if (BANK_W <= 2) pool_sel = '0;
            else pool_sel = bank[BANK_W-1:2];
        end
    endfunction

    function automatic logic [1:0] lane_sel(input logic [BANK_W-1:0] bank);
        begin
            lane_sel = bank[1:0];
        end
    endfunction

    logic [POOL_W-1:0] rd0_pool_q;
    logic [POOL_W-1:0] rd1_pool_q;
    logic [POOL_W-1:0] pair_rd0_pool_q;
    logic [POOL_W-1:0] pair_rd1_pool_q;
    logic              rd0_half_q;
    logic              rd1_half_q;
    logic              rd0_pair_q;
    logic              rd1_pair_q;
    logic [47:0]       pool_rd0_word [0:POOLS-1];
    logic [47:0]       pool_rd1_word [0:POOLS-1];

`ifdef MLDSA_TARGET_ASIC
    genvar pool_idx;
    generate
        for (pool_idx = 0; pool_idx < POOLS; pool_idx = pool_idx + 1) begin : g_pool
            logic [8:0] rd0_macro_addr;
            logic [8:0] rd1_macro_addr;
            logic [8:0] wr_macro_addr;
            logic [47:0] wr_word_data;
            logic pool_rd0_en;
            logic pool_rd1_en;
            logic pool_pair_rd0_en;
            logic pool_pair_rd1_en;
            logic pool_wr_en;
            logic pool_pair_wr_en;

            always_comb begin
                pool_rd0_en      = rd0_en && (pool_sel(rd0_bank) == POOL_W'(pool_idx));
                pool_rd1_en      = rd1_en && (pool_sel(rd1_bank) == POOL_W'(pool_idx));
                pool_pair_rd0_en = pair_rd0_en && (pool_sel(pair_rd0_bank) == POOL_W'(pool_idx));
                pool_pair_rd1_en = pair_rd1_en && (pool_sel(pair_rd1_bank) == POOL_W'(pool_idx));
                pool_wr_en       = wr_en && (pool_sel(wr_bank) == POOL_W'(pool_idx));
                pool_pair_wr_en  = pair_wr_en && (pool_sel(pair_wr_bank) == POOL_W'(pool_idx));

                rd0_macro_addr = pool_pair_rd0_en ? {lane_sel(pair_rd0_bank), pair_rd0_addr}
                                                   : {lane_sel(rd0_bank), rd0_addr[7:1]};
                rd1_macro_addr = pool_pair_rd1_en ? {lane_sel(pair_rd1_bank), pair_rd1_addr}
                                                   : {lane_sel(rd1_bank), rd1_addr[7:1]};
                wr_macro_addr  = pool_pair_wr_en  ? {lane_sel(pair_wr_bank), pair_wr_addr}
                                                   : {lane_sel(wr_bank), wr_addr[7:1]};

                wr_word_data = 48'd0;
                if (pool_pair_wr_en) begin
                    wr_word_data = pair_wr_data;
                end else if (pool_wr_en) begin
                    if (wr_addr[0]) wr_word_data = {wr_data, 24'd0};
                    else wr_word_data = {24'd0, wr_data};
                end
            end

            srambank_128x4x48_6t122 u_pool_rd0 (
                .clk     (clk),
                .ADDRESS ((pool_wr_en || pool_pair_wr_en) ? wr_macro_addr : rd0_macro_addr),
                .wd      (wr_word_data),
                .banksel ((pool_wr_en || pool_pair_wr_en) || pool_rd0_en || pool_pair_rd0_en),
                .read    (!(pool_wr_en || pool_pair_wr_en) && (pool_rd0_en || pool_pair_rd0_en)),
                .write   (pool_wr_en || pool_pair_wr_en),
                .dataout (pool_rd0_word[pool_idx])
            );

            srambank_128x4x48_6t122 u_pool_rd1 (
                .clk     (clk),
                .ADDRESS ((pool_wr_en || pool_pair_wr_en) ? wr_macro_addr : rd1_macro_addr),
                .wd      (wr_word_data),
                .banksel ((pool_wr_en || pool_pair_wr_en) || pool_rd1_en || pool_pair_rd1_en),
                .read    (!(pool_wr_en || pool_pair_wr_en) && (pool_rd1_en || pool_pair_rd1_en)),
                .write   (pool_wr_en || pool_pair_wr_en),
                .dataout (pool_rd1_word[pool_idx])
            );
        end
    endgenerate
`else
    localparam int FPGA_POOL_DEPTH = POOLS * 512;
    localparam int FPGA_POOL_ADDR_W = POOL_W + 9;

    (* ram_style = "block" *) logic [47:0] pool_rd0_mem [0:FPGA_POOL_DEPTH-1];
    (* ram_style = "block" *) logic [47:0] pool_rd1_mem [0:FPGA_POOL_DEPTH-1];

    function automatic logic [8:0] macro_addr_single(
        input logic [BANK_W-1:0] bank,
        input logic [7:0] addr
    );
        begin
            macro_addr_single = {lane_sel(bank), addr[7:1]};
        end
    endfunction

    function automatic logic [8:0] macro_addr_pair(
        input logic [BANK_W-1:0] bank,
        input logic [6:0] addr
    );
        begin
            macro_addr_pair = {lane_sel(bank), addr};
        end
    endfunction

    logic [POOL_W-1:0] wr_pool_sel;
    logic [POOL_W-1:0] rd0_pool_sel;
    logic [POOL_W-1:0] rd1_pool_sel;
    logic [8:0]        wr_macro_addr;
    logic [8:0]        rd0_macro_addr;
    logic [8:0]        rd1_macro_addr;
    logic [FPGA_POOL_ADDR_W-1:0] wr_mem_addr;
    logic [FPGA_POOL_ADDR_W-1:0] rd0_mem_addr;
    logic [FPGA_POOL_ADDR_W-1:0] rd1_mem_addr;
    logic [47:0]       wr_word_data;
    logic              write_active;

    always_comb begin
        write_active  = wr_en || pair_wr_en;
        wr_pool_sel   = pair_wr_en ? pool_sel(pair_wr_bank) : pool_sel(wr_bank);
        rd0_pool_sel  = pair_rd0_en ? pool_sel(pair_rd0_bank) : pool_sel(rd0_bank);
        rd1_pool_sel  = pair_rd1_en ? pool_sel(pair_rd1_bank) : pool_sel(rd1_bank);
        wr_macro_addr = pair_wr_en ? macro_addr_pair(pair_wr_bank, pair_wr_addr)
                                   : macro_addr_single(wr_bank, wr_addr);
        rd0_macro_addr = pair_rd0_en ? macro_addr_pair(pair_rd0_bank, pair_rd0_addr)
                                     : macro_addr_single(rd0_bank, rd0_addr);
        rd1_macro_addr = pair_rd1_en ? macro_addr_pair(pair_rd1_bank, pair_rd1_addr)
                                     : macro_addr_single(rd1_bank, rd1_addr);
        wr_mem_addr    = {wr_pool_sel,  wr_macro_addr};
        rd0_mem_addr   = {rd0_pool_sel, rd0_macro_addr};
        rd1_mem_addr   = {rd1_pool_sel, rd1_macro_addr};
        wr_word_data = 48'd0;
        if (pair_wr_en) begin
            wr_word_data = pair_wr_data;
        end else if (wr_en) begin
            if (wr_addr[0]) wr_word_data = {wr_data, 24'd0};
            else wr_word_data = {24'd0, wr_data};
        end
    end

    always_ff @(posedge clk) begin
        if (write_active) begin
            pool_rd0_mem[wr_mem_addr] <= wr_word_data;
            pool_rd1_mem[wr_mem_addr] <= wr_word_data;
        end
        if (!write_active && (rd0_en || pair_rd0_en)) begin
            pool_rd0_word[rd0_pool_sel] <= pool_rd0_mem[rd0_mem_addr];
        end
        if (!write_active && (rd1_en || pair_rd1_en)) begin
            pool_rd1_word[rd1_pool_sel] <= pool_rd1_mem[rd1_mem_addr];
        end
    end
`endif

    always_ff @(posedge clk) begin
        if (rd0_en || pair_rd0_en) begin
            rd0_pool_q <= pair_rd0_en ? pool_sel(pair_rd0_bank) : pool_sel(rd0_bank);
            rd0_half_q <= rd0_addr[0];
            rd0_pair_q <= pair_rd0_en;
        end
        if (rd1_en || pair_rd1_en) begin
            rd1_pool_q <= pair_rd1_en ? pool_sel(pair_rd1_bank) : pool_sel(rd1_bank);
            rd1_half_q <= rd1_addr[0];
            rd1_pair_q <= pair_rd1_en;
        end
        if (pair_rd0_en) pair_rd0_pool_q <= pool_sel(pair_rd0_bank);
        if (pair_rd1_en) pair_rd1_pool_q <= pool_sel(pair_rd1_bank);
    end

    always_comb begin
        rd0_data      = rd0_half_q ? pool_rd0_word[rd0_pool_q][47:24] : pool_rd0_word[rd0_pool_q][23:0];
        rd1_data      = rd1_half_q ? pool_rd1_word[rd1_pool_q][47:24] : pool_rd1_word[rd1_pool_q][23:0];
        pair_rd0_data = rd0_pair_q ? pool_rd0_word[pair_rd0_pool_q] : 48'd0;
        pair_rd1_data = rd1_pair_q ? pool_rd1_word[pair_rd1_pool_q] : 48'd0;
    end
endmodule
`endif

module mldsa_poly_ram #(
    parameter int BANKS = 16,
    parameter int SRAM_RD_LATENCY = 1
) (
    input  wire                         clk,
    input  wire                         rd0_en,
    input  wire [$clog2(BANKS)-1:0]     rd0_bank,
    input  wire [7:0]                   rd0_addr,
    output logic [MLDSA_COEFF_W-1:0]    rd0_data,
    output logic [MLDSA_COEFF_W-1:0]    rd0_data_pipe,
    output logic                        rd0_valid,
    input  wire                         pair_rd0_en,
    input  wire [$clog2(BANKS)-1:0]     pair_rd0_bank,
    input  wire [6:0]                   pair_rd0_addr,
    output logic [47:0]                 pair_rd0_data,
    output logic [47:0]                 pair_rd0_data_pipe,
    output logic                        pair_rd0_valid,
    input  wire                         rd1_en,
    input  wire [$clog2(BANKS)-1:0]     rd1_bank,
    input  wire [7:0]                   rd1_addr,
    output logic [MLDSA_COEFF_W-1:0]    rd1_data,
    output logic [MLDSA_COEFF_W-1:0]    rd1_data_pipe,
    output logic                        rd1_valid,
    input  wire                         pair_rd1_en,
    input  wire [$clog2(BANKS)-1:0]     pair_rd1_bank,
    input  wire [6:0]                   pair_rd1_addr,
    output logic [47:0]                 pair_rd1_data,
    output logic [47:0]                 pair_rd1_data_pipe,
    output logic                        pair_rd1_valid,
    input  wire                         wr_en,
    input  wire [$clog2(BANKS)-1:0]     wr_bank,
    input  wire [7:0]                   wr_addr,
    input  wire [MLDSA_COEFF_W-1:0]     wr_data,
    input  wire                         pair_wr_en,
    input  wire [$clog2(BANKS)-1:0]     pair_wr_bank,
    input  wire [6:0]                   pair_wr_addr,
    input  wire [47:0]                  pair_wr_data
);

`ifdef SYNTHESIS
    localparam int BANK_W      = (BANKS <= 1) ? 1 : $clog2(BANKS);
`ifndef MLDSA_POLY_RAM_ASAP7_LEGACY_BANKS
    localparam int POOLS       = (BANKS + 3) / 4;
    localparam int POOL_W      = (POOLS <= 1) ? 1 : $clog2(POOLS);
    logic               rd0_req_q;
    logic               rd1_req_q;
    logic               pair_rd0_req_q;
    logic               pair_rd1_req_q;

    mldsa_poly_ram_pool4_backend #(
        .BANKS(BANKS),
        .BANK_W(BANK_W),
        .POOLS(POOLS),
        .POOL_W(POOL_W)
    ) u_pool (
        .clk          (clk),
        .rd0_en       (rd0_en),
        .rd0_bank     (rd0_bank),
        .rd0_addr     (rd0_addr),
        .rd0_data     (rd0_data),
        .pair_rd0_en  (pair_rd0_en),
        .pair_rd0_bank(pair_rd0_bank),
        .pair_rd0_addr(pair_rd0_addr),
        .pair_rd0_data(pair_rd0_data),
        .rd1_en       (rd1_en),
        .rd1_bank     (rd1_bank),
        .rd1_addr     (rd1_addr),
        .rd1_data     (rd1_data),
        .pair_rd1_en  (pair_rd1_en),
        .pair_rd1_bank(pair_rd1_bank),
        .pair_rd1_addr(pair_rd1_addr),
        .pair_rd1_data(pair_rd1_data),
        .wr_en        (wr_en),
        .wr_bank      (wr_bank),
        .wr_addr      (wr_addr),
        .wr_data      (wr_data),
        .pair_wr_en   (pair_wr_en),
        .pair_wr_bank (pair_wr_bank),
        .pair_wr_addr (pair_wr_addr),
        .pair_wr_data (pair_wr_data)
    );

    always_ff @(posedge clk) begin
        rd0_req_q <= rd0_en;
        rd1_req_q <= rd1_en;
        pair_rd0_req_q <= pair_rd0_en;
        pair_rd1_req_q <= pair_rd1_en;

        rd0_valid <= rd0_req_q;
        rd1_valid <= rd1_req_q;
        pair_rd0_valid <= pair_rd0_req_q;
        pair_rd1_valid <= pair_rd1_req_q;

        if (rd0_req_q) rd0_data_pipe <= rd0_data;
        if (rd1_req_q) rd1_data_pipe <= rd1_data;
        if (pair_rd0_req_q) pair_rd0_data_pipe <= pair_rd0_data;
        if (pair_rd1_req_q) pair_rd1_data_pipe <= pair_rd1_data;
    end
`else
    localparam int GROUPS      = (BANKS + 7) / 8;
    localparam int GROUP_W     = (GROUPS <= 1) ? 1 : $clog2(GROUPS);

    function automatic logic [GROUP_W-1:0] group_sel(input logic [BANK_W-1:0] bank);
        begin
            if (BANK_W <= 3) group_sel = '0;
            else group_sel = bank[BANK_W-1:3];
        end
    endfunction

    logic [GROUP_W-1:0] rd0_group_q;
    logic [GROUP_W-1:0] rd1_group_q;
    logic [GROUP_W-1:0] pair_rd0_group_q;
    logic [GROUP_W-1:0] pair_rd1_group_q;
    logic               rd0_req_q;
    logic               rd1_req_q;
    logic               pair_rd0_req_q;
    logic               pair_rd1_req_q;

    logic [MLDSA_COEFF_W-1:0] grp_rd0_data [0:GROUPS-1];
    logic [MLDSA_COEFF_W-1:0] grp_rd1_data [0:GROUPS-1];
    logic [47:0] grp_pair_rd0_data [0:GROUPS-1];
    logic [47:0] grp_pair_rd1_data [0:GROUPS-1];

    genvar group_idx;
    generate
        for (group_idx = 0; group_idx < GROUPS; group_idx = group_idx + 1) begin : g_group
            wire group_rd0_en = rd0_en && (group_sel(rd0_bank) == GROUP_W'(group_idx));
            wire group_rd1_en = rd1_en && (group_sel(rd1_bank) == GROUP_W'(group_idx));
            wire group_pair_rd0_en = pair_rd0_en && (group_sel(pair_rd0_bank) == GROUP_W'(group_idx));
            wire group_pair_rd1_en = pair_rd1_en && (group_sel(pair_rd1_bank) == GROUP_W'(group_idx));
            wire group_wr_en = wr_en && (group_sel(wr_bank) == GROUP_W'(group_idx));
            wire group_pair_wr_en = pair_wr_en && (group_sel(pair_wr_bank) == GROUP_W'(group_idx));

            mldsa_poly_ram_group8 u_group (
                .clk          (clk),
                .rd0_en       (group_rd0_en),
                .rd0_bank     (rd0_bank[2:0]),
                .rd0_addr     (rd0_addr),
                .rd0_data     (grp_rd0_data[group_idx]),
                .pair_rd0_en  (group_pair_rd0_en),
                .pair_rd0_bank(pair_rd0_bank[2:0]),
                .pair_rd0_addr(pair_rd0_addr),
                .pair_rd0_data(grp_pair_rd0_data[group_idx]),
                .rd1_en       (group_rd1_en),
                .rd1_bank     (rd1_bank[2:0]),
                .rd1_addr     (rd1_addr),
                .rd1_data     (grp_rd1_data[group_idx]),
                .pair_rd1_en  (group_pair_rd1_en),
                .pair_rd1_bank(pair_rd1_bank[2:0]),
                .pair_rd1_addr(pair_rd1_addr),
                .pair_rd1_data(grp_pair_rd1_data[group_idx]),
                .wr_en        (group_wr_en),
                .wr_bank      (wr_bank[2:0]),
                .wr_addr      (wr_addr),
                .wr_data      (wr_data),
                .pair_wr_en   (group_pair_wr_en),
                .pair_wr_bank (pair_wr_bank[2:0]),
                .pair_wr_addr (pair_wr_addr),
                .pair_wr_data (pair_wr_data)
            );
        end
    endgenerate

    always_ff @(posedge clk) begin
        if (rd0_en) rd0_group_q <= group_sel(rd0_bank);
        if (rd1_en) rd1_group_q <= group_sel(rd1_bank);
        if (pair_rd0_en) pair_rd0_group_q <= group_sel(pair_rd0_bank);
        if (pair_rd1_en) pair_rd1_group_q <= group_sel(pair_rd1_bank);

        rd0_req_q <= rd0_en;
        rd1_req_q <= rd1_en;
        pair_rd0_req_q <= pair_rd0_en;
        pair_rd1_req_q <= pair_rd1_en;

        rd0_valid <= rd0_req_q;
        rd1_valid <= rd1_req_q;
        pair_rd0_valid <= pair_rd0_req_q;
        pair_rd1_valid <= pair_rd1_req_q;

        if (rd0_req_q) begin
            rd0_data_pipe <= grp_rd0_data[rd0_group_q];
        end
        if (rd1_req_q) begin
            rd1_data_pipe <= grp_rd1_data[rd1_group_q];
        end
        if (pair_rd0_req_q) begin
            pair_rd0_data_pipe <= grp_pair_rd0_data[pair_rd0_group_q];
        end
        if (pair_rd1_req_q) begin
            pair_rd1_data_pipe <= grp_pair_rd1_data[pair_rd1_group_q];
        end
    end

    always_comb begin
        rd0_data      = grp_rd0_data[rd0_group_q];
        rd1_data      = grp_rd1_data[rd1_group_q];
        pair_rd0_data = grp_pair_rd0_data[pair_rd0_group_q];
        pair_rd1_data = grp_pair_rd1_data[pair_rd1_group_q];
    end
`endif
`else
    logic [MLDSA_COEFF_W-1:0] mem [0:BANKS-1][0:MLDSA_N-1];
    logic                     rd0_req_q;
    logic                     rd1_req_q;
    logic                     pair_rd0_req_q;
    logic                     pair_rd1_req_q;

    always_ff @(posedge clk) begin
        if (wr_en) mem[wr_bank][wr_addr] <= wr_data;
        if (pair_wr_en) begin
            mem[pair_wr_bank][{pair_wr_addr, 1'b0}] <= pair_wr_data[23:0];
            mem[pair_wr_bank][{pair_wr_addr, 1'b1}] <= pair_wr_data[47:24];
        end
        if (rd0_en) rd0_data <= mem[rd0_bank][rd0_addr];
        if (rd1_en) rd1_data <= mem[rd1_bank][rd1_addr];
        if (pair_rd0_en) begin
            pair_rd0_data <= {
                mem[pair_rd0_bank][{pair_rd0_addr, 1'b1}],
                mem[pair_rd0_bank][{pair_rd0_addr, 1'b0}]
            };
        end
        if (pair_rd1_en) begin
            pair_rd1_data <= {
                mem[pair_rd1_bank][{pair_rd1_addr, 1'b1}],
                mem[pair_rd1_bank][{pair_rd1_addr, 1'b0}]
            };
        end

        rd0_req_q <= rd0_en;
        rd1_req_q <= rd1_en;
        pair_rd0_req_q <= pair_rd0_en;
        pair_rd1_req_q <= pair_rd1_en;

        rd0_valid <= rd0_req_q;
        rd1_valid <= rd1_req_q;
        pair_rd0_valid <= pair_rd0_req_q;
        pair_rd1_valid <= pair_rd1_req_q;

        if (rd0_req_q) begin
            rd0_data_pipe <= rd0_data;
        end
        if (rd1_req_q) begin
            rd1_data_pipe <= rd1_data;
        end
        if (pair_rd0_req_q) begin
            pair_rd0_data_pipe <= pair_rd0_data;
        end
        if (pair_rd1_req_q) begin
            pair_rd1_data_pipe <= pair_rd1_data;
        end
    end
`endif
endmodule

`default_nettype wire
