`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

module mldsa_byte_store_async #(
    parameter int DEPTH = 256,
    parameter int ADDR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH)
) (
    input  wire              clk,
    input  wire              wr_en,
    input  wire [ADDR_W-1:0] wr_addr,
    input  wire [7:0]        wr_data,
    input  wire [ADDR_W-1:0] rd_addr,
    output logic [7:0]       rd_data
);
`ifdef MLDSA_TARGET_ASIC
    localparam int BYTES_PER_WORD = 6;
    localparam int WORDS = (DEPTH + BYTES_PER_WORD - 1) / BYTES_PER_WORD;
    localparam int WORDS_PER_BANK = 512;
    localparam int BANKS = (WORDS + WORDS_PER_BANK - 1) / WORDS_PER_BANK;
    localparam int BANK_W = (BANKS <= 1) ? 1 : $clog2(BANKS);
    localparam int WORD_W = (WORDS <= 1) ? 1 : $clog2(WORDS);

    logic [WORD_W-1:0] wr_word_idx;
    logic [WORD_W-1:0] rd_word_idx;
    logic [BANK_W-1:0] wr_bank_sel;
    logic [BANK_W-1:0] rd_bank_sel;
    logic [8:0]        wr_row_sel;
    logic [8:0]        rd_row_sel;
    logic [2:0]        wr_lane_sel;
    logic [2:0]        rd_lane_sel;
    logic [BANK_W-1:0] rd_bank_sel_q;
    logic [2:0]        rd_lane_sel_q;
    logic [47:0]       bank_rd_word [0:BANKS-1];
    logic [47:0]       rd_word_mux;

    assign wr_word_idx = wr_addr / BYTES_PER_WORD;
    assign rd_word_idx = rd_addr / BYTES_PER_WORD;
    assign wr_bank_sel = wr_word_idx / WORDS_PER_BANK;
    assign rd_bank_sel = rd_word_idx / WORDS_PER_BANK;
    assign wr_row_sel  = wr_word_idx % WORDS_PER_BANK;
    assign rd_row_sel  = rd_word_idx % WORDS_PER_BANK;
    assign wr_lane_sel = wr_addr % BYTES_PER_WORD;
    assign rd_lane_sel = rd_addr % BYTES_PER_WORD;

    genvar bs_bank;
    generate
        for (bs_bank = 0; bs_bank < BANKS; bs_bank = bs_bank + 1) begin : g_byte_store_bank
            logic [47:0] macro_wr_data;
            logic        macro_wr_en;
            logic        macro_rd_en;

            always_comb begin
                macro_wr_en = wr_en && (wr_bank_sel == BANK_W'(bs_bank));
                macro_rd_en = !macro_wr_en && (rd_bank_sel == BANK_W'(bs_bank));
                macro_wr_data  = {48{1'b0}};
                if (macro_wr_en) begin
                    unique case (wr_lane_sel)
                        3'd0: macro_wr_data[7:0]   = wr_data;
                        3'd1: macro_wr_data[15:8]  = wr_data;
                        3'd2: macro_wr_data[23:16] = wr_data;
                        3'd3: macro_wr_data[31:24] = wr_data;
                        3'd4: macro_wr_data[39:32] = wr_data;
                        default: macro_wr_data[47:40] = wr_data;
                    endcase
                end
            end

            srambank_128x4x48_6t122 u_byte_store_sram (
                .clk     (clk),
                .ADDRESS (macro_wr_en ? wr_row_sel : rd_row_sel),
                .wd      (macro_wr_data),
                .banksel (macro_wr_en || macro_rd_en),
                .read    (macro_rd_en),
                .write   (macro_wr_en),
                .dataout (bank_rd_word[bs_bank])
            );
        end
    endgenerate

    always_ff @(posedge clk) begin
        rd_bank_sel_q <= rd_bank_sel;
        rd_lane_sel_q <= rd_lane_sel;
    end

    always_comb begin
        rd_word_mux = bank_rd_word[rd_bank_sel_q];
        unique case (rd_lane_sel_q)
            3'd0: rd_data = rd_word_mux[7:0];
            3'd1: rd_data = rd_word_mux[15:8];
            3'd2: rd_data = rd_word_mux[23:16];
            3'd3: rd_data = rd_word_mux[31:24];
            3'd4: rd_data = rd_word_mux[39:32];
            default: rd_data = rd_word_mux[47:40];
        endcase
    end
`else
    (* ram_style = "block" *) logic [7:0] mem [0:DEPTH-1];

    always_ff @(posedge clk) begin
        if (wr_en) mem[wr_addr] <= wr_data;
        rd_data <= mem[rd_addr];
    end
`endif
endmodule

module mldsa_ntt_poly_mem_compat (
    input  wire                     clk,
    input  wire                     load_we,
    input  wire [7:0]               load_addr,
    input  wire [MLDSA_COEFF_W-1:0] load_data,
    input  wire                     src_bank_sel,
    input  wire                     src_rd_req,
    input  wire                     src_rd_dual,
    input  wire [7:0]               src_rd0_addr,
    input  wire [7:0]               src_rd1_addr,
    output logic                    src_rd_valid,
    output logic [MLDSA_COEFF_W-1:0] src_rd0_data,
    output logic [MLDSA_COEFF_W-1:0] src_rd1_data,
    input  wire                     dst_bank_sel,
    input  wire                     dst_wr0_en,
    input  wire [7:0]               dst_wr0_addr,
    input  wire [MLDSA_COEFF_W-1:0] dst_wr0_data,
    input  wire                     dst_wr1_en,
    input  wire [7:0]               dst_wr1_addr,
    input  wire [MLDSA_COEFF_W-1:0] dst_wr1_data,
    input  wire                     host_bank_sel,
    input  wire [7:0]               host_rd_addr,
    output logic [MLDSA_COEFF_W-1:0] host_rd_data
);
    logic                    bank0_rd_valid;
    logic [MLDSA_COEFF_W-1:0] bank0_rd0_data;
    logic [MLDSA_COEFF_W-1:0] bank0_rd1_data;
    logic [MLDSA_COEFF_W-1:0] bank0_host_rd_data;
    logic                    bank1_rd_valid;
    logic [MLDSA_COEFF_W-1:0] bank1_rd0_data;
    logic [MLDSA_COEFF_W-1:0] bank1_rd1_data;
    logic [MLDSA_COEFF_W-1:0] bank1_host_rd_data;

    mldsa_ntt_cg_bank u_bank0 (
        .clk         (clk),
        .load_we     (load_we),
        .load_addr   (load_addr),
        .load_data   (load_data),
        .rd_req      (src_rd_req && !src_bank_sel),
        .rd_dual     (src_rd_dual),
        .rd0_addr    (src_rd0_addr),
        .rd1_addr    (src_rd1_addr),
        .rd_valid    (bank0_rd_valid),
        .rd0_data    (bank0_rd0_data),
        .rd1_data    (bank0_rd1_data),
        .wr0_en      (dst_wr0_en && !dst_bank_sel),
        .wr0_addr    (dst_wr0_addr),
        .wr0_data    (dst_wr0_data),
        .wr1_en      (dst_wr1_en && !dst_bank_sel),
        .wr1_addr    (dst_wr1_addr),
        .wr1_data    (dst_wr1_data),
        .host_rd_addr(host_rd_addr),
        .host_rd_data(bank0_host_rd_data)
    );

    mldsa_ntt_cg_bank u_bank1 (
        .clk         (clk),
        .load_we     (1'b0),
        .load_addr   (8'd0),
        .load_data   ('0),
        .rd_req      (src_rd_req && src_bank_sel),
        .rd_dual     (src_rd_dual),
        .rd0_addr    (src_rd0_addr),
        .rd1_addr    (src_rd1_addr),
        .rd_valid    (bank1_rd_valid),
        .rd0_data    (bank1_rd0_data),
        .rd1_data    (bank1_rd1_data),
        .wr0_en      (dst_wr0_en && dst_bank_sel),
        .wr0_addr    (dst_wr0_addr),
        .wr0_data    (dst_wr0_data),
        .wr1_en      (dst_wr1_en && dst_bank_sel),
        .wr1_addr    (dst_wr1_addr),
        .wr1_data    (dst_wr1_data),
        .host_rd_addr(host_rd_addr),
        .host_rd_data(bank1_host_rd_data)
    );

    always_comb begin
        src_rd_valid = src_bank_sel ? bank1_rd_valid : bank0_rd_valid;
        src_rd0_data = src_bank_sel ? bank1_rd0_data : bank0_rd0_data;
        src_rd1_data = src_bank_sel ? bank1_rd1_data : bank0_rd1_data;
        host_rd_data = host_bank_sel ? bank1_host_rd_data : bank0_host_rd_data;
    end
endmodule

module mldsa_ntt_cg_bank (
    input  wire                     clk,
    input  wire                     load_we,
    input  wire [7:0]               load_addr,
    input  wire [MLDSA_COEFF_W-1:0] load_data,
    input  wire                     rd_req,
    input  wire                     rd_dual,
    input  wire [7:0]               rd0_addr,
    input  wire [7:0]               rd1_addr,
    output logic                    rd_valid,
    output logic [MLDSA_COEFF_W-1:0] rd0_data,
    output logic [MLDSA_COEFF_W-1:0] rd1_data,
    input  wire                     wr0_en,
    input  wire [7:0]               wr0_addr,
    input  wire [MLDSA_COEFF_W-1:0] wr0_data,
    input  wire                     wr1_en,
    input  wire [7:0]               wr1_addr,
    input  wire [MLDSA_COEFF_W-1:0] wr1_data,
    input  wire [7:0]               host_rd_addr,
    output logic [MLDSA_COEFF_W-1:0] host_rd_data
);
    function automatic logic [1:0] mem_sel(input logic [7:0] addr);
        begin
            mem_sel = {addr[7], addr[0]};
        end
    endfunction

    function automatic logic [5:0] mem_off(input logic [7:0] addr);
        begin
            mem_off = addr[6:1];
        end
    endfunction

    logic                    rd_req_q;
    logic                    rd_dual_q;
    logic [1:0]              rd0_sel_q;
    logic [1:0]              rd1_sel_q;
    logic [MLDSA_COEFF_W-1:0] mem0_rd_q;
    logic [MLDSA_COEFF_W-1:0] mem1_rd_q;
    logic [MLDSA_COEFF_W-1:0] mem2_rd_q;
    logic [MLDSA_COEFF_W-1:0] mem3_rd_q;
    logic [MLDSA_COEFF_W-1:0] mem0_dbg_q;
    logic [MLDSA_COEFF_W-1:0] mem1_dbg_q;
    logic [MLDSA_COEFF_W-1:0] mem2_dbg_q;
    logic [MLDSA_COEFF_W-1:0] mem3_dbg_q;
    logic                     mem0_wr_en;
    logic                     mem1_wr_en;
    logic                     mem2_wr_en;
    logic                     mem3_wr_en;
    logic [5:0]               mem0_wr_addr;
    logic [5:0]               mem1_wr_addr;
    logic [5:0]               mem2_wr_addr;
    logic [5:0]               mem3_wr_addr;
    logic [MLDSA_COEFF_W-1:0] mem0_wr_data;
    logic [MLDSA_COEFF_W-1:0] mem1_wr_data;
    logic [MLDSA_COEFF_W-1:0] mem2_wr_data;
    logic [MLDSA_COEFF_W-1:0] mem3_wr_data;

    always_comb begin
        mem0_wr_en   = load_we && (mem_sel(load_addr) == 2'd0);
        mem0_wr_addr = mem_off(load_addr);
        mem0_wr_data = load_data;
        if (wr0_en && (mem_sel(wr0_addr) == 2'd0)) begin
            mem0_wr_en   = 1'b1;
            mem0_wr_addr = mem_off(wr0_addr);
            mem0_wr_data = wr0_data;
        end
        if (wr1_en && (mem_sel(wr1_addr) == 2'd0)) begin
            mem0_wr_en   = 1'b1;
            mem0_wr_addr = mem_off(wr1_addr);
            mem0_wr_data = wr1_data;
        end

        mem1_wr_en   = load_we && (mem_sel(load_addr) == 2'd1);
        mem1_wr_addr = mem_off(load_addr);
        mem1_wr_data = load_data;
        if (wr0_en && (mem_sel(wr0_addr) == 2'd1)) begin
            mem1_wr_en   = 1'b1;
            mem1_wr_addr = mem_off(wr0_addr);
            mem1_wr_data = wr0_data;
        end
        if (wr1_en && (mem_sel(wr1_addr) == 2'd1)) begin
            mem1_wr_en   = 1'b1;
            mem1_wr_addr = mem_off(wr1_addr);
            mem1_wr_data = wr1_data;
        end

        mem2_wr_en   = load_we && (mem_sel(load_addr) == 2'd2);
        mem2_wr_addr = mem_off(load_addr);
        mem2_wr_data = load_data;
        if (wr0_en && (mem_sel(wr0_addr) == 2'd2)) begin
            mem2_wr_en   = 1'b1;
            mem2_wr_addr = mem_off(wr0_addr);
            mem2_wr_data = wr0_data;
        end
        if (wr1_en && (mem_sel(wr1_addr) == 2'd2)) begin
            mem2_wr_en   = 1'b1;
            mem2_wr_addr = mem_off(wr1_addr);
            mem2_wr_data = wr1_data;
        end

        mem3_wr_en   = load_we && (mem_sel(load_addr) == 2'd3);
        mem3_wr_addr = mem_off(load_addr);
        mem3_wr_data = load_data;
        if (wr0_en && (mem_sel(wr0_addr) == 2'd3)) begin
            mem3_wr_en   = 1'b1;
            mem3_wr_addr = mem_off(wr0_addr);
            mem3_wr_data = wr0_data;
        end
        if (wr1_en && (mem_sel(wr1_addr) == 2'd3)) begin
            mem3_wr_en   = 1'b1;
            mem3_wr_addr = mem_off(wr1_addr);
            mem3_wr_data = wr1_data;
        end
    end

    mldsa_sp_sram64x24_wrapper u_mem0 (
        .clk     (clk),
        .rd_en   (rd_req && (mem_sel(rd0_addr) == 2'd0 || (rd_dual && (mem_sel(rd1_addr) == 2'd0)))),
        .rd_addr ((mem_sel(rd0_addr) == 2'd0) ? mem_off(rd0_addr) : mem_off(rd1_addr)),
        .rd_data (mem0_rd_q),
        .wr_en   (mem0_wr_en),
        .wr_addr (mem0_wr_addr),
        .wr_data (mem0_wr_data),
        .dbg_addr (mem_off(host_rd_addr)),
        .dbg_data (mem0_dbg_q)
    );

    mldsa_sp_sram64x24_wrapper u_mem1 (
        .clk     (clk),
        .rd_en   (rd_req && (mem_sel(rd0_addr) == 2'd1 || (rd_dual && (mem_sel(rd1_addr) == 2'd1)))),
        .rd_addr ((mem_sel(rd0_addr) == 2'd1) ? mem_off(rd0_addr) : mem_off(rd1_addr)),
        .rd_data (mem1_rd_q),
        .wr_en   (mem1_wr_en),
        .wr_addr (mem1_wr_addr),
        .wr_data (mem1_wr_data),
        .dbg_addr (mem_off(host_rd_addr)),
        .dbg_data (mem1_dbg_q)
    );

    mldsa_sp_sram64x24_wrapper u_mem2 (
        .clk     (clk),
        .rd_en   (rd_req && (mem_sel(rd0_addr) == 2'd2 || (rd_dual && (mem_sel(rd1_addr) == 2'd2)))),
        .rd_addr ((mem_sel(rd0_addr) == 2'd2) ? mem_off(rd0_addr) : mem_off(rd1_addr)),
        .rd_data (mem2_rd_q),
        .wr_en   (mem2_wr_en),
        .wr_addr (mem2_wr_addr),
        .wr_data (mem2_wr_data),
        .dbg_addr (mem_off(host_rd_addr)),
        .dbg_data (mem2_dbg_q)
    );

    mldsa_sp_sram64x24_wrapper u_mem3 (
        .clk     (clk),
        .rd_en   (rd_req && (mem_sel(rd0_addr) == 2'd3 || (rd_dual && (mem_sel(rd1_addr) == 2'd3)))),
        .rd_addr ((mem_sel(rd0_addr) == 2'd3) ? mem_off(rd0_addr) : mem_off(rd1_addr)),
        .rd_data (mem3_rd_q),
        .wr_en   (mem3_wr_en),
        .wr_addr (mem3_wr_addr),
        .wr_data (mem3_wr_data),
        .dbg_addr (mem_off(host_rd_addr)),
        .dbg_data (mem3_dbg_q)
    );

    always_ff @(posedge clk) begin
        rd_req_q  <= rd_req;
        rd_dual_q <= rd_dual;
        rd0_sel_q <= mem_sel(rd0_addr);
        rd1_sel_q <= mem_sel(rd1_addr);
    end

    always_comb begin
        rd_valid = rd_req_q;
        unique case (rd0_sel_q)
            2'd0: rd0_data = mem0_rd_q;
            2'd1: rd0_data = mem1_rd_q;
            2'd2: rd0_data = mem2_rd_q;
            default: rd0_data = mem3_rd_q;
        endcase
        if (rd_dual_q) begin
            unique case (rd1_sel_q)
                2'd0: rd1_data = mem0_rd_q;
                2'd1: rd1_data = mem1_rd_q;
                2'd2: rd1_data = mem2_rd_q;
                default: rd1_data = mem3_rd_q;
            endcase
        end else begin
            rd1_data = '0;
        end

        unique case (mem_sel(host_rd_addr))
            2'd0: host_rd_data = mem0_dbg_q;
            2'd1: host_rd_data = mem1_dbg_q;
            2'd2: host_rd_data = mem2_dbg_q;
            default: host_rd_data = mem3_dbg_q;
        endcase
    end

`ifndef SYNTHESIS
    always_ff @(posedge clk) begin
        if (wr0_en && wr1_en) begin
            assert (mem_sel(wr0_addr) != mem_sel(wr1_addr))
                else $error("mldsa_ntt_cg_bank write conflict: wr0_addr=%0d wr1_addr=%0d", wr0_addr, wr1_addr);
        end
    end
`endif
endmodule

module mldsa_sp_sram64x24_wrapper (
    input  wire                     clk,
    input  wire                     rd_en,
    input  wire [5:0]               rd_addr,
    output logic [MLDSA_COEFF_W-1:0] rd_data,
    input  wire                     wr_en,
    input  wire [5:0]               wr_addr,
    input  wire [MLDSA_COEFF_W-1:0] wr_data,
    input  wire [5:0]               dbg_addr,
    output logic [MLDSA_COEFF_W-1:0] dbg_data
);
`ifdef MLDSA_TARGET_ASIC
    logic        macro_rd_en;
    logic        macro_rd_is_dbg_q;
    logic [7:0]  macro_rd_addr;
    logic [31:0] macro_wr_data;
    logic [31:0] macro_rd_word;

    always_comb begin
        macro_rd_en    = rd_en || !wr_en;
        macro_rd_addr  = rd_en ? {2'b00, rd_addr} : {2'b00, dbg_addr};
        macro_wr_data  = {8'd0, wr_data};
    end

    always_ff @(posedge clk) begin
        macro_rd_is_dbg_q <= !rd_en && !wr_en;
    end

    always_comb begin
        rd_data  = macro_rd_word[23:0];
        dbg_data = macro_rd_is_dbg_q ? macro_rd_word[23:0] : '0;
    end

    srambank_64x4x32_6t122 u_sram64x24 (
        .clk     (clk),
        .ADDRESS (wr_en ? {2'b00, wr_addr} : macro_rd_addr),
        .wd      (macro_wr_data),
        .banksel (wr_en || macro_rd_en),
        .read    (!wr_en && macro_rd_en),
        .write   (wr_en),
        .dataout (macro_rd_word)
    );
`else
    (* ram_style = "block" *) logic [MLDSA_COEFF_W-1:0] mem [0:63];

    always_ff @(posedge clk) begin
        if (wr_en) begin
            mem[wr_addr] <= wr_data;
        end
        if (rd_en) begin
            rd_data <= mem[rd_addr];
        end
    end

    always_comb begin
        dbg_data = mem[dbg_addr];
    end
`endif
endmodule

module mldsa_ntt_zeta_rom (
    input  wire                     clk,
    input  wire [7:0]               addr,
    output logic [MLDSA_COEFF_W-1:0] data
);
`ifdef SYNTHESIS
    (* rom_style = "block" *) logic [MLDSA_COEFF_W-1:0] rom [0:255];

    initial begin
        $readmemh("shared_core/rtl/mldsa_ntt_zeta_rom.memh", rom);
    end

    always_comb begin
        data = rom[addr];
    end
`else
    (* rom_style = "block" *) logic [MLDSA_COEFF_W-1:0] rom [0:255];

    initial begin
        $readmemh("shared_core/rtl/mldsa_ntt_zeta_rom.memh", rom);
    end

    always_comb begin
        data = rom[addr];
    end
`endif
endmodule

module mldsa_sampler_challenge_mem_compat (
    input  wire                     clk,
    input  wire                     clr_we,
    input  wire [7:0]               clr_addr,
    input  wire [MLDSA_COEFF_W-1:0] clr_data,
    input  wire [7:0]               rd0_addr,
    output logic [MLDSA_COEFF_W-1:0] rd0_data,
    input  wire [7:0]               rd1_addr,
    output logic [MLDSA_COEFF_W-1:0] rd1_data,
    input  wire                     wr0_en,
    input  wire [7:0]               wr0_addr,
    input  wire [MLDSA_COEFF_W-1:0] wr0_data,
    input  wire                     wr1_en,
    input  wire [7:0]               wr1_addr,
    input  wire [MLDSA_COEFF_W-1:0] wr1_data
);
`ifdef SYNTHESIS
    function automatic logic [1:0] mem_sel(input logic [7:0] a);
        begin
            mem_sel = {a[7], a[0]};
        end
    endfunction

    function automatic logic [5:0] mem_off(input logic [7:0] a);
        begin
            mem_off = a[6:1];
        end
    endfunction

    logic [1:0] rd_sel_q;
    logic [MLDSA_COEFF_W-1:0] mem0_rd_q;
    logic [MLDSA_COEFF_W-1:0] mem1_rd_q;
    logic [MLDSA_COEFF_W-1:0] mem2_rd_q;
    logic [MLDSA_COEFF_W-1:0] mem3_rd_q;
    logic mem0_wr_en, mem1_wr_en, mem2_wr_en, mem3_wr_en;
    logic [5:0] mem0_wr_addr, mem1_wr_addr, mem2_wr_addr, mem3_wr_addr;
    logic [MLDSA_COEFF_W-1:0] mem0_wr_data, mem1_wr_data, mem2_wr_data, mem3_wr_data;
    logic challenge_rd_en;
    logic [7:0] challenge_rd_addr_sel;

    always_comb begin
        challenge_rd_en = 1'b1;
        challenge_rd_addr_sel = rd0_addr;
        if (rd0_addr == rd1_addr) begin
            challenge_rd_addr_sel = rd0_addr;
        end

        mem0_wr_en   = clr_we && (mem_sel(clr_addr) == 2'd0);
        mem0_wr_addr = mem_off(clr_addr);
        mem0_wr_data = clr_data;
        if (wr0_en && (mem_sel(wr0_addr) == 2'd0)) begin
            mem0_wr_en   = 1'b1;
            mem0_wr_addr = mem_off(wr0_addr);
            mem0_wr_data = wr0_data;
        end
        if (wr1_en && (mem_sel(wr1_addr) == 2'd0)) begin
            mem0_wr_en   = 1'b1;
            mem0_wr_addr = mem_off(wr1_addr);
            mem0_wr_data = wr1_data;
        end

        mem1_wr_en   = clr_we && (mem_sel(clr_addr) == 2'd1);
        mem1_wr_addr = mem_off(clr_addr);
        mem1_wr_data = clr_data;
        if (wr0_en && (mem_sel(wr0_addr) == 2'd1)) begin
            mem1_wr_en   = 1'b1;
            mem1_wr_addr = mem_off(wr0_addr);
            mem1_wr_data = wr0_data;
        end
        if (wr1_en && (mem_sel(wr1_addr) == 2'd1)) begin
            mem1_wr_en   = 1'b1;
            mem1_wr_addr = mem_off(wr1_addr);
            mem1_wr_data = wr1_data;
        end

        mem2_wr_en   = clr_we && (mem_sel(clr_addr) == 2'd2);
        mem2_wr_addr = mem_off(clr_addr);
        mem2_wr_data = clr_data;
        if (wr0_en && (mem_sel(wr0_addr) == 2'd2)) begin
            mem2_wr_en   = 1'b1;
            mem2_wr_addr = mem_off(wr0_addr);
            mem2_wr_data = wr0_data;
        end
        if (wr1_en && (mem_sel(wr1_addr) == 2'd2)) begin
            mem2_wr_en   = 1'b1;
            mem2_wr_addr = mem_off(wr1_addr);
            mem2_wr_data = wr1_data;
        end

        mem3_wr_en   = clr_we && (mem_sel(clr_addr) == 2'd3);
        mem3_wr_addr = mem_off(clr_addr);
        mem3_wr_data = clr_data;
        if (wr0_en && (mem_sel(wr0_addr) == 2'd3)) begin
            mem3_wr_en   = 1'b1;
            mem3_wr_addr = mem_off(wr0_addr);
            mem3_wr_data = wr0_data;
        end
        if (wr1_en && (mem_sel(wr1_addr) == 2'd3)) begin
            mem3_wr_en   = 1'b1;
            mem3_wr_addr = mem_off(wr1_addr);
            mem3_wr_data = wr1_data;
        end
    end

    mldsa_sp_sram64x24_wrapper u_challenge_mem0 (
        .clk     (clk),
        .rd_en   (challenge_rd_en && (mem_sel(challenge_rd_addr_sel) == 2'd0)),
        .rd_addr (mem_off(challenge_rd_addr_sel)),
        .rd_data (mem0_rd_q),
        .wr_en   (mem0_wr_en),
        .wr_addr (mem0_wr_addr),
        .wr_data (mem0_wr_data),
        .dbg_addr(mem_off(rd1_addr)),
        .dbg_data()
    );

    mldsa_sp_sram64x24_wrapper u_challenge_mem1 (
        .clk     (clk),
        .rd_en   (challenge_rd_en && (mem_sel(challenge_rd_addr_sel) == 2'd1)),
        .rd_addr (mem_off(challenge_rd_addr_sel)),
        .rd_data (mem1_rd_q),
        .wr_en   (mem1_wr_en),
        .wr_addr (mem1_wr_addr),
        .wr_data (mem1_wr_data),
        .dbg_addr(mem_off(rd1_addr)),
        .dbg_data()
    );

    mldsa_sp_sram64x24_wrapper u_challenge_mem2 (
        .clk     (clk),
        .rd_en   (challenge_rd_en && (mem_sel(challenge_rd_addr_sel) == 2'd2)),
        .rd_addr (mem_off(challenge_rd_addr_sel)),
        .rd_data (mem2_rd_q),
        .wr_en   (mem2_wr_en),
        .wr_addr (mem2_wr_addr),
        .wr_data (mem2_wr_data),
        .dbg_addr(mem_off(rd1_addr)),
        .dbg_data()
    );

    mldsa_sp_sram64x24_wrapper u_challenge_mem3 (
        .clk     (clk),
        .rd_en   (challenge_rd_en && (mem_sel(challenge_rd_addr_sel) == 2'd3)),
        .rd_addr (mem_off(challenge_rd_addr_sel)),
        .rd_data (mem3_rd_q),
        .wr_en   (mem3_wr_en),
        .wr_addr (mem3_wr_addr),
        .wr_data (mem3_wr_data),
        .dbg_addr(mem_off(rd1_addr)),
        .dbg_data()
    );

    always_ff @(posedge clk) begin
        rd_sel_q <= mem_sel(challenge_rd_addr_sel);
    end

    always_comb begin
        unique case (rd_sel_q)
            2'd0: rd0_data = mem0_rd_q;
            2'd1: rd0_data = mem1_rd_q;
            2'd2: rd0_data = mem2_rd_q;
            default: rd0_data = mem3_rd_q;
        endcase
        rd1_data = rd0_data;
    end
`else
    logic [MLDSA_COEFF_W-1:0] mem [0:MLDSA_N-1];

    always_ff @(posedge clk) begin
        if (clr_we) mem[clr_addr] <= clr_data;
        if (wr0_en) mem[wr0_addr] <= wr0_data;
        if (wr1_en) mem[wr1_addr] <= wr1_data;
    end

    always_comb begin
        rd0_data = mem[rd0_addr];
        rd1_data = mem[rd1_addr];
    end
`endif
endmodule

`default_nettype wire
