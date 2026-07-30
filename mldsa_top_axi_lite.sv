`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

module mldsa_top_axi_lite #(
    parameter int BANKS = 65
) (
    input  wire                         clk,
    input  wire                         rst_n,

    input  wire [11:0]                  s_axi_awaddr,
    input  wire                         s_axi_awvalid,
    output logic                        s_axi_awready,
    input  wire [31:0]                  s_axi_wdata,
    input  wire [3:0]                   s_axi_wstrb,
    input  wire                         s_axi_wvalid,
    output logic                        s_axi_wready,
    output wire [1:0]                   s_axi_bresp,
    output logic                        s_axi_bvalid,
    input  wire                         s_axi_bready,
    input  wire [11:0]                  s_axi_araddr,
    input  wire                         s_axi_arvalid,
    output logic                        s_axi_arready,
    output logic [31:0]                 s_axi_rdata,
    output wire [1:0]                   s_axi_rresp,
    output logic                        s_axi_rvalid,
    input  wire                         s_axi_rready,

    output wire                         irq
);

    localparam logic [31:0] CORE_ID      = 32'h4D4C_4453;
    localparam logic [11:0] A_CTRL       = 12'h000;
    localparam logic [11:0] A_STATUS     = 12'h004;
    localparam logic [11:0] A_CFG        = 12'h008;
    localparam logic [11:0] A_POLY_BANK  = 12'h00C;
    localparam logic [11:0] A_POLY_ADDR  = 12'h010;
    localparam logic [11:0] A_POLY_WDATA = 12'h014;
    localparam logic [11:0] A_POLY_CTRL  = 12'h018;
    localparam logic [11:0] A_POLY_RDATA = 12'h01C;
    localparam logic [11:0] A_SEED_ADDR  = 12'h020;
    localparam logic [11:0] A_SEED_DATA  = 12'h024;
    localparam logic [11:0] A_UOP_DBG    = 12'h028;
    localparam logic [11:0] A_STATE_DBG  = 12'h02C;
    localparam logic [11:0] A_CYCLES     = 12'h030;
    localparam logic [11:0] A_STD_CFG    = 12'h034;
    localparam logic [11:0] A_CTX_SEL    = 12'h038;
    localparam logic [11:0] A_CTX_ADDR   = 12'h03C;
    localparam logic [11:0] A_CTX_DATA   = 12'h040;
    localparam logic [11:0] A_ID         = 12'hF00;

    logic                        core_start;
    logic                        core_busy;
    logic                        core_done;
    logic                        core_pass;
    logic                        core_error;
    logic [7:0]                  core_err_code;
    logic [7:0]                  core_uop_dbg;
    logic [7:0]                  core_state_dbg;
    logic                        load_poly_we_q;
    logic                        load_seed_we_q;
    logic                        host_rd_issue_q;
    logic                        host_rd_capture_q;
    logic [1:0]                  level_sel_reg;
    logic [1:0]                  op_mode_reg;
    logic [$clog2(BANKS)-1:0]    poly_bank_reg;
    logic [7:0]                  poly_addr_reg;
    logic [23:0]                 poly_wdata_reg;
    logic [23:0]                 poly_rdata_reg;
    logic [4:0]                  seed_addr_reg;
    logic [7:0]                  seed_data_reg;
    logic                        standard_mode_reg;
    logic [12:0]                 message_len_reg;
    logic [1:0]                  ctx_sel_reg;
    logic [12:0]                 ctx_addr_reg;
    logic [7:0]                  ctx_data_reg;
    logic                        load_ctx_we_q;
    logic [7:0]                  err_latched;
    logic                        done_latched;
    logic                        pass_latched;
    logic                        error_latched;
    logic [31:0]                 op_cycle_counter;
    logic [31:0]                 last_op_cycles;
    logic [23:0]                 host_rd_data;

    logic wr_fire;
    logic rd_fire;

    function automatic logic [31:0] apply_wstrb(
        input logic [31:0] old_value,
        input logic [31:0] new_value,
        input logic [3:0]  strobe
    );
        logic [31:0] merged;
        begin
            merged = old_value;
            if (strobe[0]) merged[7:0]   = new_value[7:0];
            if (strobe[1]) merged[15:8]  = new_value[15:8];
            if (strobe[2]) merged[23:16] = new_value[23:16];
            if (strobe[3]) merged[31:24] = new_value[31:24];
            apply_wstrb = merged;
        end
    endfunction

    function automatic logic [31:0] read_word(input logic [11:0] addr);
        logic [31:0] data;
        begin
            data = 32'd0;
            if (addr == A_STATUS) begin
                data = {16'd0, err_latched, 4'd0, pass_latched, error_latched, done_latched, core_busy};
            end else if (addr == A_CFG) begin
                data = {22'd0, level_sel_reg, 6'd0, op_mode_reg};
            end else if (addr == A_POLY_BANK) begin
                data = {{(32-$clog2(BANKS)){1'b0}}, poly_bank_reg};
            end else if (addr == A_POLY_ADDR) begin
                data = {24'd0, poly_addr_reg};
            end else if (addr == A_POLY_WDATA) begin
                data = {8'd0, poly_wdata_reg};
            end else if (addr == A_POLY_CTRL) begin
                data = {30'd0, host_rd_issue_q, load_poly_we_q};
            end else if (addr == A_POLY_RDATA) begin
                data = {8'd0, poly_rdata_reg};
            end else if (addr == A_SEED_ADDR) begin
                data = {27'd0, seed_addr_reg};
            end else if (addr == A_SEED_DATA) begin
                data = {24'd0, seed_data_reg};
            end else if (addr == A_UOP_DBG) begin
                data = {24'd0, core_uop_dbg};
            end else if (addr == A_STATE_DBG) begin
                data = {24'd0, core_state_dbg};
            end else if (addr == A_CYCLES) begin
                data = last_op_cycles;
            end else if (addr == A_STD_CFG) begin
                data = {3'd0, message_len_reg, 15'd0, standard_mode_reg};
            end else if (addr == A_CTX_SEL) begin
                data = {30'd0, ctx_sel_reg};
            end else if (addr == A_CTX_ADDR) begin
                data = {19'd0, ctx_addr_reg};
            end else if (addr == A_CTX_DATA) begin
                data = {24'd0, ctx_data_reg};
            end else if (addr == A_ID) begin
                data = CORE_ID;
            end
            read_word = data;
        end
    endfunction

    assign wr_fire = (!s_axi_bvalid) && s_axi_awvalid && s_axi_wvalid;
    assign rd_fire = (!s_axi_rvalid) && s_axi_arvalid;
    assign s_axi_bresp = 2'b00;
    assign s_axi_rresp = 2'b00;
    assign irq = done_latched;

    mldsa_top_wrapper #(.BANKS(BANKS)) u_core (
        .clk           (clk),
        .rst_n         (rst_n),
        .load_poly_we  (load_poly_we_q),
        .load_poly_bank(poly_bank_reg),
        .load_poly_addr(poly_addr_reg),
        .load_poly_data(poly_wdata_reg),
        .host_rd_en    (host_rd_issue_q && !host_rd_capture_q),
        .host_rd_bank  (poly_bank_reg),
        .host_rd_addr  (poly_addr_reg),
        .load_seed_we  (load_seed_we_q),
        .load_seed_addr(seed_addr_reg),
        .load_seed_data(seed_data_reg),
        .standard_mode (standard_mode_reg),
        .load_ctx_we   (load_ctx_we_q),
        .load_ctx_sel  (ctx_sel_reg),
        .load_ctx_addr (ctx_addr_reg),
        .load_ctx_data (ctx_data_reg),
        .message_len   (message_len_reg),
        .start         (core_start),
        .level_sel     (level_sel_reg),
        .op_mode       (op_mode_reg),
        .busy          (core_busy),
        .done          (core_done),
        .pass          (core_pass),
        .error         (core_error),
        .err_code      (core_err_code),
        .uop_dbg       (core_uop_dbg),
        .state_dbg     (core_state_dbg),
        .host_rd_data  (host_rd_data)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        logic [31:0] merged;
        if (!rst_n) begin
            s_axi_awready <= 1'b0;
            s_axi_wready <= 1'b0;
            s_axi_bvalid <= 1'b0;
            s_axi_arready <= 1'b0;
            s_axi_rvalid <= 1'b0;
            s_axi_rdata <= 32'd0;
            core_start <= 1'b0;
            load_poly_we_q <= 1'b0;
            load_seed_we_q <= 1'b0;
            load_ctx_we_q <= 1'b0;
            host_rd_issue_q <= 1'b0;
            host_rd_capture_q <= 1'b0;
            level_sel_reg <= MLDSA_LEVEL_65;
            op_mode_reg <= MLDSA_OP_SIGN;
            poly_bank_reg <= '0;
            poly_addr_reg <= 8'd0;
            poly_wdata_reg <= 24'd0;
            poly_rdata_reg <= 24'd0;
            seed_addr_reg <= 5'd0;
            seed_data_reg <= 8'd0;
            standard_mode_reg <= 1'b0;
            message_len_reg <= 13'd0;
            ctx_sel_reg <= 2'd0;
            ctx_addr_reg <= 13'd0;
            ctx_data_reg <= 8'd0;
            err_latched <= 8'd0;
            done_latched <= 1'b0;
            pass_latched <= 1'b0;
            error_latched <= 1'b0;
            op_cycle_counter <= 32'd0;
            last_op_cycles <= 32'd0;
        end else begin
            s_axi_awready <= 1'b0;
            s_axi_wready <= 1'b0;
            s_axi_arready <= 1'b0;
            core_start <= 1'b0;

            if (load_poly_we_q) load_poly_we_q <= 1'b0;
            if (load_seed_we_q) load_seed_we_q <= 1'b0;
            if (load_ctx_we_q) load_ctx_we_q <= 1'b0;

            if (host_rd_issue_q) begin
                if (!host_rd_capture_q) begin
                    host_rd_capture_q <= 1'b1;
                end else begin
                    poly_rdata_reg <= host_rd_data;
                    host_rd_issue_q <= 1'b0;
                    host_rd_capture_q <= 1'b0;
                end
            end

            if (core_busy) begin
                op_cycle_counter <= op_cycle_counter + 32'd1;
            end

            if (core_done || core_error) begin
                done_latched <= 1'b1;
                pass_latched <= core_done ? core_pass : 1'b0;
                error_latched <= core_error;
                err_latched <= core_err_code;
                last_op_cycles <= op_cycle_counter;
                op_cycle_counter <= 32'd0;
            end

            if (wr_fire) begin
                s_axi_awready <= 1'b1;
                s_axi_wready <= 1'b1;
                s_axi_bvalid <= 1'b1;

                if (s_axi_awaddr == A_CTRL) begin
                    if (s_axi_wdata[0] && !core_busy) begin
                        core_start <= 1'b1;
                        done_latched <= 1'b0;
                        pass_latched <= 1'b0;
                        error_latched <= 1'b0;
                        err_latched <= 8'd0;
                        op_cycle_counter <= 32'd0;
                    end
                    if (s_axi_wdata[1]) begin
                        done_latched <= 1'b0;
                        pass_latched <= 1'b0;
                        error_latched <= 1'b0;
                        err_latched <= 8'd0;
                    end
                end else if (s_axi_awaddr == A_STATUS) begin
                    if (s_axi_wdata[1]) begin
                        done_latched <= 1'b0;
                        pass_latched <= 1'b0;
                        error_latched <= 1'b0;
                        err_latched <= 8'd0;
                    end
                end else if (s_axi_awaddr == A_CFG) begin
                    merged = apply_wstrb({22'd0, level_sel_reg, 6'd0, op_mode_reg}, s_axi_wdata, s_axi_wstrb);
                    level_sel_reg <= merged[9:8];
                    op_mode_reg <= merged[1:0];
                end else if (s_axi_awaddr == A_POLY_BANK) begin
                    merged = apply_wstrb({{(32-$clog2(BANKS)){1'b0}}, poly_bank_reg}, s_axi_wdata, s_axi_wstrb);
                    poly_bank_reg <= merged[$clog2(BANKS)-1:0];
                end else if (s_axi_awaddr == A_POLY_ADDR) begin
                    merged = apply_wstrb({24'd0, poly_addr_reg}, s_axi_wdata, s_axi_wstrb);
                    poly_addr_reg <= merged[7:0];
                end else if (s_axi_awaddr == A_POLY_WDATA) begin
                    merged = apply_wstrb({8'd0, poly_wdata_reg}, s_axi_wdata, s_axi_wstrb);
                    poly_wdata_reg <= merged[23:0];
                end else if (s_axi_awaddr == A_POLY_CTRL) begin
                    if (!core_busy) begin
                        if (s_axi_wdata[0]) load_poly_we_q <= 1'b1;
                        if (s_axi_wdata[1]) begin
                            host_rd_issue_q <= 1'b1;
                            host_rd_capture_q <= 1'b0;
                        end
                    end
                end else if (s_axi_awaddr == A_SEED_ADDR) begin
                    merged = apply_wstrb({27'd0, seed_addr_reg}, s_axi_wdata, s_axi_wstrb);
                    seed_addr_reg <= merged[4:0];
                end else if (s_axi_awaddr == A_SEED_DATA) begin
                    merged = apply_wstrb({24'd0, seed_data_reg}, s_axi_wdata, s_axi_wstrb);
                    seed_data_reg <= merged[7:0];
                    if (!core_busy) load_seed_we_q <= 1'b1;
                end else if (s_axi_awaddr == A_STD_CFG) begin
                    merged = apply_wstrb({3'd0, message_len_reg, 15'd0, standard_mode_reg}, s_axi_wdata, s_axi_wstrb);
                    standard_mode_reg <= merged[0];
                    message_len_reg <= merged[28:16];
                end else if (s_axi_awaddr == A_CTX_SEL) begin
                    merged = apply_wstrb({30'd0, ctx_sel_reg}, s_axi_wdata, s_axi_wstrb);
                    ctx_sel_reg <= merged[1:0];
                end else if (s_axi_awaddr == A_CTX_ADDR) begin
                    merged = apply_wstrb({19'd0, ctx_addr_reg}, s_axi_wdata, s_axi_wstrb);
                    ctx_addr_reg <= merged[12:0];
                end else if (s_axi_awaddr == A_CTX_DATA) begin
                    merged = apply_wstrb({24'd0, ctx_data_reg}, s_axi_wdata, s_axi_wstrb);
                    ctx_data_reg <= merged[7:0];
                    if (!core_busy) load_ctx_we_q <= 1'b1;
                end
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end

            if (rd_fire) begin
                s_axi_arready <= 1'b1;
                s_axi_rvalid <= 1'b1;
                s_axi_rdata <= read_word(s_axi_araddr);
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end
endmodule

`default_nettype wire
