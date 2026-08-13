`timescale 1ns/1ps
`default_nettype none

import mldsa_pkg::*;

module mldsa65_sign_verify_subsys #(
    parameter int BANKS = 8
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         load_we,
    input  wire [$clog2(BANKS)-1:0]     load_bank,
    input  wire [7:0]                   load_addr,
    input  wire [MLDSA_COEFF_W-1:0]     load_data,
    input  wire                         start_sign,
    input  wire                         start_verify,
    output logic                        busy,
    output logic                        sign_done,
    output logic                        sign_pass,
    output logic                        verify_done,
    output logic                        verify_pass,
    output logic [7:0]                  err_code,
    output logic [7:0]                  state_dbg
);

    localparam logic [$clog2(BANKS)-1:0] B_Z       = 3'd0;
    localparam logic [$clog2(BANKS)-1:0] B_W       = 3'd1;
    localparam logic [$clog2(BANKS)-1:0] B_W1_SIGN = 3'd2;
    localparam logic [$clog2(BANKS)-1:0] B_HINT_S  = 3'd3;
    localparam logic [$clog2(BANKS)-1:0] B_HINT_V  = 3'd4;
    localparam logic [$clog2(BANKS)-1:0] B_W1_VER  = 3'd5;

    localparam int W1_PACK_BYTES   = MLDSA_N * 3;
    localparam int HINT_PACK_BYTES = MLDSA_N / 8;
    localparam int W1_ADDR_W       = (W1_PACK_BYTES <= 1) ? 1 : $clog2(W1_PACK_BYTES);
    localparam int HINT_ADDR_W     = (HINT_PACK_BYTES <= 1) ? 1 : $clog2(HINT_PACK_BYTES);
    localparam logic [MLDSA_COEFF_W-1:0] L65_GAMMA2 = 24'd261888;
    localparam logic [MLDSA_COEFF_W-1:0] L65_Z_BOUND = 24'd524092;

    typedef enum logic [7:0] {
        S_IDLE,
        S_SIGN_NORM_KICK,
        S_SIGN_NORM_FEED,
        S_SIGN_NORM_WAIT,
        S_SIGN_READ_W,
        S_SIGN_DECOMP_KICK,
        S_SIGN_DECOMP_WAIT,
        S_SIGN_HINT_KICK,
        S_SIGN_HINT_WAIT,
        S_SIGN_WRITE,
        S_SIGN_PACKW_KICK,
        S_SIGN_PACKW_READ,
        S_SIGN_PACKW_PUSH,
        S_SIGN_PACKW_WAIT,
        S_SIGN_PACKH_KICK,
        S_SIGN_PACKH_READ,
        S_SIGN_PACKH_PUSH,
        S_SIGN_PACKH_WAIT,
        S_SIGN_DONE,
        S_VER_UNPACKH_KICK,
        S_VER_UNPACKH_FEED,
        S_VER_UNPACKH_WAIT,
        S_VER_NORM_KICK,
        S_VER_NORM_FEED,
        S_VER_NORM_WAIT,
        S_VER_READ_W,
        S_VER_USEH_KICK,
        S_VER_USEH_WAIT,
        S_VER_WRITE,
        S_VER_PACKW_KICK,
        S_VER_PACKW_READ,
        S_VER_PACKW_PUSH,
        S_VER_PACKW_WAIT,
        S_VER_CMP_KICK,
        S_VER_CMP_FEED,
        S_VER_CMP_WAIT,
        S_VER_DONE,
        S_ERROR
    } state_e;

    state_e st;

    logic                         ram_rd0_en;
    logic [$clog2(BANKS)-1:0]     ram_rd0_bank;
    logic [7:0]                   ram_rd0_addr;
    logic [MLDSA_COEFF_W-1:0]     ram_rd0_data;
    logic                         ram_rd1_en;
    logic [$clog2(BANKS)-1:0]     ram_rd1_bank;
    logic [7:0]                   ram_rd1_addr;
    logic [MLDSA_COEFF_W-1:0]     ram_rd1_data;
    logic                         ram_wr_en;
    logic [$clog2(BANKS)-1:0]     ram_wr_bank;
    logic [7:0]                   ram_wr_addr;
    logic [MLDSA_COEFF_W-1:0]     ram_wr_data;

    logic                         ctrl_rd0_en;
    logic [$clog2(BANKS)-1:0]     ctrl_rd0_bank;
    logic [7:0]                   ctrl_rd0_addr;
    logic                         ctrl_rd1_en;
    logic [$clog2(BANKS)-1:0]     ctrl_rd1_bank;
    logic [7:0]                   ctrl_rd1_addr;
    logic                         ctrl_wr_en;
    logic [$clog2(BANKS)-1:0]     ctrl_wr_bank;
    logic [7:0]                   ctrl_wr_addr;
    logic [MLDSA_COEFF_W-1:0]     ctrl_wr_data;

    logic                         check_start;
    logic [1:0]                   check_mode;
    logic [15:0]                  check_item_count;
    logic [MLDSA_COEFF_W-1:0]     check_limit;
    logic [MLDSA_COEFF_W-1:0]     check_data_a;
    logic [MLDSA_COEFF_W-1:0]     check_data_b;
    logic                         check_data_valid;
    logic                         check_data_ready;
    logic                         check_pass;
    logic [15:0]                  check_fail_index;
    logic [15:0]                  check_accum_value;
    logic                         check_busy;
    logic                         check_done;
    logic [2:0]                   check_state_dbg;

    logic                         dh_start;
    logic [1:0]                   dh_op;
    logic [MLDSA_COEFF_W-1:0]     dh_coeff_in;
    logic [MLDSA_COEFF_W-1:0]     dh_aux_in;
    logic [19:0]                  dh_gamma2;
    logic [MLDSA_COEFF_W-1:0]     dh_high_out;
    logic [MLDSA_COEFF_W-1:0]     dh_low_out;
    logic                         dh_hint_out;
    logic                         dh_busy;
    logic                         dh_done;

    logic                         pack_start;
    logic [3:0]                   pack_mode;
    logic [15:0]                  pack_item_count;
    logic [MLDSA_COEFF_W-1:0]     pack_coeff_in;
    logic                         pack_coeff_in_valid;
    logic                         pack_coeff_in_ready;
    logic [7:0]                   pack_byte_out;
    logic                         pack_byte_out_valid;
    logic                         pack_byte_out_ready;
    logic [7:0]                   pack_byte_in;
    logic                         pack_byte_in_valid;
    logic                         pack_byte_in_ready;
    logic [MLDSA_COEFF_W-1:0]     pack_coeff_out;
    logic                         pack_coeff_out_valid;
    logic                         pack_coeff_out_ready;
    logic                         pack_busy;
    logic                         pack_done;
    logic                         pack_error;

    logic                        sign_w1_wr_en;
    logic [W1_ADDR_W-1:0]       sign_w1_wr_addr;
    logic [7:0]                 sign_w1_wr_data;
    logic [W1_ADDR_W-1:0]       sign_w1_rd_addr;
    logic [7:0]                 sign_w1_rd_data;
    logic                        verify_w1_wr_en;
    logic [W1_ADDR_W-1:0]       verify_w1_wr_addr;
    logic [7:0]                 verify_w1_wr_data;
    logic [W1_ADDR_W-1:0]       verify_w1_rd_addr;
    logic [7:0]                 verify_w1_rd_data;
    logic                        hint_wr_en;
    logic [HINT_ADDR_W-1:0]     hint_wr_addr;
    logic [7:0]                 hint_wr_data;
    logic [HINT_ADDR_W-1:0]     hint_rd_addr;
    logic [7:0]                 hint_rd_data;

    logic [7:0] coeff_idx;
    logic [9:0] byte_idx;
    logic [MLDSA_COEFF_W-1:0] coeff_hold0;
    logic [MLDSA_COEFF_W-1:0] coeff_hold1;
    logic                     sign_norm_ok;
    logic                     verify_norm_ok;

    mldsa_poly_ram #(.BANKS(BANKS)) u_ram (
        .clk      (clk),
        .rd0_en   (ram_rd0_en),
        .rd0_bank (ram_rd0_bank),
        .rd0_addr (ram_rd0_addr),
        .rd0_data (ram_rd0_data),
        .pair_rd0_en   (1'b0),
        .pair_rd0_bank ('0),
        .pair_rd0_addr (7'd0),
        .pair_rd0_data (),
        .rd1_en   (ram_rd1_en),
        .rd1_bank (ram_rd1_bank),
        .rd1_addr (ram_rd1_addr),
        .rd1_data (ram_rd1_data),
        .pair_rd1_en   (1'b0),
        .pair_rd1_bank ('0),
        .pair_rd1_addr (7'd0),
        .pair_rd1_data (),
        .wr_en    (ram_wr_en),
        .wr_bank  (ram_wr_bank),
        .wr_addr  (ram_wr_addr),
        .wr_data  (ram_wr_data),
        .pair_wr_en   (1'b0),
        .pair_wr_bank ('0),
        .pair_wr_addr (7'd0),
        .pair_wr_data (48'd0)
    );

    mldsa_check u_check (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (check_start),
        .mode        (check_mode),
        .item_count  (check_item_count),
        .limit_value (check_limit),
        .data_a      (check_data_a),
        .data_b      (check_data_b),
        .data_valid  (check_data_valid),
        .data_ready  (check_data_ready),
        .pass        (check_pass),
        .fail_index  (check_fail_index),
        .accum_value (check_accum_value),
        .busy        (check_busy),
        .done        (check_done),
        .state_dbg   (check_state_dbg)
    );

    mldsa_decompose_hint u_dh (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (dh_start),
        .op_code   (dh_op),
        .coeff_in  (dh_coeff_in),
        .aux_in    (dh_aux_in),
        .gamma2    (dh_gamma2),
        .high_out  (dh_high_out),
        .low_out   (dh_low_out),
        .hint_out  (dh_hint_out),
        .busy      (dh_busy),
        .done      (dh_done),
        .state_dbg ()
    );

    mldsa_pack_unpack u_pack (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (pack_start),
        .mode           (pack_mode),
        .item_count     (pack_item_count),
        .coeff_in       (pack_coeff_in),
        .coeff_in_valid (pack_coeff_in_valid),
        .coeff_in_ready (pack_coeff_in_ready),
        .byte_out       (pack_byte_out),
        .byte_out_valid (pack_byte_out_valid),
        .byte_out_ready (pack_byte_out_ready),
        .byte_in        (pack_byte_in),
        .byte_in_valid  (pack_byte_in_valid),
        .byte_in_ready  (pack_byte_in_ready),
        .coeff_out      (pack_coeff_out),
        .coeff_out_valid(pack_coeff_out_valid),
        .coeff_out_ready(pack_coeff_out_ready),
        .busy           (pack_busy),
        .done           (pack_done),
        .error          (pack_error),
        .state_dbg      ()
    );

    mldsa_byte_store_async #(
        .DEPTH (W1_PACK_BYTES)
    ) u_sign_w1_store (
        .clk    (clk),
        .wr_en  (sign_w1_wr_en),
        .wr_addr(sign_w1_wr_addr),
        .wr_data(sign_w1_wr_data),
        .rd_addr(sign_w1_rd_addr),
        .rd_data(sign_w1_rd_data)
    );

    mldsa_byte_store_async #(
        .DEPTH (W1_PACK_BYTES)
    ) u_verify_w1_store (
        .clk    (clk),
        .wr_en  (verify_w1_wr_en),
        .wr_addr(verify_w1_wr_addr),
        .wr_data(verify_w1_wr_data),
        .rd_addr(verify_w1_rd_addr),
        .rd_data(verify_w1_rd_data)
    );

    mldsa_byte_store_async #(
        .DEPTH (HINT_PACK_BYTES)
    ) u_hint_store (
        .clk    (clk),
        .wr_en  (hint_wr_en),
        .wr_addr(hint_wr_addr),
        .wr_data(hint_wr_data),
        .rd_addr(hint_rd_addr),
        .rd_data(hint_rd_data)
    );

    always_comb begin
        ctrl_rd0_en   = 1'b0;
        ctrl_rd0_bank = B_Z;
        ctrl_rd0_addr = coeff_idx;
        ctrl_rd1_en   = 1'b0;
        ctrl_rd1_bank = B_HINT_V;
        ctrl_rd1_addr = coeff_idx;
        ctrl_wr_en    = 1'b0;
        ctrl_wr_bank  = B_W1_SIGN;
        ctrl_wr_addr  = coeff_idx;
        ctrl_wr_data  = dh_high_out;

        check_start      = 1'b0;
        check_mode       = MLDSA_CHECK_NORM;
        check_item_count = 16'd256;
        check_limit      = L65_Z_BOUND;
        check_data_a     = ram_rd0_data;
        check_data_b     = 24'd0;
        check_data_valid = 1'b0;

        dh_start     = 1'b0;
        dh_op        = MLDSA_DH_DECOMPOSE;
        dh_coeff_in  = coeff_hold0;
        dh_aux_in    = coeff_hold1;
        dh_gamma2    = L65_GAMMA2[19:0];

        pack_start          = 1'b0;
        pack_mode           = MLDSA_PACK_COEFF24_PACK;
        pack_item_count     = 16'd256;
        pack_coeff_in       = ram_rd0_data;
        pack_coeff_in_valid = 1'b0;
        pack_byte_out_ready = 1'b1;
        pack_byte_in        = 8'd0;
        pack_byte_in_valid  = 1'b0;
        pack_coeff_out_ready= 1'b1;

        sign_w1_wr_en   = 1'b0;
        sign_w1_wr_addr = byte_idx[W1_ADDR_W-1:0];
        sign_w1_wr_data = pack_byte_out;
        sign_w1_rd_addr = byte_idx[W1_ADDR_W-1:0];
        verify_w1_wr_en   = 1'b0;
        verify_w1_wr_addr = byte_idx[W1_ADDR_W-1:0];
        verify_w1_wr_data = pack_byte_out;
        verify_w1_rd_addr = byte_idx[W1_ADDR_W-1:0];
        hint_wr_en   = 1'b0;
        hint_wr_addr = byte_idx[HINT_ADDR_W-1:0];
        hint_wr_data = pack_byte_out;
        hint_rd_addr = byte_idx[HINT_ADDR_W-1:0];

        unique case (st)
            S_SIGN_NORM_KICK: begin
                check_start = 1'b1;
                ctrl_rd0_en = 1'b1;
                ctrl_rd0_bank = B_Z;
                ctrl_rd0_addr = 8'd0;
            end
            S_SIGN_NORM_FEED: begin
                if (coeff_idx != 8'd255) begin
                    ctrl_rd0_en = 1'b1;
                    ctrl_rd0_bank = B_Z;
                    ctrl_rd0_addr = coeff_idx + 8'd1;
                end
                check_data_a = ram_rd0_data;
                check_data_valid = check_data_ready;
            end
            S_SIGN_READ_W: begin
                ctrl_rd0_en = 1'b1;
                ctrl_rd0_bank = B_W;
                ctrl_rd0_addr = coeff_idx;
            end
            S_SIGN_DECOMP_KICK: begin
                dh_start = 1'b1;
                dh_op    = MLDSA_DH_DECOMPOSE;
                dh_coeff_in = ram_rd0_data;
            end
            S_SIGN_HINT_KICK: begin
                dh_start = 1'b1;
                dh_op    = MLDSA_DH_MAKE_HINT;
                dh_coeff_in = coeff_hold0;
                dh_aux_in   = 24'd0;
            end
            S_SIGN_WRITE: begin
                ctrl_wr_en   = 1'b1;
                ctrl_wr_bank = B_W1_SIGN;
                ctrl_wr_addr = coeff_idx;
                ctrl_wr_data = dh_high_out;
            end
            S_SIGN_PACKW_KICK: begin
                pack_start = 1'b1;
                pack_mode  = MLDSA_PACK_COEFF24_PACK;
                pack_item_count = 16'd256;
            end
            S_SIGN_PACKW_READ: begin
                ctrl_rd0_en = 1'b1;
                ctrl_rd0_bank = B_W1_SIGN;
                ctrl_rd0_addr = coeff_idx;
            end
            S_SIGN_PACKW_PUSH: begin
                pack_coeff_in = ram_rd0_data;
                pack_coeff_in_valid = 1'b1;
            end
            S_SIGN_PACKH_KICK: begin
                pack_start = 1'b1;
                pack_mode  = MLDSA_PACK_BIT1_PACK;
                pack_item_count = 16'd256;
            end
            S_SIGN_PACKH_READ: begin
                ctrl_rd0_en = 1'b1;
                ctrl_rd0_bank = B_HINT_S;
                ctrl_rd0_addr = coeff_idx;
            end
            S_SIGN_PACKH_PUSH: begin
                pack_coeff_in = 24'd0;
                pack_coeff_in_valid = 1'b1;
            end
            S_VER_UNPACKH_KICK: begin
                pack_start = 1'b1;
                pack_mode  = MLDSA_PACK_BIT1_UNPACK;
                pack_item_count = 16'd256;
            end
            S_VER_UNPACKH_FEED: begin
                pack_byte_in = hint_rd_data;
                pack_byte_in_valid = 1'b1;
                if (pack_coeff_out_valid) begin
                    ctrl_wr_en   = 1'b1;
                    ctrl_wr_bank = B_HINT_V;
                    ctrl_wr_addr = coeff_idx;
                    ctrl_wr_data = pack_coeff_out;
                end
            end
            S_VER_UNPACKH_WAIT: begin
                if (pack_coeff_out_valid) begin
                    ctrl_wr_en   = 1'b1;
                    ctrl_wr_bank = B_HINT_V;
                    ctrl_wr_addr = coeff_idx;
                    ctrl_wr_data = pack_coeff_out;
                end
            end
            S_VER_NORM_KICK: begin
                check_start = 1'b1;
                ctrl_rd0_en = 1'b1;
                ctrl_rd0_bank = B_Z;
                ctrl_rd0_addr = 8'd0;
            end
            S_VER_NORM_FEED: begin
                if (coeff_idx != 8'd255) begin
                    ctrl_rd0_en = 1'b1;
                    ctrl_rd0_bank = B_Z;
                    ctrl_rd0_addr = coeff_idx + 8'd1;
                end
                check_data_a = ram_rd0_data;
                check_data_valid = check_data_ready;
            end
            S_VER_READ_W: begin
                ctrl_rd0_en = 1'b1;
                ctrl_rd0_bank = B_W;
                ctrl_rd0_addr = coeff_idx;
                ctrl_rd1_en = 1'b1;
                ctrl_rd1_bank = B_HINT_V;
                ctrl_rd1_addr = coeff_idx;
            end
            S_VER_USEH_KICK: begin
                dh_start = 1'b1;
                dh_op    = MLDSA_DH_USE_HINT;
                dh_coeff_in = ram_rd0_data;
                dh_aux_in   = ram_rd1_data;
            end
            S_VER_WRITE: begin
                ctrl_wr_en   = 1'b1;
                ctrl_wr_bank = B_W1_VER;
                ctrl_wr_addr = coeff_idx;
                ctrl_wr_data = dh_high_out;
            end
            S_VER_PACKW_KICK: begin
                pack_start = 1'b1;
                pack_mode  = MLDSA_PACK_COEFF24_PACK;
                pack_item_count = 16'd256;
            end
            S_VER_PACKW_READ: begin
                ctrl_rd0_en = 1'b1;
                ctrl_rd0_bank = B_W1_VER;
                ctrl_rd0_addr = coeff_idx;
            end
            S_VER_PACKW_PUSH: begin
                pack_coeff_in = ram_rd0_data;
                pack_coeff_in_valid = 1'b1;
            end
            S_VER_CMP_KICK: begin
                check_start = 1'b1;
                check_mode = MLDSA_CHECK_BYTEEQ;
                check_item_count = W1_PACK_BYTES;
                check_limit = 24'd0;
            end
            S_VER_CMP_FEED: begin
                check_mode = MLDSA_CHECK_BYTEEQ;
                check_item_count = W1_PACK_BYTES;
                check_limit = 24'd0;
                check_data_a = {16'd0, sign_w1_rd_data};
                check_data_b = {16'd0, verify_w1_rd_data};
                check_data_valid = check_data_ready;
            end
            default: ;
        endcase

        if ((st == S_SIGN_PACKW_PUSH || st == S_SIGN_PACKW_WAIT) && pack_byte_out_valid) begin
            sign_w1_wr_en = 1'b1;
        end

        if ((st == S_SIGN_PACKH_PUSH || st == S_SIGN_PACKH_WAIT) && pack_byte_out_valid) begin
            hint_wr_en = 1'b1;
        end

        if ((st == S_VER_PACKW_PUSH || st == S_VER_PACKW_WAIT) && pack_byte_out_valid) begin
            verify_w1_wr_en = 1'b1;
        end

        if ((st == S_VER_UNPACKH_FEED || st == S_VER_UNPACKH_WAIT) && pack_byte_out_valid) begin
            hint_wr_en = 1'b1;
        end
    end

    always_comb begin
        if (st == S_IDLE) begin
            ram_rd0_en   = 1'b0;
            ram_rd0_bank = '0;
            ram_rd0_addr = 8'd0;
            ram_rd1_en   = 1'b0;
            ram_rd1_bank = '0;
            ram_rd1_addr = 8'd0;
            ram_wr_en    = load_we;
            ram_wr_bank  = load_bank;
            ram_wr_addr  = load_addr;
            ram_wr_data  = load_data;
        end else begin
            ram_rd0_en   = ctrl_rd0_en;
            ram_rd0_bank = ctrl_rd0_bank;
            ram_rd0_addr = ctrl_rd0_addr;
            ram_rd1_en   = ctrl_rd1_en;
            ram_rd1_bank = ctrl_rd1_bank;
            ram_rd1_addr = ctrl_rd1_addr;
            ram_wr_en    = ctrl_wr_en;
            ram_wr_bank  = ctrl_wr_bank;
            ram_wr_addr  = ctrl_wr_addr;
            ram_wr_data  = ctrl_wr_data;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st            <= S_IDLE;
            busy          <= 1'b0;
            sign_done     <= 1'b0;
            sign_pass     <= 1'b0;
            verify_done   <= 1'b0;
            verify_pass   <= 1'b0;
            err_code      <= 8'd0;
            state_dbg     <= 8'd0;
            coeff_idx     <= 8'd0;
            byte_idx      <= 10'd0;
            coeff_hold0   <= 24'd0;
            coeff_hold1   <= 24'd0;
            sign_norm_ok  <= 1'b0;
            verify_norm_ok<= 1'b0;
        end else begin
            sign_done   <= 1'b0;
            verify_done <= 1'b0;
            state_dbg   <= st;
            unique case (st)
                S_IDLE: begin
                    busy <= 1'b0;
                    err_code <= 8'd0;
                    if (start_sign) begin
                        busy <= 1'b1;
                        sign_pass <= 1'b0;
                        coeff_idx <= 8'd0;
                        byte_idx <= 10'd0;
                        st <= S_SIGN_NORM_KICK;
                    end else if (start_verify) begin
                        busy <= 1'b1;
                        verify_pass <= 1'b0;
                        coeff_idx <= 8'd0;
                        byte_idx <= 10'd0;
                        st <= S_VER_UNPACKH_KICK;
                    end
                end
                S_SIGN_NORM_KICK: begin
                    coeff_idx <= 8'd0;
                    st <= S_SIGN_NORM_FEED;
                end
                S_SIGN_NORM_FEED: begin
                    if (check_data_ready) begin
                        if (coeff_idx == 8'd255) st <= S_SIGN_NORM_WAIT;
                        else coeff_idx <= coeff_idx + 8'd1;
                    end
                end
                S_SIGN_NORM_WAIT: begin
                    if (check_done) begin
                        sign_norm_ok <= check_pass;
                        if (!check_pass) begin
                            err_code <= 8'h11;
                            st <= S_ERROR;
                        end else begin
                            coeff_idx <= 8'd0;
                            st <= S_SIGN_READ_W;
                        end
                    end
                end
                S_SIGN_READ_W: begin
                    st <= S_SIGN_DECOMP_KICK;
                end
                S_SIGN_DECOMP_KICK: begin
                    st <= S_SIGN_DECOMP_WAIT;
                end
                S_SIGN_DECOMP_WAIT: begin
                    if (dh_done) begin
                        coeff_hold0 <= dh_high_out;
                        st <= S_SIGN_HINT_KICK;
                    end
                end
                S_SIGN_HINT_KICK: begin
                    st <= S_SIGN_HINT_WAIT;
                end
                S_SIGN_HINT_WAIT: begin
                    if (dh_done) st <= S_SIGN_WRITE;
                end
                S_SIGN_WRITE: begin
                    if (coeff_idx == 8'd255) begin
                        coeff_idx <= 8'd0;
                        byte_idx <= 10'd0;
                        st <= S_SIGN_PACKW_KICK;
                    end else begin
                        coeff_idx <= coeff_idx + 8'd1;
                        st <= S_SIGN_READ_W;
                    end
                end
                S_SIGN_PACKW_KICK: begin
                    coeff_idx <= 8'd0;
                    byte_idx <= 10'd0;
                    st <= S_SIGN_PACKW_READ;
                end
                S_SIGN_PACKW_READ: begin
                    st <= S_SIGN_PACKW_PUSH;
                end
                S_SIGN_PACKW_PUSH: begin
                    if (pack_coeff_in_ready) begin
                        if (coeff_idx == 8'd255) st <= S_SIGN_PACKW_WAIT;
                        else begin
                            coeff_idx <= coeff_idx + 8'd1;
                            st <= S_SIGN_PACKW_READ;
                        end
                    end
                    if (pack_byte_out_valid) begin
                        byte_idx <= byte_idx + 10'd1;
                    end
                end
                S_SIGN_PACKW_WAIT: begin
                    if (pack_byte_out_valid) begin
                        byte_idx <= byte_idx + 10'd1;
                    end
                    if (pack_done) begin
                        coeff_idx <= 8'd0;
                        byte_idx <= 10'd0;
                        st <= S_SIGN_PACKH_KICK;
                    end
                end
                S_SIGN_PACKH_KICK: begin
                    coeff_idx <= 8'd0;
                    byte_idx <= 10'd0;
                    st <= S_SIGN_PACKH_READ;
                end
                S_SIGN_PACKH_READ: begin
                    st <= S_SIGN_PACKH_PUSH;
                end
                S_SIGN_PACKH_PUSH: begin
                    if (pack_coeff_in_ready) begin
                        if (coeff_idx == 8'd255) st <= S_SIGN_PACKH_WAIT;
                        else begin
                            coeff_idx <= coeff_idx + 8'd1;
                            st <= S_SIGN_PACKH_READ;
                        end
                    end
                    if (pack_byte_out_valid) begin
                        byte_idx <= byte_idx + 10'd1;
                    end
                end
                S_SIGN_PACKH_WAIT: begin
                    if (pack_byte_out_valid) begin
                        byte_idx <= byte_idx + 10'd1;
                    end
                    if (pack_done) st <= S_SIGN_DONE;
                end
                S_SIGN_DONE: begin
                    busy <= 1'b0;
                    sign_done <= 1'b1;
                    sign_pass <= sign_norm_ok;
                    st <= S_IDLE;
                end
                S_VER_UNPACKH_KICK: begin
                    byte_idx <= 10'd0;
                    coeff_idx <= 8'd0;
                    st <= S_VER_UNPACKH_FEED;
                end
                S_VER_UNPACKH_FEED: begin
                    if (pack_byte_in_ready) begin
                        if (byte_idx == HINT_PACK_BYTES-1) st <= S_VER_UNPACKH_WAIT;
                        else byte_idx <= byte_idx + 10'd1;
                    end
                    if (pack_coeff_out_valid) begin
                        coeff_idx <= coeff_idx + 8'd1;
                    end
                end
                S_VER_UNPACKH_WAIT: begin
                    if (pack_coeff_out_valid) begin
                        coeff_idx <= coeff_idx + 8'd1;
                    end
                    if (pack_done) begin
                        coeff_idx <= 8'd0;
                        st <= S_VER_NORM_KICK;
                    end
                end
                S_VER_NORM_KICK: begin
                    coeff_idx <= 8'd0;
                    st <= S_VER_NORM_FEED;
                end
                S_VER_NORM_FEED: begin
                    if (check_data_ready) begin
                        if (coeff_idx == 8'd255) st <= S_VER_NORM_WAIT;
                        else coeff_idx <= coeff_idx + 8'd1;
                    end
                end
                S_VER_NORM_WAIT: begin
                    if (check_done) begin
                        verify_norm_ok <= check_pass;
                        if (!check_pass) begin
                            err_code <= 8'h21;
                            st <= S_ERROR;
                        end else begin
                            coeff_idx <= 8'd0;
                            st <= S_VER_READ_W;
                        end
                    end
                end
                S_VER_READ_W: begin
                    st <= S_VER_USEH_KICK;
                end
                S_VER_USEH_KICK: begin
                    st <= S_VER_USEH_WAIT;
                end
                S_VER_USEH_WAIT: begin
                    if (dh_done) st <= S_VER_WRITE;
                end
                S_VER_WRITE: begin
                    if (coeff_idx == 8'd255) begin
                        coeff_idx <= 8'd0;
                        byte_idx <= 10'd0;
                        st <= S_VER_PACKW_KICK;
                    end else begin
                        coeff_idx <= coeff_idx + 8'd1;
                        st <= S_VER_READ_W;
                    end
                end
                S_VER_PACKW_KICK: begin
                    coeff_idx <= 8'd0;
                    byte_idx <= 10'd0;
                    st <= S_VER_PACKW_READ;
                end
                S_VER_PACKW_READ: begin
                    st <= S_VER_PACKW_PUSH;
                end
                S_VER_PACKW_PUSH: begin
                    if (pack_coeff_in_ready) begin
                        if (coeff_idx == 8'd255) st <= S_VER_PACKW_WAIT;
                        else begin
                            coeff_idx <= coeff_idx + 8'd1;
                            st <= S_VER_PACKW_READ;
                        end
                    end
                    if (pack_byte_out_valid) begin
                        byte_idx <= byte_idx + 10'd1;
                    end
                end
                S_VER_PACKW_WAIT: begin
                    if (pack_byte_out_valid) begin
                        byte_idx <= byte_idx + 10'd1;
                    end
                    if (pack_done) begin
                        byte_idx <= 10'd0;
                        st <= S_VER_CMP_KICK;
                    end
                end
                S_VER_CMP_KICK: begin
                    byte_idx <= 10'd0;
                    st <= S_VER_CMP_FEED;
                end
                S_VER_CMP_FEED: begin
                    if (check_data_ready) begin
                        if (byte_idx == W1_PACK_BYTES-1) st <= S_VER_CMP_WAIT;
                        else byte_idx <= byte_idx + 10'd1;
                    end
                end
                S_VER_CMP_WAIT: begin
                    if (check_done) begin
                        verify_pass <= verify_norm_ok && check_pass;
                        st <= S_VER_DONE;
                    end
                end
                S_VER_DONE: begin
                    busy <= 1'b0;
                    verify_done <= 1'b1;
                    st <= S_IDLE;
                end
                S_ERROR: begin
                    busy <= 1'b0;
                    sign_done <= 1'b0;
                    verify_done <= 1'b0;
                    verify_pass <= 1'b0;
                    sign_pass <= 1'b0;
                    st <= S_IDLE;
                end
                default: st <= S_ERROR;
            endcase
        end
    end
endmodule

`default_nettype wire
