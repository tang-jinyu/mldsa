`timescale 1ns/1ps
`default_nettype none

module mldsa65_keygen_pack #(
    parameter int K = 6,
    parameter int L = 5,
    parameter int ETA = 4,
    parameter int PK_BYTES = 1952,
    parameter int SK_BYTES = 4032
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    output logic       rd_valid,
    output logic [2:0] rd_kind,
    output logic [12:0] rd_addr,
    input  wire [23:0] rd_data,
    output logic       byte_valid,
    output logic [2:0] byte_region,
    output logic [12:0] byte_addr,
    output logic [7:0] byte_data,
    output logic       done,
    output logic       error
);
    localparam int T1_COEFFS = K * 256;
    localparam int T0_COEFFS = K * 256;
    localparam int S1_COEFFS = L * 256;
    localparam int S2_COEFFS = K * 256;
    localparam int ETA_GROUP_COEFFS = (ETA == 2) ? 8 : 2;
    localparam int ETA_GROUP_BYTES = (ETA == 2) ? 3 : 1;

    localparam logic [2:0] REGION_PK = 3'd0;
    localparam logic [2:0] REGION_SK = 3'd1;
    localparam logic [2:0] RD_RHO = 3'd0;
    localparam logic [2:0] RD_K = 3'd1;
    localparam logic [2:0] RD_T1 = 3'd2;
    localparam logic [2:0] RD_S1 = 3'd3;
    localparam logic [2:0] RD_S2 = 3'd4;
    localparam logic [2:0] RD_T0 = 3'd5;

    typedef enum logic [4:0] {
        S_IDLE,
        S_SHAKE_START,
        S_PK_RHO,
        S_PK_T1_LOAD,
        S_PK_T1_EMIT,
        S_TR_READ,
        S_SK_RHO,
        S_SK_K,
        S_SK_TR,
        S_SK_S1_LOAD,
        S_SK_S1_EMIT,
        S_SK_S2_LOAD,
        S_SK_S2_EMIT,
        S_SK_T0_LOAD,
        S_SK_T0_EMIT,
        S_DONE,
        S_ERROR
    } state_e;

    state_e st;
    logic [12:0] pk_idx;
    logic [12:0] sk_idx;
    logic [12:0] coeff_base;
    logic [3:0] load_idx;
    logic [3:0] emit_idx;
    logic [9:0] t1_buf [0:3];
    logic signed [4:0] eta_buf [0:7];
    logic signed [13:0] t0_buf [0:7];
    logic [7:0] tr_mem [0:63];
    logic [6:0] tr_idx;

    logic shake_start;
    logic shake_in_valid;
    logic shake_in_ready;
    logic [7:0] shake_in_data;
    logic shake_out_valid;
    logic shake_out_ready;
    logic [7:0] shake_out_data;
    logic shake_done;
    logic shake_error;

    mldsa65_shake_stream u_shake (
        .clk(clk),
        .rst_n(rst_n),
        .start(shake_start),
        .mode_shake256(1'b1),
        .in_len(16'(PK_BYTES)),
        .out_len(16'd64),
        .in_valid(shake_in_valid),
        .in_ready(shake_in_ready),
        .in_data(shake_in_data),
        .out_valid(shake_out_valid),
        .out_ready(shake_out_ready),
        .out_data(shake_out_data),
        .busy(),
        .done(shake_done),
        .error(shake_error)
    );

    function automatic [7:0] t1_pack_byte(
        input logic [2:0] idx,
        input logic [9:0] c0,
        input logic [9:0] c1,
        input logic [9:0] c2,
        input logic [9:0] c3
    );
        begin
            unique case (idx)
                3'd0: t1_pack_byte = c0[7:0];
                3'd1: t1_pack_byte = {c1[5:0], c0[9:8]};
                3'd2: t1_pack_byte = {c2[3:0], c1[9:6]};
                3'd3: t1_pack_byte = {c3[1:0], c2[9:4]};
                default: t1_pack_byte = c3[9:2];
            endcase
        end
    endfunction

    function automatic [3:0] eta_raw(input logic signed [4:0] coeff);
        int signed tmp;
        begin
            tmp = ETA - int'(coeff);
            eta_raw = tmp[3:0];
        end
    endfunction

    function automatic [7:0] eta_pack_byte(
        input logic [1:0] idx,
        input logic signed [4:0] c0,
        input logic signed [4:0] c1,
        input logic signed [4:0] c2,
        input logic signed [4:0] c3,
        input logic signed [4:0] c4,
        input logic signed [4:0] c5,
        input logic signed [4:0] c6,
        input logic signed [4:0] c7
    );
        logic [2:0] t0;
        logic [2:0] t1;
        logic [2:0] t2;
        logic [2:0] t3;
        logic [2:0] t4;
        logic [2:0] t5;
        logic [2:0] t6;
        logic [2:0] t7;
        logic [3:0] r0;
        logic [3:0] r1;
        logic [3:0] r2;
        logic [3:0] r3;
        logic [3:0] r4;
        logic [3:0] r5;
        logic [3:0] r6;
        logic [3:0] r7;
        begin
            if (ETA == 2) begin
                r0 = eta_raw(c0);
                r1 = eta_raw(c1);
                r2 = eta_raw(c2);
                r3 = eta_raw(c3);
                r4 = eta_raw(c4);
                r5 = eta_raw(c5);
                r6 = eta_raw(c6);
                r7 = eta_raw(c7);
                t0 = r0[2:0];
                t1 = r1[2:0];
                t2 = r2[2:0];
                t3 = r3[2:0];
                t4 = r4[2:0];
                t5 = r5[2:0];
                t6 = r6[2:0];
                t7 = r7[2:0];
                unique case (idx)
                    2'd0: eta_pack_byte = {t2[1:0], t1, t0};
                    2'd1: eta_pack_byte = {t5[0], t4, t3, t2[2]};
                    default: eta_pack_byte = {t7, t6, t5[2:1]};
                endcase
            end else begin
                eta_pack_byte = {eta_raw(c1), eta_raw(c0)};
            end
        end
    endfunction

    function automatic [12:0] t0_enc(input logic signed [13:0] coeff);
        int signed tmp;
        begin
            tmp = 4096 - int'(coeff);
            t0_enc = tmp[12:0];
        end
    endfunction

    function automatic [7:0] t0_pack_byte(
        input logic [3:0] idx,
        input logic signed [13:0] c0,
        input logic signed [13:0] c1,
        input logic signed [13:0] c2,
        input logic signed [13:0] c3,
        input logic signed [13:0] c4,
        input logic signed [13:0] c5,
        input logic signed [13:0] c6,
        input logic signed [13:0] c7
    );
        logic [12:0] e0;
        logic [12:0] e1;
        logic [12:0] e2;
        logic [12:0] e3;
        logic [12:0] e4;
        logic [12:0] e5;
        logic [12:0] e6;
        logic [12:0] e7;
        begin
            e0 = t0_enc(c0);
            e1 = t0_enc(c1);
            e2 = t0_enc(c2);
            e3 = t0_enc(c3);
            e4 = t0_enc(c4);
            e5 = t0_enc(c5);
            e6 = t0_enc(c6);
            e7 = t0_enc(c7);
            unique case (idx)
                4'd0:  t0_pack_byte = e0[7:0];
                4'd1:  t0_pack_byte = {e1[2:0], e0[12:8]};
                4'd2:  t0_pack_byte = e1[10:3];
                4'd3:  t0_pack_byte = {e2[5:0], e1[12:11]};
                4'd4:  t0_pack_byte = {e3[0], e2[12:6]};
                4'd5:  t0_pack_byte = e3[8:1];
                4'd6:  t0_pack_byte = {e4[3:0], e3[12:9]};
                4'd7:  t0_pack_byte = e4[11:4];
                4'd8:  t0_pack_byte = {e5[6:0], e4[12]};
                4'd9:  t0_pack_byte = {e6[1:0], e5[12:7]};
                4'd10: t0_pack_byte = e6[9:2];
                4'd11: t0_pack_byte = {e7[4:0], e6[12:10]};
                default: t0_pack_byte = e7[12:5];
            endcase
        end
    endfunction

    always_comb begin
        rd_valid = 1'b0;
        rd_kind = RD_RHO;
        rd_addr = 13'd0;
        byte_valid = 1'b0;
        byte_region = REGION_PK;
        byte_addr = 13'd0;
        byte_data = 8'd0;
        shake_start = (st == S_SHAKE_START);
        shake_in_valid = 1'b0;
        shake_in_data = byte_data;
        shake_out_ready = (st == S_TR_READ);

        unique case (st)
            S_PK_RHO: begin
                rd_valid = shake_in_ready;
                rd_kind = RD_RHO;
                rd_addr = pk_idx;
                byte_valid = shake_in_ready;
                byte_region = REGION_PK;
                byte_addr = pk_idx;
                byte_data = rd_data[7:0];
                shake_in_valid = shake_in_ready;
                shake_in_data = rd_data[7:0];
`ifdef MLDSA_DEBUG_DISPLAY
                if (shake_in_ready && pk_idx < 13'd8) begin
                    $display("KEYGEN_PACK_PK_RHO addr=%0d data=%02x rd=%06x",
                             pk_idx, rd_data[7:0], rd_data);
                end
`endif
            end
            S_PK_T1_LOAD: begin
                rd_valid = 1'b1;
                rd_kind = RD_T1;
                rd_addr = coeff_base + {9'd0, load_idx};
            end
            S_PK_T1_EMIT: begin
                byte_valid = shake_in_ready;
                byte_region = REGION_PK;
                byte_addr = pk_idx;
                byte_data = t1_pack_byte(emit_idx[2:0], t1_buf[0], t1_buf[1], t1_buf[2], t1_buf[3]);
                shake_in_valid = shake_in_ready;
                shake_in_data = byte_data;
`ifdef MLDSA_DEBUG_DISPLAY
                if (shake_in_ready && coeff_base == 13'd0) begin
                    $display("KEYGEN_PACK_T1_EMIT pk_idx=%0d emit_idx=%0d t1={%0d,%0d,%0d,%0d} data=%02x",
                             pk_idx, emit_idx, t1_buf[0], t1_buf[1], t1_buf[2], t1_buf[3], byte_data);
                end
`endif
            end
            S_SK_RHO: begin
                rd_valid = 1'b1;
                rd_kind = RD_RHO;
                rd_addr = sk_idx;
                byte_valid = 1'b1;
                byte_region = REGION_SK;
                byte_addr = sk_idx;
                byte_data = rd_data[7:0];
            end
            S_SK_K: begin
                rd_valid = 1'b1;
                rd_kind = RD_K;
                rd_addr = sk_idx - 13'd32;
                byte_valid = 1'b1;
                byte_region = REGION_SK;
                byte_addr = sk_idx;
                byte_data = rd_data[7:0];
            end
            S_SK_TR: begin
                byte_valid = 1'b1;
                byte_region = REGION_SK;
                byte_addr = sk_idx;
                byte_data = tr_mem[sk_idx[5:0]];
            end
            S_SK_S1_LOAD: begin
                rd_valid = 1'b1;
                rd_kind = RD_S1;
                rd_addr = coeff_base + {9'd0, load_idx};
            end
            S_SK_S1_EMIT: begin
                byte_valid = 1'b1;
                byte_region = REGION_SK;
                byte_addr = sk_idx;
                byte_data = eta_pack_byte(emit_idx[1:0], eta_buf[0], eta_buf[1], eta_buf[2], eta_buf[3],
                                          eta_buf[4], eta_buf[5], eta_buf[6], eta_buf[7]);
            end
            S_SK_S2_LOAD: begin
                rd_valid = 1'b1;
                rd_kind = RD_S2;
                rd_addr = coeff_base + {9'd0, load_idx};
            end
            S_SK_S2_EMIT: begin
                byte_valid = 1'b1;
                byte_region = REGION_SK;
                byte_addr = sk_idx;
                byte_data = eta_pack_byte(emit_idx[1:0], eta_buf[0], eta_buf[1], eta_buf[2], eta_buf[3],
                                          eta_buf[4], eta_buf[5], eta_buf[6], eta_buf[7]);
            end
            S_SK_T0_LOAD: begin
                rd_valid = 1'b1;
                rd_kind = RD_T0;
                rd_addr = coeff_base + {9'd0, load_idx};
            end
            S_SK_T0_EMIT: begin
                byte_valid = 1'b1;
                byte_region = REGION_SK;
                byte_addr = sk_idx;
                byte_data = t0_pack_byte(
                    emit_idx,
                    t0_buf[0],
                    t0_buf[1],
                    t0_buf[2],
                    t0_buf[3],
                    t0_buf[4],
                    t0_buf[5],
                    t0_buf[6],
                    t0_buf[7]
                );
            end
            default: ;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE;
            pk_idx <= 13'd0;
            sk_idx <= 13'd0;
            coeff_base <= 13'd0;
            load_idx <= 4'd0;
            emit_idx <= 4'd0;
            tr_idx <= 7'd0;
            done <= 1'b0;
            error <= 1'b0;
        end else begin
            done <= 1'b0;
            unique case (st)
                S_IDLE: begin
                    error <= 1'b0;
                    if (start) begin
                        pk_idx <= 13'd0;
                        sk_idx <= 13'd0;
                        coeff_base <= 13'd0;
                        load_idx <= 4'd0;
                        emit_idx <= 4'd0;
                        tr_idx <= 7'd0;
                        st <= S_SHAKE_START;
                    end
                end
                S_SHAKE_START: begin
                    pk_idx <= 13'd0;
                    st <= S_PK_RHO;
                end
                S_PK_RHO: begin
                    if (shake_error) st <= S_ERROR;
                    else if (shake_in_ready) begin
                        if (pk_idx == 13'd31) begin
                            coeff_base <= 13'd0;
                            load_idx <= 4'd0;
                        pk_idx <= 13'd32;
                            st <= S_PK_T1_LOAD;
                        end else begin
                            pk_idx <= pk_idx + 13'd1;
                        end
                    end
                end
                S_PK_T1_LOAD: begin
                    t1_buf[load_idx[1:0]] <= rd_data[9:0];
                    if (load_idx == 4'd3) begin
                        emit_idx <= 4'd0;
                        st <= S_PK_T1_EMIT;
                    end else begin
                        load_idx <= load_idx + 4'd1;
                    end
                end
                S_PK_T1_EMIT: begin
                    if (shake_error) st <= S_ERROR;
                    else if (shake_in_ready) begin
                        if (emit_idx == 4'd4) begin
                            if (coeff_base == 13'(T1_COEFFS - 4)) begin
                                tr_idx <= 7'd0;
                                st <= S_TR_READ;
                            end else begin
                                coeff_base <= coeff_base + 13'd4;
                                load_idx <= 4'd0;
                                st <= S_PK_T1_LOAD;
                            end
                            pk_idx <= pk_idx + 13'd1;
                        end else begin
                            emit_idx <= emit_idx + 4'd1;
                            pk_idx <= pk_idx + 13'd1;
                        end
                    end
                end
                S_TR_READ: begin
                    if (shake_error) st <= S_ERROR;
                    else if (shake_out_valid) begin
                        tr_mem[tr_idx[5:0]] <= shake_out_data;
                        if (tr_idx == 7'd63) begin
                            sk_idx <= 13'd0;
                            st <= S_SK_RHO;
                        end else begin
                            tr_idx <= tr_idx + 7'd1;
                        end
                    end else if (shake_done && tr_idx != 7'd64) begin
                        st <= S_ERROR;
                    end
                end
                S_SK_RHO: begin
                    if (sk_idx == 13'd31) begin
                        sk_idx <= 13'd32;
                        st <= S_SK_K;
                    end else begin
                        sk_idx <= sk_idx + 13'd1;
                    end
                end
                S_SK_K: begin
                    if (sk_idx == 13'd63) begin
                        sk_idx <= 13'd64;
                        st <= S_SK_TR;
                    end else begin
                        sk_idx <= sk_idx + 13'd1;
                    end
                end
                S_SK_TR: begin
                    if (sk_idx == 13'd127) begin
                        sk_idx <= 13'd128;
                        coeff_base <= 13'd0;
                        load_idx <= 4'd0;
                        st <= S_SK_S1_LOAD;
                    end else begin
                        sk_idx <= sk_idx + 13'd1;
                    end
                end
                S_SK_S1_LOAD: begin
                    eta_buf[load_idx[2:0]] <= rd_data[4:0];
                    if (int'(load_idx) == ETA_GROUP_COEFFS - 1) begin
                        emit_idx <= 4'd0;
                        st <= S_SK_S1_EMIT;
                    end else begin
                        load_idx <= load_idx + 4'd1;
                    end
                end
                S_SK_S1_EMIT: begin
                    if (int'(emit_idx) == ETA_GROUP_BYTES - 1) begin
                        if (coeff_base == 13'(S1_COEFFS - ETA_GROUP_COEFFS)) begin
                            sk_idx <= sk_idx + 13'd1;
                            coeff_base <= 13'd0;
                            load_idx <= 4'd0;
                            st <= S_SK_S2_LOAD;
                        end else begin
                            sk_idx <= sk_idx + 13'd1;
                            coeff_base <= coeff_base + 13'(ETA_GROUP_COEFFS);
                            load_idx <= 4'd0;
                            st <= S_SK_S1_LOAD;
                        end
                    end else begin
                        sk_idx <= sk_idx + 13'd1;
                        emit_idx <= emit_idx + 4'd1;
                    end
                end
                S_SK_S2_LOAD: begin
                    eta_buf[load_idx[2:0]] <= rd_data[4:0];
                    if (int'(load_idx) == ETA_GROUP_COEFFS - 1) begin
                        emit_idx <= 4'd0;
                        st <= S_SK_S2_EMIT;
                    end else begin
                        load_idx <= load_idx + 4'd1;
                    end
                end
                S_SK_S2_EMIT: begin
                    if (int'(emit_idx) == ETA_GROUP_BYTES - 1) begin
                        if (coeff_base == 13'(S2_COEFFS - ETA_GROUP_COEFFS)) begin
                            sk_idx <= sk_idx + 13'd1;
                            coeff_base <= 13'd0;
                            load_idx <= 4'd0;
                            st <= S_SK_T0_LOAD;
                        end else begin
                            sk_idx <= sk_idx + 13'd1;
                            coeff_base <= coeff_base + 13'(ETA_GROUP_COEFFS);
                            load_idx <= 4'd0;
                            st <= S_SK_S2_LOAD;
                        end
                    end else begin
                        sk_idx <= sk_idx + 13'd1;
                        emit_idx <= emit_idx + 4'd1;
                    end
                end
                S_SK_T0_LOAD: begin
                    t0_buf[load_idx[2:0]] <= rd_data[13:0];
                    if (load_idx == 4'd7) begin
                        emit_idx <= 4'd0;
                        st <= S_SK_T0_EMIT;
                    end else begin
                        load_idx <= load_idx + 4'd1;
                    end
                end
                S_SK_T0_EMIT: begin
                    if (emit_idx == 4'd12) begin
                        if (coeff_base == 13'(T0_COEFFS - 8)) begin
                            st <= S_DONE;
                        end else begin
                            sk_idx <= sk_idx + 13'd1;
                            coeff_base <= coeff_base + 13'd8;
                            load_idx <= 4'd0;
                            st <= S_SK_T0_LOAD;
                        end
                    end else begin
                        sk_idx <= sk_idx + 13'd1;
                        emit_idx <= emit_idx + 4'd1;
                    end
                end
                S_DONE: begin
                    done <= 1'b1;
                    st <= S_IDLE;
                end
                S_ERROR: begin
                    error <= 1'b1;
                    done <= 1'b1;
                    st <= S_IDLE;
                end
                default: st <= S_ERROR;
            endcase
        end
    end
endmodule

`default_nettype wire
