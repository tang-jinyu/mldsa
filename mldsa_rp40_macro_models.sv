`timescale 1ns/1ps
`default_nettype none

// Lightweight declarations for RP40 hard macros used by RTL wrappers.
// For gate-level/silicon simulations, compile the vendor macro Verilog instead
// and define MLDSA_RP40_VENDOR_MACRO_MODEL to skip these black boxes.
`ifdef MLDSA_USE_RP40_SRAM
`ifndef MLDSA_RP40_VENDOR_MACRO_MODEL
module SRAM_128x48 (
    input  wire [6:0]  AB,
    input  wire [6:0]  AA,
    input  wire [47:0] DI,
    input  wire        CEBB,
    input  wire        CEBA,
    input  wire [47:0] WEB,
    input  wire        CKB,
    input  wire        CKA,
    input  wire        LS,
    input  wire        EMCEB,
    input  wire [3:0]  EMCB,
    input  wire        EMCEA,
    input  wire [3:0]  EMCA,
    output logic [47:0] DO
);
`ifndef SYNTHESIS
    logic [47:0] mem [0:127];
    integer i;

    always_ff @(posedge CKA) begin
        if (!CEBA) begin
            for (i = 0; i < 48; i = i + 1) begin
                if (!WEB[i]) mem[AA][i] <= DI[i];
            end
        end
    end

    always_ff @(posedge CKB) begin
        if (!CEBB) begin
            DO <= mem[AB];
        end
    end
`endif
endmodule

module ROM_512x24 (
    output logic [23:0] DO,
    input  wire [8:0]  A,
    input  wire        CEB,
    input  wire        CK,
    input  wire        LS,
    input  wire        EMCE,
    input  wire [3:0]  EMC
);
`ifndef SYNTHESIS
    logic [23:0] mem [0:511];

    initial begin : init_rom_512x24
        integer idx;
        for (idx = 0; idx < 512; idx = idx + 1) begin
            mem[idx] = '0;
        end
        $readmemh("shared_core/rtl/mldsa_ntt_zeta_rom.memh", mem, 0, 255);
    end

    always_ff @(posedge CK) begin
        if (!CEB) begin
            DO <= mem[A];
        end
    end
`endif
endmodule
`endif
`endif

`default_nettype wire
