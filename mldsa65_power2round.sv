`timescale 1ns/1ps
`default_nettype none

module mldsa65_power2round #(
    parameter int Q = 8380417,
    parameter int D = 13
) (
    input  wire [23:0] coeff_in,
    output logic [9:0] t1_out,
    output logic signed [13:0] t0_out
);
    logic [23:0] coeff_q;
    int unsigned high;
    int signed low;

    always_comb begin
        coeff_q = (coeff_in >= 24'(Q)) ? coeff_in - 24'(Q) : coeff_in;
        high = (int'(coeff_q) + (1 << (D - 1)) - 1) >> D;
        low = int'(coeff_q) - (high <<< D);
        if (low <= -(1 << (D - 1))) begin
            high = high - 1;
            low = low + (1 << D);
        end
        t1_out = high[9:0];
        t0_out = low[13:0];
    end
endmodule

`default_nettype wire
