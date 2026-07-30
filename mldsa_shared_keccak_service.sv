`timescale 1ns/1ps
`default_nettype none

// Synthesis-oriented shared Keccak service anchor.
// In functional simulation we still keep the original per-block XOF path so
// that the already-closed 44/65/87 KAT regressions remain stable.
module mldsa_shared_keccak_service (
    input  wire clk,
    input  wire rst_n,
    output wire busy,
    output wire done
);
    logic [63:0] out_data_unused;
    logic        out_valid_unused;
    logic        squeeze_ready_unused;
    logic        absorb_done_unused;

    (* dont_touch = "true" *)
    mldsa_keccak_xof_adapter u_shared_keccak (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (1'b0),
        .mode_shake256 (1'b1),
        .in_data       (64'd0),
        .in_valid      (1'b0),
        .in_ready      (),
        .in_last       (1'b0),
        .in_last_pos   (8'd0),
        .out_data      (out_data_unused),
        .out_valid     (out_valid_unused),
        .out_ready     (1'b0),
        .squeeze_done  (1'b0),
        .squeeze_ready (squeeze_ready_unused),
        .absorb_done   (absorb_done_unused),
        .busy          (busy),
        .done          (done)
    );
endmodule

`default_nettype wire
