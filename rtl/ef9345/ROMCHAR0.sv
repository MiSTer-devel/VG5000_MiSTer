// ----------------------------------------------------------------------------------
// Part of the Verilog translation of the orpigame EF9345 core, 2026.
// Copyright D. Laurens (original design). This source describes Open Hardware
// and is licensed under the CERN-OHL-S v2 (see LICENSE in this directory).
//
// ROMCHAR0: 2048 x 8 character ROM expected by CharactersDecoder (the VHDL
// sources do not include it, see vdu/rom/README.md): G0 set ($000-$4FF) and G10 set ($500-$77F).
// 10 consecutive bytes per character (one per scan line), bit 7 = left dot,
// synchronous read (one clock of latency).
// Contents: $readmemh of INIT_FILE (one hex byte per line; default from the
// macro EF9345_ROMCHAR0_HEX, else "romchar0.hex"), e.g. made from
// the EF9345 character ROM (see rtl/ef9345/README.md).
// 2026: default file rtl/ef9345/romcharN.hex (Quartus project directory).
// ----------------------------------------------------------------------------------

`ifndef EF9345_ROMCHAR0_HEX
`define EF9345_ROMCHAR0_HEX "rtl/ef9345/romchar0.hex"
`endif

module ROMCHAR0 #(
    parameter INIT_FILE = `EF9345_ROMCHAR0_HEX
) (
    input  logic        CLK,
    input  logic [10:0] ADDR,
    output logic [7:0]  DO
);

logic [7:0] rom [0:2047];

initial $readmemh(INIT_FILE, rom);

always_ff @(posedge CLK)
  DO <= rom[ADDR];

endmodule
