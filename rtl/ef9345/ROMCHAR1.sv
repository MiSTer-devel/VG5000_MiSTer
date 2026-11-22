// ----------------------------------------------------------------------------------
// Part of the Verilog translation of the orpigame EF9345 core, 2026.
// Copyright D. Laurens (original design). This source describes Open Hardware
// and is licensed under the CERN-OHL-S v2 (see LICENSE in this directory).
//
// ROMCHAR1: 512 x 8 character ROM expected by CharactersDecoder (the VHDL
// sources do not include it, see vdu/rom/README.md): G11 set ($000-$13F), G20 set ($140-$18F) and the G10 reconstruction mask ($190).
// 10 consecutive bytes per character (one per scan line), bit 7 = left dot,
// synchronous read (one clock of latency).
// Contents: $readmemh of INIT_FILE (one hex byte per line; default from the
// macro EF9345_ROMCHAR1_HEX, else "romchar1.hex"), e.g. made from
// the EF9345 character ROM (see rtl/ef9345/README.md).
// 2026: default file rtl/ef9345/romcharN.hex (Quartus project directory).
// ----------------------------------------------------------------------------------

`ifndef EF9345_ROMCHAR1_HEX
`define EF9345_ROMCHAR1_HEX "rtl/ef9345/romchar1.hex"
`endif

module ROMCHAR1 #(
    parameter INIT_FILE = `EF9345_ROMCHAR1_HEX
) (
    input  logic        CLK,
    input  logic [8:0]  ADDR,
    output logic [7:0]  DATA
);

logic [7:0] rom [0:511];

initial $readmemh(INIT_FILE, rom);

always_ff @(posedge CLK)
  DATA <= rom[ADDR];

endmodule
