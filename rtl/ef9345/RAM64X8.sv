// ----------------------------------------------------------------------------------
// Author: Denis Laurens
//
// Create Date:    11:30:44 03/03/2015
// Design Name:
// Module Name:    RAM64X8 - Behavioral
// Project Name:
// Target Devices: XC7A35T
// Tool versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
// ----------------------------------------------------------------------------------
// Copyright D. Laurens. This source describes Open Hardware and is licensed under
// the CERN-OHL-S v2 (see LICENSE in this directory).
//
// Verilog translation of RAM64X8.vhd, 2026.
// 2026: integer loop variable (Quartus 17).
// The four Xilinx RAM64X2S primitives (64 x 2 distributed RAM: synchronous
// write, asynchronous read, INIT = 0) are one inferred 64 x 8 array with the
// same behaviour. Quartus maps an asynchronous-read RAM this small to MLAB
// cells or logic.
// ----------------------------------------------------------------------------------

module RAM64X8 (
    input  logic       Clk,
    input  logic       We,
    input  logic [7:0] Q_in,
    input  logic [5:0] Address,
    output logic [7:0] Q_Out
);

// --------------------------------------
//
// Internal RAM64X2S Memory
//
// --------------------------------------
logic [7:0] mem [0:63];

integer i;
initial begin
  for (i = 0; i < 64; i = i + 1)
    mem[i] = 8'h00;
end

always_ff @(posedge Clk) begin
  if (We)
    mem[Address] <= Q_in;
end

assign Q_Out = mem[Address];

endmodule
