// ---------------------------------------------------------------------------------
// Author: D.LAURENS
//
// Create Date:    14:49:30 05/19/2015
// Design Name:
// Module Name:    RAMVIDEO - Behavioral
// Project Name:
// Target Devices: XC7A35T
// Tool versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 1.00 - File Created
// Additional Comments:
//
//
// Purpose : Implements the dual-port video memory subsystem used by the
//           EF9345 display engine.
//
// Memory Organization
//
// The video memory is implemented using two True Dual-Port (TDP) RAM blocks,
// each organized as 32-bit x 1K words, together with three 64-bit distributed
// RAM blocks.
//
// This organization provides storage for 26 display lines of 40 characters
// per VRAM bank.
//
// Special Line Selection
//
// The EnableLine signal selects the 26th display line whenever register R4
// or R6 contains an odd value less than 8.
//
// Port Organization
//
// Port A : Host CPU interface.
//          Supports both read and write accesses initiated by the host CPU.
//
// Port B : Video pipeline interface.
//          Continuously supplies display data to the video pipeline for display
//          generation.
//
// ---------------------------------------------------------------------------------
// Copyright D. Laurens. This source describes Open Hardware and is licensed under
// the CERN-OHL-S v2 (see LICENSE in this directory).
//
// Verilog translation of VRAM.vhd, 2026.
// The two Xilinx BRAM_TDP_MACRO (36Kb, 32 x 1024, WRITE_FIRST, DO_REG = 0) are
// replaced by EF9345_BRAM_TDP (inferred RAM, same behaviour, see that file);
// RAM64X8 (RAM64X2S primitives) by an inferred 64 x 8 RAM.
// ---------------------------------------------------------------------------------

module VRAM (
    input  logic        Clk,
    input  logic        EnableLine,   // Validation of line 1 of the even RAM blocks
    input  logic [7:0]  WE,
    input  logic [9:0]  VRAM_Ad_MCU,  // VRAM MCU address
    input  logic [9:0]  VRAM_Ad_DISP, // Display VRAM address
    input  logic [63:0] QinMCUVRAM,   // MCU input data
    output logic [63:0] QOutMCUVRAM,  // MCU output data
    output logic [63:0] QOutDisplay   // Video display data
);

// Signal declaration
logic [2:0]  WE_A;
logic [2:0]  WE_B;
logic [3:0]  WE_VRAM0;
logic [23:0] QOutMCUVRAM_B;
logic [31:0] QOutMCUVRAM_A;

// Alias
wire [3:0] WE_VRAM1 = WE[7:4];

// The eight 1 KB video VRAM banks are implemented using two 32-bit x 1024
// True Dual-Port (TDP) RAM blocks.
//
// Each RAM block provides independent write-enable control for each byte,
// allowing four independent 8-bit x 1 KB VRAM banks to be stored and
// accessed simultaneously.
//
// The RAM macro configuration derived from the table above is as follows:
// ---------------------------------------------------------------------------
//        19-36    |  "36Kb"   |     1024    |   10-bit   |    4-bit
// ---------------------------------------------------------------------------
// MACRO 32bits x 1024
EF9345_BRAM_TDP VIDEORAM0 (
    .CLK   (Clk),
    .DOA   (QOutMCUVRAM_A[31:0]),
    .DOB   (QOutDisplay[31:0]),
    .ADDRA (VRAM_Ad_MCU[9:0]),
    .ADDRB (VRAM_Ad_DISP[9:0]),
    .DIA   (QinMCUVRAM[31:0]),
    .WEA   (WE_VRAM0)
);

// MACRO 32bits x 1024
EF9345_BRAM_TDP VIDEORAM1 (
    .CLK   (Clk),
    .DOA   (QOutMCUVRAM[63:32]),
    .DOB   (QOutDisplay[63:32]),
    .ADDRA (VRAM_Ad_MCU[9:0]),
    .ADDRB (VRAM_Ad_DISP[9:0]),
    .DIA   (QinMCUVRAM[63:32]),
    .WEA   (WE_VRAM1)
);

// --------------------------------------
//
//          VRAM LINE BANK0
//
// --------------------------------------
RAM64X8 RAMLINE0 (
    .Clk     (Clk),
    .We      (WE_B[0]),
    .Address (VRAM_Ad_MCU[5:0]),
    .Q_in    (QinMCUVRAM[7:0]),
    .Q_Out   (QOutMCUVRAM_B[7:0])
);

// --------------------------------------
//
//          VRAM LINE BANK1
//
// --------------------------------------
RAM64X8 RAMLINE1 (
    .Clk     (Clk),
    .We      (WE_B[1]),
    .Address (VRAM_Ad_MCU[5:0]),
    .Q_in    (QinMCUVRAM[15:8]),
    .Q_Out   (QOutMCUVRAM_B[15:8])
);

// --------------------------------------
//
//          VRAM LINE BANK2
//
// --------------------------------------
RAM64X8 RAMLINE2 (
    .Clk     (Clk),
    .We      (WE_B[2]),
    .Address (VRAM_Ad_MCU[5:0]),
    .Q_in    (QinMCUVRAM[23:16]),
    .Q_Out   (QOutMCUVRAM_B[23:16])
);

// VRAM0 WE
assign WE_VRAM0 = {WE[3], WE_A};

// Dual-port VRAM write signal MUX
always_comb begin
  WE_A = '0;
  WE_B = '0;

  if (EnableLine == 1'b0)
    WE_A = WE[2:0];
  // Line VRAM write signal MUX
  else
    WE_B = WE[2:0];
end

// Delay process
always_ff @(posedge Clk) begin
  // VRAM0 data
  QOutMCUVRAM[31:24] <= QOutMCUVRAM_A[31:24];

  // Output data MUX for the dual-port VRAM and the line VRAM
  if (EnableLine == 1'b0)
    QOutMCUVRAM[23:0] <= QOutMCUVRAM_A[23:0];
  else
    QOutMCUVRAM[23:0] <= QOutMCUVRAM_B;
end

endmodule
