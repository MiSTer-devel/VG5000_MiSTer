// ---------------------------------------------------------------------------------
// Author: D.LAURENS
//
// Create Date:    14:49:30 01/07/2026
// Design Name:
// Module Name:    EF9345 - Behavioral
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
// Top level of the Thomson EF9345 VDU.
//
// ---------------------------------------------------------------------------------
//
//                     EF9345 FPGA Compatible Video Controller
//
// Module : EF9345
//
// Description : Top-level module of the EF9345 FPGA implementation.
//
// Compatibility : Functionally compatible with the Thomson EF9345.
//
// Notes : Internal implementation has been redesigned for FPGA technology
//         while preserving the externally observable behaviour expected by
//         the original hardware and software.
//
// ---------------------------------------------------------------------------------
//
//                            Module Architecture
//
// Controller         : Video controller and command execution engine.
//
// CharactersDecoder  : Character generator and character decoding logic.
//
// VRAM               : Dual-port VRAM subsystem composed of multiple RAM blocks.
//
// ---------------------------------------------------------------------------------
//
//                           Clock and Video Timing
//
// Input Clock : 130 MHz
//
// The video output supports the XGA standard (1024 x 768 @ 60 Hz).
//
// Pixel Clock      : 65 MHz
// Horizontal Rate  : 48.36 kHz
// Vertical Rate    : 60 Hz
//
// The 130 MHz system clock was selected to satisfy the internal processing
// requirements imposed by the VGA video pipeline.
//
// This video resolution was chosen to preserve the original EF9345 display
// aspect ratio while supporting both display modes.
//
//  - 40 x 25 characters (320 x 250 pixels)
//  - 80 x 25 characters (480 x 250 pixels)
//
// The pixel ratio between both modes is 3:2 because characters are encoded
// using 8 pixels in 40-column mode and 6 pixels in 80-column mode.
//
// To obtain identical display widths, character cells are internally expanded
// to:
//
//   - 24 pixels (3 x 8) in 40-column mode
//   - 12 pixels (2 x 6) in 80-column mode
//
// Both modes therefore produce an active display width of 960 pixels.
//
// Each character is displayed over 30 scanlines (3 x 10), resulting in an
// active display height of 750 pixels.
//
// ---------------------------------------------------------------------------------
//
//                              VRAM Organization
//
// The implemented VRAM size is 8 KB instead of the 16 KB available in the
// original EF9345.
//
// The memory subsystem is implemented using two 4 KB dual-port RAM blocks.
//
// This architecture allows simultaneous video fetches and CPU read/write
// accesses without memory arbitration conflicts.
//
// ---------------------------------------------------------------------------------
//
//                         Supported EF9345 Commands
//
// IND : Command execution
// VSM : Vertical blanking masking
// VRM : S2 bit 2 reflects the VBLANK state
// CLF : 24-bit page fill using registers R1, R2 and R3
// CLG : 16-bit page fill using registers R1, R2 and R3
// KRF : 24-bit / 16-bit character read or write (40-column mode)
// KRL : 12-bit character read or write (80-column mode)
// OCT : Private memory byte read or write
// MVB : Single buffer copy
// MVD : Double buffer copy
// MVT : Triple buffer copy
// INY : Increment Y register
// NOP : No operation
//
// ---------------------------------------------------------------------------------
// Copyright D. Laurens. This source describes Open Hardware and is licensed under
// the CERN-OHL-S v2 (see LICENSE in this directory).
//
// Verilog translation of EF9345.vhd, 2026.
// 2026 (VG5000 MiSTer): FIXES parameter, Reset input, PixTick/HVisible/VVisible
// outputs for the 15 kHz wrapper ef9345_vg5k.sv; the clock is now 48 MHz.
// ---------------------------------------------------------------------------------

// FIXES (2026, one bit per fix of the orpigame original, all on by default;
// each bit can be built on its own to attribute a screen difference to one
// fix):
//   bit 0  G10 mask read at $190 (real adder for the ROM address; was $180)
//   bit 1  CharacterSet = G11 test (VHDL "CharacterSet=11", never true)
//   bit 2  pixel order after a double-width character (RVBPixel underflow)
//   bit 3  status S2 = vertical sync (0 for the 2 VS lines, datasheet and real
//          chip), VRM aliases $9C-$9F
//   bit 4  no MAT display filter (the margin follows MAT at once, as the chip)
//   bit 5  KRC command and 80-column short codes (TGS7:6 = 10)
//   bit 6  command decoding as measured on real chips (fabio-d suite): X
//          auto-increment wraps from 39/47/55/63 only, INY $B0-$BF, KRC/KRL
//          bits 2:1 and OCT bit 1 don't care, move a = 00/11 aliases, R1 =
//          last byte moved
//   bit 7  display as measured on real chips: double-height slices one TV
//          line later, character flash toggling with the cursor-flash rise
//   bit 8  a non-stop move (MVB/MVD/MVT, a = 1x) ends by itself after one page
//          (32 x 40 words), as in MAME and dcvg5k; software relies on it
//          (Daniel Coulom's "Demo Mode 80 Colonnes" polls Busy after $FA). The
//          datasheet says "never stops until aborted"; the real-chip suite
//          aborts after 0.1 s and cannot tell.
module EF9345 #(
    parameter logic [8:0] FIXES = 9'h1FF
) (
    input  logic       Clk,
    input  logic       Reset,
    input  logic       CS_n,
    input  logic       DS,
    input  logic       AS,
    input  logic       RW,
    input  logic [7:0] Data_In,
    output logic       BLUE,
    output logic       GREEN,
    output logic       RED,
    output logic       H_Sync_n,
    output logic       V_Sync_n,
    output logic       Overlay,
    output logic       HP,
    output logic       PixTick,
    output logic       HVisible,
    output logic       VVisible,
    output logic [7:0] Data_Out
);

logic        EnableLine;
logic [7:0]  VRAMWe;
logic [9:0]  VRAM_Ad_MCU;
logic [9:0]  VRAM_Ad_DISP;
logic [63:0] QinMCUVRAM;
logic [63:0] QOutDisplay;
logic [63:0] QOutMCUVRAM;

EF9345_Controller #(.FIXES(FIXES)) XLXI_5 (
    .Clk          (Clk),
    .Reset        (Reset),
    .EnableLine   (EnableLine),
    .CS_n         (CS_n),
    .Data_In      (Data_In),
    .Data_Out     (Data_Out),
    .DS           (DS),
    .AS           (AS),
    .RW           (RW),
    .QInDisplay   (QOutDisplay),
    .QInMCUVRAM   (QOutMCUVRAM),
    .blue         (BLUE),
    .green        (GREEN),
    .h_sync_n     (H_Sync_n),
    .red          (RED),
    .Overlay      (Overlay),
    .VRAM_Ad_MCU  (VRAM_Ad_MCU),
    .VRAM_Ad_DISP (VRAM_Ad_DISP),
    .QOutMCUVRAM  (QinMCUVRAM),
    .VRAMWe       (VRAMWe),
    .v_sync_n     (V_Sync_n),
    .HP           (HP),
    .PixTick      (PixTick),
    .HVisible     (HVisible),
    .VVisible     (VVisible)
);

VRAM XLXI_8 (
    .Clk          (Clk),
    .EnableLine   (EnableLine),
    .WE           (VRAMWe),
    .VRAM_Ad_DISP (VRAM_Ad_DISP),
    .VRAM_Ad_MCU  (VRAM_Ad_MCU),
    .QinMCUVRAM   (QinMCUVRAM),
    .QOutMCUVRAM  (QOutMCUVRAM),
    .QOutDisplay  (QOutDisplay)
);

endmodule
