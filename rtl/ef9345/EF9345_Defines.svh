// ----------------------------------------------------------------------------------
// Author: Denis Laurens
//
// Create Date:    16:02:07 05/28/2015
// Design Name:
// Module Name:    EF9345_Defines - Behavioral
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
// Verilog translation of EF9345_Defines.vhd, 2026.
// 2026 (VG5000 MiSTer): the XGA timing constants are replaced by the native
// EF9345 625-line raster at 48 MHz / 24 MHz grid (datasheet figure 7).
// The VHDL package becomes a list of localparams; this file is `included inside
// the body of every module that needs the constants (no include guard, on
// purpose: each module gets its own copy of the localparams).
// ----------------------------------------------------------------------------------

// Active-high signals
localparam logic ACTIF   = 1'b1;
localparam logic INACTIF = 1'b0;

// FPGA base frequency in hertz.
// 2026 (VG5000 MiSTer): the core runs from clk_sys = 48 MHz (was 130 MHz).
// The pixel grid is Clk/2 = 24 MHz (was 65 MHz): a 40-column dot lasts 3 grid
// pixels (8 MHz dot clock) and an 80-column dot 2 grid pixels (12 MHz), so a
// character cell is exactly 1 us (40 col.) or 0.5 us (80 col.) as on the chip.
localparam int MEGAHERTZ      = 1000000;
localparam int FREQUENCE_BASE = 48*MEGAHERTZ;

// Maximum number of characters per line in 40-column mode
localparam int NbrOfHorChar40 = 40;
// Maximum number of characters per line in 80-column mode
localparam int NbrOfHorChar80 = 80;
// Maximum number of lines: 25.
localparam int NbrOfVerChar   = 25;
// Line zoom. 2026: 1 (native 625-line raster, one TV line per character
// slice); the XGA original used 3. The display routines count the repeated
// lines with DblLine (0 .. NbrOfLineZoom-1).
localparam int NbrOfLineZoom  = 1;
localparam logic [1:0] LastZoomLine = 2'(NbrOfLineZoom-1);
// Number of grid pixels per line for the 40/80-column display modes:
// 40 x 8 x 3 = 80 x 6 x 2 = 960 grid pixels = 40 us (bulk, datasheet figure 7).
localparam int NbrOfPixelPerLine = 6*NbrOfHorChar80*2;

// Native EF9345 raster (datasheet figure 7, CLK = 12 MHz, TGS0 = 0, non
// interlaced), in 24 MHz grid pixels and TV lines:
//   line  64 us = 1536 : H blanking 10 us (sync 4.5 us at its start),
//                        margin 6 us, bulk 40 us, margin 6 us, H blanking 2 us
//   frame 312 lines    : blanking 25 (VS = first 2 lines), margin 16,
//                        page 250 (service row + 24 rows of 10 lines),
//                        margin 18, blanking 3
// h_Count / v_Count origin = first margin pixel / first margin line.
localparam int MarginWidth   = 144;  // 6 us
localparam int TopMargin     = 16;
localparam int BottomMargin  = 18;

// Position of the graphics window (bulk) on the screen.
localparam int XTopWindow    = 2+MarginWidth-1;                     // 145
localparam int YTopWindow    = TopMargin;                           // 16
localparam int XBottomWindow = NbrOfPixelPerLine+XTopWindow-1;      // 1104
localparam int YBottomWindow = NbrOfLineZoom*10*NbrOfVerChar+YTopWindow; // 266

// Anticipation of the first character of a line
localparam int XStarDisplay  = XTopWindow-8;

// Position of the screen border (margin).
// XTopBorder must keep XStarDisplay - XTopBorder = 4 (mod 6): StartCntPulse
// starts with the margin, and the phase of the 40/80-column dot counters
// (CntPulse40 mod 6 clocks, CntPulse80 mod 4) at the first-character prefetch
// depends on it. The left margin is therefore 143 grid pixels (5.96 us).
localparam int XTopBorder    = XTopWindow-MarginWidth;              // 1
localparam int XBottomBorder = XBottomWindow+MarginWidth;           // 1248
localparam int YTopBorder    = YTopWindow-TopMargin;                // 0
localparam int YBottomBorder = YBottomWindow+BottomMargin;          // 284

// Service line, S flag of the PAT register
localparam int SMaintenanceLine = YTopWindow+(NbrOfLineZoom*10);

// Constants for the Ub and Lb flags of the PAT register
localparam int LbBorderDisplay = SMaintenanceLine+(12*NbrOfLineZoom*10); // The margin covers 12+1 lines
localparam int UbBorderDisplay = YBottomWindow-(12*NbrOfLineZoom*10);    // The margin covers 12+1 lines

// Constant for bit 0 of the TGS register (number of lines per frame).
// TGS0 = 0 : 625-line standard, the bulk is displayed for 240 TV lines (24 rows).
// TGS0 = 1 : 525-line standard, the bulk is displayed for 200 TV lines (20 rows).
// With TGS0 = 1, bulk rows 21 to 24 are displayed in the margin color (the
// 262-line frame itself is not implemented: the raster stays at 312 lines).
localparam int TGS0BorderDisplay = SMaintenanceLine+(20*NbrOfLineZoom*10);

localparam int h_Count_Max = 1535;   // 64 us
localparam int v_Count_Max = 311;    // 312 lines, 50.08 Hz

// Synchronization signals (h_Count / v_Count values, active low outputs)
//   H: margin ends at XBottomBorder; front porch 2 us, sync 4.5 us, back porch
//      5.5 us up to the next XTopBorder.
//   V: margin ends at YBottomBorder; 3 blank lines, VS 2 lines, 23 blank lines.
localparam int H_VIDEO       = XBottomBorder;           // first blanked pixel
localparam int H_FRONT_PORCH = 48;                      // 2 us
localparam int H_HORIZ_SYNC  = H_FRONT_PORCH + 108;     // 4.5 us sync
localparam int H_BACK_PORCH  = h_Count_Max+1-H_VIDEO+XTopBorder;
localparam int V_VIDEO       = YBottomBorder;           // first blanked line
localparam int V_FRONT_PORCH = 3;
localparam int V_HORIZ_SYNC  = V_FRONT_PORCH + 2;       // 2 lines
localparam int V_BACK_PORCH  = v_Count_Max+1-V_VIDEO+YTopBorder;

// ASN
localparam logic [3:0] ASN_2X = 4'b0010;

// Register address definitions
localparam logic [2:0] AddressR0 = 3'b000;
localparam logic [2:0] AddressR1 = 3'b001;
localparam logic [2:0] AddressR2 = 3'b010;
localparam logic [2:0] AddressR3 = 3'b011;
localparam logic [2:0] AddressR4 = 3'b100;
localparam logic [2:0] AddressR5 = 3'b101;
localparam logic [2:0] AddressR6 = 3'b110;
localparam logic [2:0] AddressR7 = 3'b111;

// Command definitions
localparam logic [2:0] CMD_TGS = 3'b001;
localparam logic [2:0] CMD_MAT = 3'b010;
localparam logic [2:0] CMD_PAT = 3'b011;
localparam logic [2:0] CMD_DOR = 3'b100;
localparam logic [2:0] CMD_ROR = 3'b111;

localparam logic [3:0] CMD_NOP = 4'b1001;
localparam logic [5:0] CMD_CLF = 6'b000001;
localparam logic [7:0] CMD_INY = 8'b10110000;

localparam logic       CLF      = INACTIF;
localparam logic       KRF      = INACTIF;
localparam logic       KRG      = ACTIF;
localparam logic [1:0] CMD_VSM  = 2'b10;
localparam logic [1:0] CMD_VRM  = 2'b01;
localparam logic [1:0] CMD_MVB  = 2'b01;
localparam logic [1:0] CMD_MVD  = 2'b10;
localparam logic [1:0] CMD_COPY = 2'b11;
localparam logic [3:0] CMD_IND  = 4'b1000;
localparam logic [3:0] CMD_KRF  = 4'b0000;
localparam logic [3:0] CMD_KRL  = 4'b0101;
localparam logic [3:0] CMD_OCT  = 4'b0011;

localparam logic [2:0] CMD_CLF_i  = 3'b000;
localparam logic [2:0] CMD_KRF_i  = 3'b001;
localparam logic [2:0] CMD_KRL_i  = 3'b010;
localparam logic [2:0] CMD_OCT_i  = 3'b011;
localparam logic [2:0] CMD_COPY_i = 3'b100;

// The copy commands (MVB/MVD/MVT) under the no-stop condition run until a new
// command aborts them (datasheet, page 36): there is no transfer count limit.

// Operating mode definitions
localparam logic [2:0] LongMode40  = 3'b000;
localparam logic [2:0] VarMode40   = 3'b001;
localparam logic [2:0] ShortMode40 = 3'b100;
localparam logic [2:0] LongMode80  = 3'b011;
localparam logic [2:0] ShortMode80 = 3'b010;

// 16KB
localparam logic [3:0] Block0 = 4'b0000;
localparam logic [3:0] Block1 = 4'b0001;
localparam logic [3:0] Block2 = 4'b0010;
localparam logic [3:0] Block3 = 4'b0011;
localparam logic [3:0] Block4 = 4'b0100;
localparam logic [3:0] Block5 = 4'b0101;
localparam logic [3:0] Block6 = 4'b0110;
localparam logic [3:0] Block7 = 4'b0111;

localparam logic PointerR6R7 = INACTIF;
localparam logic PointerR4R5 = ACTIF;

localparam logic EvenRow     = INACTIF;
localparam logic OddRow      = ACTIF;

localparam logic EvenColumn  = INACTIF;
localparam logic OddColumn   = ACTIF;

// Value to subtract to return to the start of the RAM line
localparam logic [9:0]  FirstVRAMAdress       = 10'b0000100111;
// Multiplication constant 40
localparam logic [5:0]  forty                 = 6'b101000;
// Constant 999
localparam logic [9:0]  NineHundredNinetyNine = 10'b1111100111;
// Null address value
localparam logic [9:0]  ZeroValue             = 10'b0000000000;
// Null value of the character double-height array
localparam logic [39:0] ZeroValueFlgTab       = 40'h0;

// Timing for the EF9345 character blinking
localparam int BlinkTempo = (FREQUENCE_BASE-1)/2; // About 1s

// CONSTANT DECLARATIONS FOR THE CharactersDecoder.vhd MODULE
localparam logic [2:0] G0             = 3'b000;
localparam logic [2:0] G0Underlining  = 3'b001;
localparam logic [2:0] G10            = 3'b010;
localparam logic [2:0] G11            = 3'b011;
localparam logic [2:0] G20            = 3'b100;
localparam logic [2:0] G20Underlining = 3'b101;
localparam logic [2:0] G21            = 3'b110;
localparam logic [2:0] G21Underlining = 3'b111;
localparam logic       Underlining    = ACTIF;

// Blink masking
localparam logic [7:0] BlinkMask40 = 8'h00;
localparam logic [7:0] BlinkMask80 = 8'hFF;

// Cursor type
localparam logic FixedComplemented = INACTIF; // Steady cursor, complements the character
localparam logic FixedUnderlined   = ACTIF;   // Steady cursor, underlines the character
localparam logic CursorBlinking    = ACTIF;   // Blinking cursor, complements the character
localparam logic CursorNotBlink    = INACTIF; // Steady cursor

localparam logic [10:0] G0_Address   = 11'b00000000000; // $000
localparam logic [10:0] G10_Address  = 11'b10100000000; // $500
localparam logic [8:0]  G11_Address  = 9'b000000000;    // $000;
localparam logic [8:0]  G20_LAddress = 9'b101000000;    // $140
localparam logic [8:0]  G20_HAddress = 9'b101101000;    // $168
localparam logic [8:0]  Mask_Address = 9'b110010000;    // $190

localparam logic [1:0] Bichrome     = 2'b10;
localparam logic [1:0] Quadrichrome = 2'b11;

localparam logic       UserG0Bichrome  = 1'b0;
localparam logic [1:0] UserG10Bichrome = 2'b10;
localparam logic [1:0] UserG11Bichrome = 2'b11;

localparam logic HighResolution  = 1'b0; // High-resolution quadrichrome mode
localparam logic LowResolution   = 1'b1; // Low-resolution quadrichrome mode

localparam logic FourFirstSlices = 1'b0; // Use of the first 4 buffer addresses
localparam logic FourLasttSlices = 1'b1; // Use of the last 4 buffer addresses

// Color definitions for quadrichrome characters
localparam logic [1:0] C0 = 2'b00;
localparam logic [1:0] C1 = 2'b01;
localparam logic [1:0] C2 = 2'b10;
localparam logic [1:0] C3 = 2'b11;

// Color definitions
localparam logic [2:0] BlackColour   = 3'b000;
localparam logic [2:0] RedColour     = 3'b001;
localparam logic [2:0] GreenColour   = 3'b010;
localparam logic [2:0] YellowColour  = 3'b011;
localparam logic [2:0] BlueColour    = 3'b100;
localparam logic [2:0] MagentaColour = 3'b101;
localparam logic [2:0] CyanColour    = 3'b110;
localparam logic [2:0] WhiteColour   = 3'b111;

// MCU Type
localparam logic Motorola = 1'b0;
localparam logic Intel    = 1'b1;
