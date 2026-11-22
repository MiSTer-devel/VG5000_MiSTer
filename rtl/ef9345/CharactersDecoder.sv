// ---------------------------------------------------------------------------------
// Author: D.LAURENS
//
// Create Date:    16:56:28 05/22/2015
// Design Name:
// Module Name:    CharactersDecoder - Behavioral
// Project Name:
// Target Devices: XC7A35T
// Tool versions:
// Description:
//
// Dependencies: EF9345_Defines.vhd
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// ---------------------------------------------------------------------------------
//
//                           Character Decoder
//
// Purpose : Reconstruct the original EF9345 character sets and associated
//           display attributes.
//
// Character ROM Organization
//
// AROMChar1 contains the G0 and G10 character sets:
//
//                 +-------------------+
//                 | G0 Character Set  |  $000 - $4FF
//                 +-------------------+
//                 | G10 Character Set |  $500 - $77F
//                 +-------------------+
//
// AROMChar0 contains the G11 and G20 character sets:
//
//                 +-------------------+
//                 | G11 Character Set |  $000 - $13F
//                 +-------------------+
//                 | G20 Character Set |  $140 - $18F
//                 +-------------------+
//                 | Reconstruction    |  $190
//                 | Mask              |
//                 +-------------------+
//
// Character Reconstruction
//
// The G0 character set is entirely stored in AROMChar1.
//
// The G10 character set is only partially stored (characters $40-$FF).
// Characters below $40 are reconstructed by combining the corresponding
// character from the upper part of the G10 table with the reconstruction mask
// stored in AROMChar0.
//
// Example:
//
//     Character($FF) AND Mask
//
// The G11 character set is fully defined.
// The G21 character set is reconstructed from the G0 character set.
// The G20 character set is fully defined.
//
// Character Attributes
//
// Negative            : Video inversion.
// Blink               : Enables character blinking when PATBlink is active.
// Conceal             : Hides the character when PATConceal is active.
// UserBichCharacter   : Selects a user-defined bichrome character.
// UserQuadriCharacter : Selects a user-defined quadrichrome character.
// Index               : Character line index used to address the ROMs.
// DisplayCursor       : Enables hardware cursor display.
// CursorMode          : Selects the hardware cursor rendering mode.
//
// ---------------------------------------------------------------------------------
// Copyright D. Laurens. This source describes Open Hardware and is licensed under
// the CERN-OHL-S v2 (see LICENSE in this directory).
//
// Verilog translation of CharactersDecoder.vhd, 2026.
// 2026 (VG5000 MiSTer): FIXES[0] (G10 mask address adder), FIXES[1] (G11 test);
// the blink counter runs from the 48 MHz clock (FREQUENCE_BASE).
// Translation note: the VHDL tests "CharacterSet=11" (a 3-bit vector against the
// integer 11), which is never true; the translation keeps it (G10 only) and
// FIXES[1] corrects it.
// ---------------------------------------------------------------------------------

module CharactersDecoder #(
    parameter logic [8:0] FIXES = 9'h1FF   // see EF9345.sv
) (
    input  logic       Clk,
    input  logic       Negative,
    input  logic       FlashReverse,  // 80 columns: reverse attribute (flash phase only)
    input  logic       Blink,
    input  logic       PATBlink,
    input  logic       PATConceal,
    input  logic       DisplayCursor, // Cursor validation
    input  logic [1:0] CursorMode,    // Cursor type
    input  logic [3:0] Index,
    input  logic [7:0] R1,
    input  logic [7:0] R2,
    output logic       Complement,    // Request for additional character information
    output logic       Overlay,       // Video overlay
    output logic [7:0] Qout
);

/* verilator lint_off UNUSEDPARAM */
`include "EF9345_Defines.svh"
/* verilator lint_on UNUSEDPARAM */

// ALIAS DECLARATIONS
wire       Conceal       = R2[2];   // Masking
wire [2:0] CharSet       = R2[6:4];
wire [1:0] UserCharacter = R2[7:6]; // User character
wire       CursorFixed   = CursorMode[0]; // Cursor type: complements or underlines the character
wire       CursorFlash   = CursorMode[1]; // Blinking or steady cursor.

// SIGNAL DECLARATIONS
logic        Test;
logic        Mask            = 1'b0;
logic        Blinking;                     // Cursor blinking, 1 Hz
logic        BlinkingChar;                 // Character flashing, 0.5 Hz
logic        Underlinecursor = 1'b0;       // Cursor underline
logic [1:0]  CntBlinking     = 2'b00;
logic [2:0]  CharacterSet;
logic [4:0]  AROM0           = 5'd0;
logic [6:0]  AROM1           = 7'd0;
logic [7:0]  QOut_GraphicSet;
logic [7:0]  QOut_Underline  = 8'h00;
logic [7:0]  Qout_Attribut;
logic [8:0]  ROM0Address     = 9'd0;
logic [10:0] ROM1Address     = 11'd0;

// Blinking counter
logic [25:0] Cnt = 26'd0;   // integer range 0 to BlinkTempo

// ROM0/ROM1
logic [7:0]  QChar0;
logic [7:0]  QChar1;
logic [8:0]  AROMChar0;
logic [10:0] AROMChar1;

// -----------------------------------------------
//
// G0/G10 ROM (ROMCHAR0, 2 KB, block RAM)
// G0 : $000-$4FF, G10 : $500-$77F
// Addressed by AROMChar1, output QChar1
//
// -----------------------------------------------
ROMCHAR0 ROM0 (
    .CLK  (Clk),
    .DO   (QChar1),
    .ADDR (AROMChar1)
);

// -----------------------------------------------
//
// G11/G20/G21 ROM (ROMCHAR1, 512 bytes, logic)
// G11 : $000-$13F, G20 accents : $140,
// G21 accents : $168, G10 mask : $190
// Addressed by AROMChar0, output QChar0
//
// -----------------------------------------------
ROMCHAR1 ROM1 (
    .CLK  (Clk),
    .DATA (QChar0),
    .ADDR (AROMChar0)
);

// ------------------------------------------------------------------------
// Blinking clocks.
// Cnt counts 0.5 s; CntBlinking is incremented at each 0.5 s step.
//   CntBlinking(0) : period 1 s (1 Hz)   -> flashing cursor
//   CntBlinking(1) : period 2 s (0.5 Hz) -> flashing characters
// Datasheet: cursor flash 50% / 50%, 1 Hz (MAT(4:6), page 13);
// character flash 50% / 50%, 0.5 Hz (PAT6, pages 13, 24 and 31).
// ------------------------------------------------------------------------
always_ff @(posedge Clk) begin
  if (Cnt == 26'(BlinkTempo)) begin
    CntBlinking <= CntBlinking + 2'd1;
    Cnt <= 26'd0;
  end else begin
    Cnt <= Cnt + 26'd1;
  end
end

// Blinking: 1 Hz for the cursor, 0.5 Hz for the characters
assign Blinking     = CntBlinking[0];
// 2026 fix (FIXES[7]): the real chip toggles the character flash when the
// cursor flash turns on (fabio-d animated references: phase order
// (cursor, char) = 00, 11, 01, 10), not when it turns off.
assign BlinkingChar = CntBlinking[1] ^ (FIXES[7] & CntBlinking[0]);

// --------------
//    MUX
// --------------
always_comb begin
  case (CharSet)
    G0, G0Underlining:   CharacterSet = G0;
    G10:                 CharacterSet = G10;
    G11:                 CharacterSet = G11;
    G20, G20Underlining: CharacterSet = G20;
    default:             CharacterSet = G21;
  endcase
end

// --------------------------------------------------------------------------------------
// ROM address generation.
// Multiplication by 10 is performed using shifts as follows:
// 10 x AROM0 = AROM0 x (2 + 8) = 2 x AROM0 + 8 x AROM0
// Multiplication by 2 is implemented as a one-bit left shift,
// while multiplication by 8 is implemented as a three-bit left shift.
// Since the FPGA provides dedicated hardware multipliers, they are used instead.
// AROMChar0 <= (AROM0&"000")+(AROM0&'0')+ Index + ROM0Address;
// AROMChar1 <= (AROM1&"000")+(AROM1&'0')+ Index + ROM1Address;
// (A and B are 5-bit sums whose carry replaces bit 4 of the base address,
//  exactly as in the VHDL.)
// --------------------------------------------------------------------------------------
always_comb begin : rom_address
  logic [4:0]  A;
  logic [4:0]  B;
  logic [8:0]  Cv;
  logic [10:0] D;
  A  = {1'b0, Index} + {1'b0, ROM0Address[3:0]};
  B  = {1'b0, Index} + {1'b0, ROM1Address[3:0]};

  Cv = {ROM0Address[8:5], A};
  D  = {ROM1Address[10:5], B};
  // 2026 fix (FIXES[0]): a real adder. The slice sum above replaces bit 4 of
  // the base, so the G10 mask at $190 was read at $180 (G21 accent bytes).
  // All the other bases have bit 4 = 0, for which both forms are equal.
  if (FIXES[0]) begin
    Cv = ROM0Address + {5'b00000, Index};
    D  = ROM1Address + {7'b0000000, Index};
  end

  AROMChar0 = (AROM0 * 4'b1010) + Cv;
  AROMChar1 = (AROM1 * 4'b1010) + D;
end

// -----------------------------------------------------------
// Calculation of addresses for the various character sets
// Management of underline and masking flags
// -----------------------------------------------------------
always_ff @(posedge Clk) begin : address_flags
  logic Cursor;
  // --------------------------------------------
  // Calculation of the ROM1 character offset
  // --------------------------------------------
  if (CharacterSet == G0)
    AROM1 <= R1[6:0];
  // Address G20/G21
  else if (CharacterSet == G20 || CharacterSet == G21)
    AROM1 <= {2'b00, R1[4:0]} + 7'b1100000;
  else
    AROM1 <= {1'b0, R1[5:0]};

  // --------------------------------------------
  // Calculation of the ROM0 character offset
  // --------------------------------------------
  if (CharacterSet == G11)
    AROM0 <= R1[4:0];
  // Address G20/G21
  else if (CharacterSet == G20 || CharacterSet == G21)
    AROM0 <= {3'b000, R1[6:5]};
  // Address 0x000h
  else
    AROM0 <= '0;

  // --------------------------------------
  // Address of the ROM1 character set
  // --------------------------------------
  // Address 0x500h
  // (VHDL: "CharacterSet=G10 or CharacterSet=11"; the second term is never true.)
  // 2026 fix (FIXES[1]): the VHDL "CharacterSet=11" meant G11. No visible
  // effect: the G11 dots come from ROMCHAR1 (QChar0) only.
  if (CharacterSet == G10 || (FIXES[1] && CharacterSet == G11))
    ROM1Address <= G10_Address;
  // Address 0x000h
  else
    ROM1Address <= G0_Address;

  // --------------------------------------
  // Address of the ROM0 character set
  // --------------------------------------
  // Address 0x000h
  case (CharacterSet)
    G11:     ROM0Address <= G11_Address;
    // Address 0x140h
    G20:     ROM0Address <= G20_LAddress;
    // Address 0x168h
    G21:     ROM0Address <= G20_HAddress;
    // Address 0x190h
    default: ROM0Address <= Mask_Address;
  endcase

  // G10 graphic mask
  if (R1[6] == INACTIF && CharacterSet == G10)
    Mask <= ACTIF;
  // No mask
  else
    Mask <= INACTIF;

  // Underline
  if (Test == ACTIF)
    QOut_Underline <= 8'hFF; // Underline activated
  // Underline disabled
  else
    QOut_Underline <= QOut_GraphicSet;

  // Blinking enabled
  if (CursorFlash == CursorBlinking && Blinking == ACTIF)
    Cursor = ACTIF;
  // No blinking
  else if (CursorFlash == CursorNotBlink)
    Cursor = ACTIF;
  // No underline
  else
    Cursor = INACTIF;

  // Cursor must be displayed ?
  if (DisplayCursor == ACTIF) begin
    // Cursor complement signal
    if (CursorFixed == FixedComplemented)
      Complement <= Cursor;
    // Cursor underline
    // (2026 fix, FIXES[7]: the real chip shows no underlined cursor on the
    //  semigraphic G10 / G11 sets, fabio-d test_cursor_underline_40columns)
    else if (Index == 4'b1001 && !(FIXES[7] && CharSet[2:1] == 2'b01))
      Underlinecursor <= Cursor;
    else
      Underlinecursor <= INACTIF;
  end else begin
    Underlinecursor <= INACTIF;
    Complement      <= INACTIF;
  end
end

assign Test = (CharSet[0] == ACTIF && (Index == 4'b1001) &&
               (CharSet == G0Underlining || CharSet == G20Underlining || CharSet == G21Underlining)) ? ACTIF : INACTIF;

// Underline
assign Qout_Attribut = (Negative == ACTIF) ? ~QOut_Underline : QOut_Underline;

// ---------------------------------------------------------------------------------
// Qout is the formatted character output signal.
// When UserQuadriCharacter is asserted, the user-defined character is a
// quadrichrome character, and no formatting is applied.
// When UnderlineCursor is asserted, the cursor is displayed with an underline.
// ---------------------------------------------------------------------------------
//
// Conceal and flash (datasheet, "Displaying the Attributes", pages 24 and 31)
//  - the attributes are processed in this order: underline, flash, conceal,
//    colour, negative. Conceal and the off phase of flash force the background
//    on the whole cell BEFORE the negative attribute: a concealed or blanked
//    negative character is therefore a negative space, i.e. a full cell of its
//    foreground colour (x"FF" here, since negative inverts the dots);
//  - "the phase depends on the negative attribute": negative characters flash
//    in opposite phase to normal ones.
// In 80 columns, reverse is done by swapping the colours in R3 (Negative = 0),
// so the blank line is already right; only the phase needs FlashReverse.
// ---------------------------------------------------------------------------------
always_comb begin : attribute_mux
  logic [7:0] BlankLine;
  logic       FlashOff;
  // Blank line, processed before the negative attribute
  if (Negative == ACTIF)
    BlankLine = ~BlinkMask40;
  else
    BlankLine = BlinkMask40;

  // Off phase of a flashing character: opposite phase for negative characters
  FlashOff = BlinkingChar ^ (Negative | FlashReverse);

  // 2026 fix (FIXES[7]): the underlined cursor takes the colour of the
  // displayed dots, i.e. the background colour on a negative character
  // (fabio-d test_40columns_cursor).
  if (Underlinecursor == ACTIF)
    Qout = (FIXES[7] && Negative == ACTIF) ? 8'h00 : 8'hFF;                // Q(0)
  else if (Conceal == ACTIF && PATConceal == ACTIF)
    Qout = BlankLine;                                                      // Q(1)
  else if (UserCharacter == Quadrichrome)
    Qout = {R1[1], R1[0], R1[3], R1[2], R1[5], R1[4], R1[7], R1[6]};       // Q(2)
  // Character flashing at 0.5 Hz (the cursor uses Blinking, 1 Hz)
  else if (PATBlink == ACTIF && Blink == ACTIF && FlashOff == ACTIF)
    Qout = BlankLine;                                                      // Q(3)
  else
    Qout = Qout_Attribut;                                                  // Q(4)
end

// Video overlay
assign Overlay = R2[0];

// -----------------------------------------------------------------------------------
// Generation of punctuation characters and the G10 graphic character set.
// When UserBichCharacter is asserted, the user-defined character is interpreted
// as a bichrome character.
// -----------------------------------------------------------------------------------
always_comb begin : graphic_set_mux
  logic [1:0] G2x;
  G2x = {CharacterSet[2], CharacterSet[0]};

  if (UserCharacter == Bichrome)
    QOut_GraphicSet = {R1[0], R1[1], R1[2], R1[3], R1[4], R1[5], R1[6], R1[7]}; // Q(0)
  else if (CharacterSet == G10 && Mask == ACTIF)
    QOut_GraphicSet = QChar0 & QChar1;                                         // Q(1)
  // G20 and G21 compare
  else if (G2x == 2'b10)
    QOut_GraphicSet = QChar0 | QChar1;                                         // Q(2)
  else if (CharSet == G11)
    QOut_GraphicSet = QChar0;                                                  // Q(3)
  else
    QOut_GraphicSet = QChar1;                                                  // Q(4)
end

endmodule
