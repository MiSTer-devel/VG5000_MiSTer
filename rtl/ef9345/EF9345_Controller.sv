// ----------------------------------------------------------------------------------
// Author: Denis Laurens
//
// Create Date:    14:49:30 01/07/2026
// Design Name:
// Module Name:    EF9345_Controller - Behavioral
// Project Name:
// Target Devices: XC7A35T
// Tool versions:
// Description:
//
// Dependencies: EF9345_Defines.vhd
//
// Revision:
// Revision 1.00 - File Created
// Input frequency:
// The module's operating frequency is 130 MHz.
//
// Display:
// The supported video format is XGA mode, providing a screen resolution of 1024x768 at 60 Hz.
// The pixel frequency is 65 MHz, with a line frequency of 48.36 kHz and a VBLANK rate of 60 Hz.
// These video specifications are what necessitated such a high input frequency of 130 MHz.
// This video resolution was chosen based on the display modes supported by the EF9345.
// Two modes are available: the first is a 40x25 character mode (corresponding to 320x250 pixels),
// and the second is an 80x25 character mode (corresponding to 480x250 pixels).
// The pixel ratio between the two is 3/2; in 40-character mode, a character is encoded using 8 bits,
// whereas in 80-character mode, it is encoded using 6 bits.
// To ensure consistency between the two modes, a character is encoded using 24 bits (3x8) for the 40-character mode,
// and 12 bits (2x6) for the 80-character mode.
// This results in a resolution of 960 pixels (3x320) for the 40-character mode, and 960 pixels (80x12) for the 80-character mode.
// To maintain a uniform size, each character is displayed across 30 lines (3x10 lines),
// resulting in a vertical resolution of 750 pixels. --
// VRAM
// The addressable area is 8 KB, compared to 16 KB in the actual circuit.
// Two 4 KB dual-port RAM modules are used to simultaneously handle display (via continuous reading of three RAM banks) and updates (via read/write commands).
// With this type of architecture, sharing the VRAM presents no issues.
//
// The emulated routines are as follows:
// IND, command processing
// VSM, VBLANK masking
// VRM, where bit 2 of status register S2 is determined by the VBLANK signal
// CLF, 24-bit page fill using values pointed to by registers R1, R2, and R3
// CLG, 16-bit page fill using values pointed to by registers R1, R2, and R3
// KRF, 24-bit and 16-bit, display or read a character in 40-column mode
// KRL, 12-bit, display or read a character in 80-column mode
// OCT, write or read a byte in private memory
// MVB, single-pass copy from one buffer to another
// MVD, double-pass copy from one buffer to another
// MVT, triple-pass copy from one buffer to another
// INY, increment Y
// NOP, no operation
// ----------------------------------------------------------------------------------
// Copyright D. Laurens. This source describes Open Hardware and is licensed under
// the CERN-OHL-S v2 (see LICENSE in this directory).
//
// Verilog translation of EF9345_Controller.vhd, 2026.
// 2026 (VG5000 MiSTer) changes, each marked "2026" in the code: 48 MHz clock and
// native 15 kHz raster (line zoom 1, NbrOfLineZoom-based row counters, MAT
// filter rescaled), blanking outputs, synchronous reset of the bus / command
// engine, and the fixes selected by FIXES (see EF9345.sv).
// Translation notes:
//  * One always_ff per VHDL clocked process, statements kept in the VHDL order:
//    a later nonblocking assignment overrides an earlier one, as the last signal
//    assignment of a VHDL process does.
//  * VHDL signals without an initial value ('U' in simulation) start at 0 here,
//    as the FPGA flip-flops do and as GHDL --synth/Vivado produce.
//  * Integer signals use the width of their VHDL range; "x - 1" on 0 wraps as in
//    the synthesised VHDL (VHDL simulation would stop on the range error).
//  * The process variables of the video controller (TypeModeTab, EnableTypeTab_A,
//    ModeTypeTab_A, UserEnableTypeTab_A) keep their value from one clock to the
//    next: they are registers here, updated through blocking copies.
//  * The enum MAE_LIST is a localparam encoding; its values are internal only.
// ----------------------------------------------------------------------------------

module EF9345_Controller #(
    parameter logic [8:0] FIXES = 9'h1FF   // see EF9345.sv
) (
    input  logic        Clk,           // 48 MHz main clock (2026; 130 MHz in the original)
    input  logic        Reset,         // 2026: synchronous reset of the bus / command engine
    input  logic        DS,            // Data Strobe, ACTIVE high
    input  logic        AS,            // Address Strobe, ACTIVE on the falling edge
    input  logic        CS_n,          // Chip Select, ACTIVE low
    input  logic        RW,            // Read/Write
    input  logic [7:0]  Data_In,       // Input data bus
    input  logic [63:0] QInDisplay,    // VRAM data for display
    input  logic [63:0] QInMCUVRAM,    // VRAM data for refresh
    output logic        h_sync_n,      // Horizontal synchronization signal
    output logic        v_sync_n,      // Vertical synchronization signal
    output logic        red,           // RED signal
    output logic        green,         // GREEN signal
    output logic        blue,          // BLUE signal
    output logic        EnableLine,    // 26th line enable
    output logic        Overlay,       // Video overlay
    output logic        HP,
    output logic        PixTick,       // 2026: 24 MHz grid pixel enable (RGB valid after this clock)
    output logic        HVisible,      // 2026: margin + bulk, horizontally (same pipeline stage as RGB)
    output logic        VVisible,      // 2026: margin + page lines (same pipeline stage as RGB)
    output logic [7:0]  VRAMWe,        // Memory write
    output logic [7:0]  Data_Out,      // Output data bus
    output logic [9:0]  VRAM_Ad_MCU,   // VRAM write address
    output logic [9:0]  VRAM_Ad_DISP,  // VRAM read address
    output logic [63:0] QOutMCUVRAM    // VRAM data for refresh
);

/* verilator lint_off UNUSEDPARAM */
`include "EF9345_Defines.svh"
/* verilator lint_on UNUSEDPARAM */

// ------------- FSM DECLARATION --------------------------
// Command processing state machine
localparam logic [3:0] STEP0    = 4'd0,
                       STEP1    = 4'd1,
                       STEP2    = 4'd2,
                       STEP3    = 4'd3,
                       STEP4    = 4'd4,
                       STEP5    = 4'd5,
                       INCR6R7  = 4'd6,
                       INCR5    = 4'd7,
                       INCR7    = 4'd8,
                       KRLINCR7 = 4'd9,
                       EXIT_CMD = 4'd10;
logic [3:0] MAE = STEP0;
logic [10:0] CopyCount = 11'd0;   // 2026 (FIXES[8]): words moved by a non-stop move
logic [7:1]  PendMask  = 7'd0;    // 2026 (FIXES[8]): R1-R7 writes held during a move
logic [7:0]  PendVal [1:7];       // (only read where PendMask is set)

// ------------ SIGNAL DECLARATIONS ------------------------

// Clock divided
logic       ClkDiv2_0 = 1'b1;
logic       ClkDiv2_1 = 1'b1;
logic       ClkDiv2_2 = 1'b1;
logic [1:0] ClkDiv2_3 = 2'b11;

// Counter for the 40-column and 80-column modes
logic [2:0] CntPulse40 = 3'd0;   // integer range 0 to 5
logic [1:0] CntPulse80 = 2'd0;   // integer range 0 to 3

// Column counter start signal
logic StartCntPulse = INACTIF;

// Horizontal counter
logic [10:0] h_Count = 11'(h_Count_Max);
// Vertical counter
logic        v_CountReached = 1'b0;
logic [9:0]  v_Count   = 10'(v_Count_Max);
logic [9:0]  v_Count_i = 10'(v_Count_Max);

// Stop of the display routines
logic StopCurrentLine40 = INACTIF; // Stop for the 40-column mode
logic StopCurrentLine80 = INACTIF; // Stop for the 80-column mode

// Display enable signals
logic       UpdatePixel  = INACTIF; // Line pixel update
logic       LimitDisplay = INACTIF; // Display valid window
logic       StartDisplay = INACTIF; // Display start
logic       Border       = INACTIF; // Window border
logic [1:0] STATE        = 2'd0;    // FSM for the first character initialization

// Character definition signals
logic [7:0] R3         = 8'h00; // Signal pointing to the character color
logic [7:0] CurrentR2  = 8'h00; // Current R2 data
logic [7:0] QR1        = 8'h00; // VRAM output data
logic [7:0] QR2        = 8'h00; // VRAM output data
logic [7:0] QR3        = 8'h00; // VRAM output data
logic [7:0] LatchR1    = 8'h00; // Latched R1 data
logic [7:0] LatchR2    = 8'h00; // Latched R2 data
logic [7:0] LatchR3    = 8'h00; // Latched R3 data
logic [7:0] LatchQuser = 8'h00; // Latched user data
// (CurrentR3 of the VHDL is declared but never used.)

logic [3:0] PtrBQ = 4'd0;          // integer range 0 to 11

// Character double-width and double-height signal
logic        UserEnableTypeTabFlg = INACTIF;         // Copy flag for the current UserEnableTypeTab value
logic        EnableTypeTabFlg     = INACTIF;         // Copy flag for the current EnableTypeTab value
logic [39:0] ModeTypeTab          = ZeroValueFlgTab; // Array of doubling modes: height or height+width
logic [39:0] EnableTypeTab        = ZeroValueFlgTab; // Mode enable array
logic [39:0] UserEnableTypeTab    = ZeroValueFlgTab; // Bichrome/quadrichrome mode enable array
logic [5:0]  CntCaracters         = 6'd0; // Character counter (integer range 0 to 39)
logic [5:0]  CntCaracters_A       = 6'd0; // Character counter
logic [5:0]  CntCaracters_B       = 6'd0; // Character counter
logic [5:0]  TabCntCntCaracters   = 6'd0; // Character counter for double-height/width management

// Variables of the video controller process (state kept between clocks)
logic [2:0] TypeModeTab         = 3'b000;
logic       EnableTypeTab_A     = 1'b0;
logic       ModeTypeTab_A       = 1'b0;
logic       UserEnableTypeTab_A = 1'b0;

// Current line counter, 40-column and 80-column modes.
logic [3:0] LineCounter = 4'h0; // Current line counter

// 40-column mode line counter for character double-height management.
logic [3:0] LineCounterDH         = 4'h0;  // Character double-height line counter, register R2
logic [3:0] LineCounterDblMAT     = 4'h0;  // Line counter for double-height-doubled characters, register MAT
logic [5:0] USerLineCounter       = 6'd0;  // Current line counter for user-defined characters
logic [5:0] USerLineCounterDH     = 6'd0;  // Character double-height line counter, register R2, for user-defined characters
logic [5:0] USerLineCounterDblMAT = 6'd0;  // Line counter for double-height-doubled characters, register MAT, for user-defined characters
logic [5:0] UserLowLineCounter    = 6'd0;  // Line counter for low-resolution quadrichrome characters

// 80-column mode line counter for character double-height management.
logic [3:0] LineCounterDH80 = 4'h0; // Double-height line counter via the MAT register
// 2026 fix (FIXES[7]): the real EF9345 shows R2 double-height slices one TV
// line later than LineCounterDH (fabio-d test_double_size_render: line k of an
// upper half shows slice (k-1)/2, line 0 slice 0; the first line of a lower
// half repeats slice 4). MAT7 global doubling is not delayed.
logic [3:0] DispLCDH = 4'h0, DispLCDblMAT = 4'h0;
logic [3:0] UserLine80      = 4'h0; // Line counter for user-defined characters

logic [4:0] CurrentLine = 5'd0; // Points to the current line
logic [2:0] PixelCnt    = 3'd7; // Current pixel counter
logic [2:0] RVBPixel    = 3'd7; // Pixel counter for RGB indexing

// Line doubling signal
logic [1:0] DblLine = 2'd0;     // integer range 0 to 2

// Internal address redirection signal
logic [9:0] Int_DISP_VRAMAddr = ZeroValue;
logic [9:0] DISP_VRAMAddr     = ZeroValue;

// Registers
logic [7:0] R0Register    = 8'h00;
logic [7:0] R1Register    = 8'h00;
logic [7:0] R2Register    = 8'h00;
logic [7:0] R3Register    = 8'h00;
logic [7:0] R4Register    = 8'h00;
logic [7:0] R5Register    = 8'h00;
logic [7:0] R6Register    = 8'h00;
logic [7:0] R7Register    = 8'h00;
logic [7:0] StatuRegister = 8'h00;

logic [7:0] TGSRegister   = 8'h00; // 0 at power-up: 625 lines, 24 bulk rows
logic [7:0] MATRegister   = 8'h00;
// Display copy of MAT: takes a new MAT value only once it has been stable for
// MAT_STABLE_CYCLES (see the MAT display filter process).
localparam int MAT_STABLE_CYCLES = 3023;      // 3024 Clk periods = 63 us at 48 MHz (was 8191 at 130 MHz)
logic [7:0]  MATDisplay   = 8'h00;
logic [7:0]  MATLast      = 8'h00;
logic [12:0] MATStable    = 13'd0;  // integer range 0 to MAT_STABLE_CYCLES
logic [7:0]  PATRegister  = 8'h00;
logic [7:0]  DORRegister  = 8'h00;
logic [7:0]  RORRegister  = 8'h00;

// VRAM read pointer from registers R6 and R7
logic [5:0]  X = 6'd0;
logic [4:0]  Y = 5'd0;
logic [10:0] MainPointer;

// VRAM read pointer from registers R4 and R5
logic [5:0]  Xp = 6'd0;
logic [4:0]  Yp = 5'd0;
logic [10:0] AuxiliaryPointer;

// Current pointer
logic       CurrentPointer = INACTIF;
logic [3:0] BlockPointer   = 4'b0000;

// Command execution flag
logic       ExecCommand    = INACTIF;
logic       CmdExecEnabled = INACTIF;
logic       CmdExec        = INACTIF;
logic [7:0] CurrentCommand = 8'h00;

// Copy command recognition flag
logic MOVE_BUFFER;

// VSYNC read flag
logic VsyncRd = 1'b0;

// MCU VRAM read/write signal
logic [2:0] RW_VRAM = 3'b000;

// VRAM refresh/update signal
logic       UpdateVRAM   = INACTIF;
logic [4:0] RowLine      = 5'd0;   // XGA line within the displayed row (30 lines = 10 TV lines), integer range 0 to 29
logic       CmdSuspend;            // Row boundary window (row buffer fetch, last/first TV line)
logic       BusyHold     = 1'b0;   // Busy held at 1 for the uP until the end of the window
logic       UpdateVRAM_i = INACTIF;

// VRAM data signals
logic [7:0] SaveMCUData1 = 8'h00;
logic [7:0] SaveMCUData2 = 8'h00;
logic [7:0] SaveMCUData3 = 8'h00;

logic [7:0] UserCaracter;

logic [7:0] ReadMCUData1;
logic [7:0] ReadMCUData2;
logic [7:0] ReadMCUData3;

// Signals for the CLEAR PAGE commands
logic [7:0] R1ClearCmd = 8'h00;
logic [7:0] R2ClearCmd = 8'h00;
logic [7:0] R3ClearCmd = 8'h00;

// Enable signal for the Ub, Lb and S flags of the PAT register
logic FlagUbLb = INACTIF;
logic FlagS    = INACTIF;

// Line selection signal for character double-height/width
logic DblHeight  = INACTIF; // Double-height signal, bit 1 of register R2
logic DblWidth   = INACTIF; // Double width active
logic EnDblWidth = INACTIF; // DblWidth flag present
logic ToggleLine = INACTIF;
// First line of a 10-line half (slice 0 of an upper half starts here, the lower
// half shows the last upper slice, 4, once more).
wire HalfStartDH    = (ToggleLine == INACTIF) && (LineCounterDH == 4'h0 || LineCounterDH == 4'h5);
wire [3:0] LCDH_v     = (FIXES[7] && !HalfStartDH)   ? DispLCDH     : LineCounterDH;
// R2 double height under MAT7 (4 TV lines per slice): delayed by one doubled
// line (2 TV lines), the same delay in the doubled domain.
wire HalfStartDblM  = (LineCounterDblMAT == 4'h0 || LineCounterDblMAT == 4'h5) && !LineCounterDH[0];
wire [3:0] LCDblMAT_v = (FIXES[7] && !HalfStartDblM) ? DispLCDblMAT : LineCounterDblMAT;
wire [3:0] LCDH80_v   = LineCounterDH80;     // (80 columns: MAT7 only, not delayed)

logic DH = 1'b0; // Double-height flag, DH bit of the MAT register

// Address for user-defined characters
logic UserDblHeight    = INACTIF; // Double-height signal for bichrome characters
logic ReadQUser        = INACTIF; // User data fetch enable
logic EnableBich       = INACTIF; // Bichrome character enable
logic EnableQuad       = INACTIF; // Quadrichrome character enable
logic EnableBandQ      = INACTIF; // Quadrichrome character enable
logic LatchEnableQuad  = INACTIF; // Quadrichrome character enable
logic LatchEnableQuad0 = INACTIF; // Quadrichrome character enable

logic [5:0] UserOffset = 6'b000000;  // Character slice offset

// YOR register roll-over management signals
logic [4:0]  InternalYOR = 5'b00000;      // Internal YOR register
logic [10:0] YORAddress  = 11'b00000000000; // First line to display after the service line

// User address signals
logic [10:0] UserAddress    = 11'd0; // Character address
logic [4:0]  UserAddressA   = 5'd0;
logic [10:0] UserAddressB   = 11'd0;
logic [10:0] UserAddressMul = 11'd0;

// MSB and LSB signals for user-defined characters, 80-column mode
logic MSBUser80 = INACTIF;
logic LSBUser80 = INACTIF;

// Quadrichrome mode color
logic [2:0] Color0;
logic [2:0] Color1;
logic [2:0] Color2;
logic [2:0] Color3;
logic [2:0] QuadriColour0 = 3'b000;
logic [2:0] QuadriColour1 = 3'b000;
logic [2:0] QuadriColour2 = 3'b000;
logic [2:0] QuadriColour3 = 3'b000;

// Active color in 80-column mode
logic [2:0] ActiveColour80 = 3'b000;

// Display mode
logic       ParityRow = EvenRow; // Column parity signal, 80-column mode
logic [2:0] DisplayMode;

// RGB signals
logic [7:0] Qgreen = 8'h00;
logic [7:0] Qblue  = 8'h00;
logic [7:0] QRed   = 8'h00;

// Latch of the complement signal
logic LatchComplement = INACTIF;

// Signals used for the short 16-bit mode, MODE32/40 columns
logic [7:0] QA = 8'h00; // Intermediate signal QR1
logic [7:0] QB = 8'h00; // Intermediate signal QR2
logic [7:0] QC = 8'h00; // Intermediate signal QR3

logic m_latchm = INACTIF; // CONCEAL bit latch
logic m_latchi = INACTIF; // INSERT bit latch
logic m_latchu = INACTIF; // UNDERLINE bit latch

logic [2:0] m_latchc0 = BlackColour; // Character color latch

// Enable signals for the 16-bit to 24-bit conversion routine
logic RazLatchs         = 1'b0;
logic Convert16To24bits = 1'b0;

// Management signals
logic       LCS             = 1'b0;
logic       ASN             = 1'b0;
logic       fAS;
logic       fDS;
logic       fRW;
logic       fData           = 1'b0;
logic       RW_Register     = 1'b0;
logic       MCUType         = 1'b0;
logic [1:0] Sr_AS           = 2'b00;
logic [1:0] Sr_DS           = 2'b00;
logic [1:0] Sr_RW           = 2'b00;
logic [2:0] AddressRegister = 3'b000; // Circuit address bus
logic [2:0] CurrentCmd_i    = 3'b000;
wire MoveBusy;   // 2026 (FIXES[8]): a move is running
logic [7:0] AD_Latched      = 8'h00;

// Attribute module
logic       Blink        = 1'b0;
logic       Complement;
logic       DisplayCursor;
logic       Negative     = 1'b0;
logic       FlashReverse = INACTIF; // 80 columns: reverse attribute, used for the flash phase
logic       PATBlink;
logic       PATConceal;
logic       OverlayIn;
logic [7:0] QIns         = 8'hFF;   // I value of each dot of the current character
logic [7:0] QROMFg;                 // Foreground dots before the negative attribute (40 columns)
logic [7:0] Fg80User;               // Lit blocks of an 80-column semi-graphic character
logic       Insert80     = INACTIF; // Insert attribute of the current 80-column character
logic       WindowPixel;            // A character dot is being displayed
logic       InsPixel;               // I value of the current dot
logic       ForceBlack;             // RGB forced to black (boxing, inlay)
logic [1:0] CursorMode;
logic [3:0] Index        = 4'h0;
logic [7:0] QROM;
logic [7:0] R1           = 8'h00;
logic [7:0] R2           = 8'h00;

//
logic [2:0] bp;   // First bank of a character code
logic [2:0] bp1;  // Second bank: block + 1 modulo 4 in the district
logic [2:0] bp2;  // Third bank : block + 2 modulo 4 in the district

logic [7:0] VRAMInMCU_array_1 [0:7];
logic [7:0] VRAMDisplay_array [0:7]; // The 8 banks read by the display port
logic [7:0] VRAMOutMCU_array  [0:7];

// ------------ ALIAS DECLARATIONS -----------------------
wire [4:0] R6_Bit4_0     = R6Register[4:0];   // R6 on the first 5 bits
wire [5:0] R7_Bit5_0     = R7Register[5:0];   // R7 on the first 6 bits
// 2026 fix (FIXES[6]): X auto-increment wraps to 0 only from 39, 47, 55 or 63
// (real chip, fabio-d test_memory_access_incr_x); X = 40..46 counts up. The
// original wrapped from any X >= 39.
wire XWrapM = FIXES[6] ? (R7_Bit5_0 == 6'd39 || (R7_Bit5_0[2:0] == 3'b111 && R7_Bit5_0[5:3] >= 3'b101))
                       : (R7_Bit5_0 >= 6'b100111);
wire [4:0] R4_Bit4_0     = R4Register[4:0];   // R4 on the first 5 bits
wire [5:0] R5_Bit5_0     = R5Register[5:0];   // R5 on the first 6 bits
wire XWrapA = FIXES[6] ? (R5_Bit5_0 == 6'd39 || (R5_Bit5_0[2:0] == 3'b111 && R5_Bit5_0[5:3] >= 3'b101))
                       : (R5_Bit5_0 >= 6'b100111);
wire [3:0] Command       = R0Register[7:4];   // Main command pointer
wire [2:0] INDIRECT_CMD  = R0Register[2:0];   // Special command pointer
wire [2:0] ROR_Int       = RORRegister[7:5];  // ROR pointer for display
wire [4:0] YOR           = RORRegister[4:0];  // YOR pointer for display
wire       ParityColumn  = R7Register[7];     // Column selection in 80-column mode
wire       ReadWrite     = R0Register[3];     // R0 register Read/Write
wire       IncPointer    = CurrentCommand[0]; // Increment request for pointers R6/R7 or R4/R5
// Stop condition for the MVB, MVD and MVT commands. 2026 fix (FIXES[6]): on the
// real chip a = 00 is an alias of 01 (stop) and a = 11 of 10 (no stop)
// (fabio-d test_memory_move_stop / _nonstop): the stop bit is a(1) = 0.
wire       StopCondition = FIXES[6] ? ~CurrentCommand[1] : CurrentCommand[0];
wire [3:0] CurrentCmd    = CurrentCommand[7:4];
wire       ReadWriteCmd  = CurrentCommand[3]; // R0 register Read/Write
wire       UserCharacter80 = LatchR1[7];      // User-defined character in 80-column mode

// R3 and LatchR3 signal aliases
wire [2:0] CaracterColour   = R3[6:4]; // Character color
wire [2:0] BackGroundColour = R3[2:0]; // Background color
wire       F                = LatchR3[3]; // Blink
wire       N                = LatchR3[7]; // Reverse video

// Color aliases for the quadrichrome mode
wire CBlack   = R3[0];
wire CRed     = R3[1];
wire CGreen   = R3[2];
wire CYellow  = R3[3];
wire CBlue    = R3[4];
wire CMagenta = R3[5];
wire CCyan    = R3[6];
wire CWhite   = R3[7];

// Color aliases for the 80-column mode
wire EvenReverseColour = LatchR3[7];
wire EvenBlink         = LatchR3[6];  // Even column blink
wire EvenUnderline     = LatchR3[5];  // Even column underline
wire EvenActiveCoulour = LatchR3[4];
wire OddReverseColour  = LatchR3[3];
wire OddBlink          = LatchR3[2];  // Odd column blink
wire OddUnderline      = LatchR3[1];  // Odd column underline
wire OddActiveCoulour  = LatchR3[0];

// CurrentR2 double width/height signal aliases
logic UpdateTabDH  = INACTIF;         // Height/width array update signal
wire  DoubleHeight = CurrentR2[1];    // Height doubling
wire  DoubleWidth  = CurrentR2[3];    // Width doubling

// QR2/LatchR2 signal aliases
wire       BichDblHeight       = QR2[1];      // Height doubling
wire [1:0] UserCaracterType    = QR2[7:6];    // Bichrome/quadrichrome type
wire       QuadriResolution    = LatchR2[1];  // Quadrichrome character resolution type
wire       SubsetIndex         = LatchR2[2];  // Buffer addressing index, low-resolution quadrichrome mode
wire       UserBichromeType    = LatchR2[5];  // User-defined character type G0 or G10/G11
wire       LatchR2DblHeight    = LatchR2[1];  // Double height
wire       LatchR2USerCaracter = LatchR2[7];  // User-defined character
wire [2:0] SetNumber           = LatchR2[5:3]; // Block to use for the quadrichrome mode
wire [1:0] UserG10G11Type      = LatchR2[5:4]; // User-defined character type G10 or G11, bichrome mode

// PAT register signal aliases
wire EnableBlinking = PATRegister[6]; // Blink enable
wire EnableMask     = PATRegister[3]; // Character conceal enable
wire Lb             = PATRegister[2]; // Lower screen display
wire Ub             = PATRegister[1]; // Upper screen display
wire S              = PATRegister[0]; // Service line display

// TGS register signal aliases
logic Mode80   = 1'b0; // TGSRegister(7) : Mode40/Mode80
logic Mode80_i = 1'b0; // TGSRegister(7) : Mode40/Mode80

// MAT register signal aliases (display side: filtered copy MATDisplay)
wire       Internal_DH  = MATDisplay[7];   // Character double-height enable
wire       C            = MATDisplay[6];   // Cursor enable
wire [1:0] CM           = MATDisplay[5:4]; // Cursor type definition
wire [2:0] MarginColour = MATDisplay[2:0]; // Margin color
wire       MarginInsert = MATDisplay[3];   // Insert value of the margin (IM)
wire [1:0] InsertMode   = PATRegister[5:4]; // Insert mode

// DOR register signal aliases
wire [2:0] FirstColour    = DORRegister[2:0]; // 1st active color for 80-column mode characters
wire [2:0] SecondColour   = DORRegister[6:4]; // 2nd active color for 80-column mode characters
wire [3:0] AddressUserG0  = DORRegister[3:0]; // Block number for user-defined G0 characters
wire [1:0] AddressUserG10 = DORRegister[5:4]; // Block number for user-defined G10/G11 characters
// Z3Q = DORRegister(7): first block number for quadrichrome characters (unused, 8 KB)

// System status signal aliases (StatuRegister bits)
//   7 Busy, 6 AL, 5 LXm, 4 LXa, 3 MSBr1, 2 Sync
wire Busy = StatuRegister[7]; // Flag Busy

// Keep only bits 6..2, which form the buffer number of a user-defined character
wire [4:0] UserNumberBuffer = LatchR1[6:2];
// Keep only bits 1..0, which form the character number within the buffer of a user-defined character
wire [1:0] UserNumberChar   = LatchR1[1:0];


// ---------------------------------------------------------------------------------
// Insert output I (datasheet, page 14, table 3, and page 15, PAT(5:4)).
// In the active area, I depends on the insert mode and on the insert attribute i
// of each character:
//   PAT5 PAT4 = 1 1 : active area mark   I = 1
//               1 0 : character mark     I = i
//               0 1 : boxing             I = i, RGB forced to black when I = 0
//               0 0 : inlay              I = i on the foreground dots only, RGB
//                                        forced to black when I = 0
// In the margin, the disabled service row or half-screens and the retrace,
// I = IM = MAT3. InsertVector gives the I value of the 8 dots of a character.
// ---------------------------------------------------------------------------------
function automatic logic [7:0] InsertVector(input logic [1:0] Mode, input logic i,
                                            input logic [7:0] Fg);
  case (Mode)
    // Active area mark
    2'b11:   InsertVector = 8'hFF;
    // Inlay: only the foreground dots of an inserted character
    2'b00:   InsertVector = {8{i}} & Fg;
    // Character mark and boxing: the whole window of an inserted character
    default: InsertVector = {8{i}};
  endcase
endfunction

// Character attributes
CharactersDecoder #(.FIXES(FIXES)) XLXI_10 (
    .Clk           (Clk),
    .Blink         (Blink),
    .CursorMode    (CursorMode),
    .DisplayCursor (DisplayCursor),
    .Index         (Index),
    .Negative      (Negative),
    .FlashReverse  (FlashReverse),
    .PATBlink      (PATBlink),
    .PATConceal    (PATConceal),
    .Overlay       (OverlayIn),
    .R1            (R1),
    .R2            (R2),
    .Complement    (Complement),
    .Qout          (QROM)
);

// ---------------------------------------------------------
// Horizontal and vertical line counter management
// for video display generation
// ---------------------------------------------------------
always_ff @(posedge Clk) begin
  // 65 MHz clock
  if (ClkDiv2_0 == ACTIF) begin
    // Increment the h_Count counter
    if (h_Count == 11'(h_Count_Max)) begin
      h_Count <= 11'd0;
      // Increment the v_Count counter
      if (v_Count_i == 10'(v_Count_Max)) begin
        v_CountReached <= ACTIF;
        v_Count        <= 10'd0;
        v_Count_i      <= 10'd0;
      end else begin
        v_CountReached <= INACTIF;
        v_Count        <= v_Count + 10'd1;
        v_Count_i      <= v_Count_i + 10'd1;
      end
    end else begin
      // Increment the h_Count counter
      h_Count <= h_Count + 11'd1;
    end
  end
  ClkDiv2_0 <= ~ClkDiv2_0;
end

// ---------------------------------------------------------------------------------
// Command execution suspension (datasheet, page 35, note 4): "the execution of
// any command (except VRM, VSM) is suspended during the last and first TV line
// of an active row", while the EF9345 fetches the row buffers (104 us). A row
// is 10 TV lines, i.e. 30 XGA lines here, from YTopWindow (service row, then
// rows of 30 lines): the window covers lines 27 to 29 and 0 to 2 of each row.
// A command started (or running) in this window keeps the Busy bit read by the
// uP at 1 until its end (BusyHold): some programs poll Busy to synchronize with
// the rows (Mission Omega, VG5000). The command itself runs at once: freezing
// its steps broke the VG5000 ROM, which does not always test Busy (15/11).
// ---------------------------------------------------------------------------------
always_ff @(posedge Clk) begin
  if ((ClkDiv2_0 == ACTIF) && (h_Count == 11'(h_Count_Max))) begin
    // New line: v_Count_i + 1, or 0 after the last line of the frame
    if (v_Count_i == 10'(YTopWindow-1))
      RowLine <= 5'd0;
    else if (RowLine == 5'(NbrOfLineZoom*10-1))
      RowLine <= 5'd0;
    else
      RowLine <= RowLine + 5'd1;
  end
end

assign CmdSuspend = (v_Count >= 10'(YTopWindow) && v_Count < 10'(YBottomWindow) &&
                     (RowLine < 5'(NbrOfLineZoom) || RowLine > 5'(NbrOfLineZoom*9-1))) ? ACTIF : INACTIF;

// ---------------------------------------------------------------------------------
// MAT display filter (deliberate difference from the EF9345, 15/11).
// The VG5000 ROM writes MAT twice per screen refresh with a stale R1: at $0056
// the IND MAT command is started before R1 is loaded (about 16 us), and at
// $0164 R1 holds the character under the cursor (about 18 us). The XGA screen
// shows the whole margin, and the margin colour then flashes on one line, left
// and right of the page; no frame interrupt timing hides both cases. The
// display therefore uses MATDisplay, which takes a new MAT value only once it
// has been stable for 63 us. MAT read back by IND is still MATRegister.
// ---------------------------------------------------------------------------------
always_ff @(posedge Clk) begin
  if (FIXES[4]) begin
    // 2026 fix: no filter. On the 15 kHz raster the VG5000 ROM's stale MAT
    // writes fall in the vertical blanking that follows the frame interrupt,
    // as on the real machine.
    MATDisplay <= MATRegister;
  end else if (MATRegister != MATLast) begin
    MATLast   <= MATRegister;
    MATStable <= 13'd0;
  end else if (MATStable == 13'(MAT_STABLE_CYCLES)) begin
    MATDisplay <= MATLast;
  end else begin
    MATStable <= MATStable + 13'd1;
  end
end

// ----------------------------------------------------------------------------
//                        Display Control Signals
//
// Border        : Enables rendering of the display window border.
//
// StartDisplay  : Marks the beginning of the active display area and
//                 initializes rendering of the first character.
//
// LimitDisplay  : Remains asserted during the 960 active display pixels,
//                 enabling the rendering of a complete display line.
//
// UpdateVRAM    : Enables VRAM read and write accesses required to update
//                 the display contents.
//
// StartCntPulse : Resets the horizontal pixel counters used by both the
//                 40-column and 80-column display modes.
//
// UpdatePixel   : Enables generation of the current display pixel.
// ----------------------------------------------------------------------------
always_ff @(posedge Clk) begin
  // Clk65Mhz
  if (ClkDiv2_1 == ACTIF) begin
    // Signals are valid within the YTopWindow and YBottomWindow boundaries.
    if (v_Count >= 10'(YTopWindow) && v_Count < 10'(YBottomWindow)) begin
      // Enable display of the window border.
      if (h_Count >= 11'(XStarDisplay) && h_Count < 11'(XTopWindow-1)) begin
        StartDisplay  <= ACTIF;
        UpdatePixel   <= ACTIF;
        Border        <= ACTIF;
        StartCntPulse <= ACTIF;
        UpdateVRAM    <= INACTIF;
        LimitDisplay  <= INACTIF;

      // Enable display of a complete display line.
      end else if (h_Count >= 11'(XTopWindow-1) && h_Count <= 11'(XBottomWindow-1)) begin
        UpdateVRAM    <= UpdateVRAM_i; // Release the VRAM for data updates.
        StartCntPulse <= ACTIF;
        UpdatePixel   <= ACTIF;
        LimitDisplay  <= ACTIF;
        StartDisplay  <= INACTIF;
        Border        <= INACTIF;

      // Enable display of the window border.
      end else if (h_Count >= 11'(XTopBorder) && h_Count < 11'(XBottomBorder)) begin
        StartCntPulse <= ACTIF;
        Border        <= ACTIF;
        UpdateVRAM    <= ACTIF; // Release the VRAM for data updates.
        UpdatePixel   <= INACTIF;
        LimitDisplay  <= INACTIF;
        StartDisplay  <= INACTIF;

      // Disable all signals outside the active display area.
      end else begin
        StartCntPulse <= INACTIF;
        UpdatePixel   <= INACTIF;
        LimitDisplay  <= INACTIF;
        StartDisplay  <= INACTIF;
        Border        <= INACTIF;
        UpdateVRAM    <= ACTIF; // Release the VRAM for data updates.
      end

    // Enable display of the window border.
    end else if (v_Count >= 10'(YTopBorder) && v_Count < 10'(YBottomBorder) &&
                 h_Count >= 11'(XTopBorder) && h_Count < 11'(XBottomBorder)) begin
      StartCntPulse <= INACTIF;
      UpdatePixel   <= INACTIF;
      LimitDisplay  <= INACTIF;
      StartDisplay  <= INACTIF;
      Border        <= ACTIF;
      UpdateVRAM    <= ACTIF; // Release the VRAM for data updates.

    // Disable all signals outside the active display area.
    end else begin
      StartCntPulse <= INACTIF;
      UpdatePixel   <= INACTIF;
      LimitDisplay  <= INACTIF;
      StartDisplay  <= INACTIF;
      Border        <= INACTIF;
      UpdateVRAM    <= ACTIF; // Release the VRAM for data updates.
    end
  end
  ClkDiv2_1 <= ~ClkDiv2_1;
end

// 2026: blanking for the 15 kHz output. The margin (Border) and the bulk are the
// visible part of the raster; everything else is blanked (datasheet figure 7:
// R, G, B are low during the blanking intervals). Registered on the same
// 24 MHz phase as Border, so HVisible / VVisible line up with RGB.
logic HVisible_r = INACTIF;
logic VVisible_r = INACTIF;
always_ff @(posedge Clk) begin
  if (ClkDiv2_1 == ACTIF) begin
    HVisible_r <= (h_Count >= 11'(XTopBorder) && h_Count < 11'(XBottomBorder)) ? ACTIF : INACTIF;
    VVisible_r <= (v_Count >= 10'(YTopBorder) && v_Count < 10'(YBottomBorder)) ? ACTIF : INACTIF;
  end
end
assign PixTick  = ClkDiv2_1;
assign HVisible = HVisible_r;
assign VVisible = VVisible_r;

// --------------------------------------------------------------------
// Generation of half-screen enable signals and the service line
// signal based on the Ub, Lb, and S flags of the PAT register.
// --------------------------------------------------------------------
always_ff @(posedge Clk) begin
  // Clk65Mhz
  if (ClkDiv2_2 == ACTIF) begin
    // Active display area.
    if (h_Count >= 11'(XTopWindow-1) && h_Count < 11'(XBottomBorder)) begin
      // Service line management.
      // When the S flag is inactive, the service line is displayed.
      if ((v_Count >= 10'(YTopWindow) && v_Count <= 10'(SMaintenanceLine)) && (S == INACTIF))
        FlagS <= ACTIF;
      // Service line disabled.
      else
        FlagS <= INACTIF;

      // When Ub (PAT register bit 1) is inactive, the upper part of the screen
      // is displayed using the margin color.
      if (v_Count >= 10'(SMaintenanceLine) && v_Count <= 10'(LbBorderDisplay) && Ub == INACTIF)
        FlagUbLb <= ACTIF;
      // When Lb (PAT register bit 2) is inactive, the lower part of the screen
      // is displayed using the margin color.
      else if (v_Count >= 10'(UbBorderDisplay) && v_Count <= 10'(YBottomWindow) && Lb == INACTIF)
        FlagUbLb <= ACTIF;
      // When TGS0 (TGS register bit 0) is set, the 525-line standard is selected:
      // the datasheet displays the bulk for 200 TV lines instead of 240, i.e.
      // 20 rows instead of 24. The XGA timing is unchanged; bulk rows 21 to 24
      // are displayed using the margin color, like a disabled half-screen.
      // The VRAM rows behind them are still fetched, but never shown.
      else if (v_Count >= 10'(TGS0BorderDisplay) && v_Count <= 10'(YBottomWindow) && TGSRegister[0] == ACTIF)
        FlagUbLb <= ACTIF;
      // Display both the upper and lower halves of the screen.
      else
        FlagUbLb <= INACTIF;

    // Display both the upper and lower halves of the screen.
    end else begin
      FlagUbLb <= INACTIF;
      FlagS    <= INACTIF;
    end
  end
  ClkDiv2_2 <= ~ClkDiv2_2;
end

// ---------------------------------------------------------------------
// Address offset for user-defined bichrome/quadrichrome characters
// ---------------------------------------------------------------------
always_ff @(posedge Clk) begin
  if (LCDH_v < 4'h5 && QuadriResolution == LowResolution && EnableQuad == ACTIF)
    PtrBQ <= 4'd0;
  else if (LCDH_v > 4'h4 && QuadriResolution == LowResolution && EnableQuad == ACTIF)
    PtrBQ <= 4'd1;
  else if (DH == ACTIF && UserDblHeight == INACTIF && UserEnableTypeTabFlg == INACTIF && EnableBich == ACTIF)
    PtrBQ <= 4'd2;
  else if (LCDblMAT_v > 4'h4 && DH == ACTIF && UserDblHeight == ACTIF && UserEnableTypeTabFlg == INACTIF && EnableBandQ == ACTIF)
    PtrBQ <= 4'd3;
  else if (LCDblMAT_v < 4'h5 && DH == ACTIF && UserEnableTypeTabFlg == ACTIF && EnableBandQ == ACTIF)
    PtrBQ <= 4'd4;
  else if (DH == ACTIF && UserDblHeight == ACTIF && UserEnableTypeTabFlg == INACTIF && EnableBandQ == ACTIF)
    PtrBQ <= 4'd5;
  else if (DH == ACTIF && UserEnableTypeTabFlg == ACTIF && EnableBandQ == ACTIF)
    PtrBQ <= 4'd6;
  else if (LCDH_v < 4'h5 && UserDblHeight == ACTIF && UserEnableTypeTabFlg == INACTIF && EnableBich == ACTIF)
    PtrBQ <= 4'd7;
  else if (LCDH_v > 4'h4 && UserDblHeight == ACTIF && UserEnableTypeTabFlg == INACTIF && EnableBich == ACTIF)
    PtrBQ <= 4'd8;
  else if (LCDH_v < 4'h5 && UserEnableTypeTabFlg == ACTIF && EnableBich == ACTIF)
    PtrBQ <= 4'd9;
  else if (LCDH_v > 4'h4 && UserEnableTypeTabFlg == ACTIF && EnableBich == ACTIF)
    PtrBQ <= 4'd10;
  else
    PtrBQ <= 4'd11;
end

// ---------------------------------------------------------------------------------------------------
//
//                                   VIDEO CONTROLLER
//
// ---------------------------------------------------------------------------------------------------
always_ff @(posedge Clk) begin : video_controller
  // VHDL process variables (TypeModeTab, EnableTypeTab_A, ModeTypeTab_A,
  // UserEnableTypeTab_A): working copies of the registers.
  logic [2:0] TypeModeTab_v;
  logic       EnableTypeTab_A_v;
  logic       ModeTypeTab_A_v;
  logic       UserEnableTypeTab_A_v;
  TypeModeTab_v         = TypeModeTab;
  EnableTypeTab_A_v     = EnableTypeTab_A;
  ModeTypeTab_A_v       = ModeTypeTab_A;
  UserEnableTypeTab_A_v = UserEnableTypeTab_A;

  // Delay the UserEnableTypeTab vector bit by one clock cycle
  // to increase the propagation time available for the logic below.
  // The flag of character c is stored at index c-1 (it is read while character c
  // is prefetched, i.e. during cell c-1). Column 0 has no cell -1: its flag is
  // stored at index 39 (never used by another column, column 39 uses index 38)
  // and read during StartDisplay, when the first character of the line is fetched.
  if (StartDisplay == ACTIF)
    UserEnableTypeTabFlg <= UserEnableTypeTab[39];
  else
    UserEnableTypeTabFlg <= UserEnableTypeTab[CntCaracters];

  // Delay the EnableTypeTab vector bit by one clock cycle
  // to increase the propagation time available for the logic below.
  EnableTypeTabFlg <= EnableTypeTab[TabCntCntCaracters];

  // Split the signal to improve timing.
  CntCaracters_A <= CntCaracters;
  CntCaracters_B <= CntCaracters;

  // Update the EnableTypeTab array.
  if (TypeModeTab_v[0] == ACTIF) begin
    EnableTypeTab[CntCaracters_A] <= EnableTypeTab_A_v;
    TypeModeTab_v[0] = INACTIF;
  end

  // Update the ModeTypeTab array.
  if (TypeModeTab_v[1] == ACTIF) begin
    ModeTypeTab[CntCaracters_A] <= ModeTypeTab_A_v;
    TypeModeTab_v[1] = INACTIF;
  end

  // Update the UserEnableTypeTab array.
  if (TypeModeTab_v[2] == ACTIF) begin
    UserEnableTypeTab[CntCaracters_B] <= UserEnableTypeTab_A_v;
    TypeModeTab_v[2] = INACTIF;
  end

  // Increment the CntPulse80 counter for
  // synchronization in 80-column mode.
  if (StartCntPulse == ACTIF) begin
    if (CntPulse80 == 2'd3)
      CntPulse80 <= 2'd0;
    else
      CntPulse80 <= CntPulse80 + 2'd1;
  end else begin
    CntPulse80 <= 2'd0;
  end

  // Increment the CntPulse40 counter for
  // synchronization in 40-column mode.
  if (StartCntPulse == ACTIF) begin
    if (CntPulse40 == 3'd5)
      CntPulse40 <= 3'd0;
    else
      CntPulse40 <= CntPulse40 + 3'd1;
  end else begin
    CntPulse40 <= 3'd4;
  end

  // -------------------------------------------------------------------------------
  //                         VRAM Display Addressing
  //
  // Purpose : Compute the VRAM address of the first display line after the
  //           service line using the YOR register.
  //
  // Notes   : The YOR register defines the starting VRAM address offset,
  //           providing a hardware vertical scrolling mechanism.
  //
  //           The display line management logic automatically wraps the VRAM
  //           address pointer when the end of the display memory is reached.
  //
  // Example : YOR = 15
  //
  //   Displayed Line   VRAM Address   Physical VRAM Line
  //   --------------   ------------   ------------------
  //         0               0          Service line
  //         1             320          Line 8
  //         2             360          Line 9
  //         ...
  //        17             960          Line 24
  //        18              40          Line 1
  //         ...
  //        24             280          Line 7
  // -------------------------------------------------------------------------------

  InternalYOR <= YOR - 5'b00111;     // YOR minus 7
  YORAddress  <= InternalYOR * 6'(forty); // Multiplying of InternalOR per 40 to get the first address

  // Disabling of height doubling for the service line
  // when bit 7 of the MAT register is ACTIVE (DH flag).
  if (CurrentLine == 5'd0)
    DH <= INACTIF;
  else
    DH <= Internal_DH;

  // ------------------------------------------------------------------------
  // Reset of counters when the vertical scanning beam returns.
  // The return movement is from the bottom of the screen toward the top.
  // ------------------------------------------------------------------------
  if (v_CountReached == ACTIF) begin
    ToggleLine        <= INACTIF;
    EnDblWidth        <= INACTIF;
    UpdateVRAM_i      <= ACTIF; // Set flag UpdateVRAM
    Int_DISP_VRAMAddr <= ZeroValue;
    ModeTypeTab       <= ZeroValueFlgTab;
    EnableTypeTab     <= ZeroValueFlgTab;
    UserEnableTypeTab <= ZeroValueFlgTab;
    DblLine           <= 2'd0;
    STATE             <= 2'd0;
    CntCaracters      <= 6'd0;
    PixelCnt          <= 3'd7;   // Intermediate pixel counter
    RVBPixel          <= 3'd7;   // Current pixel counter
    CurrentLine       <= '0;
    LineCounter       <= '0;     // Current line counter
    LineCounterDH     <= '0;     // Line counter for double-height characters (40-column mode)
    LineCounterDH80   <= '0;     // Line counter for double-height characters (80-column mode, MAT register)
    LineCounterDblMAT <= '0;     // Line counter for double-height-doubled characters (MAT register)
    DispLCDH          <= '0;
    DispLCDblMAT      <= '0;

  // ----------------------------------------------------------------------------------------------
  //
  //                            Display routine in 40-column mode
  //
  // ----------------------------------------------------------------------------------------------
  end else if ((Mode80 == INACTIF) && (StartCntPulse == ACTIF)) begin
    // Management of the character double-width flag
    // ModeTypeTab: array storing the character mode:
    // ACTIVE: double-height mode
    // INACTIVE: double-height + double-width mode
    if (LatchEnableQuad == INACTIF && (DoubleWidth == ACTIF ||
        (ModeTypeTab[CntCaracters] == INACTIF && EnableTypeTab[CntCaracters] == ACTIF)))
      DblWidth <= ACTIF;
    else
      DblWidth <= INACTIF;

    // ----------------------------------------------------------------------------------
    // Management of the current line index and retrieval of characters from the ROM.
    // Selection of the base counter.
    // ----------------------------------------------------------------------------------
    if (DH == ACTIF) begin
      Index <= LineCounterDH;   // (MAT7 global doubling: not delayed on the real chip)
      if (EnableTypeTabFlg == ACTIF) begin
        // (2026, FIXES[7]: R2 double height delayed by one doubled line,
        //  the lower half starts with slice 4; the MAT7 doubling alone is not)
        if (FIXES[7] && HalfStartDblM)
          Index <= 4'h4;
        else if (LCDblMAT_v < 4'h5)
          Index <= LCDblMAT_v + 4'h5;
        else
          Index <= LCDblMAT_v;
      end else if (DblHeight == ACTIF) begin
        if (LCDblMAT_v > 4'h4)
          Index <= LCDblMAT_v - 4'h5;
        else
          Index <= LCDblMAT_v;
      end
    end else begin
      Index <= LineCounter;
      if (EnableTypeTabFlg == ACTIF) begin
        if (FIXES[7] && HalfStartDH)
          Index <= 4'h4;
        else if (LCDH_v < 4'h5)
          Index <= LCDH_v + 4'h5;
        else
          Index <= LCDH_v;
      end else if (DblHeight == ACTIF) begin
        if (LCDH_v > 4'h4)
          Index <= LCDH_v - 4'h5;
        else
          Index <= LCDH_v;
      end
    end

    // Multiplication of line signals by 4. These signals are used for addressing
    // high-resolution user-defined characters (bichrome or quadrichrome).
    USerLineCounter       <= {LineCounter[3:0], 2'b00};       // Multiplication by 4
    USerLineCounterDH     <= {LCDH_v[3:0], 2'b00};     // Multiplication by 4
    USerLineCounterDblMAT <= {LCDblMAT_v[3:0], 2'b00}; // Multiplication by 4

    // ReadQUser is ACTIVE; user data is being retrieved.
    if (ReadQUser == ACTIF) begin
      // Divide the LineCounter signal by 2, which allows for the addressing of a low-resolution
      // quadrichrome character. Additionally, increment the character address
      // by 20 if the character uses the final addressing segment.
      if (QuadriResolution == LowResolution && EnableQuad == ACTIF && SubsetIndex == FourLasttSlices)
        UserLowLineCounter <= USerLineCounterDH + 6'b010100;
      else
        UserLowLineCounter <= USerLineCounterDH;

      // Pipeline computation of the user-defined character address.
      //
      // Bits 1..0 of register LatchR1 select the character number within the buffer.
      // Bits 6..2 of register LatchR1 select the buffer itself. After linearization, each
      // buffer corresponds to a 40-byte address line.
      //
      // Each buffer stores four bichrome characters arranged as follows:
      //
      //  $00  $01  $02  $03  $04  $05  $06  $07       $24  $25  $26  $27
      // T0C0 T0C1 T0C2 T0C3 T1C0 T1C1 T1C2 T1C3 .... T9C0 T9C1 T9C2 T9C3
      //
      // T denotes the character slice (scan line), numbered from 0 to 9.
      //
      // Example: a value of $26 in LatchR1 selects the third character stored in buffer 9.

      // Retrieving the character number
      UserAddressB <= {9'b000000000, UserNumberChar};
      // There are buffers 0, then 8, 9, 10... 127; we therefore check
      // for the presence of the first buffer.
      if (UserNumberBuffer > 5'b00111) begin
        UserAddressA   <= UserNumberBuffer - 5'b00111;  // Subtraction
        UserAddressMul <= UserAddressA * 6'(forty);     // Multiplication
      end else begin
        UserAddressMul <= '0;
      end
      // Calculation of the definition address for the user-defined character
      UserAddress <= UserAddressB + UserAddressMul;

      // Management of the addressing offset for bichrome and quadrichrome user-defined characters
      case (PtrBQ)
        4'd0:    UserOffset <= UserLowLineCounter;              // Low-resolution quadrichrome character
        4'd1:    UserOffset <= UserLowLineCounter - 6'b010100;  // Low-resolution quadrichrome character
        4'd2:    UserOffset <= USerLineCounterDH;
        4'd3:    UserOffset <= USerLineCounterDblMAT - 6'b010100;
        4'd4:    UserOffset <= USerLineCounterDblMAT + 6'b010100;
        4'd5:    UserOffset <= USerLineCounterDblMAT;
        4'd6:    UserOffset <= USerLineCounterDblMAT;
        4'd7:    UserOffset <= USerLineCounterDH;
        4'd8:    UserOffset <= USerLineCounterDH - 6'b010100;
        4'd9:    UserOffset <= USerLineCounterDH + 6'b010100;
        4'd10:   UserOffset <= USerLineCounterDH;
        default: UserOffset <= USerLineCounter;
      endcase

      // VRAM addressing
      DISP_VRAMAddr <= UserAddress[9:0] + 10'(UserOffset);  // User character
    // Standard character
    end else begin
      DISP_VRAMAddr <= Int_DISP_VRAMAddr;
    end

    // Management of the quadrichrome character attribute using two-bit blocks.
    // The constants C3, C2, C1, and C0 represent the various combinations
    // of the 2-bit binary encoding.
    // C3="11", C2="10", C1="01", and C0="00"
    // Bit block analyzed: Bit7..Bit6
    case (QROM[7:6])
      C3:      QuadriColour3 <= Color3;
      C2:      QuadriColour3 <= Color2;
      C1:      QuadriColour3 <= Color1;
      default: QuadriColour3 <= Color0;
    endcase

    // Bit block analyzed: Bit5..Bit4
    case (QROM[5:4])
      C3:      QuadriColour2 <= Color3;
      C2:      QuadriColour2 <= Color2;
      C1:      QuadriColour2 <= Color1;
      default: QuadriColour2 <= Color0;
    endcase

    // Bit block analyzed: Bit3..Bit2
    case (QROM[3:2])
      C3:      QuadriColour1 <= Color3;
      C2:      QuadriColour1 <= Color2;
      C1:      QuadriColour1 <= Color1;
      default: QuadriColour1 <= Color0;
    endcase

    // Bit block analyzed: Bit1..Bit0
    case (QROM[1:0])
      C3:      QuadriColour0 <= Color3;
      C2:      QuadriColour0 <= Color2;
      C1:      QuadriColour0 <= Color1;
      default: QuadriColour0 <= Color0;
    endcase

    // Fetch and latch the first data word to be processed just before the
    // LimitDisplay signal is asserted.
    //
    // Each processing step is synchronized with the CntPulse80 signal, allowing the
    // corresponding ROM and VRAM data to be read and displayed correctly.
    if ((StartDisplay == ACTIF) && (CntPulse80 == 2'd1)) begin
      // Processing and fetch of user-defined characters
      if (STATE == 2'd0) begin
        if (UserCaracterType == Bichrome) begin
          ReadQUser       <= ACTIF; // Enable the user data fetch
          EnableBich      <= ACTIF;
          EnableBandQ     <= ACTIF;
          LatchEnableQuad <= INACTIF;
          // Bit 1 set: double height
          UserDblHeight   <= BichDblHeight;
        // Quadrichrome character
        end else if (UserCaracterType == Quadrichrome) begin
          ReadQUser       <= ACTIF; // Enable the user data fetch
          EnableQuad      <= ACTIF;
          EnableBandQ     <= ACTIF;
          LatchEnableQuad <= ACTIF; // Enable the quadrichrome user-defined character
          UserDblHeight   <= INACTIF;
        // Disable the quadrichrome user-defined character
        end else begin
          ReadQUser       <= INACTIF; // Disable the user data fetch
          LatchEnableQuad <= INACTIF;
        end
        LatchR1 <= QR1;
        LatchR2 <= QR2;
        LatchR3 <= QR3;
        STATE   <= 2'd1;

      // Wait one cycle since the RAM is synchronous
      end else if (STATE == 2'd1) begin
        STATE <= 2'd2;

      // Update the data used to display the first character of the
      // current line.
      end else if (STATE == 2'd2) begin
        ReadQUser   <= INACTIF;  // Disable the user data fetch
        EnableBich  <= INACTIF;
        EnableQuad  <= INACTIF;
        EnableBandQ <= INACTIF;
        CurrentR2   <= LatchR2;   // Attribute
        if (LatchR2USerCaracter == ACTIF) // User-defined character
          R1 <= UserCaracter;
        else
          R1 <= LatchR1;
        R2 <= LatchR2; // Attribute
        R3 <= LatchR3; // Color
        // Bit 1 of register R2, which controls the optional double-height attribute of the
        // current character, is latched only when the character is of the standard type.
        if (UserDblHeight == INACTIF)
          DblHeight <= LatchR2DblHeight; // Double height
        else
          DblHeight <= INACTIF;
        // Reverse video
        Negative <= N;
        // Blink (the flash phase follows Negative in 40 columns)
        Blink        <= F;
        FlashReverse <= INACTIF;
        // Reset the TabCntCntCaracters counter
        TabCntCntCaracters <= 6'd0;
        // Jump to step 3
        STATE <= 2'd3;

      // State 3
      end else if (STATE == 2'd3) begin
        // Update the color vectors for the quadrichrome user-defined characters
        if (LatchEnableQuad == ACTIF) begin
          // Build the Qgreen vector.
          QIns <= InsertVector(InsertMode, OverlayIn, 8'hFF); // quadrichrome: every dot is foreground
          Qgreen[7:6] <= {QuadriColour3[1], QuadriColour3[1]};
          Qgreen[5:4] <= {QuadriColour2[1], QuadriColour2[1]};
          Qgreen[3:2] <= {QuadriColour1[1], QuadriColour1[1]};
          Qgreen[1:0] <= {QuadriColour0[1], QuadriColour0[1]};

          // Build the Qblue vector.
          Qblue[7:6] <= {QuadriColour3[2], QuadriColour3[2]};
          Qblue[5:4] <= {QuadriColour2[2], QuadriColour2[2]};
          Qblue[3:2] <= {QuadriColour1[2], QuadriColour1[2]};
          Qblue[1:0] <= {QuadriColour0[2], QuadriColour0[2]};

          // Build the QRed vector.
          QRed[7:6] <= {QuadriColour3[0], QuadriColour3[0]};
          QRed[5:4] <= {QuadriColour2[0], QuadriColour2[0]};
          QRed[3:2] <= {QuadriColour1[0], QuadriColour1[0]};
          QRed[1:0] <= {QuadriColour0[0], QuadriColour0[0]};

        // Update the Qgreen, Qblue and QRed color vectors
        end else begin
          QIns <= InsertVector(InsertMode, OverlayIn, QROMFg); // insert value of each dot
          for (int count = 0; count < 8; count++) begin
            if (QROM[count] == ACTIF) begin
              Qgreen[count] <= CaracterColour[1];
              Qblue[count]  <= CaracterColour[2];
              QRed[count]   <= CaracterColour[0];
            end else begin
              Qgreen[count] <= BackGroundColour[1];
              Qblue[count]  <= BackGroundColour[2];
              QRed[count]   <= BackGroundColour[0];
            end
          end
        end
        // Clear the StopCurrentLine40 signal
        StopCurrentLine40 <= INACTIF;
        // Latch the complement signal.
        // This delayed copy keeps the signal in phase with the
        // cursor display signal
        // DisplayCursor.
        LatchComplement <= Complement;
      end

    // When LimitDisplay is ACTIVE, the controller is within
    // the active display area.
    end else if ((LimitDisplay == ACTIF) && (StopCurrentLine40 == INACTIF)) begin
      // Processing when the counter reaches 1
      if (CntPulse40 == 3'd1) begin
        // Reset the STATE vector
        STATE <= 2'd0;
        // This routine retrieves the data required for display and manages character
        // double-height and/or double-width rendering.
        //
        // Signals QR1, QR2 and QR3 contain the (n+1) character data, allowing the next
        // character and its attributes to be processed in advance.
        //
        // EnableTypeTab and ModeTypeTab are used to control standard character scaling.
        // UserEnableTypeTab is used to control user-defined character scaling.
        //
        // CurrentQUser points to the current scan line data of the user-defined character
        // addressed by register R1.
        case (PixelCnt)
          3'd7: begin
            // Reset flag UpdateVRAM
            UpdateVRAM_i <= INACTIF;
            // Increment the VRAM address
            if (CntCaracters < 6'd39)
              Int_DISP_VRAMAddr <= Int_DISP_VRAMAddr + 10'd1;
            // Decrement the RGB pixel counter.
            if (DblWidth == INACTIF)
              RVBPixel <= RVBPixel - 3'd1;
            // Decrement the current pixel counter.
            PixelCnt <= PixelCnt - 3'd1;
          end

          // Wait one cycle since the RAMs are synchronous
          3'd6: begin
            // Decrement the RGB pixel counter.
            RVBPixel <= RVBPixel - 3'd1;
            // Decrement the current pixel counter.
            PixelCnt <= PixelCnt - 3'd1;
          end

          // At this stage, VRAM outputs QR1, QR2 and QR3 are available.
          3'd5: begin
            LatchR1 <= QR1;
            LatchR2 <= QR2;
            LatchR3 <= QR3;
            // Decrement the RGB pixel counter.
            if (DblWidth == INACTIF)
              RVBPixel <= RVBPixel - 3'd1;

            // Asserting the ReadQUser flag enables retrieval of the data addressed by register
            // R1. This is valid only when bit 7 of R2 is set, indicating that the current
            // character is a user-defined bichrome or quadrichrome character.
            if (UserCaracterType == Bichrome) begin
              ReadQUser        <= ACTIF;
              EnableBich       <= ACTIF;
              EnableBandQ      <= ACTIF;
              LatchEnableQuad0 <= INACTIF;
              // Bit 1 set: double height
              UserDblHeight    <= BichDblHeight;
            // Quadrichrome character
            end else if (UserCaracterType == Quadrichrome) begin
              ReadQUser        <= ACTIF;
              EnableQuad       <= ACTIF;
              EnableBandQ      <= ACTIF;
              LatchEnableQuad0 <= ACTIF; // Enable the quadrichrome user-defined character
              LatchEnableQuad  <= ACTIF;
              UserDblHeight    <= INACTIF; // Double height does not exist in quadrichrome mode
            // Disable the quadrichrome user-defined character
            end else begin
              LatchEnableQuad0 <= INACTIF;
            end
            // Decrement the current pixel counter.
            PixelCnt <= PixelCnt - 3'd1;
          end

          // Pixel 4
          3'd4: begin
            // Decrement the RGB pixel counter.
            // 2026 fix (FIXES[2]): not below 0. In the cell that follows a
            // double-width character (DblWidth = 0, EnDblWidth = 1) the VHDL
            // decremented 0 (integer range error; synthesis wraps to 7).
            if (!(FIXES[2] && RVBPixel == 3'd0))
              RVBPixel <= RVBPixel - 3'd1;
            // Decrement the current pixel counter.
            PixelCnt <= PixelCnt - 3'd1;
          end

          // Pixel 3
          3'd3: begin
            // Set flag UpdateVRAM
            UpdateVRAM_i <= ACTIF;
            // Decrement the RGB pixel counter.
            if (DblWidth == INACTIF)
              RVBPixel <= RVBPixel - 3'd1;
            // Latch the user data pointed to by R1, and disable
            // the ReadQUser flag.
            LatchQuser  <= UserCaracter;
            EnableBich  <= INACTIF;
            EnableQuad  <= INACTIF;
            EnableBandQ <= INACTIF;
            ReadQUser   <= INACTIF;
            // Decrement the current pixel counter.
            PixelCnt    <= PixelCnt - 3'd1;
          end

          // Pixel 2
          3'd2: begin
            // Anticipate the processing flag of the character width/height management array
            if (DblLine == LastZoomLine) begin
              // Update on the last line
              if ((LineCounter == 4'd9 && DH == INACTIF) ||
                  (LineCounterDH == 4'h9 && DH == ACTIF && ToggleLine == ACTIF))
                UpdateTabDH <= ACTIF;
              else
                UpdateTabDH <= INACTIF;
            end else begin
              UpdateTabDH <= INACTIF;
            end

            // Decrement the double-width counter (2026 fix, FIXES[2]: not below 0)
            if (!(FIXES[2] && RVBPixel == 3'd0))
              RVBPixel <= RVBPixel - 3'd1;
            // Decrement the current pixel counter.
            PixelCnt <= PixelCnt - 3'd1;
          end

          // Pixel 1
          3'd1: begin
            // Decrement the RGB pixel counter.
            if (DblWidth == INACTIF)
              RVBPixel <= RVBPixel - 3'd1;
            // Decrement the current pixel counter.
            PixelCnt <= PixelCnt - 3'd1;
          end

          // PixelCnt=0, the data serialization is complete
          default: begin
            // Update the R1, R2 and R3 data for the current mode
            if ((DblWidth == INACTIF) || (RVBPixel == 3'd0)) begin
              CurrentR2     <= LatchR2;
              UserDblHeight <= INACTIF;
              // Reset the double-width flag
              EnDblWidth    <= INACTIF;
              // Restart the double-width line counter
              RVBPixel      <= 3'd7;
            end else begin
              // Set the double-width flag
              EnDblWidth    <= ACTIF;
              // Decrement the counter
              RVBPixel      <= RVBPixel - 3'd1;
            end

            // Disable the quadrichrome character
            if (LatchEnableQuad0 == INACTIF)
              LatchEnableQuad <= INACTIF;
            LatchEnableQuad0 <= INACTIF;

            // Increment the character counter
            if (CntCaracters < 6'd39)
              CntCaracters <= CntCaracters + 6'd1;
            else
              // Request to stop the current line, it is finished
              StopCurrentLine40 <= ACTIF;

            if ((DblWidth == INACTIF) || (RVBPixel == 3'd0)) begin
              // Update the color vectors for the quadrichrome user-defined characters
              if (LatchEnableQuad0 == ACTIF) begin
                // Build the Qgreen vector.
                QIns <= InsertVector(InsertMode, OverlayIn, 8'hFF); // quadrichrome: every dot is foreground
                Qgreen[7:6] <= {QuadriColour3[1], QuadriColour3[1]};
                Qgreen[5:4] <= {QuadriColour2[1], QuadriColour2[1]};
                Qgreen[3:2] <= {QuadriColour1[1], QuadriColour1[1]};
                Qgreen[1:0] <= {QuadriColour0[1], QuadriColour0[1]};

                // Build the Qblue vector.
                Qblue[7:6] <= {QuadriColour3[2], QuadriColour3[2]};
                Qblue[5:4] <= {QuadriColour2[2], QuadriColour2[2]};
                Qblue[3:2] <= {QuadriColour1[2], QuadriColour1[2]};
                Qblue[1:0] <= {QuadriColour0[2], QuadriColour0[2]};

                // Build the QRed vector.
                QRed[7:6] <= {QuadriColour3[0], QuadriColour3[0]};
                QRed[5:4] <= {QuadriColour2[0], QuadriColour2[0]};
                QRed[3:2] <= {QuadriColour1[0], QuadriColour1[0]};
                QRed[1:0] <= {QuadriColour0[0], QuadriColour0[0]};

              // Update the Qgreen, Qblue and QRed color vectors
              end else begin
                QIns <= InsertVector(InsertMode, OverlayIn, QROMFg); // insert value of each dot
                for (int count = 0; count < 8; count++) begin
                  if (QROM[count] == ACTIF) begin
                    Qgreen[count] <= CaracterColour[1];
                    Qblue[count]  <= CaracterColour[2];
                    QRed[count]   <= CaracterColour[0];
                  end else begin
                    Qgreen[count] <= BackGroundColour[1];
                    Qblue[count]  <= BackGroundColour[2];
                    QRed[count]   <= BackGroundColour[0];
                  end
                end
              end
            end

            // Restart the line counter
            PixelCnt <= 3'd7;
            // Latching of the complement signal.
            // This delayed copy keeps the signal in phase with the
            // cursor display signal
            // DisplayCursor.
            LatchComplement <= Complement;
          end
        endcase

      // Character data is prefetched on Pulse40 phase 2 to ensure
      // sufficient time for the data to be updated
      // from the CharactersDecoder module.
      end else if ((PixelCnt == 3'd1) && (CntPulse40 == 3'd2)) begin
        // Management of the arrays used for double-width and/or double-height
        // characters.
        if (UpdateTabDH == ACTIF) begin
          // Height/width array processing
          // If the flag is set, it is set to INACTIVE
          if (CurrentR2[7:6] == 2'b11) begin
            TypeModeTab_v[0]  = ACTIF;
            EnableTypeTab_A_v = INACTIF;
          end else if (EnableTypeTab[CntCaracters_A] == ACTIF) begin
            TypeModeTab_v[0]  = ACTIF;
            EnableTypeTab_A_v = INACTIF;
          // Otherwise, the various modes are processed:
          // only the double-height mode, alone or with double width, is kept
          end else if (DoubleHeight == ACTIF) begin
            TypeModeTab_v[1:0] = 2'b11;
            EnableTypeTab_A_v  = ACTIF;
            ModeTypeTab_A_v    = ~DoubleWidth;
          end

          // User-defined character processing
          // Column 0 uses index 39. Previously it used index 0, which is the
          // index of column 1: with two double-height user characters in
          // columns 0 and 1 (e.g. a double-width one), both toggles cancelled
          // out and the upper half was displayed again on the second row.
          if (CntCaracters_B == 6'd0) begin
            CntCaracters_B <= 6'd39;
            if (UserEnableTypeTab[39] == ACTIF) begin
              TypeModeTab_v[2]      = ACTIF;
              UserEnableTypeTab_A_v = INACTIF;
            // Otherwise, the various modes are processed:
            // only the double-height mode, alone or with double width, is kept
            end else if (DoubleHeight == ACTIF) begin
              TypeModeTab_v[2]      = ACTIF;
              // DH and DH+DW
              UserEnableTypeTab_A_v = ACTIF;
            end
          end else begin
            CntCaracters_B <= CntCaracters_B - 6'd1;
            if (UserEnableTypeTab[CntCaracters_B - 6'd1] == ACTIF) begin
              TypeModeTab_v[2]      = ACTIF;
              UserEnableTypeTab_A_v = INACTIF;
            // Otherwise, the various modes are processed:
            // only the double-height mode, alone or with double width, is kept
            end else if (DoubleHeight == ACTIF) begin
              TypeModeTab_v[2]      = ACTIF;
              // DH and DH+DW
              UserEnableTypeTab_A_v = ACTIF;
            end
          end
        end

      // Anticipation of the time-critical data processing
      end else if ((PixelCnt == 3'd1) && (CntPulse40 == 3'd3)) begin
        // Anticipate the data to be processed, if
        // the current character is not in double width (DblWidth) or
        // has reached the end of doubling (EnDblWidth)
        if ((DblWidth == INACTIF) || (EnDblWidth == ACTIF)) begin
          if (LatchR2USerCaracter == ACTIF) // User-defined character
            R1 <= LatchQuser;
          else
            R1 <= LatchR1;
          R2 <= LatchR2; // Attribute
          R3 <= LatchR3; // Color

          // Bit 1 of register R2, which controls the optional double-height attribute of the
          // current character, is latched only when the character is of the standard type.
          if (UserDblHeight == INACTIF)
            DblHeight <= LatchR2DblHeight; // Double height
          else
            DblHeight <= INACTIF;
          // Reverse video
          Negative <= N;
          // Blink (the flash phase follows Negative in 40 columns)
          Blink        <= F;
          FlashReverse <= INACTIF;
        end
        // Character counter + 1 for the double-height/width array address
        if (CntCaracters < 6'd39)
          TabCntCntCaracters <= CntCaracters + 6'd1;
        // The reset correctly initializes the double-height management array
        // when the first character of a line is displayed
        else
          TabCntCntCaracters <= 6'd0;
      end

    // ------------------------------------------------------------------------
    //        Update of internal variables and processing outside the
    //                     active display area.
    // ------------------------------------------------------------------------
    end else if (StopCurrentLine40 == ACTIF) begin
      // Clear the StopCurrentLine40 signal
      StopCurrentLine40 <= INACTIF;
      // 2026 (FIXES[7]): counter values of the line just displayed
      DispLCDH     <= LineCounterDH;
      // Reset the character counter
      CntCaracters <= 6'd0;
      // Reset the pixel counter
      RVBPixel <= 3'd7;

      if (DblLine == LastZoomLine) begin
        // Logical inversion for the character double-height clock generation
        ToggleLine <= ~ToggleLine;
        // Scan line counters used for double-height character generation. Two counters are
        // implemented:
        //
        // The first counter generates standard double-height characters by incrementing the
        // scan line once every two display lines.
        //
        // The second counter generates double-height characters that are already defined as
        // double-height through register R2. In this mode, the scan line is incremented once
        // every four display lines. This operating mode is enabled when bit 7 of the MAT
        // register is set.
        if (ToggleLine == ACTIF) begin
          // Reset the standard-mode character doubling counter
          if (LineCounterDH == 4'h9)
            LineCounterDH <= 4'h0;
          else
            // Increment the counter
            LineCounterDH <= LineCounterDH + 4'd1;

          DispLCDblMAT <= LineCounterDblMAT;   // 2026 (FIXES[7])
          // Reset the MAT-register character doubling counter
          if (LineCounterDH[0] == ACTIF) begin
            if (LineCounterDblMAT == 4'h9)
              LineCounterDblMAT <= 4'h0;
            else
              // Increment the counter
              LineCounterDblMAT <= LineCounterDblMAT + 4'd1;
          end
        end
      end

      // Reset the scan line counter when it reaches 9.
      if ((LineCounter == 4'h9) && (DblLine == LastZoomLine)) begin
        LineCounter <= '0;
        DblLine     <= 2'd0;
        // Increment the display line counter.
        CurrentLine <= CurrentLine + 5'd1;

        // After displaying the service line, load the starting address of the
        // second display line according to the YOR register value.
        // Clear the character scaling vectors and associated counters.
        if (CurrentLine == 5'h0) begin
          // Clear the double-size flags when bit 7 of the MAT register is set.
          if (Internal_DH == ACTIF) begin
            ModeTypeTab       <= ZeroValueFlgTab;
            EnableTypeTab     <= ZeroValueFlgTab;
            UserEnableTypeTab <= ZeroValueFlgTab;
          end
          // Restart the double-height counters at the beginning of the bulk.
          // Datasheet (BULK : MAT7) : the service row is displayed for 10 TV lines,
          // then each bulk row buffer for 20 TV lines when MAT7 = 1. The first bulk
          // row (YOR) must therefore start with its upper half. Without this reset,
          // LineCounterDH was already at 5 and only the lower half of row YOR
          // was displayed (the 80-column path, LineCounterDH80, was already right).
          // LineCounterDH and LineCounterDblMAT must restart together: the latter
          // must be at 0 or 5 at the start of each VRAM row (every 20 scan lines),
          // otherwise R2-doubled characters under MAT7 are displayed as
          // 3,4,0,1,2 / 8,9,5,6,7.
          // Without MAT7, R2 double height only needs LineCounterDH at 0 or 5 at
          // the start of each row, which remains true.
          ToggleLine        <= INACTIF;
          LineCounterDH     <= '0;
          LineCounterDblMAT <= '0;
          Int_DISP_VRAMAddr <= YORAddress[9:0]; // Position to the second display line.

        // When bit 7 of the MAT register is set, increment the VRAM address only once every two scan lines.
        end else if ((DH == ACTIF) && (LineCounterDH == 4'h4)) begin
          Int_DISP_VRAMAddr <= Int_DISP_VRAMAddr - FirstVRAMAdress;

        // Prevent VRAM address overflow. The new address is set to the second
        // VRAM display line, i.e. x"040".
        // IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT --
        // This test must remain after the one above to correctly generate
        // the last double-height character before the roll-over occurs.
        end else if (Int_DISP_VRAMAddr == NineHundredNinetyNine) begin
          Int_DISP_VRAMAddr <= {4'b0000, forty};

        // Increment the VRAM address.
        end else begin
          Int_DISP_VRAMAddr <= Int_DISP_VRAMAddr + 10'd1;
        end

      // Increment the scan line counter and subtract 39 from the current
      // VRAM address. This subtraction repositions the address to the
      // beginning of the next display line.
      end else begin
        Int_DISP_VRAMAddr <= Int_DISP_VRAMAddr - FirstVRAMAdress;
        if (DblLine == LastZoomLine) begin
          LineCounter <= LineCounter + 4'd1;
          DblLine     <= 2'd0;
        end else begin
          DblLine     <= DblLine + 2'd1;
        end
      end
    end
  end

  // ----------------------------------------------------------------------------------------------
  //
  //                            Display routine in 80-column mode
  //
  // (In the VHDL this "if" follows the v_CountReached / 40-column chain: it is
  //  evaluated on every clock in 80-column mode, including v_CountReached.)
  // ----------------------------------------------------------------------------------------------
  if (Mode80 == ACTIF) begin
    // Reset flag UpdateVRAM
    UpdateVRAM_i  <= INACTIF;
    // VRAM addressing
    DISP_VRAMAddr <= Int_DISP_VRAMAddr;

    // Generation of the current scan line index used to access the character ROM.
    //
    // When the DH flag (bit 7) of the MAT register is set, double-height characters are
    // enabled.
    //
    // UserLine80 is the scan line counter used for user-defined semi-graphic characters.
    // Its routing selects either the normal-height or the double-height character
    // representation.
    //
    // Double-height characters are disabled for the service line, i.e. the first display
    // line.
    if (DH == ACTIF) begin
      Index      <= LCDH80_v;
      UserLine80 <= LCDH80_v;
    // Normal indexing
    end else begin
      Index      <= LineCounter;
      UserLine80 <= LineCounter;
    end

    // User-defined semi-graphic character handling.
    //
    // MSBUser80 selects the three most significant bits (B7..B5).
    // LSBUser80 selects the three least significant bits (B4..B2).
    //
    // A user-defined character is encoded as follows:
    //
    // R1 = 1 C6 C5 C4 C3 C2 C1 C0   --> Bit 7 set: user-defined character identifier.
    // R3 = A3 A2 A1 D -  -  -  -    --> Even columns.
    // R3 = -  -  -  - A3 A2 A1 D    --> Odd columns.
    //
    // Registers R1 and R3 define the pixels composing the character pattern.
    // The resulting character matrix is organized as follows:
    //
    // C0 C0 C0 C1 C1 C1  --> Row 0
    // C0 C0 C0 C1 C1 C1  --> Row 1
    // C2 C2 C2 C3 C3 C3  --> Row 2
    // C2 C2 C2 C3 C3 C3  --> Row 3
    // C4 C4 C4 C5 C5 C5  --> Row 4
    // C4 C4 C4 C5 C5 C5  --> Row 5
    // C6 C6 C6 A1 A1 A1  --> Row 6
    // C6 C6 C6 A1 A1 A1  --> Row 7
    // A2 A2 A2 A3 A3 A3  --> Row 8
    // A2 A2 A2 A3 A3 A3  --> Row 9
    //
    // UserLine80 indexes MSBUser80 and LSBUser80 to generate the corresponding
    // character scan line.
    case (UserLine80)
      // Rows 0,1
      4'h0, 4'h1: begin
        MSBUser80 <= LatchR1[0];
        LSBUser80 <= LatchR1[1];
      end
      // Rows 2,3
      4'h2, 4'h3: begin
        MSBUser80 <= LatchR1[2];
        LSBUser80 <= LatchR1[3];
      end
      // Rows 4,5
      4'h4, 4'h5: begin
        MSBUser80 <= LatchR1[4];
        LSBUser80 <= LatchR1[5];
      end
      // Rows 6,7
      4'h6, 4'h7: begin
        MSBUser80 <= LatchR1[6];
        // Column parity test
        if (ParityRow == EvenRow)
          LSBUser80 <= LatchR3[5];
        // Odd column
        else
          LSBUser80 <= LatchR3[1];
      end
      // Rows 8,9
      default: begin
        // Column parity test
        if (ParityRow == EvenRow) begin
          MSBUser80 <= LatchR3[6];
          LSBUser80 <= LatchR3[7];
        // Odd column
        end else begin
          MSBUser80 <= LatchR3[2];
          LSBUser80 <= LatchR3[3];
        end
      end
    endcase

    // Color management for the semi-graphic characters
    // R3 = A3 A2 A1 D -  -  -  -    --> Even columns
    // R3 = -  -  -  - A3 A2 A1 D    --> Odd columns
    // When bit 4 or bit 0 is ACTIVE, the second color is selected,
    // otherwise the first one.
    if ((ParityRow == EvenRow) && (LatchR3[4] == INACTIF))
      ActiveColour80 <= FirstColour;
    // Select the second color
    else if ((ParityRow == EvenRow) && (LatchR3[4] == ACTIF))
      ActiveColour80 <= SecondColour;
    // Select the first color
    else if ((ParityRow == OddRow) && (LatchR3[0] == INACTIF))
      ActiveColour80 <= FirstColour;
    // Select the second color
    else if ((ParityRow == OddRow) && (LatchR3[0] == ACTIF))
      ActiveColour80 <= SecondColour;

    // Disable the user data fetch
    ReadQUser   <= INACTIF;
    EnableBich  <= INACTIF;
    EnableQuad  <= INACTIF;
    EnableBandQ <= INACTIF;
    // No reverse video in 80-column mode
    Negative    <= INACTIF;
    // Fetch and store the first data to be processed
    // just before the LimitDisplay signal is asserted.
    // The first character to be processed starts on an even row.
    if ((StartDisplay == ACTIF) && (CntPulse80 == 2'd1)) begin
      // Step 0
      if (STATE == 2'd0) begin
        // The first useful row is the even one
        ParityRow <= EvenRow;
        // Update the R1 and R3 data for routine n+1
        LatchR1   <= QR1;
        LatchR3   <= QR3;
        STATE     <= 2'd1;
      // Update registers R1, R2 and R3
      end else if (STATE == 2'd1) begin
        R1 <= QR1;
        // Insert attribute: the colour select attribute D picks I0 (DOR3) or I1
        // (DOR7), as it picks the colour (datasheet, figure 33).
        Insert80 <= DORRegister[3];
        if (EvenActiveCoulour == ACTIF)
          Insert80 <= DORRegister[7];
        // Only one attribute in 80-column mode: the G0 set underline
        // Character underline and blink management
        // The reverse attribute sets the flash phase (FlashReverse); reverse
        // itself is done by swapping the colours of R3 below.
        if (UserCharacter80 == INACTIF) begin
          R2           <= {3'b000, EvenUnderline, 4'b0001};
          Blink        <= EvenBlink;
          FlashReverse <= EvenReverseColour;
        // No formatting is applied to user-defined characters.
        end else begin
          R2           <= 8'h01;   // Underline disabled.
          Blink        <= INACTIF; // Blink disabled.
          FlashReverse <= INACTIF;
        end

        R3[7] <= INACTIF;

        // Update register R3
        if (EvenReverseColour == INACTIF && EvenActiveCoulour == INACTIF) begin
          R3[6:4] <= FirstColour;
          R3[2:0] <= MarginColour;
        end else if (EvenReverseColour == ACTIF && EvenActiveCoulour == INACTIF) begin
          R3[6:4] <= MarginColour;
          R3[2:0] <= FirstColour;
        end else if (EvenReverseColour == INACTIF && EvenActiveCoulour == ACTIF) begin
          R3[6:4] <= SecondColour;
          R3[2:0] <= MarginColour;
        end else begin
          R3[6:4] <= MarginColour;
          R3[2:0] <= SecondColour;
        end
        STATE <= 2'd2;

      // STATE2
      end else if (STATE == 2'd2) begin
        STATE <= 2'd3;

      // Update the Qgreen, Qblue and QRed color vectors.
      end else if (STATE == 2'd3) begin
        // Update the vectors
        if (UserCharacter80 == INACTIF) begin
          // Standard character.
          QIns <= InsertVector(InsertMode, Insert80, QROM);    // insert value of each dot
          for (int count = 0; count < 8; count++) begin
            if (QROM[count] == ACTIF) begin
              Qgreen[count] <= CaracterColour[1];
              Qblue[count]  <= CaracterColour[2];
              QRed[count]   <= CaracterColour[0];
            end else begin
              Qgreen[count] <= BackGroundColour[1];
              Qblue[count]  <= BackGroundColour[2];
              QRed[count]   <= BackGroundColour[0];
            end
          end
        // User-defined character.
        end else begin
          // Process the MSB of the user-defined character.
          QIns <= InsertVector(InsertMode, Insert80, Fg80User); // lit blocks are foreground
          for (int count = 5; count < 8; count++) begin
            // When MSBUser80 is asserted, either the primary or secondary
            // color is selected.
            if (MSBUser80 == ACTIF) begin
              Qgreen[count] <= ActiveColour80[1];
              Qblue[count]  <= ActiveColour80[2];
              QRed[count]   <= ActiveColour80[0];
            // Otherwise, the margin color is used.
            end else begin
              Qgreen[count] <= MarginColour[1];
              Qblue[count]  <= MarginColour[2];
              QRed[count]   <= MarginColour[0];
            end
          end

          // Process the LSB of the user-defined character.
          for (int count = 2; count < 5; count++) begin
            // When LSBUser80 is asserted, either the primary or secondary
            // color is selected.
            if (LSBUser80 == ACTIF) begin
              Qgreen[count] <= ActiveColour80[1];
              Qblue[count]  <= ActiveColour80[2];
              QRed[count]   <= ActiveColour80[0];
            // Otherwise, the margin color is used.
            end else begin
              Qgreen[count] <= MarginColour[1];
              Qblue[count]  <= MarginColour[2];
              QRed[count]   <= MarginColour[0];
            end
          end
        end
        // Clear the StopCurrentLine80 signal.
        StopCurrentLine80 <= INACTIF;
        // Latch the complement signal.
        // This delayed copy keeps the signal in phase with the
        // cursor display signal
        // DisplayCursor.
        LatchComplement <= Complement;
      end

    // When LimitDisplay is asserted, the controller is within
    // the active display area. Processing starts when the
    // CntPulse80 counter reaches phase 3.
    end else if ((LimitDisplay == ACTIF) && (CntPulse80 == 2'd1)) begin
      STATE <= 2'd0;
      // This routine retrieves the display data according to the
      // parity of the current row. Register R1 points to the
      // even-row characters, while register R2 points to the odd-row characters.
      case (PixelCnt)
        3'd7: begin
          // Increment the VRAM address and the character counter
          // when the current row parity is odd.
          if ((CntCaracters < 6'd39) && (ParityRow == OddRow))
            Int_DISP_VRAMAddr <= Int_DISP_VRAMAddr + 10'd1;
          // Decrement the RGB pixel counter.
          RVBPixel <= RVBPixel - 3'd1;
          // Decrement the current pixel counter.
          PixelCnt <= PixelCnt - 3'd1;
        end

        // At this stage, VRAM outputs QR1, QR2 and QR3 are available.
        // When the parity is even, the character to be displayed is selected from R2.
        // When the parity is odd, the character to be displayed is selected from R1.
        // This register selection is intentionally reversed with respect to the parity,
        // since it corresponds to the next character (n+1) to be displayed.
        3'd5: begin
          if (ParityRow == EvenRow)
            LatchR1 <= QR2; // Odd-row character data.
          else
            LatchR1 <= QR1; // Even-row character data.
          LatchR3 <= QR3;   // Color and attribute data.

          // Decrement the RGB pixel counter.
          RVBPixel <= RVBPixel - 3'd1;
          // Decrement the current pixel counter.
          PixelCnt <= PixelCnt - 3'd1;
        end

        // Update registers R1 and R2 in advance to compensate
        // for the character ROM access latency.
        3'd4: begin
          R1 <= LatchR1;
          // Insert attribute of the column being prepared: DOR3 or DOR7 according to
          // its colour select attribute D (datasheet, figure 33).
          if (ParityRow == OddRow) begin
            if (EvenActiveCoulour == ACTIF)
              Insert80 <= DORRegister[7];
            else
              Insert80 <= DORRegister[3];
          end else begin
            if (OddActiveCoulour == ACTIF)
              Insert80 <= DORRegister[7];
            else
              Insert80 <= DORRegister[3];
          end
          // G0 underline and/or blink attribute for odd columns.
          // FlashReverse carries the reverse attribute of the same column, which
          // sets the flash phase (reverse itself is done in R3 below).
          if (ParityRow == OddRow && UserCharacter80 == INACTIF) begin
            R2           <= {3'b000, EvenUnderline, 4'b0001};
            Blink        <= EvenBlink;
            FlashReverse <= EvenReverseColour;
          // G0 underline and/or blink attribute for even columns.
          end else if (UserCharacter80 == INACTIF) begin
            R2           <= {3'b000, OddUnderline, 4'b0001};
            Blink        <= OddBlink;
            FlashReverse <= OddReverseColour;
          // No formatting is applied to user-defined characters.
          end else begin
            R2           <= 8'h01;   // Underline disabled.
            Blink        <= INACTIF; // Blink disabled.
            FlashReverse <= INACTIF;
          end

          if (ParityRow == OddRow) begin
            if (EvenReverseColour == INACTIF && EvenActiveCoulour == INACTIF)
              R3 <= {1'b0, FirstColour, 1'b0, MarginColour};
            else if (EvenReverseColour == ACTIF && EvenActiveCoulour == INACTIF)
              R3 <= {1'b0, MarginColour, 1'b0, FirstColour};
            else if (EvenReverseColour == INACTIF && EvenActiveCoulour == ACTIF)
              R3 <= {1'b0, SecondColour, 1'b0, MarginColour};
            else
              R3 <= {1'b0, MarginColour, 1'b0, SecondColour};
          end else begin
            if (OddReverseColour == INACTIF && OddActiveCoulour == INACTIF)
              R3 <= {1'b0, FirstColour, 1'b0, MarginColour};
            else if (OddReverseColour == ACTIF && OddActiveCoulour == INACTIF)
              R3 <= {1'b0, MarginColour, 1'b0, FirstColour};
            else if (OddReverseColour == INACTIF && OddActiveCoulour == ACTIF)
              R3 <= {1'b0, SecondColour, 1'b0, MarginColour};
            else
              R3 <= {1'b0, MarginColour, 1'b0, SecondColour};
          end
          // Decrement the RGB pixel counter.
          RVBPixel <= RVBPixel - 3'd1;
          // Decrement the current pixel counter.
          PixelCnt <= PixelCnt - 3'd1;

          // Toggle the row parity.
          ParityRow <= ~ParityRow;
        end

        // Restart the scan line sequence when PixelCnt reaches zero.
        3'd2: begin
          // Increment the character counter.
          if ((CntCaracters < 6'd39) && (ParityRow == EvenRow))
            CntCaracters <= CntCaracters + 6'd1;
          // Assert the end-of-line flag.
          if ((CntCaracters == 6'd39) && (ParityRow == EvenRow))
            StopCurrentLine80 <= ACTIF;
          // Update the Qgreen, Qblue and QRed color vectors.
          if (UserCharacter80 == INACTIF) begin
            // Standard character.
            QIns <= InsertVector(InsertMode, Insert80, QROM);    // insert value of each dot
            for (int count = 0; count < 8; count++) begin
              if (QROM[count] == ACTIF) begin
                Qgreen[count] <= CaracterColour[1];
                Qblue[count]  <= CaracterColour[2];
                QRed[count]   <= CaracterColour[0];
              end else begin
                Qgreen[count] <= BackGroundColour[1];
                Qblue[count]  <= BackGroundColour[2];
                QRed[count]   <= BackGroundColour[0];
              end
            end
          // User-defined character.
          end else begin
            // Process the MSB of the user-defined character.
            QIns <= InsertVector(InsertMode, Insert80, Fg80User); // lit blocks are foreground
            for (int count = 5; count < 8; count++) begin
              // When MSBUser80 is asserted, either the primary or secondary
              // color is selected.
              if (MSBUser80 == ACTIF) begin
                Qgreen[count] <= ActiveColour80[1];
                Qblue[count]  <= ActiveColour80[2];
                QRed[count]   <= ActiveColour80[0];
              // Otherwise, the margin color is used.
              end else begin
                Qgreen[count] <= MarginColour[1];
                Qblue[count]  <= MarginColour[2];
                QRed[count]   <= MarginColour[0];
              end
            end

            // Process the LSB of the user-defined character.
            for (int count = 2; count < 5; count++) begin
              // When LSBUser80 is asserted, either the primary or secondary
              // color is selected.
              if (LSBUser80 == ACTIF) begin
                Qgreen[count] <= ActiveColour80[1];
                Qblue[count]  <= ActiveColour80[2];
                QRed[count]   <= ActiveColour80[0];
              // Otherwise, the margin color is used.
              end else begin
                Qgreen[count] <= MarginColour[1];
                Qblue[count]  <= MarginColour[2];
                QRed[count]   <= MarginColour[0];
              end
            end
          end

          // Reload the pixel counters.
          RVBPixel <= 3'd7;
          PixelCnt <= 3'd7;
          // Latch the complement signal.
          // This delayed copy keeps the signal in phase with the
          // cursor display signal
          // DisplayCursor.
          LatchComplement <= Complement;
        end

        // Decrement the pixel counters.
        default: begin
          RVBPixel <= RVBPixel - 3'd1;
          PixelCnt <= PixelCnt - 3'd1;
        end
      endcase

    // ------------------------------------------------------------------------
    // Update internal variables and perform processing outside the
    // active display area.
    // ------------------------------------------------------------------------
    end else if (StopCurrentLine80 == ACTIF) begin
      // Clear the StopCurrentLine80 end-of-line flag.
      StopCurrentLine80 <= INACTIF;
      // Reset the pixel counter.
      RVBPixel <= 3'd7;
      // Reset the character counter.
      CntCaracters <= 6'd0;

      // Character scaling is not applied to the service line.
      // Only display lines greater than zero support double-height characters.
      if ((DblLine == LastZoomLine) && (DH == ACTIF)) begin
        // Toggle the PixelCnt80 clock generation.
        ToggleLine <= ~ToggleLine;
        if (ToggleLine == ACTIF) begin
          // Increment the scan line counters for double-height characters in 80-column mode.
          // Reset the standard double-height counter.
          if (LineCounterDH80 == 4'h9)
            LineCounterDH80 <= '0;
          // Increment the standard double-height counter.
          else
            LineCounterDH80 <= LineCounterDH80 + 4'd1;
        end
      end

      // Reset the scan line counter when it reaches 9.
      if ((LineCounter == 4'h9) && (DblLine == LastZoomLine)) begin
        LineCounter <= '0;
        DblLine     <= 2'd0;
        // Increment the display line counter.
        CurrentLine <= CurrentLine + 5'd1;

        // After displaying the service line, load the starting address of the
        // second display line according to the YOR register value.
        if (CurrentLine == 5'h0) begin
          Int_DISP_VRAMAddr <= YORAddress[9:0]; // Position to the second display line.

        // When bit 7 of the MAT register is set, increment the VRAM address only once every two scan lines.
        end else if ((DH == ACTIF) && (LineCounterDH80 == 4'h4)) begin
          Int_DISP_VRAMAddr <= Int_DISP_VRAMAddr - FirstVRAMAdress;

        // Prevent VRAM address overflow. The new address is set to the second
        // VRAM display line, i.e. x"040".
        // IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT --
        // This test must remain after the one above to correctly generate
        // the last double-height character before the roll-over occurs.
        end else if (Int_DISP_VRAMAddr == NineHundredNinetyNine) begin
          Int_DISP_VRAMAddr <= {4'b0000, forty};

        // Increment the VRAM address.
        end else begin
          Int_DISP_VRAMAddr <= Int_DISP_VRAMAddr + 10'd1;
        end
      // Increment the scan line counter and subtract 39 from the current
      // VRAM address. This subtraction repositions the address to the
      // beginning of the next display line.
      end else begin
        Int_DISP_VRAMAddr <= Int_DISP_VRAMAddr - FirstVRAMAdress;
        if (DblLine == LastZoomLine) begin
          LineCounter <= LineCounter + 4'd1;
          DblLine     <= 2'd0;
        end else begin
          DblLine     <= DblLine + 2'd1;
        end
      end
    end
  end

  // Process variables: keep their value for the next clock.
  TypeModeTab         <= TypeModeTab_v;
  EnableTypeTab_A     <= EnableTypeTab_A_v;
  ModeTypeTab_A       <= ModeTypeTab_A_v;
  UserEnableTypeTab_A <= UserEnableTypeTab_A_v;
end

// Logical OR for the Command backup
assign MOVE_BUFFER = Command[1] | Command[0];
assign MoveBusy    = FIXES[8] && (ExecCommand == ACTIF) && (CurrentCmd_i == CMD_COPY_i);

// Address Strobe falling edge pulse
assign fAS = (Sr_AS == 2'b10) ? ACTIF : INACTIF;

// Data Strobe falling edge pulse
assign fDS = (Sr_DS == 2'b10) ? ACTIF : INACTIF;

// RW Strobe falling edge pulse
assign fRW = (Sr_RW == 2'b10) ? ACTIF : INACTIF;

// -------------------------------------------------------------------------------------------------------------------------
//
//                                                 Command Processor
//
// Module : Microprocessor Interface & Command Processor
//
// This module implements the EF9345 microprocessor interface and command execution logic.
//
//  Command table
//
// +----------------------------+------+----------+-----------+---------------+-------------+--------+-------+------+
// | Command                    |Memo  | Code     | Parameter | Status        | Arguments   | Ptr    | Write | Read |
// |                            |      |76543210  |           |AL LXm LXa R17 |R1 R2 R3     |R4 R5   | Time  | Time |
// +----------------------------+------+----------+-----------+---------------+-------------+--------+-------+------+
// | Indirect                   | IND  |1000      |R/W r      |0  0   0   0   |D  -  -      |-  - MP | 2     | 3.5  |
// | 40 Char - 24 bits          | KRF  |0000      |R/W 00I    |X  X   0   0   |C  B  A      |-  - MP | 4     | 7.5  |
// | 40 Char - 16 bits          | KRG  |0000      |R/W 01I    |X  X   0   0   |A* B* W      |-  - MP | 5.5   | 7.5  |
// | 80 Char - 8 bits           | KRC  |0100      |R/W 00I    |X  X   0   0   |C  -  -      |-  - MP | 9     | 9.5  |
// | 80 Char -12 bits           | KRL  |0101      |R/W 00I    |X  X   0   0   |C  -  A      |-  - MP |12.5   |11.5  |
// | 40 Char Variable           | KRV  |0010      |R/W 00I    |X  X   X   X   |C  B  A      |- XF MP |3+3+j  |3.5+6j|
// | Expansion                  | EXP  |0110      |0000       |X  0   X   0   |C  B  A      |PW XFMP |<247   |  -   |
// | Compression                | CMP  |0111      |0000       |X  0   X   0   |C  B  A      |PW XFMP |<402   |  -   |
// | Expanded Characters        | KRE  |0001      |R/W 00I    |X  X   0   0   |C  B  A      |PW - MP | 4     | 7.5  |
// | Byte                       | OCT  |0011      |R/W p0I    |X  X   X   0   |D  -  -      |AP MP   | 4     | 4.5  |
// | Move Buffer                | MVB  |1101      |ssaa       |0  0   0   0   |W  -  -      |AP MP   |2+4n   |  -   |
// | Move Double Buffer         | MVD  |1110      |ssaa       |0  0   0   0   |W  -  -      |AP MP   |2+8n   |  -   |
// | Move Triple Buffer         | MVT  |1111      |ssaa       |0  0   0   0   |W  -  -      |AP MP   |2+12n  |  -   |
// | Clear Page 24 bits         | CLF  |0000      |0101       |X  X   0   0   |C  B  A      |-  - MP |<4700  |  -   |
// | Clear Page 16 bits         | CLG  |0000      |0111       |X  X   0   0   |A* B* W      |-  - MP |<5800  |  -   |
// | Vertical Sync Mask Set     | VSM  |1001      |1001       |0  0   0   0   |-  -  -      |-  - -  | 1     |  -   |
// | Vertical Sync Mask Reset   | VRM  |1001      |0101       |-  -   -   -   |-  -  -      |-  - -  | 1     |  -   |
// | Increment Y                | INY  |1011      |0000       |0  0   0   0   |-  -  -      |-  Y -  | 2     |  -   |
// | No Operation               | NOP  |1001      |0001       |-  -   -   -   |-  -  -      |-  - -  | 1     |  -   |
// +----------------------------+------+----------+-----------+---------------+-------------+--------+-------+------+
//
// NOTICE:
// The EF9345 status flags are cleared at the beginning of every command execution. Table 4 indicates which flags may
// subsequently be updated by the current command, rather than which flags retain their previous value.
//
// Supported Commands: IND, VSM, VRM, CLF, CLG, KRF, KRL, OCT, MVB, MVD, MVT, INY, NOP.
// Unsupported commands: KRC, KRV, CMP, EXP (not known to be used by existing Alice,
// VG5000 or common EF9345 software).
// The implementation follows the original EF9345 hardware behavior,
// including the AS/DS bus protocol, register management, status flag
// updates and command execution sequence.
// -------------------------------------------------------------------------------------------------------------------------
always_ff @(posedge Clk) begin
  // Enable of the microprocessor interface and command engine.
  // ClkDiv2_3 is a 2-bit counter incremented at each Clk edge; its bit 1 is
  // set for two consecutive periods out of four. The process therefore runs
  // on 2 Clk periods out of 4 at 130 MHz (65 million activations per second,
  // grouped in pairs).
  if (ClkDiv2_3[1] == ACTIF) begin
    // AS Shift register
    Sr_AS <= {Sr_AS[0], AS};

    // DS Shift register
    Sr_DS <= {Sr_DS[0], DS};

    // RW Shift register
    Sr_RW <= {Sr_RW[0], RW};

    // Motorola or intel microprocessor
    if (MCUType == Intel)
      fData <= fDS | fRW;
    else
      fData <= DS;

    // The VRAMs are in read mode by default
    RW_VRAM <= 3'b000;
    // Update of the Sync vertical synchronization flag.
    // If the VsyncRd flag is set, Sync is always zero.
    // The VSM and VRM commands enable or disable VsyncRd.
    if (VsyncRd == ACTIF)
      StatuRegister[2] <= INACTIF;  // Sync
    // 2026 fix (FIXES[3]): datasheet (VRM/VSM): "status S2 follows the vertical
    // sync state: it is reset for 2 TV lines per frame and stays at 1 during
    // the remaining period"; confirmed on real chips (fabio-d test_status_vsync:
    // duty cycle of S2 = 0 about 2/312). The original set S2 for the 250
    // displayed lines only (0 for 62 lines).
    else if (FIXES[3])
      StatuRegister[2] <= v_sync_n;
    // It is set when the first pixel is displayed, and cleared after the last one.
    else if ((LineCounter == 4'h0) && (CurrentLine == 5'd0) && (DblLine == 2'd0))
      // Set the vertical synchronization flag
      StatuRegister[2] <= ACTIF;    // Sync
    // Wait for the end of the display and clear the Sync flag
    else if ((LineCounter == 4'h9) && (CurrentLine == 5'd24) && (DblLine == LastZoomLine))
      StatuRegister[2] <= INACTIF;  // Sync

    // Busy seen by the microprocessor during the row boundaries (CmdSuspend).
    // A command started (CmdExec) or still running (Busy) in the window keeps
    // the Busy bit read in STATUS at 1 until the end of the window, as the
    // suspended EF9345 does. The command itself is executed at once, as
    // before: the VG5000 ROM writes R6/R7 before testing Busy ($018C) and
    // refreshes the service row without testing it ($00D3), and its frame
    // interrupt falls inside the displayed rows on this board.
    if (CmdSuspend == INACTIF)
      BusyHold <= INACTIF;
    else if ((CmdExec == ACTIF) || (Busy == ACTIF))
      BusyHold <= ACTIF;

    // The falling edge of AS latches to AD(0:7) bus state
    // and CS signal into the temporary A address register (Figure 36).
    //  A(0:2) = i   : This register index designates one out of 8 direct access registers Ri.
    //  A3 = XQR     : This is the execution request bit.
    //  A(4:7) = ASN : This is the Auto-Selection Nibble
    //  A8 = LCS     : This is the latched value of CS input pin.
    //  EF9345 is selected when the following condition is met : ASN = 2(Hexa) and LCS = 0.
    // Therefore, EF9345 is mapped in the hexadecimal microprocessor addressing space form XX20 to XX2F.
    // (EF9345 doc page 33)
    if (fAS == ACTIF) begin
      // Save DS signal to check MCU type interface
      MCUType <= DS;
      // CS state is latched
      LCS <= CS_n;
      // ASN test
      if (Data_In[7:4] == ASN_2X) begin
        ASN             <= ACTIF;
        AddressRegister <= Data_In[2:0];
        CmdExecEnabled  <= Data_In[3];
      end else begin
        ASN             <= INACTIF;
      end
      RW_Register <= INACTIF;
    end
    // The execution of any command starts at the
    // trailing edge of DS when (and only when) :
    // EF9345 has been selected,
    // XQR has been set,
    // at the previous AS falling edge.  (EF9345 doc page 35)
    //
    // When this input is strobed high by AS, the output buffers are selected
    // while DS is low for a read cycle (R/W = 1).
    // In write cycle, data present on AD(0:7) lines are strobed by R/W low
    // When this input is strobed low by AS, R/W gives the direction of data
    // transfer on AD(0:7) bus. DS high strobes the data to be written during
    // a write cycle (R/W = 0) or enables the output buffers during a read
    // cycle (R/W = 1) (EF9345 doc page 2)
    if ((fData == ACTIF) && (LCS == INACTIF) && (ASN == ACTIF)) begin
      AD_Latched  <= Data_In;
      RW_Register <= ACTIF;
    end else begin
      RW_Register <= INACTIF;
    end

    // Write register
    if (RW == INACTIF && RW_Register == ACTIF) begin
      // 2026 fix (FIXES[8]): while a move runs, a plain write to R1-R7 is held
      // and applied when the next command starts (see the end of this block)
      if (MoveBusy && AddressRegister != AddressR0 && CmdExecEnabled == INACTIF) begin
        PendMask[AddressRegister] <= 1'b1;
        PendVal[AddressRegister]  <= AD_Latched;
      end else
      case (AddressRegister)
        // Save R0 register
        AddressR0: R0Register <= AD_Latched;
        // Save R1 register
        AddressR1: R1Register <= AD_Latched;
        // Save R2 register
        AddressR2: R2Register <= AD_Latched;
        // Save R3 register
        AddressR3: R3Register <= AD_Latched;
        // Save R4 register
        AddressR4: R4Register <= AD_Latched;
        // Save R5 register
        AddressR5: R5Register <= AD_Latched;
        // Save R6 register
        AddressR6: R6Register <= AD_Latched;
        // Save R7 register
        default:   R7Register <= AD_Latched;
      endcase
      // Command flag ?
      // Intel bus: one write per cycle (WR edge), the command starts here.
      // Motorola bus: DS is read as a level, so this branch runs at every
      // activation while DS is high; the command is started once, on the
      // trailing edge of DS (see below).
      if (MCUType == Intel)
        CmdExec <= CmdExecEnabled;
    end

    // Read register
    if (RW == ACTIF && RW_Register == ACTIF) begin
      case (AddressRegister)
        AddressR0: begin
          // R0 register reading
          if (CmdExecEnabled == ACTIF) begin
            // Command started here on the Intel bus only (see the
            // trailing edge of DS below for the Motorola bus).
            if (MCUType == Intel)
              CmdExec <= ACTIF;
            // Reading STATUS with XQR = 1 is effective (datasheet, page 35,
            // note 3): the real status is returned, i.e. the Busy bit of the
            // previous command, and the command in R0 is (re)started. Busy
            // was previously forced to 1 here: a program that polls R0 with
            // XQR until Busy = 0 (Mission Omega, VG5000) never left its loop.
            Data_Out <= {(Busy | BusyHold), StatuRegister[6:0]};
          end else begin
            Data_Out <= {(Busy | BusyHold), StatuRegister[6:0]};
          end
        end
        // R1 register reading
        AddressR1: Data_Out <= R1Register;
        // R2 register reading
        AddressR2: Data_Out <= R2Register;
        // R3 register reading
        AddressR3: Data_Out <= R3Register;
        // R4 register reading
        AddressR4: Data_Out <= R4Register;
        // R5 register reading
        AddressR5: Data_Out <= R5Register;
        // R6 register reading
        AddressR6: Data_Out <= R6Register;
        // R7 register reading
        default:   Data_Out <= R7Register;
      endcase
    end

    // Motorola bus: the command starts on the trailing edge of DS
    // (datasheet, notes on command execution, point 1), once per bus cycle,
    // after the last register write made while DS was high. fDS is the
    // falling edge of DS seen by the Sr_DS shift register. Starting it at
    // every activation while DS is high would run INY several times.
    if ((MCUType == Motorola) && (fDS == ACTIF) && (LCS == INACTIF) && (ASN == ACTIF) &&
        (CmdExecEnabled == ACTIF))
      CmdExec <= ACTIF;

    // -------------------------------------------
    //  FSM=EXIT_CMD, finished command
    // -------------------------------------------
    if (MAE == EXIT_CMD) begin
      RW_VRAM          <= 3'b000; // Switching to read mode
      StatuRegister[7] <= INACTIF; // Busy
      ExecCommand      <= INACTIF;
      MAE              <= STEP0;

    // ---------------------------------------------------
    // FSM=INCR7, increment pointer R7
    // and set the overflow flag Al
    // ---------------------------------------------------
    end else if (MAE == INCR6R7) begin
      // Check for overflow
      // If R7_Bit5_0 >= 39, then R7_Bit5_0 = 0
      if (XWrapM) begin
        R7Register[5:0] <= 6'b000000;
        // Set LXm (overflow of buffer R7)
        StatuRegister[5] <= ACTIF;  // LXm
        // Set Al (overflow of buffer R7 due to auto-increment)
        StatuRegister[6] <= ACTIF;  // AL
        // Check for overflow
        // If R6_Bit4_0 = 31 (all bits high), then R6_Bit4_0 = 8
        if (R6_Bit4_0 == 5'b11111)
          R6Register[4:0] <= 5'b01000;
        else
          // Increment the R6 pointer
          R6Register[4:0] <= R6_Bit4_0 + 5'd1;
      end else begin
        // Increment R7
        R7Register[5:0] <= R7_Bit5_0 + 6'd1;
      end
      RW_VRAM          <= 3'b000; // Switch to read mode
      StatuRegister[7] <= INACTIF; // Busy
      ExecCommand      <= INACTIF;
      MAE              <= STEP0;  // Exit

    // ---------------------------------------------------
    // FSM=INCR7, increment pointer R7
    // and set the overflow flag Al
    // ---------------------------------------------------
    end else if (MAE == INCR7) begin
      // Check for overflow
      // If R7_Bit5_0 >= 39, then R7_Bit5_0 = 0
      if (XWrapM) begin
        R7Register[5:0] <= 6'b000000;
        // Set LXm (overflow of buffer R7)
        StatuRegister[5] <= ACTIF;  // LXm
        // Set Al (overflow of buffer R7 due to auto-increment)
        StatuRegister[6] <= ACTIF;  // AL
      end else begin
        // Increment R7
        R7Register[5:0] <= R7_Bit5_0 + 6'd1;
      end
      RW_VRAM          <= 3'b000; // Switch to read mode
      StatuRegister[7] <= INACTIF; // Busy
      ExecCommand      <= INACTIF;
      MAE              <= STEP0;  // Exit

    // ---------------------------------------------------
    // FSM=INCR5, increment pointer R5
    // and set the overflow flag Al
    // ---------------------------------------------------
    end else if (MAE == INCR5) begin
      // Check for overflow
      // If R5_Bit5_0 >= 39, then R5_Bit5_0 = 0
      if (XWrapA) begin
        R5Register[5:0] <= 6'b000000;
        // Set LXm (overflow of buffer R5)
        StatuRegister[4] <= ACTIF;  // LXa
        // Set Al (overflow of buffer R5 due to auto-increment)
        StatuRegister[6] <= ACTIF;  // AL
      end else begin
        // Increment R5
        R5Register[5:0] <= R5_Bit5_0 + 6'd1;
      end
      RW_VRAM          <= 3'b000; // Switch to read mode
      StatuRegister[7] <= INACTIF; // Busy
      ExecCommand      <= INACTIF;
      MAE              <= STEP0;  // Exit

    // ------------------------------------------------------------------------------------
    // FSM = KRLINCR7: R7 pointer increment in 80-column mode.
    //
    // The ParityColumn signal selects whether the current access targets an even or odd
    // column.
    //
    // The VRAM is organized into 1024-byte blocks.
    // Register R1 points to the first VRAM block, which stores data for even columns.
    // Register R2 points to the second VRAM block, which stores data for odd columns.
    // The third VRAM block is dedicated to character attributes and color information.
    //
    // ParityColumn corresponds to the parity of bit 7 of the R7 pointer.
    // A logic high indicates an odd-column access, while a logic low indicates an
    // even-column access.
    //
    // When ParityColumn indicates an even column, only the parity is toggled so that the
    // next access targets the corresponding odd column.
    //
    // When ParityColumn indicates an odd column, the parity is toggled and the R7 pointer
    // is incremented so that the next access targets the following even column.
    // ------------------------------------------------------------------------------------
    end else if (MAE == KRLINCR7) begin
      // Toggle the parity
      R7Register[7] <= ~ParityColumn;
      // Even parity processing
      if (ParityColumn == OddColumn) begin
        // Check for overflow
        // If R7_Bit5_0 >= 39, then R7_Bit5_0 = 0
        if (XWrapM) begin
          R7Register[5:0] <= 6'b000000;
          // Set LXm (overflow of buffer R7)
          StatuRegister[5] <= ACTIF;  // LXm
          // Set Al (overflow of buffer R7 due to auto-increment)
          StatuRegister[6] <= ACTIF;  // AL
        end else begin
          // Increment R7
          R7Register[5:0] <= R7_Bit5_0 + 6'd1;
        end
      end
      RW_VRAM          <= 3'b000; // Switch to read mode
      StatuRegister[7] <= INACTIF; // Busy
      ExecCommand      <= INACTIF;
      MAE              <= STEP0;   // Exit

    // -------------------------------------------------------------------------------------
    // A command is pending execution when the ExecCommand signal is asserted.
    // Commands are executed only during the non-display periods in order to avoid
    // READ/WRITE access collisions in the VRAM.
    // -------------------------------------------------------------------------------------
    // The command steps are NOT suspended at the row boundaries: only the
    // Busy bit read by the microprocessor is held (BusyHold). Freezing the
    // steps (15/11) broke the VG5000 ROM display (see BusyHold).
    // 2026 fix (FIXES[8]): in 80 columns UpdateVRAM is low for the whole 40 us
    // line, so a command waited up to a line (the real chip: KRL 12.5 us) and
    // programs that load the next character after a few us (the 80-column
    // demo: OUT $8F,$29 + OUT $CF,c every 5-8 us) overwrote it before it ran.
    // The VRAM is dual-ported and 80 columns fetch no user characters (the
    // shared BlockPointer), so the steps may run during the line.
    end else if ((ExecCommand == ACTIF) && ((UpdateVRAM == ACTIF) ||
                 (FIXES[8] && Mode80 == ACTIF && EnableBandQ == INACTIF))) begin
      // Command decoder
      case (CurrentCmd_i)
        // ----------------------------------------------------------------------------------
        //                        CLF/CLG Command
        // CLF ($05): 24-bit page fill. Repeats KRF (write + auto-increment):
        //            R1, R2 and R3 are written at each position.
        // CLG ($07): 16-bit page fill. Repeats KRG (write + auto-increment):
        //            only R1 and R2 are written; R3 is a working register (W).
        //
        // Datasheet (Table 4, note 4, page 36): the command repeats KRF or KRG with Y
        // incrementation when X overflows; when the last position of a row is reached,
        // Y is incremented and the process starts again on the next row.
        //
        // No end condition: the command never stops by itself. After the last row
        // (R6 = 31) the fill goes on from row 8, and Busy stays set until a new
        // command written with XQR aborts it (datasheet, notes on command execution,
        // point 3). This is how SGS-Thomson uses it in the EF9345 application note
        // (page 34, MPFILL routine): MP = (0, 0), CLF, wait about 15 ms, then a NOP
        // "to abort CLF". The "< 4700 us" of the datasheet is the time of one pass.
        // The Alice ROM uses the same scheme with its non-stop MVT copy.
        //
        // Sequence for each position (each step waits for UpdateVRAM):
        //   STEP0, STEP1 : wait for MP (X, Y, BlockPointer) to be up to date
        //   STEP2        : write (3 banks for CLF, 2 banks for CLG)
        //   STEP3        : write in progress
        //   STEP4        : LXm/AL, increment of R7 (and R6)
        // ----------------------------------------------------------------------------------
        CMD_CLF_i: begin
          case (MAE)
            STEP0: begin
              RW_VRAM <= 3'b000;
              MAE     <= STEP1;
            end
            STEP1: begin
              MAE     <= STEP2;
            end
            STEP2: begin
              SaveMCUData1 <= R1ClearCmd; // C (CLF) or A* (CLG)
              SaveMCUData2 <= R2ClearCmd; // B (CLF) or B* (CLG)
              SaveMCUData3 <= R3ClearCmd; // A (CLF only)
              // CLF writes three banks, CLG only two
              if (CurrentCommand[1] == CLF)
                RW_VRAM <= 3'b111;
              else
                RW_VRAM <= 3'b011;
              MAE <= STEP3;
            end
            STEP3: begin
              MAE <= STEP4;
            end
            default: begin
              // Last position of the row (X = 39): LXm and AL are set, since an
              // increment always follows the access (datasheet, status bits S5/S6).
              if (XWrapM) begin
                R7Register[5:0]  <= 6'b000000;
                StatuRegister[5] <= ACTIF;  // LXm
                StatuRegister[6] <= ACTIF;  // AL
                // Last row of the page: R6 wraps from 31 to 8, as for any Y
                // increment, and the fill goes on (Busy stays set).
                if (R6_Bit4_0 == 5'b11111) begin
                  R6Register[4:0] <= 5'b01000;
                  MAE             <= STEP0;
                // Next row
                end else begin
                  R6Register[4:0] <= R6_Bit4_0 + 5'd1;
                  MAE             <= STEP0;
                end
              // Next position in the row
              end else begin
                R7Register[5:0] <= R7_Bit5_0 + 6'd1;
                MAE             <= STEP0;
              end
            end
          endcase
        end

        // -------------------------------------------------------------------------------
        //                        KRF/KRG Command
        // KRG writes a character in 24-bit mode using registers R1, R2 and R3.
        // The character is fully defined by its code, attributes and colors.
        // KRF operates identically, except that it uses the 16-bit format
        // and therefore only uses registers R1 and R2.
        // These commands are used in 40-column mode.
        // Registers R1, R2 and R3 define the character attributes.
        // The command supports both write and read operations.
        // -------------------------------------------------------------------------------
        CMD_KRF_i: begin
          // Read mode
          if (ReadWriteCmd == ACTIF) begin
            // Read R1
            // Read R2
            // Read R3
            case (MAE)
              STEP0: begin
                // Wait one cycle since the VRAM is synchronous
                RW_VRAM <= 3'b000; // Read mode
                MAE     <= STEP1;
              end
              STEP1: begin
                MAE     <= STEP2;
                // LXm: the main pointer holds X = 39 before a possible increment
                // (datasheet, status bit S5), even without auto-increment.
                // AL is set by INCR7 when the increment takes place.
                if (R7_Bit5_0 == 6'b100111)
                  StatuRegister[5] <= ACTIF;  // LXm
              end
              default: begin
                R1Register <= ReadMCUData1; // Fetch R1
                R2Register <= ReadMCUData2; // Fetch R2
                // KRF/KRG command?
                if (CurrentCommand[1] == KRF)
                  R3Register <= ReadMCUData3; // Fetch R3
                // Auto-increment required?
                if (IncPointer == ACTIF)
                  MAE <= INCR7;
                else
                  MAE <= EXIT_CMD;
              end
            endcase

          // Write the data
          end else begin
            case (MAE)
              STEP0: begin
                // Write R1
                // Write R2
                // Write R3
                SaveMCUData1 <= R1Register; // Fetch the R1 data
                SaveMCUData2 <= R2Register;
                // KRF/KRG command?
                if (CurrentCommand[1] == KRF) begin
                  SaveMCUData3 <= R3Register;
                  // Write
                  RW_VRAM <= 3'b111;
                end else begin
                  RW_VRAM <= 3'b011;
                end
                MAE <= STEP1;
              end
              STEP1: begin
                MAE <= STEP2;
                // LXm: the main pointer holds X = 39 before a possible increment
                // (datasheet, status bit S5), even without auto-increment.
                // AL is set by INCR7 when the increment takes place.
                if (R7_Bit5_0 == 6'b100111)
                  StatuRegister[5] <= ACTIF;  // LXm
              end
              default: begin
                // Auto-increment required?
                if (IncPointer == ACTIF)
                  MAE <= INCR7;
                else
                  MAE <= EXIT_CMD;
              end
            endcase
          end
        end

        // ------------------------------------------------------------------------------
        //                           KRL Command
        // Writes a character in 12-bit format for 80-column mode.
        // Registers R1 and R3 are used to define the character code and its colors.
        // The command supports both write and read operations.
        // ------------------------------------------------------------------------------
        CMD_KRL_i: begin
          // Read R1
          // Read R2
          // Read R3
          case (MAE)
            STEP0: begin
              // Wait one cycle since the VRAM is synchronous
              RW_VRAM <= 3'b000; // Read mode
              MAE     <= STEP1;
            end
            STEP1: begin
              MAE     <= STEP2;
              // LXm: the main pointer holds X = 39 before a possible increment
              // (datasheet, status bit S5), even without auto-increment.
              // In 80 columns, AL is set by KRLINCR7 only when X actually
              // wraps, i.e. after the odd column.
              if (R7_Bit5_0 == 6'b100111)
                StatuRegister[5] <= ACTIF;  // LXm
            end
            STEP2: begin
              // Read mode
              if (ReadWriteCmd == ACTIF) begin
                // KRC (CurrentCommand[4] = 0) leaves R3 unchanged
                if (CurrentCommand[4] == ACTIF)
                  R3Register <= ReadMCUData3; // Fetch R3
                // If bit 7 of R7 is set, R2 is fetched and written into R1:
                // the requested data is in an odd column
                if (ParityColumn == OddColumn)
                  R1Register <= ReadMCUData2; // Fetch R1 from ReadMCUData2
                else
                  R1Register <= ReadMCUData1; // Fetch R1 from ReadMCUData1
                MAE <= STEP4;
              // Write the data
              end else begin
                // Write R1 to SaveMCUData1 for the even column
                SaveMCUData1 <= R1Register;
                // Write R1 to SaveMCUData2 for the odd column
                SaveMCUData2 <= R1Register;

                // When bit 7 of R7 is set, data is written to odd columns, corresponding to register
                // R2. Otherwise, data is written to even columns, corresponding to register R1.
                //
                // For odd columns, the LSB of R3 is updated. For even columns, the MSB is updated.
                // Register R3 stores the character color as well as the underline and/or blink
                // attributes.
                if (ParityColumn == OddColumn) begin
                  SaveMCUData3 <= {ReadMCUData3[7:4], R3Register[3:0]}; // LSB write
                  RW_VRAM      <= 3'b110;  // Odd columns
                end else begin
                  SaveMCUData3 <= {R3Register[7:4], ReadMCUData3[3:0]}; // MSB write
                  RW_VRAM      <= 3'b101;  // Even columns
                end
                // KRC: the C byte only (no attribute nibble in the third buffer)
                if (CurrentCommand[4] == INACTIF)
                  RW_VRAM <= (ParityColumn == OddColumn) ? 3'b010 : 3'b001;
                MAE <= STEP3;
              end
            end
            STEP3: begin
              MAE <= STEP4;
            end
            default: begin
              // Auto-increment required?
              if (IncPointer == ACTIF)
                MAE <= KRLINCR7;
              else
                MAE <= EXIT_CMD;
            end
          endcase
        end

        // -------------------------------------------------------------------------------
        //                            OCT Command
        // VRAM write/read. The VRAM pointers are R6/R7 or R4/R5.
        // The data written or read is register R1.
        // -------------------------------------------------------------------------------
        CMD_OCT_i: begin
          case (MAE)
            STEP0: begin
              RW_VRAM <= 3'b000; // VRAM in read mode
              MAE     <= STEP1;
              // LXm / LXa: the pointer used (MP if p = 0, AP if p = 1) holds
              // X = 39 before a possible increment (datasheet, status bits
              // S5 and S4), even without auto-increment. AL is set by
              // INCR6R7 or INCR5 when the increment takes place.
              if ((CurrentPointer == PointerR6R7) && (R7_Bit5_0 == 6'b100111))
                StatuRegister[5] <= ACTIF;  // LXm
              else if ((CurrentPointer == PointerR4R5) && (R5_Bit5_0 == 6'b100111))
                StatuRegister[4] <= ACTIF;  // LXa
            end
            STEP1: begin
              // Read mode
              if (ReadWriteCmd == ACTIF) begin
                R1Register <= ReadMCUData1; // Fetch the data
                MAE        <= STEP3;
              // Write mode
              end else begin
                // Write the data
                SaveMCUData1 <= R1Register;
                // Write
                RW_VRAM      <= 3'b001; // Write the data
                MAE          <= STEP2;
              end
            end
            STEP2: begin
              MAE <= STEP3;
            end
            default: begin
              // Auto-increment required?
              if (IncPointer == ACTIF) begin
                // Increment R6/R7
                if (CurrentPointer == INACTIF)
                  MAE <= INCR6R7;
                // Increment R4/R5
                else
                  MAE <= INCR5;
              // No increment
              end else begin
                MAE <= EXIT_CMD;
              end
            end
          endcase
        end

        // ------------------------------------------------------------------------------
        // MVB command: Copy a single buffer.
        // MVD command: Copy a double buffer.
        // MVT command: Copy a triple buffer.
        // The execution time required for the copy operation is identical,
        // whether one, two or three buffers are copied.
        // Stop condition, bits 1-0 (a a) of the command:
        //   01 : stop at the end of the source or destination buffer (row).
        //   10 : no stop. Datasheet (page 36): "the process never stops until
        //        aborted"; a main pointer overflow increments Y, so a whole
        //        block or page may be initialized. The copy therefore runs until
        //        a new command written with XQR aborts it (the Alice ROM sends a
        //        NOP after a fixed delay). R6 wraps from 31 to 8.
        // ------------------------------------------------------------------------------
        CMD_COPY_i: begin
          // Read mode
          case (MAE)
            STEP0: begin
              RW_VRAM <= 3'b000; // Read mode
              MAE     <= STEP1;
            end
            // Fetch the data and toggle the current pointer
            STEP1: begin
              // Toggle the pointer
              CurrentPointer <= ~CurrentPointer;
              SaveMCUData1   <= ReadMCUData1; // Fetch data no. 1
              SaveMCUData2   <= ReadMCUData2; // Fetch data no. 2
              SaveMCUData3   <= ReadMCUData3; // Fetch data no. 3
              // 2026 fix (FIXES[6]): R1 is the working register: it keeps the
              // last byte moved, of the last buffer (fabio-d test_memory_move_stop)
              if (FIXES[6])
                R1Register <= (CurrentCmd[1:0] == CMD_MVB) ? ReadMCUData1 :
                              (CurrentCmd[1:0] == CMD_MVD) ? ReadMCUData2 : ReadMCUData3;
              MAE            <= STEP2;
            end
            // Write the data
            STEP2: begin
              // Check the copy type: 1, 2 or 3 buffers.
              if (CurrentCmd[1:0] == CMD_MVB)       // A single buffer is used
                RW_VRAM <= 3'b001;
              // Two buffers are used
              else if (CurrentCmd[1:0] == CMD_MVD)
                RW_VRAM <= 3'b011;
              // All buffers are used (MVT)
              else
                RW_VRAM <= 3'b111;
              MAE <= STEP3;
            end
            // Increment the pointers
            default: begin
              // Toggle the pointer
              CurrentPointer <= ~CurrentPointer;
              MAE            <= STEP0; // Next transfer (the stop condition clears ExecCommand)

              // 2026 fix (FIXES[8]): a non-stop move ends after one page,
              // 32 x 40 words (MAME, dcvg5k; see EF9345.sv)
              CopyCount <= CopyCount + 11'd1;
              if (FIXES[8] && StopCondition == INACTIF && CopyCount == 11'd1279) begin
                StatuRegister[7] <= INACTIF; // Busy
                ExecCommand      <= INACTIF;
              end

              // Check for R4/R5 pointer overflow
              // If R5_Bit5_0 >= 39, then R5_Bit5_0 = R5_Bit5_0 - 40
              if (XWrapA) begin
                // Reset register R5
                R5Register[5:0] <= 6'b000000;
                // Check the exit condition
                // If bit 0 is set, exit on line overflow
                if (StopCondition == ACTIF) begin
                  StatuRegister[7] <= INACTIF; // Busy
                  ExecCommand      <= INACTIF;
                end
              end else begin
                // Increment R5
                R5Register[5:0] <= R5_Bit5_0 + 6'd1;
              end

              // Check for R6/R7 pointer overflow
              // If R7_Bit5_0 >= 39, then R7_Bit5_0 = R7_Bit5_0 - 40
              if (XWrapM) begin
                // Reset register R7
                R7Register[5:0] <= 6'b000000;
                // Check the exit condition
                // If bit 0 is set, exit on line overflow
                if (StopCondition == ACTIF) begin
                  StatuRegister[7] <= INACTIF; // Busy
                  ExecCommand      <= INACTIF;
                // Increment register R6
                end else begin
                  // Check for register R6 overflow
                  // If R6_Bit4_0 = 31, then R6_Bit4_0 = 8
                  if (R6_Bit4_0 == 5'b11111)
                    R6Register[4:0] <= 5'b01000;
                  else
                    // Increment the R6 pointer
                    R6Register[4:0] <= R6_Bit4_0 + 5'd1;
                end
              end else begin
                // Increment R7
                R7Register[5:0] <= R7_Bit5_0 + 6'd1;
              end
            end
          endcase
        end
        // --------------------------------------------------------
        //               Command unimplemented
        //
        //  KRC : 80 Characters - 8 bits
        //  KRV : 40 Characters Variable
        //  EXP : Expansion
        //  CMP : Compression
        //  KRE : Expanded Characters
        // --------------------------------------------------------
        default: begin
          ExecCommand <= INACTIF;
        end
      endcase
    end

    // --------------------------------------------------------
    //            Internal register management
    // --------------------------------------------------------
    // Check for a command execution request
    if (CmdExec == ACTIF) begin
      // Reset Flag status register
      StatuRegister[6] <= INACTIF; // AL
      StatuRegister[4] <= INACTIF; // LXa
      StatuRegister[5] <= INACTIF; // LXm
      StatuRegister[3] <= INACTIF; // MSBr1

      // Clear the command flag
      CmdExec <= INACTIF;

      // --------------------------------------------------------------
      //       Command processing
      // IND command
      //       7 6 5 4  3  2 1 0
      // IND : 1 0 0 0 R/W   r
      // Indirect commands
      // --------------------------------------------------------------
      if (Command == CMD_IND) begin
        case (INDIRECT_CMD)
          // TGS command processing
          CMD_TGS: begin
            // If ReadWrite is ACTIVE, the register data
            // is transferred into R1
            if (ReadWrite == ACTIF) begin
              R1Register <= TGSRegister;
            // Write the R1 data to TGS
            end else begin
              TGSRegister <= R1Register;
              Mode80      <= R1Register[7]; // TGSRegister(7) : Mode40/Mode80
              Mode80_i    <= R1Register[7]; // TGSRegister(7) : Mode40/Mode80
            end
          end

          // MAT command processing
          CMD_MAT: begin
            // If ReadWrite is ACTIVE, the register data
            // is transferred into R1
            if (ReadWrite == ACTIF)
              R1Register <= MATRegister;
            // Write the R1 data to MAT
            else
              MATRegister <= R1Register;
          end

          // PAT command processing
          CMD_PAT: begin
            // If ReadWrite is ACTIVE, the register data
            // is transferred into R1
            if (ReadWrite == ACTIF)
              R1Register <= PATRegister;
            // Write the R1 data to PAT
            else
              PATRegister <= R1Register;
          end

          // DOR command processing
          CMD_DOR: begin
            // If ReadWrite is ACTIVE, the register data
            // is transferred into R1
            if (ReadWrite == ACTIF)
              R1Register <= DORRegister;
            // Write the R1 data to DOR
            else
              DORRegister <= R1Register;
          end

          // ROR command processing
          CMD_ROR: begin
            // If ReadWrite is ACTIVE, the register data
            // is transferred into R1
            if (ReadWrite == ACTIF) begin
              R1Register <= RORRegister;
            // Write the R1 data to ROR
            end else begin
              // Clamp the YOR value
              if (R1Register[4:0] < 5'b01000) begin
                RORRegister[4:0] <= 5'b01000;
                RORRegister[7:5] <= R1Register[7:5];
              end else begin
                RORRegister <= R1Register;
              end
            end
          end

          // r = 000 : internal character ROM. Datasheet (figure 37): a slice can
          //           only be read (IND $88, slice address in R6/R7); writing is
          //           impossible. The ROM read is not implemented: no effect.
          // r = 101 and 110 : not defined by the datasheet: no effect.
          // (Previously these codes fell into the ROR branch and could change
          //  the displayed page and its vertical origin.)
          default: ;
        endcase

      // --------------------------------------------------------------
      // INY command
      //       7 6 5 4 3 2 1 0
      // INY : 1 0 1 1 0 0 0 0
      // Increment register R6
      // --------------------------------------------------------------
      end else if (FIXES[6] ? (R0Register[7:4] == 4'b1011) : (R0Register == CMD_INY)) begin  // 2026: $B1-$BF are INY aliases (real chip)
        // Check for overflow
        if (R6_Bit4_0 == 5'b11111)
          R6Register[4:0] <= 5'b01000;
        else
          // Increment register R6
          R6Register[4:0] <= R6_Bit4_0 + 5'b0001;
        StatuRegister[7] <= ACTIF;    // The BUSY flag is set
        MAE              <= EXIT_CMD; // Request to stop a command

      // --------------------------------------------------------------
      // NOP, VSM and VRM commands
      //       7 6 5 4 3 2 1 0
      // NOP : 1 0 0 1 0 0 0 1
      // VSM : 1 0 0 1 1 0 0 0
      // VRM : 1 0 0 1 0 1 0 0
      //
      // The NOP command is always executed; VSM applies a mask
      // to bit 2 of the STATUS register (S2), so the Sync signal
      // is always zero. VRM removes the masking, and the Sync
      // value becomes that of VBLANK.
      // --------------------------------------------------------------
      end else if (Command == CMD_NOP) begin
        // VSM command: the VsyncRd masking signal is enabled
        if (R0Register[3:2] == CMD_VSM)
          VsyncRd <= ACTIF;
        // VRM command: the VsyncRd masking signal is disabled
        // (2026 fix, FIXES[3]: $9C-$9F are VRM aliases on the real chip,
        //  fabio-d test_status_vsync; $90-$93 stay NOP.)
        else if (R0Register[3:2] == CMD_VRM || (FIXES[3] && R0Register[3:2] == 2'b11))
          VsyncRd <= INACTIF;

        StatuRegister[7] <= ACTIF;    // The BUSY flag is set
        MAE              <= EXIT_CMD; // Request to stop a command

      // Check that the command exists and save it
      // for execution during the horizontal/vertical blanking
      end else begin
        // By default, the command exists
        ExecCommand      <= ACTIF;   // A command is pending
        StatuRegister[7] <= ACTIF;   // The BUSY flag is set
        RW_VRAM          <= 3'b000;  // VRAM in read mode

        // -------------------------------------------------------------
        // CLF/CLG commands
        //       7 6 5 4 3 2 1 0
        // CLF : 0 0 0 0 0 1 0 1
        // CLG : 0 0 0 0 0 1 1 1
        // Clear 24-bit or 16-bit page; fill three consecutive blocks
        // with the value pointed to by registers R1, R2,
        // and R3.
        // -------------------------------------------------------------
        if ((R0Register[7:2] == CMD_CLF) && (R0Register[0] == ACTIF)) begin
          // Use the R6/R7 pointer
          CurrentPointer <= PointerR6R7;
          // Save registers R1, R2 and R3.
          R1ClearCmd     <= R1Register;
          R2ClearCmd     <= R2Register;
          R3ClearCmd     <= R3Register;
          CurrentCommand <= R0Register; // Save the command
          CurrentCmd_i   <= CMD_CLF_i;
          MAE            <= STEP0;   // Reset the state machine

        // --------------------------------------------------------------
        // KRF/KRG commands
        //       7 6 5 4  3  2 1 0
        // KRF : 0 0 0 0 R/W 0 0 I
        // KRG : 0 0 0 0 R/W 0 1 I
        // Commands are identified based on bits 7..4
        // which are set to zero, along with bit 2, which is also set
        // to zero.
        // --------------------------------------------------------------
        end else if ((Command == CMD_KRF) && (R0Register[2] == INACTIF)) begin
          // Use the R6/R7 pointer by default
          CurrentPointer <= PointerR6R7;
          CurrentCommand <= R0Register; // Save the command
          CurrentCmd_i   <= CMD_KRF_i;
          MAE            <= STEP0;      // Reset the state machine

        // --------------------------------------------------------------
        // KRL command
        //       7 6 5 4  3  2 1 0
        // KRL : 0 1 0 1 R/W 0 0 I
        // --------------------------------------------------------------
        // 2026 fix (FIXES[5]): KRC  0 1 0 0 R/W 0 0 I : the KRL sequence on
        // the C byte only (datasheet figure 31): R1 <-> buffer B (even column)
        // or B+1 (odd column, R7 bit 7), same auto-incrementation as KRL.
        // (2026, FIXES[6]: bits 2:1 are don't care, $42-$46 / $52-$56 aliases)
        end else if (((Command == CMD_KRL) || (FIXES[5] && Command == 4'b0100)) &&
                     (FIXES[6] || R0Register[2:1] == 2'b00)) begin
          // Use the R6/R7 pointer by default
          CurrentPointer <= PointerR6R7;
          CurrentCommand <= R0Register; // Save the command
          CurrentCmd_i   <= CMD_KRL_i;
          MAE            <= STEP0;      // Reset the state machine

        // --------------------------------------------------------------
        // OCT command
        //       7 6 5 4  3  2 1 0
        // OCT : 0 0 1 1 R/W p 0 I
        // --------------------------------------------------------------
        end else if ((Command == CMD_OCT) && (FIXES[6] || R0Register[1] == INACTIF)) begin  // 2026: $32/$36 aliases
          CurrentPointer <= R0Register[2];
          CurrentCommand <= R0Register; // Save the command
          CurrentCmd_i   <= CMD_OCT_i;
          MAE            <= STEP0;      // Reset the state machine

        // --------------------------------------------------------------
        // MVB/MVD/MVT command
        //       7 6 5 4 3 2 1 0
        // MVB : 1 1 0 1 x x x x
        // MVD : 1 1 1 0 x x x x
        // MVT : 1 1 1 1 x x x x
        // Bits 7..6 set indicate a buffer copy command
        // buffer
        // --------------------------------------------------------------
        end else if ((Command[3:2] == CMD_COPY) && (MOVE_BUFFER == ACTIF)) begin
          CurrentPointer <= ~R0Register[2];
          CurrentCommand <= R0Register; // Save the command
          CurrentCmd_i   <= CMD_COPY_i;
          CopyCount      <= 11'd0;
          MAE            <= STEP0;      // Reset the state machine

        // --------------------------------------------------------
        //               Command unimplemented
        //
        //  KRC : 80 Characters - 8 bits
        //  KRV : 40 Characters Variable
        //  EXP : Expansion
        //  CMP : Compression
        //  KRE : Expanded Characters
        // --------------------------------------------------------
        end else begin
          ExecCommand      <= INACTIF;
          StatuRegister[7] <= INACTIF; // Busy
        end
      end
    end

    // 2026 fix (FIXES[8]): moves run in the background here (about 200 us
    // for a page) while MAME / dcvg5k complete them at once; programs written
    // on those (D. Coulom's 80-column demo) go on loading R1-R7 for the next
    // command without waiting. The move keeps its own pointers: plain writes
    // were held (above); a write with XQR stops the move at once (the next
    // command aborts it anyway) and applies the held writes, then its own.
    if (FIXES[8] && PendMask != 7'd0 && ExecCommand == INACTIF) begin
      for (int r = 1; r <= 7; r++)
        if (PendMask[r]) begin
          case (r)
            1: R1Register <= PendVal[1];
            2: R2Register <= PendVal[2];
            3: R3Register <= PendVal[3];
            4: R4Register <= PendVal[4];
            5: R5Register <= PendVal[5];
            6: R6Register <= PendVal[6];
            default: R7Register <= PendVal[7];
          endcase
        end
      PendMask <= 7'd0;
    end
    if (MoveBusy && RW == INACTIF && RW_Register == ACTIF && CmdExecEnabled == ACTIF) begin
      ExecCommand <= INACTIF;
      MAE         <= STEP0;
      for (int r = 1; r <= 7; r++)
        if (PendMask[r]) begin
          case (r)
            1: R1Register <= PendVal[1];
            2: R2Register <= PendVal[2];
            3: R3Register <= PendVal[3];
            4: R4Register <= PendVal[4];
            5: R5Register <= PendVal[5];
            6: R6Register <= PendVal[6];
            default: R7Register <= PendVal[7];
          endcase
        end
      PendMask <= 7'd0;
      case (AddressRegister)
        AddressR0: R0Register <= AD_Latched;
        AddressR1: R1Register <= AD_Latched;
        AddressR2: R2Register <= AD_Latched;
        AddressR3: R3Register <= AD_Latched;
        AddressR4: R4Register <= AD_Latched;
        AddressR5: R5Register <= AD_Latched;
        AddressR6: R6Register <= AD_Latched;
        default:   R7Register <= AD_Latched;
      endcase
    end
  end
  ClkDiv2_3 <= ClkDiv2_3 + 2'd1;

  // 2026: synchronous reset of the bus interface and of the command engine
  // (placed last, it overrides the assignments above). The EF9345 has no reset
  // pin: the registers R1-R7, TGS/MAT/PAT/DOR/ROR, the VSM/VRM mask and the
  // VRAM keep their contents, as on a VG5000 whose reset button only resets
  // the Z80. Power-up values come from the declarations.
  if (Reset == ACTIF) begin
    Sr_AS            <= 2'b00;
    Sr_DS            <= 2'b11;
    Sr_RW            <= 2'b11;
    fData            <= INACTIF;
    ASN              <= INACTIF;
    LCS              <= 1'b1;
    RW_Register      <= INACTIF;
    CmdExec          <= INACTIF;
    CmdExecEnabled   <= INACTIF;
    ExecCommand      <= INACTIF;
    BusyHold         <= INACTIF;
    RW_VRAM          <= 3'b000;
    MAE              <= STEP0;
    StatuRegister[7] <= INACTIF;
  end

  // Main pointer computing
  // Y=0 for R6<8,
  // Y=R6-7, to perform row linearization
  // for R6<25
  if (R6_Bit4_0 < 5'b01000)
    Y <= 5'b00000; // Null value if R6(4 downto 0) < 8;
  else
    Y <= R6_Bit4_0 - 5'b00111;  // R6(4 downto 0) - 7;

  // X=R7 for R7<40,
  // X=R7-40 for R7>39, modulo 40
  if (R7_Bit5_0 > 6'b100111)
    X <= R7_Bit5_0 - forty;
  else
    X <= R7_Bit5_0;

  // Auxiliary pointer computing
  // Y=0 for R4<8,
  // Y=R4-7, to perform row linearization
  // for R4<25
  if (R4_Bit4_0 < 5'b01000)
    Yp <= 5'b00000; // Null value if R4(4 downto 0) < 8;
  else
    Yp <= R4_Bit4_0 - 5'b00111;  // R4(4 downto 0) - 7;

  // X=R5 for R5<40,
  // X=R5-40 for R5>39, modulo 40
  if (R5_Bit5_0 > 6'b100111)
    Xp <= R5_Bit5_0 - forty;
  else
    Xp <= R5_Bit5_0;
end

assign HP = ClkDiv2_3[1];

// -----------------------------------------------------------------------------
//                    Raw VRAM Data Processing
//
// This module fetches and latches raw data read from the VRAM, then
// performs code translation according to the selected display mode.
//
// Supported modes:
//   - 24-bit mode: uses the three raw data bytes QC, QB and QA to
//     populate registers R1, R2 and R3.
//   - 16-bit mode: uses the two raw data bytes QC and QB, then
//     translates them to populate registers R1, R2 and R3.
// -----------------------------------------------------------------------------
always_ff @(posedge Clk) begin
  // Read QC, QB and QA from the VRAM according to the ROR register.
  // Displayed triplet: Z = ROR(7) ROR(5) ROR(6) 0 (Z3 Z2 Z1, Z0 = 0). With 8 KB,
  // Z3 = ROR(7) is ignored. The 3 buffers are at blocks B, B+1, B+2 modulo 4 in
  // the district (datasheet, figure 8), e.g. Z = 2 displays blocks 2, 3 and 0.
  case (ROR_Int[1:0])
    // Z = 0 (or 8) : blocks 0, 1, 2
    2'b00: begin
      QC <= QInDisplay[7:0];
      QB <= QInDisplay[15:8];
      QA <= QInDisplay[23:16];
    end
    // Z = 4 (or 12) : blocks 4, 5, 6
    2'b01: begin
      QC <= QInDisplay[39:32];
      QB <= QInDisplay[47:40];
      QA <= QInDisplay[55:48];
    end
    // Z = 2 (or 10) : blocks 2, 3, 0
    2'b10: begin
      QC <= QInDisplay[23:16];
      QB <= QInDisplay[31:24];
      QA <= QInDisplay[7:0];
    end
    // Z = 6 (or 14) : blocks 6, 7, 4
    default: begin
      QC <= QInDisplay[55:48];
      QB <= QInDisplay[63:56];
      QA <= QInDisplay[39:32];
    end
  endcase

  // Reset signal for the 16-bit mode latches.
  if (h_Count == 11'(XStarDisplay-3)) begin
    RazLatchs         <= ACTIF;
    Convert16To24bits <= INACTIF;
  // Trigger the 16-bit to 24-bit conversion routine for the first
  // character of the display line.
  end else if (h_Count == 11'(XStarDisplay-1)) begin
    Convert16To24bits <= ACTIF;
    RazLatchs         <= INACTIF;
  // Trigger the 16-bit to 24-bit conversion routine for all remaining
  // characters of the display line.
  end else if (PixelCnt == 3'd5 && CntPulse40 > 3'd2 && LimitDisplay == ACTIF) begin
    Convert16To24bits <= ACTIF;
  end else begin
    RazLatchs         <= INACTIF;
    Convert16To24bits <= INACTIF;
  end

  // ----------------------------------------------------------------------------------------
  // Short 16-bit mode.
  // QC corresponds to data A*.
  // QB corresponds to data B*.
  //
  // 16-bit to 24-bit data mapping:
  //   QR1 <-- B*
  //   QR2 <-- A*
  //   QR3 <-- A*
  //
  // QC and QB are translated into signals QR1, QR2 and QR3.
  // In this 16-bit mode, character data is encoded on only two bytes instead of three.
  // A conversion is therefore performed to reconstruct the corresponding 24-bit
  // character format required by the video pipeline.
  // Character attributes are cleared at the beginning of each new display line.
  // ----------------------------------------------------------------------------------------
  if (DisplayMode == ShortMode40) begin
    // Reset the attribute latches at the beginning of each new line.
    if (RazLatchs == ACTIF) begin
      m_latchm   <= INACTIF;
      m_latchi   <= INACTIF;
      m_latchu   <= INACTIF;
      m_latchc0  <= BlackColour;
      // Immediately update the color and attribute registers.
      QR3[2:0]   <= BlackColour;
      QR2[0]     <= INACTIF; // Insert attribute.
      QR2[2]     <= INACTIF; // Conceal attribute.
      QR2[4]     <= INACTIF; // Underline attribute.

    // Update of registers R1, R2 and R3.
    // These registers are updated for only a few FPGA clock cycles in order to avoid
    // display artifacts caused by the latching of certain character attributes and color
    // information.
    end else if (Convert16To24bits == ACTIF) begin
      // Update the background color field of register R3.
      // QB* --> QR3
      QR3[2:0] <= m_latchc0;
      // Update the foreground color field of register R3.
      // QB* --> QR3
      QR3[6:4] <= QC[2:0];

      // Update the Insert bit of register R2.
      QR2[0] <= m_latchi;
      // Update the Conceal bit of register R2.
      QR2[2] <= m_latchm;

      // Update register R1.
      // B* --> QR1
      QR1 <= QB;

      // Process the delimiter located at the beginning of each line (negative space).
      if (QB[7:5] == 3'b100) begin
        // Register R1 is cleared.
        QR1 <= 8'h00;

        // Update register R2.
        QR2[7:5] <= 3'b001;
        // Underline bit of register R2.
        QR2[4] <= QB[2];
        // Clear bits 3 and 1.
        QR2[3] <= INACTIF;
        QR2[1] <= INACTIF;

        // Enable reverse video.
        QR3[7] <= ACTIF;
        // Disable blinking.
        QR3[3] <= INACTIF;

        // Latch the Underline, Insert and Conceal bits.
        m_latchm <= QB[0];
        m_latchi <= QB[1];
        m_latchu <= QB[2];

        // Latch the background color.
        m_latchc0 <= QC[6:4];

      // Process the G10 character set.
      end else if (QB[7] == INACTIF && QC[7] == ACTIF) begin
        // Update register R2.
        QR2[7:3] <= 5'b00100;
        // Clear bit 1.
        QR2[1] <= INACTIF;

        // Update the blink bit of register R3.
        QR3[3] <= QC[3];
        // Disable reverse video.
        QR3[7] <= INACTIF;

        // Latch the background color.
        m_latchc0 <= QC[6:4];

      // Process the G0 character set.
      end else if (QB[7] == INACTIF && QC[7] == INACTIF) begin
        // Update register R2.
        QR2[7:5] <= 3'b000;
        // Underline bit of register R2.
        QR2[4] <= m_latchu;
        // Double-width flag.
        QR2[3] <= QC[5];
        // Double-height flag.
        QR2[1] <= QC[4];

        // Update the blink bit of register R3.
        QR3[3] <= QC[3];
        // Update the reverse video bit of register R3.
        QR3[7] <= QC[6];

      // Process the G'0 character set.
      end else if (QB[7] == ACTIF && QC[7] == INACTIF) begin
        // Update register R2.
        QR2[7:5] <= 3'b100;
        // Underline bit of register R2.
        QR2[4] <= m_latchu;
        QR2[3] <= QC[5];
        QR2[1] <= QC[4];

        // Update the blink bit of register R3.
        QR3[3] <= QC[3];
        // Update the reverse video bit of register R3.
        QR3[7] <= QC[6];

      // Process the G'10 character set.
      end else if (QB[7] == ACTIF && QC[7] == ACTIF) begin
        // Update register R2.
        QR2[7:3] <= 5'b10100;
        QR2[1] <= INACTIF;

        // Update the blink bit of register R3.
        QR3[3] <= QC[3];
        // Reverse video is disabled.
        QR3[7] <= INACTIF;

        // Latch the background color.
        m_latchc0 <= QC[6:4];
      end

      // Latch the background color when bit 7 of QC is asserted.
      if (QC[7] == ACTIF) begin
        // Latch the background color.
        m_latchc0 <= QC[6:4];
        // Immediately update the background color.
        QR3[2:0] <= QC[6:4];
      end
    end

  // 2026 fix (FIXES[5]): 80 characters short codes (TGS7:6 = 10). Datasheet
  // (80 char/row character codes): the short code is the long code with a 0
  // A nibble (positive, not underlined, not flashing, colour 0), and has no
  // semigraphic set: C(0:6) addresses G0. The third buffer is not used.
  end else if (FIXES[5] && TGSRegister[7:6] == 2'b10) begin
    QR1 <= {1'b0, QC[6:0]};
    QR2 <= {1'b0, QB[6:0]};
    QR3 <= 8'h00;

  // 24-bit mode.
  end else begin
    // Read registers R1, R2 and R3.
    QR1 <= QC;
    QR2 <= QB;
    QR3 <= QA;
  end
end

// Display mode
assign DisplayMode = {PATRegister[7], TGSRegister[7:6]};

// --------------------------------------------------------------------------------------------------------
//                     VRAM Memory Bank and Character Set Selection
//
// This module implements the EF9345 VRAM bank selection mechanism
// controlled by the DOR register.
//
// In 40-column mode, the DOR register defines the base VRAM banks
// assigned to:
//   - Quadrichrome character definitions.
//   - Bichrome semi-graphic character sets G'10 and G'11.
//   - User-defined alphanumeric character set G'0.
//
//        ______________________________________________________________
//       |      |       |       |       |       |       |       |       |
//  DOR  | Z3Q  | Z3G'1 | Z2G'1 | Z1G'1 | Z3G'0 | Z2G'0 | Z1G'0 | Z0G'0 |
//       |______|_______|_______|_______|_______|_______|_______|_______|
//
// Bit description:
//
//   Z3Q (bit 7)
//     Selects the Quadrichrome character definition area.
//       '0' : Lower 8 KB VRAM space (banks 0 to 7).
//       '1' : Upper 8 KB VRAM space (banks 8 to 15).
//
//     The original EF9345 supports up to 16 KB of VRAM.
//     This implementation supports only 8 KB; therefore Z3Q is
//     internally forced to '0'.
//
//   Z3G'1..Z1G'1 (bits 6..4)
//     Select the first VRAM bank containing the bichrome
//     semi-graphic character sets G'10 and G'11.
//
//     G'10 always starts on an even-numbered bank, while
//     G'11 occupies the following odd-numbered bank.
//
//   Z3G'0..Z0G'0 (bits 3..0)
//     Select the VRAM bank containing the user-defined
//     alphanumeric character set G'0.
//
// Example: DOR = $23
//
//          __________________
// Bank 0  | Screen Memory    |
// Bank 1  |                  |
// Bank 2  |__________________|
// Bank 3  | G'0              |
//         |__________________|
// Bank 4  | G'10             |
// Bank 5  | G'11             |
//         |__________________|
// Bank 6  | Q6               |
// Bank 7  | Q7               |
//         |__________________|
//
// Internal pointers generated by this module:
//
//   SetNumber  : Derived from bits Z3G'1..Z1G'1.
//                Selects the G'10/G'11 character set pair.
//
//   G0 Pointer : Points to the user-defined alphanumeric
//                character set (G'0).
//
//   G10/G11    : Point to the bichrome semi-graphic
//                character sets.
//
// Remaining pointers (R4/R5 and R6/R7) are reserved for
// EF9345 command processing.
//
// EnableQuad enables Quadrichrome memory addressing.
//
// This implementation reproduces the original EF9345 behavior
// while intentionally limiting the VRAM size to 8 KB.
// --------------------------------------------------------------------------------------------------------
always_ff @(posedge Clk) begin
  if (EnableQuad == ACTIF) begin
    BlockPointer <= {1'b0, SetNumber[2], SetNumber[0], SetNumber[1]};
  end else if (EnableBich == ACTIF) begin
    // UserBichromeType INACTIVE, using the pointer of the user G0 set
    if (UserBichromeType == UserG0Bichrome)
      BlockPointer <= AddressUserG0;
    // Using the pointer of the user G10 set
    else if (UserG10G11Type == UserG10Bichrome)
      BlockPointer <= {1'b0, AddressUserG10, 1'b0}; // Bit 6=0, only 8KB of VRAM available
    // G11 pointer
    else
      BlockPointer <= {1'b0, AddressUserG10, 1'b1}; // Bit 6=0, only 8KB of VRAM available
  // Registers R6/R7
  // Linearization of the auxiliary pointer formed by registers R6 and R7
  // The Z signal corresponds to the bank selection.
  end else if (CurrentPointer == PointerR6R7) begin
    BlockPointer[3:1] <= {R6Register[7], R6Register[5], R7Register[6]};
    if (Mode80 == INACTIF)
      BlockPointer[0] <= R7Register[7];
    else
      BlockPointer[0] <= INACTIF;
  // Registers R4/R5
  // Linearization of the auxiliary pointer formed by registers R4 and R5
  // The Zp signal corresponds to the bank selection.
  end else begin
    BlockPointer <= {R6Register[6], R4Register[5], R5Register[6], R5Register[7]};
  end
end

// Horizontal synchronization signal
// (h_Video_On / v_Video_On of the VHDL are computed but never used.)
assign h_sync_n = (h_Count >= 11'(H_VIDEO+H_FRONT_PORCH) && h_Count < 11'(H_VIDEO+H_HORIZ_SYNC)) ? INACTIF : ACTIF;

// Vertical synchronization signal
assign v_sync_n = (v_Count_i >= 10'(V_VIDEO+V_FRONT_PORCH) && v_Count_i < 10'(V_VIDEO+V_HORIZ_SYNC)) ? INACTIF : ACTIF;

// Insert output (datasheet, page 14). The foreground dots are those generated by
// a 1 in the character generator, before the negative attribute (the decoder
// performs the negative by inverting the dots in 40 columns).
assign QROMFg      = (Negative == INACTIF) ? QROM : ~QROM;
assign Fg80User    = {MSBUser80, MSBUser80, MSBUser80, LSBUser80, LSBUser80, LSBUser80, 2'b00};
assign WindowPixel = (UpdatePixel == ACTIF && FlagUbLb == INACTIF && FlagS == INACTIF) ? ACTIF : INACTIF;
assign InsPixel    = QIns[RVBPixel];
assign Overlay     = (WindowPixel == ACTIF) ? InsPixel : MarginInsert;
// Boxing and inlay (PAT5 = 0): black is forced where I = 0, after the cursor
// complement (the insert attribute is processed last, datasheet page 24).
assign ForceBlack  = (WindowPixel == ACTIF && InsertMode[1] == INACTIF && InsPixel == INACTIF) ? ACTIF : INACTIF;

// RGB signal generation
// The FlagUbLb and FlagS flags allow updating the half-margins based on the Ub and Lb bits of the PAT register.
assign green = (FlagUbLb == ACTIF || FlagS == ACTIF)                ? MarginColour[1]   : // Display half-margins based on Ub, Lb bits of PAT register.
               (Border == ACTIF)                                    ? MarginColour[1]   : // Display border
               (ForceBlack == ACTIF)                                ? 1'b0              : // Insert: black forced (boxing, inlay)
               (UpdatePixel == ACTIF && LatchComplement == ACTIF)   ? ~Qgreen[RVBPixel] : // Update line pixels with complement
               (UpdatePixel == ACTIF)                               ? Qgreen[RVBPixel]  : // Update line pixels
                                                                      INACTIF;            // Display frame in black

assign blue  = (FlagUbLb == ACTIF || FlagS == ACTIF)                ? MarginColour[2]   : // Display half-margins based on Ub, Lb bits of PAT register.
               (Border == ACTIF)                                    ? MarginColour[2]   : // Display border
               (ForceBlack == ACTIF)                                ? 1'b0              : // Insert: black forced (boxing, inlay)
               (UpdatePixel == ACTIF && LatchComplement == ACTIF)   ? ~Qblue[RVBPixel]  : // Update line pixels with complement
               (UpdatePixel == ACTIF)                               ? Qblue[RVBPixel]   : // Update line pixels
                                                                      INACTIF;            // Display frame in black

assign red   = (FlagUbLb == ACTIF || FlagS == ACTIF)                ? MarginColour[0]   : // Display half-margins based on Ub, Lb bits of PAT register.
               (Border == ACTIF)                                    ? MarginColour[0]   : // Display border
               (ForceBlack == ACTIF)                                ? 1'b0              : // Insert: black forced (boxing, inlay)
               (UpdatePixel == ACTIF && LatchComplement == ACTIF)   ? ~QRed[RVBPixel]   : // Update line pixels with complement
               (UpdatePixel == ACTIF)                               ? QRed[RVBPixel]    : // Update line pixels
                                                                      INACTIF;            // Display frame in black

// ----------------------------------------------------------------------------------
// Generate the colors for quadrichrome characters pointed to by R3.
// The R3 register is organized as follows:
//  Bit7    Bit6    Bit5     Bit4    Bit3    Bit2    Bit1    Bit0
//  White   Cyan   Magenta   Blue    Yellow  Green   Red     Black
// The rightmost asserted bit defines Color 0, the second asserted bit
// defines Color 1, the third asserted bit defines Color 2, and the fourth
// asserted bit defines Color 3.
// If more than four bits are set, the remaining asserted bits are ignored.
// If fewer than four bits are set, the undefined colors are assigned to white.
// The purpose of this combinational logic is to generate the four color signals
// (Color0, Color1, Color2 and Color3) according to the asserted bits, starting
// from bit 0 up to bit 7.
// The advantage of this implementation is that the entire processing is purely
// combinational, making the color signals available immediately. Although a
// sequential implementation would have required less logic, it would also have
// introduced additional latency.
// ----------------------------------------------------------------------------------
assign Color0 = (CBlack   == ACTIF) ? BlackColour   :
                (CRed     == ACTIF) ? RedColour     :
                (CGreen   == ACTIF) ? GreenColour   :
                (CYellow  == ACTIF) ? YellowColour  :
                (CBlue    == ACTIF) ? BlueColour    :
                (CMagenta == ACTIF) ? MagentaColour :
                (CCyan    == ACTIF) ? CyanColour    :
                                      WhiteColour;

assign Color1 = (CRed     == ACTIF && Color0 != RedColour)     ? RedColour     :
                (CGreen   == ACTIF && Color0 != GreenColour)   ? GreenColour   :
                (CYellow  == ACTIF && Color0 != YellowColour)  ? YellowColour  :
                (CBlue    == ACTIF && Color0 != BlueColour)    ? BlueColour    :
                (CMagenta == ACTIF && Color0 != MagentaColour) ? MagentaColour :
                (CCyan    == ACTIF && Color0 != CyanColour)    ? CyanColour    :
                                                                 WhiteColour;

assign Color2 = (CGreen   == ACTIF && Color1 != GreenColour   && Color0 != GreenColour)   ? GreenColour   :
                (CYellow  == ACTIF && Color1 != YellowColour  && Color0 != YellowColour)  ? YellowColour  :
                (CBlue    == ACTIF && Color1 != BlueColour    && Color0 != BlueColour)    ? BlueColour    :
                (CMagenta == ACTIF && Color1 != MagentaColour && Color0 != MagentaColour) ? MagentaColour :
                (CCyan    == ACTIF && Color1 != CyanColour    && Color0 != CyanColour)    ? CyanColour    :
                                                                                            WhiteColour;

assign Color3 = (CWhite   == ACTIF) ? WhiteColour   :
                (CCyan    == ACTIF) ? CyanColour    :
                (CMagenta == ACTIF) ? MagentaColour :
                (CBlue    == ACTIF) ? BlueColour    :
                (CYellow  == ACTIF) ? YellowColour  :
                                      WhiteColour;

// Set the cursor modes
// bit5 bit4
//  0     0  --> Steady cursor, inverts the character.
//  1     0  --> Blinking cursor, inverts the character.
//  0     1  --> Steady cursor, underlines the character.
//  1     1  --> Blinking cursor, underlines the character.
assign CursorMode = CM;

// ----------------------------------------------------------------------------------------
//                          Cursor Display Enable Generation
//
// The C flag (bit 6) of the MAT register enables cursor display when it is set.
//
// In 40-column mode, the cursor is displayed when the current VRAM address matches
// MainPointer, which corresponds to the primary address formed by registers R6 and R7.
//
// In 80-column mode, the address parity must also be taken into account before
// asserting the DisplayCursor signal.
// ----------------------------------------------------------------------------------------
always_comb begin : cursor_enable
  logic Test0;
  logic Test1;
  Test0 = INACTIF;
  if (Int_DISP_VRAMAddr == MainPointer[9:0])
    Test0 = ACTIF;

  Test1 = INACTIF;
  if ((ParityRow == ParityColumn) || (Mode80_i == INACTIF))
    Test1 = ACTIF;

  DisplayCursor = Test0 & Test1 & C;
end

assign PATBlink   = EnableBlinking; // Blink enable (PAT register)
assign PATConceal = EnableMask;     // Character masking enable (PAT register)

// Computation of the final address for pointers R6/R7
// Y is multiplied by 40 + X
assign MainPointer = (Y * forty) + 11'(X);

// Computation of the final address for pointers R4/R5
// Yp is multiplied by 40 + Xp
assign AuxiliaryPointer = (Yp * forty) + 11'(Xp);

// Validation of line 1 when register R4 or R6 is <8 and has an odd value.
// Test on R4_Bit4_0 and R6_Bit4_0 < 8.
assign EnableLine = (R4_Bit4_0[4:3] == 2'b00 && R4_Bit4_0[0] == ACTIF && CurrentPointer == PointerR4R5) ? ACTIF :
                    (R6_Bit4_0[4:3] == 2'b00 && R6_Bit4_0[0] == ACTIF && CurrentPointer == PointerR6R7) ? ACTIF :
                                                                                                          INACTIF;

// MCU VRAM addressing
assign VRAM_Ad_MCU = (CurrentPointer == PointerR6R7) ? MainPointer[9:0] : AuxiliaryPointer[9:0];

// VRAM addressing for display management
assign VRAM_Ad_DISP = DISP_VRAMAddr;

// Bank numbers.
// The 16 KB space of the EF9345 is made of 16 blocks of 1 KB, Z = D(1:0) & B(1:0),
// grouped in 4 districts of 4 blocks. The 2 or 3 buffers of a character code (and
// the row buffers displayed) lie at successive block addresses MODULO 4 inside a
// district (datasheet, figure 8 and page 11: "row buffers lay inside a district,
// at two or three successive block addresses (modulo 4)"). Only the block bits
// B(1:0) are incremented; the district bit is kept.
// With 8 KB, Z3 (physical A13) is ignored: block Z is the same memory as block
// Z + 8, as on an 8 KB EF9345 board where A13 is not connected.
// (Previously bp+1 and bp+2 were computed modulo 8: a code starting at block 2 or
// 3 of a district spilled into the next district, e.g. into the displayed page.)
assign bp  = BlockPointer[2:0];
// district = bp / 4, block = bp mod 4
assign bp1 = {bp[2], bp[1:0] + 2'd1};
assign bp2 = {bp[2], bp[1:0] + 2'd2};

// Routing of display-related VRAM data: bank of the user-defined character
// slice (Z3 ignored, as for the CPU accesses).
always_comb begin
  for (int i = 0; i < 8; i++)
    VRAMDisplay_array[i] = QInDisplay[8*i +: 8];
end
assign UserCaracter = VRAMDisplay_array[bp];

// -----------------------
// VRAM R/W management --
// -----------------------
// RW_VRAM(0) writes bank bp, RW_VRAM(1) bank bp1, RW_VRAM(2) bank bp2.
always_comb begin
  VRAMWe = '0;

  if (RW_VRAM[0] == ACTIF)
    VRAMWe[bp] = ACTIF;

  if (RW_VRAM[1] == ACTIF)
    VRAMWe[bp1] = ACTIF;

  if (RW_VRAM[2] == ACTIF)
    VRAMWe[bp2] = ACTIF;
end

// -----------------------------------------------
//      VRAM write output bus management
// -----------------------------------------------
// SaveMCUData1 goes to bank bp, SaveMCUData2 to bank bp1, SaveMCUData3 to bank
// bp2. The other banks receive SaveMCUData1 but are not write-enabled.
always_comb begin
  for (int i = 0; i < 8; i++) begin
    if (3'(i) == bp1)
      VRAMOutMCU_array[i] = SaveMCUData2;
    else if (3'(i) == bp2)
      VRAMOutMCU_array[i] = SaveMCUData3;
    else
      VRAMOutMCU_array[i] = SaveMCUData1;
  end
end

// VRAM write output bus management
always_comb begin
  for (int i = 0; i < 8; i++)
    QOutMCUVRAM[8*i +: 8] = VRAMOutMCU_array[i];
end

// MCU VRAM data routing
always_comb begin
  for (int i = 0; i < 8; i++)
    VRAMInMCU_array_1[i] = QInMCUVRAM[8*i +: 8]; // Memory bank i
end

// The 3 bytes of a code are read from banks bp, bp1 and bp2
assign ReadMCUData1 = VRAMInMCU_array_1[bp];
assign ReadMCUData2 = VRAMInMCU_array_1[bp1];
assign ReadMCUData3 = VRAMInMCU_array_1[bp2];

endmodule
