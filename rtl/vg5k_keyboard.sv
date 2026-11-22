//-----------------------------------------------------------------------------
// VG5000 keyboard: PS/2 (hps_io ps2_key) -> 8 x 8 matrix, active low.
// Structure, queue and scan tracking from SC3000_MiSTer rtl/sc3000_keyboard.sv.
//
// Matrix = MAME vg5k.cpp INPUT_PORTS ROW1..ROW8 = ports 80..87, bit = column
// (MAME src/mame/philips/vg5k.cpp). The Δ key is not in the matrix: it
// pulls /NMI (nmi_key, PC End, as MAME).
//
// Two layouts (P3 Input "Keyboard"):
//   PC (pc_layout = 1, default): keys type what is printed on a US PC keyboard.
//     Letters go to the VG5000 key with the same letter (the VG5000 is AZERTY:
//     PC A -> VG A, PC Q -> VG Q ...). Symbols that sit elsewhere on the VG5000
//     are moved there with the VG Shift as that character needs ('!' = Shift +
//     the '2' key, '.' = Shift + '+', ':' = the ':' key ...). Characters the
//     VG5000 has no key for ('\', '{', '}', '`', '~') are dropped.
//   AZERTY (pc_layout = 0): by position, as MAME maps it: right for a French
//     PC keyboard (the key labelled A on an AZERTY keyboard types A).
// Extra keys in both: Home / F1 = LIST, F2 / Esc = RUN-STOP, Insert = INS,
// Backspace / Delete = EFFC, Ctrl = CTRL, arrows, Caps Lock = LCK,
// ']' (PC layout: AltGr not needed) ... see base_pos. PRT = PC Page Down
// in the PC layout (its ']' key is a character there).
//
// Changes to the matrix go through a queue (1024 changes = 512 characters,
// block RAM; when it is full a key is dropped whole, press and release). Each
// change stays until the machine has read it: at least GAP (40 ms) and two
// full scans of rows 0-7 (ports 80-87), or two reads of the row the change is
// in, so a key is never missed however long BASIC is busy. (The ROM's scan at
// 02A7 stops at the first row with a key down, so while a key is held it never
// reads all rows; waiting for the 160 ms fallback there made BASIC's short key
// repeat, 8 interrupts, type the key twice.) A program that reads only other
// rows gets the next change after 4 x GAP of row activity; none, after 2 s.
//-----------------------------------------------------------------------------

module vg5k_keyboard #(
  parameter CLK_HZ = 48000000
) (
  input         clk_sys,
  input         reset,
  input         pc_layout,
  input  [2:0]  row_sel,       // the row the CPU reads (A2:A0)
  input         row_rd,        // pulse: the CPU started reading row_sel
  input  [10:0] ps2_key,
  output [7:0]  row_data,      // active low
  output reg    nmi_key = 1'b0 // Δ (End), 1 = held
);

  localparam int GAP = CLK_HZ / 25;   // 40 ms

  wire       key_event_raw = ps2_key[10];
  reg        stb_q = 1'b0;
  wire       ev    = stb_q ^ key_event_raw;
  wire [8:0] code  = ps2_key[8:0];
  wire       down  = ps2_key[9];

  reg lshift = 1'b0, rshift = 1'b0;
  wire pc_shift = lshift | rshift;

  localparam [6:0] NONE = 7'h7F;
  function automatic [6:0] P(input [2:0] r, input [2:0] c); P = {1'b0, r, c}; endfunction

  // keys at the same place in both layouts
  function automatic [6:0] common_pos(input [8:0] c);
    case (c)
      9'h16C, 9'h005: common_pos = P(0, 0);   // Home, F1   LIST
      9'h16B: common_pos = P(0, 3);           // Left
      9'h174: common_pos = P(0, 4);           // Right
      9'h172: common_pos = P(0, 5);           // Down
      9'h014, 9'h114: common_pos = P(0, 6);   // Ctrl       CTRL / ACCENT
      9'h170: common_pos = P(0, 7);           // Insert     INSC / INSL
      9'h006, 9'h076: common_pos = P(1, 0);   // F2, Esc    RUN / STOP
      9'h029: common_pos = P(1, 2);           // Space
      9'h058: common_pos = P(1, 3);           // Caps Lock  LCK
      9'h05A, 9'h15A: common_pos = P(1, 5);   // Enter      RET
      9'h175: common_pos = P(1, 6);           // Up
      9'h066, 9'h171: common_pos = P(7, 1);   // Backspace, Delete  EFFC / EFFL
      9'h17A: common_pos = P(6, 6);           // Page Down  PRT
      9'h00D: common_pos = P(4, 0);           // Tab        ÷ _
      default: common_pos = NONE;
    endcase
  endfunction

  // AZERTY: MAME's KEYCODE mapping (by position)
  function automatic [6:0] pos_map(input [8:0] c);
    case (c)
      9'h01C: pos_map = P(1, 1);  9'h015: pos_map = P(1, 7);
      9'h01A: pos_map = P(2, 0);  9'h022: pos_map = P(2, 1);  9'h021: pos_map = P(2, 2);  9'h02A: pos_map = P(2, 3);
      9'h032: pos_map = P(2, 4);  9'h016: pos_map = P(2, 5);  9'h03A: pos_map = P(2, 6);  9'h01D: pos_map = P(2, 7);
      9'h01B: pos_map = P(3, 0);  9'h024: pos_map = P(3, 1);  9'h026: pos_map = P(3, 2);  9'h025: pos_map = P(3, 3);
      9'h02E: pos_map = P(3, 4);  9'h036: pos_map = P(3, 5);  9'h01E: pos_map = P(3, 6);  9'h049: pos_map = P(3, 7);
      9'h04E: pos_map = P(4, 1);  9'h02B: pos_map = P(4, 2);  9'h034: pos_map = P(4, 3);  9'h03C: pos_map = P(4, 4);
      9'h043: pos_map = P(4, 5);  9'h044: pos_map = P(4, 6);  9'h04D: pos_map = P(4, 7);
      9'h045: pos_map = P(5, 1);  9'h041: pos_map = P(5, 2);  9'h04A: pos_map = P(5, 3);  9'h052: pos_map = P(5, 4);
      9'h03D: pos_map = P(5, 5);  9'h03E: pos_map = P(5, 6);  9'h046: pos_map = P(5, 7);
      9'h00E: pos_map = P(6, 0);  9'h05D: pos_map = P(6, 1);  9'h02D: pos_map = P(6, 2);  9'h02C: pos_map = P(6, 3);
      9'h035: pos_map = P(6, 4);  9'h054: pos_map = P(6, 5);  9'h05B: pos_map = P(6, 6);  9'h023: pos_map = P(6, 7);
      9'h055: pos_map = P(7, 0);  9'h031: pos_map = P(7, 2);  9'h03B: pos_map = P(7, 3);  9'h033: pos_map = P(7, 4);
      9'h042: pos_map = P(7, 5);  9'h04B: pos_map = P(7, 6);  9'h04C: pos_map = P(7, 7);
      default: pos_map = common_pos(c);
    endcase
  endfunction

  // PC layout: {pos, sh} for a key with the PC Shift state s.
  // sh: 0 = follow the PC Shift, 2 = VG Shift off, 3 = VG Shift on
  function automatic [8:0] pc_map(input [8:0] c, input s);
    case (c)
      // letters: the VG5000 key with that letter
      9'h01C: pc_map = {P(1, 7), 2'd0};  9'h032: pc_map = {P(2, 4), 2'd0};  9'h021: pc_map = {P(2, 2), 2'd0};
      9'h023: pc_map = {P(6, 7), 2'd0};  9'h024: pc_map = {P(3, 1), 2'd0};  9'h02B: pc_map = {P(4, 2), 2'd0};
      9'h034: pc_map = {P(4, 3), 2'd0};  9'h033: pc_map = {P(7, 4), 2'd0};  9'h043: pc_map = {P(4, 5), 2'd0};
      9'h03B: pc_map = {P(7, 3), 2'd0};  9'h042: pc_map = {P(7, 5), 2'd0};  9'h04B: pc_map = {P(7, 6), 2'd0};
      9'h03A: pc_map = {P(7, 7), 2'd0};  9'h031: pc_map = {P(7, 2), 2'd0};  9'h044: pc_map = {P(4, 6), 2'd0};
      9'h04D: pc_map = {P(4, 7), 2'd0};  9'h015: pc_map = {P(1, 1), 2'd0};  9'h02D: pc_map = {P(6, 2), 2'd0};
      9'h01B: pc_map = {P(3, 0), 2'd0};  9'h02C: pc_map = {P(6, 3), 2'd0};  9'h03C: pc_map = {P(4, 4), 2'd0};
      9'h02A: pc_map = {P(2, 3), 2'd0};  9'h01D: pc_map = {P(2, 0), 2'd0};  9'h022: pc_map = {P(2, 1), 2'd0};
      9'h035: pc_map = {P(6, 4), 2'd0};  9'h01A: pc_map = {P(2, 7), 2'd0};
      // digits row
      9'h016: pc_map = s ? {P(3, 6), 2'd3} : {P(2, 5), 2'd2};   // 1 !
      9'h01E: pc_map = s ? {P(3, 7), 2'd3} : {P(3, 6), 2'd2};   // 2 @
      9'h026: pc_map = s ? {P(2, 5), 2'd3} : {P(3, 2), 2'd2};   // 3 #
      9'h025: pc_map = s ? {P(3, 4), 2'd3} : {P(3, 3), 2'd2};   // 4 $
      9'h02E: pc_map = s ? {P(3, 5), 2'd3} : {P(3, 4), 2'd2};   // 5 %
      9'h036: pc_map = s ? {P(7, 0), 2'd3} : {P(3, 5), 2'd2};   // 6 ^
      9'h03D: pc_map = s ? {P(5, 5), 2'd3} : {P(5, 5), 2'd2};   // 7 &
      9'h03E: pc_map = s ? {P(2, 6), 2'd3} : {P(5, 6), 2'd2};   // 8 *
      9'h046: pc_map = s ? {P(5, 7), 2'd3} : {P(5, 7), 2'd2};   // 9 (
      9'h045: pc_map = s ? {P(5, 1), 2'd3} : {P(5, 1), 2'd2};   // 0 )
      9'h04E: pc_map = s ? {P(4, 0), 2'd3} : {P(6, 0), 2'd2};   // - _
      9'h055: pc_map = s ? {P(6, 1), 2'd2} : {P(7, 0), 2'd2};   // = +
      9'h054: pc_map = s ? {NONE, 2'd0}    : {P(5, 2), 2'd3};   // [ {
      9'h05B: pc_map = s ? {NONE, 2'd0}    : {P(5, 2), 2'd2};   // ] }
      9'h05D: pc_map = s ? {P(4, 1), 2'd3} : {NONE, 2'd0};      // \ |
      9'h04C: pc_map = s ? {P(2, 6), 2'd2} : {P(3, 7), 2'd2};   // ; :
      9'h052: pc_map = s ? {P(3, 2), 2'd3} : {P(5, 6), 2'd3};   // ' "
      9'h041: pc_map = s ? {P(6, 5), 2'd2} : {P(5, 4), 2'd2};   // , <
      9'h049: pc_map = s ? {P(6, 5), 2'd3} : {P(6, 1), 2'd3};   // . >
      9'h04A: pc_map = s ? {P(6, 0), 2'd3} : {P(5, 4), 2'd3};   // / ?
      9'h00E: pc_map = {NONE, 2'd0};                             // ` ~
      default: pc_map = {common_pos(c), 2'd0};
    endcase
  endfunction

  // ---------------------------------------------------------------- queue of matrix changes
  // entry = {press, pos[6:0], sh[1:0]}
  reg [9:0] q [0:1023];
  reg [9:0] wp = 0, rp = 0;
  wire      q_empty = wp == rp;
  wire      q_full  = (wp + 10'd1) == rp;
  wire      q_room2 = (wp + 10'd2) != rp && !q_full;

  reg [6:0] held_pos [0:511];
  reg [6:0] rel_pos;
  reg       rel_pending = 1'b0;
  wire [8:0] map_now = pc_layout ? pc_map(code, pc_shift) : {pos_map(code), 2'd0};
  wire       is_shift = code == 9'h012 || code == 9'h059;
  wire       is_nmi   = code == 9'h169;
  wire       push_press = ev && !reset && !is_shift && !is_nmi && down &&
                          map_now[8:2] != NONE && q_room2;
  wire       push_rel   = rel_pending && !reset && !q_full && rel_pos != NONE;
  wire       q_we = push_press | push_rel;
  wire [1:0] sh_now = (map_now[1:0] == 2'd0) ? {1'b1, pc_shift} : map_now[1:0];
  wire [9:0] q_wd = push_rel ? {1'b0, rel_pos, 2'd0} : {1'b1, map_now[8:2], sh_now};

  always @(posedge clk_sys) begin
    rel_pending <= 1'b0;
    if (reset) begin
      stb_q <= key_event_raw; lshift <= 1'b0; rshift <= 1'b0; wp <= 0; nmi_key <= 1'b0;
    end else begin
      stb_q <= key_event_raw;
      if (ev) begin
        if (code == 9'h012) lshift <= down;
        else if (code == 9'h059) rshift <= down;
        else if (is_nmi) nmi_key <= down;
        else if (down) begin
          if (push_press) held_pos[code] <= map_now[8:2];
          else            held_pos[code] <= NONE;
        end else begin
          rel_pos <= held_pos[code];
          rel_pending <= 1'b1;
        end
      end
      if (q_we) wp <= wp + 10'd1;
    end
  end

  always @(posedge clk_sys) if (q_we) q[wp] <= q_wd;

  // ---------------------------------------------------------------- apply, one change per GAP
  reg [7:0]  mat [0:7];          // 1 = pressed
  reg [31:0] t = 0;
  reg [1:0]  shf = 2'd0;         // VG Shift forced by the last key: 2 = off, 3 = on
  reg [6:0]  shf_pos = NONE;
  reg  [9:0] e;
  reg        q_ne = 1'b0;
  always @(posedge clk_sys) e <= q[rp];

  localparam int PART = GAP * 4;
  localparam int IDLE = GAP * 50;
  reg  [7:0] seen = 8'd0;
  reg  [1:0] full = 2'd0;
  reg        act = 1'b0;
  reg [31:0] since = 0;
  reg  [2:0] krow = 3'd0;            // row of the last change
  reg  [1:0] krow_n = 2'd0;          // reads of that row since the change (saturates at 2)
  wire       applied;
  wire [7:0] seen_now = seen | (8'd1 << row_sel);
  always @(posedge clk_sys) begin
    if (reset || applied) begin
      seen <= 8'd0; full <= 2'd0; act <= 1'b0; since <= 0; krow_n <= 2'd0;
      if (applied) krow <= e[7:5];
    end else begin
      if (row_rd && row_sel == krow && krow_n != 2'd2) krow_n <= krow_n + 2'd1;
      if (since != 32'hFFFFFFFF) since <= since + 1;
      if (row_rd) begin
        act <= 1'b1;
        if (seen_now == 8'hFF) begin
          seen <= 8'd0;
          if (full != 2'd2) full <= full + 2'd1;
        end else seen <= seen_now;
      end
    end
  end
  wire read_out = (full == 2'd2) || (krow_n == 2'd2) || (act && since >= PART) || (since >= IDLE);
  assign applied = !reset && t == 0 && !q_empty && q_ne && read_out;

  always @(posedge clk_sys) begin
    q_ne <= !q_empty;
    if (reset) begin
      for (int i = 0; i < 8; i++) mat[i] <= 8'd0;
      rp <= 0; t <= 0; shf <= 2'd0; shf_pos <= NONE; q_ne <= 1'b0;
    end else begin
      if (t != 0) t <= t - 1;
      else if (applied) begin
        rp <= rp + 10'd1;
        t  <= GAP;
        mat[e[7:5]][e[4:2]] <= e[9];
        if (e[9]) begin
          shf <= e[1:0];
          shf_pos <= e[8:2];
        end else if (e[8:2] == shf_pos) begin
          shf <= 2'd0;
          shf_pos <= NONE;
        end
      end
    end
  end

  wire vg_shift = shf[1] ? shf[0] : pc_shift;

  logic [7:0] r;
  always_comb begin
    r = mat[row_sel];
    if (row_sel == 3'd0) r[2] = vg_shift;
  end
  assign row_data = ~r;

endmodule
