//-----------------------------------------------------------------------------
// Philips VG5000µ machine (shared by the MiSTer top VG5000.sv and the
// simulation top verilator/vg5000_sim.sv).
//
//   Z80A    t80sv_T80pa, 4 MHz (CE from vg5k_clocks), one wait state on every
//           M1 (Technical Manual p4 "a wait state on every M1"; MAME vg5k.cpp
//           z80_m1_w adjusts icount by -1 per opcode fetch)
//   Video   EF9345 (rtl/ef9345/ef9345_vg5k.sv), 8 KB private
//           VRAM inside it; its frame interrupt drives /INT (TM p4, p18)
//   ROM     16 KB at 0000-3FFF (BASIC 1.1 / 1.0), loaded through rom_wr
//   RAM     16 KB at 4000-7FFF (screen buffer 4000-47CF, system variables to
//           49FB). Expansion (MAME RAM option, VG5216 = 16 KB): mem_size 1 =
//           32 KB total (8000-BFFF), 2 = 48 KB total (8000-FFFF).
//           Unmapped memory reads FF (MAME unmap_value_high).
//
// I/O (Formation Technique p10-11: a 74LS138 on A7 A6 A5 with /RD /WR,
// enabled by /IORQ and A7 = 1, so 80-FF are partially decoded; 00-7F belong
// to the extension bus). MAME vg5k_io uses the conventional addresses:
//   read  80-9F  keyboard row A2:A0 (80-87 conventional), active low
//   read  A0-BF  cassette in (AF): MAME returns FF / 00 from the level; the
//                ROM only looks at D7
//   read  C0-DF  EF9345 data (CF)
//   write 80-9F  EF9345 address latch (8F)
//   write A0-BF  AF latch: D0 cassette out, D1 motor relay, D3 beeper (FT p13)
//   write C0-DF  EF9345 data (CF)
//   write E0-FF  NMI diode, international boards only: this is a French
//                board, nothing happens (as MAME); the ROM's write to EF at
//                reset then leaves the French messages and keyboard table
//   read  07/08  joysticks (VG5200 / VG5216), active low: b0 up, b1 right,
//                b2 down, b3 left, b4 action 1, b5 action 2 (bulletin 0007)
//   read  10     printer status: 00 = ready (printer stub: always ready)
//   write 10/11  printer strobe / data: ignored
//   anything else in 00-7F reads FF
//-----------------------------------------------------------------------------

module vg5000 (
  input               clk,            // 48 MHz
  input               reset,          // active high, synchronous
  input               ce_cpu_p,
  input               ce_cpu_n,
  input               turbo,          // the CPU runs 4x (tape turbo): pace EF9345 accesses
  input        [1:0]  mem_size,       // 0 = 16 KB, 1 = 32 KB, 2 = 48 KB

  input               rom_wr,         // ROM download (16 KB)
  input        [13:0] rom_addr,
  input        [7:0]  rom_data,

  // keyboard: the row the CPU reads, its 8 keys (active low), read pulse
  output       [2:0]  kbd_row,
  input        [7:0]  kbd_data,
  output              kbd_rd,         // 1-clock pulse when the CPU starts reading a row
  input               nmi_key,        // Δ key, 1 = pressed
  input        [5:0]  joy0,           // 1 = pressed {action2, action1, left, down, right, up}
  input        [5:0]  joy1,

  input               tape_in,
  output              tape_out,
  output              motor,
  output              beeper,

  // video (EF9345)
  output              ce_pix,
  output              vid_r, vid_g, vid_b,
  output              hsync, vsync, hblank, vblank,

  // debug taps
  output       [15:0] dbg_a,
  output       [7:0]  dbg_di,
  output       [7:0]  dbg_do,
  output              dbg_m1_n,
  output              dbg_mreq_n,
  output              dbg_iorq_n,
  output              dbg_rd_n,
  output              dbg_wr_n,
  output              dbg_rfsh_n,
  output              dbg_int_n,
  output       [211:0] dbg_reg
);

  wire        reset_n = ~reset;

  // CPU ---------------------------------------------------------------------
  wire [15:0] a;
  wire [7:0]  d_from_cpu;
  logic [7:0] d_to_cpu;
  wire        m1_n, mreq_n, iorq_n, rd_n, wr_n, rfsh_n, halt_n;
  wire        irq_n;
  wire        wait_n;

  t80sv_T80pa #(.Mode(0)) cpu (
    .RESET_n(reset_n), .CLK(clk), .CEN_p(ce_cpu_p), .CEN_n(ce_cpu_n),
    .WAIT_n(wait_n), .INT_n(irq_n), .NMI_n(~nmi_key), .BUSRQ_n(1'b1),
    .OUT0(1'b0), .DI(d_to_cpu), .R800_mode(1'b0), .DIRSet(1'b0), .DIR(212'd0),
    .M1_n(m1_n), .MREQ_n(mreq_n), .IORQ_n(iorq_n), .RD_n(rd_n), .WR_n(wr_n),
    .RFSH_n(rfsh_n), .HALT_n(halt_n), .BUSAK_n(), .A(a), .DO(d_from_cpu), .REG(dbg_reg));

  // M1 wait state: /WAIT is low for exactly one T2 sample of every opcode
  // fetch (and of the interrupt acknowledge M1), giving 5 T-states per M1.
  // t80sv samples WAIT_n at the falling phase (ce_cpu_n) of T2.
  wire fetch = ~m1_n & rfsh_n & (~mreq_n | ~iorq_n);
  reg  m1_waited = 1'b0;
  always @(posedge clk) begin
    if (reset || !fetch) m1_waited <= 1'b0;
    else if (ce_cpu_n) m1_waited <= 1'b1;
  end
  wire m1_wait_n = ~(fetch & ~m1_waited);

  // Decode ------------------------------------------------------------------
  wire mem   = ~mreq_n & rfsh_n;
  wire io    = ~iorq_n & m1_n;
  wire io_rd = io & ~rd_n;
  wire io_wr = io & ~wr_n;
  wire io_hi = io & a[7];
  wire [1:0] grp = a[6:5];

  wire rom_sel = a[15:14] == 2'b00;
  wire ram_sel = a[15:14] == 2'b01;
  wire exp_sel = (a[15:14] == 2'b10 && mem_size != 2'd0) ||
                 (a[15:14] == 2'b11 && mem_size == 2'd2);

  // ROM ---------------------------------------------------------------------
  reg  [7:0] rom [0:16383];
  reg  [7:0] rom_q;
  always @(posedge clk) begin
    if (rom_wr) rom[rom_addr] <= rom_data;
    rom_q <= rom[a[13:0]];
  end

  // RAM: 16 KB internal + 32 KB expansion ------------------------------------
  reg  [7:0] ram [0:16383];
  reg  [7:0] ram_q;
  always @(posedge clk) begin
    if (mem & ram_sel & ~wr_n) ram[a[13:0]] <= d_from_cpu;
    ram_q <= ram[a[13:0]];
  end
  reg  [7:0] xram [0:32767];
  reg  [7:0] xram_q;
  always @(posedge clk) begin
    if (mem & exp_sel & ~wr_n) xram[a[14:0]] <= d_from_cpu;
    xram_q <= xram[a[14:0]];
  end

  // Port AF latch -------------------------------------------------------------
  reg [7:0] af = 8'h00;
  always @(posedge clk) begin
    if (reset) af <= 8'h00;
    else if (io_hi & io_wr & grp == 2'b01) af <= d_from_cpu;
  end
  assign tape_out = af[0];
  assign motor    = af[1];
  assign beeper   = af[3];

  // EF9345 -------------------------------------------------------------------
  wire ef_as_w = io_hi & io_wr & grp == 2'b00;
  wire ef_ds_w = io_hi & io_wr & grp == 2'b10;
  wire ef_ds_r = io_hi & io_rd & grp == 2'b10;
  reg  ef_as_q = 1'b0, ef_ds_w_q = 1'b0, ef_ds_r_q = 1'b0;
  always @(posedge clk) begin
    ef_as_q   <= ef_as_w;
    ef_ds_w_q <= ef_ds_w;
    ef_ds_r_q <= ef_ds_r;
  end
  // Under turbo the CPU would issue EF9345 commands 4x faster than the
  // chip finishes them (the ROM's first-row screen copy at 00E0 does not poll
  // the busy flag). While turbo is on, each EF9345 access waits until 6 µs
  // have passed since the previous one (MAME: KRG busy 5.5 µs).
  wire ef_acc = io_hi & (io_rd | io_wr) & (grp == 2'b00 || grp == 2'b10);
  reg  [8:0] ef_gap = 9'd0;
  reg        ef_acc_q = 1'b0;
  always @(posedge clk) begin
    ef_acc_q <= ef_acc & ef_ok;
    if (ef_acc & ef_ok & ~ef_acc_q) ef_gap <= 9'd288;
    else if (ef_gap != 0) ef_gap <= ef_gap - 9'd1;
  end
  wire ef_ok   = ~turbo | ef_gap == 9'd0 | ef_acc_q;
  wire ef_wait_n = ~(ef_acc & ~ef_ok);
  assign wait_n = m1_wait_n & ef_wait_n;

  wire [7:0] ef_q;
  ef9345_vg5k video (
    .clk(clk), .reset(reset),
    .addr_wr(ef_as_w & ef_ok & ~ef_as_q), .data_wr(ef_ds_w & ef_ok & ~ef_ds_w_q),
    .data_rd(ef_ds_r & ef_ok & ~ef_ds_r_q), .din(d_from_cpu), .dout(ef_q),
    .ce_pix(ce_pix), .r(vid_r), .g(vid_g), .b(vid_b),
    .hsync(hsync), .vsync(vsync), .hblank(hblank), .vblank(vblank),
    .irq_n(irq_n));

  // Keyboard -------------------------------------------------------------------
  wire kbd_sel = io_hi & io_rd & grp == 2'b00;
  reg  kbd_q = 1'b0;
  always @(posedge clk) kbd_q <= kbd_sel;
  assign kbd_row = a[2:0];
  assign kbd_rd  = kbd_sel & ~kbd_q;

  // Data to the CPU ------------------------------------------------------------
  always_comb begin
    d_to_cpu = 8'hFF;
    if (mem) d_to_cpu = rom_sel ? rom_q : ram_sel ? ram_q : exp_sel ? xram_q : 8'hFF;
    else if (io_rd) begin
      if (a[7]) case (grp)
        2'b00: d_to_cpu = kbd_data;
        2'b01: d_to_cpu = {8{tape_in}};
        2'b10: d_to_cpu = ef_q;
        default: d_to_cpu = 8'hFF;
      endcase
      else case (a[7:0])
        8'h07: d_to_cpu = {2'b11, ~joy0};
        8'h08: d_to_cpu = {2'b11, ~joy1};
        8'h10: d_to_cpu = 8'h00;
        default: d_to_cpu = 8'hFF;
      endcase
    end
  end

  assign dbg_a      = a;
  assign dbg_di     = d_to_cpu;
  assign dbg_do     = d_from_cpu;
  assign dbg_m1_n   = m1_n;
  assign dbg_mreq_n = mreq_n;
  assign dbg_iorq_n = iorq_n;
  assign dbg_rd_n   = rd_n;
  assign dbg_wr_n   = wr_n;
  assign dbg_rfsh_n = rfsh_n;
  assign dbg_int_n  = irq_n;

endmodule
