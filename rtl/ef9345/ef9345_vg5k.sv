// ---------------------------------------------------------------------------------
// ef9345_vg5k - EF9345 as wired on the Philips VG5000, for the VG5000 MiSTer core.
//
// Wrapper around the EF9345 core (Verilog translation of D. Laurens' orpigame
// VHDL EF9345, fixed and retimed to the native 15 kHz raster in 2026).
//
// Copyright D. Laurens (EF9345 core). This source describes Open Hardware and is
// licensed under the CERN-OHL-S v2 (see LICENSE in this directory).
// 2026: written for the VG5000 MiSTer core (new file; derived work of the core).
// ---------------------------------------------------------------------------------
//
// Bus (VG5000 schematic, Mokona redraw v1.5: 74LS138 7807 on A5/A6/A7, /WR, /IORQ):
//   OUT 8F  -> Y0 -> inverter -> AS   (address latch, Intel mode)
//   OUT CF  -> Y2 -> R/W pin          (= /WR strobe in Intel mode)
//   IN  CF  -> Y6 -> DS pin           (= /RD strobe in Intel mode)
//   CS_n tied low; AD0-7 on the Z80 data bus. MAME vg5k.cpp: 8F = offset latch,
//   CF = data_r/data_w(offset). The address byte is 0x20 + Ri (+ 8 = execute).
// The machine gives one-clock pulses (or levels: only the rising edge counts);
// this wrapper replays them on the core pins
// as Intel bus cycles (AS high then low with the address, R/W low with the data,
// DS low for a read). The core samples the pins at 24 MHz with edge detection,
// so each strobe lasts STROBE clocks and the bus value is held STROBE more
// clocks after it. A cycle takes 2*STROBE+2 clocks (34 at 48 MHz = 0.71 us);
// one more request arriving meanwhile is queued. dout is the core's data
// output register: it is updated about 6 clocks after data_rd (STATUS with
// the Busy bit S7, R1..R7) and then holds its value until the next read.
//
// Video: native EF9345 raster, see README.md in this directory.
//   clk = 48 MHz, ce_pix = 24 MHz (every other clock) in both modes:
//   40 columns: 8 dots x 3 = 24 ce_pix per character (8 MHz dot clock),
//   80 columns: 6 dots x 2 = 12 ce_pix per character (12 MHz dot clock).
//   Line 64 us = 1536 ce_pix, of which 1247 visible (margin 6 us + bulk
//   40 us + margin 6 us), hsync 4.5 us (active high here).
//   Frame 312 lines (50.08 Hz), 284 visible (margin 16 + page 250 + margin 18),
//   vsync 2 lines (active high, starts at the hsync leading edge).
// irq_n: the PC/VS pin (TGS4 = 0 on the VG5000: V sync), which drives the Z80
//   /INT through 7811 (AND with INT EX): low for the 2 VS lines (128 us) once
//   per frame (VG5000 Technical Manual 1.3: "frame blank start signal").
// ---------------------------------------------------------------------------------

module ef9345_vg5k #(
  parameter logic [8:0] FIXES = 9'h1FF   // fixes of the original, see EF9345.sv (keep the default)
) (
  input  logic       clk,        // clk_sys 48 MHz
  input  logic       reset,
  // Z80 side, already decoded by the machine (VG5000: OUT 8F = address
  // latch / AS, OUT CF = data write, IN CF = data read; partial decode
  // is done by the machine)
  input  logic       addr_wr,    // 1-clk pulse: latch din as EF9345 address (OUT 8F)
  input  logic       data_wr,    // 1-clk pulse: write din (OUT CF)
  input  logic       data_rd,    // 1-clk pulse at start of IN CF; dout valid while rd held
  input  logic [7:0] din,
  output logic [7:0] dout,
  // video, native EF9345 PAL raster (15.625 kHz / 50 Hz)
  output logic       ce_pix,     // pixel enable, 24 MHz, constant in every mode
  output logic       r, g, b,    // 1-bit RGB (8 colours)
  output logic       hsync, vsync, hblank, vblank,
  output logic       irq_n       // to Z80 /INT: frame interrupt as on the VG5000
);

localparam int STROBE = 16;

// ------------------------------------------------------------------
// Bus cycle sequencer
// ------------------------------------------------------------------
localparam logic [1:0] OP_ADDR = 2'd0, OP_WR = 2'd1, OP_RD = 2'd2;

// (no declaration initialisers: the control state is reset, the data
//  registers op / bus / pend_op / pend_din are only read after being loaded)
logic       busy_q;
logic [5:0] cnt;
logic [1:0] op;
logic [7:0] bus;
logic       pend;
logic [1:0] pend_op;
logic [7:0] pend_din;
logic       pin_as;   // AS, active high (Intel)
logic       pin_ds;   // DS = /RD (Intel)
logic       pin_rw;   // R/W = /WR (Intel)

// Requests are taken on the rising edge of addr_wr / data_wr / data_rd, so the
// machine may give one-clock pulses or hold the strobe for the whole I/O cycle.
// din is sampled on that first clock (the Z80 data are valid with /WR).
logic addr_wr_d, data_wr_d, data_rd_d;
logic [1:0] req_op;
logic req, rq_a, rq_w, rq_r;
always_comb begin
  rq_a   = addr_wr & ~addr_wr_d;
  rq_w   = data_wr & ~data_wr_d;
  rq_r   = data_rd & ~data_rd_d;
  req    = rq_a | rq_w | rq_r;
  req_op = rq_a ? OP_ADDR : rq_w ? OP_WR : OP_RD;
end

always_ff @(posedge clk) begin
  addr_wr_d <= addr_wr;
  data_wr_d <= data_wr;
  data_rd_d <= data_rd;
  if (reset) begin
    addr_wr_d <= 1'b0;
    data_wr_d <= 1'b0;
    data_rd_d <= 1'b0;
    busy_q <= 1'b0;
    pend   <= 1'b0;
    cnt    <= '0;
    pin_as <= 1'b0;
    pin_ds <= 1'b1;
    pin_rw <= 1'b1;
  end else begin
    if (!busy_q && (pend || req)) begin
      // start the queued request first, else the new one
      busy_q <= 1'b1;
      cnt    <= '0;
      op     <= pend ? pend_op  : req_op;
      bus    <= pend ? pend_din : din;
      pend   <= pend && req;            // a new request waits behind the queued one
      if (pend && req) begin
        pend_op  <= req_op;
        pend_din <= din;
      end
    end else if (req) begin
      // a cycle is running: queue (a third request overwrites the queued one)
      pend     <= 1'b1;
      pend_op  <= req_op;
      pend_din <= din;
    end

    if (busy_q) begin
      cnt <= cnt + 6'd1;
      // strobe: cnt 0 .. STROBE-2; AD held until the end of the cycle
      pin_as <= (op == OP_ADDR) && (cnt < 6'(STROBE - 1));
      pin_rw <= !((op == OP_WR) && (cnt < 6'(STROBE - 1)));
      pin_ds <= !((op == OP_RD) && (cnt < 6'(2*STROBE - 1)));
      if (cnt == 6'(2*STROBE))
        busy_q <= 1'b0;
    end
  end
end

// ------------------------------------------------------------------
// EF9345 core
// ------------------------------------------------------------------
logic core_r, core_g, core_b, core_hs_n, core_vs_n;
logic pix_tick, h_vis, v_vis;
logic [7:0] core_dout;
/* verilator lint_off UNUSEDSIGNAL */
logic core_ins, core_hp;   // I (insert) and HP (4 MHz video clock) pins: not used on the VG5000
/* verilator lint_on UNUSEDSIGNAL */

EF9345 #(.FIXES(FIXES)) ef9345 (
  .Clk      (clk),
  .Reset    (reset),
  .CS_n     (1'b0),
  .DS       (pin_ds),
  .AS       (pin_as),
  .RW       (pin_rw),
  .Data_In  (bus),
  .BLUE     (core_b),
  .GREEN    (core_g),
  .RED      (core_r),
  .H_Sync_n (core_hs_n),
  .V_Sync_n (core_vs_n),
  .Overlay  (core_ins),
  .HP       (core_hp),
  .PixTick  (pix_tick),
  .HVisible (h_vis),
  .VVisible (v_vis),
  .Data_Out (core_dout)
);

assign dout = core_dout;

// ------------------------------------------------------------------
// Video outputs, all registered on the 24 MHz pixel enable
// ------------------------------------------------------------------
// The core updates its margin / window flags on the clock edges where PixTick
// is high, but its dot counters run at 48 MHz, so a dot may change one clock
// later. Sampling one clock after PixTick (samp) catches both: every 40-column
// dot is then exactly 3 pixels and every 80-column dot 2, with the character
// cells aligned on the margin edge (as the original XGA output sampled them).
logic samp;
logic hs_q;
always_ff @(posedge clk) begin
  samp <= pix_tick;
  if (samp) begin
    r      <= core_r & h_vis & v_vis;
    g      <= core_g & h_vis & v_vis;
    b      <= core_b & h_vis & v_vis;
    hblank <= ~h_vis;
    vblank <= ~v_vis;
    hs_q   <= ~core_hs_n;
    hsync  <= ~core_hs_n;
    // vsync changes at the leading edge of hsync
    if (~core_hs_n & ~hs_q)
      vsync <= ~core_vs_n;
  end
  irq_n <= core_vs_n;
end

assign ce_pix = samp;

endmodule
