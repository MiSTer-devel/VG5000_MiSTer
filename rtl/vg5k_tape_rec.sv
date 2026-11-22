//-----------------------------------------------------------------------------
// VG5000 tape recorder: what CSAVE writes on port AF D0 becomes .k7 bytes,
// written through an hps_io virtual disk (OSD "Tape save").
// File writer from SC3000_MiSTer rtl/sc3000_tape_rec.sv; new decoder.
//
// The ROM writer (3AA9-3AF2) sends each byte as start bit 0, 8 data bits LSB
// first, stop bits 1 1, after a leader of 1 bits. A bit is one long square
// cycle (0) or two short cycles (1), at 1200 or 2400 baud. Full cycles are
// timed in CPU clocks (turbo-proof), and
// classified against 1.5 x the running short-cycle length, which is learnt
// from the leader (all short cycles) at each motor start, so both baud rates
// work. The ROM's cycle is: wait, OUT 03 (high), wait, OUT 02 (low), so a
// cycle is timed from falling edge to falling edge (rising edges would mix
// the halves of two cycles). A 0 after at least two 1s is a start bit; the 8 bits after it are a
// byte; leader and stop bits are not stored. The bytes (D3 x10, header,
// D6 x10, data, 00 x10) are exactly a .k7 file.
//
// The file: bytes from offset 0 (every mount starts at 0), 512-byte sectors.
// Main can't grow a mounted image: mount a big enough empty file (any
// content; the rest of the last sector is written as 00, which the player
// skips). When the motor stops, the sector in use is written out.
//-----------------------------------------------------------------------------

module vg5k_tape_rec (
  input             clk,
  input             ce_cpu,        // CPU T-state enable (turbo included)
  input             motor,         // port AF D1
  input             tape_out,      // port AF D0

  input             img_mounted,
  input             img_readonly,
  input      [63:0] img_size,
  output reg [31:0] sd_lba = 0,
  output reg        sd_wr = 0,
  input             sd_ack,
  input       [8:0] sd_buff_addr,
  output reg  [7:0] sd_buff_din,

  output reg        active = 0,    // a writable file is mounted
  output reg        full = 0,
  output reg [24:0] length = 0     // bytes recorded since the mount
);

  reg  [24:0] limit = 0;

  // ---------------------------------------------------------------- two-bank sector buffer
  reg  [7:0] buffer [0:1023];
  reg  [9:0] wa;
  reg  [7:0] wd;
  reg        we = 0;
  reg        out_bank;
  reg  [8:0] out_fill;          // bytes valid in it (0 = all 512)
  always @(posedge clk) begin
    if (we) buffer[wa] <= wd;
    sd_buff_din <= (out_fill != 0 && sd_buff_addr >= out_fill) ? 8'h00 : buffer[{out_bank, sd_buff_addr}];
  end

  // ---------------------------------------------------------------- decoder
  reg        lvl_q = 0;
  reg [15:0] t = 0;             // CPU clocks since the last falling edge
  reg [15:0] shrt = 0;          // short-cycle estimate (0 = learn from the next cycle)
  reg        seen_edge = 0;     // a falling edge since the motor started
  reg        half = 0;          // first short cycle of a 1 seen
  reg [3:0]  ones = 0;          // consecutive 1 bits (saturates)
  reg        in_byte = 0;
  reg [3:0]  nbits = 0;
  reg [7:0]  sh = 0;
  reg        emit = 0;
  reg [7:0]  emit_b;
  reg        motor_q = 0;
  wire [16:0] thr = {1'b0, shrt} + {2'b0, shrt[15:1]};   // 1.5 x short

  task automatic got_bit(input b);
    if (in_byte) begin
      sh <= {b, sh[7:1]};
      if (nbits == 4'd7) begin in_byte <= 1'b0; emit <= 1'b1; emit_b <= {b, sh[7:1]}; ones <= 0; end
      nbits <= nbits + 4'd1;
    end else if (!b) begin
      if (ones >= 4'd2) begin in_byte <= 1'b1; nbits <= 0; end
      ones <= 0;
    end else if (ones != 4'hF) ones <= ones + 4'd1;
  endtask

  always @(posedge clk) begin
    emit <= 1'b0;
    motor_q <= motor;
    if (img_mounted || (motor && !motor_q)) begin
      t <= 0; shrt <= 0; seen_edge <= 0; half <= 0; ones <= 0; in_byte <= 0;
    end else if (ce_cpu && motor) begin
      lvl_q <= tape_out;
      if (t != 16'hFFFF) t <= t + 16'd1;
      if (!tape_out && lvl_q) begin
        t <= 0;
        seen_edge <= 1'b1;
`ifdef VG5K_REC_DEBUG
        $display("rec edge t=%0d shrt=%0d half=%0d ones=%0d in_byte=%0d nbits=%0d", t, shrt, half, ones, in_byte, nbits);
`endif
        if (seen_edge) begin
          if (shrt == 0) shrt <= t;                               // first leader cycle
          else if ({1'b0, t} < thr) begin                         // short
            shrt <= shrt - (shrt >> 3) + (t >> 3);                // follow slow drift
            if (half) begin half <= 1'b0; got_bit(1'b1); end
            else half <= 1'b1;
          end else if ({1'b0, t} < {thr, 1'b0}) begin             // long (up to 3 x short)
            half <= 1'b0; got_bit(1'b0);
          end else begin                                          // a gap: start over
            half <= 1'b0; ones <= 0; in_byte <= 1'b0;
          end
        end
      end
    end
  end

  // ---------------------------------------------------------------- file writer
  reg        flush_req = 0;
  reg [1:0]  pend = 0;
  reg [31:0] pend_lba [0:1];
  reg [8:0]  pend_fill [0:1];
  reg        ack_q = 0;
  reg        busy = 0;

  always @(posedge clk) begin
    we <= 1'b0;
    ack_q <= sd_ack;
    if (!motor && motor_q) flush_req <= 1'b1;
    if (img_mounted) begin
      active    <= ~img_readonly & |img_size;
      limit     <= (|img_size[63:25]) ? 25'h1FFFFFF : img_size[24:0];
      length    <= 0;
      full      <= 0;
      pend      <= 0;
      flush_req <= 0;
    end else begin
      if (emit && active && !full) begin
        wa <= length[9:0]; wd <= emit_b; we <= 1'b1;
        length <= length + 25'd1;
        if (length + 25'd1 >= limit) full <= 1'b1;
        if (length[8:0] == 9'd511) begin
          pend[length[9]]      <= 1'b1;
          pend_lba[length[9]]  <= {16'd0, length[24:9]};
          pend_fill[length[9]] <= 9'd0;
        end
      end
      if (flush_req && !we && !emit) begin
        flush_req <= 1'b0;
        if (length[8:0] != 0) begin
          pend[length[9]]      <= 1'b1;
          pend_lba[length[9]]  <= {16'd0, length[24:9]};
          pend_fill[length[9]] <= length[8:0];
        end
      end
      if (!busy && !sd_wr) begin
        if (pend[0] || pend[1]) begin
          out_bank <= pend[0] ? 1'b0 : 1'b1;
          out_fill <= pend[0] ? pend_fill[0] : pend_fill[1];
          sd_lba   <= pend[0] ? pend_lba[0] : pend_lba[1];
          sd_wr    <= 1'b1;
          busy     <= 1'b1;
        end
      end
      if (sd_wr && sd_ack) sd_wr <= 1'b0;
      if (busy && ack_q && !sd_ack) begin
        busy <= 1'b0;
        pend[out_bank] <= 1'b0;
      end
    end
  end

endmodule
