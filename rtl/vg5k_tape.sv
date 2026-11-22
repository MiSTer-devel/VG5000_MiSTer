//-----------------------------------------------------------------------------
// VG5000 tape deck: plays a .k7 byte image or a .wav recording into the
// cassette input (port AF D7), from a byte port (DDR3 on the MiSTer, an array
// in the sim) with a req / ack handshake.
//
// .k7 player: regenerates exactly the waveform MAME src/lib/formats/
// vg5k_cas.cpp makes from the image (BSD-3, S. Ronco), at its 44.1 kHz sample
// grid:
//   - a D3 byte starts a header block of 32 bytes: 44100 samples of silence,
//     30000 short cycles of sync, then the end-of-byte mark
//   - a D6 byte starts a data block of (16-bit LE at the 4th..3rd byte before
//     it) + 20 bytes: 10000 samples of silence, 7200 short cycles + EOB
//   - anything else outside a block is skipped up to the next D3 / D6
//   - each byte: 8 bits LSB first, 1 = two short cycles (5 high + 5 low
//     samples), 0 = one long cycle (10 + 10), then the end-of-byte mark
//     (4 short cycles + 1 long = stop bits 1 1 and the next start bit 0;
//     the ROM writer at 3AA9 sends start 0, 8 data bits, stop 1 1)
//   - 10000 samples of silence at the end, then `ended`
//   Silence reads 0 (MAME: level 0 is not > 0.03). MAME refuses an image that
//   does not start D3 D3 D3; this player plays it anyway (skipping to a D3).
// .wav player: rtl/vg5k_wav.sv.
//
// Timebase: the CPU clock enable (rising phase) at 4 MHz, 441 / 40000 per
// enable = 44100 Hz, so turbo (CPU 4x) plays the tape 4x and the ROM's
// loader, which times bits with CPU loops after calibrating on the sync
// (3AF3), sees the same pulses (tape-standard rule 7).
// The deck plays while the motor relay (port AF D1) is on; it pauses in place.
// Turbo is requested while the deck plays, the tape has not ended, and Tape
// speed was 4x when the motor last started (tape-standard rules 1, 6).
//-----------------------------------------------------------------------------

module vg5k_tape (
  input             clk,
  input             reset,        // rewind (mount, Rewind tape)
  input             ce_cpu,       // CPU T-state enable (turbo-scaled)

  // the image as it downloads (format detection, header for the hint)
  input             dl,
  input             dl_wr,
  input      [24:0] dl_addr,
  input      [7:0]  dl_data,

  input      [24:0] size,         // image length, 0 = no tape
  input             motor,
  input             turbo_osd,    // Tape speed = 4x
  output            turbo_req,

  output     [24:0] rd_addr,
  output            rd_req,
  input             rd_ack,
  input      [7:0]  rd_data,

  output            level,
  output            ended,
  output     [24:0] position,     // bytes consumed
  output reg        is_wav = 1'b0,
  output            playing,      // deck plays (motor on, tape loaded, not ended)
  output            moving,       // samples are advancing now
  output reg        sec_tick = 1'b0, // one pulse per second of tape time played
  output reg        hdr_valid = 1'b0,
  output reg [7:0]  hdr_type = 8'd0,
  output reg [47:0] hdr_name = 48'd0,
  output reg        hdr_autorun = 1'b0, // header byte 12h != 0: CLOAD runs the program itself
  output reg [7:0]  file_count = 8'd0   // D3 headers seen by the player so far
);

  // ------------------------------------------------------------ format, header
  always @(posedge clk) if (dl_wr && dl_addr < 25'd4)
    case (dl_addr[1:0])
      2'd0: is_wav <= dl_data == "R";
      2'd1: is_wav <= is_wav && dl_data == "I";
      2'd2: is_wav <= is_wav && dl_data == "F";
      2'd3: is_wav <= is_wav && dl_data == "F";
    endcase
  always @(posedge clk) begin
    if (dl && dl_addr == 25'd0 && dl_wr) hdr_valid <= 1'b0;
    if (dl_wr && dl_addr == 25'd10) begin hdr_type <= dl_data; hdr_valid <= 1'b1; end
    if (dl_wr && dl_addr == 25'd18) hdr_autorun <= dl_data != 8'h00;   // ASCII start line ("10")
    if (dl_wr && dl_addr >= 25'd11 && dl_addr <= 25'd16)
      hdr_name <= {hdr_name[39:0], dl_data};
  end

  wire loaded = size != 25'd0 && !dl;

  // ------------------------------------------------------------ 44.1 kHz grid
  reg  [15:0] acc = 16'd0;
  reg         tick;
  wire        run;
  always @(posedge clk) begin
    tick <= 1'b0;
    if (reset) acc <= 16'd0;
    else if (ce_cpu && run) begin
      if (acc + 16'd441 >= 16'd40000) begin acc <= acc + 16'd441 - 16'd40000; tick <= 1'b1; end
      else acc <= acc + 16'd441;
    end
  end
  reg [15:0] sec_cnt = 16'd0;
  always @(posedge clk) begin
    sec_tick <= 1'b0;
    if (reset) sec_cnt <= 16'd0;
    else if (tick) begin
      if (sec_cnt == 16'd44099) begin sec_cnt <= 16'd0; sec_tick <= 1'b1; end
      else sec_cnt <= sec_cnt + 16'd1;
    end
  end

  // ------------------------------------------------------------ .k7 player
  // byte prefetch
  reg  [24:0] pos = 25'd0;        // next byte to fetch
  reg  [7:0]  nb;                 // prefetched byte
  reg         nb_v = 1'b0;
  reg         k_req = 1'b0;
  wire        k_at_end = pos >= size;

  localparam [2:0] S_IDLE = 3'd0, S_DECIDE = 3'd1, S_SIL = 3'd2, S_SYNC = 3'd3,
                   S_BYTE = 3'd4, S_EOB = 3'd5, S_TAIL = 3'd6, S_END = 3'd7;
  reg  [2:0]  st = S_END;
  reg  [15:0] n;                  // samples / cycles left in this phase
  reg  [16:0] blk = 17'd0;        // bytes left in the block
  reg  [7:0]  hist [0:3];         // last bytes consumed: hist[3] = 4 back
  reg  [7:0]  cur;
  reg  [2:0]  bit_i;
  reg         half2;              // second short cycle of a '1'
  reg  [2:0]  eob_i;
  reg         sync_hdr;           // the sync in progress is a header's (30000) or data's (7200)
  // cycle generator: len samples high then len low
  reg  [4:0]  cyc_n = 5'd0;       // samples left in the current cycle (2 x len .. 1)
  reg  [4:0]  cyc_len;
  reg         k_level = 1'b0;
  wire        cyc_busy = cyc_n != 5'd0;

  wire k_play = loaded && !is_wav && motor;
  assign run = is_wav ? (loaded && motor) : (k_play && st != S_END);

  // a byte is taken from the prefetch buffer
  // (re)start at the rewind, and when an image becomes loaded (the end of a
  // download: reset and !loaded drop together)
  reg  loaded_q = 1'b0;
  always @(posedge clk) loaded_q <= loaded;
  always @(posedge clk) begin
    if (reset || !loaded || !loaded_q || is_wav) begin
      pos <= 25'd0; nb_v <= 1'b0; k_req <= 1'b0;
      st <= (loaded && !is_wav) ? S_DECIDE : S_END;
      cyc_n <= 5'd0; k_level <= 1'b0; blk <= 17'd0; file_count <= 8'd0;
      for (int i = 0; i < 4; i++) hist[i] <= 8'd0;
    end else begin
      // prefetch
      if (!nb_v && !k_req && !k_at_end) k_req <= 1'b1;
      if (k_req && rd_ack && !is_wav) begin k_req <= 1'b0; nb <= rd_data; nb_v <= 1'b1; pos <= pos + 25'd1; end

      // cycle generator
      if (tick && cyc_busy) begin
        cyc_n   <= cyc_n - 5'd1;
        k_level <= (cyc_n - 5'd1) > {1'b0, cyc_len[3:0]} && (cyc_n - 5'd1) != 0 ? 1'b1 : 1'b0;
      end

      case (st)
        S_DECIDE: if (!cyc_busy) begin
          if (blk != 0) begin
            if (nb_v) begin
              cur <= nb; nb_v <= 1'b0; blk <= blk - 17'd1;
              hist[0] <= nb; hist[1] <= hist[0]; hist[2] <= hist[1]; hist[3] <= hist[2];
              bit_i <= 3'd0; half2 <= 1'b0; st <= S_BYTE;
            end else if (k_at_end && !k_req) begin n <= 16'd10000; st <= S_TAIL; end
          end else if (nb_v) begin
            if (nb == 8'hD3) begin
              blk <= 17'd32; n <= 16'd44100; sync_hdr <= 1'b1; st <= S_SIL;
              file_count <= file_count + 8'd1;
            end else if (nb == 8'hD6) begin
              blk <= {1'b0, hist[2], hist[3]} + 17'd20; n <= 16'd10000; sync_hdr <= 1'b0; st <= S_SIL;
            end else begin
              nb_v <= 1'b0;
              hist[0] <= nb; hist[1] <= hist[0]; hist[2] <= hist[1]; hist[3] <= hist[2];
            end
          end else if (k_at_end && !k_req) begin n <= 16'd10000; st <= S_TAIL; end
        end
        S_SIL: if (tick) begin
          k_level <= 1'b0;
          if (n == 16'd1) begin n <= sync_hdr ? 16'd30000 : 16'd7200; st <= S_SYNC; end
          else n <= n - 16'd1;
        end
        S_SYNC: if (!cyc_busy && !(tick && cyc_busy)) begin
          if (n == 16'd0) begin eob_i <= 3'd0; st <= S_EOB; end
          else begin n <= n - 16'd1; cyc_len <= 5'd5; cyc_n <= 5'd10; k_level <= 1'b1; end
        end
        S_BYTE: if (!cyc_busy) begin
          if (cur[bit_i]) begin
            cyc_len <= 5'd5; cyc_n <= 5'd10; k_level <= 1'b1;
            if (half2) begin
              half2 <= 1'b0;
              if (bit_i == 3'd7) begin eob_i <= 3'd0; st <= S_EOB; end
              bit_i <= bit_i + 3'd1;
            end else half2 <= 1'b1;
          end else begin
            cyc_len <= 5'd10; cyc_n <= 5'd20; k_level <= 1'b1;
            if (bit_i == 3'd7) begin eob_i <= 3'd0; st <= S_EOB; end
            bit_i <= bit_i + 3'd1;
          end
        end
        S_EOB: if (!cyc_busy) begin
          k_level <= 1'b1;
          if (eob_i < 3'd4) begin cyc_len <= 5'd5; cyc_n <= 5'd10; eob_i <= eob_i + 3'd1; end
          else begin cyc_len <= 5'd10; cyc_n <= 5'd20; st <= S_DECIDE; end
        end
        S_TAIL: if (tick && !cyc_busy) begin
          k_level <= 1'b0;
          if (n == 16'd1) st <= S_END; else n <= n - 16'd1;
        end
        default: ;
      endcase
    end
  end

  // ------------------------------------------------------------ .wav player
  wire [24:0] w_addr, w_pos;
  wire        w_req, w_tape, w_running, w_eot;
  vg5k_wav wav (
    .clock(clk), .rewind(reset || !loaded || !loaded_q), .cpu_ce(ce_cpu), .play(motor),
    .size(is_wav && loaded ? size : 25'd0),
    .rd_addr(w_addr), .rd_req(w_req), .rd_ack(rd_ack && is_wav), .rd_data(rd_data),
    .tape(w_tape), .running(w_running), .eot(w_eot), .pos(w_pos),
    .valid(), .rate(), .data_len(), .align());

  // ------------------------------------------------------------ outputs
  assign rd_addr  = is_wav ? w_addr : pos;
  assign rd_req   = is_wav ? w_req : k_req;
  assign level    = is_wav ? w_tape : k_level;
  assign ended    = is_wav ? (loaded && w_eot) : (loaded && st == S_END);
  assign position = is_wav ? w_pos : pos;
  assign playing  = loaded && motor && !ended;
  assign moving   = playing && run;

  reg turbo_sel = 1'b0, motor_q = 1'b0;
  always @(posedge clk) begin
    motor_q <= motor;
    if (motor && !motor_q) turbo_sel <= turbo_osd;
  end
  assign turbo_req = turbo_sel && playing;

endmodule
