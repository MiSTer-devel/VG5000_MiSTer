// Simulation top for the VG5000 core: rtl/vg5000.sv + the tape / keyboard
// modules, without sys/ or hps_io. clk = clk_sys (48 MHz) with the hardware
// clock enables (rtl/vg5k_clocks.sv), so tape turbo works as on the MiSTer.
//
// Downloads (ioctl-like): index 0 = ROM, 1 = tape image (.k7 or .wav, the
// player is chosen from the first bytes as in VG5000.sv).

module vg5000_sim #(
  parameter TAPE_AW = 23              // 8 MB tape image (hardware: DDR3)
) (
  input               clk,
  input               reset,

  input               dl_busy,
  input               dl_wr,
  input        [7:0]  dl_index,
  input        [24:0] dl_addr,
  input        [7:0]  dl_data,

  input        [1:0]  mem_size,

  // keyboard matrix from the harness: bit row*8 + col, 1 = pressed (MAME ROW1..ROW8)
  input        [63:0] key_matrix,
  input               use_ps2,          // 1: rtl/vg5k_keyboard.sv fed with ps2_key
  input               pc_layout,
  input        [10:0] ps2_key,
  input        [5:0]  joy0,
  input        [5:0]  joy1,
  input               nmi_key,

  input               tape_rewind,
  input               tape_turbo,       // OSD Tape speed = 4x
  output              turbo_active,
  output              tape_level,
  output              tape_motor,
  output              tape_out,
  output              tape_ended,
  output       [24:0] tape_pos,
  output              tape_is_wav,

  // Tape save (hps_io VD shape)
  input               rec_img_mounted,
  input        [31:0] rec_img_size,
  output       [31:0] rec_sd_lba,
  output              rec_sd_wr,
  input               rec_sd_ack,
  input        [8:0]  rec_sd_buff_addr,
  output       [7:0]  rec_sd_buff_din,
  output       [24:0] rec_length,

  output              R, G, B,
  output              HSync, VSync, HBlank, VBlank,
  output              ce_pix,
  output              beeper,

  output       [15:0] cpu_a,
  output       [7:0]  cpu_di,
  output       [7:0]  cpu_do,
  output              cpu_m1_n,
  output              cpu_mreq_n,
  output              cpu_iorq_n,
  output              cpu_rd_n,
  output              cpu_wr_n,
  output              cpu_rfsh_n,
  output              cpu_int_n,
  output              cpu_cen_p,
  output       [211:0] cpu_reg
);

  wire ce_cpu_p, ce_cpu_n, ce_4m, turbo_req;
  vg5k_clocks clocks (.clk(clk), .turbo_req(turbo_req), .ce_cpu_p(ce_cpu_p), .ce_cpu_n(ce_cpu_n),
                      .ce_4m(ce_4m), .turbo(turbo_active));
  assign cpu_cen_p = ce_cpu_p;

  // Tape image (index 1) -------------------------------------------------------
  reg  [7:0]  tape_mem [0:(1<<TAPE_AW)-1] /*verilator public*/;
  reg  [24:0] tape_size = 25'd0;
  wire        tape_dl = dl_busy && dl_index == 8'd1;
  always @(posedge clk) begin
    if (dl_wr && dl_index == 8'd1) begin
      tape_mem[dl_addr[TAPE_AW-1:0]] <= dl_data;
      tape_size <= dl_addr + 25'd1;
    end
  end
  // byte port with a req / ack handshake (as the DDR3 path on hardware): one
  // clock of latency plus a little jitter to catch players that do not wait
  wire [24:0] t_addr;
  wire        t_req;
  reg         t_ack = 1'b0;
  reg  [7:0]  t_q;
  reg  [1:0]  t_lat = 2'd0;
  always @(posedge clk) begin
    t_ack <= 1'b0;
    if (t_req && !t_ack) begin
      t_lat <= t_lat + 2'd1;
      if (t_lat == 2'd3) begin t_q <= tape_mem[t_addr[TAPE_AW-1:0]]; t_ack <= 1'b1; end
    end
  end

  wire motor;
  assign tape_motor = motor;
  wire tape_in;
  wire [24:0] t_pos;
  vg5k_tape tape (
    .clk(clk), .reset(tape_rewind | tape_dl), .ce_cpu(ce_cpu_p),
    .dl(tape_dl), .dl_wr(dl_wr && dl_index == 8'd1), .dl_addr(dl_addr), .dl_data(dl_data),
    .size(tape_dl ? 25'd0 : tape_size), .motor(motor), .turbo_osd(tape_turbo),
    .turbo_req(turbo_req), .rd_addr(t_addr), .rd_req(t_req), .rd_ack(t_ack), .rd_data(t_q),
    .level(tape_in), .ended(tape_ended), .position(t_pos), .is_wav(tape_is_wav),
    .playing(), .moving(), .sec_tick(), .hdr_valid(), .hdr_type(), .hdr_name(), .hdr_autorun(), .file_count());
  assign tape_level = tape_in;
  assign tape_pos = t_pos;

  vg5k_tape_rec tape_rec (.clk(clk), .ce_cpu(ce_cpu_p), .motor(motor), .tape_out(tape_out),
    .img_mounted(rec_img_mounted), .img_readonly(1'b0), .img_size({32'd0, rec_img_size}),
    .sd_lba(rec_sd_lba), .sd_wr(rec_sd_wr), .sd_ack(rec_sd_ack), .sd_buff_addr(rec_sd_buff_addr),
    .sd_buff_din(rec_sd_buff_din), .active(), .full(), .length(rec_length));

  // Keyboard ----------------------------------------------------------------------
  wire [2:0] kbd_row;
  wire       kbd_rd;
  wire [7:0] ps2_row_n;
  wire       ps2_nmi;
  vg5k_keyboard #(.CLK_HZ(48000000)) keyboard (
    .clk_sys(clk), .reset(reset), .pc_layout(pc_layout), .row_sel(kbd_row), .row_rd(kbd_rd),
    .ps2_key(ps2_key), .row_data(ps2_row_n), .nmi_key(ps2_nmi));
  wire [7:0] row_keys = use_ps2 ? ~ps2_row_n : key_matrix[kbd_row*8 +: 8];

  vg5000 machine (
    .clk(clk), .reset(reset), .ce_cpu_p(ce_cpu_p), .ce_cpu_n(ce_cpu_n), .turbo(turbo_active),
    .mem_size(mem_size),
    .rom_wr(dl_wr && dl_index == 8'd0), .rom_addr(dl_addr[13:0]), .rom_data(dl_data),
    .kbd_row(kbd_row), .kbd_data(~row_keys), .kbd_rd(kbd_rd), .nmi_key(nmi_key | (use_ps2 & ps2_nmi)),
    .joy0(joy0), .joy1(joy1),
    .tape_in(tape_in), .tape_out(tape_out), .motor(motor), .beeper(beeper),
    .ce_pix(ce_pix), .vid_r(R), .vid_g(G), .vid_b(B),
    .hsync(HSync), .vsync(VSync), .hblank(HBlank), .vblank(VBlank),
    .dbg_a(cpu_a), .dbg_di(cpu_di), .dbg_do(cpu_do), .dbg_m1_n(cpu_m1_n), .dbg_mreq_n(cpu_mreq_n),
    .dbg_iorq_n(cpu_iorq_n), .dbg_rd_n(cpu_rd_n), .dbg_wr_n(cpu_wr_n), .dbg_rfsh_n(cpu_rfsh_n),
    .dbg_int_n(cpu_int_n), .dbg_reg(cpu_reg));

  // harness access to the machine's memories (DPI, independent of Verilator's inlining)
  export "DPI-C" function sim_peek;
  function int sim_peek(input int a);
    if (a >= 32'h8000) return int'(machine.xram[a[14:0]]);
    if (a >= 32'h4000) return int'(machine.ram[a[13:0]]);
    return int'(machine.rom[a[13:0]]);
  endfunction

endmodule
