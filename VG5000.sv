//============================================================================
//
//  Philips VG5000µ (Radiola / Schneider VG5000) for MiSTer.
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  Machine: rtl/vg5000.sv (Z80 t80sv, ROM, RAM, I/O). Video: rtl/ef9345/
//  (EF9345, CERN-OHL-S v2, see rtl/ef9345/README.md).
//
//============================================================================

module emu
(
	`include "sys/emu_ports.vh"
);

///////// Default values for ports not used in this core /////////

assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;

assign VGA_F1 = 0;
assign VGA_SCALER = 0;
assign VGA_DISABLE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_S = 1;
assign AUDIO_MIX = 0;

assign LED_POWER = 0;
assign LED_DISK = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////
// OSD (osd-standard.md, video-standard.md, tape-standard.md)
//
// status bits: 0 reset, [3:2] Memory, [5] Tape speed, [6] Rewind tape (button),
// [7] Delta key (button), [9:8] Aspect ratio, [11:10] Scale, [14:12] Scandoubler Fx,
// [15] Tape overlay, [16] Keyboard layout, [17] Swap joysticks, [18] Tape audio.
// Machine settings (Memory) wait for Apply and reset; video, input and tape
// rows act at once.

`include "build_id.v"
localparam CONF_STR = {
	"VG5000;;",
	"-;",
	"F1,K7 WAV,Tape;",
	"S0,K7,Tape save;",
	"F0,ROMBIN,Load ROM;",
	"-;",
	"O[3:2],Memory,16 KB,32 KB,48 KB;",
	"-;",
	"P1,Machine;",
	"P2,Video;",
	"P3,Input;",
	"P4,Audio;",
	"-;",
	"h0-,Changes pending;",
	"R[0],Apply and reset;",

	"P1O[5],Tape speed,4x,Normal;",
	"P1-;",
	"P1T[6],Rewind tape;",
	"P1T[7],Delta key (NMI);",

	"P2O[9:8],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"P2O[11:10],Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
	"P2O[14:12],Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	"P2-;",
	"P2O[15],Tape overlay,On,Off;",

	"P3O[16],Keyboard,PC (US),AZERTY;",
	"P3O[17],Swap joysticks,No,Yes;",

	"P4O[18],Tape audio,Off,On;",

	"I,Tape ready (rewound),Tape format not supported;",
	"J,Action 1,Action 2;",
	"jn,A,B;",
	"v,0;",
	"V,v",`BUILD_DATE
};

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;      // 48 MHz
wire clk_vid;      // 96 MHz: video_mixer needs >= 4x the EF9345's 24 MHz ce_pix
wire pll_locked;

pll pll
(
	.refclk  (CLK_50M   ),
	.rst     (0         ),
	.outclk_0(clk_sys   ),
	.outclk_1(clk_vid   ),
	.outclk_2(          ),
	.locked  (pll_locked)
);

///////////////////////   HPS   //////////////////////////////////

wire         forced_scandoubler;
wire   [1:0] buttons;
wire [127:0] status;
wire  [21:0] gamma_bus;
wire  [10:0] ps2_key;
wire  [31:0] joystick_0, joystick_1;

wire         ioctl_download;
wire  [15:0] ioctl_index;
wire         ioctl_wr;
wire  [26:0] ioctl_addr;
wire   [7:0] ioctl_dout;
wire         ioctl_wait;

wire   [0:0] img_mounted;
wire         img_readonly;
wire  [63:0] img_size;
wire  [31:0] sd_lba[1];
wire   [0:0] sd_rd, sd_wr, sd_ack;
wire  [13:0] sd_buff_addr;
wire   [7:0] sd_buff_dout;
wire   [7:0] sd_buff_din[1];
wire         sd_buff_wr;
assign sd_rd = 1'b0;

// settings the machine runs with, taken from the OSD at every reset
reg  [127:0] cfg = 128'd0;
wire         pending = status[3:2] != cfg[3:2];

reg          info_req = 0;
reg    [7:0] info = 8'd1;

hps_io #(.CONF_STR(CONF_STR), .VDNUM(1)) hps_io
(
	.clk_sys           (clk_sys           ),
	.HPS_BUS           (HPS_BUS           ),
	.EXT_BUS           (                  ),
	.gamma_bus         (gamma_bus         ),
	.forced_scandoubler(forced_scandoubler),
	.buttons           (buttons           ),
	.status            (status            ),
	.status_menumask   ({ 15'd0, pending } ),
	.info_req          (info_req          ),
	.info              (info              ),
	.ps2_key           (ps2_key           ),
	.joystick_0        (joystick_0        ),
	.joystick_1        (joystick_1        ),

	.ioctl_download    (ioctl_download    ),
	.ioctl_index       (ioctl_index       ),
	.ioctl_wr          (ioctl_wr          ),
	.ioctl_addr        (ioctl_addr        ),
	.ioctl_dout        (ioctl_dout        ),
	.ioctl_wait        (ioctl_wait        ),

	.img_mounted       (img_mounted       ),
	.img_readonly      (img_readonly      ),
	.img_size          (img_size          ),
	.sd_lba            (sd_lba            ),
	.sd_blk_cnt        ('{6'd0}           ),
	.sd_rd             (sd_rd             ),
	.sd_wr             (sd_wr             ),
	.sd_ack            (sd_ack            ),
	.sd_buff_addr      (sd_buff_addr      ),
	.sd_buff_dout      (sd_buff_dout      ),
	.sd_buff_din       (sd_buff_din       ),
	.sd_buff_wr        (sd_buff_wr        )
);

// boot.rom / Load ROM: index 0; tape: index 1
wire rom_dl  = ioctl_download && ioctl_index[5:0] == 6'd0;
wire tape_dl = ioctl_download && ioctl_index[5:0] == 6'd1;

///////////////////////   RESET   ////////////////////////////////

wire reset_req = !pll_locked || RESET || status[0] || buttons[1] || rom_dl;
always @(posedge clk_sys) if(reset_req) cfg <= status;

reg reset = 1'b1;
always @(posedge clk_sys) reset <= reset_req;

///////////////////////   CLOCK ENABLES   ////////////////////////

wire ce_cpu_p, ce_cpu_n, ce_4m, turbo_active, turbo_req;
vg5k_clocks clocks
(
	.clk      (clk_sys     ),
	.turbo_req(turbo_req   ),
	.ce_cpu_p (ce_cpu_p    ),
	.ce_cpu_n (ce_cpu_n    ),
	.ce_4m    (ce_4m       ),
	.turbo    (turbo_active)
);

///////////////////////   TAPE   /////////////////////////////////
// The image (.k7 or .wav) goes to DDR3; rtl/vg5k_tape.sv plays it while the
// motor relay (port AF D1) is on. Every mount starts rewound. Turbo: CPU and
// tape 4x while the deck plays (tape-standard.md); Tape speed is read at each
// motor start. Without a tape image, the line-in ADC is the cassette input.

wire        motor, tape_out, beeper;
wire [24:0] tape_size, tape_addr, tape_pos;
wire        tape_req, tape_ack, tape_level, tape_ended, tape_is_wav, tape_playing, tape_moving, tape_sec;
wire  [7:0] tape_q;
wire        hdr_valid;
wire  [7:0] hdr_type;
wire [47:0] hdr_name;
wire  [7:0] file_count;
wire        hdr_autorun;

reg tape_dl_d = 0;
always @(posedge clk_sys) begin
	tape_dl_d <= tape_dl;
	info_req  <= tape_dl_d && !tape_dl;
end

vg5k_tape_ddr tape_ddr
(
	.clk             (clk_sys           ),
	.dl              (tape_dl           ),
	.dl_addr         (ioctl_addr        ),
	.dl_data         (ioctl_dout        ),
	.dl_wr           (ioctl_wr && tape_dl),
	.dl_wait         (ioctl_wait        ),
	.size            (tape_size         ),
	.rd_addr         (tape_addr         ),
	.rd_req          (tape_req          ),
	.rd_ack          (tape_ack          ),
	.rd_data         (tape_q            ),
	.DDRAM_CLK       (DDRAM_CLK         ),
	.DDRAM_BUSY      (DDRAM_BUSY        ),
	.DDRAM_BURSTCNT  (DDRAM_BURSTCNT    ),
	.DDRAM_ADDR      (DDRAM_ADDR        ),
	.DDRAM_DOUT      (DDRAM_DOUT        ),
	.DDRAM_DOUT_READY(DDRAM_DOUT_READY  ),
	.DDRAM_RD        (DDRAM_RD          ),
	.DDRAM_DIN       (DDRAM_DIN         ),
	.DDRAM_BE        (DDRAM_BE          ),
	.DDRAM_WE        (DDRAM_WE          )
);

vg5k_tape tape
(
	.clk       (clk_sys          ),
	.reset     (tape_dl || status[6]),
	.ce_cpu    (ce_cpu_p         ),
	.dl        (tape_dl          ),
	.dl_wr     (tape_dl && ioctl_wr),
	.dl_addr   (ioctl_addr[24:0] ),
	.dl_data   (ioctl_dout       ),
	.size      (tape_dl ? 25'd0 : tape_size),
	.motor     (motor            ),
	.turbo_osd (!status[5]       ),
	.turbo_req (turbo_req        ),
	.rd_addr   (tape_addr        ),
	.rd_req    (tape_req         ),
	.rd_ack    (tape_ack         ),
	.rd_data   (tape_q           ),
	.level     (tape_level       ),
	.ended     (tape_ended       ),
	.position  (tape_pos         ),
	.is_wav    (tape_is_wav      ),
	.playing   (tape_playing     ),
	.moving    (tape_moving      ),
	.sec_tick  (tape_sec         ),
	.hdr_valid (hdr_valid        ),
	.hdr_type  (hdr_type         ),
	.hdr_name  (hdr_name         ),
	.hdr_autorun(hdr_autorun     ),
	.file_count(file_count       )
);

// Tape save: what CSAVE writes on port AF D0 goes to the mounted .k7 image
// (rtl/vg5k_tape_rec.sv). The image must exist with room for the recording.
vg5k_tape_rec tape_rec
(
	.clk         (clk_sys          ),
	.ce_cpu      (ce_cpu_p         ),
	.motor       (motor            ),
	.tape_out    (tape_out         ),
	.img_mounted (img_mounted[0]   ),
	.img_readonly(img_readonly     ),
	.img_size    (img_size         ),
	.sd_lba      (sd_lba[0]        ),
	.sd_wr       (sd_wr[0]         ),
	.sd_ack      (sd_ack[0]        ),
	.sd_buff_addr(sd_buff_addr[8:0]),
	.sd_buff_din (sd_buff_din[0]   ),
	.active      (                 ),
	.full        (                 ),
	.length      (                 )
);

// tape from the line-in ADC (a real cassette) while no image is mounted
wire adc_tape, adc_act;
ltc2308_tape ltc2308_tape
(
	.reset   (1'b0    ),
	.clk     (CLK_50M ),
	.ADC_BUS (ADC_BUS ),
	.dout    (adc_tape),
	.active  (adc_act ),
	.adc_sync(        ),
	.adc_data(        )
);
reg [1:0] adc_tape_s = 0, adc_act_s = 0;
always @(posedge clk_sys) begin
	adc_tape_s <= { adc_tape_s[0], adc_tape };
	adc_act_s  <= { adc_act_s[0],  adc_act  };
end

wire tape_loaded = tape_size != 0;
wire tape_in = tape_loaded ? tape_level : (adc_act_s[1] & adc_tape_s[1]);

///////////////////////   MACHINE   //////////////////////////////

wire [31:0] joyA = status[17] ? joystick_1 : joystick_0;
wire [31:0] joyB = status[17] ? joystick_0 : joystick_1;
// {action 2, action 1, left, down, right, up}
wire  [5:0] joy0 = { joyA[5], joyA[4], joyA[1], joyA[2], joyA[0], joyA[3] };
wire  [5:0] joy1 = { joyB[5], joyB[4], joyB[1], joyB[2], joyB[0], joyB[3] };

wire [2:0] kbd_row;
wire [7:0] kbd_data;
wire       kbd_rd, kbd_nmi;
vg5k_keyboard #(.CLK_HZ(48000000)) keyboard
(
	.clk_sys  (clk_sys   ),
	.reset    (reset     ),
	.pc_layout(!status[16]),
	.row_sel  (kbd_row   ),
	.row_rd   (kbd_rd    ),
	.ps2_key  (ps2_key   ),
	.row_data (kbd_data  ),
	.nmi_key  (kbd_nmi   )
);

wire ce_pix, HSync, VSync, HBlank, VBlank, R, G, B;

vg5000 machine
(
	.clk       (clk_sys          ),
	.reset     (reset            ),
	.ce_cpu_p  (ce_cpu_p         ),
	.ce_cpu_n  (ce_cpu_n         ),
	.turbo     (turbo_active     ),
	.mem_size  (cfg[3:2]         ),
	.rom_wr    (rom_dl && ioctl_wr && ioctl_addr < 27'd16384),
	.rom_addr  (ioctl_addr[13:0] ),
	.rom_data  (ioctl_dout       ),
	.kbd_row   (kbd_row          ),
	.kbd_data  (kbd_data         ),
	.kbd_rd    (kbd_rd           ),
	.nmi_key   (kbd_nmi || status[7]),
	.joy0      (joy0             ),
	.joy1      (joy1             ),
	.tape_in   (tape_in          ),
	.tape_out  (tape_out         ),
	.motor     (motor            ),
	.beeper    (beeper           ),
	.ce_pix    (ce_pix           ),
	.vid_r     (R                ),
	.vid_g     (G                ),
	.vid_b     (B                ),
	.hsync     (HSync            ),
	.vsync     (VSync            ),
	.hblank    (HBlank           ),
	.vblank    (VBlank           ),
	.dbg_a     (), .dbg_di(), .dbg_do(), .dbg_m1_n(), .dbg_mreq_n(), .dbg_iorq_n(),
	.dbg_rd_n  (), .dbg_wr_n(), .dbg_rfsh_n(), .dbg_int_n(), .dbg_reg()
);

///////////////////////   AUDIO   ////////////////////////////////
// 1-bit beeper (port AF D3) as +/- a fixed level (signed, no DC offset when
// silent: the level is 0 while the bit has not changed for 50 ms), plus the
// tape signal at a lower level with Tape audio On while the deck plays.

reg [21:0] beep_idle = 0;
reg        beep_q = 0;
always @(posedge clk_sys) begin
	beep_q <= beeper;
	if (beeper != beep_q) beep_idle <= 0;
	else if (beep_idle != 22'h3FFFFF) beep_idle <= beep_idle + 1'd1;
end
wire        beep_on  = beep_idle < 22'd2400000;
wire [15:0] beep_s   = !beep_on ? 16'd0 : beeper ? 16'sd6000 : -16'sd6000;
wire        tape_snd = status[18] && tape_playing;
wire [15:0] tape_s   = !tape_snd ? 16'd0 : tape_in ? 16'sd2000 : -16'sd2000;
wire [15:0] mix      = beep_s + tape_s;

assign AUDIO_L = mix;
assign AUDIO_R = mix;

///////////////////////   LEDS   /////////////////////////////////

assign LED_USER = ioctl_download || tape_moving;

///////////////////////   TAPE OVERLAY   //////////////////////////
// Status bar while the deck plays; next-step hint while it is stopped:
// "TYPE CLOAD <ret>" with a tape at its start, "TYPE RUN <ret>" after a BASIC
// file (header type 20h) without an auto-run line (header byte 12h = 0; e.g.
// "10" there makes CLOAD start the program) has played to the end. The hint goes when Return is
// pressed. (tape-standard.md rules 19-28)

reg hint_gone = 0;
reg ps2_key_q = 0;
always @(posedge clk_sys) ps2_key_q <= ps2_key[10];
always @(posedge clk_sys) begin
	if (tape_dl || status[6]) hint_gone <= 0;
	else if (ps2_key[9] && ps2_key[8:0] == 9'h05A && ps2_key[10] != ps2_key_q) hint_gone <= 1;
	if (tape_playing) hint_gone <= 0;
end

// "at the start": the deck has not played since the mount / rewind (the
// player prefetches a byte, so the position is not 0). "Played to the end":
// BASIC stops the motor inside the last block's 10-byte footer, so the
// position is within 16 bytes of the end (or the player ended).
reg tape_played = 0;
always @(posedge clk_sys) begin
	if (tape_dl || status[6]) tape_played <= 0;
	else if (tape_playing) tape_played <= 1;
end
wire at_end    = tape_ended || (tape_pos + 25'd16 >= tape_size);
wire hint_load = tape_loaded && !tape_playing && !tape_played && !hint_gone;
wire hint_run  = tape_loaded && !tape_playing && tape_played && at_end && !tape_is_wav && hdr_valid &&
                 hdr_type == 8'h20 && !hdr_autorun && !hint_gone;
//                          T    Y    P    E   ' '   C    L    O    A    D   ' '  ret
wire [175:0] txt_load = {"TYPE CLOAD ", 8'h01, 80'd0};
wire [175:0] txt_run  = {"TYPE RUN ", 8'h01, 96'd0};

wire [223:0] ov_status;
wire         ov_req, ov_ack;
vg5k_tape_status tape_status
(
	.clk       (clk_sys          ),
	.rewind    (tape_dl || status[6]),
	.loaded    (tape_loaded      ),
	.playing   (tape_playing     ),
	.fast      (turbo_active     ),
	.sec_tick  (tape_sec         ),
	.total_sec (16'd0            ),
	.total_ok  (1'b0             ),
	.position  (tape_pos         ),
	.data_start(25'd0            ),
	.data_end  (tape_size        ),
	.hint_show (hint_load || hint_run),
	.hint_len  (hint_load ? 5'd12 : 5'd10),
	.hint_text (hint_load ? txt_load : txt_run),
	.status    (ov_status        ),
	.req       (ov_req           ),
	.ack       (ov_ack           )
);

///////////////////////   VIDEO   ////////////////////////////////
// The EF9345 raster (rtl/ef9345/ef9345_vg5k.sv): PAL 15.625 kHz / 50 Hz,
// ce_pix 24 MHz (3 per 8 MHz 40-column dot, 2 per 12 MHz 80-column dot),
// 1247 x 284 visible (52 µs: margins + 40 µs page, 16 + 250 + 18 lines).
// Original aspect: PAL square pixels are 7.375 MHz, so 1247 pixels at 24 MHz
// are 1247 * 7.375 / 24 = 383.2 square pixels: 383 : 284 (1.349).
// To be checked on hardware (video-standard.md, Moving a core over, step 7).
//
// CLK_VIDEO = 96 MHz (4x the pixel enable). The machine runs on clk_sys
// (48 MHz, same PLL): its video signals are taken into clk_vid on each ce_pix.

wire [1:0] ar  = status[9:8];
wire [2:0] sfx = status[14:12];          // 0 None, 1 HQ2x, 2-4 CRT 25/50/75%
wire [2:0] sl  = sfx ? sfx - 3'd1 : 3'd0;
wire       scandoubler = sfx != 3'd0 || forced_scandoubler;

assign CLK_VIDEO = clk_vid;
assign VGA_SL = sl[1:0];

// Tape overlay on clk_sys, counted on the machine's ce_pix (video-standard
// rule 7: before the mixer, on the core's own pixel enable)
wire [23:0] ov_rgb;
vg5k_tape_overlay #(.FACTOR(4'd4)) tape_overlay
(
	.clk     (clk_sys          ),
	.ce_pix  (ce_pix           ),
	.hblank  (HBlank           ),
	.vblank  (VBlank           ),
	.enable  (!status[15]      ),
	.status  (ov_status        ),
	.req     (ov_req           ),
	.ack     (ov_ack           ),
	.rgb_in  ({{8{R}}, {8{G}}, {8{B}}}),
	.rgb_out (ov_rgb           )
);

// clk_sys -> clk_vid (same PLL): one toggle per ce_pix, the pixel held in between
reg        pix_t = 0;
reg [27:0] vid_s = 0;
always @(posedge clk_sys) if (ce_pix) begin
	pix_t <= ~pix_t;
	vid_s <= { ov_rgb, HSync, VSync, HBlank, VBlank };
end
reg  [2:0] pix_t_v = 0;
reg        ce_v = 0;
reg [27:0] vid_v = 0;
always @(posedge clk_vid) begin
	pix_t_v <= { pix_t_v[1:0], pix_t };
	ce_v    <= pix_t_v[2] != pix_t_v[1];
	if (pix_t_v[2] != pix_t_v[1]) vid_v <= vid_s;
end
wire HSv = vid_v[3], VSv = vid_v[2], HBv = vid_v[1], VBv = vid_v[0];

wire vga_de;

video_freak video_freak
(
	.CLK_VIDEO  (CLK_VIDEO  ),
	.CE_PIXEL   (CE_PIXEL   ),
	.VGA_VS     (VGA_VS     ),
	.HDMI_WIDTH (HDMI_WIDTH ),
	.HDMI_HEIGHT(HDMI_HEIGHT),
	.VGA_DE     (VGA_DE     ),
	.VIDEO_ARX  (VIDEO_ARX  ),
	.VIDEO_ARY  (VIDEO_ARY  ),
	.VGA_DE_IN  (vga_de     ),
	.ARX        ((!ar) ? 12'd383 : (ar - 1'd1)),
	.ARY        ((!ar) ? 12'd284 : 12'd0),
	.CROP_SIZE  (12'd0      ),
	.CROP_OFF   (5'd0       ),
	.SCALE      ({ 1'b0, status[11:10] })
);

video_mixer #(.LINE_LENGTH(1280), .GAMMA(1)) video_mixer
(
	.CLK_VIDEO  (CLK_VIDEO  ),
	.CE_PIXEL   (CE_PIXEL   ),
	.ce_pix     (ce_v       ),
	.scandoubler(scandoubler),
	.hq2x       (sfx == 3'd1),
	.gamma_bus  (gamma_bus  ),
	.R          (vid_v[27:20]),
	.G          (vid_v[19:12]),
	.B          (vid_v[11:4] ),
	.HSync      (HSv        ),
	.VSync      (VSv        ),
	.HBlank     (HBv        ),
	.VBlank     (VBv        ),
	.HDMI_FREEZE(HDMI_FREEZE),
	.freeze_sync(           ),
	.VGA_R      (VGA_R      ),
	.VGA_G      (VGA_G      ),
	.VGA_B      (VGA_B      ),
	.VGA_VS     (VGA_VS     ),
	.VGA_HS     (VGA_HS     ),
	.VGA_DE     (vga_de     )
);

endmodule
