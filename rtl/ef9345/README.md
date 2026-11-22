# EF9345 for the VG5000 MiSTer core

**Licence: CERN-OHL-S-2.0** (strongly reciprocal), see `LICENSE` in this
directory (copied unchanged from orpigame `vdu/LICENSE`). These files derive
from D. Laurens' VHDL EF9345 core
(https://github.com/orpigame/FPGA-Emulator, branch `vdu`, `vdu/rtl/`): a hand translation to SystemVerilog (2026), then fixed and
retimed to the native 15 kHz raster (2026). Every file keeps the original
header and says what changed.

The translation was first checked cycle for cycle against the VHDL (GHDL
netlist, Verilator), then fixed against MAME and against captures of real
EF9345 chips (fabio-d's EF9345 test suite).

## Use (machine side)

Instantiate only `ef9345_vg5k`. Files:
`ef9345.qip` (Quartus), or for Verilator all `*.sv` here with `-Irtl/ef9345`.

| Port | Behaviour |
|---|---|
| `clk` | clk_sys, 48 MHz. Everything runs on it; no other clock |
| `reset` | synchronous, active high. Resets the bus sequencer and the command engine only: like the real chip (which has no reset pin) the registers, TGS/MAT/PAT/DOR/ROR and the VRAM keep their contents; the BASIC ROM re-initialises them |
| `addr_wr` / `data_wr` / `data_rd` | **rising edge** = one request (1-clock pulses or levels held for the I/O cycle both work). `din` is taken on that clock. OUT 8F = address byte (`0x20+Ri`, +8 = execute), OUT CF = data write, IN CF = data read. Each request becomes an Intel bus cycle on the core pins (34 clocks = 0.71 us); one more request may arrive meanwhile (queued) |
| `dout` | core data register: valid about 8 clocks after the `data_rd` edge, held until the next read (STATUS with Busy = bit 7, or R1..R7) |
| `ce_pix` | 24 MHz (every other clk), constant in every mode |
| `r g b` | 1-bit colours, 0 during blanking |
| `hsync vsync` | **active high**. hsync 4.5 us; vsync 2 lines, changing at the hsync leading edge |
| `hblank vblank` | active high |
| `irq_n` | the PC/VS pin (VS when TGS4 = 0, as the VG5000 ROM sets it): low for the 2 VS lines (128 us) every 19.968 ms; on the board it is ANDed with INT EX into Z80 /INT |

Character ROM: `romchar0.hex`, `romchar1.hex`: the ROM read out of a **real
EF9345P** with IND $88 (fabio-d suite, `tests/test_font_data/test_read_ind_EF9345_b0..b3.png`),
in MAME's `charset.rom` layout. MAME's charset (marked BAD_DUMP) differs in
G0 codes $00-$0B, $0E (arrow) and $27 (apostrophe); the G10/G11 banks match.
`ROMCHAR0/1.sv` read them with `$readmemh` from **`rtl/ef9345/romchar0.hex`**
(path relative to the Quartus project directory). A Verilator build run from
another directory must pass
`-DEF9345_ROMCHAR0_HEX='"<path>/romchar0.hex"' -DEF9345_ROMCHAR1_HEX='"<path>/romchar1.hex"'`;
if the files are not found the glyphs are blank but backgrounds and margin
still show. (An all-black frame means no IND write reached the chip: the
power-up MAT = PAT = 0 gives a black margin over the whole screen.)

### Raster and aspect (for `video_freak` / the scaler)

Native EF9345 timing (datasheet figure 7, CLK = 12 MHz, TGS0 = 0, non
interlaced, which is what the VG5000 uses), counted in ce_pix (24 MHz):

| | value |
|---|---|
| line | 1536 ce_pix = 64.000 us (15.625 kHz) |
| visible per line | 1247 ce_pix = 51.96 us: margin 143 + bulk 960 + margin 144 |
| hblank | 289 ce_pix = 12.04 us: front porch 2 us, hsync 4.5 us, back porch 5.5 us |
| frame | 312 lines = 19.968 ms (50.08 Hz) |
| visible lines | 284: margin 16 + page 250 (service row + 24 rows of 10) + margin 18 |
| vblank | 28 lines: 3, vsync 2, 23 |
| 40 columns | 8 dots/char, 8 MHz: each dot = 3 ce_pix |
| 80 columns | 6 dots/char, 12 MHz: each dot = 2 ce_pix |

`ce_pix` choice: a constant 24 MHz in both modes (the least common multiple
of the 8 and 12 MHz dot clocks), so a mode change never changes the pixel
clock and the scaler sees a fixed 1247 x 284 frame. Every dot is a whole
number of pixels (3 or 2). With the scaler's horizontal filter off these
repeats are exact; the analog 15 kHz output does not care.

Aspect: PAL 4:3 is 52 us x 288 lines. The visible area is 51.96 us x 284
lines, so its display aspect is (4 x 51.96/52) : (3 x 284/288) = 1.351.
Use `VIDEO_ARX = 12'd1351, VIDEO_ARY = 12'd1000` (or 50:37) for the full
frame; plain 4:3 is 1.3 % off. If the machine crops (e.g. to the 960 x 250
bulk + some margin), scale by the same 24 MHz-pixel / TV-line ratio:
one ce_pix is 1/24 us, one line is 1/288 of the 4:3 height, so
ARX:ARY = (pixels / 24 / 52 x 4) : (lines / 288 x 3).

## Changes from the original (2026)

See `FIXES` in `EF9345.sv` (all on by default, bits 0-8; bits 6-7 are the
behaviours measured on real chips, bit 8 the non-stop move end).
Retiming: 48 MHz clock, line zoom 1, native 625-line raster constants in
`EF9345_Defines.svh`, MAT filter and blink counter rescaled, wrapper
`ef9345_vg5k.sv` (new), synchronous reset of the bus / command engine.

| File | From |
|---|---|
| `ef9345_vg5k.sv` | new: VG5000 bus, video outputs, irq |
| `EF9345.sv` | `EF9345.vhd` (top) |
| `EF9345_Controller.sv` | `EF9345_Controller.vhd` |
| `CharactersDecoder.sv` | `CharactersDecoder.vhd` |
| `EF9345_Defines.svh` | `EF9345_Defines.vhd` (package -> localparams, `` `include``d in each module) |
| `VRAM.sv` | `VRAM.vhd` |
| `RAM64X8.sv` | `RAM64X8.vhd` (Xilinx RAM64X2S -> inferred 64 x 8, async read) |
| `EF9345_BRAM_TDP.sv` | replaces Xilinx `BRAM_TDP_MACRO` as configured in `VRAM.vhd` (per-lane 1024 x 8, old-data mixed port) |
| `ROMCHAR0.sv`, `ROMCHAR1.sv` | the character ROMs the VHDL expects but does not ship |
| `ef9345_lint.vlt` | Verilator lint waivers (justified inline) |
| `ef9345.qip` | Quartus file list |

Quartus: written for Quartus 17 (explicit `generate`, no `for (genvar ...)`,
no typedefs in the wrapper); RAMs follow the inference templates; power-up
values come from declaration initialisers (Quartus honours them for
registers).
