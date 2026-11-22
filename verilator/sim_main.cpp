// Headless Verilator harness for the VG5000 core (vg5000_sim.sv).
// Modelled on SC3000_MiSTer/verilator/sim_main.cpp.
//
//   ./obj_dir/Vvg5000_sim --rom files/vg5k11.bin --frames 300 --png out.png --text-end
//   ./obj_dir/Vvg5000_sim --rom R --type 'PRINT 12*3\n@150' --frames 300 --text-end
//   ./obj_dir/Vvg5000_sim --rom R --tape game.k7 --type 'CLOAD\n@150' --until 'Ok' --frames 3000
//
// Run with --help for every option.

#include "Vvg5000_sim.h"
#include "Vvg5000_sim___024root.h"
#include "Vvg5000_sim__Dpi.h"
#include "svdpi.h"
#include "verilated.h"
#if VM_TRACE
#include "verilated_vcd_c.h"
#endif

#include <zlib.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <algorithm>
#include <sys/stat.h>
#include <functional>

static const uint64_t CLK_HZ = 48000000;

static Vvg5000_sim* top;
static uint64_t cycles = 0;
#if VM_TRACE
static VerilatedVcdC* vcd = nullptr;
#endif
static int vcd_from = -1, vcd_to = -1;

// ---------------------------------------------------------------- options
static std::string rom_path, tape_path, outdir = "out", png_path, trace_path, stop_text, wav_path, ramdump_path;
static int run_frames = 0, every = 0, trace_from = 0, trace_to = 1 << 30, mem_size = 0;
static std::set<int> shots, text_frames;
static bool text_at_end = false, io_log = false, tape_log = false, tape_turbo = false, pc_layout = true;
static int tape_at = -1;
static std::string rec_path; static size_t rec_size = 262144;
// one emulated hps_io virtual disk (write path only): from SC3000_MiSTer sim_main.cpp
struct SdDev {
  std::vector<uint8_t> img;
  int state = 0, cnt = 0, addr = 0; uint32_t lba = 0; uint64_t writes = 0;
  std::function<bool()> wr; std::function<uint32_t()> lba_in; std::function<uint8_t()> din;
  std::function<void(int)> ack; std::function<void(int)> baddr;
  void step() {
    switch (state) {
      case 0: if (wr()) { lba = lba_in(); cnt = 40; state = 1; writes++; } break;
      case 1: if (--cnt == 0) { ack(1); addr = 0; cnt = 0; state = 2; } break;
      case 2: {
        if (++cnt < 4) { if (cnt == 1) baddr(addr); break; }
        cnt = 0; size_t off = (size_t)lba * 512 + addr;
        if (off < img.size()) img[off] = din();
        if (++addr == 512) { state = 3; cnt = 8; }
        break;
      }
      case 3: if (--cnt == 0) { ack(0); state = 4; cnt = 8; } break;
      case 4: if (--cnt == 0) state = 0; break;
    }
  }
};
static SdDev recdev;
static int until_base = -1;
static int stop_after_motor = -1, motor_off_frame = -1; static bool motor_was_on = false;
struct Peek { int frame; unsigned addr, n; };
static std::vector<Peek> peeks;

// ---------------------------------------------------------------- PNG
static void put32(std::vector<uint8_t>& v, uint32_t x) {
  v.push_back(x >> 24); v.push_back(x >> 16); v.push_back(x >> 8); v.push_back(x);
}
static void chunk(FILE* f, const char* type, const std::vector<uint8_t>& data) {
  std::vector<uint8_t> b; put32(b, data.size());
  b.insert(b.end(), type, type + 4); b.insert(b.end(), data.begin(), data.end());
  uint32_t crc = crc32(0, b.data() + 4, b.size() - 4);
  put32(b, crc); fwrite(b.data(), 1, b.size(), f);
}
static bool write_png(const std::string& path, int w, int h, const std::vector<uint8_t>& rgb) {
  FILE* f = fopen(path.c_str(), "wb"); if (!f) return false;
  static const uint8_t sig[8] = {137, 80, 78, 71, 13, 10, 26, 10}; fwrite(sig, 1, 8, f);
  std::vector<uint8_t> ihdr; put32(ihdr, w); put32(ihdr, h);
  ihdr.push_back(8); ihdr.push_back(2); ihdr.push_back(0); ihdr.push_back(0); ihdr.push_back(0);
  chunk(f, "IHDR", ihdr);
  std::vector<uint8_t> raw; raw.reserve((w * 3 + 1) * h);
  for (int y = 0; y < h; y++) { raw.push_back(0); raw.insert(raw.end(), &rgb[y * w * 3], &rgb[y * w * 3] + w * 3); }
  uLongf zl = compressBound(raw.size()); std::vector<uint8_t> z(zl);
  compress2(z.data(), &zl, raw.data(), raw.size(), 6); z.resize(zl);
  chunk(f, "IDAT", z); chunk(f, "IEND", {}); fclose(f); return true;
}

// ---------------------------------------------------------------- video capture
static const int FB_W = 1600, FB_H = 320;
static std::vector<uint8_t> fb(FB_W * FB_H * 3);
static int vx = 0, vy = 0, max_x = 0, last_w = 0, last_h = 0;
static bool prev_hb = true, prev_vs = false, line_had_pixels = false;
static int frame = 0;

static void save_frame(const std::string& path) {
  if (last_w == 0) { last_w = std::min(max_x, FB_W); last_h = std::min(vy, FB_H); }
  std::vector<uint8_t> out(last_w * last_h * 3);
  for (int y = 0; y < last_h; y++) memcpy(&out[y * last_w * 3], &fb[y * FB_W * 3], last_w * 3);
  if (!write_png(path, last_w, last_h, out)) fprintf(stderr, "cannot write %s\n", path.c_str());
  uint32_t c = crc32(0, out.data(), out.size());
  // colours used (a black or one-colour picture means no display)
  std::set<uint32_t> cols;
  for (size_t i = 0; i + 2 < out.size(); i += 3) { cols.insert(out[i] << 16 | out[i + 1] << 8 | out[i + 2]); if (cols.size() > 8) break; }
  printf("f%d shot %s %dx%d crc %08X colours %zu\n", frame, path.c_str(), last_w, last_h, c, cols.size());
}

// ---------------------------------------------------------------- machine RAM / screen buffer
static uint8_t ram_rd(unsigned a) {
  static svScope sc = nullptr;
  if (!sc) { sc = svGetScopeFromName("TOP.vg5000_sim"); svSetScope(sc); }
  return (uint8_t)sim_peek(a & 0xFFFF);
}
// BASIC keeps the 40 x 25 text page at 4000-47CF, 2 bytes per cell (char, attribute)
static std::string screen_text() {
  std::string all;
  for (int y = 0; y < 25; y++) {
    std::string line;
    for (int x = 0; x < 40; x++) {
      uint8_t c = ram_rd(0x4000 + y * 80 + x * 2);
      line += (c >= 0x20 && c < 0x7F) ? char(c) : (c == 0x80 ? ' ' : '.');
    }
    while (!line.empty() && (line.back() == ' ' || line.back() == '.')) line.pop_back();
    all += line + "\n";
  }
  return all;
}
static void print_text(int f) {
  printf("--- screen at frame %d (RAM 4000-47CF) ---\n", f);
  std::string t = screen_text(); size_t p = 0;
  while (p < t.size()) { size_t q = t.find('\n', p); printf("|%s\n", t.substr(p, q - p).c_str()); p = q + 1; }
  fflush(stdout);
}

// ---------------------------------------------------------------- keyboard matrix (MAME ROW1..8 = rows 0..7)
struct RC { int row, col; };
static const std::map<std::string, RC> named_keys = {
  {"list", {0, 0}}, {"shift", {0, 2}}, {"left", {0, 3}}, {"right", {0, 4}}, {"down", {0, 5}},
  {"ctrl", {0, 6}}, {"ins", {0, 7}}, {"stop", {1, 0}}, {"run", {1, 0}}, {"space", {1, 2}}, {"lck", {1, 3}},
  {"enter", {1, 5}}, {"ret", {1, 5}}, {"up", {1, 6}}, {"prt", {6, 6}}, {"del", {7, 1}}, {"effc", {7, 1}},
};
// VG5000 key caps: unshifted / shifted character per (row, col)
static const char* KEYS_N[8] = {"\x01\x01\x01\x01\x01\x01\x01\x01", "\x01q \x01\x01\n\x01" "a",
                                "wxcvb1:z", "se3456" "2;", "\x01\x01" "fguiop", "\x01" "0]\x01,789",
                                "-+rty<\x01" "d", "=\x01njhklm"};
static const char* KEYS_S[8] = {"\x01\x01\x01\x01\x01\x01\x01\x01", "\x01Q\x01\x01\x01\x01\x01" "A",
                                "WXCVB#*Z", "SE\"\x01$%!@", "_|FGUIOP", "\x01)[\x01/&'(",
                                "?.RTY>\x01" "D", "^\x01NJHKLM"};
static bool char_key(char c, RC& rc, bool& shift) {
  for (int r = 0; r < 8; r++) for (int k = 0; k < 8; k++) {
    if (KEYS_N[r][k] == c) { rc = {r, k}; shift = false; return true; }
  }
  for (int r = 0; r < 8; r++) for (int k = 0; k < 8; k++) {
    if (KEYS_S[r][k] == c) { rc = {r, k}; shift = true; return true; }
  }
  return false;
}
static bool type_shift_letters = false;   // set if unshifted letters type lower case
struct KeyEv { int frame; RC rc; bool down; };
static std::vector<KeyEv> key_evs;
static std::map<int, int> key_count;
static void schedule_text(int f, const std::string& t) {
  for (size_t i = 0; i < t.size(); i++) {
    char c = t[i];
    if (c == '\\' && i + 1 < t.size() && t[i + 1] == 'n') { c = '\n'; i++; }
    RC rc; bool shift;
    char lc = c;
    if (!type_shift_letters && c >= 'A' && c <= 'Z') lc = c - 'A' + 'a';   // capitals unshifted (BASIC types capitals)
    if (!char_key(lc, rc, shift)) { fprintf(stderr, "cannot type '%c'\n", c); continue; }
    RC sh = named_keys.at("shift");
    if (shift) key_evs.push_back({f, sh, true});
    key_evs.push_back({f, rc, true});
    key_evs.push_back({f + 3, rc, false});
    if (shift) key_evs.push_back({f + 3, sh, false});
    f += (c == '\n') ? 20 : 6;
  }
}

struct JoyEv { int frame, len, port; uint8_t bits; };
static std::vector<JoyEv> joy_evs;
struct NmiEv { int frame, len; };
static std::vector<NmiEv> nmi_evs;

// --paste: PS/2 key events through rtl/vg5k_keyboard.sv (PC layout)
struct Ps2Ev { uint64_t cycle; int code; bool down; };
static std::vector<Ps2Ev> ps2_evs; static size_t ps2_i = 0; static int ps2_toggle = 0;
struct PasteReq { int frame; std::string text; int hold_ms, gap_ms; };
static std::vector<PasteReq> paste_reqs;
static int ps2_code(char ch, bool& shift) {
  static const char* up = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  static const int uc[] = {0x1C,0x32,0x21,0x23,0x24,0x2B,0x34,0x33,0x43,0x3B,0x42,0x4B,0x3A,0x31,0x44,0x4D,0x15,0x2D,0x1B,0x2C,0x3C,0x2A,0x1D,0x22,0x35,0x1A};
  shift = false;
  for (int i = 0; up[i]; i++) { if (up[i] == ch) { shift = true; return uc[i]; } if (up[i] + 32 == ch) return uc[i]; }
  switch (ch) {
    case '1': return 0x16; case '2': return 0x1E; case '3': return 0x26; case '4': return 0x25; case '5': return 0x2E;
    case '6': return 0x36; case '7': return 0x3D; case '8': return 0x3E; case '9': return 0x46; case '0': return 0x45;
    case ' ': return 0x29; case ',': return 0x41; case '.': return 0x49; case ';': return 0x4C; case '=': return 0x55;
    case '-': return 0x4E; case '/': return 0x4A; case '\n': return 0x5A; case '\'': return 0x52; case '[': return 0x54; case ']': return 0x5B;
    case ':': shift = true; return 0x4C; case '*': shift = true; return 0x3E; case '(': shift = true; return 0x46;
    case ')': shift = true; return 0x45; case '"': shift = true; return 0x52; case '+': shift = true; return 0x55;
    case '$': shift = true; return 0x25; case '<': shift = true; return 0x41; case '>': shift = true; return 0x49;
    case '!': shift = true; return 0x16; case '@': shift = true; return 0x1E; case '#': shift = true; return 0x26;
    case '%': shift = true; return 0x2E; case '^': shift = true; return 0x36; case '&': shift = true; return 0x3D;
    case '_': shift = true; return 0x4E; case '?': shift = true; return 0x4A; case '|': shift = true; return 0x5D;
  }
  return -1;
}
static void expand_paste(const PasteReq& p, uint64_t base) {
  const uint64_t ms = CLK_HZ / 1000; uint64_t t = base;
  std::string txt; for (size_t i = 0; i < p.text.size(); i++) {
    if (p.text[i] == '\\' && i + 1 < p.text.size() && p.text[i + 1] == 'n') { txt += '\n'; i++; } else txt += p.text[i]; }
  for (char ch : txt) { bool sh; int c = ps2_code(ch, sh); if (c < 0) continue;
    if (sh) { ps2_evs.push_back({t, 0x12, true}); t += 5 * ms; }
    ps2_evs.push_back({t, c, true}); t += p.hold_ms * ms;
    ps2_evs.push_back({t, c, false}); t += 2 * ms;
    if (sh) { ps2_evs.push_back({t, 0x12, false}); }
    t += p.gap_ms * ms; }
}

static void tick() {
  while (ps2_i < ps2_evs.size() && ps2_evs[ps2_i].cycle <= cycles) {
    const Ps2Ev& e = ps2_evs[ps2_i++]; ps2_toggle ^= 1;
    top->ps2_key = (e.code & 0x1FF) | (e.down << 9) | (ps2_toggle << 10); }
  top->clk = 0; top->eval();
#if VM_TRACE
  if (vcd && frame >= vcd_from && frame <= vcd_to) vcd->dump(cycles * 2);
#endif
  top->clk = 1; top->eval();
#if VM_TRACE
  if (vcd && frame >= vcd_from && frame <= vcd_to) vcd->dump(cycles * 2 + 1);
#endif
  cycles++;
}

static std::vector<uint8_t> read_file(const std::string& p) {
  std::vector<uint8_t> d; FILE* f = fopen(p.c_str(), "rb");
  if (!f) { fprintf(stderr, "cannot open %s\n", p.c_str()); exit(3); }
  fseek(f, 0, SEEK_END); d.resize(ftell(f)); fseek(f, 0, SEEK_SET);
  if (fread(d.data(), 1, d.size(), f) != d.size()) { fprintf(stderr, "short read %s\n", p.c_str()); exit(3); }
  fclose(f); return d;
}
static void download(int index, const std::vector<uint8_t>& d) {
  top->dl_busy = 1; top->dl_index = index; tick(); tick();
  for (size_t i = 0; i < d.size(); i++) {
    top->dl_wr = 1; top->dl_index = index; top->dl_addr = i; top->dl_data = d[i];
    tick(); top->dl_wr = 0; tick();
  }
  top->dl_wr = 0; tick();
  top->dl_busy = 0; tick(); tick();
}

// ---------------------------------------------------------------- CPU trace (MAME debugger format)
//   mame vg5k -debug -debugscript trace.cmd with
//   trace sim.tr,maincpu,noloop,{tracelog "AF=%04X BC=%04X DE=%04X HL=%04X IX=%04X IY=%04X SP=%04X ",af,bc,de,hl,ix,iy,sp}
static FILE* trace_f = nullptr;
static uint32_t reg_bits(int lo, int n) {
  uint32_t v = 0;
  for (int i = 0; i < n; i++) { int b = lo + i; if ((top->cpu_reg[b >> 5] >> (b & 31)) & 1) v |= 1u << i; }
  return v;
}
static bool prefix_pending = false;
static uint64_t trace_lines = 0;
static uint8_t last_fetch = 0;
static void trace_fetch(uint16_t pc, uint8_t op) {
  bool cont = prefix_pending;
  prefix_pending = !cont && (op == 0xCB || op == 0xDD || op == 0xED || op == 0xFD);
  if (op == 0xCB && cont) prefix_pending = false;
  if ((last_fetch == 0xDD || last_fetch == 0xFD) && op == 0xCB) prefix_pending = false;
  last_fetch = op;
  if (cont || !trace_f || frame < trace_from || frame > trace_to) return;
  uint32_t A = reg_bits(0, 8), F = reg_bits(8, 8), SP = reg_bits(48, 16);
  uint32_t BC = reg_bits(80, 16), DE = reg_bits(96, 16), HL = reg_bits(112, 16), IX = reg_bits(128, 16), IY = reg_bits(192, 16);
  if (getenv("TRACE_CYC")) fprintf(trace_f, "%llu ", (unsigned long long)cycles);
  fprintf(trace_f, "AF=%04X BC=%04X DE=%04X HL=%04X IX=%04X IY=%04X SP=%04X %04X: ; %02X\n",
          (A << 8) | F, BC, DE, HL, IX, IY, SP, pc, op);
  trace_lines++;
}

// ---------------------------------------------------------------- WAV (beeper)
static FILE* wav = nullptr; static uint32_t wav_samples = 0;
static void wav_close() {
  if (!wav) return;
  uint32_t rate = 44100, bytes = wav_samples * 2;
  fseek(wav, 0, SEEK_SET);
  auto w32 = [](uint32_t v) { fwrite(&v, 4, 1, wav); };
  auto w16 = [](uint16_t v) { fwrite(&v, 2, 1, wav); };
  fwrite("RIFF", 1, 4, wav); w32(36 + bytes); fwrite("WAVEfmt ", 1, 8, wav);
  w32(16); w16(1); w16(1); w32(rate); w32(rate * 2); w16(2); w16(16);
  fwrite("data", 1, 4, wav); w32(bytes); fclose(wav);
}

static void usage() {
  puts("Vvg5000_sim options:\n"
       "  --rom FILE              BASIC ROM (vg5k11.bin / vg5k10.bin, 16 KB), required\n"
       "  --mem 16|32|48          RAM size in KB (default 16)\n"
       "  --tape FILE[@FRAME]     tape image (.k7 or .wav), mounted at FRAME (default: before reset)\n"
       "  --turbo                 Tape speed = 4x\n"
       "  --rec FILE[:SIZE]       Tape save: record CSAVE into a blank SIZE-byte image (256 KB), written to FILE at the end\n"
       "  --tape-log              log motor on/off with the tape position\n"
       "  --frames N              stop after N frames (required)\n"
       "  --png FILE              write the last frame to FILE\n"
       "  --shots F1,F2,..        write OUTDIR/frame_NNNNN.png at these frames\n"
       "  --every K               a shot every K frames\n"
       "  --outdir DIR            (default ./out)\n"
       "  --type 'TEXT@FRAME'     type TEXT into the matrix from FRAME (\\n = RET), 6 frames per key\n"
       "  --paste 'TEXT@F[:HOLD:GAP]' type TEXT through rtl/vg5k_keyboard.sv (PS/2, PC layout)\n"
       "  --azerty                --paste uses the AZERTY (positional) layout\n"
       "  --key NAME@FRAME[:N]    hold a key N frames (list shift left right down ctrl ins stop space\n"
       "                          lck enter up prt del)\n"
       "  --joy BITS@FRAME[:N][/2] joystick: BITS from 'urdlab' (a,b = action 1, 2)\n"
       "  --nmi FRAME[:N]         press the delta key (Z80 NMI)\n"
       "  --text F1,F2,..         print the BASIC screen buffer (RAM 4000-47CF) at these frames\n"
       "  --text-end              print it at the end\n"
       "  --until 'TEXT'          stop when TEXT appears (once more than when typing ended; every 10 frames)\n"
       "  --stop-after-motor N    stop N frames after the tape motor last went off (tape sweeps)\n"
       "  --peek ADDR:N@FRAME     print N bytes of memory at FRAME\n"
       "  --ramdump FILE          write 64 KB (ROM + RAM) at the end\n"
       "  --trace FILE[:F1[:F2]]  Z80 trace in MAME tracelog format, frames F1..F2\n"
       "  --vcd FILE:F1:F2        VCD of frames F1..F2 (make vcd build)\n"
       "  --io-log                log I/O writes (frame, port, data)\n"
       "  --wav FILE              beeper as 44.1 kHz mono WAV\n");
}
static std::pair<std::string, int> split_at(const std::string& s) {
  size_t p = s.rfind('@');
  if (p == std::string::npos) { fprintf(stderr, "missing @FRAME in '%s'\n", s.c_str()); exit(3); }
  return {s.substr(0, p), atoi(s.c_str() + p + 1)};
}
static void parse_list(const char* s, std::set<int>& out) {
  std::string t(s); size_t p = 0;
  while (p < t.size()) { size_t q = t.find(',', p); if (q == std::string::npos) q = t.size();
    out.insert(atoi(t.substr(p, q - p).c_str())); p = q + 1; }
}

int main(int argc, char** argv) {
  Verilated::commandArgs(argc, argv);
  std::string vcd_path;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&]() -> const char* { if (i + 1 >= argc) { fprintf(stderr, "%s needs a value\n", a.c_str()); exit(3); } return argv[++i]; };
    if (a == "--help" || a == "-h") { usage(); return 0; }
    else if (a == "--rom") rom_path = next();
    else if (a == "--mem") { int m = atoi(next()); mem_size = m >= 48 ? 2 : m >= 32 ? 1 : 0; }
    else if (a == "--tape") { std::string v = next(); size_t at = v.rfind('@');
      if (at != std::string::npos && at > v.rfind('/')) { tape_at = atoi(v.c_str() + at + 1); v = v.substr(0, at); } tape_path = v; }
    else if (a == "--turbo") tape_turbo = true;
    else if (a == "--rec") { std::string v = next(); size_t c = v.rfind(':');
      if (c != std::string::npos && c > v.rfind('/')) { rec_size = strtoul(v.c_str() + c + 1, 0, 0); v = v.substr(0, c); } rec_path = v; }
    else if (a == "--tape-log") tape_log = true;
    else if (a == "--frames") run_frames = atoi(next());
    else if (a == "--png") png_path = next();
    else if (a == "--shots") parse_list(next(), shots);
    else if (a == "--every") every = atoi(next());
    else if (a == "--outdir") outdir = next();
    else if (a == "--type") { auto p = split_at(next()); schedule_text(p.second, p.first); }
    else if (a == "--paste") {
      std::string v = next(); int hold = 30, gap = 50; size_t at = v.rfind('@');
      std::string tail = v.substr(at + 1); int f = atoi(tail.c_str());
      size_t c1 = tail.find(':'); if (c1 != std::string::npos) { hold = atoi(tail.c_str() + c1 + 1);
        size_t c2 = tail.find(':', c1 + 1); if (c2 != std::string::npos) gap = atoi(tail.c_str() + c2 + 1); }
      paste_reqs.push_back({f, v.substr(0, at), hold, gap}); }
    else if (a == "--azerty") pc_layout = false;
    else if (a == "--key") {
      std::string s = next(); int len = 3; size_t c = s.find(':');
      if (c != std::string::npos) { len = atoi(s.c_str() + c + 1); s = s.substr(0, c); }
      auto p = split_at(s); auto it = named_keys.find(p.first);
      if (it == named_keys.end()) { fprintf(stderr, "unknown key %s\n", p.first.c_str()); return 3; }
      key_evs.push_back({p.second, it->second, true});
      key_evs.push_back({p.second + len, it->second, false});
    }
    else if (a == "--joy") {
      std::string s = next(); int port = 0, len = 3;
      if (s.size() > 2 && s.substr(s.size() - 2) == "/2") { port = 1; s = s.substr(0, s.size() - 2); }
      size_t c = s.find(':'); if (c != std::string::npos) { len = atoi(s.c_str() + c + 1); s = s.substr(0, c); }
      auto p = split_at(s); uint8_t bits = 0;
      for (char ch : p.first) bits |= ch == 'u' ? 1 : ch == 'r' ? 2 : ch == 'd' ? 4 : ch == 'l' ? 8 : ch == 'a' ? 16 : ch == 'b' ? 32 : 0;
      joy_evs.push_back({p.second, len, port, bits});
    }
    else if (a == "--nmi") {
      std::string s = next(); int len = 3; size_t c = s.find(':');
      if (c != std::string::npos) { len = atoi(s.c_str() + c + 1); s = s.substr(0, c); }
      nmi_evs.push_back({atoi(s.c_str()), len});
    }
    else if (a == "--text") parse_list(next(), text_frames);
    else if (a == "--text-end") text_at_end = true;
    else if (a == "--until") stop_text = next();
    else if (a == "--stop-after-motor") stop_after_motor = atoi(next());
    else if (a == "--peek") { auto p = split_at(next()); unsigned ad = 0, n = 16; sscanf(p.first.c_str(), "%x:%u", &ad, &n); peeks.push_back({p.second, ad, n}); }
    else if (a == "--ramdump") ramdump_path = next();
    else if (a == "--trace") {
      std::string s = next(); size_t c = s.find(':');
      if (c != std::string::npos) {
        std::string r = s.substr(c + 1); s = s.substr(0, c);
        size_t d = r.find(':'); trace_from = atoi(r.c_str());
        if (d != std::string::npos) trace_to = atoi(r.c_str() + d + 1);
      }
      trace_path = s;
    }
    else if (a == "--vcd") { std::string s = next(); size_t c = s.find(':');
      vcd_path = s.substr(0, c); if (c != std::string::npos) sscanf(s.c_str() + c + 1, "%d:%d", &vcd_from, &vcd_to); }
    else if (a == "--io-log") io_log = true;
    else if (a == "--wav") wav_path = next();
    else { fprintf(stderr, "unknown option %s\n", a.c_str()); usage(); return 3; }
  }
  if (run_frames <= 0) { fprintf(stderr, "--frames N is required\n"); return 3; }
  if (rom_path.empty()) { fprintf(stderr, "--rom FILE is required\n"); return 3; }
  for (size_t p = 1; p <= outdir.size(); p++)
    if (p == outdir.size() || outdir[p] == '/') mkdir(outdir.substr(0, p).c_str(), 0755);
  std::stable_sort(key_evs.begin(), key_evs.end(), [](const KeyEv& x, const KeyEv& y) { return x.frame < y.frame; });

  top = new Vvg5000_sim;
#if VM_TRACE
  if (!vcd_path.empty()) { Verilated::traceEverOn(true); vcd = new VerilatedVcdC; top->trace(vcd, 99); vcd->open(vcd_path.c_str()); }
#else
  if (!vcd_path.empty()) { fprintf(stderr, "--vcd needs the trace build (make vcd)\n"); return 3; }
#endif
  top->reset = 1; top->use_ps2 = !paste_reqs.empty(); top->pc_layout = pc_layout; top->ps2_key = 0;
  top->mem_size = mem_size; top->key_matrix = 0; top->joy0 = 0; top->joy1 = 0; top->nmi_key = 0;
  top->tape_rewind = 0; top->tape_turbo = tape_turbo;
  for (int i = 0; i < 16; i++) tick();
  std::vector<uint8_t> rom = read_file(rom_path);
  if (rom.size() > 16384) rom.resize(16384);
  download(0, rom);
  printf("rom: %s, %zu bytes\n", rom_path.c_str(), rom.size());
  auto mount = [&]() {
    std::vector<uint8_t> t = read_file(tape_path);
    if (t.size() > (1u << 23)) { fprintf(stderr, "tape larger than 8 MB\n"); exit(3); }
    download(1, t);   // no rewind pulse: the player must start by itself, as on the MiSTer
    printf("f%d tape: %s, %zu bytes, %s\n", frame, tape_path.c_str(), t.size(), top->tape_is_wav ? "wav" : "k7");
  };
  if (!tape_path.empty() && tape_at < 0) mount();
  recdev.wr = [] { return (bool)top->rec_sd_wr; }; recdev.lba_in = [] { return (uint32_t)top->rec_sd_lba; };
  recdev.din = [] { return (uint8_t)top->rec_sd_buff_din; };
  recdev.ack = [](int v) { top->rec_sd_ack = v; }; recdev.baddr = [](int v) { top->rec_sd_buff_addr = v; };
  top->rec_sd_ack = 0; top->rec_img_mounted = 0;
  if (!rec_path.empty()) {
    recdev.img.assign(rec_size, 0);
    top->rec_img_size = rec_size; top->rec_img_mounted = 1; for (int i = 0; i < 4; i++) tick(); top->rec_img_mounted = 0;
    printf("tape save: blank %zu-byte image\n", rec_size);
  }
  for (int i = 0; i < 64; i++) tick();
  top->reset = 0;

  if (!trace_path.empty()) { trace_f = fopen(trace_path.c_str(), "w"); if (!trace_f) { perror("trace"); return 3; } }
  if (!wav_path.empty()) { wav = fopen(wav_path.c_str(), "wb"); uint8_t h[44] = {0}; fwrite(h, 1, 44, wav); }

  size_t ki = 0;
  uint64_t wav_acc = 0;
  bool prev_io = false, prev_fetch = false, prev_rfsh = false, fetch_done = false;
  uint16_t fetch_pc = 0; uint8_t fetch_byte = 0;
  int exit_code = 0, prev_motor = 0;

  while (frame < run_frames) {
    tick();
    if (!rec_path.empty()) recdev.step();

    bool fetch = !top->cpu_m1_n && !top->cpu_mreq_n && top->cpu_iorq_n && !top->cpu_rd_n;
    if (fetch && !prev_fetch) fetch_pc = top->cpu_a;
    if (fetch) fetch_byte = top->cpu_di;
    if (!fetch && prev_fetch) fetch_done = true;
    bool rfsh = !top->cpu_rfsh_n;
    if (fetch_done && prev_rfsh && !rfsh) { trace_fetch(fetch_pc, fetch_byte); fetch_done = false; }
    prev_rfsh = rfsh; prev_fetch = fetch;

    if (top->ce_pix) {
      bool hb = top->HBlank, vb = top->VBlank;
      if (!hb && !vb) {
        if (vx < FB_W && vy < FB_H) {
          uint8_t* p = &fb[(vy * FB_W + vx) * 3]; p[0] = top->R ? 255 : 0; p[1] = top->G ? 255 : 0; p[2] = top->B ? 255 : 0;
        }
        vx++; line_had_pixels = true;
      }
      if (hb && !prev_hb) {
        if (line_had_pixels) { max_x = std::max(max_x, vx); vy++; }
        vx = 0; line_had_pixels = false;
      }
      prev_hb = hb;
    }

    bool io = !top->cpu_iorq_n && top->cpu_m1_n && !top->cpu_wr_n;
    if (io_log && io && !prev_io) printf("f%d io W %02X <- %02X\n", frame, top->cpu_a & 0xFF, top->cpu_do);
    prev_io = io;

    if (wav) { wav_acc += 44100; if (wav_acc >= CLK_HZ) { wav_acc -= CLK_HZ;
      int16_t s = top->beeper ? 8000 : -8000; fwrite(&s, 2, 1, wav); wav_samples++; } }

    if (tape_log && top->tape_motor != prev_motor) {
      printf("f%d motor %s%s at byte %u%s\n", frame, top->tape_motor ? "ON" : "OFF", top->turbo_active ? " (4x)" : "",
             (unsigned)top->tape_pos, top->tape_ended ? " (ended)" : "");
      fflush(stdout);
    }
    prev_motor = top->tape_motor;

    bool vs = top->VSync;
    if (vs && !prev_vs) {
      if (vy > 0) { last_w = std::min(max_x, FB_W); last_h = std::min(vy, FB_H); }
      vy = 0; vx = 0; max_x = 0;
      frame++;
      if (shots.count(frame) || (every && frame % every == 0)) {
        char name[64]; snprintf(name, sizeof name, "/frame_%05d.png", frame);
        save_frame(outdir + name);
      }
      if (text_frames.count(frame)) print_text(frame);
      for (auto& pk : peeks) if (pk.frame == frame) {
        printf("f%d mem %04X:", frame, pk.addr);
        for (unsigned i = 0; i < pk.n; i++) printf(" %02X", ram_rd(pk.addr + i));
        printf("\n");
      }
      if (!tape_path.empty() && tape_at == frame) mount();
      for (auto& p : paste_reqs) if (p.frame == frame) {
        expand_paste(p, cycles);
        std::sort(ps2_evs.begin() + ps2_i, ps2_evs.end(), [](const Ps2Ev& a, const Ps2Ev& b) { return a.cycle < b.cycle; }); }
      while (ki < key_evs.size() && key_evs[ki].frame <= frame) {
        KeyEv& e = key_evs[ki++]; int bit = e.rc.row * 8 + e.rc.col;
        key_count[bit] += e.down ? 1 : -1;
        if (key_count[bit] > 0) top->key_matrix |= 1ull << bit; else top->key_matrix &= ~(1ull << bit);
      }
      uint8_t j0 = 0, j1 = 0;
      for (auto& e : joy_evs) if (frame >= e.frame && frame < e.frame + e.len) (e.port ? j1 : j0) |= e.bits;
      top->joy0 = j0; top->joy1 = j1;
      bool nmi = false;
      for (auto& e : nmi_evs) if (frame >= e.frame && frame < e.frame + e.len) nmi = true;
      top->nmi_key = nmi;

      // sweeps: report the first frame a tape message appears (the program may clear it later)
      if (!tape_path.empty() && frame % 10 == 0) {
        static bool seen_found = false, seen_err = false;
        std::string scr = screen_text();
        size_t p = scr.find("Trouv");
        if (!seen_found && p != std::string::npos) {
          seen_found = true; size_t e = scr.find('\n', p);
          printf("f%d seen: %s\n", frame, scr.substr(p, e - p).c_str());
        }
        if (!seen_err) for (const char* m : {"Erreur", "rreur", "Err "}) if (scr.find(m) != std::string::npos) {
          seen_err = true; size_t q = scr.find(m), b = scr.rfind('\n', q), e = scr.find('\n', q);
          printf("f%d seen error: %s\n", frame, scr.substr(b + 1, e - b - 1).c_str()); break;
        }
      }
      // --until: the text must appear more often than when the last typed key went in
      if (!stop_text.empty() && frame % 10 == 0) {
        std::string scr = screen_text(); int cnt = 0;
        for (size_t p = scr.find(stop_text); p != std::string::npos; p = scr.find(stop_text, p + 1)) cnt++;
        bool typing = ki < key_evs.size() || ps2_i < ps2_evs.size() || !paste_reqs.empty() && frame <= paste_reqs.back().frame;
        if (typing || until_base < 0) until_base = cnt;
        else if (cnt > until_base) {
          printf("f%d found \"%s\" on screen\n", frame, stop_text.c_str()); stop_text.clear(); break;
        }
      }
      if (top->tape_motor) { motor_was_on = true; motor_off_frame = -1; }
      else if (motor_was_on && motor_off_frame < 0) motor_off_frame = frame;
      if (stop_after_motor >= 0 && motor_off_frame >= 0 && frame - motor_off_frame >= stop_after_motor) {
        printf("f%d stop: motor off since f%d\n", frame, motor_off_frame); break;
      }
      if (frame % 500 == 0) fprintf(stderr, "frame %d\n", frame);
    }
    prev_vs = vs;
  }
  if (!stop_text.empty()) { printf("\"%s\" not found\n", stop_text.c_str()); exit_code = 2; }

  if (!png_path.empty()) save_frame(png_path);
  if (text_at_end) print_text(frame);
  if (!ramdump_path.empty()) {
    FILE* f = fopen(ramdump_path.c_str(), "wb");
    if (f) { for (unsigned a = 0; a < 0x10000; a++) fputc(ram_rd(a), f); fclose(f); }
  }
  if (!rec_path.empty()) {
    for (int i = 0; i < 20000; i++) { tick(); recdev.step(); }   // let the last sector go out
    size_t n = std::min((size_t)top->rec_length, recdev.img.size());
    FILE* f = fopen(rec_path.c_str(), "wb"); if (f) { fwrite(recdev.img.data(), 1, n, f); fclose(f); }
    printf("tape save: %zu bytes, %llu sector writes -> %s\n", n, (unsigned long long)recdev.writes, rec_path.c_str());
  }
  if (trace_f) { fclose(trace_f); printf("trace: %llu instructions -> %s\n", (unsigned long long)trace_lines, trace_path.c_str()); }
#if VM_TRACE
  if (vcd) { vcd->close(); }
#endif
  printf("done: %d frames, %llu cycles, last frame %dx%d\n", frame, (unsigned long long)cycles, last_w, last_h);
  wav_close();
  delete top;
  return exit_code;
}
