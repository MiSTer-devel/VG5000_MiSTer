// Unit test for rtl/vg5k_keyboard.sv: PS/2 events in, VG5000 matrix out
// (adapted from SC3000_MiSTer verilator/tests/kbd/kbd_tb.cpp). GAP = 100 clocks (CLK_HZ 2500).
#include "Vvg5k_keyboard.h"
#include "verilated.h"
#include <cstdio>
#include <vector>
#include <string>
static Vvg5k_keyboard* k; static int fails = 0, toggle = 0;
// the machine reads rows 7..0 one per scan_div clocks (port 87 .. 80), with a read pulse
static bool scanning = true; static int scan_row = 7, scan_div = 1, scan_cnt = 0;
static void tick(int n = 1) {
  while (n--) {
    k->row_rd = 0;
    if (scanning && ++scan_cnt >= scan_div) { scan_cnt = 0; k->row_sel = scan_row; k->row_rd = 1; scan_row = (scan_row + 7) % 8; }
    k->clk_sys = 0; k->eval(); k->clk_sys = 1; k->eval();
  }
  k->row_rd = 0;
}
static void ps2(int code, bool down) { toggle ^= 1; k->ps2_key = (code & 0x1FF) | (down << 9) | (toggle << 10); tick(4); }
static bool at(int row, int col) { k->row_sel = row; k->eval(); return !((k->row_data >> col) & 1); }
static void expect(const char* what, int row, int col, int shift) {
  tick(400);
  bool p = at(row, col), sh = at(0, 2);
  bool ok = p && (shift < 0 || sh == (bool)shift);
  printf("%-34s row %d col %d pressed=%d shift=%d  %s\n", what, row, col, p, sh, ok ? "ok" : "FAIL");
  if (!ok) fails++;
}
struct Ev { int pos; bool sh; };
static std::vector<Ev> watch(int ticks) {
  std::vector<Ev> out; static int prev = -1;
  for (int i = 0; i < ticks; i++) {
    int now = -1; tick();
    for (int r = 0; r < 8; r++) { int d = ~at(r, 0) ? 0 : 0; (void)d; k->row_sel = r; k->eval();
      int v = ~k->row_data & 0xFF; if (r == 0) v &= ~0x04;
      for (int c = 0; c < 8; c++) if (v >> c & 1) now = r * 8 + c; }
    bool sh = at(0, 2);
    if (now != prev && now >= 0) out.push_back({now, sh});
    prev = now;
  }
  return out;
}
static int code_of(char ch, bool& shift) {
  const char* lo = "1234567890 ,;"; const int lc[] = {0x16,0x1E,0x26,0x25,0x2E,0x36,0x3D,0x3E,0x46,0x45,0x29,0x41,0x4C};
  shift = false;
  for (int i = 0; lo[i]; i++) if (lo[i] == ch) return lc[i];
  if (ch == ':') { shift = true; return 0x4C; }
  if (ch == '\n') return 0x5A;
  const char* up = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"; const int uc[] = {0x1C,0x32,0x21,0x23,0x24,0x2B,0x34,0x33,0x43,0x3B,0x42,0x4B,0x3A,0x31,0x44,0x4D,0x15,0x2D,0x1B,0x2C,0x3C,0x2A,0x1D,0x22,0x35,0x1A};
  for (int i = 0; up[i]; i++) if (up[i] == ch) { shift = true; return uc[i]; }
  return -1;
}
static void type(const std::string& t) {
  for (char ch : t) { bool sh; int c = code_of(ch, sh); if (c < 0) continue;
    if (sh) ps2(0x12, 1); ps2(c, 1); ps2(c, 0); if (sh) ps2(0x12, 0); }
}

int main() {
  k = new Vvg5k_keyboard; k->pc_layout = 1; k->reset = 1; tick(10); k->reset = 0; tick(10);
  ps2(0x1C, 1); expect("PC a -> VG A key (row 1 b7)", 1, 7, 0); ps2(0x1C, 0); tick(300);
  ps2(0x15, 1); expect("PC q -> VG Q key (row 1 b1)", 1, 1, 0); ps2(0x15, 0); tick(300);
  ps2(0x12, 1); ps2(0x1A, 1); expect("Shift+z -> VG Z + Shift", 2, 7, 1); ps2(0x1A, 0); ps2(0x12, 0); tick(300);
  ps2(0x16, 1); expect("1 -> '1' key, VG Shift off", 2, 5, 0); ps2(0x16, 0); tick(300);
  ps2(0x12, 1); ps2(0x16, 1); expect("Shift+1 (!) -> Shift + '2' key", 3, 6, 1); ps2(0x16, 0); ps2(0x12, 0); tick(300);
  ps2(0x12, 1); ps2(0x4C, 1); expect("Shift+; (:) -> ':' key unshifted", 2, 6, 0); ps2(0x4C, 0); ps2(0x12, 0); tick(300);
  ps2(0x12, 1); ps2(0x3E, 1); expect("Shift+8 (*) -> Shift + ':'", 2, 6, 1); ps2(0x3E, 0); ps2(0x12, 0); tick(300);
  ps2(0x49, 1); expect(". -> Shift + '+'", 6, 1, 1); ps2(0x49, 0); tick(300);
  ps2(0x12, 1); ps2(0x55, 1); expect("Shift+= (+) -> '+' unshifted", 6, 1, 0); ps2(0x55, 0); ps2(0x12, 0); tick(300);
  ps2(0x5A, 1); expect("Enter -> RET", 1, 5, -1); ps2(0x5A, 0); tick(300);
  ps2(0x169, 1); tick(10); bool nmi = k->nmi_key; ps2(0x169, 0); tick(10);
  printf("%-34s %s\n", "End -> delta (NMI)", nmi && !k->nmi_key ? "ok" : "FAIL"); if (!(nmi && !k->nmi_key)) fails++;
  bool stuck = false; for (int r = 0; r < 8; r++) if ((k->row_data, at(r, 0), (~k->row_data & 0xFF) != 0)) stuck = true;
  printf("all released: %s\n", stuck ? "FAIL" : "ok"); if (stuck) fails++;
  {
    std::string line = "10 PRINT 12:GOTO 10\n";
    std::vector<Ev> ref;
    for (char ch : line) { type(std::string(1, ch)); auto e = watch(1000); ref.insert(ref.end(), e.begin(), e.end()); }
    std::string paste; for (int i = 0; i < 20; i++) paste += line;
    std::vector<Ev> got;
    for (char ch : paste) { type(std::string(1, ch)); auto e = watch(1); got.insert(got.end(), e.begin(), e.end()); }
    ps2(0x12, 1);   // Shift held while the queue drains: queued keys keep the Shift they were typed with
    auto rest = watch(150000); ps2(0x12, 0); got.insert(got.end(), rest.begin(), rest.end());
    bool ok = got.size() == ref.size() * 20;
    for (size_t i = 0; ok && i < got.size(); i++) { const Ev& a = got[i]; const Ev& b = ref[i % ref.size()];
      if (a.pos != b.pos || a.sh != b.sh) { ok = false; printf("paste: key %zu pos %d sh %d, want %d %d\n", i, a.pos, a.sh, b.pos, b.sh); } }
    printf("paste of %zu chars: %zu presses (want %zu)  %s\n", paste.size(), got.size(), ref.size() * 20, ok ? "ok" : "FAIL");
    if (!ok) fails++;
  }
  {
    tick(2000);
    scanning = false; k->row_sel = 5;
    ps2(0x1C, 1); ps2(0x1C, 0); ps2(0x32, 1); ps2(0x32, 0);   // a, b
    tick(3000);
    bool a_held = at(1, 7), b_wait = !at(2, 4);
    scanning = true;
    auto e = watch(2000);
    bool ok = a_held && b_wait && e.size() >= 1 && e.back().pos == 2 * 8 + 4;
    printf("busy machine: 'a' held %d, 'b' waits %d, then 'b'  %s\n", a_held, b_wait, ok ? "ok" : "FAIL");
    if (!ok) fails++;
  }
  k->pc_layout = 0; ps2(0x15, 1); expect("AZERTY: PC q position -> VG A", 1, 7, 0); ps2(0x15, 0); tick(300);
  printf("%s\n", fails ? "FAILED" : "PASSED"); k->final(); delete k; return fails != 0;
}
