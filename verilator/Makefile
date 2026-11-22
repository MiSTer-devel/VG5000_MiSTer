# Verilator simulator for the VG5000 core. Needs Verilator 5.x and zlib.
#   make            obj_dir/Vvg5000_sim (clk = clk_sys 48 MHz, hardware clock enables)
#   make vcd        obj_vcd/Vvg5000_sim (--vcd support)
#   make test       regression (tests/run_tests.sh) + keyboard unit test
#   make clean
# The EF9345 is rtl/ef9345/ef9345_vg5k.sv when it exists,
# otherwise the behavioural stand-in stub/ef9345_vg5k_stub.sv.
RTL  = ../rtl
TOP  = vg5000_sim
EXE  = $(MDIR)/V$(TOP)
ROMS = files

# EF=stub forces the stand-in (also: obj_stub/ via make stub)
EF ?= $(if $(wildcard $(RTL)/ef9345/ef9345_vg5k.sv),real,stub)
ifeq ($(EF),stub)
EF_SRC   = stub/ef9345_vg5k_stub.sv
EF_FLAGS = +define+EF9345_CHARSET_HEX=\"ef9345_charset.hex\"
MDIR    ?= obj_stub
else
EF_SRC   = $(filter-out %_tb.sv,$(wildcard $(RTL)/ef9345/*.sv))
EF_FLAGS = +incdir+$(RTL)/ef9345 $(wildcard $(RTL)/ef9345/ef9345_lint.vlt) \
	+define+EF9345_ROMCHAR0_HEX=\"../rtl/ef9345/romchar0.hex\" +define+EF9345_ROMCHAR1_HEX=\"../rtl/ef9345/romchar1.hex\"
MDIR    ?= obj_dir
endif

V_SRC = sim.vlt $(TOP).sv \
	$(RTL)/vg5000.sv $(RTL)/vg5k_clocks.sv $(RTL)/vg5k_keyboard.sv $(RTL)/vg5k_tape.sv $(RTL)/vg5k_wav.sv $(RTL)/vg5k_tape_rec.sv \
	$(RTL)/t80sv/t80sv_all.sv $(EF_SRC)

V_FLAGS = --cc --exe --build -j 0 --top-module $(TOP) --no-timing -Wno-fatal -Wno-lint -Wno-style \
	--x-assign fast --x-initial fast --noassert -O3 +incdir+$(RTL) $(EF_FLAGS) \
	-CFLAGS "-O2 -std=c++17" -LDFLAGS "-lz"

.PHONY: all stub vcd test clean
all: $(EXE) $(if $(filter stub,$(EF)),ef9345_charset.hex)

stub:
	$(MAKE) EF=stub


ef9345_charset.hex: $(ROMS)/charset.rom
	xxd -p -c1 $< > $@

$(EXE): $(V_SRC) sim_main.cpp Makefile
	verilator $(V_FLAGS) --Mdir $(MDIR) $(V_SRC) sim_main.cpp

vcd: obj_vcd/V$(TOP) $(if $(filter stub,$(EF)),ef9345_charset.hex)
obj_vcd/V$(TOP): $(V_SRC) sim_main.cpp Makefile
	verilator $(V_FLAGS) --trace --Mdir obj_vcd $(V_SRC) sim_main.cpp

test: all tests/kbd/obj/Vvg5k_keyboard
	mkdir -p tests/out && ./tests/kbd/obj/Vvg5k_keyboard | tail -1 && ( ./tests/run_tests.sh; ./tests/tape_save_roundtrip.sh ) | tee tests/out/summary.txt; ! grep -q "^FAIL\|^RUNERR" tests/out/summary.txt

test-stub: stub tests/kbd/obj/Vvg5k_keyboard
	SIM=./obj_stub/Vvg5000_sim ./tests/run_tests.sh

tests/kbd/obj/Vvg5k_keyboard: $(RTL)/vg5k_keyboard.sv tests/kbd/kbd_tb.cpp
	verilator --cc --exe --build -j 8 -GCLK_HZ=2500 -Wno-fatal $(RTL)/vg5k_keyboard.sv tests/kbd/kbd_tb.cpp -Mdir tests/kbd/obj -o Vvg5k_keyboard

clean:
	rm -rf obj_dir obj_stub obj_vcd out tests/out tests/kbd/obj
