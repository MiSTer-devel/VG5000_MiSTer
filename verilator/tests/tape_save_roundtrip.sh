#!/bin/sh
# Tape save round trip: CSAVE into a blank image (rtl/vg5k_tape_rec.sv), then
# CLOAD the recorded .k7 in a new run (rtl/vg5k_tape.sv) and RUN it.
cd "$(dirname "$0")/.." || exit 1
SIM=${SIM:-./obj_dir/Vvg5000_sim}
ROM=files/vg5k11.bin
OUT=tests/out; mkdir -p "$OUT"
[ -f "$ROM" ] || { echo "SKIP    tape_save_roundtrip (no ROM)"; exit 0; }
"$SIM" --rom "$ROM" --type '10 PRINT "SAVED OK"\nCSAVE "TEST"\n@120' --rec "$OUT/rec.k7" --frames 900 > "$OUT/rec_save.log" 2>&1
"$SIM" --rom "$ROM" --tape "$OUT/rec.k7" --type 'CLOAD\n@120' --type 'RUN\n@900' --frames 1100 --text-end > "$OUT/rec_load.log" 2>&1
if [ "$(head -c 11 "$OUT/rec.k7" | od -An -tx1 | tr -d " \n")" = "d3d3d3d3d3d3d3d3d3d320" ] &&
   sed -n '/^--- screen/,$p' "$OUT/rec_load.log" | grep -q "Trouv.:TEST" &&
   sed -n '/^--- screen/,$p' "$OUT/rec_load.log" | grep -q "^| SAVED OK"; then
  echo "PASS    tape_save_roundtrip"
else
  echo "FAIL    tape_save_roundtrip ($OUT/rec.k7, $OUT/rec_load.log)"; exit 1
fi
