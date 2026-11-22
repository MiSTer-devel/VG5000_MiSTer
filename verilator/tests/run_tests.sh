#!/bin/sh
# Regression runner (after SC3000_MiSTer): one headless sim per line of tests/regress.tsv,
# in parallel. PASS / FAIL / RUNERR / SKIP (missing tape or ROM). '_' in args = space.
# SIM=./obj_stub/Vvg5000_sim tests/run_tests.sh   runs with the EF9345 stand-in.
cd "$(dirname "$0")/.." || exit 1
set -f
SIM=${SIM:-./obj_dir/Vvg5000_sim}
[ -x "$SIM" ] || SIM=./obj_stub/Vvg5000_sim
ROM=files/vg5k11.bin
OUT=tests/out
mkdir -p "$OUT"
JOBS=${JOBS:-$(sysctl -n hw.ncpu 2>/dev/null || nproc)}
[ -f "$ROM" ] || { echo "SKIP    all (no $ROM)"; exit 0; }
echo "sim: $SIM"

grep -v '^#' tests/regress.tsv | grep -v '^$' > "$OUT/list.tsv"

run_one() {
  IFS='	' read -r name tape args frames expect <<LINE
$1
LINE
  if [ "$tape" != "-" ] && [ ! -f "$tape" ]; then echo "SKIP    $name (no $tape)"; return; fi
  set --
  [ "$args" = "-" ] && args=
  for a in $args; do set -- "$@" "$(printf '%s' "$a" | tr '_' ' ')"; done
  case " $* " in *" --rom "*) ;; *) set -- --rom "$ROM" "$@" ;; esac
  [ "$tape" != "-" ] && set -- --tape "$tape" "$@"
  "$SIM" --frames "$frames" --png "$OUT/$name.png" --text-end "$@" > "$OUT/$name.log" 2>&1
  rc=$?
  if [ $rc -ne 0 ] && [ $rc -ne 2 ]; then echo "RUNERR  $name (exit $rc, $OUT/$name.log)"; return; fi
  case "$expect" in
    ok:*) want=${expect#ok:}
          cols=$(grep " shot $OUT/$name.png " "$OUT/$name.log" | awk '{print $NF}')
          if ! sed -n '/^--- screen/,$p' "$OUT/$name.log" | grep -qF -- "$want"; then echo "FAIL    $name (\"$want\" not on screen)"
          elif [ "${cols:-0}" -lt 2 ]; then echo "FAIL    $name (picture has $cols colour(s), $OUT/$name.png)"
          else echo "PASS    $name ($OUT/$name.png)"; fi ;;
    text:*) want=${expect#text:}
          if sed -n '/^--- screen/,$p' "$OUT/$name.log" | grep -qF -- "$want"; then echo "PASS    $name"
          else echo "FAIL    $name (\"$want\" not on screen)"; fi ;;
  esac
}
export -f run_one 2>/dev/null
export SIM OUT ROM
# parallel without GNU parallel: a background job per line, JOBS at a time
n=0
while IFS= read -r line; do
  run_one "$line" &
  n=$((n + 1)); if [ $n -ge "$JOBS" ]; then wait; n=0; fi
done < "$OUT/list.tsv"
wait
