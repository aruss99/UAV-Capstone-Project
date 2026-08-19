#!/usr/bin/env bash
# build_hdl.sh - lint, simulate and synthesise the Verilog sources.
#
# Uses the oss-cad-suite already installed at C:\oss-cad-suite.
#   iverilog  - does each file even compile?
#   vvp       - run the benches, measure the PWM outputs for real
#   yosys     - synthesise for iCE40HX1K and report ACTUAL LUT utilisation
#               (project estimates put this at 11% for 2 PWM + debounce and
#                ~25% for serial output; this measures it)

set -uo pipefail
export PATH="/c/oss-cad-suite/bin:/c/oss-cad-suite/lib:$PATH"

WB="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HDL="$WB/hdl"
OUT="$WB/out/hdl"
mkdir -p "$OUT"

hr() { printf '%s\n' "----------------------------------------------------------------------"; }

echo
echo "======================================================================"
echo " 1. COMPILE CHECK  (iverilog -t null)"
echo "======================================================================"
# i2cmodule.v and HDLDroneModel3.v depend on modules that are not present
# (OpenCores I2C byte controller; HDL Coder helpers). Black-box stubs in tb/
# let them elaborate so the rest of the file can actually be checked.
declare -A EXTRA=(
  [i2cmodule.v]="tb/i2c_master_byte_ctrl_stub.v"
  [HDLDroneModel3.v]="tb/hdlcoder_stubs.v"
  [shift_register.v]=""
)
for f in PWM.v ESC_PWM.v CODE_WORKS/PWM.v CODE_WORKS/ESC_PWM.v \
         mojo_spi.v i2cmodule.v shift_register.v HDLDroneModel3.v; do
  printf '%-28s ' "$f"
  extra="${EXTRA[$f]:-}"
  if err=$( cd "$HDL" && iverilog -g2012 -t null $extra "$f" 2>&1); then
    if [ -n "$err" ]; then echo "PASS (with warnings)"; sed 's/^/      /' <<<"$err" | head -6
    else echo "PASS"; fi
  else
    echo "FAIL"; sed 's/^/      /' <<<"$err" | head -8
  fi
done

echo
echo "======================================================================"
echo " 2. PWM MEASUREMENT  (12 MHz clock, buttons idle)"
echo "======================================================================"
# Both should measure 50 Hz servo / 500 Hz ESC. A PWM counter too narrow to
# reach its terminal count wraps early and gives 91.55 Hz on the ESC channel.
for variant in "ESC_PWM.v:hdl/ESC_PWM.v" "CODE_WORKS/ESC_PWM.v:hdl/CODE_WORKS/ESC_PWM.v"; do
  src="${variant%%:*}"; label="${variant#*:}"
  hr; echo " $label"; hr
  vcd="$OUT/$(echo "$src" | tr '/.' '__').vcd"
  if iverilog -g2012 -o "$OUT/sim.vvp" -DDUMPFILE="\"$vcd\"" \
       -s esc_pwm_measure_tb "$HDL/$src" "$HDL/tb/esc_pwm_measure_tb.v" 2>&1 | sed 's/^/  /'; then
    vvp "$OUT/sim.vvp" 2>&1 | grep -vE '^VCD info|^$' | sed 's/^/ /'
    echo "  vcd: $vcd"
  fi
done

echo
echo "======================================================================"
echo " 3. CODE_WORKS TESTBENCH  (CODE_WORKS/ESC_PWM_tb.v, as written)"
echo "======================================================================"
if iverilog -g2012 -o "$OUT/orig_tb.vvp" -s PWM_Generator_Verilog_tb \
     "$HDL/CODE_WORKS/ESC_PWM.v" "$HDL/CODE_WORKS/ESC_PWM_tb.v" 2>&1 | sed 's/^/  /'; then
  echo "  compiles and elaborates OK"
  timeout 60 vvp "$OUT/orig_tb.vvp" 2>&1 | head -20 | sed 's/^/  /'
  echo "  (bench drives buttons only; it prints nothing and dumps no VCD)"
fi

hr
echo " shift8 + its testbench"
hr
if iverilog -g2012 -o "$OUT/shift8.vvp" -s shift8_tb \
     "$HDL/shift_register.v" "$HDL/shift_register_tb.v" 2>&1 | sed 's/^/  /'; then
  timeout 60 vvp "$OUT/shift8.vvp" 2>&1 | sed 's/^/ /'
fi

echo
echo "======================================================================"
echo " 4. iCE40HX1K SYNTHESIS  (yosys synth_ice40)   budget: 1280 LUT4"
echo "======================================================================"
# yosys `stat` prints one block per module in the design plus a design-wide
# summary; take only the last block, which is the whole-design total.
report_stat() {
  local log="$1"
  local blk
  blk=$(awk '/=== design hierarchy ===|=== [A-Za-z_0-9\\]+ ===/{buf=""} {buf=buf $0 ORS} END{printf "%s", buf}' "$log")
  # fall back to the final "Printing statistics" section
  blk=$(awk '/Printing statistics/{n++} n{print}' "$log" | tail -40)
  # `stat` may appear more than once in the log; c[$2]=$1 keeps the last value
  # per cell type rather than summing duplicates across blocks.
  echo "$blk" | grep -E '^ +[0-9]+ +SB_(LUT4|DFF[A-Z]*|CARRY|RAM[0-9A-Z]*)$' \
    | awk '{c[$2]=$1}
           END{ for(k in c) printf "  %-12s %6d\n", k, c[k] }' | sort
  echo "$blk" | grep -E '^ +[0-9]+ +SB_(LUT4|DFF[A-Z]*)$' \
    | awk '{c[$2]=$1}
           END{ for(k in c) if(k=="SB_LUT4") l=c[k]; else f+=c[k];
                if(l) printf "  => %d LUT4 = %.1f%% of the iCE40HX1K (1280 LUT4);  %d flip-flops\n", l, 100*l/1280, f }'
}

# yosys tokenises its -p script on whitespace, so a path containing a space
# breaks it. Run from $HDL and pass relative paths instead.
synth() {
  local label="$1" top="$2"; shift 2
  hr; echo " $label   (top = $top)"; hr
  local log="$OUT/yosys_${label// /_}.log"
  if ( cd "$HDL" && yosys -p "read_verilog -sv $*; synth_ice40 -top $top; stat" ) > "$log" 2>&1; then
    report_stat "$log"
  else
    echo "  SYNTHESIS FAILED - see $log"
    grep -iE '^ERROR|error:' "$log" | head -5 | sed 's/^/    /'
  fi
}

synth "servo PWM only"        PWM_Generator_Verilog PWM.v
synth "servo+ESC PWM"         PWM_Generator_Verilog CODE_WORKS/ESC_PWM.v
synth "SPI master"            spi                   mojo_spi.v
synth "servo+ESC PWM and SPI" top_pwm_spi           tb/top_pwm_spi.v CODE_WORKS/ESC_PWM.v mojo_spi.v

hr
echo " generated controller HDLDroneModel3.v  (top-level glue ONLY)"
hr
echo "  8 modules it instantiates are not present:"
echo "    HDLDroneModel3_tc, Discrete_PID_Controller{,1,2},"
echo "    nfp_wire_single, nfp_mul_single, nfp_sub_single, nfp_gain_pow2_single"
echo "  Black-boxing them costs the 3 PI controllers and every floating-point"
echo "  operator, so the number below is a FLOOR for the real design."
log="$OUT/yosys_generated_controller_blackbox.log"
if ( cd "$HDL" && yosys -p "
      read_verilog -sv tb/hdlcoder_stubs.v HDLDroneModel3.v;
      synth_ice40 -top HDLDroneModel3;
      stat" ) > "$log" 2>&1; then
  report_stat "$log"
else
  echo "  synthesis failed even with black boxes - see $log"
  grep -iE '^ERROR' "$log" | head -3 | sed 's/^/    /'
fi

echo
echo "logs and VCDs in $OUT"
echo "view a waveform with:  /c/oss-cad-suite/bin/gtkwave <file>.vcd"
echo
