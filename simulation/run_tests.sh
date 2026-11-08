#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
build="$(mktemp -d)"
trap 'rm -rf "$build"' EXIT
cd "$build"
compiler=("${IVERILOG:-iverilog}")
if [[ -n "${IVERILOG_BASE:-}" ]]; then compiler+=(-B "$IVERILOG_BASE"); fi
for test in tb_i2c_master tb_adxl345_top tb_hcsr04 tb_oled tb_arbiter; do
    case "$test" in
        tb_i2c_master) bench="$root/simulation/$test.v" ;;
        tb_adxl345_top) bench="$root/tb/$test.v" ;;
        *) bench="$root/tb/$test.sv" ;;
    esac
    "${compiler[@]}" -g2012 -s "$test" -o "$build/$test" "$bench" "$root"/src/*.sv
    "${VVP:-vvp}" "$build/$test"
done
"${compiler[@]}" -g2012 -s top_module -o "$build/top" "$root"/src/*.sv
