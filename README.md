# FPGA anti-collision system

Gowin GW1NR-9C design using a 27 MHz clock, an HC-SR04 distance sensor,
an ADXL345 accelerometer, an SSD1306 OLED, LEDs and buzzer.
The accelerometer and OLED use separate I2C buses in `top_module`.

Open `anti-collision_system.gprj` in Gowin to synthesize and place/route.
Pin assignments are in `src/constraint.cst`; the input clock constraint is
in `src/timing.sdc`. Rebuild before programming: existing `impl/` artifacts
do not include the latest source fixes.

Run the self-checking simulations with Icarus Verilog installed:

```sh
bash simulation/run_tests.sh
```

The runner builds in a temporary directory and checks I2C ACK/NACK recovery
and clock stretching, accelerometer register transfers and angle saturation,
ultrasonic pulse timing/filtering/timeout recovery, complete OLED frame
transfers, shared-bus arbitration and top-level compilation.
`IVERILOG`, `IVERILOG_BASE` and `VVP` can select a custom Icarus installation.

For an additional RTL structural check with Yosys:

```sh
yosys -p 'read_verilog -sv src/*.sv; hierarchy -check -top top_module; proc; check -assert'
```

Simulations do not replace Gowin timing closure or on-board sensor/display
validation.
