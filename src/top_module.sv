// HC-SR04 x4 + ADXL345 + OLED build. Full system (ADXL345 + OLED + buzzer): top_module_full.sv.bak
module top_module (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [3:0] echo_pin,     // [i] = HC-SR04_(i+1), numbered left to right
    output wire [3:0] trig_pin,
    inout  wire       scl,          // shared I2C: ADXL345 (0x53) + SSD1306 (0x3C)
    inout  wire       sda,
    output wire       hcsr04_led,
    output wire       buzzer
);
    // ─── HC-SR04 x4 ───────────────────────────────────────────────
    // Pulses < 2 cm are ignored as noise; >400 cm, missing or stuck ECHO
    // give sample_valid=0.
    wire [3:0]  dist_valid;     // one-cycle pulse per measurement
    wire [3:0]  sample_valid;
    wire [35:0] distance_cm;    // sensor i at [i*9 +: 9]

    genvar s;
    generate
        for (s = 0; s < 4; s = s + 1) begin : g_hcsr04
            hcsr04_measure #(.CLK_FREQ_HZ(27_000_000)) u_hcsr04 (
                .clk              (clk),
                .rst_n            (rst_n),
                .echo             (echo_pin[s]),
                .trig             (trig_pin[s]),
                .sample_done      (dist_valid[s]),
                .sample_valid     (sample_valid[s]),
                .error_code       (),
                .echo_width_cycles(),
                .echo_width_us    (),
                .distance_cm      (distance_cm[s*9 +: 9])
            );
        end
    endgenerate

    // ─── Warning zone per sensor ──────────────────────────────────
    // 0: >20 cm / invalid -> off, 1: 10-20 cm -> 0.5 Hz,
    // 2: 5-9 cm -> 1 Hz,          3: <5 cm -> 10 Hz
    function [1:0] dist_zone;
        input       valid;
        input [8:0] d;
        begin
            if (!valid || d > 9'd20) dist_zone = 2'd0;
            else if (d >= 9'd10)     dist_zone = 2'd1;
            else if (d >= 9'd5)      dist_zone = 2'd2;
            else                     dist_zone = 2'd3;
        end
    endfunction

    reg [7:0] sensor_zone;      // sensor i at [i*2 +: 2]
    integer   z;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sensor_zone <= 8'd0;
        else
            for (z = 0; z < 4; z = z + 1)
                if (dist_valid[z])
                    sensor_zone[z*2 +: 2] <= dist_zone(sample_valid[z],
                                                       distance_cm[z*9 +: 9]);
    end

    // OLED "!" per sensor; the most urgent sensor drives LED + buzzer.
    wire [3:0] obstacle = {|sensor_zone[7:6], |sensor_zone[5:4],
                           |sensor_zone[3:2], |sensor_zone[1:0]};
    wire [1:0] zone_01  = (sensor_zone[1:0] > sensor_zone[3:2]) ?
                          sensor_zone[1:0] : sensor_zone[3:2];
    wire [1:0] zone_23  = (sensor_zone[5:4] > sensor_zone[7:6]) ?
                          sensor_zone[5:4] : sensor_zone[7:6];
    wire [1:0] zone     = (zone_01 > zone_23) ? zone_01 : zone_23;

    reg [1:0] zone_prev;
    wire      zone_changed = (zone != zone_prev);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            zone_prev <= 2'd0;
        else
            zone_prev <= zone;
    end

    // ─── Shared LED/buzzer blink: 10 Hz / 1 Hz / 0.5 Hz ───────────
    reg [24:0] blink_half;      // half period in 27 MHz cycles

    always @(*) begin
        case (zone)
            2'd3:    blink_half = 25'd1_350_000;    // 10 Hz, 0.1 s period
            2'd2:    blink_half = 25'd13_500_000;   // 1 Hz, 1 s period
            default: blink_half = 25'd27_000_000;   // 0.5 Hz, 2 s period
        endcase
    end

    reg [24:0] blink_cnt;
    reg        blink_on;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            blink_cnt <= 25'd0;
            blink_on  <= 1'b0;
        end else if (zone == 2'd0) begin
            blink_cnt <= 25'd0;
            blink_on  <= 1'b0;
        end else if (zone_changed) begin
            // Each new level starts ON so the warning is immediate.
            blink_cnt <= 25'd0;
            blink_on  <= 1'b1;
        end else if (blink_cnt >= blink_half - 1'b1) begin
            blink_cnt <= 25'd0;
            blink_on  <= ~blink_on;
        end else
            blink_cnt <= blink_cnt + 1'b1;
    end

    // ─── Buzzer tone 2 kHz (passive buzzer, as in full design) ────
    localparam BUZ_HALF = 14'd6750;
    reg [13:0] buz_pwm_cnt;
    reg        buz_pwm;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buz_pwm_cnt <= 14'd0;
            buz_pwm     <= 1'b0;
        end else if (buz_pwm_cnt >= BUZ_HALF - 1) begin
            buz_pwm_cnt <= 14'd0;
            buz_pwm     <= ~buz_pwm;
        end else
            buz_pwm_cnt <= buz_pwm_cnt + 1'b1;
    end

    assign hcsr04_led = blink_on;
    assign buzzer     = blink_on & buz_pwm;

    // ─── ADXL345: X tilt = steering angle ─────────────────────────
    wire [7:0] steer_angle;     // 0..90
    wire       steer_sign;      // 0 = right, 1 = left

    wire       adxl_ena, adxl_rw, adxl_stop, adxl_busy, adxl_ack_err;
    wire [6:0] adxl_addr;
    wire [7:0] adxl_data_wr, adxl_data_rd;

    adxl345_top u_adxl345 (
        .clk              (clk),
        .rst_n            (rst_n),
        .i2c_ena          (adxl_ena),
        .i2c_addr         (adxl_addr),
        .i2c_rw           (adxl_rw),
        .i2c_data_wr      (adxl_data_wr),
        .i2c_stop         (adxl_stop),
        .i2c_busy         (adxl_busy),
        .i2c_data_rd      (adxl_data_rd),
        .i2c_ack_err      (adxl_ack_err),
        .steer_dir        (),
        .danger_threshold (),
        .steer_angle      (steer_angle),
        .steer_sign       (steer_sign),
        .y_angle          (),
        .y_sign           (),
        .valid            (),
        .accel_raw        ()
    );

    // ─── OLED: car outline, "!" per sensor, reverse guide lines ───
    wire       oled_ena, oled_rw, oled_stop, oled_busy, oled_ack_err;
    wire [6:0] oled_addr;
    wire [7:0] oled_data_wr, oled_data_rd;

    ssd1306_oled u_oled (
        .clk              (clk),
        .rst_n            (rst_n),
        .obstacle         (obstacle),
        .steer_angle      (steer_angle),
        .steer_sign       (steer_sign),
        .m_ena            (oled_ena),
        .m_addr           (oled_addr),
        .m_rw             (oled_rw),
        .m_data_wr        (oled_data_wr),
        .m_stop           (oled_stop),
        .m_busy           (oled_busy),
        .m_data_rd        (oled_data_rd),
        .m_ack_err        (oled_ack_err)
    );

    // ─── Shared I2C bus (ADXL345 priority) ────────────────────────
    i2c_arbiter u_i2c_arb (
        .clk        (clk),
        .rst_n      (rst_n),
        .p0_ena     (adxl_ena),
        .p0_addr    (adxl_addr),
        .p0_rw      (adxl_rw),
        .p0_data_wr (adxl_data_wr),
        .p0_stop    (adxl_stop),
        .p0_busy    (adxl_busy),
        .p0_data_rd (adxl_data_rd),
        .p0_ack_err (adxl_ack_err),
        .p1_ena     (oled_ena),
        .p1_addr    (oled_addr),
        .p1_rw      (oled_rw),
        .p1_data_wr (oled_data_wr),
        .p1_stop    (oled_stop),
        .p1_busy    (oled_busy),
        .p1_data_rd (oled_data_rd),
        .p1_ack_err (oled_ack_err),
        .scl        (scl),
        .sda        (sda)
    );

endmodule
