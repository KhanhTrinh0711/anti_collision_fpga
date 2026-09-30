module top_module (
    input  wire clk,
    input  wire rst_n,
    input  wire echo_pin,
    output wire trig_pin,
    inout  wire scl,        
    inout  wire sda,
    inout  wire oled_scl,   
    inout  wire oled_sda,
    output wire ext_led,
    output wire steer_led,
    output wire buzzer,
    output wire uart_tx_debug
);
    // ─── HC-SR04 ──────────────────────────────────────────────────
    wire [15:0] distance_cm;
    wire        dist_valid;
    wire        dist_timeout;

    hcsr04_controller u_hcsr04 (
        .clk_27m    (clk),
        .rst_n      (rst_n),
        .echo_pin   (echo_pin),
        .trig_pin   (trig_pin),
        .distance_cm(distance_cm),
        .valid      (dist_valid),
        .timeout    (dist_timeout)
    );

    // ─── ADXL345 ──────────────────────────────────────────────────
    wire [1:0] steer_dir;
    wire [6:0] danger_threshold;
    wire [7:0] steer_angle;
    wire       steer_sign;
    wire       steer_valid;

    adxl345_top u_adxl345 (
        .clk              (clk),
        .rst_n            (rst_n),
        .scl              (scl),
        .sda              (sda),
        .steer_dir        (steer_dir),
        .danger_threshold (danger_threshold),
        .steer_angle      (steer_angle),
        .steer_sign       (steer_sign),
        .valid            (steer_valid),
        .accel_raw        (accel_raw)
    );

    //─── UART ─────────────────────────────────────────────────────
    debug_uart_tx u_debug (
        .clk              (clk),
        .rst_n            (rst_n),
        .valid            (steer_valid),
        .steer_dir        (steer_dir),
        .steer_angle      (steer_angle),
        .steer_sign       (steer_sign),
        .danger_threshold (danger_threshold),
        .accel_raw        (accel_raw),
        .uart_tx          (uart_tx_debug)
    );

    // ─── Zone từ HC-SR04 ──────────────────────────────────────────
    reg [1:0] zone;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            zone <= 2'd0;
        else if (dist_valid) begin
            if (dist_timeout || distance_cm > 16'd50)
                zone <= 2'd0;
            else if (distance_cm > 16'd20)
                zone <= 2'd1;
            else if (distance_cm > 16'd10)
                zone <= 2'd2;
            else
                zone <= 2'd3;
        end
    end

    // ─── Detect zone thay đổi ────────────────────────────────────
    reg [1:0] zone_prev;
    wire      zone_changed = (zone != zone_prev);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) 
            zone_prev <= 2'd0;
        else        
            zone_prev <= zone;
    end

    // ─── OLED ─────────────────────────────────────────────────────
    wire rear_left_alert  = (zone >= 2'd1);
    wire rear_right_alert = 1'b0;
    wire side_left_alert  = 1'b0;
    wire side_right_alert = 1'b0;

    ssd1306_oled u_oled (
        .clk              (clk),
        .rst_n            (rst_n),
        .rear_left_alert  (rear_left_alert),
        .rear_right_alert (rear_right_alert),
        .side_left_alert  (side_left_alert),
        .side_right_alert (side_right_alert),
        .steer_angle      (steer_angle),
        .steer_sign       (steer_sign),
        .oled_scl         (oled_scl),
        .oled_sda         (oled_sda)
    );

    // ─── Steer zone từ ADXL345 ───────────────────────────────────
    localparam LEVEL_SAFE    = 2'd0;
    localparam LEVEL_WARNING = 2'd1;
    localparam LEVEL_DANGER  = 2'd2;

    reg [1:0] steer_zone;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            steer_zone <= LEVEL_SAFE;
        else if (dist_valid && !dist_timeout) begin
            if (distance_cm < {9'd0, danger_threshold})
                steer_zone <= LEVEL_DANGER;
            else if (distance_cm < ({9'd0, danger_threshold} + 16'd30))
                steer_zone <= LEVEL_WARNING;
            else
                steer_zone <= LEVEL_SAFE;
        end else if (dist_timeout)
            steer_zone <= LEVEL_SAFE;
    end

    // ─── Blink counters ───────────────────────────────────────────
    localparam BLINK_SLOW = 25'd13_500_000;
    localparam BLINK_FAST = 25'd2_700_000;

    reg [24:0] slow_cnt, fast_cnt;
    reg        blink_slow_tick, blink_fast_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            slow_cnt <= 25'd0; 
            blink_slow_tick <= 1'b0;
        end else begin
            blink_slow_tick <= 1'b0;
            if (slow_cnt >= BLINK_SLOW - 1) begin
                slow_cnt <= 25'd0; 
                blink_slow_tick <= 1'b1;
            end else 
                slow_cnt <= slow_cnt + 1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fast_cnt <= 25'd0; 
            blink_fast_tick <= 1'b0;
        end else begin
            blink_fast_tick <= 1'b0;
            if (fast_cnt >= BLINK_FAST - 1) begin
                fast_cnt <= 25'd0; 
                blink_fast_tick <= 1'b1;
            end else 
                fast_cnt <= fast_cnt + 1'b1;
        end
    end

    // ─── LED HC-SR04 ──────────────────────────────────────────────
    reg led_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            led_state <= 1'b0;
        else if (zone_changed)
            led_state <= 1'b0;
        else case (zone)
            2'd0: led_state <= 1'b0;
            2'd1: if (blink_slow_tick) led_state <= ~led_state;
            2'd2: if (blink_fast_tick) led_state <= ~led_state;
            2'd3: led_state <= 1'b1;
            default: led_state <= 1'b0;
        endcase
    end

    // ─── LED ADXL345 ──────────────────────────────────────────────
    reg steer_led_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) 
            steer_led_state <= 1'b0;
        else case (steer_zone)
            LEVEL_SAFE:    steer_led_state <= 1'b0;
            LEVEL_WARNING: if (blink_slow_tick) steer_led_state <= ~steer_led_state;
            LEVEL_DANGER:  if (blink_fast_tick) steer_led_state <= ~steer_led_state;
            default:       steer_led_state <= 1'b0;
        endcase
    end

    // ─── Buzzer PWM 2kHz ──────────────────────────────────────────
    localparam BUZ_HALF = 14'd6750;
    reg [13:0] buz_pwm_cnt;
    reg        buz_pwm;
    reg        buz_enable;

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

    // ─── Buzzer enable ────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            buz_enable <= 1'b0;
        else if (zone_changed)
            buz_enable <= 1'b0;
        else case (zone)
            2'd0: buz_enable <= 1'b0;
            2'd1: if (blink_slow_tick) buz_enable <= ~buz_enable;
            2'd2: if (blink_fast_tick) buz_enable <= ~buz_enable;
            2'd3: buz_enable <= 1'b1;
            default: buz_enable <= 1'b0;
        endcase
    end

    assign ext_led   = led_state;
    assign steer_led = steer_led_state;
    assign buzzer    = buz_enable ? buz_pwm : 1'b0;

endmodule