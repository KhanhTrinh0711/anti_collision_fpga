module hcsr04_measure #(
    parameter integer CLK_FREQ_HZ = 27_000_000
) (
    input wire clk,
    input wire rst_n,
    input wire echo,
    output reg trig,
    output reg sample_done,
    output reg sample_valid,
    output reg [1:0] error_code,
    output reg [31:0] echo_width_cycles,
    output reg [15:0] echo_width_us,
    output reg [8:0] distance_cm
);
    // Clock must be an integer multiple of 1 MHz (27 MHz on this board).
    localparam integer CYCLES_PER_US = CLK_FREQ_HZ / 1_000_000;
    localparam integer CYCLES_PER_CM = CYCLES_PER_US * 58;
    // Datasheet minimum is 10 us; exactly 10 us leaves no margin on clone modules.
    localparam integer TRIG_CYCLES = CYCLES_PER_US * 20;
    localparam integer PERIOD_CYCLES = CYCLES_PER_US * 60_000;
    localparam integer TIMEOUT_CYCLES = CYCLES_PER_US * 30_000;
    localparam integer MIN_ECHO_CYCLES = CYCLES_PER_CM * 2;
    localparam integer MAX_ECHO_CYCLES = CYCLES_PER_CM * 400;
    localparam [1:0] IDLE = 0, TRIGGER = 1, WAIT_ECHO = 2, MEASURE = 3;
    // error_code: 0=OK, 1=no rising ECHO, 2=ECHO stuck high,
    //             3=pulse longer than 400 cm (pulses < 2 cm are ignored as noise).
    reg [1:0] state;
    reg [31:0] timer;
    reg [31:0] pulse_cycles;
    reg echo_meta, echo_sync;
    // Count complete microseconds and centimeters during ECHO. This avoids
    // a long combinational divider on the 27 MHz timing path.
    localparam integer US_REMAINDER_BITS = (CYCLES_PER_US < 2) ? 1 : $clog2(CYCLES_PER_US);
    localparam integer CM_REMAINDER_BITS = $clog2(CYCLES_PER_CM);
    reg [US_REMAINDER_BITS-1:0] us_remainder;
    reg [CM_REMAINDER_BITS-1:0] cm_remainder;
    reg [15:0] pulse_us;
    reg [9:0] whole_cm;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            echo_meta <= 0;
            echo_sync <= 0;
        end else begin
            echo_meta <= echo;
            echo_sync <= echo_meta;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            timer <= 0;
            pulse_cycles <= 0;
            us_remainder <= 0;
            cm_remainder <= 0;
            pulse_us <= 0;
            whole_cm <= 0;
            trig <= 0;
            sample_done <= 0;
            sample_valid <= 0;
            error_code <= 0;
            echo_width_cycles <= 0;
            echo_width_us <= 0;
            distance_cm <= 0;
        end else begin
            sample_done <= 0;
            case (state)
                IDLE: begin
                    trig <= 0;
                    if (timer >= PERIOD_CYCLES-1) begin
                        timer <= 0;
                        if (echo_sync) begin
                            // Do not trigger a sensor whose ECHO is already high.
                            sample_done <= 1;
                            sample_valid <= 0;
                            error_code <= 2;
                            echo_width_cycles <= 0;
                            echo_width_us <= 0;
                            distance_cm <= 0;
                        end else begin
                            trig <= 1;
                            state <= TRIGGER;
                        end
                    end else timer <= timer + 1'b1;
                end
                TRIGGER: begin
                    if (timer >= TRIG_CYCLES-1) begin
                        trig <= 0;
                        timer <= 0;
                        state <= WAIT_ECHO;
                    end else timer <= timer + 1'b1;
                end
                WAIT_ECHO: begin
                    if (echo_sync) begin
                        pulse_cycles <= 1; // Count the first sampled high cycle.
                        us_remainder <= (CYCLES_PER_US == 1) ? 0 : 1;
                        pulse_us <= (CYCLES_PER_US == 1) ? 1 : 0;
                        cm_remainder <= 1;
                        whole_cm <= 0;
                        // timer keeps running: a rejected glitch must not extend the 30 ms wait.
                        state <= MEASURE;
                    end else if (timer >= TIMEOUT_CYCLES-1) begin
                        timer <= 0;
                        state <= IDLE;
                        sample_done <= 1;
                        sample_valid <= 0;
                        error_code <= 1;
                        echo_width_cycles <= 0;
                        echo_width_us <= 0;
                        distance_cm <= 0;
                    end else timer <= timer + 1'b1;
                end
                MEASURE: begin
                    if (!echo_sync && pulse_cycles < MIN_ECHO_CYCLES) begin
                        // Pulse shorter than 2 cm: noise/trigger crosstalk, not the echo.
                        // Keep waiting for the real echo instead of ending the sample.
                        state <= WAIT_ECHO;
                    end else if (!echo_sync) begin
                        timer <= 0;
                        state <= IDLE;
                        sample_done <= 1;
                        echo_width_cycles <= pulse_cycles;
                        echo_width_us <= pulse_us;
                        if (pulse_cycles >= MIN_ECHO_CYCLES && pulse_cycles <= MAX_ECHO_CYCLES) begin
                            sample_valid <= 1;
                            error_code <= 0;
                            // Round to the nearest cm, using the full cycle count.
                            distance_cm <= whole_cm + (cm_remainder >= CYCLES_PER_CM/2);
                        end else begin
                            sample_valid <= 0;
                            error_code <= 3;
                            distance_cm <= 0;
                        end
                    end else if (pulse_cycles >= TIMEOUT_CYCLES) begin
                        timer <= 0;
                        state <= IDLE;
                        sample_done <= 1;
                        sample_valid <= 0;
                        error_code <= 2;
                        echo_width_cycles <= pulse_cycles;
                        echo_width_us <= pulse_us;
                        distance_cm <= 0;
                    end else begin
                        pulse_cycles <= pulse_cycles + 1'b1;
                        if (us_remainder >= CYCLES_PER_US-1) begin
                            us_remainder <= 0;
                            pulse_us <= pulse_us + 1'b1;
                        end else us_remainder <= us_remainder + 1'b1;
                        if (cm_remainder >= CYCLES_PER_CM-1) begin
                            cm_remainder <= 0;
                            whole_cm <= whole_cm + 1'b1;
                        end else cm_remainder <= cm_remainder + 1'b1;
                    end
                end
                default: begin
                    state <= IDLE;
                    timer <= 0;
                    trig <= 0;
                    sample_valid <= 0;
                end
            endcase
        end
    end
endmodule
