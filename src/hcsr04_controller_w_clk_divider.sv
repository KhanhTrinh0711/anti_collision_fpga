module hcsr04_controller (
    input  wire        clk_27m,
    input  wire        rst_n,
    input  wire        echo_pin,
    output reg         trig_pin,
    output reg  [15:0] distance_cm,
    output reg         valid,
    output reg         timeout
);
    localparam TIMEOUT_US  = 16'd38000;
    localparam TRIG_US     = 4'd10;
    localparam MEAS_PERIOD = 20'd540000;

    localparam IDLE      = 3'd0;
    localparam TRIGGER   = 3'd1;
    localparam WAIT_ECHO = 3'd2;
    localparam COUNTING  = 3'd3;
    localparam CALCULATE = 3'd4;
    localparam CALC_WAIT = 3'd5;
    localparam UPDATE    = 3'd6;
    localparam OUTPUT    = 3'd7;

    reg [2:0] state;

    reg [4:0] cnt_1us;
    wire tick_1us = (cnt_1us == 5'd26);

    always @(posedge clk_27m or negedge rst_n) begin
        if (!rst_n)        cnt_1us <= 5'd0;
        else if (tick_1us) cnt_1us <= 5'd0;
        else               cnt_1us <= cnt_1us + 1'b1;
    end

    reg [19:0] period_cnt;
    wire period_done = (period_cnt == MEAS_PERIOD - 1);

    always @(posedge clk_27m or negedge rst_n) begin
        if (!rst_n)
            period_cnt <= 20'd0;
        else if (state == IDLE) begin
            if (period_done) period_cnt <= 20'd0;
            else             period_cnt <= period_cnt + 1'b1;
        end else
            period_cnt <= 20'd0;
    end

    reg [3:0] trig_cnt;
    wire trig_done = (trig_cnt == TRIG_US - 1);

    always @(posedge clk_27m or negedge rst_n) begin
        if (!rst_n)
            trig_cnt <= 4'd0;
        else if (state == TRIGGER) begin
            if (tick_1us) trig_cnt <= trig_cnt + 1'b1;
        end else
            trig_cnt <= 4'd0;
    end

    reg [15:0] echo_time_us;

    always @(posedge clk_27m or negedge rst_n) begin
        if (!rst_n)
            echo_time_us <= 16'd0;
        else if (state == WAIT_ECHO)
            echo_time_us <= 16'd0;
        else if (state == COUNTING && tick_1us) begin
            if (echo_time_us < TIMEOUT_US)
                echo_time_us <= echo_time_us + 1'b1;
        end
    end

    // ─── Latch timeout flag khi rời COUNTING ─────────────────────
    reg timeout_flag;

    always @(posedge clk_27m or negedge rst_n) begin
        if (!rst_n)
            timeout_flag <= 1'b0;
        else if (state == COUNTING) begin
            if (echo_time_us >= TIMEOUT_US)
                timeout_flag <= 1'b1;
            else
                timeout_flag <= 1'b0;
        end
    end

    reg [15:0] raw_cm;

    always @(posedge clk_27m or negedge rst_n) begin
        if (!rst_n)
            raw_cm <= 16'd0;
        else if (state == CALCULATE)
            raw_cm <= (({16'd0, echo_time_us} * 32'd9) >> 9);
    end

    // ─── Moving average — khởi tạo bằng giá trị đầu tiên ────────
    reg [15:0] sample [0:3];
    reg        sample_valid;  // đã có ít nhất 1 mẫu hợp lệ

    always @(posedge clk_27m or negedge rst_n) begin
        if (!rst_n) begin
            sample[0]    <= 16'd0;
            sample[1]    <= 16'd0;
            sample[2]    <= 16'd0;
            sample[3]    <= 16'd0;
            sample_valid <= 1'b0;
        end else if (state == UPDATE && !timeout_flag) begin
            if (!sample_valid) begin
                // Lần đầu: điền tất cả 4 mẫu bằng raw_cm
                sample[0]    <= raw_cm;
                sample[1]    <= raw_cm;
                sample[2]    <= raw_cm;
                sample[3]    <= raw_cm;
                sample_valid <= 1'b1;
            end else begin
                sample[3] <= sample[2];
                sample[2] <= sample[1];
                sample[1] <= sample[0];
                sample[0] <= raw_cm;
            end
        end
    end

    wire [17:0] filtered_cm = (sample[0] + sample[1] +
                                sample[2] + sample[3]) >> 2;

    always @(posedge clk_27m or negedge rst_n) begin
        if (!rst_n) begin
            state       <= IDLE;
            trig_pin    <= 1'b0;
            distance_cm <= 16'd0;
            valid       <= 1'b0;
            timeout     <= 1'b0;
        end else begin
            valid <= 1'b0;

            case (state)
                IDLE: begin
                    trig_pin <= 1'b0;
                    if (period_done) state <= TRIGGER;
                end

                TRIGGER: begin
                    trig_pin <= 1'b1;
                    if (tick_1us && trig_done) begin
                        trig_pin <= 1'b0;
                        state    <= WAIT_ECHO;
                    end
                end

                WAIT_ECHO: begin
                    trig_pin <= 1'b0;
                    if (echo_pin) state <= COUNTING;
                end

                COUNTING: begin
                    if (!echo_pin || echo_time_us >= TIMEOUT_US)
                        state <= CALCULATE;
                end

                CALCULATE: begin
                    state <= CALC_WAIT;
                end

                CALC_WAIT: begin
                    state <= UPDATE;
                end

                UPDATE: begin
                    state <= OUTPUT;
                end

                OUTPUT: begin
                    if (timeout_flag) begin
                        timeout     <= 1'b1;
                        distance_cm <= 16'd0;
                    end else begin
                        timeout     <= 1'b0;
                        distance_cm <= filtered_cm[15:0];
                    end
                    valid <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule