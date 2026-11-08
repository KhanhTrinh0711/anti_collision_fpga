module adxl345_top #(
    parameter DELAY_50MS = 27'd1_350_000
) (
    input  wire        clk,
    input  wire        rst_n,
    // Request port to the shared I2C bus (i2c_arbiter)
    output reg         i2c_ena,
    output reg  [6:0]  i2c_addr,
    output reg         i2c_rw,
    output reg  [7:0]  i2c_data_wr,
    output reg         i2c_stop,
    input  wire        i2c_busy,
    input  wire [7:0]  i2c_data_rd,
    input  wire        i2c_ack_err,
    output reg  [1:0]  steer_dir,
    output reg  [6:0]  danger_threshold,
    output reg  [7:0]  steer_angle,   // góc 0-90°
    output reg         steer_sign,    // 0=phải, 1=trái
    output reg  [7:0]  y_angle,       // góc trục Y 0-90°
    output reg         y_sign,        // 1=âm
    output reg         valid,
    output wire [15:0] accel_raw
);
    localparam ADXL_ADDR  = 7'h53;
    localparam REG_POWER  = 8'h2D;
    localparam REG_DATAX0 = 8'h32;
    localparam MEASURE_EN = 8'h08;

    localparam signed [15:0] THRESH_POS =  16'sd65;
    localparam signed [15:0] THRESH_NEG = -16'sd65;

    localparam S_INIT_REG    = 4'd0;
    localparam S_INIT_REG_W  = 4'd1;
    localparam S_INIT_DATA   = 4'd2;
    localparam S_INIT_DATA_W = 4'd3;
    localparam S_SET_REG     = 4'd4;
    localparam S_SET_WAIT    = 4'd5;
    localparam S_READ_LOW    = 4'd6;
    localparam S_READ_LOW_W  = 4'd7;
    localparam S_READ_HIGH   = 4'd8;
    localparam S_READ_HIGH_W = 4'd9;
    localparam S_EVALUATE    = 4'd10;
    localparam S_DELAY       = 4'd11;
    localparam S_READ_Y0     = 4'd12;
    localparam S_READ_Y0_W   = 4'd13;
    localparam S_READ_Y1     = 4'd14;
    localparam S_READ_Y1_W   = 4'd15;

    reg [3:0] state;

    reg [26:0] delay_cnt;
    wire delay_done = (delay_cnt == DELAY_50MS - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            delay_cnt <= 27'd0;
        else if (state == S_DELAY) begin
            if (delay_done) delay_cnt <= 27'd0;
            else            delay_cnt <= delay_cnt + 1'b1;
        end else
            delay_cnt <= 27'd0;
    end

    reg [7:0]          data_x_low;
    reg [7:0]          data_x_high;
    wire signed [15:0] accel_x = {data_x_high, data_x_low};
    reg [7:0]          data_y_low;
    reg [7:0]          data_y_high;
    wire signed [15:0] accel_y = {data_y_high, data_y_low};

    // Tính góc: angle = |accel| * 90 / 256
    // accel tối đa 256 LSB = 1g = 90°
    function [7:0] accel_to_angle;
        input [15:0] accel;
        reg [15:0] accel_abs;
        reg [23:0] angle_product;
        begin
            accel_abs = accel[15] ? (~accel + 1'b1) : accel;
            angle_product = {8'd0, accel_abs} * 24'd90;
            accel_to_angle = (accel_abs >= 16'd256) ?
                             8'd90 : angle_product[15:8];
        end
    endfunction

    wire [7:0] angle_raw   = accel_to_angle(accel_x);
    wire [7:0] angle_y_raw = accel_to_angle(accel_y);

    reg prev_busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_INIT_REG;
            i2c_ena          <= 1'b0;
            i2c_addr         <= ADXL_ADDR;
            i2c_rw           <= 1'b0;
            i2c_data_wr      <= 8'd0;
            i2c_stop         <= 1'b1;
            data_x_low       <= 8'd0;
            data_x_high      <= 8'd0;
            data_y_low       <= 8'd0;
            data_y_high      <= 8'd0;
            y_angle          <= 8'd0;
            y_sign           <= 1'b0;
            steer_dir        <= 2'b00;
            danger_threshold <= 7'd20;
            steer_angle      <= 8'd0;
            steer_sign       <= 1'b0;
            valid            <= 1'b0;
            prev_busy        <= 1'b0;
        end else begin
            prev_busy <= i2c_busy;
            valid     <= 1'b0;

            case (state)
                S_INIT_REG: begin
                    i2c_addr    <= ADXL_ADDR;
                    i2c_rw      <= 1'b0;
                    i2c_data_wr <= REG_POWER;
                    i2c_stop    <= 1'b0;
                    i2c_ena     <= 1'b1;
                    state       <= S_INIT_REG_W;
                end

                S_INIT_REG_W: begin
                    i2c_ena <= 1'b0;
                    if (prev_busy && !i2c_busy) begin
                        if (i2c_ack_err)
                            state <= S_INIT_REG;
                        else
                            state <= S_INIT_DATA;
                    end
                end

                S_INIT_DATA: begin
                    i2c_addr    <= ADXL_ADDR;
                    i2c_rw      <= 1'b0;
                    i2c_data_wr <= MEASURE_EN;
                    i2c_stop    <= 1'b1;
                    i2c_ena     <= 1'b1;
                    state       <= S_INIT_DATA_W;
                end

                S_INIT_DATA_W: begin
                    i2c_ena <= 1'b0;
                    if (prev_busy && !i2c_busy) begin
                        if (i2c_ack_err)
                            state <= S_INIT_REG;
                        else
                            state <= S_SET_REG;
                    end
                end

                S_SET_REG: begin
                    i2c_addr    <= ADXL_ADDR;
                    i2c_rw      <= 1'b0;
                    i2c_data_wr <= REG_DATAX0;
                    i2c_stop    <= 1'b0;
                    i2c_ena     <= 1'b1;
                    state       <= S_SET_WAIT;
                end

                S_SET_WAIT: begin
                    i2c_ena <= 1'b0;
                    if (prev_busy && !i2c_busy) begin
                        if (i2c_ack_err)
                            state <= S_INIT_REG;
                        else
                            state <= S_READ_LOW;
                    end
                end

                S_READ_LOW: begin
                    i2c_addr <= ADXL_ADDR;
                    i2c_rw   <= 1'b1;
                    i2c_stop <= 1'b0;
                    i2c_ena  <= 1'b1;
                    state    <= S_READ_LOW_W;
                end

                S_READ_LOW_W: begin
                    i2c_ena <= 1'b0;
                    if (prev_busy && !i2c_busy) begin
                        if (i2c_ack_err)
                            state <= S_INIT_REG;
                        else begin
                            data_x_low <= i2c_data_rd;
                            state      <= S_READ_HIGH;
                        end
                    end
                end

                S_READ_HIGH: begin
                    i2c_addr <= ADXL_ADDR;
                    i2c_rw   <= 1'b1;
                    i2c_stop <= 1'b0;  // burst continues to DATAY0/DATAY1
                    i2c_ena  <= 1'b1;
                    state    <= S_READ_HIGH_W;
                end

                S_READ_HIGH_W: begin
                    i2c_ena <= 1'b0;
                    if (prev_busy && !i2c_busy) begin
                        if (i2c_ack_err)
                            state <= S_INIT_REG;
                        else begin
                            data_x_high <= i2c_data_rd;
                            state       <= S_READ_Y0;
                        end
                    end
                end

                S_READ_Y0: begin
                    i2c_addr <= ADXL_ADDR;
                    i2c_rw   <= 1'b1;
                    i2c_stop <= 1'b0;
                    i2c_ena  <= 1'b1;
                    state    <= S_READ_Y0_W;
                end

                S_READ_Y0_W: begin
                    i2c_ena <= 1'b0;
                    if (prev_busy && !i2c_busy) begin
                        if (i2c_ack_err)
                            state <= S_INIT_REG;
                        else begin
                            data_y_low <= i2c_data_rd;
                            state      <= S_READ_Y1;
                        end
                    end
                end

                S_READ_Y1: begin
                    i2c_addr <= ADXL_ADDR;
                    i2c_rw   <= 1'b1;
                    i2c_stop <= 1'b1;
                    i2c_ena  <= 1'b1;
                    state    <= S_READ_Y1_W;
                end

                S_READ_Y1_W: begin
                    i2c_ena <= 1'b0;
                    if (prev_busy && !i2c_busy) begin
                        if (i2c_ack_err)
                            state <= S_INIT_REG;
                        else begin
                            data_y_high <= i2c_data_rd;
                            state       <= S_EVALUATE;
                        end
                    end
                end

                S_EVALUATE: begin
                    // Cập nhật góc và dấu
                    steer_angle <= angle_raw;
                    steer_sign  <= accel_x[15]; // bit dấu: 1=âm=trái
                    y_angle     <= angle_y_raw;
                    y_sign      <= accel_y[15];

                    if (accel_x > THRESH_POS) begin
                        steer_dir        <= 2'b10;
                        danger_threshold <= 7'd40;
                    end else if (accel_x < THRESH_NEG) begin
                        steer_dir        <= 2'b01;
                        danger_threshold <= 7'd40;
                    end else begin
                        steer_dir        <= 2'b00;
                        danger_threshold <= 7'd20;
                    end
                    valid <= 1'b1;
                    state <= S_DELAY;
                end

                S_DELAY: begin
                    if (delay_done)
                        state <= S_SET_REG;
                end

                default: state <= S_INIT_REG;
            endcase
        end
    end

    assign accel_raw = {data_x_high, data_x_low};

endmodule
