`timescale 1ns / 1ps

module tb_adxl345_top;

    // Tín hiệu điều khiển DUT
    reg        clk;
    reg        rst_n;
    wire       scl;
    wire       sda;

    // Tín hiệu ngõ ra quan sát
    wire [1:0] steer_dir;
    wire [6:0] danger_threshold;
    wire [7:0] steer_angle;
    wire       steer_sign;
    wire [7:0] y_angle;
    wire       y_sign;
    wire       valid;

    // Trở kéo pull-up cho bus I2C (Tri-state)
    pullup(scl);
    pullup(sda);

    // -------------------------------------------------------------
    // 1. Khởi tạo DUT (adxl345_top)
    // -------------------------------------------------------------
    wire       i2c_ena, i2c_rw, i2c_stop, i2c_busy, i2c_ack_err, i2c_bus_active;
    wire [6:0] i2c_addr;
    wire [7:0] i2c_data_wr, i2c_data_rd;

    i2c_master u_i2c (
        .clk(clk), .rst_n(rst_n), .ena(i2c_ena), .addr(i2c_addr), .rw(i2c_rw),
        .data_wr(i2c_data_wr), .stop(i2c_stop), .busy(i2c_busy),
        .data_rd(i2c_data_rd), .ack_error(i2c_ack_err),
        .bus_active(i2c_bus_active), .scl(scl), .sda(sda)
    );

    adxl345_top u_dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .i2c_ena          (i2c_ena),
        .i2c_addr         (i2c_addr),
        .i2c_rw           (i2c_rw),
        .i2c_data_wr      (i2c_data_wr),
        .i2c_stop         (i2c_stop),
        .i2c_busy         (i2c_busy),
        .i2c_data_rd      (i2c_data_rd),
        .i2c_ack_err      (i2c_ack_err),
        .steer_dir        (steer_dir),
        .danger_threshold (danger_threshold),
        .steer_angle      (steer_angle),
        .steer_sign       (steer_sign),
        .y_angle          (y_angle),
        .y_sign           (y_sign),
        .valid            (valid)
    );

    // Rút ngắn thời gian trễ 50ms xuống 50 chu kỳ để mô phỏng nhanh
    defparam u_dut.DELAY_50MS = 27'd50;

    // -------------------------------------------------------------
    // 2. Tạo xung Clock 27MHz (Chu kỳ xấp xỉ 37.037 ns)
    // -------------------------------------------------------------
    always #18.518 clk = ~clk;

    // -------------------------------------------------------------
    // 3. I2C Slave Model (Phản hồi mô phỏng cảm biến ADXL345)
    // -------------------------------------------------------------
    reg [7:0] sim_accel_x_low;
    reg [7:0] sim_accel_x_high;
    reg [7:0] sim_accel_y_low;
    reg [7:0] sim_accel_y_high;

    reg sda_slave_out;
    reg sda_slave_en;
    assign sda = sda_slave_en ? sda_slave_out : 1'bz;

    reg [7:0] rx_byte;
    reg [7:0] curr_reg;
    integer bit_cnt;

    initial begin
        sda_slave_en     = 1'b0;
        sda_slave_out    = 1'b0;
        sim_accel_x_low  = 8'h00;
        sim_accel_x_high = 8'h00;
        sim_accel_y_low  = 8'h00;
        sim_accel_y_high = 8'h00;
    end

    // Task nhận 1 byte từ Master và gửi ACK
    task i2c_recv_byte;
        output [7:0] data;
        begin
            for (bit_cnt = 7; bit_cnt >= 0; bit_cnt = bit_cnt - 1) begin
                @(posedge scl);
                data[bit_cnt] = sda;
            end
            // Tạo xung ACK: kéo SDA xuống mức 0
            @(negedge scl);
            sda_slave_out = 1'b0;
            sda_slave_en  = 1'b1;
            @(posedge scl);
            @(negedge scl);
            sda_slave_en  = 1'b0;
        end
    endtask

    // Task gửi 1 byte từ Slave về Master và chờ ACK/NACK
    task i2c_send_byte;
        input [7:0] data;
        begin
            for (bit_cnt = 7; bit_cnt >= 0; bit_cnt = bit_cnt - 1) begin
                sda_slave_out = 1'b0;
                sda_slave_en = !data[bit_cnt];
                @(posedge scl);
                @(negedge scl);
            end
            // Thả SDA để nhận ACK/NACK từ Master
            sda_slave_en = 1'b0;
            @(posedge scl);
            @(negedge scl);
        end
    endtask

    // Luồng lắng nghe tín hiệu START và đọc/ghi trên bus I2C
    always begin
        @(negedge sda);
        if (scl == 1'b1) begin
            i2c_recv_byte(rx_byte);

            if (rx_byte[7:1] == 7'h53) begin // Địa chỉ ADXL345: 0x53
                if (rx_byte[0] == 1'b0) begin
                    i2c_recv_byte(rx_byte);
                    curr_reg = rx_byte;
                    if (curr_reg == 8'h2D) begin
                        i2c_recv_byte(rx_byte);
                        if (rx_byte !== 8'h08) $fatal(1, "Invalid POWER_CTL");
                    end
                end else begin
                    if (curr_reg !== 8'h32) $fatal(1, "Invalid data register");
                    i2c_send_byte(sim_accel_x_low);
                    i2c_send_byte(sim_accel_x_high);
                    i2c_send_byte(sim_accel_y_low);
                    i2c_send_byte(sim_accel_y_high);
                end
            end
        end
    end

    // -------------------------------------------------------------
    // 4. Kịch bản kiểm thử (Test Scenarios)
    // -------------------------------------------------------------
    initial begin
        // Thiết lập dump sóng cho GTKWave
        $dumpfile("tb_adxl345.vcd");
        $dumpvars(0, tb_adxl345_top);

        // Khởi tạo trạng thái ban đầu
        clk   = 1'b0;
        rst_n = 1'b0;
        #200;
        rst_n = 1'b1;

        $display("==========================================================");
        $display("          BAT DAU MO PHONG ADXL345 TOP (IVERILOG)         ");
        $display("==========================================================");

        // CASE 1: Xe đi thẳng (Ax = 0)
        sim_accel_x_low  = 8'h00;
        sim_accel_x_high = 8'h00;
        @(posedge valid);
        #10;
        if (steer_dir !== 0 || danger_threshold !== 20 || steer_angle !== 0 || steer_sign !== 0) $fatal(1, "Scenario 1 failed");
        $display("[TEST 1 - DI THANG] Ax=0    | Dir=%b | Thresh=%0d cm | Angle=%0d deg | Sign=%b", 
                 steer_dir, danger_threshold, steer_angle, steer_sign);

        // CASE 2: Bẻ lái Phải (Ax = +128 LSB ~ 45 độ)
        sim_accel_x_low  = 8'h80;
        sim_accel_x_high = 8'h00;
        @(posedge valid);
        #10;
        if (steer_dir !== 2 || danger_threshold !== 40 || steer_angle !== 45 || steer_sign !== 0) $fatal(1, "Scenario 2 failed");
        $display("[TEST 2 - RE PHAI ] Ax=+128 | Dir=%b | Thresh=%0d cm | Angle=%0d deg | Sign=%b", 
                 steer_dir, danger_threshold, steer_angle, steer_sign);

        // CASE 3: Bẻ lái Trái (Ax = -128 LSB = 0xFF80)
        sim_accel_x_low  = 8'h80;
        sim_accel_x_high = 8'hFF;
        @(posedge valid);
        #10;
        if (steer_dir !== 1 || danger_threshold !== 40 || steer_angle !== 45 || steer_sign !== 1) $fatal(1, "Scenario 3 failed");
        $display("[TEST 3 - RE TRAI ] Ax=-128 | Dir=%b | Thresh=%0d cm | Angle=%0d deg | Sign=%b", 
                 steer_dir, danger_threshold, steer_angle, steer_sign);

        // CASE 4: Lắc nhẹ trục lái quanh vị trí thẳng (Ax = +30 LSB trong vùng deadzone)
        sim_accel_x_low  = 8'd30;
        sim_accel_x_high = 8'h00;
        @(posedge valid);
        #10;
        if (steer_dir !== 0 || danger_threshold !== 20 || steer_angle !== 10 || steer_sign !== 0) $fatal(1, "Scenario 4 failed");
        $display("[TEST 4 - LAC NHE ] Ax=+30  | Dir=%b | Thresh=%0d cm | Angle=%0d deg | Sign=%b", 
                 steer_dir, danger_threshold, steer_angle, steer_sign);

        sim_accel_x_low = 8'h00;
        sim_accel_x_high = 8'h02;
        @(posedge valid); #10;
        if (steer_angle !== 90) $fatal(1, "Positive saturation failed");
        sim_accel_x_high = 8'h80;
        @(posedge valid); #10;
        if (steer_angle !== 90 || steer_sign !== 1)
            $fatal(1, "Negative saturation failed");

        // Trục Y: Ay = -128 LSB (0xFF80) ~ -45 độ, X không đổi
        sim_accel_y_low  = 8'h80;
        sim_accel_y_high = 8'hFF;
        @(posedge valid); #10;
        if (y_angle !== 45 || y_sign !== 1 || steer_angle !== 90)
            $fatal(1, "Y axis angle failed");
        #1000;
        $display("==========================================================");
        $display("                MO PHONG HOAN TAT THANH CONG              ");
        $display("==========================================================");
        $finish;
    end

    initial begin
        #20_000_000;
        $fatal(1, "ADXL345 timeout");
    end
endmodule
