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
    wire       valid;

    // Trở kéo pull-up cho bus I2C (Tri-state)
    pullup(scl);
    pullup(sda);

    // -------------------------------------------------------------
    // 1. Khởi tạo DUT (adxl345_top)
    // -------------------------------------------------------------
    adxl345_top u_dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .scl              (scl),
        .sda              (sda),
        .steer_dir        (steer_dir),
        .danger_threshold (danger_threshold),
        .steer_angle      (steer_angle),
        .steer_sign       (steer_sign),
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

    reg sda_slave_out;
    reg sda_slave_en;
    assign sda = sda_slave_en ? sda_slave_out : 1'bz;

    reg [7:0] rx_byte;
    reg [7:0] curr_reg;
    reg [2:0] bit_cnt;

    initial begin
        sda_slave_en     = 1'b0;
        sda_slave_out    = 1'b0;
        sim_accel_x_low  = 8'h00;
        sim_accel_x_high = 8'h00;
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
                @(negedge scl);
                sda_slave_out = data[bit_cnt];
                sda_slave_en  = 1'b1;
                @(posedge scl);
            end
            // Thả SDA để nhận ACK/NACK từ Master
            @(negedge scl);
            sda_slave_en  = 1'b0;
            @(posedge scl);
        end
    endtask

    // Luồng lắng nghe tín hiệu START và đọc/ghi trên bus I2C
    always begin
        @(negedge sda);
        if (scl == 1'b1) begin
            i2c_recv_byte(rx_byte);

            if (rx_byte[7:1] == 7'h53) begin // Địa chỉ ADXL345: 0x53
                if (rx_byte[0] == 1'b0) begin
                    // Lệnh Ghi: lưu địa chỉ thanh ghi hoặc cấu hình
                    i2c_recv_byte(rx_byte);
                    curr_reg = rx_byte;
                end else begin
                    // Lệnh Đọc: xuất dữ liệu theo địa chỉ thanh ghi hiện tại
                    if (curr_reg == 8'h32) begin
                        i2c_send_byte(sim_accel_x_low);
                        curr_reg = 8'h33; // Tự động tăng sang byte cao
                    end else if (curr_reg == 8'h33) begin
                        i2c_send_byte(sim_accel_x_high);
                    end else begin
                        i2c_send_byte(8'h00);
                    end
                end
            end
        end
    end

    // -------------------------------------------------------------
    // 4. Kịch bản kiểm thử (Test Scenarios)
    // -------------------------------------------------------------
    initial begin
        // Thiết lập dump sóng cho GTKWave
        $dumpfile("simulation/tb_adxl345.vcd");
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
        $display("[TEST 1 - DI THANG] Ax=0    | Dir=%b | Thresh=%0d cm | Angle=%0d deg | Sign=%b", 
                 steer_dir, danger_threshold, steer_angle, steer_sign);

        // CASE 2: Bẻ lái Phải (Ax = +128 LSB ~ 45 độ)
        sim_accel_x_low  = 8'h80;
        sim_accel_x_high = 8'h00;
        @(posedge valid);
        #10;
        $display("[TEST 2 - RE PHAI ] Ax=+128 | Dir=%b | Thresh=%0d cm | Angle=%0d deg | Sign=%b", 
                 steer_dir, danger_threshold, steer_angle, steer_sign);

        // CASE 3: Bẻ lái Trái (Ax = -128 LSB = 0xFF80)
        sim_accel_x_low  = 8'h80;
        sim_accel_x_high = 8'hFF;
        @(posedge valid);
        #10;
        $display("[TEST 3 - RE TRAI ] Ax=-128 | Dir=%b | Thresh=%0d cm | Angle=%0d deg | Sign=%b", 
                 steer_dir, danger_threshold, steer_angle, steer_sign);

        // CASE 4: Lắc nhẹ trục lái quanh vị trí thẳng (Ax = +30 LSB trong vùng deadzone)
        sim_accel_x_low  = 8'd30;
        sim_accel_x_high = 8'h00;
        @(posedge valid);
        #10;
        $display("[TEST 4 - LAC NHE ] Ax=+30  | Dir=%b | Thresh=%0d cm | Angle=%0d deg | Sign=%b", 
                 steer_dir, danger_threshold, steer_angle, steer_sign);

        #1000;
        $display("==========================================================");
        $display("                MO PHONG HOAN TAT THANH CONG              ");
        $display("==========================================================");
        $finish;
    end

endmodule
