`timescale 1ns/1ps

module tb_i2c_master;

    localparam CLK_PERIOD = 37; // ~27MHz

    reg        clk;
    reg        rst_n;
    reg        ena;
    reg [6:0]  addr;
    reg        rw;
    reg [7:0]  data_wr;
    reg        stop;

    wire       busy;
    wire [7:0] data_rd;
    wire       ack_error;
    wire       scl;
    wire       sda;

    // Trở kéo pull-up cho bus I2C chuẩn Open-Drain
    pullup (scl);
    pullup (sda);

    // Slave model: điều khiển kéo SDA xuống 0 khi ACK
    reg sda_slave_en;
    assign sda = sda_slave_en ? 1'b0 : 1'bz;

    // Khởi tạo DUT
    i2c_master #(
        .CLK_FREQ(27_000_000),
        .I2C_FREQ(100_000)
    ) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .ena      (ena),
        .addr     (addr),
        .rw       (rw),
        .data_wr  (data_wr),
        .stop     (stop),
        .busy     (busy),
        .data_rd  (data_rd),
        .ack_error(ack_error),
        .scl      (scl),
        .sda      (sda)
    );

    // Tạo xung Clock 27MHz
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    task wait_cycles;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1)
                @(posedge clk);
        end
    endtask

    // Theo dõi trạng thái FSM nội bộ của DUT
    wire [3:0] dut_state = dut.state;

    localparam ST_SLV_ACK = 4'd3;
    localparam ST_WR_ACK  = 4'd5;

    reg do_ack;

    // Giữ SDA = 0 trong toàn bộ thời gian của ST_SLV_ACK và ST_WR_ACK
    // để DUT lấy mẫu chính xác ở phase 2
    always @(*) begin
        if (do_ack && (dut_state == ST_SLV_ACK || dut_state == ST_WR_ACK))
            sda_slave_en = 1'b1;
        else
            sda_slave_en = 1'b0;
    end

    // Task phát lệnh ghi 1 byte I2C
    task i2c_write;
        input [6:0] t_addr;
        input [7:0] t_data;
        input       t_stop;
        input       t_ack;
        begin
            do_ack  = t_ack;
            addr    = t_addr;
            rw      = 1'b0;
            data_wr = t_data;
            stop    = t_stop;
            ena     = 1'b1;
            @(posedge clk); #1;
            ena     = 1'b0;

            // Chờ Master bắt đầu và kết thúc truyền
            wait(busy == 1'b1);
            wait(busy == 1'b0);
            #1;

            do_ack  = 1'b0;
        end
    endtask

    // Task kiểm tra kết quả
    task check_result;
        input integer tid;
        input         exp_err;
        begin
            $write("TEST %0d: ack_error=%b | ", tid, ack_error);
            if (ack_error == exp_err)
                $display("PASS");
            else
                $display("FAIL (expected %b)", exp_err);
        end
    endtask

    always @(negedge busy) begin
        if (rst_n)
            $display("  [%0t ns] Done ack_error=%b", $time, ack_error);
    end

    // Kịch bản kiểm thử
    initial begin
        rst_n        = 0;
        ena          = 0;
        addr         = 7'h00;
        rw           = 0;
        data_wr      = 8'h00;
        stop         = 1;
        sda_slave_en = 0;
        do_ack       = 0;

        wait_cycles(10);
        rst_n = 1;
        wait_cycles(5);

        $display("=== TEST 1: Write 0x00 addr=0x3C ACK ===");
        i2c_write(7'h3C, 8'h00, 1'b0, 1'b1);
        check_result(1, 1'b0);
        wait_cycles(10);

        $display("=== TEST 2: Write 0xAE ACK ===");
        i2c_write(7'h3C, 8'hAE, 1'b1, 1'b1);
        check_result(2, 1'b0);
        wait_cycles(10);

        $display("=== TEST 3: Write 0x40 ACK ===");
        i2c_write(7'h3C, 8'h40, 1'b0, 1'b1);
        check_result(3, 1'b0);
        wait_cycles(10);

        $display("=== TEST 4: Write 0xFF ACK ===");
        i2c_write(7'h3C, 8'hFF, 1'b1, 1'b1);
        check_result(4, 1'b0);
        wait_cycles(10);

        $display("=== TEST 5: NACK expected ===");
        i2c_write(7'h3C, 8'hAF, 1'b1, 1'b0);
        check_result(5, 1'b1);
        wait_cycles(10);

        $display("=== ALL DONE ===");
        $finish;
    end

    // Timeout dự phòng
    initial begin
        #2_000_000_000; // 2 giây mô phỏng
        $display("TIMEOUT");
        $finish;
    end

    // Xuất file VCD
    initial begin
        $dumpfile("tb_i2c_master.vcd");
        $dumpvars(0, tb_i2c_master);
    end

endmodule
