`timescale 1ns/1ps
module tb_oled;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;
    tri1 scl, sda;
    reg slave_low = 0, active = 0;
    assign sda = slave_low ? 1'b0 : 1'bz;
    wire ena, rw, stop, busy, ack_err, bus_active;
    wire [6:0] addr;
    wire [7:0] data_wr, data_rd;
    ssd1306_oled #(.POWERUP_DELAY(10)) dut(clk, rst_n,
                     4'b1001, 8'd90, 1'b0, ena, addr, rw, data_wr, stop,
                     busy, data_rd, ack_err);
    i2c_master u_i2c(clk, rst_n, ena, addr, rw, data_wr, stop,
                     busy, data_rd, ack_err, bus_active, scl, sda);
    integer bits = 0, bytes_seen = 0, transactions = 0;
    reg [7:0] shift = 0;
    always @(negedge sda) if (scl) begin
        if (active) $fatal(1, "Unexpected repeated START");
        active = 1;
        bits = 0;
        bytes_seen = 0;
    end
    always @(posedge scl) if (active) begin
        if (bits < 8) begin
            shift = {shift[6:0], sda};
            bits = bits + 1;
            if (bits == 8) begin
                if (bytes_seen == 0 && shift !== 8'h78)
                    $fatal(1, "Invalid OLED address");
                if (transactions >= 25 && (transactions-25)%3 == 2) begin
                    if (bytes_seen == 1 && shift !== 8'h40)
                        $fatal(1, "Missing pixel control byte");
                    if (bytes_seen >= 2 &&
                        shift !== (dut.fbuf[(transactions-25)/3][bytes_seen-2] |
                                   dut.guide_byte((transactions-25)/3, bytes_seen-2)))
                        $fatal(1, "Pixel stream mismatch");
                end else if (bytes_seen == 1 && shift !== 0)
                    $fatal(1, "Missing command control byte");
            end
        end else begin
            if (sda !== 0) $fatal(1, "Missing ACK");
            bits = 0;
            bytes_seen = bytes_seen + 1;
        end
    end
    always @(negedge scl) slave_low = active && bits == 8;
    always @(posedge sda) if (scl && active) begin
        active = 0;
        if (transactions < 25 || (transactions-25)%3 == 0) begin
            if (bytes_seen != 3) $fatal(1, "Invalid command length");
        end else if ((transactions-25)%3 == 1) begin
            if (bytes_seen != 4) $fatal(1, "Invalid column command length");
        end else if (bytes_seen != 130) $fatal(1, "Invalid page length");
        transactions = transactions + 1;
        if (transactions == 49) begin
            // 90 deg right: offset = d^2 * 90 / 4096
            if (dut.guide_off[0] !== 0 || dut.guide_off[16] !== 5 ||
                dut.guide_off[32] !== 22 || dut.guide_byte(3'd7, 7'd99) !== 8'h80)
                $fatal(1, "Guide line offsets wrong");
            $display("PASS: OLED initialization and all 1024 framebuffer bytes");
            $finish;
        end
    end
    initial begin repeat (5) @(negedge clk); rst_n = 1; end
    initial begin #60_000_000; $fatal(1, "OLED timeout"); end
endmodule
