`timescale 1ns/1ps
module tb_uart;
    reg clk = 0, rst_n = 0, valid = 0;
    always #5 clk = ~clk;
    wire tx;
    debug_uart_tx dut(clk, rst_n, valid, 2'b01, 8'd45, 1'b1,
                      7'd40, 16'hff80, tx);
    reg [8*26-1:0] expected = {"AX:-00128 D:L T:40 A:045", 8'h0d, 8'h0a};
    reg [7:0] received;
    integer i, j;
    initial begin
        repeat (5) @(negedge clk); rst_n = 1;
        @(negedge clk); valid = 1;
        @(negedge clk); valid = 0;
        for (i = 0; i < 26; i = i + 1) begin
            @(negedge tx);
            repeat (117) @(negedge clk);
            if (tx !== 0) $fatal(1, "Invalid start bit");
            for (j = 0; j < 8; j = j + 1) begin
                repeat (234) @(negedge clk);
                received[j] = tx;
            end
            repeat (234) @(negedge clk);
            if (tx !== 1 || received !== expected[(25-i)*8 +: 8])
                $fatal(1, "UART byte %0d mismatch: %h", i, received);
        end
        $display("PASS: complete UART message and framing");
        $finish;
    end
    initial begin #1_000_000; $fatal(1, "UART timeout"); end
endmodule
