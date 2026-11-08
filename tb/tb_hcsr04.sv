`timescale 1ns/1ps
module tb_hcsr04;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0, echo = 0;
    wire trig, valid, timeout;
    wire [15:0] distance;
    hcsr04_controller #(.MEAS_PERIOD(54), .TIMEOUT_US(1000)) dut
        (clk, rst_n, echo, trig, distance, valid, timeout);
    time trigger_start, previous_trigger;
    always @(posedge trig) trigger_start = $time;
    always @(negedge trig) if (rst_n && $time - trigger_start != 5400)
        $fatal(1, "Trigger must last exactly 540 clock cycles");
    task result(input expected_timeout, input [15:0] expected_distance);
        begin
            @(posedge valid); #1;
            if (timeout !== expected_timeout || distance !== expected_distance)
                $fatal(1, "Unexpected result: timeout=%b distance=%d", timeout, distance);
        end
    endtask
    task pulse_echo(input integer us);
        begin
            @(negedge trig);
            repeat (54) @(negedge clk);
            echo = 1;
            repeat (us * 27) @(negedge clk);
            echo = 0;
        end
    endtask
    initial begin
        repeat (4) @(negedge clk);
        rst_n = 1;
        result(1, 0); // No echo must terminate.
        fork pulse_echo(580); result(0, 10); join
        fork pulse_echo(116); result(0, 8); join // (10+10+10+2)/4
        @(negedge trig);
        @(negedge clk); echo = 1;
        result(1, 0); // Stuck-high echo must terminate.
        @(negedge clk); echo = 0;
        fork pulse_echo(580); result(0, 8); join // Timeouts do not enter filter.
        @(negedge clk); echo = 1; previous_trigger = trigger_start;
        result(1, 0); // Echo high before trigger reports timeout without a burst.
        if (trigger_start != previous_trigger) $fatal(1, "Triggered during stale echo");
        echo = 0;
        $display("PASS: ultrasonic trigger, filtering, timeouts and recovery");
        $finish;
    end
    initial begin #10_000_000; $fatal(1, "Ultrasonic test timeout"); end
endmodule
