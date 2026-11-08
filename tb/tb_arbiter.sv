`timescale 1ns/1ps
module tb_arbiter;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;
    reg e0 = 0, e1 = 0, stop0 = 0;
    reg [7:0] data0 = 8'h2d;
    wire b0, b1, err0, err1;
    wire [7:0] rd0, rd1;
    tri1 scl, sda;
    wire ack = dut.u_i2c.state == 3 || dut.u_i2c.state == 5;
    assign sda = ack ? 1'b0 : 1'bz;
    i2c_arbiter dut(clk, rst_n,
        e0, 7'h53, 1'b0, data0, stop0, b0, rd0, err0,
        e1, 7'h3c, 1'b0, 8'hae, 1'b1, b1, rd1, err1, scl, sda);
    initial begin
        repeat (4) @(negedge clk); rst_n = 1;
        @(negedge clk); e0 = 1; e1 = 1;
        @(negedge clk); e0 = 0; e1 = 0;
        if (!b0 || !b1 || dut.owner !== 0) $fatal(1, "Priority/queue failed");
        @(negedge b0);
        if (!b1 || !dut.u_i2c.bus_active) $fatal(1, "Lost transaction ownership");
        @(negedge clk); data0 = 8'h08; stop0 = 1; e0 = 1;
        @(negedge clk); e0 = 0;
        wait(b0); @(negedge b0);
        if (err0) $fatal(1, "Port 0 ACK error");
        // The queued port must stay busy throughout ownership handover.
        @(negedge b1);
        if (err1 || dut.u_i2c.saved_addr_rw !== 8'h78)
            $fatal(1, "Queued port was not serviced");
        $display("PASS: arbiter priority, continuation and queued request");
        $finish;
    end
    initial begin #2_000_000; $fatal(1, "Arbiter timeout"); end
endmodule
