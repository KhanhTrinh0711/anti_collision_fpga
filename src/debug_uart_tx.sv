module debug_uart_tx (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid,
    input  wire [1:0]  steer_dir,
    input  wire [7:0]  steer_angle,
    input  wire        steer_sign,
    input  wire [6:0]  danger_threshold,
    input  wire [15:0] accel_raw,
    output reg         uart_tx
);
    localparam BAUD_DIV = 12'd234;

    localparam U_IDLE  = 2'd0;
    localparam U_START = 2'd1;
    localparam U_DATA  = 2'd2;
    localparam U_STOP  = 2'd3;

    reg [7:0]  tx_buf [0:25];
    reg [4:0]  tx_len;
    reg [4:0]  tx_idx;
    reg [1:0]  u_state;
    reg [11:0] baud_cnt;
    reg [2:0]  bit_cnt;
    reg [7:0]  shift_reg;
    reg        sending;

    // ─── Biến trung gian để tính digits ──────────────────────────
    reg [15:0] accel_abs;
    reg [15:0] tmp;
    reg [3:0]  d4, d3, d2, d1, d0;  // 5 chữ số accel
    reg [3:0]  t2, t1, t0;          // 3 chữ số angle
    reg [3:0]  th1, th0;            // 2 chữ số threshold

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_tx   <= 1'b1;
            u_state   <= U_IDLE;
            baud_cnt  <= 12'd0;
            bit_cnt   <= 3'd0;
            tx_idx    <= 5'd0;
            tx_len    <= 5'd0;
            sending   <= 1'b0;
            shift_reg <= 8'hFF;
        end else begin

            // ── Build buffer khi valid và không đang gửi ──────────
            if (valid && !sending) begin

                // Tính accel tuyệt đối
                accel_abs = accel_raw[15] ?
                            (~accel_raw + 16'd1) : accel_raw;

                // Tách 5 chữ số accel
                tmp = accel_abs;
                d4 = tmp / 16'd10000;
                tmp = tmp % 16'd10000;
                d3 = tmp / 16'd1000;
                tmp = tmp % 16'd1000;
                d2 = tmp / 16'd100;
                tmp = tmp % 16'd100;
                d1 = tmp / 16'd10;
                d0 = tmp % 16'd10;

                // Tách 3 chữ số angle
                t2 = steer_angle / 8'd100;
                t1 = (steer_angle % 8'd100) / 8'd10;
                t0 = steer_angle % 8'd10;

                // Tách 2 chữ số threshold
                th1 = {9'd0, danger_threshold} / 8'd10;
                th0 = {9'd0, danger_threshold} % 8'd10;

                // Ghi buffer
                tx_buf[0]  <= 8'h41;                         // 'A'
                tx_buf[1]  <= 8'h58;                         // 'X'
                tx_buf[2]  <= 8'h3A;                         // ':'
                tx_buf[3]  <= accel_raw[15] ? 8'h2D : 8'h2B;// '+'/'-'
                tx_buf[4]  <= 8'h30 + {4'b0, d4};
                tx_buf[5]  <= 8'h30 + {4'b0, d3};
                tx_buf[6]  <= 8'h30 + {4'b0, d2};
                tx_buf[7]  <= 8'h30 + {4'b0, d1};
                tx_buf[8]  <= 8'h30 + {4'b0, d0};
                tx_buf[9]  <= 8'h20;                         // ' '
                tx_buf[10] <= 8'h44;                         // 'D'
                tx_buf[11] <= 8'h3A;                         // ':'
                tx_buf[12] <= (steer_dir == 2'b10) ? 8'h52 : // 'R'
                              (steer_dir == 2'b01) ? 8'h4C : // 'L'
                                                     8'h53;  // 'S'
                tx_buf[13] <= 8'h20;                         // ' '
                tx_buf[14] <= 8'h54;                         // 'T'
                tx_buf[15] <= 8'h3A;                         // ':'
                tx_buf[16] <= 8'h30 + {4'b0, th1};
                tx_buf[17] <= 8'h30 + {4'b0, th0};
                tx_buf[18] <= 8'h20;                         // ' '
                tx_buf[19] <= 8'h41;                         // 'A'
                tx_buf[20] <= 8'h3A;                         // ':'
                tx_buf[21] <= 8'h30 + {4'b0, t2};
                tx_buf[22] <= 8'h30 + {4'b0, t1};
                tx_buf[23] <= 8'h30 + {4'b0, t0};
                tx_buf[24] <= 8'h0D;                         // '\r'
                tx_buf[25] <= 8'h0A;                         // '\n'

                tx_len  <= 5'd26;
                tx_idx  <= 5'd0;
                sending <= 1'b1;
            end

            // ── UART TX FSM ───────────────────────────────────────
            case (u_state)
                U_IDLE: begin
                    uart_tx <= 1'b1;
                    if (sending && tx_idx < tx_len) begin
                        shift_reg <= tx_buf[tx_idx];
                        baud_cnt  <= 12'd0;
                        u_state   <= U_START;
                    end else if (sending && tx_idx >= tx_len) begin
                        sending <= 1'b0;
                    end
                end

                U_START: begin
                    uart_tx <= 1'b0;
                    if (baud_cnt >= BAUD_DIV - 1) begin
                        baud_cnt <= 12'd0;
                        bit_cnt  <= 3'd0;
                        u_state  <= U_DATA;
                    end else
                        baud_cnt <= baud_cnt + 1'b1;
                end

                U_DATA: begin
                    uart_tx <= shift_reg[0];
                    if (baud_cnt >= BAUD_DIV - 1) begin
                        baud_cnt  <= 12'd0;
                        shift_reg <= {1'b0, shift_reg[7:1]};
                        if (bit_cnt == 3'd7) begin
                            bit_cnt <= 3'd0;
                            u_state <= U_STOP;
                        end else
                            bit_cnt <= bit_cnt + 1'b1;
                    end else
                        baud_cnt <= baud_cnt + 1'b1;
                end

                U_STOP: begin
                    uart_tx <= 1'b1;
                    if (baud_cnt >= BAUD_DIV - 1) begin
                        baud_cnt <= 12'd0;
                        tx_idx   <= tx_idx + 1'b1;
                        u_state  <= U_IDLE;
                    end else
                        baud_cnt <= baud_cnt + 1'b1;
                end

                default: u_state <= U_IDLE;
            endcase
        end
    end

endmodule