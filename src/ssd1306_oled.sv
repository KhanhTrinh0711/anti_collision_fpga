module ssd1306_oled (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rear_left_alert,
    input  wire        rear_right_alert,
    input  wire        side_left_alert,
    input  wire        side_right_alert,
    input  wire [7:0]  steer_angle,
    input  wire        steer_sign,
    inout  wire        oled_scl,
    inout  wire        oled_sda
);
    localparam OLED_ADDR = 7'h3C;

    // Mỗi lệnh SSD1306 = 2 byte: [0x00][CMD]
    // Tổng 25 lệnh × 2 byte = 50 byte
    // init_step chạy từ 0 đến 49
    // step chẵn (0,2,4...) → gửi 0x00 (control byte)
    // step lẻ  (1,3,5...) → gửi CMD tương ứng
    localparam INIT_TOTAL = 6'd50;

    localparam S_INIT        = 4'd0;
    localparam S_INIT_WAIT   = 4'd1;
    localparam S_RENDER      = 4'd2;
    localparam S_SET_PAGE    = 4'd3;
    localparam S_SET_PAGE_W  = 4'd4;
    localparam S_SET_COL     = 4'd5;
    localparam S_SET_COL_W   = 4'd6;
    localparam S_SEND_DATA   = 4'd7;
    localparam S_SEND_DATA_W = 4'd8;
    localparam S_NEXT        = 4'd9;
    localparam S_SET_COL_LOW_W  = 4'd10;
    localparam S_SET_COL_HIGH_W = 4'd11;
    localparam S_SEND_PIXEL     = 4'd12;
    localparam S_SEND_PIXEL_W   = 4'd13;

    reg [3:0] state;
    reg [5:0] init_step;
    reg [2:0] page;
    reg [6:0] col;
    reg       prev_busy;

    reg        m_ena;
    reg [6:0]  m_addr;
    reg        m_rw;
    reg [7:0]  m_data_wr;
    reg        m_stop;
    wire       m_busy;
    wire [7:0] m_data_rd;
    wire       m_ack_err;

    i2c_master #(
        .CLK_FREQ(27_000_000),
        .I2C_FREQ(400_000)
    ) u_oled_i2c (
        .clk      (clk),
        .rst_n    (rst_n),
        .ena      (m_ena),
        .addr     (m_addr),
        .rw       (m_rw),
        .data_wr  (m_data_wr),
        .stop     (m_stop),
        .busy     (m_busy),
        .data_rd  (m_data_rd),
        .ack_error(m_ack_err),
        .scl      (oled_scl),
        .sda      (oled_sda)
    );

    // ─── Init command lookup (25 lệnh) ───────────────────────────
    // index = init_step >> 1 (lấy số lệnh)
    reg [7:0] init_cmd;

    always @(*) begin
        case (init_step >> 1)
            5'd0:  init_cmd = 8'hAE; // Display OFF
            5'd1:  init_cmd = 8'hD5; // Set clock div
            5'd2:  init_cmd = 8'h80;
            5'd3:  init_cmd = 8'hA8; // Set multiplex
            5'd4:  init_cmd = 8'h3F; // 64 rows
            5'd5:  init_cmd = 8'hD3; // Display offset
            5'd6:  init_cmd = 8'h00;
            5'd7:  init_cmd = 8'h40; // Start line
            5'd8:  init_cmd = 8'h8D; // Charge pump
            5'd9:  init_cmd = 8'h14; // Enable
            5'd10: init_cmd = 8'h20; // Memory mode
            5'd11: init_cmd = 8'h02; // Page addressing mode
            5'd12: init_cmd = 8'hA1; // Segment remap
            5'd13: init_cmd = 8'hC8; // COM scan dir
            5'd14: init_cmd = 8'hDA; // COM pins
            5'd15: init_cmd = 8'h12;
            5'd16: init_cmd = 8'h81; // Contrast
            5'd17: init_cmd = 8'hCF;
            5'd18: init_cmd = 8'hD9; // Pre-charge
            5'd19: init_cmd = 8'hF1;
            5'd20: init_cmd = 8'hDB; // VCOMH
            5'd21: init_cmd = 8'h40;
            5'd22: init_cmd = 8'hA4; // Display RAM
            5'd23: init_cmd = 8'hA6; // Normal display
            5'd24: init_cmd = 8'hAF; // Display ON
            default: init_cmd = 8'h00;
        endcase
    end

    // ─── Frame buffer ─────────────────────────────────────────────
    reg [7:0] fbuf [0:7][0:127];

    // ─── Font ─────────────────────────────────────────────────────
    function [7:0] font_col;
        input [7:0] ch;
        input [2:0] col_idx;
        reg [47:0] glyph;
        begin
            case (ch)
                "0": glyph = 48'h3E_51_49_45_3E_00;
                "1": glyph = 48'h00_42_7F_40_00_00;
                "2": glyph = 48'h42_61_51_49_46_00;
                "3": glyph = 48'h21_41_45_4B_31_00;
                "4": glyph = 48'h18_14_12_7F_10_00;
                "5": glyph = 48'h27_45_45_45_39_00;
                "6": glyph = 48'h3C_4A_49_49_30_00;
                "7": glyph = 48'h01_71_09_05_03_00;
                "8": glyph = 48'h36_49_49_49_36_00;
                "9": glyph = 48'h06_49_49_29_1E_00;
                "+": glyph = 48'h08_08_3E_08_08_00;
                "-": glyph = 48'h08_08_08_08_08_00;
                "!": glyph = 48'h00_00_5F_00_00_00;
                default: glyph = 48'h00_00_00_00_00_00;
            endcase
            font_col = glyph[(5 - col_idx) * 8 +: 8];
        end
    endfunction

    // ─── Draw tasks ───────────────────────────────────────────────
    task draw_rect;
        input [6:0] x0, x1;
        input [2:0] y0, y1;
        integer i, j;
        begin
            for (i = x0; i <= x1; i = i + 1) begin
                fbuf[y0][i] = fbuf[y0][i] | 8'h01;
                fbuf[y1][i] = fbuf[y1][i] | 8'h80;
            end
            for (j = y0; j <= y1; j = j + 1) begin
                fbuf[j][x0] = 8'hFF;
                fbuf[j][x1] = 8'hFF;
            end
        end
    endtask

    task draw_char;
        input [7:0] ch;
        input [6:0] x;
        input [2:0] pg;
        integer k;
        begin
            for (k = 0; k < 5; k = k + 1)
                fbuf[pg][x+k] = font_col(ch, k[2:0]);
        end
    endtask

    task draw_steering;
        input [6:0] cx;
        input [2:0] pg;
        begin
            fbuf[pg][cx-7] = 8'h18;
            fbuf[pg][cx-6] = 8'h24;
            fbuf[pg][cx-5] = 8'h42;
            fbuf[pg][cx-4] = 8'h81;
            fbuf[pg][cx-3] = 8'h81;
            fbuf[pg][cx-2] = 8'h81;
            fbuf[pg][cx-1] = 8'hBD;
            fbuf[pg][cx]   = 8'hBD;
            fbuf[pg][cx+1] = 8'hBD;
            fbuf[pg][cx+2] = 8'h81;
            fbuf[pg][cx+3] = 8'h81;
            fbuf[pg][cx+4] = 8'h81;
            fbuf[pg][cx+5] = 8'h42;
            fbuf[pg][cx+6] = 8'h24;
            fbuf[pg][cx+7] = 8'h18;
        end
    endtask

    task render_frame;
        integer i, j;
        reg [7:0] angle_h, angle_t, angle_l;
        begin
            for (i = 0; i < 8; i = i + 1)
                for (j = 0; j < 128; j = j + 1)
                    fbuf[i][j] = 8'h00;

            draw_rect(7'd30, 7'd90, 3'd1, 3'd4);

            if (rear_left_alert)  draw_char("!", 7'd25, 3'd0);
            if (rear_right_alert) draw_char("!", 7'd88, 3'd0);
            if (side_left_alert)  draw_char("!", 7'd18, 3'd2);
            if (side_right_alert) draw_char("!", 7'd95, 3'd2);

            draw_steering(7'd20, 3'd6);

            if (steer_sign) draw_char("-", 7'd36, 3'd6);
            else            draw_char("+", 7'd36, 3'd6);

            angle_h = steer_angle / 8'd100;
            angle_t = (steer_angle % 8'd100) / 8'd10;
            angle_l = steer_angle % 8'd10;

            draw_char("0" + angle_h, 7'd42, 3'd6);
            draw_char("0" + angle_t, 7'd48, 3'd6);
            draw_char("0" + angle_l, 7'd54, 3'd6);
        end
    endtask

    // ─── FSM ──────────────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_INIT;
            init_step <= 6'd0;
            page      <= 3'd0;
            col       <= 7'd0;
            m_ena     <= 1'b0;
            m_addr    <= OLED_ADDR;
            m_rw      <= 1'b0;
            m_data_wr <= 8'h00;
            m_stop    <= 1'b1;
            prev_busy <= 1'b0;
        end else begin
            prev_busy <= m_busy;

            case (state)

                // ── Init: gửi từng byte [0x00][CMD] ──────────────
                S_INIT: begin
                    m_addr <= OLED_ADDR;
                    m_rw   <= 1'b0;

                    if (init_step[0] == 1'b0) begin
                        // Byte chẵn → control byte 0x00
                        m_data_wr <= 8'h00;
                        m_stop    <= 1'b0;  // không STOP, còn CMD tiếp theo
                    end else begin
                        // Byte lẻ → CMD thực sự
                        m_data_wr <= init_cmd;
                        m_stop    <= 1'b1;  // STOP sau CMD
                    end

                    m_ena <= 1'b1;
                    state <= S_INIT_WAIT;
                end

                S_INIT_WAIT: begin
                    m_ena <= 1'b0;
                    if (prev_busy && !m_busy) begin
                        if (init_step < INIT_TOTAL - 1) begin
                            init_step <= init_step + 1'b1;
                            state     <= S_INIT;
                        end else begin
                            state <= S_RENDER;
                        end
                    end
                end

                // ── Render frame buffer ───────────────────────────
                S_RENDER: begin
                    render_frame();
                    page  <= 3'd0;
                    col   <= 7'd0;
                    state <= S_SET_PAGE;
                end

                // ── Set page: gửi [0x00][0xB0|page] ──────────────
                S_SET_PAGE: begin
                    m_addr    <= OLED_ADDR;
                    m_rw      <= 1'b0;
                    m_data_wr <= 8'h00;     // control byte
                    m_stop    <= 1'b0;
                    m_ena     <= 1'b1;
                    state     <= S_SET_PAGE_W;
                end

                S_SET_PAGE_W: begin
                    m_ena <= 1'b0;
                    if (prev_busy && !m_busy) begin
                        // Gửi tiếp page address
                        m_addr    <= OLED_ADDR;
                        m_rw      <= 1'b0;
                        m_data_wr <= 8'hB0 | {5'b0, page};
                        m_stop    <= 1'b1;
                        m_ena     <= 1'b1;
                        state     <= S_SET_COL;
                    end
                end

                // ── Set col: gửi [0x00][col_low][col_high] ────────
                S_SET_COL: begin
                    m_ena <= 1'b0;
                    if (prev_busy && !m_busy) begin
                        m_addr    <= OLED_ADDR;
                        m_rw      <= 1'b0;
                        m_data_wr <= 8'h00;  // control byte
                        m_stop    <= 1'b0;
                        m_ena     <= 1'b1;
                        state     <= S_SET_COL_W;
                    end
                end

                S_SET_COL_W: begin
                    m_ena <= 1'b0;
                    if (prev_busy && !m_busy) begin
                        m_addr    <= OLED_ADDR;
                        m_rw      <= 1'b0;
                        m_data_wr <= {4'b0, col[3:0]};
                        m_stop    <= 1'b0;
                        m_ena     <= 1'b1;
                        state     <= S_SET_COL_LOW_W;
                    end
                end

                S_SET_COL_LOW_W: begin
                    m_ena <= 1'b0;
                    if (prev_busy && !m_busy) begin
                        m_addr    <= OLED_ADDR;
                        m_rw      <= 1'b0;
                        m_data_wr <= 8'h10 | {4'b0, col[6:4]};
                        m_stop    <= 1'b1;
                        m_ena     <= 1'b1;
                        state     <= S_SET_COL_HIGH_W;
                    end
                end

                S_SET_COL_HIGH_W: begin
                    m_ena <= 1'b0;
                    if (prev_busy && !m_busy) begin
                        col   <= 7'd0;
                        state <= S_SEND_DATA;
                    end
                end

                S_SEND_DATA: begin
                    m_addr    <= OLED_ADDR;
                    m_rw      <= 1'b0;
                    m_data_wr <= 8'h40;
                    m_stop    <= 1'b0;
                    m_ena     <= 1'b1;
                    state     <= S_SEND_DATA_W;
                end

                S_SEND_DATA_W: begin
                    m_ena <= 1'b0;
                    if (prev_busy && !m_busy) begin
                        col   <= 7'd0;
                        state <= S_SEND_PIXEL;
                    end
                end

                S_SEND_PIXEL: begin
                    m_addr    <= OLED_ADDR;
                    m_rw      <= 1'b0;
                    m_data_wr <= fbuf[page][col];
                    m_stop    <= (col == 7'd127);
                    m_ena     <= 1'b1;
                    state     <= S_SEND_PIXEL_W;
                end

                S_SEND_PIXEL_W: begin
                    m_ena <= 1'b0;
                    if (prev_busy && !m_busy) begin
                        if (col < 7'd127) begin
                            col   <= col + 1'b1;
                            state <= S_SEND_PIXEL;
                        end else begin
                            state <= S_NEXT;
                        end
                    end
                end

                // ── Next page hoặc render lại ─────────────────────
                S_NEXT: begin
                    col <= 7'd0;
                    if (page < 3'd7) begin
                        page  <= page + 1'b1;
                        state <= S_SET_PAGE;
                    end else
                        state <= S_RENDER;
                end

                default: state <= S_INIT;
            endcase
        end
    end

endmodule