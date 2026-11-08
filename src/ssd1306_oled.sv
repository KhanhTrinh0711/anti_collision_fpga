module ssd1306_oled #(
    parameter POWERUP_DELAY = 22'd2_700_000 // 100 ms at 27 MHz
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [3:0]  obstacle,        // HC-SR04_1..4 (left to right) see an obstacle
    input  wire [7:0]  steer_angle,     // ADXL345 X tilt, 0..90
    input  wire        steer_sign,      // 0 = right, 1 = left
    // Request port to the shared I2C bus (i2c_arbiter)
    output reg         m_ena,
    output reg  [6:0]  m_addr,
    output reg         m_rw,
    output reg  [7:0]  m_data_wr,
    output reg         m_stop,
    input  wire        m_busy,
    input  wire [7:0]  m_data_rd,
    input  wire        m_ack_err
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
    localparam S_POWER_WAIT     = 4'd14;
    localparam S_GUIDE          = 4'd15;

    // Top view, front up: car outline x 50..77, y 0..30 (rear bumper at y 30).
    // Reverse guide lines run from y 31 (d = 0) to y 63 (d = 32).
    localparam [6:0] CAR_X0   = 7'd50;
    localparam [6:0] CAR_X1   = 7'd77;
    localparam [5:0] CAR_Y1   = 6'd30;
    localparam [5:0] GUIDE_Y0 = 6'd31;

    reg [21:0] powerup_cnt;
    reg [3:0] state;
    reg [5:0] init_step;
    reg [2:0] page;
    reg [6:0] col;
    reg       prev_busy;

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
            5'd21: init_cmd = 8'h20; // VCOMH = 0.77 * VCC
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
                "!": glyph = 48'h00_00_5F_00_00_00;
                default: glyph = 48'h00_00_00_00_00_00;
            endcase
            font_col = glyph[(5 - col_idx) * 8 +: 8];
        end
    endfunction

    // ─── Draw tasks ───────────────────────────────────────────────
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

    task set_px;
        input [6:0] x;
        input [5:0] y;
        begin
            fbuf[y[5:3]][x] = fbuf[y[5:3]][x] | (8'h01 << y[2:0]);
        end
    endtask

    // Car outline + "!" at each sensor that sees an obstacle:
    // 1 = left side, 2 = rear left, 3 = rear right, 4 = right side.
    task render_frame;
        integer i, j;
        begin
            for (i = 0; i < 8; i = i + 1)
                for (j = 0; j < 128; j = j + 1)
                    fbuf[i][j] = 8'h00;

            for (i = CAR_X0; i <= CAR_X1; i = i + 1) begin
                set_px(i, 6'd0);
                set_px(i, CAR_Y1);
            end
            for (j = 0; j <= CAR_Y1; j = j + 1) begin
                set_px(CAR_X0, j);
                set_px(CAR_X1, j);
            end

            if (obstacle[0]) draw_char("!", 7'd43, 3'd2); // lit column 45
            if (obstacle[1]) draw_char("!", 7'd53, 3'd4); // lit column 55
            if (obstacle[2]) draw_char("!", 7'd70, 3'd4); // lit column 72
            if (obstacle[3]) draw_char("!", 7'd80, 3'd2); // lit column 82
        end
    endtask

    // ─── Reverse guide lines ──────────────────────────────────────
    // Lateral shift d rows behind the bumper: d^2 * angle / 4096, a parabola
    // approximating the circular reverse path (max 22 px at 90, d = 32).
    // Overlaid while streaming: storing them in fbuf at variable addresses
    // would turn most of fbuf into real registers.
    reg [4:0]  guide_off [0:32];
    reg [5:0]  guide_d;
    reg [16:0] guide_acc;       // angle * d^2
    reg [12:0] guide_step;      // angle * (2d + 1)
    reg [6:0]  guide_angle;     // adxl345_top saturates at 90
    reg        guide_left;

    function [7:0] guide_byte;
        input [2:0] pg;
        input [6:0] c;
        integer b;
        reg [5:0] y;
        reg [6:0] o, xl, xr;
        begin
            guide_byte = 8'h00;
            for (b = 0; b < 8; b = b + 1) begin
                y = {pg, b[2:0]};
                if (y >= GUIDE_Y0) begin
                    // Reversing with the wheel turned right swings the rear right.
                    o  = {2'b00, guide_off[y - GUIDE_Y0]};
                    xl = guide_left ? CAR_X0 - o : CAR_X0 + o;
                    xr = guide_left ? CAR_X1 - o : CAR_X1 + o;
                    guide_byte[b] = (c == xl) || (c == xr);
                end
            end
        end
    endfunction

    // ─── FSM ──────────────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_POWER_WAIT;
            powerup_cnt <= 22'd0;
            init_step <= 6'd0;
            page      <= 3'd0;
            col       <= 7'd0;
            m_ena     <= 1'b0;
            m_addr    <= OLED_ADDR;
            m_rw      <= 1'b0;
            m_data_wr <= 8'h00;
            m_stop    <= 1'b1;
            prev_busy <= 1'b0;
            guide_d     <= 6'd0;
            guide_acc   <= 17'd0;
            guide_step  <= 13'd0;
            guide_angle <= 7'd0;
            guide_left  <= 1'b0;
        end else begin
            prev_busy <= m_busy;

            case (state)

                // The FPGA boots faster than the OLED supply settles.
                S_POWER_WAIT: begin
                    if (powerup_cnt == POWERUP_DELAY - 1)
                        state <= S_INIT;
                    else
                        powerup_cnt <= powerup_cnt + 1'b1;
                end

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
                        if (m_ack_err) begin
                            init_step <= 6'd0;
                            state     <= S_INIT;
                        end else if (init_step < INIT_TOTAL - 1) begin
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
                    page        <= 3'd0;
                    col         <= 7'd0;
                    guide_d     <= 6'd0;
                    guide_acc   <= 17'd0;
                    guide_step  <= {6'd0, steer_angle[6:0]};
                    guide_angle <= steer_angle[6:0];
                    guide_left  <= steer_sign;
                    state       <= S_GUIDE;
                end

                // ── Guide offsets, one row per cycle: (d+1)^2 = d^2 + 2d + 1 ──
                S_GUIDE: begin
                    guide_off[guide_d] <= guide_acc[16:12];
                    guide_acc  <= guide_acc + guide_step;
                    guide_step <= guide_step + {5'd0, guide_angle, 1'b0};
                    if (guide_d == 6'd32)
                        state <= S_SET_PAGE;
                    else
                        guide_d <= guide_d + 1'b1;
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
                        if (m_ack_err) begin
                            init_step <= 6'd0;
                            state     <= S_INIT;
                        end else begin
                            m_addr    <= OLED_ADDR;
                            m_rw      <= 1'b0;
                            m_data_wr <= 8'hB0 | {5'b0, page};
                            m_stop    <= 1'b1;
                            m_ena     <= 1'b1;
                            state     <= S_SET_COL;
                        end
                    end
                end

                // ── Set col: gửi [0x00][col_low][col_high] ────────
                S_SET_COL: begin
                    m_ena <= 1'b0;
                    if (prev_busy && !m_busy) begin
                        if (m_ack_err) begin
                            init_step <= 6'd0;
                            state     <= S_INIT;
                        end else begin
                            m_addr    <= OLED_ADDR;
                            m_rw      <= 1'b0;
                            m_data_wr <= 8'h00;
                            m_stop    <= 1'b0;
                            m_ena     <= 1'b1;
                            state     <= S_SET_COL_W;
                        end
                    end
                end

                S_SET_COL_W: begin
                    m_ena <= 1'b0;
                    if (prev_busy && !m_busy) begin
                        if (m_ack_err) begin
                            init_step <= 6'd0;
                            state     <= S_INIT;
                        end else begin
                            m_addr    <= OLED_ADDR;
                            m_rw      <= 1'b0;
                            m_data_wr <= {4'b0, col[3:0]};
                            m_stop    <= 1'b0;
                            m_ena     <= 1'b1;
                            state     <= S_SET_COL_LOW_W;
                        end
                    end
                end

                S_SET_COL_LOW_W: begin
                    m_ena <= 1'b0;
                    if (prev_busy && !m_busy) begin
                        if (m_ack_err) begin
                            init_step <= 6'd0;
                            state     <= S_INIT;
                        end else begin
                            m_addr    <= OLED_ADDR;
                            m_rw      <= 1'b0;
                            m_data_wr <= 8'h10 | {5'b0, col[6:4]};
                            m_stop    <= 1'b1;
                            m_ena     <= 1'b1;
                            state     <= S_SET_COL_HIGH_W;
                        end
                    end
                end

                S_SET_COL_HIGH_W: begin
                    m_ena <= 1'b0;
                    if (prev_busy && !m_busy) begin
                        if (m_ack_err) begin
                            init_step <= 6'd0;
                            state     <= S_INIT;
                        end else begin
                            col   <= 7'd0;
                            state <= S_SEND_DATA;
                        end
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
                        if (m_ack_err) begin
                            init_step <= 6'd0;
                            state     <= S_INIT;
                        end else begin
                            col   <= 7'd0;
                            state <= S_SEND_PIXEL;
                        end
                    end
                end

                S_SEND_PIXEL: begin
                    m_addr    <= OLED_ADDR;
                    m_rw      <= 1'b0;
                    m_data_wr <= fbuf[page][col] | guide_byte(page, col);
                    m_stop    <= (col == 7'd127);
                    m_ena     <= 1'b1;
                    state     <= S_SEND_PIXEL_W;
                end

                S_SEND_PIXEL_W: begin
                    m_ena <= 1'b0;
                    if (prev_busy && !m_busy) begin
                        if (m_ack_err) begin
                            init_step <= 6'd0;
                            state     <= S_INIT;
                        end else if (col < 7'd127) begin
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
