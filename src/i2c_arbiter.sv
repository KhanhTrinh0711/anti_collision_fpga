module i2c_arbiter (
    input  wire clk,
    input  wire rst_n,

    // Port 0: ADXL345 (ưu tiên cao hơn)
    input  wire        p0_ena,
    input  wire [6:0]  p0_addr,
    input  wire        p0_rw,
    input  wire [7:0]  p0_data_wr,
    input  wire        p0_stop,
    output wire        p0_busy,
    output wire [7:0]  p0_data_rd,
    output wire        p0_ack_err,

    // Port 1: OLED SSD1306
    input  wire        p1_ena,
    input  wire [6:0]  p1_addr,
    input  wire        p1_rw,
    input  wire [7:0]  p1_data_wr,
    input  wire        p1_stop,
    output wire        p1_busy,
    output wire [7:0]  p1_data_rd,
    output wire        p1_ack_err,

    // I2C bus
    inout  wire        scl,
    inout  wire        sda
);
    // ─── I2C master dùng chung ────────────────────────────────────
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
        .I2C_FREQ(100_000)
    ) u_i2c (
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
        .scl      (scl),
        .sda      (sda)
    );

    // ─── Arbiter state ────────────────────────────────────────────
    // owner: 0 = ADXL345, 1 = OLED
    // ADXL345 có priority cao hơn
    reg owner;
    reg bus_free;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            owner    <= 1'b0;
            bus_free <= 1'b1;
        end else begin
            if (bus_free) begin
                if (p0_ena) begin
                    owner    <= 1'b0;  // ADXL345
                    bus_free <= 1'b0;
                end else if (p1_ena) begin
                    owner    <= 1'b1;  // OLED
                    bus_free <= 1'b0;
                end
            end else begin
                // Bus giải phóng khi m_busy xuống LOW
                if (!m_busy && !p0_ena && !p1_ena)
                    bus_free <= 1'b1;
            end
        end
    end

    // ─── Mux input sang i2c_master ───────────────────────────────
    always @(*) begin
        if (!bus_free && owner == 1'b0) begin
            m_ena     = p0_ena;
            m_addr    = p0_addr;
            m_rw      = p0_rw;
            m_data_wr = p0_data_wr;
            m_stop    = p0_stop;
        end else begin
            m_ena     = p1_ena;
            m_addr    = p1_addr;
            m_rw      = p1_rw;
            m_data_wr = p1_data_wr;
            m_stop    = p1_stop;
        end
    end

    // ─── Output routing ───────────────────────────────────────────
    assign p0_busy    = (owner == 1'b0) ? m_busy    : 1'b0;
    assign p0_data_rd = (owner == 1'b0) ? m_data_rd : 8'd0;
    assign p0_ack_err = (owner == 1'b0) ? m_ack_err : 1'b0;

    assign p1_busy    = (owner == 1'b1) ? m_busy    : 1'b0;
    assign p1_data_rd = (owner == 1'b1) ? m_data_rd : 8'd0;
    assign p1_ack_err = (owner == 1'b1) ? m_ack_err : 1'b0;

endmodule