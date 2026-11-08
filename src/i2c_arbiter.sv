module i2c_arbiter (
    input  wire clk,
    input  wire rst_n,

    // Port 0: ADXL345 (higher priority)
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

    inout  wire        scl,
    inout  wire        sda
);
    reg        owner;
    reg        bus_owned;
    reg        p0_pending;
    reg        p1_pending;
    reg [6:0]  p0_addr_pending;
    reg        p0_rw_pending;
    reg [7:0]  p0_data_pending;
    reg        p0_stop_pending;
    reg [6:0]  p1_addr_pending;
    reg        p1_rw_pending;
    reg [7:0]  p1_data_pending;
    reg        p1_stop_pending;

    wire       m_ena;
    wire [6:0] m_addr;
    wire       m_rw;
    wire [7:0] m_data_wr;
    wire       m_stop;
    wire       m_busy;
    wire       m_bus_active;
    wire [7:0] m_data_rd;
    wire       m_ack_err;
    wire       select_p0 = !bus_owned && (p0_pending || p0_ena);
    wire       select_p1 = !bus_owned && !select_p0 &&
                           (p1_pending || p1_ena);

    assign m_ena = bus_owned ? (owner ? p1_ena : p0_ena) :
                   select_p0 ? (p0_pending ? 1'b1 : p0_ena) :
                   select_p1 ? (p1_pending ? 1'b1 : p1_ena) : 1'b0;
    assign m_addr = bus_owned ? (owner ? p1_addr : p0_addr) :
                    select_p0 ? (p0_pending ? p0_addr_pending : p0_addr) :
                    select_p1 ? (p1_pending ? p1_addr_pending : p1_addr) :
                    7'd0;
    assign m_rw = bus_owned ? (owner ? p1_rw : p0_rw) :
                  select_p0 ? (p0_pending ? p0_rw_pending : p0_rw) :
                  select_p1 ? (p1_pending ? p1_rw_pending : p1_rw) : 1'b0;
    assign m_data_wr = bus_owned ? (owner ? p1_data_wr : p0_data_wr) :
                       select_p0 ? (p0_pending ? p0_data_pending : p0_data_wr) :
                       select_p1 ? (p1_pending ? p1_data_pending : p1_data_wr) :
                       8'd0;
    assign m_stop = bus_owned ? (owner ? p1_stop : p0_stop) :
                    select_p0 ? (p0_pending ? p0_stop_pending : p0_stop) :
                    select_p1 ? (p1_pending ? p1_stop_pending : p1_stop) :
                    1'b1;

    i2c_master #(
        .CLK_FREQ(27_000_000),
        .I2C_FREQ(100_000)
    ) u_i2c (
        .clk       (clk),
        .rst_n     (rst_n),
        .ena       (m_ena),
        .addr      (m_addr),
        .rw        (m_rw),
        .data_wr   (m_data_wr),
        .stop      (m_stop),
        .busy      (m_busy),
        .data_rd   (m_data_rd),
        .ack_error (m_ack_err),
        .bus_active(m_bus_active),
        .scl       (scl),
        .sda       (sda)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            owner           <= 1'b0;
            bus_owned       <= 1'b0;
            p0_pending       <= 1'b0;
            p1_pending       <= 1'b0;
            p0_addr_pending  <= 7'd0;
            p0_rw_pending    <= 1'b0;
            p0_data_pending  <= 8'd0;
            p0_stop_pending  <= 1'b1;
            p1_addr_pending  <= 7'd0;
            p1_rw_pending    <= 1'b0;
            p1_data_pending  <= 8'd0;
            p1_stop_pending  <= 1'b1;
        end else if (bus_owned) begin
            if (owner == 1'b0 && p1_ena && !p1_pending) begin
                p1_pending      <= 1'b1;
                p1_addr_pending <= p1_addr;
                p1_rw_pending   <= p1_rw;
                p1_data_pending <= p1_data_wr;
                p1_stop_pending <= p1_stop;
            end else if (owner == 1'b1 && p0_ena && !p0_pending) begin
                p0_pending      <= 1'b1;
                p0_addr_pending <= p0_addr;
                p0_rw_pending   <= p0_rw;
                p0_data_pending <= p0_data_wr;
                p0_stop_pending <= p0_stop;
            end

            if (!m_busy && !m_bus_active)
                bus_owned <= 1'b0;
        end else if (select_p0) begin
            owner     <= 1'b0;
            bus_owned <= 1'b1;
            if (p0_pending)
                p0_pending <= 1'b0;
            if (p1_ena && !p1_pending) begin
                p1_pending      <= 1'b1;
                p1_addr_pending <= p1_addr;
                p1_rw_pending   <= p1_rw;
                p1_data_pending <= p1_data_wr;
                p1_stop_pending <= p1_stop;
            end
        end else if (select_p1) begin
            owner     <= 1'b1;
            bus_owned <= 1'b1;
            if (p1_pending)
                p1_pending <= 1'b0;
            if (p0_ena && !p0_pending) begin
                p0_pending      <= 1'b1;
                p0_addr_pending <= p0_addr;
                p0_rw_pending   <= p0_rw;
                p0_data_pending <= p0_data_wr;
                p0_stop_pending <= p0_stop;
            end
        end
    end

    assign p0_busy    = p0_pending || (bus_owned && !owner && m_busy);
    assign p0_data_rd = (bus_owned && !owner) ? m_data_rd : 8'd0;
    assign p0_ack_err = (bus_owned && !owner) ? m_ack_err : 1'b0;

    assign p1_busy    = p1_pending || (bus_owned && owner && m_busy);
    assign p1_data_rd = (bus_owned && owner) ? m_data_rd : 8'd0;
    assign p1_ack_err = (bus_owned && owner) ? m_ack_err : 1'b0;
endmodule
