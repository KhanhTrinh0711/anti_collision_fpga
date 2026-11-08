    module i2c_master #(
    parameter CLK_FREQ = 27_000_000, 
    parameter I2C_FREQ = 100_000     
)(
    input  wire       clk,         
    input  wire       rst_n,       
    input  wire       ena,          
    input  wire [6:0] addr,        
    input  wire       rw,          
    input  wire [7:0] data_wr,     
    input  wire       stop,     // Emit STOP after this byte; 0 continues the current transfer.

    output reg        busy,    
    output reg  [7:0] data_rd,      
    output reg        ack_error,   
    output reg        bus_active,

    inout  wire       scl,
    inout  wire       sda
);

    localparam DIVIDER = (CLK_FREQ + (I2C_FREQ * 4) - 1) / (I2C_FREQ * 4);

    localparam STATE_IDLE    = 4'd0;
    localparam STATE_START   = 4'd1;
    localparam STATE_ADDR    = 4'd2;
    localparam STATE_SLV_ACK = 4'd3;
    localparam STATE_WRITE   = 4'd4;
    localparam STATE_WR_ACK  = 4'd5;
    localparam STATE_READ    = 4'd6;
    localparam STATE_MSTR_ACK= 4'd7;
    localparam STATE_STOP    = 4'd8;
    localparam STATE_RESTART = 4'd9;
    localparam STATE_RECOVER = 4'd10;

    reg [3:0]  state;
    reg [31:0] clk_cnt;
    reg [1:0]  phase;          
    reg [2:0]  bit_idx;        
    reg [7:0]  saved_addr_rw;
    reg [7:0]  saved_data;
    reg        saved_stop;
    reg        abort_transfer;
    reg [3:0]  recover_cnt;

    reg scl_enable;
    reg sda_enable;

    assign scl = scl_enable ? 1'b0 : 1'bz;
    assign sda = sda_enable ? 1'b0 : 1'bz;

    // Bus levels are asynchronous to clk (slave drive, slow pull-up edges).
    reg [1:0] scl_sync;
    reg [1:0] sda_sync;
    wire scl_in = scl_sync[1];
    wire sda_in = sda_sync[1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_sync <= 2'b11;
            sda_sync <= 2'b11;
        end else begin
            scl_sync <= {scl_sync[0], scl};
            sda_sync <= {sda_sync[0], sda};
        end
    end

    wire cycle_pulse = (clk_cnt == DIVIDER - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_cnt <= 32'd0;
            phase   <= 2'd0;
        end else if (busy && !scl_enable && !scl_in) begin
            // A slave may hold SCL low after the master releases it.
            clk_cnt <= clk_cnt;
            phase <= phase;
        end else if (busy) begin
            if (cycle_pulse) begin
                clk_cnt <= 32'd0;
                phase   <= phase + 1'b1;
            end else begin
                clk_cnt <= clk_cnt + 1'b1;
            end
        end else begin
            clk_cnt <= 32'd0;
            phase   <= 2'd0;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= STATE_IDLE;
            busy          <= 1'b0;
            scl_enable    <= 1'b0;
            sda_enable    <= 1'b0;
            data_rd       <= 8'd0;
            ack_error     <= 1'b0;
            bit_idx       <= 3'd7;
            saved_addr_rw <= 8'd0;
            saved_data    <= 8'd0;
            saved_stop    <= 1'b0;
            bus_active    <= 1'b0;
            abort_transfer<= 1'b0;
            recover_cnt   <= 4'd0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    scl_enable <= bus_active;
                    sda_enable <= 1'b0;
                    if (ena) begin
                        busy          <= 1'b1;
                        saved_addr_rw <= {addr, rw};
                        saved_data    <= data_wr;
                        saved_stop    <= stop;
                        abort_transfer<= 1'b0;
                        if (!bus_active)
                            ack_error <= 1'b0;
                        if (bus_active && saved_addr_rw != {addr, rw}) begin
                            state <= STATE_RESTART;
                        end else if (bus_active) begin
                            bit_idx <= 3'd7;
                            state   <= rw ? STATE_READ : STATE_WRITE;
                        end else begin
                            state <= STATE_START;
                        end
                    end else begin
                        busy <= 1'b0;
                    end
                end

                STATE_RESTART: begin
                    if (cycle_pulse) begin
                        case (phase)
                            2'd0: sda_enable <= 1'b0;
                            2'd1: scl_enable <= 1'b0;
                            2'd2: sda_enable <= 1'b1;
                            2'd3: begin
                                scl_enable <= 1'b1;
                                bit_idx <= 3'd7;
                                state <= STATE_ADDR;
                            end
                        endcase
                    end
                end

                STATE_START: begin
                    if (cycle_pulse) begin
                        case (phase)
                            2'd0: begin
                                sda_enable <= 1'b0;
                                scl_enable <= 1'b0;
                                // A slave interrupted mid-read (FPGA reset or
                                // reprogram) keeps SDA low until clocked out.
                                if (!sda_in) begin
                                    recover_cnt <= 4'd0;
                                    state       <= STATE_RECOVER;
                                end
                            end
                            2'd1: begin sda_enable <= 1'b1; scl_enable <= 1'b0; end
                            2'd2: begin sda_enable <= 1'b1; scl_enable <= 1'b0; end
                            2'd3: begin
                                scl_enable <= 1'b1;
                                bit_idx    <= 3'd7;
                                state      <= STATE_ADDR;
                            end
                        endcase
                    end
                end

                STATE_RECOVER: begin
                    if (cycle_pulse) begin
                        case (phase)
                            2'd1: scl_enable <= 1'b1;
                            2'd3: begin
                                scl_enable <= 1'b0;
                                if (sda_in) begin
                                    state <= STATE_START;
                                end else if (recover_cnt == 4'd8) begin
                                    ack_error <= 1'b1;
                                    busy      <= 1'b0;
                                    state     <= STATE_IDLE;
                                end else begin
                                    recover_cnt <= recover_cnt + 1'b1;
                                end
                            end
                        endcase
                    end
                end

                STATE_ADDR: begin
                    if (cycle_pulse) begin
                        case (phase)
                            2'd0: sda_enable <= ~saved_addr_rw[bit_idx]; 
                            2'd1: scl_enable <= 1'b0;                    
                            2'd2: scl_enable <= 1'b0;                   
                            2'd3: begin
                                scl_enable <= 1'b1;                      
                                if (bit_idx == 3'd0) begin
                                    state <= STATE_SLV_ACK;
                                end else begin
                                    bit_idx <= bit_idx - 1'b1;
                                end
                            end
                        endcase
                    end
                end

                STATE_SLV_ACK: begin
                    if (cycle_pulse) begin
                        case (phase)
                            2'd0: sda_enable <= 1'b0; 
                            2'd1: scl_enable <= 1'b0; 
                            2'd2: begin

                                if (sda_in == 1'b1) begin
                                    ack_error      <= 1'b1;
                                    abort_transfer <= 1'b1;
                                end
                            end
                            2'd3: begin
                                scl_enable <= 1'b1;
                                bit_idx    <= 3'd7;
                                if (abort_transfer) begin
                                    bus_active <= 1'b0;
                                    state      <= STATE_STOP;
                                end else if (saved_addr_rw[0] == 1'b0)
                                    state <= STATE_WRITE; 
                                else
                                    state <= STATE_READ;  
                            end
                        endcase
                    end
                end

                STATE_WRITE: begin
                    if (cycle_pulse) begin
                        case (phase)
                            2'd0: sda_enable <= ~saved_data[bit_idx];
                            2'd1: scl_enable <= 1'b0;
                            2'd2: scl_enable <= 1'b0;
                            2'd3: begin
                                scl_enable <= 1'b1;
                                if (bit_idx == 3'd0) begin
                                    state <= STATE_WR_ACK;
                                end else begin
                                    bit_idx <= bit_idx - 1'b1;
                                end
                            end
                        endcase
                    end
                end

                STATE_WR_ACK: begin
                    if (cycle_pulse) begin
                        case (phase)
                            2'd0: sda_enable <= 1'b0;
                            2'd1: scl_enable <= 1'b0;
                            2'd2: begin
                                if (sda_in == 1'b1) begin
                                    ack_error      <= 1'b1;
                                    abort_transfer <= 1'b1;
                                end
                            end
                            2'd3: begin
                                scl_enable <= 1'b1;
                                if (saved_stop || abort_transfer) begin
                                    bus_active <= 1'b0;
                                    state <= STATE_STOP;
                                end else begin
                                    bus_active <= 1'b1;
                                    state <= STATE_IDLE;
                                end
                            end
                        endcase
                    end
                end

                STATE_READ: begin
                    if (cycle_pulse) begin
                        case (phase)
                            2'd0: sda_enable <= 1'b0; 
                            2'd1: scl_enable <= 1'b0;
                            2'd2: data_rd[bit_idx] <= sda_in;
                            2'd3: begin
                                scl_enable <= 1'b1; 
                                if (bit_idx == 3'd0) begin
                                    state <= STATE_MSTR_ACK;
                                end else begin
                                    bit_idx <= bit_idx - 1'b1;
                                end
                            end
                        endcase
                    end
                end

                STATE_MSTR_ACK: begin
                    if (cycle_pulse) begin
                        case (phase)
                            2'd0: sda_enable <= ~saved_stop;
                            2'd1: scl_enable <= 1'b0;
                            2'd2: scl_enable <= 1'b0;
                            2'd3: begin
                                scl_enable <= 1'b1;
                                if (saved_stop) begin
                                    bus_active <= 1'b0;
                                    state      <= STATE_STOP;
                                end else begin
                                    bus_active <= 1'b1;
                                    state      <= STATE_IDLE;
                                end
                            end
                        endcase
                    end
                end

                STATE_STOP: begin
                    if (cycle_pulse) begin
                        case (phase)
                            2'd0: begin sda_enable <= 1'b1; scl_enable <= 1'b1; end
                            2'd1: begin sda_enable <= 1'b1; scl_enable <= 1'b0; end 
                            2'd2: begin sda_enable <= 1'b0; scl_enable <= 1'b0; end 
                            2'd3: begin
                                busy  <= 1'b0;
                                state <= STATE_IDLE;
                                bus_active <= 1'b0;
                            end
                        endcase
                    end
                end

                default: begin
                    state      <= STATE_IDLE;
                    busy       <= 1'b0;
                    scl_enable <= 1'b0;
                    sda_enable <= 1'b0;
                    bus_active <= 1'b0;
                end
            endcase
        end
    end

endmodule
