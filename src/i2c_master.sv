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
    input  wire       stop,     

    output reg        busy,    
    output reg  [7:0] data_rd,      
    output reg        ack_error,   

    inout  wire       scl,
    inout  wire       sda
);

    localparam DIVIDER = CLK_FREQ / (I2C_FREQ * 4);

    localparam STATE_IDLE    = 4'd0;
    localparam STATE_START   = 4'd1;
    localparam STATE_ADDR    = 4'd2;
    localparam STATE_SLV_ACK = 4'd3;
    localparam STATE_WRITE   = 4'd4;
    localparam STATE_WR_ACK  = 4'd5;
    localparam STATE_READ    = 4'd6;
    localparam STATE_MSTR_ACK= 4'd7;
    localparam STATE_STOP    = 4'd8;

    reg [3:0]  state;
    reg [15:0] clk_cnt;
    reg [1:0]  phase;          
    reg [2:0]  bit_idx;        
    reg [7:0]  saved_addr_rw;
    reg [7:0]  saved_data;
    reg        saved_stop;
    
    reg scl_enable;
    reg sda_enable; 

    assign scl = scl_enable ? 1'b0 : 1'bz;
    assign sda = sda_enable ? 1'b0 : 1'bz;

    wire cycle_pulse = (clk_cnt == DIVIDER - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_cnt <= 16'd0;
            phase   <= 2'd0;
        end else if (busy) begin
            if (cycle_pulse) begin
                clk_cnt <= 16'd0;
                phase   <= phase + 1'b1;
            end else begin
                clk_cnt <= clk_cnt + 1'b1;
            end
        end else begin
            clk_cnt <= 16'd0;
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
        end else begin
            case (state)
                STATE_IDLE: begin
                    scl_enable <= 1'b0;
                    sda_enable <= 1'b0; 
                    if (ena) begin
                        busy          <= 1'b1;
                        saved_addr_rw <= {addr, rw};
                        saved_data    <= data_wr;
                        saved_stop    <= stop;
                        ack_error     <= 1'b0;
                        state         <= STATE_START;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                STATE_START: begin
                    if (cycle_pulse) begin
                        case (phase)
                            2'd0: begin sda_enable <= 1'b0; scl_enable <= 1'b0; end
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

                                if (sda == 1'b1) ack_error <= 1'b1;
                            end
                            2'd3: begin
                                scl_enable <= 1'b1;
                                bit_idx    <= 3'd7;
                                if (saved_addr_rw[0] == 1'b0)
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
                                if (sda == 1'b1) ack_error <= 1'b1;
                            end
                            2'd3: begin
                                scl_enable <= 1'b1;
                                if (saved_stop)
                                    state <= STATE_STOP;
                                else
                                    state <= STATE_IDLE;
                            end
                        endcase
                    end
                end

                STATE_READ: begin
                    if (cycle_pulse) begin
                        case (phase)
                            2'd0: sda_enable <= 1'b0; 
                            2'd1: scl_enable <= 1'b0;
                            2'd2: data_rd[bit_idx] <= sda; 
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
                            2'd0: sda_enable <= 1'b0; 
                            2'd1: scl_enable <= 1'b0;
                            2'd2: scl_enable <= 1'b0;
                            2'd3: begin
                                scl_enable <= 1'b1;
                                state      <= STATE_STOP;
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
                            end
                        endcase
                    end
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end

endmodule