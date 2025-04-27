/* Copyright (c) 2025 Maveric NU. All rights reserved. */

// ----------------------------------------------------------------------------------------------------------------------------------
// This is a Verilog module that realizes the logic for configuring SSD1306 OLED display. It uses I2C for writing data.
// ----------------------------------------------------------------------------------------------------------------------------------
/* verilator lint_off WIDTH */

module oled_config 
#(
    parameter EXTERNAL_CLK_FRQ = 27000000, // For Tang Nano 9k boards: 27 MHz.
              I2C_CLK_FRQ     = 100000,    // For SSD1306 OLED.
              ADDR_WIDTH      = 7,
              DATA_WIDTH      = 8
)
(
    // Input interface.
    input  wire i_clk,
    input  wire i_arst,
    input  wire i_start,

    // I2C signals.
    inout  wire io_sda,
    output wire o_scl,

    // Output interface.
    output wire o_led_error
);

    wire s_arst;
    wire s_start;
    assign s_arst = ~i_arst;
    assign s_start = ~i_start;

    //-------------------------------------------
    // Internal nets.
    //-------------------------------------------
    
    // I2C signals.
    wire s_i2c_data_done;
    wire s_i2c_addr_done;
    wire s_i2c_ready;
    wire s_i2c_rw_failure;
    
    reg s_i2c_start;
    reg s_i2c_last;
    reg [DATA_WIDTH - 1:0] s_i2c_data;

    assign o_led_error = ~s_i2c_rw_failure;

    // Done signals.
    wire s_init_done;
    wire s_data_done;
    reg  s_data_st;

    // Counter.
    reg [4:0]  s_index;
    reg [9:0] s_count;

    //-------------------------------------------
    // FSM States
    //-------------------------------------------
    localparam IDLE        = 3'd0;
    localparam SEND_ADDR_I = 3'd1;
    localparam SEND_CMD_I  = 3'd2;
    localparam SEND_DATA_I = 3'd3;
    localparam SEND_ADDR_D = 3'd4;
    localparam SEND_CMD_D  = 3'd5;
    localparam SEND_DATA_D = 3'd6;

    //-------------------------------------------
    // Counter.
    //-------------------------------------------
    always @(posedge i_clk or posedge s_arst) begin
        if (s_arst) begin 
            s_index <= 5'b11111;
            s_count <= 10'b11_1111_1111;
        end
        else if (s_i2c_addr_done & (~s_data_st)) s_index <= s_index + 5'b1;
        else if (s_i2c_addr_done & (s_data_st))  s_count <= s_count + 10'b1;
    end

    assign s_init_done = s_index >= 22;
    assign s_data_done = s_count >= 1023;

    //------------------------------------------------------------------------------------
    // Sequence of instructions for initialization of OLED. 
    // Retrieved from Lushay Labs at https://learn.lushaylabs.com/tang-nano-9k-graphics/
    //------------------------------------------------------------------------------------
    localparam SETUP_INSTR_CNT = 23;
    reg [DATA_WIDTH - 1:0] mem_init_commands [SETUP_INSTR_CNT - 1:0];
    always @( posedge i_clk or posedge s_arst) begin
        if ( s_arst ) begin
            mem_init_commands[0]  <= 8'hAE;  // display off
            mem_init_commands[1]  <= 8'h81;  // contast value to 0x7F according to datasheet
            mem_init_commands[2]  <= 8'h7F;  
            mem_init_commands[3]  <= 8'hA6;  // normal screen mode (not inverted)
            mem_init_commands[4]  <= 8'h20;  // horizontal addressing mode
            mem_init_commands[5]  <= 8'h00;  
            mem_init_commands[6]  <= 8'hC8;  // normal scan direction
            mem_init_commands[7]  <= 8'h40;  // first line to start scanning from
            mem_init_commands[8]  <= 8'hA1;  // address 0 is segment 0
            mem_init_commands[9]  <= 8'hA8;  // mux ratio
            mem_init_commands[10] <= 8'h3f;  // 63 (64 -1)
            mem_init_commands[11] <= 8'hD3;  // display offset
            mem_init_commands[12] <= 8'h00;  // no offset
            mem_init_commands[13] <= 8'hD5;  // clock divide ratio
            mem_init_commands[14] <= 8'h80;  // set to default ratio/osc frequency
            mem_init_commands[15] <= 8'hD9;  // set precharge
            mem_init_commands[16] <= 8'h22;  // switch precharge to 0x22 default
            mem_init_commands[17] <= 8'hDB;  // vcom deselect level
            mem_init_commands[18] <= 8'h20;  // 0x20 
            mem_init_commands[19] <= 8'h8D;  // charge pump config
            mem_init_commands[20] <= 8'h14;  // enable charge pump
            mem_init_commands[21] <= 8'hA4;  // resume RAM content
            mem_init_commands[22] <= 8'hAF;  // display on
        end    
    end


    //--------------------------------------------
    // Register to save image.
    //--------------------------------------------
    reg [7:0] screenBuffer [1023:0];
    initial $readmemh("image.hex", screenBuffer);

    //--------------------------------------------
    // I2C module instance.
    //--------------------------------------------
    i2c_write_master #( 
        .EXTERNAL_CLK_FRQ ( EXTERNAL_CLK_FRQ ),
        .I2C_CLK_FRQ     ( I2C_CLK_FRQ     ),
        .ADDR_WIDTH      ( ADDR_WIDTH      ),
        .DATA_WIDTH      ( DATA_WIDTH      )
    ) I2C0 (
        .i_clk        ( i_clk            ),
        .i_arst       ( s_arst           ),
        .i_start      ( s_i2c_start      ),
        .i_last       ( s_i2c_last       ),
        .i_rw_request ( 1'b0             ), // Only write.
        .i_addr       ( 7'h3C            ), // Write address according to the datasheet.
        .i_data       ( s_i2c_data       ),
        .io_sda       ( io_sda           ),
        .o_data_done  ( s_i2c_data_done  ),
        .o_addr_done  ( s_i2c_addr_done  ),
        .o_ready      ( s_i2c_ready      ),
        .o_scl        ( o_scl            ),
        .o_rw_failure ( s_i2c_rw_failure ) 
    );

    //----------------------------------------------
    // FSM Logic.
    //----------------------------------------------
    reg [2:0] PS, NS;

    // FSM: PS Synchronization.
    always @(posedge i_clk or posedge s_arst) begin
        if (s_arst) PS <= IDLE;
        else        PS <= NS;
    end

    // FSM: NS Logic.
    always @(*) begin
        // Default value.
        NS = PS;

        case (PS)
            IDLE:        if (s_start & s_i2c_ready) NS = SEND_ADDR_I;
            SEND_ADDR_I: if (s_i2c_addr_done) NS = SEND_CMD_I;
            SEND_CMD_I:  if (s_i2c_data_done) NS = SEND_DATA_I;
            SEND_DATA_I: if (s_i2c_data_done)
                            if (s_init_done) NS = SEND_ADDR_D;
                            else             NS = SEND_ADDR_I;
            SEND_ADDR_D: if (s_i2c_addr_done) NS = SEND_CMD_D;
            SEND_CMD_D:  if (s_i2c_data_done) NS = SEND_DATA_D;
            SEND_DATA_D: if (s_i2c_data_done)
                            if (s_data_done) NS = IDLE;
                            else             NS = SEND_ADDR_D;
            default: NS = PS;
        endcase
    end

    // FSM Output Logic.
    always @(*) begin
        // Default values.
        s_i2c_start = 0;
        s_i2c_last  = 0;
        s_i2c_data  = 0;
        s_data_st   = 0;

        case (PS)
            IDLE: if (s_start & s_i2c_ready) s_i2c_start = 1'b1;
            SEND_ADDR_I: if (s_i2c_ready) s_i2c_start = 1'b1;
            SEND_CMD_I:  s_i2c_data = 8'h00;
            SEND_DATA_I: begin
                s_i2c_last = 1'b1; 
                s_i2c_data = mem_init_commands[s_index];
            end
            SEND_ADDR_D: begin
                if (s_i2c_ready) s_i2c_start = 1'b1;
                s_data_st = 1'b1; 
            end
            SEND_CMD_D: begin 
                s_i2c_data = 8'h40;
                s_data_st  = 1'b1;
            end
            SEND_DATA_D: begin
                s_i2c_last = 1'b1;
                s_i2c_data = screenBuffer [ s_count [ 9:0 ] ];
                s_data_st  = 1'b1;
            end
            default: begin
                s_i2c_start = 0;
                s_i2c_last  = 0;
                s_i2c_data  = 0; 
                s_data_st   = 0; 
            end
        endcase
    end

/* verilator lint_off WIDTH */
endmodule