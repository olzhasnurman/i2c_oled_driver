/* Copyright (c) 2025 Maveric NU. All rights reserved. */

// ---------------------------------------------------------------------------------------------------------------------
// This is a Verilog module that realizes IIC/I2C protocol. This is the master module for write operation.
// ---------------------------------------------------------------------------------------------------------------------
/* verilator lint_off WIDTH */

module i2c_write_master 
#(
    parameter EXTERNAL_CLK_FRQ = 4000000, // For Tang Nano 9k boards: 27 MHz.
              I2C_CLK_FRQ     = 100000,   // For SSD1306 OLED.
              ADDR_WIDTH      = 7,
              DATA_WIDTH      = 8
) 
(
    // Input Interface.
    input  wire                      i_clk,
    input  wire                      i_arst,
    input  wire                      i_start,
    input  wire                      i_last,
    input  wire                      i_rw_request, // Read or Write request. 0 - write, 1 - read.
    input  wire [ADDR_WIDTH - 1:0]   i_addr,
    input  wire [DATA_WIDTH - 1:0]   i_data,
    
    // Inout.
    inout  wire                      io_sda,

    // Output Interface.
    output wire                      o_data_done,
    output wire                      o_addr_done,
    output wire                      o_ready,
    output wire                      o_scl,
    output wire                      o_rw_failure 
);

    //----------------------------------------------
    // Local parameters.
    //----------------------------------------------
    localparam COUNT_TO    = EXTERNAL_CLK_FRQ / (2 * I2C_CLK_FRQ);
    localparam COUNT_WIDTH = $clog2(COUNT_TO);
    localparam WAIT_CYCLES = 10; // Need to wait at least 250 ns. At 27 MHz clock frequency it is ~ 7 cycles.

    //----------------------------------------------
    // FSM States
    //----------------------------------------------
    localparam IDLE      = 4'd0;
    localparam START     = 4'd1;
    localparam WAIT_S    = 4'd2;
    localparam RST_CLK   = 4'd3;
    localparam ADDR      = 4'd4;
    localparam DATA      = 4'd5;
    localparam DATA_LAST = 4'd6;
    localparam WAIT_F1   = 4'd7;
    localparam WAIT_F2   = 4'd8;
    localparam WAIT_F3   = 4'd9;
    localparam FINISH    = 4'd10;
    localparam FAIL      = 4'd11;

    //----------------------------------------------
    // Internal Nets.
    //----------------------------------------------
    // I2C Signals.
    wire  s_sda_out_en;
    wire  s_sda_out;
    reg  s_scl_clk;

    // Data.
    wire s_data_bit;
    wire  s_addr_state;
    wire  s_transmission;
    reg  [DATA_WIDTH - 1:0] s_data_reg;

    wire  s_wait_done;
    wire  s_rst_clk;
    wire  s_wait_start;

    // Counter.
    reg [COUNT_WIDTH - 1:0]           s_count_clk;
    reg [$clog2(WAIT_CYCLES) - 1:0]   s_count_wait;
    reg [3:0]                         s_count_data;

    // State registers
    reg [3:0] PS, NS;
    
    //----------------------------------------------
    // Continious Assignments.
    //----------------------------------------------
    assign io_sda = s_sda_out_en ? s_sda_out : 1'bz;
    assign s_sda_out_en = ~(s_count_data == 4'd8 & (s_count_clk >= 2));
    assign s_data_bit = s_data_reg[DATA_WIDTH - 1];
    assign s_wait_done = (s_count_wait == WAIT_CYCLES);

    //----------------------------------------------
    // Clock division.
    //----------------------------------------------
    always @(posedge i_clk or posedge i_arst) begin
        if (i_arst) begin
            s_scl_clk   <= 1'b1;
            s_count_clk <= 0;
        end
        else if (i_start & (~s_transmission) | s_rst_clk) begin
            s_scl_clk   <= 1'b1;
            s_count_clk <= 0;
        end
        else if (s_count_clk == COUNT_TO - 1) begin
            s_scl_clk <= ~s_scl_clk;
            s_count_clk <= 0;
        end
        else s_count_clk <= s_count_clk + 1'b1;
    end

    //-----------------------------------------------
    // Wait timer.
    //-----------------------------------------------
    always @(posedge i_clk) begin
        if (~s_wait_start) s_count_wait <= 0;
        else               s_count_wait <= s_count_wait + 1'b1; 
    end

    //----------------------------------------------
    // Counter.
    //----------------------------------------------
    always @(negedge s_scl_clk or posedge i_arst) begin
        if (i_arst)                          s_count_data <= 4'b0;
        else if (i_start & (~s_transmission)) s_count_data <= 4'b0;
        else if (s_count_data == 4'd8)       s_count_data <= 4'b0;
        else if (s_transmission)             s_count_data <= s_count_data + 4'b1; 
    end

    //----------------------------------------------
    // Data scheduling.
    //----------------------------------------------
    always @(posedge i_clk or posedge i_arst) begin
        if (i_arst) s_data_reg <= 8'b0;
        else if (~s_scl_clk & (s_count_clk == 2)) begin
            if (s_addr_state) begin
                if (s_count_data == 4'b0) s_data_reg <= {i_addr, i_rw_request};
                else                      s_data_reg <= s_data_reg << 1; 
            end
            else begin
                if (s_count_data == 4'b0) s_data_reg <= i_data;
                else                      s_data_reg <= s_data_reg << 1;
            end
        end
    end

    //----------------------------------------------
    // FSM: PS Synchronization.
    //----------------------------------------------
    always @(posedge i_clk or posedge i_arst) begin
        if (i_arst) PS <= IDLE;
        else        PS <= NS;
    end

    //----------------------------------------------
    // FSM: NS Logic.
    //----------------------------------------------
    always @(*) begin
        NS = PS;

        case (PS)
            IDLE: if (i_start) NS = START;

            START: if (~s_scl_clk) NS = WAIT_S;

            WAIT_S: if (s_wait_done) NS = RST_CLK;

            RST_CLK: NS = ADDR;

            ADDR: if ((s_count_data == 4'd8) & (s_scl_clk) & (s_count_clk == COUNT_TO/2)) 
                     if (io_sda) NS = FAIL;
                     else        NS = DATA;

            DATA: if (i_last) NS = DATA_LAST;
                  else if ((s_count_data == 4'd8) & (s_scl_clk) & (s_count_clk == COUNT_TO/2)) 
                      if (io_sda) NS = FAIL;
                      else        NS = DATA;

            DATA_LAST: if ((s_count_data == 4'd8) & (s_scl_clk) & (s_count_clk == COUNT_TO/2)) 
                          if (io_sda) NS = FAIL;
                          else        NS = WAIT_F1;

            WAIT_F1: if (~s_scl_clk) NS = WAIT_F2;
            WAIT_F2: if (s_scl_clk)  NS = WAIT_F3;
            WAIT_F3: if (~s_scl_clk) NS = FINISH;
            FINISH: if (s_scl_clk)   NS = IDLE;
            FAIL:                     NS = IDLE;
            default: NS = PS;
        endcase
    end

    //----------------------------------------------
    // FSM: Output Logic.
    //----------------------------------------------
    reg s_sda_out_reg, o_scl_reg, o_rw_failure_reg, s_addr_state_reg;
    reg s_transmission_reg, s_wait_start_reg, s_rst_clk_reg;
    reg o_ready_reg, o_data_done_reg, o_addr_done_reg;

    always @(*) begin
        // Default values
        s_sda_out_reg      = 1'b1;
        o_scl_reg          = 1'b1;
        o_rw_failure_reg   = 1'b0;
        s_addr_state_reg   = 1'b0;
        s_transmission_reg = 1'b0;
        s_wait_start_reg   = 1'b0;
        s_rst_clk_reg      = 1'b0;
        o_ready_reg        = 1'b0;
        o_data_done_reg    = 1'b0;
        o_addr_done_reg    = 1'b0;

        case (PS)
            IDLE: begin
                s_sda_out_reg = 1'b1;
                o_scl_reg     = 1'b1;
                o_ready_reg   = 1'b1;
            end
            START: begin
                s_sda_out_reg    = 1'b0;
                o_scl_reg        = s_scl_clk;
                s_addr_state_reg = 1'b1;
            end
            WAIT_S: begin
                s_sda_out_reg    = 1'b0;
                o_scl_reg        = s_scl_clk;
                s_wait_start_reg = 1'b1;
                s_addr_state_reg = 1'b1;
            end
            RST_CLK: begin
                s_sda_out_reg    = 1'b0;
                o_scl_reg        = s_scl_clk;
                s_rst_clk_reg    = 1'b1;
                s_addr_state_reg = 1'b1;
            end
            ADDR: begin
                s_sda_out_reg      = s_data_bit;
                o_scl_reg          = s_scl_clk;
                s_addr_state_reg   = 1'b1;
                s_transmission_reg = 1'b1;
                o_addr_done_reg    = (s_count_data == 4'd8) & (s_scl_clk) & (s_count_clk == COUNT_TO/2);
            end 
            DATA: begin
                s_sda_out_reg      = s_data_bit;
                o_scl_reg          = s_scl_clk;
                s_transmission_reg = 1'b1;
                o_data_done_reg    = (s_count_data == 4'd8) & (s_scl_clk) & (s_count_clk == COUNT_TO/2);
            end
            DATA_LAST: begin
                s_sda_out_reg      = s_data_bit;
                o_scl_reg          = s_scl_clk;
                s_transmission_reg = 1'b1;
                o_data_done_reg    = (s_count_data == 4'd8) & (s_scl_clk) & (s_count_clk == COUNT_TO/2);
            end
            WAIT_F1: begin
                s_sda_out_reg = 1'b0;
                o_scl_reg     = s_scl_clk;
            end
            WAIT_F2: begin
                s_sda_out_reg = 1'b0;
                o_scl_reg     = 1'b0;
            end
            WAIT_F3: begin
                s_sda_out_reg = 1'b0;
                o_scl_reg     = 1'b1;
            end
            FINISH: begin
                s_sda_out_reg = 1'b1;
                o_scl_reg     = 1'b1;
            end
            FAIL: begin
                o_scl_reg        = s_scl_clk;
                o_rw_failure_reg = 1'b1;
            end
            default: begin
                s_sda_out_reg      = 1'b1;
                o_scl_reg          = 1'b1;
                o_rw_failure_reg   = 1'b0;
                s_addr_state_reg   = 1'b0;
                s_transmission_reg = 1'b0;
                s_wait_start_reg   = 1'b0;
                s_rst_clk_reg      = 1'b0;
                o_ready_reg        = 1'b0;
                o_data_done_reg    = 1'b0;
                o_addr_done_reg    = 1'b0;
            end
        endcase
    end

    // Assign the output registers to the output ports
    assign s_sda_out      = s_sda_out_reg;
    assign o_scl          = o_scl_reg;
    assign o_rw_failure   = o_rw_failure_reg;
    assign s_addr_state   = s_addr_state_reg;
    assign s_transmission = s_transmission_reg;
    assign s_wait_start   = s_wait_start_reg;
    assign s_rst_clk      = s_rst_clk_reg;
    assign o_ready        = o_ready_reg;
    assign o_data_done    = o_data_done_reg;
    assign o_addr_done    = o_addr_done_reg;

/* verilator lint_off WIDTH */    
endmodule