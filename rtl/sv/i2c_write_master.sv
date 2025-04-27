/* Copyright (c) 2025 Maveric NU. All rights reserved. */

// ---------------------------------------------------------------------------------------------------------------------
// This is a custom SystemVerilog module that realizes IIC/I2C protocol. This is the master module for write operation.
// ---------------------------------------------------------------------------------------------------------------------
/* verilator lint_off WIDTH */

module i2c_write_master 
#(
    parameter EXTERNAL_CLK_FRQ = 4000000, // For Tang Nano 9k boards: 27 MHz.
              I2C_CLK_FRQ      = 100000,   // For SSD1306 OLED.
              ADDR_WIDTH       = 7,
              DATA_WIDTH       = 8
) 
(
    // Input Interface.
    input  logic                      i_clk,
    input  logic                      i_arst,
    input  logic                      i_start,
    input  logic                      i_last,
    input  logic                      i_rw_request, // Read or Write request. 0 - write, 1 - read.
    input  logic [ ADDR_WIDTH - 1:0 ] i_addr,
    input  logic [ DATA_WIDTH - 1:0 ] i_data,
    
    // Inout.
    inout  logic                      io_sda,

    // Output Interface.
    output logic                      o_data_done,
    output logic                      o_addr_done,
    output logic                      o_ready,
    output logic                      o_scl,
    output logic                      o_rw_failure 
);

    //----------------------------------------------
    // Local parameters.
    //----------------------------------------------
    localparam COUNT_TO    = EXTERNAL_CLK_FRQ / ( 2 * I2C_CLK_FRQ );
    localparam COUNT_WIDTH = $clog2 ( COUNT_TO );

    localparam WAIT_CYCLES = 10; // Need to wait at least 250 ns. At 27 MHz clock frequency it is ~ 7 cycles. For safety took 10 cycles.


    //----------------------------------------------
    // Internal Nets.
    //----------------------------------------------

    // I2C Signals.
    logic s_sda_out_en;
    logic s_sda_out;
    logic s_scl_clk;

    // Data.
    logic s_data_bit;
    logic s_addr_state;
    logic s_transmission;
    logic [ DATA_WIDTH - 1:0 ] s_data_reg;

    logic s_wait_done;
    logic s_rst_clk;
    logic s_wait_start;

    // Counter.
    logic [ COUNT_WIDTH           - 1 : 0 ] s_count_clk;
    logic [ $clog2 (WAIT_CYCLES ) - 1 : 0 ] s_count_wait;
    logic [                         3 : 0 ] s_count_data;


    
    //----------------------------------------------
    // Continious Assignments.
    //----------------------------------------------
    assign io_sda   = s_sda_out_en ? s_sda_out : 1'bz;
    assign s_sda_out_en = ~ ( s_count_data == 4'd8 & ( s_count_clk >= 2 ) );



    //----------------------------------------------
    // Clock division.
    //----------------------------------------------
    always_ff @( posedge i_clk, posedge i_arst ) begin
        if ( i_arst ) begin
            s_scl_clk   <= 1'b1;
            s_count_clk <= '0;
        end
        else if ( i_start & ( ~ s_transmission ) | s_rst_clk ) begin
            s_scl_clk   <= 1'b1;
            s_count_clk <= '0;
        end
        else if ( s_count_clk == COUNT_TO - 1 ) begin
            s_scl_clk <= ~ s_scl_clk;
            s_count_clk <= '0;
        end
        else s_count_clk <= s_count_clk + { { ( COUNT_WIDTH - 1 ) { 1'b0 } }, 1'b1 };
    end



    //-----------------------------------------------
    // Wait timer.
    //-----------------------------------------------
    always_ff @( posedge i_clk ) begin
        if ( ~ s_wait_start ) s_count_wait <= '0;
        else                  s_count_wait <= s_count_wait + 4'b1; 
    end
    assign s_wait_done = ( s_count_wait == WAIT_CYCLES );



    //----------------------------------------------
    // Counter.
    //----------------------------------------------
    always_ff @( negedge s_scl_clk, posedge i_arst ) begin
        if      ( i_arst                         ) s_count_data <= 4'b0;
        else if ( i_start & ( ~ s_transmission ) ) s_count_data <= 4'b0;
        else if ( s_count_data == 4'd8           ) s_count_data <= 4'b0;
        else if ( s_transmission                 ) s_count_data <= s_count_data + 4'b1; 
    end



    //----------------------------------------------
    // Data scheduling.
    //----------------------------------------------
    always_ff @( posedge i_clk, posedge i_arst ) begin
        if ( i_arst           ) s_data_reg <= 8'b0;
        else if ( ~ s_scl_clk & ( s_count_clk == 2 ) ) begin
            if ( s_addr_state ) begin
                if ( s_count_data == 4'b0 ) s_data_reg <= { i_addr, i_rw_request};
                else                        s_data_reg <= s_data_reg << 1; 
            end
            else begin
                if ( s_count_data == 4'b0 ) s_data_reg <= i_data;
                else                        s_data_reg <= s_data_reg << 1;
            end
        end
    end

    assign s_data_bit = s_data_reg [ DATA_WIDTH - 1 ];



    //----------------------------------------------
    // FSM Logic.
    //----------------------------------------------

    // FSM: States.
    typedef enum logic [ 3:0 ]
    {
        IDLE      = 4'd0,
        START     = 4'd1,
        WAIT_S    = 4'd2,
        RST_CLK   = 4'd3,
        ADDR      = 4'd4,
        DATA      = 4'd5,
        DATA_LAST = 4'd6,
        WAIT_F1   = 4'd7,
        WAIT_F2   = 4'd8,
        WAIT_F3   = 4'd9,
        FINISH    = 4'd10,
        FAIL      = 4'd11
    } t_state;

    t_state PS;
    t_state NS;


    // FSM: PS Syncronization.
    always_ff @( posedge i_clk, posedge i_arst ) begin
        if ( i_arst ) PS <= IDLE;
        else          PS <= NS;
    end


    // FSM: NS Logic.
    always_comb begin
        NS = PS;


        case ( PS )
            IDLE: if ( i_start ) NS = START;

            START: if ( ~ s_scl_clk ) NS = WAIT_S;

            WAIT_S: if ( s_wait_done ) NS = RST_CLK;

            RST_CLK: NS = ADDR;

            ADDR: if ( ( s_count_data == 4'd8 ) & ( s_scl_clk ) & ( s_count_clk == COUNT_TO/2 ) ) 
                      if ( io_sda ) NS = FAIL;
                      else          NS = DATA;

            DATA: if ( i_last ) NS = DATA_LAST;
                  else if ( ( s_count_data == 4'd8 ) & ( s_scl_clk ) & ( s_count_clk == COUNT_TO/2 ) ) 
                      if ( io_sda ) NS = FAIL;
                      else          NS = DATA;

            DATA_LAST: if ( ( s_count_data == 4'd8 ) & ( s_scl_clk ) & ( s_count_clk == COUNT_TO/2 ) ) 
                           if ( io_sda ) NS = FAIL;
                           else          NS = WAIT_F1;

            WAIT_F1: if ( ~ s_scl_clk ) NS = WAIT_F2;
            WAIT_F2: if ( s_scl_clk   ) NS = WAIT_F3;
            WAIT_F3: if ( ~s_scl_clk  ) NS = FINISH;
            FINISH : if ( s_scl_clk   ) NS = IDLE;
            FAIL   :                    NS = IDLE;  
            default : NS = PS;
        endcase
    end



    // FSM: Output Logic.
    always_comb begin
        // Default values.
        s_sda_out      = 1'b1;
        o_scl          = 1'b1;
        o_rw_failure   = 1'b0;
        s_addr_state   = 1'b0;
        s_transmission = 1'b0;
        s_wait_start   = 1'b0;
        s_rst_clk      = 1'b0;
        o_ready        = 1'b0;
        o_data_done    = 1'b0;
        o_addr_done    = 1'b0;

        case ( PS )
            IDLE: begin
                s_sda_out = 1'b1;
                o_scl     = 1'b1;
                o_ready   = 1'b1;
            end
            START: begin
                s_sda_out = 1'b0;
                o_scl     = s_scl_clk;
                s_addr_state   = 1'b1;
            end
            WAIT_S: begin
                s_sda_out        = 1'b0;
                o_scl            = s_scl_clk;
                s_wait_start     = 1'b1;
                s_addr_state     = 1'b1;
            end
            RST_CLK: begin
                s_sda_out        = 1'b0;
                o_scl            = s_scl_clk;
                s_rst_clk        = 1'b1;
                s_addr_state     = 1'b1;
            end
            ADDR: begin
                s_sda_out      = s_data_bit;
                o_scl          = s_scl_clk;
                s_addr_state   = 1'b1;
                s_transmission = 1'b1;
                o_addr_done    = ( s_count_data == 4'd8 ) & ( s_scl_clk ) & ( s_count_clk == COUNT_TO/2 );
            end 
            DATA: begin
                s_sda_out      = s_data_bit;
                o_scl          = s_scl_clk;
                s_transmission = 1'b1;
                o_data_done    = ( s_count_data == 4'd8 ) & ( s_scl_clk ) & ( s_count_clk == COUNT_TO/2 );
            end
            DATA_LAST: begin
                s_sda_out      = s_data_bit;
                o_scl          = s_scl_clk;
                s_transmission = 1'b1;
                o_data_done    = ( s_count_data == 4'd8 ) & ( s_scl_clk ) & ( s_count_clk == COUNT_TO/2 );
            end
            WAIT_F1: begin
                s_sda_out      = 1'b0;
                o_scl          = s_scl_clk;
            end
            WAIT_F2: begin
                s_sda_out      = 1'b0;
                o_scl          = 1'b0;
            end
            WAIT_F3: begin
                s_sda_out = 1'b0;
                o_scl     = 1'b1;
            end
            FINISH: begin
                s_sda_out    = 1'b1;
                o_scl        = 1'b1;
            end
            FAIL: begin
                o_scl        = s_scl_clk;
                o_rw_failure = 1'b1;
            end
            default: begin
                s_sda_out      = 1'b1;
                o_scl          = 1'b1;
                o_rw_failure   = 1'b0;
                s_addr_state   = 1'b0;
                s_transmission = 1'b0;
                s_wait_start   = 1'b0;
                s_rst_clk      = 1'b0;
                o_ready        = 1'b0;
                o_data_done    = 1'b0;
            end
        endcase
    end

/* verilator lint_off WIDTH */    
endmodule