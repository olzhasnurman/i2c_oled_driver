#include <stdlib.h>
#include <iostream>
#include <cstdlib>
#include <memory>
#include <random>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vi2c_write_master.h"

#define MAX_SIM_TIME 10000
vluint64_t sim_time = 0;
vluint64_t posedge_cnt = 0;


void dut_reset (Vi2c_write_master *dut, vluint64_t &sim_time){
    
    if( sim_time < 100 ){
        dut->i_arst = 1;
    }
    else {
        dut->i_arst = 0;
    }
}

int main(int argc, char** argv, char** env) {
    Verilated::commandArgs(argc, argv);
    Vi2c_write_master *dut = new Vi2c_write_master;
    Verilated::traceEverOn(true);
    VerilatedVcdC* sim_trace = new VerilatedVcdC;
    dut->trace(sim_trace, 10);
    sim_trace->open("./waveform.vcd");

    // Setup random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint16_t> dist(0, 255);

    uint8_t data = static_cast<uint8_t>(dist(gen));;
    uint8_t data_count = 0;

    while (sim_time < MAX_SIM_TIME) {
        dut_reset(dut, sim_time);
        dut->i_clk ^= 1;
        if ( sim_time == 120 || sim_time == 121 ) dut->i_start = 1;
        else dut->i_start = 0;
        dut->i_rw_request = 1;
        dut->i_addr = 7;
        dut->i_data = data;
        if ( (dut->o_data_done) &  (dut->i_clk)) {
            data = static_cast<uint8_t>(dist(gen));
            data_count++;
        }

        if ( data_count > 5 ) dut->i_last = 1;
        else dut->i_last = 0;

        dut->eval();

        if (dut->i_clk == 1){
            posedge_cnt++;
        }

        sim_trace->dump(sim_time);
        sim_time++;
    }

    sim_trace->close();
    delete sim_trace;
    delete dut;
    exit(EXIT_SUCCESS);
}
