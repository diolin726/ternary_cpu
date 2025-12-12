#include "Vtop_cpu.h"
#include "verilated.h"
#include "verilated_vcd_c.h"
#include <cstdint>
#include <cstring>

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);

    Vtop_cpu* top = new Vtop_cpu;

    VerilatedVcdC* tfp = new VerilatedVcdC;
    top->trace(tfp, 99);
    tfp->open("wave.vcd");

    // constants
    const int WORDS = 32768 / 32; // 1024
    // prepare code words: fill zeros then place the 11 instructions at the end
    uint32_t code_words[WORDS];
    std::memset(code_words, 0, sizeof(code_words));
    const uint32_t instr[8] = {
        0x00a00413,
        0x00100313,
        0x00400def,
        0x00540663,
        0x006282b3,
        0x000d8e67,
        0x01c02023,
        0x00002483
    };
    
    // Map so that top->input_code[0] == instr[10] (LSB), top->input_code[1] == instr[9], ...
    // zero the whole array first (if needed), then fill the first 11 words in reverse order:
    for (int i = 0; i < WORDS; ++i) top->input_code[i] = 0;
    for (int i = 0; i < 8; ++i) {
        top->input_code[i] = instr[i];
    }

    // reset values: p = 2'b10, n = 2'b00
    top->rst_n = 0; // n
    top->clk = 0;
    top->eval();
    tfp->dump(0);

    // simulate: release reset after a few half-cycles, then run for a while
    vluint64_t time = 1;
    const vluint64_t max_time = 400; // adjust as needed
    // hold reset low for a couple of half-cycles
    top->clk = 1; top->rst_n = 0; top->eval(); tfp->dump(time++); 
    top->clk = 0; top->rst_n = 0; top->eval(); tfp->dump(time++); 
    top->clk = 1; top->rst_n = 2; top->eval(); tfp->dump(time++); 
    // p (2'b10) -> release reset

    // main simulation loop
    while (time < max_time && !Verilated::gotFinish()) {
        top->clk = 0; top->eval(); tfp->dump(time++);
        top->clk = 1; top->eval(); tfp->dump(time++);
    }

    // finish
    tfp->close();
    top->final();
    delete top;
    delete tfp;
    return 0;
}