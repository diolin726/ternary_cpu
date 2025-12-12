`timescale 1ns/1ps
`include "top_cpu.v"
module top_cpu_tb;
    parameter p = 2'b10, n = 2'b00, o = 2'b01;
    reg [1:0] rst_n;
    reg clk;
    reg [32768  -1 : 0  ] code; //1024行
    top_cpu tb (
        .input_code(code),
        .clk(clk),
        .rst_n(rst_n)
        );
    integer i;
    always #5 clk = ~clk;
    initial begin
        $dumpfile("wave.vcd");   // 產生波形檔
        $dumpvars(2, top_cpu_tb);
        rst_n = n;
        clk = 0 ;
        code = {
        {1014{32'b0}},
        32'b00000000101000000000010000010011,
        32'b00000000000100000000001100010011,
        32'b00000000110000000000110111101111,
        32'b00000000000000000000000000010011,
        32'b00000000000000000000000000010011,
        32'b00000000010101000000100001100011,
        32'b00000000000000000000000000010011,
        32'b00000000011000101000001010110011,
        32'b00000000100011011000111001100111,
        32'b00000001110000000010000000100011
        };
        rst_n = p;
        #200;

        $finish;
    end
endmodule