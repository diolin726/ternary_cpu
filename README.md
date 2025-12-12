# Ternary CPU

A ternary (base-3) CPU implementation written in Verilog at the gate level.

## Overview

This project implements a ternary CPU using Verilog gate-level design. Unlike traditional binary computers that use two states (0 and 1), this ternary CPU operates with three states: negative (-), zero (0), and positive (+).

## Implementation Status

⚠️ **Work in Progress** - Many parts of this implementation still need modification and refinement.

## Ternary Encoding Approaches

There are three different approaches for mapping ternary voltage levels to binary representation in Verilog simulation:

### 1. Type 0: Binary Simulation Mode (Simple Approach)
- **Encoding**: `{input_bit, 1'b0}` - concatenates input bit with zero
- Uses 2-bit representation where the MSB carries the binary value
- **Status**: ✅ Currently implemented in most places
- This is the simplest approach and is used for Verilog simulation purposes only
- Example: binary 1 → `2'b10` (represents +), binary 0 → `2'b00` (represents -/0)

### 2. Type 1: Positive-Zero Mapping
- **Encoding**: Binary 1, 0 maps to Ternary +, 0
- Uses `isp` module: if input is + (p) output +, otherwise output 0 (o)
- **Status**: ⏳ Partially implemented but not used throughout the design

### 3. Type 2: Positive-Negative Mapping  
- **Encoding**: Direct pass-through assignment
- Relies on the assumption that binary representation directly maps to ternary states
- **Status**: ⏳ Partially implemented but not used throughout the design

### Current Situation

Due to time constraints, **Type 0** (the simplest simulation approach) is currently used in most parts of the codebase. The other two approaches, which would allow for proper voltage-level ternary representation, are planned for future implementation.

## Architecture

The CPU follows a classic pipeline architecture with the following stages:

- **IF (Instruction Fetch)**: Fetches instructions from ROM
- **ID (Instruction Decode)**: Decodes instructions and reads register file
- **EX (Execute)**: Executes ALU operations
- **MEM (Memory)**: Memory read/write operations
- **WB (Write Back)**: Writes results back to register file

## Key Components

### Ternary Logic Gates
- `terand`: Ternary AND gate
- `teror`: Ternary OR gate
- `onot`, `pnot`, `nnot`: Ternary NOT variants (balanced NOT gates)
- `ternand`, `ternor`: Ternary NAND and NOR gates
- `ternmul`: Ternary multiplication
- Additional ternary operators based on [ternary logic principles](https://louis-dr.github.io/ternlogic.html)

### Arithmetic Units
- `baladder`: Balanced ternary adder
- `baltit_to_bit`: Converts balanced ternary to binary
- `bit_to_tit_force`: Converts binary to ternary (supports multiple encoding modes)

### CPU Modules
- Program Counter (PC)
- Register File
- ALU (Arithmetic Logic Unit)
- Control Unit
- ROM (Read-Only Memory)

## Ternary Number System

This implementation uses **balanced ternary** representation where:
- `p = 2'b10` represents +1
- `o = 2'b01` represents 0
- `n = 2'b00` represents -1

In balanced ternary, numbers are represented as sums of powers of 3 with coefficients of -1, 0, or +1.

## Files

- `basic.v`: Basic ternary logic gates and arithmetic operations
- `cpu.v`: CPU pipeline stages (IF, ID, EX, MEM, WB) and control logic
- `top_cpu.v`: Top-level CPU module integrating all components
- `top_cpu_tb.v`: Testbench for the CPU
- `top_cpu_forcpp.v`: C++ simulation interface

## Future Work

- [ ] Implement Type 1 encoding (Positive-Zero mapping) throughout the design
- [ ] Implement Type 2 encoding (Positive-Negative mapping) throughout the design
- [ ] Complete modules marked with `[to_do]` comments
- [ ] Optimize gate-level implementation for better performance
- [ ] Add comprehensive test suite
- [ ] Improve documentation with circuit diagrams

## Requirements

- Verilog simulator (e.g., Icarus Verilog, Verilator, or ModelSim)
- For C++ simulation: C++ compiler with Verilator support

## Notes

Since Verilog simulation fundamentally operates on binary logic, this implementation simulates ternary behavior using binary representations. True ternary CPU operation would require actual ternary voltage level hardware.
