# Ternary CPU

A ternary (base-3) CPU implementation written in Verilog at the gate level.

## Overview

This project implements a ternary CPU using Verilog gate-level design. Unlike traditional binary computers that use two states (0 and 1), this ternary CPU operates with three states: negative (-), zero (0), and positive (+).

## Implementation Status

⚠️ **Work in Progress** - Many parts of this implementation still need modification and refinement.

## Ternary Encoding Approaches

There are three different approaches for mapping ternary voltage levels to binary representation in Verilog simulation:

### 1. Type 0: Binary Simulation Mode (Currently Implemented)
- **Encoding**: `{input_bit, 1'b0}` - concatenates single-bit binary input with zero
- **Result**: Binary 1 → `2'b10` (p = +1), Binary 0 → `2'b00` (n = -1)
- **Status**: ✅ Currently used throughout the codebase
- **Note**: This simulates ternary using binary values. It doesn't represent true ternary voltage levels, but allows Verilog simulation to work with 2-bit representations of ternary states.

### 2. Type 1: Positive-Zero Voltage Mapping (Planned)
- **Intended Use**: For actual ternary hardware with voltage levels
- **Mapping**: Binary 1 → Ternary + voltage, Binary 0 → Ternary 0 voltage
- **Implementation**: Would use `isp` module for conversion
- **Status**: ⏳ Module infrastructure exists (`bit_to_tit_force` with `is_ter_make=1`) but not deployed

### 3. Type 2: Positive-Negative Voltage Mapping (Planned)
- **Intended Use**: For actual ternary hardware with voltage levels  
- **Mapping**: Binary 1 → Ternary + voltage, Binary 0 → Ternary - voltage
- **Implementation**: Would use direct assignment for conversion
- **Status**: ⏳ Module infrastructure exists (`bit_to_tit_force` with `is_ter_make=2`) but not deployed

### Current Situation

Due to time constraints, **Type 0** (the binary simulation approach) is currently used throughout the codebase. Types 1 and 2 would be needed for interfacing with actual ternary voltage-level hardware, where different voltage ranges represent ternary states. The infrastructure for these types exists in the `bit_to_tit_force` module but is not currently utilized. Future work will implement these to support real ternary hardware implementations.

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
- `n = 2'b00` represents -1 (negative)
- `o = 2'b01` represents 0 (zero)
- `p = 2'b10` represents +1 (positive)

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
