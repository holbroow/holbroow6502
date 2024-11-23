// Will Holbrook | Lancaster University Third Year Project 2024 (SCC 300: EmuPC)
// Main.c (Loads a (currently hard coded) program into memory and calls the 6502 CPU to 
// run from that address, hence running the stored program.)

#include "Bus.h"
#include "holbroow6502.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

// Main function
int main() {
    // Initialize Bus
    Bus* bus = init_bus();

    // Initialize CPU
    Cpu* cpu = init_cpu(bus);

    // Program code to load:    Multiply 10 by 3

    // LDX #10
    bus_write(bus, 0x8000, 0xA2); // Opcode for LDX Immediate
    bus_write(bus, 0x8001, 0x0A); // Operand: #10
    // STX $0000
    bus_write(bus, 0x8002, 0x8E); // Opcode for STX Absolute
    bus_write(bus, 0x8003, 0x00); // Low byte of address
    bus_write(bus, 0x8004, 0x00); // High byte of address
    // LDX #3
    bus_write(bus, 0x8005, 0xA2); // Opcode for LDX Immediate
    bus_write(bus, 0x8006, 0x03); // Operand: #3
    // STX $0001
    bus_write(bus, 0x8007, 0x8E); // Opcode for STX Absolute
    bus_write(bus, 0x8008, 0x01); // Low byte of address
    bus_write(bus, 0x8009, 0x00); // High byte of address
    // LDY $0000
    bus_write(bus, 0x800A, 0xAC); // Opcode for LDY Absolute
    bus_write(bus, 0x800B, 0x00); // Low byte of address
    bus_write(bus, 0x800C, 0x00); // High byte of address
    // LDA #0
    bus_write(bus, 0x800D, 0xA9); // Opcode for LDA Immediate
    bus_write(bus, 0x800E, 0x00); // Operand: #0
    // CLC
    bus_write(bus, 0x800F, 0x18); // Opcode for CLC
    // ADC $0001
    bus_write(bus, 0x8010, 0x65); // Opcode for ADC Zero Page
    bus_write(bus, 0x8011, 0x01); // Operand: $0001 (Zero Page Address)
    // DEY
    bus_write(bus, 0x8012, 0x88); // Opcode for DEY
    // BNE loop
    bus_write(bus, 0x8013, 0xD0); // Opcode for BNE Relative
    bus_write(bus, 0x8014, 0xFB); // Operand: Relative offset -5 (0xFB)
    // STA $0002
    bus_write(bus, 0x8015, 0x85); // Opcode for STA Zero Page
    bus_write(bus, 0x8016, 0x02); // Operand: $0002 (Zero Page Address)
    // NOP
    bus_write(bus, 0x8017, 0xEA); // Opcode for NOP
    // NOP
    bus_write(bus, 0x8018, 0xEA); // Opcode for NOP
    // NOP
    bus_write(bus, 0x8019, 0xEA); // Opcode for NOP

    // Set PC to start of program
    cpu->PC = 0x8000;

    // Run the CPU
    run_cpu(cpu);

    // Cleanup
    free(cpu);
    free(bus);
    // Add any additional cleanup for the Bus if necessary

    return 0;
}
