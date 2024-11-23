// holbroow6502.h
#ifndef HOLBROOW6502_H
#define HOLBROOW6502_H

#include "Bus.h"
#include <stdint.h>
#include <stdbool.h>

// Status flags
#define FLAG_CARRY      0x01
#define FLAG_ZERO       0x02
#define FLAG_INTERRUPT  0x04
#define FLAG_DECIMAL    0x08
#define FLAG_BREAK      0x10
#define FLAG_UNUSED     0x20
#define FLAG_OVERFLOW   0x40
#define FLAG_NEGATIVE   0x80

// CPU Structure
typedef struct {
    // CPU Registers
    uint8_t A;
    uint8_t X;
    uint8_t Y;
    uint8_t SP;
    uint16_t PC;
    uint8_t STATUS;

    // Bus
    Bus* bus;   // Reference to the bus

    // Is the CPU running?
    bool running;
} Cpu;

// Enums for instructions and addressing modes
typedef enum {
    // Load/Store Operations
    LDA,
    LDX,
    LDY,
    STA,
    STX,
    STY,
    // Register Transfers
    TAX,
    TAY,
    TXA,
    TYA,
    // Stack Operations
    TSX,
    TXS,
    PHA,
    PHP,
    PLA,
    PLP,
    // Logical Instructions
    AND,
    EOR,
    ORA,
    BIT,
    // Arithmetic Instructions
    ADC,
    SBC,
    CMP,
    CPX,
    CPY,
    // Increments & Decrements
    INC,
    INX,
    INY,
    DEC,
    DEX,
    DEY,
    // Shifts
    ASL,
    LSR,
    ROL,
    ROR,
    // Jumps & Calls
    JMP,
    JSR,
    RTS,
    // Branches
    BCC,
    BCS,
    BEQ,
    BMI,
    BNE,
    BPL,
    BVC,
    BVS,
    // Status Flag Changes
    CLC,
    CLD,
    CLI,
    CLV,
    SEC,
    SED,
    SEI,
    // System Functions
    BRK,
    NOP,
    RTI,
} Instruction;

typedef enum {
    IMMEDIATE,
    ZERO_PAGE,
    ZERO_PAGE_X,
    ZERO_PAGE_Y,
    ABSOLUTE,
    ABSOLUTE_X,
    ABSOLUTE_Y,
    INDIRECT,
    INDEXED_INDIRECT,    // (Indirect,X)
    INDIRECT_INDEXED,    // (Indirect),Y
    RELATIVE,
    ACCUMULATOR,
    IMPLIED,
} AddressingMode;

// Opcode structure
typedef struct {
    Instruction instruction;
    AddressingMode addressing_mode;
    uint8_t bytes;
    uint8_t cycles;
} Opcode;

// Declare the opcode_table as extern
extern const Opcode opcode_table[256];

// Function to initialize the CPU
Cpu* init_cpu(Bus* bus);

// Function to run the CPU
void run_cpu(Cpu* cpu);

// Declare all handle_* functions
void handle_LDA(Cpu* cpu, uint8_t opcode);
void handle_LDX(Cpu* cpu, uint8_t opcode);
void handle_LDY(Cpu* cpu, uint8_t opcode);
void handle_STA(Cpu* cpu, uint8_t opcode);
void handle_STX(Cpu* cpu, uint8_t opcode);
void handle_STY(Cpu* cpu, uint8_t opcode);
void handle_TAX(Cpu* cpu, uint8_t opcode);
void handle_TAY(Cpu* cpu, uint8_t opcode);
void handle_TXA(Cpu* cpu, uint8_t opcode);
void handle_TYA(Cpu* cpu, uint8_t opcode);
void handle_TSX(Cpu* cpu, uint8_t opcode);
void handle_TXS(Cpu* cpu, uint8_t opcode);
void handle_PHA(Cpu* cpu, uint8_t opcode);
void handle_PHP(Cpu* cpu, uint8_t opcode);
void handle_PLA(Cpu* cpu, uint8_t opcode);
void handle_PLP(Cpu* cpu, uint8_t opcode);
void handle_AND(Cpu* cpu, uint8_t opcode);
void handle_EOR(Cpu* cpu, uint8_t opcode);
void handle_ORA(Cpu* cpu, uint8_t opcode);
void handle_BIT(Cpu* cpu, uint8_t opcode);
void handle_ADC(Cpu* cpu, uint8_t opcode);
void handle_SBC(Cpu* cpu, uint8_t opcode);
void handle_CMP(Cpu* cpu, uint8_t opcode);
void handle_CPX(Cpu* cpu, uint8_t opcode);
void handle_CPY(Cpu* cpu, uint8_t opcode);
void handle_INC(Cpu* cpu, uint8_t opcode);
void handle_INX(Cpu* cpu, uint8_t opcode);
void handle_INY(Cpu* cpu, uint8_t opcode);
void handle_DEC(Cpu* cpu, uint8_t opcode);
void handle_DEX(Cpu* cpu, uint8_t opcode);
void handle_DEY(Cpu* cpu, uint8_t opcode);
void handle_ASL(Cpu* cpu, uint8_t opcode);
void handle_LSR(Cpu* cpu, uint8_t opcode);
void handle_ROL(Cpu* cpu, uint8_t opcode);
void handle_ROR(Cpu* cpu, uint8_t opcode);
void handle_JMP(Cpu* cpu, uint8_t opcode);
void handle_JSR(Cpu* cpu, uint8_t opcode);
void handle_RTS(Cpu* cpu, uint8_t opcode);
void handle_BCC(Cpu* cpu, uint8_t opcode);
void handle_BCS(Cpu* cpu, uint8_t opcode);
void handle_BEQ(Cpu* cpu, uint8_t opcode);
void handle_BMI(Cpu* cpu, uint8_t opcode);
void handle_BNE(Cpu* cpu, uint8_t opcode);
void handle_BPL(Cpu* cpu, uint8_t opcode);
void handle_BVC(Cpu* cpu, uint8_t opcode);
void handle_BVS(Cpu* cpu, uint8_t opcode);
void handle_CLC(Cpu* cpu, uint8_t opcode);
void handle_CLD(Cpu* cpu, uint8_t opcode);
void handle_CLI(Cpu* cpu, uint8_t opcode);
void handle_CLV(Cpu* cpu, uint8_t opcode);
void handle_SEC(Cpu* cpu, uint8_t opcode);
void handle_SED(Cpu* cpu, uint8_t opcode);
void handle_SEI(Cpu* cpu, uint8_t opcode);
void handle_BRK(Cpu* cpu, uint8_t opcode);
void handle_NOP(Cpu* cpu, uint8_t opcode);
void handle_RTI(Cpu* cpu, uint8_t opcode);

// Helper functions
uint8_t fetch_operand(Cpu* cpu, AddressingMode mode, uint16_t* address);
void set_zero_flag(Cpu* cpu, uint8_t value);
void set_negative_flag(Cpu* cpu, uint8_t value);
void set_carry_flag(Cpu* cpu, bool set);
void set_overflow_flag(Cpu* cpu, bool set);
void set_interrupt_flag(Cpu* cpu, bool set);
void push_stack(Cpu* cpu, uint8_t value);
uint8_t pull_stack(Cpu* cpu);

#endif // HOLBROOW6502_H
