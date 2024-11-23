// Will Holbrook | Lancaster University Third Year Project 2024 (SCC 300: EmuPC)
// holbroow6502.c (Cpu)

// Great resource, this file is mostly inspired from this datasheet.
// https://www.nesdev.org/obelisk-6502-guide

#include <Bus.c>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

typedef struct Bus Bus;

Bus* init_bus();
void write(Bus* bus, uint16_t address, uint8_t data);
uint8_t read(Bus* bus, uint16_t address);

// Status flags
#define FLAG_CARRY      0x00000001
#define FLAG_ZERO       0x00000010
#define FLAG_INTERRUPT  0x00000100
#define FLAG_DECIMAL    0x00001000
#define FLAG_BREAK      0x00010000
#define FLAG_UNUSED     0x00100000
#define FLAG_OVERFLOW   0x01000000
#define FLAG_NEGATIVE   0x10000000

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
    Bus* bus;   // reference to the bus

    // Is the CPU running???
    bool running;
} Cpu;

// Opcode Structure
typedef struct {
    Instruction instruction;
    AddressingMode addressing_mode;
    uint8_t bytes;
    uint8_t cycles;
} Opcode;

// All 52 Instructions enumerated
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

// Addressing Modes enumerated
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

// Opcode Lookup Table
static const Opcode opcode_table[256] = {
    // Initialize all opcodes to NOP with Implied addressing
    // This ensures that any unspecified opcode defaults to NOP
    [0 ... 255] = {NOP, IMPLIED, 1, 2},
    
    //// Load/Store Operations
    // LDA
    [0xA9] = {LDA, IMMEDIATE, 2, 2},
    [0xA5] = {LDA, ZERO_PAGE, 2, 3},
    [0xB5] = {LDA, ZERO_PAGE_X, 2, 4},
    [0xAD] = {LDA, ABSOLUTE, 3, 4},
    [0xBD] = {LDA, ABSOLUTE_X, 3, 4}, // +1 cycle if page crossed
    [0xB9] = {LDA, ABSOLUTE_Y, 3, 4}, // +1 cycle if page crossed
    [0xA1] = {LDA, INDEXED_INDIRECT, 2, 6},
    [0xB1] = {LDA, INDIRECT_INDEXED, 2, 5}, // +1 cycle if page crossed

    // LDX
    [0xA2] = {LDX, IMMEDIATE, 2, 2},
    [0xA6] = {LDX, ZERO_PAGE, 2, 3},
    [0xB6] = {LDX, ZERO_PAGE_Y, 2, 4},
    [0xAE] = {LDX, ABSOLUTE, 3, 4},
    [0xBE] = {LDX, ABSOLUTE_Y, 3, 4}, // +1 cycle if page crossed

    // LDY
    [0xA0] = {LDY, IMMEDIATE, 2, 2},
    [0xA4] = {LDY, ZERO_PAGE, 2, 3},
    [0xB4] = {LDY, ZERO_PAGE_X, 2, 4},
    [0xAC] = {LDY, ABSOLUTE, 3, 4},
    [0xBC] = {LDY, ABSOLUTE_X, 3, 4}, // +1 cycle if page crossed

    // STA
    [0x85] = {STA, ZERO_PAGE, 2, 3},
    [0x95] = {STA, ZERO_PAGE_X, 2, 4},
    [0x8D] = {STA, ABSOLUTE, 3, 4},
    [0x9D] = {STA, ABSOLUTE_X, 3, 5},
    [0x99] = {STA, ABSOLUTE_Y, 3, 5},
    [0x81] = {STA, INDEXED_INDIRECT, 2, 6},
    [0x91] = {STA, INDIRECT_INDEXED, 2, 6},

    // STX
    [0x86] = {STX, ZERO_PAGE, 2, 3},
    [0x96] = {STX, ZERO_PAGE_Y, 2, 4},
    [0x8E] = {STX, ABSOLUTE, 3, 4},

    // STY
    [0x84] = {STY, ZERO_PAGE, 2, 3},
    [0x94] = {STY, ZERO_PAGE_X, 2, 4},
    [0x8C] = {STY, ABSOLUTE, 3, 4},

    //// Register Transfers
    // TAX
    [0xAA] = {TAX, IMPLIED, 1, 2},
    // TAY
    [0xA8] = {TAY, IMPLIED, 1, 2},
    // TXA
    [0x8A] = {TXA, IMPLIED, 1, 2},
    // TYA
    [0x98] = {TYA, IMPLIED, 1, 2},

    //// Stack Operations
    // TSX
    [0xBA] = {TSX, IMPLIED, 1, 2},
    // TXS
    [0x9A] = {TXS, IMPLIED, 1, 2},
    // PHA
    [0x48] = {PHA, IMPLIED, 1, 3},
    // PHP
    [0x08] = {PHP, IMPLIED, 1, 3},
    // PLA
    [0x68] = {PLA, IMPLIED, 1, 4},
    // PLP
    [0x28] = {PLP, IMPLIED, 1, 4},

    //// Logical Instructions
    // AND
    [0x29] = {AND, IMMEDIATE, 2, 2},
    [0x25] = {AND, ZERO_PAGE, 2, 3},
    [0x35] = {AND, ZERO_PAGE_X, 2, 4},
    [0x2D] = {AND, ABSOLUTE, 3, 4},
    [0x3D] = {AND, ABSOLUTE_X, 3, 4}, // +1 cycle if page crossed
    [0x39] = {AND, ABSOLUTE_Y, 3, 4}, // +1 cycle if page crossed
    [0x21] = {AND, INDEXED_INDIRECT, 2, 6},
    [0x31] = {AND, INDIRECT_INDEXED, 2, 5}, // +1 cycle if page crossed

    // EOR
    [0x49] = {EOR, IMMEDIATE, 2, 2},
    [0x45] = {EOR, ZERO_PAGE, 2, 3},
    [0x55] = {EOR, ZERO_PAGE_X, 2, 4},
    [0x4D] = {EOR, ABSOLUTE, 3, 4},
    [0x5D] = {EOR, ABSOLUTE_X, 3, 4}, // +1 cycle if page crossed
    [0x59] = {EOR, ABSOLUTE_Y, 3, 4}, // +1 cycle if page crossed
    [0x41] = {EOR, INDEXED_INDIRECT, 2, 6},
    [0x51] = {EOR, INDIRECT_INDEXED, 2, 5}, // +1 cycle if page crossed

    // ORA
    [0x09] = {ORA, IMMEDIATE, 2, 2},
    [0x05] = {ORA, ZERO_PAGE, 2, 3},
    [0x15] = {ORA, ZERO_PAGE_X, 2, 4},
    [0x0D] = {ORA, ABSOLUTE, 3, 4},
    [0x1D] = {ORA, ABSOLUTE_X, 3, 4}, // +1 cycle if page crossed
    [0x19] = {ORA, ABSOLUTE_Y, 3, 4}, // +1 cycle if page crossed
    [0x01] = {ORA, INDEXED_INDIRECT, 2, 6},
    [0x11] = {ORA, INDIRECT_INDEXED, 2, 5}, // +1 cycle if page crossed

    // BIT
    [0x24] = {BIT, ZERO_PAGE, 2, 3},
    [0x2C] = {BIT, ABSOLUTE, 3, 4},

    //// Arithmetic Instructions
    // ADC
    [0x69] = {ADC, IMMEDIATE, 2, 2},
    [0x65] = {ADC, ZERO_PAGE, 2, 3},
    [0x75] = {ADC, ZERO_PAGE_X, 2, 4},
    [0x6D] = {ADC, ABSOLUTE, 3, 4},
    [0x7D] = {ADC, ABSOLUTE_X, 3, 4}, // +1 cycle if page crossed
    [0x79] = {ADC, ABSOLUTE_Y, 3, 4}, // +1 cycle if page crossed
    [0x61] = {ADC, INDEXED_INDIRECT, 2, 6},
    [0x71] = {ADC, INDIRECT_INDEXED, 2, 5}, // +1 cycle if page crossed

    // SBC
    [0xE9] = {SBC, IMMEDIATE, 2, 2},
    [0xE5] = {SBC, ZERO_PAGE, 2, 3},
    [0xF5] = {SBC, ZERO_PAGE_X, 2, 4},
    [0xED] = {SBC, ABSOLUTE, 3, 4},
    [0xFD] = {SBC, ABSOLUTE_X, 3, 4}, // +1 cycle if page crossed
    [0xF9] = {SBC, ABSOLUTE_Y, 3, 4}, // +1 cycle if page crossed
    [0xE1] = {SBC, INDEXED_INDIRECT, 2, 6},
    [0xF1] = {SBC, INDIRECT_INDEXED, 2, 5}, // +1 cycle if page crossed

    // CMP
    [0xC9] = {CMP, IMMEDIATE, 2, 2},
    [0xC5] = {CMP, ZERO_PAGE, 2, 3},
    [0xD5] = {CMP, ZERO_PAGE_X, 2, 4},
    [0xCD] = {CMP, ABSOLUTE, 3, 4},
    [0xDD] = {CMP, ABSOLUTE_X, 3, 4}, // +1 cycle if page crossed
    [0xD9] = {CMP, ABSOLUTE_Y, 3, 4}, // +1 cycle if page crossed
    [0xC1] = {CMP, INDEXED_INDIRECT, 2, 6},
    [0xD1] = {CMP, INDIRECT_INDEXED, 2, 5}, // +1 cycle if page crossed

    // CPX
    [0xE0] = {CPX, IMMEDIATE, 2, 2},
    [0xE4] = {CPX, ZERO_PAGE, 2, 3},
    [0xEC] = {CPX, ABSOLUTE, 3, 4},

    // CPY
    [0xC0] = {CPY, IMMEDIATE, 2, 2},
    [0xC4] = {CPY, ZERO_PAGE, 2, 3},
    [0xCC] = {CPY, ABSOLUTE, 3, 4},

    //// Increments & Decrements
    // INC
    [0xE6] = {INC, ZERO_PAGE, 2, 5},
    [0xF6] = {INC, ZERO_PAGE_X, 2, 6},
    [0xEE] = {INC, ABSOLUTE, 3, 6},
    [0xFE] = {INC, ABSOLUTE_X, 3, 7},

    // INX
    [0xE8] = {INX, IMPLIED, 1, 2},

    // INY
    [0xC8] = {INY, IMPLIED, 1, 2},

    // DEC
    [0xC6] = {DEC, ZERO_PAGE, 2, 5},
    [0xD6] = {DEC, ZERO_PAGE_X, 2, 6},
    [0xCE] = {DEC, ABSOLUTE, 3, 6},
    [0xDE] = {DEC, ABSOLUTE_X, 3, 7},

    // DEX
    [0xCA] = {DEX, IMPLIED, 1, 2},

    // DEY
    [0x88] = {DEY, IMPLIED, 1, 2},

    //// Shifts
    // ASL
    [0x0A] = {ASL, ACCUMULATOR, 1, 2},
    [0x06] = {ASL, ZERO_PAGE, 2, 5},
    [0x16] = {ASL, ZERO_PAGE_X, 2, 6},
    [0x0E] = {ASL, ABSOLUTE, 3, 6},
    [0x1E] = {ASL, ABSOLUTE_X, 3, 7},

    // LSR
    [0x4A] = {LSR, ACCUMULATOR, 1, 2},
    [0x46] = {LSR, ZERO_PAGE, 2, 5},
    [0x56] = {LSR, ZERO_PAGE_X, 2, 6},
    [0x4E] = {LSR, ABSOLUTE, 3, 6},
    [0x5E] = {LSR, ABSOLUTE_X, 3, 7},

    // ROL
    [0x2A] = {ROL, ACCUMULATOR, 1, 2},
    [0x26] = {ROL, ZERO_PAGE, 2, 5},
    [0x36] = {ROL, ZERO_PAGE_X, 2, 6},
    [0x2E] = {ROL, ABSOLUTE, 3, 6},
    [0x3E] = {ROL, ABSOLUTE_X, 3, 7},

    // ROR
    [0x6A] = {ROR, ACCUMULATOR, 1, 2},
    [0x66] = {ROR, ZERO_PAGE, 2, 5},
    [0x76] = {ROR, ZERO_PAGE_X, 2, 6},
    [0x6E] = {ROR, ABSOLUTE, 3, 6},
    [0x7E] = {ROR, ABSOLUTE_X, 3, 7},

    //// Jumps & Calls
    // JMP
    [0x4C] = {JMP, ABSOLUTE, 3, 3},
    [0x6C] = {JMP, INDIRECT, 3, 5},
    // JSR
    [0x20] = {JSR, ABSOLUTE, 3, 6},
    // RTS
    [0x60] = {RTS, IMPLIED, 1, 6},

    // Branches
    // BCC
    [0x90] = {BCC, RELATIVE, 2, 2}, 
    // BCS
    [0xB0] = {BCS, RELATIVE, 2, 2},
    // BEQ
    [0xF0] = {BEQ, RELATIVE, 2, 2},
    // BMI
    [0x30] = {BMI, RELATIVE, 2, 2},
    // BNE
    [0xD0] = {BNE, RELATIVE, 2, 2},
    // BPL
    [0x10] = {BPL, RELATIVE, 2, 2},
    // BVC
    [0x50] = {BVC, RELATIVE, 2, 2},
    // BVS
    [0x70] = {BVS, RELATIVE, 2, 2},

    //// Status Flag Changes
    // CLC
    [0x18] = {CLC, IMPLIED, 1, 2},
    // CLD
    [0xD8] = {CLD, IMPLIED, 1, 2},
    // CLI
    [0x58] = {CLI, IMPLIED, 1, 2},
    // CLV
    [0xB8] = {CLV, IMPLIED, 1, 2},
    // SEC
    [0x38] = {SEC, IMPLIED, 1, 2},
    // SED
    [0xF8] = {SED, IMPLIED, 1, 2},
    // SEI
    [0x78] = {SEI, IMPLIED, 1, 2},

    //// System Functions
    // BRK
    [0x00] = {BRK, IMPLIED, 1, 7},
    // NOP
    [0xEA] = {NOP, IMPLIED, 1, 2},
    // RTI
    [0x40] = {RTI, IMPLIED, 1, 6},

    // Additional Illegal Opcodes can (and hopefully will) be mapped to ILLEGAL or treated as NOP
};



//// Helper Functions
// Fetch operand based on addressing mode
uint8_t fetch_operand(Cpu* cpu, AddressingMode mode, uint16_t* address) {
    uint8_t value = 0;
    switch (mode) {
        case IMMEDIATE:
            value = read(cpu->bus, cpu->PC++);
            break;
        case ZERO_PAGE:
            *address = read(cpu->bus, cpu->PC++);
            value = read(cpu->bus, *address);
            break;
        case ZERO_PAGE_X:
            *address = (read(cpu->bus, cpu->PC++) + cpu->X) & 0xFF;
            value = read(cpu->bus, *address);
            break;
        case ZERO_PAGE_Y:
            *address = (read(cpu->bus, cpu->PC++) + cpu->Y) & 0xFF;
            value = read(cpu->bus, *address);
            break;
        case ABSOLUTE:
            *address = read(cpu->bus, cpu->PC++) | (read(cpu->bus, cpu->PC++) << 8);
            value = read(cpu->bus, *address);
            break;
        case ABSOLUTE_X:
            *address = (read(cpu->bus, cpu->PC++) | (read(cpu->bus, cpu->PC++) << 8)) + cpu->X;
            value = read(cpu->bus, *address);
            break;
        case ABSOLUTE_Y:
            *address = (read(cpu->bus, cpu->PC++) | (read(cpu->bus, cpu->PC++) << 8)) + cpu->Y;
            value = read(cpu->bus, *address);
            break;
        case INDEXED_INDIRECT:
            {
                uint8_t zp = read(cpu->bus, cpu->PC++);
                uint16_t addr = (zp + cpu->X) & 0xFF;
                *address = read(cpu->bus, addr) | (read(cpu->bus, (addr + 1) & 0xFF) << 8);
                value = read(cpu->bus, *address);
            }
            break;
        case INDIRECT_INDEXED:
            {
                uint8_t zp = read(cpu->bus, cpu->PC++);
                uint16_t addr = read(cpu->bus, zp) | (read(cpu->bus, (zp + 1) & 0xFF) << 8);
                *address = addr + cpu->Y;
                value = read(cpu->bus, *address);
            }
            break;
        case RELATIVE:
            {
                int8_t offset = (int8_t)read(cpu->bus, cpu->PC++);
                *address = cpu->PC + offset;
                // Note: Branch instructions handle the PC update
            }
            break;
        case ACCUMULATOR:
            // No operand to fetch
            break;
        case IMPLIED:
            // No operand to fetch
            break;
        default:
            // Handle unsupported addressing modes
            break;
    }
    return value;
}

// Set flags
// Zero Flag
void set_zero_flag(Cpu* cpu, uint8_t value) {
    if (value == 0) {
        cpu->STATUS |= FLAG_ZERO;
    } else {
        cpu->STATUS &= ~FLAG_ZERO;
    }
}

// Negative Flag
void set_negative_flag(Cpu* cpu, uint8_t value) {
    if (value & 0x80) {
        cpu->STATUS |= FLAG_NEGATIVE;
    } else {
        cpu->STATUS &= ~FLAG_NEGATIVE;
    }
}

// Set Carry Flag
void set_carry_flag(Cpu* cpu, bool set) {
    if (set) {
        cpu->STATUS |= FLAG_CARRY;
    } else {
        cpu->STATUS &= ~FLAG_CARRY;
    }
}

// Set Overflow Flag
void set_overflow_flag(Cpu* cpu, bool set) {
    if (set) {
        cpu->STATUS |= FLAG_OVERFLOW;
    } else {
        cpu->STATUS &= ~FLAG_OVERFLOW;
    }
}

// Set Interrupt Disable Flag
void set_interrupt_flag(Cpu* cpu, bool set) {
    if (set) {
        cpu->STATUS |= FLAG_INTERRUPT;
    } else {
        cpu->STATUS &= ~FLAG_INTERRUPT;
    }
}

// Stack Push/Pull
// Push a byte onto the stack
void push_stack(Cpu* cpu, uint8_t value) {
    write(cpu->bus, 0x0100 + cpu->SP, value);
    cpu->SP--;
}

// Pull a byte from the stack
uint8_t pull_stack(Cpu* cpu) {
    cpu->SP++;
    return read(cpu->bus, 0x0100 + cpu->SP);
}



//// Instruction Handlers (This is where computation with values occurs 'on the CPU'!)
void handle_LDA(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    uint8_t value = fetch_operand(cpu, opcode_table[opcode].addressing_mode, &address);
    cpu->A = value;
    set_zero_flag(cpu, cpu->A);
    set_negative_flag(cpu, cpu->A);
}

void handle_LDX(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    uint8_t value = fetch_operand(cpu, opcode_table[opcode].addressing_mode, &address);
    cpu->X = value;
    set_zero_flag(cpu, cpu->X);
    set_negative_flag(cpu, cpu->X);
}

void handle_LDY(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    uint8_t value = fetch_operand(cpu, opcode_table[opcode].addressing_mode, &address);
    cpu->Y = value;
    set_zero_flag(cpu, cpu->Y);
    set_negative_flag(cpu, cpu->Y);
}

void handle_STA(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0; // Target address to store the Accumulator

    // Retrieve the addressing mode for the current opcode
    AddressingMode mode = opcode_table[opcode].addressing_mode;

    switch(mode) {
        case ZERO_PAGE:
            // Zero Page addressing: next byte is the zero page address
            address = read(cpu->bus, cpu->PC++);
            break;
        case ZERO_PAGE_X:
            // Zero Page,X addressing: zero page address plus X register, wrapped around 0xFF
            address = (read(cpu->bus, cpu->PC++) + cpu->X) & 0xFF;
            break;
        case ZERO_PAGE_Y:
            // Zero Page,Y addressing: zero page address plus Y register, wrapped around 0xFF
            address = (read(cpu->bus, cpu->PC++) + cpu->Y) & 0xFF;
            break;
        case ABSOLUTE:
            // Absolute addressing: next two bytes form the 16-bit address
            address = read(cpu->bus, cpu->PC++) | (read(cpu->bus, cpu->PC++) << 8);
            break;
        case ABSOLUTE_X:
            // Absolute,X addressing: absolute address plus X register
            address = (read(cpu->bus, cpu->PC++) | (read(cpu->bus, cpu->PC++) << 8)) + cpu->X;
            break;
        case ABSOLUTE_Y:
            // Absolute,Y addressing: absolute address plus Y register
            address = (read(cpu->bus, cpu->PC++) | (read(cpu->bus, cpu->PC++) << 8)) + cpu->Y;
            break;
        case INDEXED_INDIRECT:
            // (Indirect,X) addressing:
            // 1. Add X to the zero page address (with wrap-around)
            // 2. Fetch the 16-bit address from the resulting zero page location
            {
                uint8_t zp = read(cpu->bus, cpu->PC++);
                uint16_t ptr = (zp + cpu->X) & 0xFF;
                address = read(cpu->bus, ptr) | (read(cpu->bus, (ptr + 1) & 0xFF) << 8);
            }
            break;
        case INDIRECT_INDEXED:
            // (Indirect),Y addressing:
            // 1. Fetch the 16-bit base address from the zero page address
            // 2. Add Y to the base address
            {
                uint8_t zp = read(cpu->bus, cpu->PC++);
                uint16_t base = read(cpu->bus, zp) | (read(cpu->bus, (zp + 1) & 0xFF) << 8);
                address = base + cpu->Y;
            }
            break;
        default:
            // STA does not support other addressing modes
            printf("STA encountered with unsupported addressing mode: %d\n", mode);
            cpu->running = false; // Halt the CPU or handle as needed
            return;
    }

    // Write the value of the Accumulator to the target address
    write(cpu->bus, address, cpu->A);
}

void handle_STX(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    AddressingMode mode = opcode_table[opcode].addressing_mode;

    switch(mode) {
        case ZERO_PAGE:
            address = read(cpu->bus, cpu->PC++);
            break;
        case ZERO_PAGE_Y:
            address = (read(cpu->bus, cpu->PC++) + cpu->Y) & 0xFF;
            break;
        case ABSOLUTE:
            address = read(cpu->bus, cpu->PC++) | (read(cpu->bus, cpu->PC++) << 8);
            break;
        default:
            printf("STX encountered with unsupported addressing mode: %d\n", mode);
            cpu->running = false;
            return;
    }

    write(cpu->bus, address, cpu->X);
}

void handle_STY(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    AddressingMode mode = opcode_table[opcode].addressing_mode;

    switch(mode) {
        case ZERO_PAGE:
            address = read(cpu->bus, cpu->PC++);
            break;
        case ZERO_PAGE_X:
            address = (read(cpu->bus, cpu->PC++) + cpu->X) & 0xFF;
            break;
        case ABSOLUTE:
            address = read(cpu->bus, cpu->PC++) | (read(cpu->bus, cpu->PC++) << 8);
            break;
        default:
            printf("STY encountered with unsupported addressing mode: %d\n", mode);
            cpu->running = false;
            return;
    }

    write(cpu->bus, address, cpu->Y);
}

// Register Transfers
void handle_TAX(Cpu* cpu, uint8_t opcode) {
    cpu->X = cpu->A;
    set_zero_flag(cpu, cpu->X);
    set_negative_flag(cpu, cpu->X);
}

void handle_TAY(Cpu* cpu, uint8_t opcode) {
    cpu->Y = cpu->A;
    set_zero_flag(cpu, cpu->Y);
    set_negative_flag(cpu, cpu->Y);
}

void handle_TXA(Cpu* cpu, uint8_t opcode) {
    cpu->A = cpu->X;
    set_zero_flag(cpu, cpu->A);
    set_negative_flag(cpu, cpu->A);
}

void handle_TYA(Cpu* cpu, uint8_t opcode) {
    cpu->A = cpu->Y;
    set_zero_flag(cpu, cpu->A);
    set_negative_flag(cpu, cpu->A);
}

// Stack Operations
void handle_TSX(Cpu* cpu, uint8_t opcode) {
    cpu->X = cpu->SP;
    set_zero_flag(cpu, cpu->X);
    set_negative_flag(cpu, cpu->X);
}

void handle_TXS(Cpu* cpu, uint8_t opcode) {
    cpu->SP = cpu->X;
    // No flags are affected
}

void handle_PHA(Cpu* cpu, uint8_t opcode) {
    push_stack(cpu, cpu->A);
}

void handle_PHP(Cpu* cpu, uint8_t opcode) {
    // Push the status register with break flag set
    uint8_t status = cpu->STATUS | FLAG_BREAK | FLAG_UNUSED;
    push_stack(cpu, status);
}

void handle_PLA(Cpu* cpu, uint8_t opcode) {
    cpu->A = pull_stack(cpu);
    set_zero_flag(cpu, cpu->A);
    set_negative_flag(cpu, cpu->A);
}

void handle_PLP(Cpu* cpu, uint8_t opcode) {
    cpu->STATUS = pull_stack(cpu);
    // Ensure unused bit is set
    cpu->STATUS |= FLAG_UNUSED;
}

// Logical Instructions
void handle_AND(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    uint8_t value = fetch_operand(cpu, opcode_table[opcode].addressing_mode, &address);
    cpu->A &= value;
    set_zero_flag(cpu, cpu->A);
    set_negative_flag(cpu, cpu->A);
}

void handle_EOR(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    uint8_t value = fetch_operand(cpu, opcode_table[opcode].addressing_mode, &address);
    cpu->A ^= value;
    set_zero_flag(cpu, cpu->A);
    set_negative_flag(cpu, cpu->A);
}

void handle_ORA(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    uint8_t value = fetch_operand(cpu, opcode_table[opcode].addressing_mode, &address);
    cpu->A |= value;
    set_zero_flag(cpu, cpu->A);
    set_negative_flag(cpu, cpu->A);
}

void handle_BIT(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    uint8_t value = fetch_operand(cpu, opcode_table[opcode].addressing_mode, &address);
    uint8_t result = cpu->A & value;
    set_zero_flag(cpu, result);
    // Set overflow flag based on bit 6 of value
    if (value & 0x40) {
        cpu->STATUS |= FLAG_OVERFLOW;
    } else {
        cpu->STATUS &= ~FLAG_OVERFLOW;
    }
    // Set negative flag based on bit 7 of value
    set_negative_flag(cpu, value);
}

// Arithmetic Instructions
void handle_ADC(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    uint8_t value = fetch_operand(cpu, opcode_table[opcode].addressing_mode, &address);
    uint16_t sum = cpu->A + value + ((cpu->STATUS & FLAG_CARRY) ? 1 : 0);

    // Set Carry Flag
    set_carry_flag(cpu, sum > 0xFF);

    // Set Overflow Flag
    if (((cpu->A ^ sum) & (value ^ sum) & 0x80) != 0) {
        set_overflow_flag(cpu, true);
    } else {
        set_overflow_flag(cpu, false);
    }

    cpu->A = sum & 0xFF;
    set_zero_flag(cpu, cpu->A);
    set_negative_flag(cpu, cpu->A);
}

void handle_SBC(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    uint8_t value = fetch_operand(cpu, opcode_table[opcode].addressing_mode, &address);
    uint8_t carry = (cpu->STATUS & FLAG_CARRY) ? 1 : 0;
    uint16_t diff = cpu->A - value - (1 - carry);

    // Set Carry Flag if no borrow
    set_carry_flag(cpu, cpu->A >= (value + (1 - carry)));

    // Set Overflow Flag
    if (((cpu->A ^ diff) & (value ^ diff) & 0x80) != 0) {
        set_overflow_flag(cpu, true);
    } else {
        set_overflow_flag(cpu, false);
    }

    cpu->A = diff & 0xFF;
    set_zero_flag(cpu, cpu->A);
    set_negative_flag(cpu, cpu->A);
}

void handle_CMP(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    uint8_t value = fetch_operand(cpu, opcode_table[opcode].addressing_mode, &address);
    uint16_t result = cpu->A - value;

    // Set Carry Flag if A >= value
    set_carry_flag(cpu, cpu->A >= value);

    set_zero_flag(cpu, result & 0xFF);
    set_negative_flag(cpu, result & 0xFF);
}

void handle_CPX(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    uint8_t value = fetch_operand(cpu, opcode_table[opcode].addressing_mode, &address);
    uint16_t result = cpu->X - value;

    // Set Carry Flag if X >= value
    set_carry_flag(cpu, cpu->X >= value);

    set_zero_flag(cpu, result & 0xFF);
    set_negative_flag(cpu, result & 0xFF);
}

void handle_CPY(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    uint8_t value = fetch_operand(cpu, opcode_table[opcode].addressing_mode, &address);
    uint16_t result = cpu->Y - value;

    // Set Carry Flag if Y >= value
    set_carry_flag(cpu, cpu->Y >= value);

    set_zero_flag(cpu, result & 0xFF);
    set_negative_flag(cpu, result & 0xFF);
}

// Increments & Decrements
void handle_INC(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    uint8_t value = fetch_operand(cpu, opcode_table[opcode].addressing_mode, &address);
    value++;
    write(cpu->bus, address, value);
    set_zero_flag(cpu, value);
    set_negative_flag(cpu, value);
}

void handle_INX(Cpu* cpu, uint8_t opcode) {
    cpu->X++;
    set_zero_flag(cpu, cpu->X);
    set_negative_flag(cpu, cpu->X);
}

void handle_INY(Cpu* cpu, uint8_t opcode) {
    cpu->Y++;
    set_zero_flag(cpu, cpu->Y);
    set_negative_flag(cpu, cpu->Y);
}

void handle_DEC(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    uint8_t value = fetch_operand(cpu, opcode_table[opcode].addressing_mode, &address);
    value--;
    write(cpu->bus, address, value);
    set_zero_flag(cpu, value);
    set_negative_flag(cpu, value);
}

void handle_DEX(Cpu* cpu, uint8_t opcode) {
    cpu->X--;
    set_zero_flag(cpu, cpu->X);
    set_negative_flag(cpu, cpu->X);
}

void handle_DEY(Cpu* cpu, uint8_t opcode) {
    cpu->Y--;
    set_zero_flag(cpu, cpu->Y);
    set_negative_flag(cpu, cpu->Y);
}

// Shifts
void handle_ASL(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    uint8_t value;
    bool is_accumulator = false;

    if (opcode_table[opcode].addressing_mode == ACCUMULATOR) {
        value = cpu->A;
        is_accumulator = true;
    } else {
        value = fetch_operand(cpu, opcode_table[opcode].addressing_mode, &address);
    }

    // Set Carry Flag based on bit 7
    set_carry_flag(cpu, (value & 0x80) != 0);

    value <<= 1;
    set_zero_flag(cpu, value);
    set_negative_flag(cpu, value);

    if (is_accumulator) {
        cpu->A = value;
    } else {
        write(cpu->bus, address, value);
    }
}

void handle_LSR(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    uint8_t value;
    bool is_accumulator = false;

    if (opcode_table[opcode].addressing_mode == ACCUMULATOR) {
        value = cpu->A;
        is_accumulator = true;
    } else {
        value = fetch_operand(cpu, opcode_table[opcode].addressing_mode, &address);
    }

    // Set Carry Flag based on bit 0
    set_carry_flag(cpu, (value & 0x01) != 0);

    value >>= 1;
    set_zero_flag(cpu, value);
    set_negative_flag(cpu, value);

    if (is_accumulator) {
        cpu->A = value;
    } else {
        write(cpu->bus, address, value);
    }
}

void handle_ROL(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    uint8_t value;
    bool is_accumulator = false;

    if (opcode_table[opcode].addressing_mode == ACCUMULATOR) {
        value = cpu->A;
        is_accumulator = true;
    } else {
        value = fetch_operand(cpu, opcode_table[opcode].addressing_mode, &address);
    }

    // Save current Carry Flag
    uint8_t carry_in = (cpu->STATUS & FLAG_CARRY) ? 1 : 0;

    // Set Carry Flag based on bit 7
    set_carry_flag(cpu, (value & 0x80) != 0);

    value = (value << 1) | carry_in;
    set_zero_flag(cpu, value);
    set_negative_flag(cpu, value);

    if (is_accumulator) {
        cpu->A = value;
    } else {
        write(cpu->bus, address, value);
    }
}

void handle_ROR(Cpu* cpu, uint8_t opcode) {
    uint16_t address = 0;
    uint8_t value;
    bool is_accumulator = false;

    if (opcode_table[opcode].addressing_mode == ACCUMULATOR) {
        value = cpu->A;
        is_accumulator = true;
    } else {
        value = fetch_operand(cpu, opcode_table[opcode].addressing_mode, &address);
    }

    // Save current Carry Flag
    uint8_t carry_in = (cpu->STATUS & FLAG_CARRY) ? 0x80 : 0;

    // Set Carry Flag based on bit 0
    set_carry_flag(cpu, (value & 0x01) != 0);

    value = (value >> 1) | carry_in;
    set_zero_flag(cpu, value);
    set_negative_flag(cpu, value);

    if (is_accumulator) {
        cpu->A = value;
    } else {
        write(cpu->bus, address, value);
    }
}

// Jumps & Calls
void handle_JMP(Cpu* cpu, uint8_t opcode) {
    AddressingMode mode = opcode_table[opcode].addressing_mode;
    if (mode == ABSOLUTE) {
        uint16_t addr = read(cpu->bus, cpu->PC++) | (read(cpu->bus, cpu->PC++) << 8);
        cpu->PC = addr;
    } else if (mode == INDIRECT) {
        uint16_t ptr = read(cpu->bus, cpu->PC++) | (read(cpu->bus, cpu->PC++) << 8);
        uint16_t addr;
        // Handle the 6502 JMP indirect page boundary bug
        if ((ptr & 0x00FF) == 0x00FF) {
            addr = read(cpu->bus, ptr) | (read(cpu->bus, ptr & 0xFF00) << 8);
        } else {
            addr = read(cpu->bus, ptr) | (read(cpu->bus, ptr + 1) << 8);
        }
        cpu->PC = addr;
    } else {
        printf("JMP encountered with unsupported addressing mode: %d\n", mode);
        cpu->running = false;
    }
}

void handle_JSR(Cpu* cpu, uint8_t opcode) {
    uint16_t addr = read(cpu->bus, cpu->PC++) | (read(cpu->bus, cpu->PC++) << 8);
    uint16_t return_addr = cpu->PC - 1;
    // Push high byte first
    push_stack(cpu, (return_addr >> 8) & 0xFF);
    push_stack(cpu, return_addr & 0xFF);
    cpu->PC = addr;
}

void handle_RTS(Cpu* cpu, uint8_t opcode) {
    uint8_t low = pull_stack(cpu);
    uint8_t high = pull_stack(cpu);
    uint16_t return_addr = (high << 8) | low;
    cpu->PC = return_addr + 1;
}

// Branches
void handle_BCC(Cpu* cpu, uint8_t opcode) {
    if (!(cpu->STATUS & FLAG_CARRY)) {
        int8_t offset = (int8_t)read(cpu->bus, cpu->PC++);
        uint16_t new_pc = cpu->PC + offset;
        // Optionally handle cycle count for page crossing
        cpu->PC = new_pc;
    } else {
        cpu->PC += 1;
    }
}

void handle_BCS(Cpu* cpu, uint8_t opcode) {
    if (cpu->STATUS & FLAG_CARRY) {
        int8_t offset = (int8_t)read(cpu->bus, cpu->PC++);
        uint16_t new_pc = cpu->PC + offset;
        cpu->PC = new_pc;
    } else {
        cpu->PC +=1;
    }
}

void handle_BEQ(Cpu* cpu, uint8_t opcode) {
    if (cpu->STATUS & FLAG_ZERO) {
        int8_t offset = (int8_t)read(cpu->bus, cpu->PC++);
        uint16_t new_pc = cpu->PC + offset;
        cpu->PC = new_pc;
    } else {
        cpu->PC +=1;
    }
}

void handle_BMI(Cpu* cpu, uint8_t opcode) {
    if (cpu->STATUS & FLAG_NEGATIVE) {
        int8_t offset = (int8_t)read(cpu->bus, cpu->PC++);
        uint16_t new_pc = cpu->PC + offset;
        cpu->PC = new_pc;
    } else {
        cpu->PC +=1;
    }
}

void handle_BNE(Cpu* cpu, uint8_t opcode) {
    if (!(cpu->STATUS & FLAG_ZERO)) {
        int8_t offset = (int8_t)read(cpu->bus, cpu->PC++);
        uint16_t new_pc = cpu->PC + offset;
        cpu->PC = new_pc;
    } else {
        cpu->PC +=1;
    }
}

void handle_BPL(Cpu* cpu, uint8_t opcode) {
    if (!(cpu->STATUS & FLAG_NEGATIVE)) {
        int8_t offset = (int8_t)read(cpu->bus, cpu->PC++);
        uint16_t new_pc = cpu->PC + offset;
        cpu->PC = new_pc;
    } else {
        cpu->PC +=1;
    }
}

void handle_BVC(Cpu* cpu, uint8_t opcode) {
    if (!(cpu->STATUS & FLAG_OVERFLOW)) {
        int8_t offset = (int8_t)read(cpu->bus, cpu->PC++);
        uint16_t new_pc = cpu->PC + offset;
        cpu->PC = new_pc;
    } else {
        cpu->PC +=1;
    }
}

void handle_BVS(Cpu* cpu, uint8_t opcode) {
    if (cpu->STATUS & FLAG_OVERFLOW) {
        int8_t offset = (int8_t)read(cpu->bus, cpu->PC++);
        uint16_t new_pc = cpu->PC + offset;
        cpu->PC = new_pc;
    } else {
        cpu->PC +=1;
    }
}

// Status Flag Changes
void handle_CLC(Cpu* cpu, uint8_t opcode) {
    set_carry_flag(cpu, false);
}

void handle_CLD(Cpu* cpu, uint8_t opcode) {
    cpu->STATUS &= ~FLAG_DECIMAL;
}

void handle_CLI(Cpu* cpu, uint8_t opcode) {
    cpu->STATUS &= ~FLAG_INTERRUPT;
}

void handle_CLV(Cpu* cpu, uint8_t opcode) {
    set_overflow_flag(cpu, false);
}

void handle_SEC(Cpu* cpu, uint8_t opcode) {
    set_carry_flag(cpu, true);
}

void handle_SED(Cpu* cpu, uint8_t opcode) {
    cpu->STATUS |= FLAG_DECIMAL;
}

void handle_SEI(Cpu* cpu, uint8_t opcode) {
    set_interrupt_flag(cpu, true);
}

// System Functions
void handle_BRK(Cpu* cpu, uint8_t opcode) {
    cpu->PC++; // BRK is a 2-byte instruction
    push_stack(cpu, (cpu->PC >> 8) & 0xFF);
    push_stack(cpu, cpu->PC & 0xFF);
    uint8_t status = cpu->STATUS | FLAG_BREAK | FLAG_UNUSED;
    push_stack(cpu, status);
    set_interrupt_flag(cpu, true);
    // Set PC to IRQ vector at 0xFFFE
    uint16_t irq_low = read(cpu->bus, 0xFFFE);
    uint16_t irq_high = read(cpu->bus, 0xFFFF) << 8;
    cpu->PC = irq_low | irq_high;
}

void handle_NOP(Cpu* cpu, uint8_t opcode) {
    // NOP does nothing
    // NOTE TO SELF: PC is already incremented by fetch
}

void handle_RTI(Cpu* cpu, uint8_t opcode) {
    // Pull status
    cpu->STATUS = pull_stack(cpu);
    // Ensure unused bit is set
    cpu->STATUS |= FLAG_UNUSED;
    // Pull PC
    uint8_t low = pull_stack(cpu);
    uint8_t high = pull_stack(cpu);
    cpu->PC = (high << 8) | low;
}
