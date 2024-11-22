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

// All 52 Instructions
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

// Addressing Modes
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
