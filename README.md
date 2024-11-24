# holbroow6502

![C Language](https://img.shields.io/badge/language-C-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)

**Will Holbrook**  
*Lancaster University Third Year Project 2024*  
**Course:** Computer Science BSc (G400)  
**Module:** SCC 300 - EmuPC  
**Supervisor:** Professor Andrew Scott

## Overview

`holbroow6502` is an implementation of a fully functional 6502 CPU emulator written in C. This project is part of the SCC 300 Third Year Project at Lancaster University, overseen by Professor Andrew Scott.

## Features

- **6502 CPU Emulation:** Accurately emulates the 6502 architecture.
- **Instruction Set:** Implements all 6502 instructions.
- **Testing Framework:** Includes programs to test CPU states and instruction sequences.
- **Memory Management:** Utilizes a dedicated bus system for memory operations.

## Project Structure

### Files

| File             | Description                                                                                          |
| ---------------- | ---------------------------------------------------------------------------------------------------- |
| `Main.c`         | A simple program used to test the CPU's state by running test instruction sequences.                 |
| `holbroow6502.c` | The CPU implementation file containing all instructions, structured according to its header file.    |
| `holbroow6502.h` | Header file for the CPU, defining structures and function prototypes.                                |
| `Bus.c`          | Implements the Bus, which manages a block of memory and handles read/write operations.               |
| `Bus.h`          | Header file for the Bus system.                                                                     |

## Getting Started

### Prerequisites

- **C Compiler:** Ensure you have a C compiler installed (e.g., GCC).
- **Make (optional):** For using Makefiles if available.

### Installation

1. **Clone the Repository:**
    ```bash
    git clone https://github.com/yourusername/holbroow6502.git
    ```
2. **Navigate to the Project Directory:**
    ```bash
    cd holbroow6502
    ```
3. **Compile the Project:**
    ```bash
    gcc Main.c holbroow6502.c Bus.c -o holbroow6502
    ```

### Usage

Run the emulator using the compiled executable:
```bash
./holbroow6502
