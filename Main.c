// Will Holbrook | Lancaster University Third Year Project 2024 (SCC 300: EmuPC)
// Main.c (Loads a (currently hard coded) program into memory and calls the 6502 CPU to 
// run from that address, hence running the stored program.)

#include "Bus.h"
#include "holbroow6502.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>

void load_program(Bus* bus, uint16_t start_address, const char* hex_string) {
    uint16_t current_address = start_address;   // Set the current memory offset to the intended start pos
    const char* ptr = hex_string;               // Set a pointer to the passed string containing the object code
    char buffer[3] = {0};                       // Two characters for a HEX byte
    int byte_index = 0;                         // Allow indexing through bytes

    while (*ptr != '\0') {  // While not at the end of the object code HEX string
        // Skip any spaces or newlines, should there be any
        if (isspace((unsigned char)*ptr)) {
            ptr++;
            continue;
        }

        // Read two hex characters to form a byte
        buffer[0] = toupper((unsigned char)*ptr);   // First character as first 4 bits
        ptr++;
        if (*ptr == '\0') {
            fprintf(stderr, "MANAGER: Error, Incomplete byte in hex string while loading program bytes.\n");
            break;
        }
        buffer[1] = toupper((unsigned char)*ptr);   // Second character as second 4 bits to form the full byte
        ptr++;
        buffer[2] = '\0'; // Null-terminate the string

        // Convert the hex string to a byte
        char* endptr;
        uint8_t byte = (uint8_t)strtol(buffer, &endptr, 16);
        if (*endptr != '\0') {
            fprintf(stderr, "MANAGER: Error, Invalid hex byte '%s' while loading program bytes.\n", buffer);
            continue; // Skip invalid byte
        }

        // Write the byte to memory
        bus_write(bus, current_address, byte);
        current_address++;

        // Check for memory bounds
        if (current_address > 0xFFFF) {
            fprintf(stderr, "MANAGER: Error, Reached end of memory while loading program.\n");
            break;
        }
    }

    printf("MANAGER: Program loaded successfully, starting at 0x%04X.\n", start_address);
}

// Main function
int main() {
    // Initialize Bus
    Bus* bus = init_bus();

    // Initialize CPU
    Cpu* cpu = init_cpu(bus);


    // Program code to load:    Multiply 10 by 3
    load_program(bus, 0x8000, "A2 0A 8E 00 00 A2 03 8E 01 00 AC 00 00 A9 00 18 6D 01 00 88 D0 FA 8D 02 00 EA EA EA");


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

