// Will Holbrook | Lancaster University Third Year Project 2024 (SCC 300: EmuPC)
// Bus.c (Bus, which initalises memory and allows read and writes to said memory, as nothing else is currently attached to the bus.)

#include "Bus.h"

// Function to initialize the bus
Bus* init_bus() {
    Bus* bus = (Bus*)malloc(sizeof(Bus));
    if (!bus) {
        fprintf(stderr, "Error: Failed to allocate memory for the Bus.\n");
        exit(1);
    }

    // Initialize all memory to zero
    for (size_t i = 0; i < MEMORY_SIZE; i++) {
        bus->memory[i] = 0;
    }

    return bus;
}

// Function to write data to the bus
void bus_write(Bus* bus, uint16_t address, uint8_t data) {
    if (address < MEMORY_SIZE) {
        bus->memory[address] = data;
    } else {
        fprintf(stderr, "Error: Address 0x%04X out of range for write operation.\n", address);
    }
}

// Function to read data from the bus
uint8_t bus_read(Bus* bus, uint16_t address) {
    if (address < MEMORY_SIZE) {
        return bus->memory[address];
    } else {
        fprintf(stderr, "Error: Address 0x%04X out of range for read operation.\n", address);
        return 0; // Return 0 or handle as appropriate
    }
}
