// Will Holbrook | Lancaster University Third Year Project 2024 (SCC 300: EmuPC)
// Bus.c

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#define MEMORY_SIZE 2048 // 2KB

// Define the Bus structure
typedef struct {
    uint8_t memory[MEMORY_SIZE];
} Bus;

Bus* init_bus() {
    Bus* bus = (Bus*)malloc(sizeof(Bus));
    if (!bus) {
        fprintf(stderr, "Error: Failed to allocate memory for the Bus.");
        exit(1);
    }
    
    // Initialise all memory to zero for use
    for (size_t i = 0; i < MEMORY_SIZE; i++)
    {
        bus->memory[i] = 0;
    }
    return bus;
}

// Function to write data to the bus
void bus_write(Bus* bus, uint16_t address, uint8_t data) {
    if (address < MEMORY_SIZE) {
        bus->memory[address] = data;
    } else {
        fprintf(stderr, "Error: Address to write on bus not in range.");
    }
}

// Function to read data from the bus
uint8_t bus_read(Bus* bus, uint16_t address) {
    if (address < MEMORY_SIZE) {
        return bus->memory[address];
    } else {
        fprintf(stderr, "Error: Address to read on bus not in range.");
    }
}
