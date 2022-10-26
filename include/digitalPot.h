#pragma once
// Library for MCP413X/415X/423X/425X series digital potentiometers
// This is sepcifically targeted for the MCP4151
#include <SPI.h>

#define SPI_PORT    SPI1

#define NUM_CS_PINS 4
#define CS_0_PIN    37
#define CS_1_PIN    36
#define CS_2_PIN    35
#define CS_3_PIN    34

const uint8_t csPins[NUM_CS_PINS] = { CS_0_PIN, CS_1_PIN, CS_2_PIN, CS_3_PIN };

void digitalPotInit();
void digitalPotWrite(uint8_t value);
void digitalPotIncrement();
void digitalPotDecrement();