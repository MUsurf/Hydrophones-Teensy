#include "digitalPot.h"

void digitalPotInit() {
    for(int i = 0; i < NUM_CS_PINS; i++) {
        pinMode(csPins[i], OUTPUT);
    }

    SPI_PORT.begin();

    digitalPotWrite(255);
}

void digitalPotWrite(uint8_t value) {
    for(int i = 0; i < NUM_CS_PINS; i++) {
        digitalWriteFast(csPins[i], LOW);
        delay(1);
        SPI_PORT.transfer(0);
        SPI_PORT.transfer(value);
        delay(1);
        digitalWriteFast(csPins[i], HIGH);
        delay(1);
    }
}

void digitalPotIncrement() {
    for(int i = 0; i < NUM_CS_PINS; i++) {
        digitalWriteFast(csPins[i], LOW);
        delay(1);
        SPI_PORT.transfer(0b00000100);
        delay(1);
        digitalWriteFast(csPins[i], HIGH);
        delay(1);
    }
}

void digitalPotDecrement() {
    for(int i = 0; i < NUM_CS_PINS; i++) {
        digitalWriteFast(csPins[i], LOW);
        delay(1);
        SPI_PORT.transfer(0b00001000);
        delay(1);
        digitalWriteFast(csPins[i], HIGH);
        delay(1);
    }
}