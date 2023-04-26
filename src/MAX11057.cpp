#include "MAX11057.h"

MAX11057::MAX11057(MAX11057_pins *pins_) {
    pins = pins_;

    // Make sure DB0=CR2 and DB1=CR3
    pins->CR[2] = pins->DB[0];
    pins->CR[3] = pins->DB[1];

    // Set pinmodes
    for(uint8_t i = 0; i < 14; i++) {
        pinMode(pins->DB[i], INPUT);
    }
    
    pinMode(pins->CR[0], OUTPUT);   // CR2 and CR3 are DB pins as well.
    pinMode(pins->CR[1], OUTPUT);   // Switching those to OUTPUTs is handled by writeConfigRegister()

    pinMode(pins->WR, OUTPUT);
    pinMode(pins->CS, OUTPUT);
    pinMode(pins->RD, OUTPUT);
    pinMode(pins->CONVST, OUTPUT);
    pinMode(pins->SHDN, OUTPUT);
    pinMode(pins->EOC, INPUT);
}

MAX11057::~MAX11057() {
}

void MAX11057::writeConfigRegister(bool reference, bool data_fmt_, bool convst_mode_) {
    // Note: Timing diagram for this: datasheet Fig. 4 (pg 18).
    // Corresponding minimum timings at end of Electrical Characteristics table (pg. 5)
    // Timings for this function are probably small enough that delays are not required in this function
    data_fmt = data_fmt_;
    convst_mode = convst_mode_;
    
    // Set CR2 and CR3 pins to OUTPUTS (They are also DB pins)
    pinMode(pins->CR[2], OUTPUT);
    pinMode(pins->CR[3], OUTPUT);

    // Set ADC into programming mode
    digitalWrite(pins->CS, LOW);
    digitalWrite(pins->WR, LOW);

    // Write configuration
    digitalWrite(pins->CR[0], convst_mode);
    digitalWrite(pins->CR[1], LOW);
    digitalWrite(pins->CR[2], data_fmt);
    digitalWrite(pins->CR[3], reference);

    // End programming mode
    digitalWrite(pins->WR, HIGH);
    digitalWrite(pins->CS, HIGH);

    // Set CR2 and CR3 back to INPUTS
    pinMode(pins->CR[2], INPUT);
    pinMode(pins->CR[3], INPUT);
}
