/*
    Datasheet
    https://www.analog.com/media/en/technical-documentation/data-sheets/MAX11047-MAX11059.pdf
*/
#include <Arduino.h>

// Pin definitions for the MAX11057 ADC
typedef struct MAX11057_pins_ {
    uint8_t DB[14] = { 0 };     // Data bus pins. Driven by ADC. Use to read each bit of the output sample
    uint8_t CR[4] = { 0 };      // Configuration register pins. Driven by MCU. CR2 and CR3 set internally based on DB0 and DB1 respectively
    uint8_t WR = 0;             // Write registers pin. Driven by MCU. Drive LOW to write to configuration registers
    uint8_t RD = 0;             // Read input pin. Driven by MCU. Drive LOW to from ADC. Each rising edge puts the next channel on the data bus
    uint8_t CS = 0;             // Chip select pin. Drvien by MCU. Drive LOW when reading from or writing to ADC
    uint8_t CONVST = 0;         // Conversion start pin. Driven by MCU. Drive HIGH to begin conversion
    uint8_t SHDN = 0;           // Shutdown pin. Driven by MCU. Drive HIGH to put ADC into shutdown mode. Configuration register contents are not erased in shutdown.
    uint8_t EOC = 0;            // End of conversion pin. Driven by ADC. ADC drives EOC LOW when conversion is completed
} MAX11057_pins;

class MAX11057 {
private:
    MAX11057_pins *pins = NULL;
    bool data_fmt = 0;
    bool convst_mode = 0;

public:
    MAX11057(MAX11057_pins *pins);
    ~MAX11057();

    /*
        Write a new configuration to the ADC configuration register
        reference:  selects the internal or external reference
            - 0: internal reference
            - 1: external reference supplied at REFIO pin
            - ADC default: 0

        data_fmt:  selects output data format
            - 0: offset binary
            - 1: two's complement
            - ADC default: 0

        convst_mode:  selects the aquisition mode
            - 0: drive CONVST LOW for each aquisition
            - 1: aquisition starts as soon as previous conversion completes
            - ADC default: 0
    */
    void writeConfigRegister(bool reference, bool data_fmt, bool convst_mode);
};

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

inline void MAX11057::writeConfigRegister(bool reference, bool data_fmt_, bool convst_mode_) {
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
