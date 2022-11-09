/*
 *  Hydrophones ADC input and DSP bearing calculation
 *  for Mizzou SURF
 *  Mason Fleck
 * 
 *  FIR Filter information, variables and constants are in firFilter files
 *  DSP information and functions are in dspFunctions files
 *  ADC information and variables are in adcConfig files
 * 
 *  Useful References:
 *      - http://pedvide.github.io/ADC/docs/Teensy_4_html/index.html
 *      - https://github.com/pedvide/ADC/blob/master/examples/synchronizedMeasurements/synchronizedMeasurements.ino
 *      - https://www.keil.com/pack/doc/CMSIS/DSP/html/index.html
 */

#include <Arduino.h>
#include "dspFunctions.h"
#include "digitalPot.h"

// hardware serial port for talking to computer
// https://www.pjrc.com/teensy/td_uart.html
#define computerSerial Serial1

// Some various timers
elapsedMillis ledTimer;

// Delay variable
int8_t delaySamples;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(ADC_PIN_0, INPUT);
    pinMode(ADC_PIN_1, INPUT);

    computerSerial.begin(9600);

    initADC();

    digitalPotInit();

    setWindowFunction(HAMMING);

    // Initialize FIR filter
    arm_fir_init_f32(&fir_instance_0, FILTER_TAP_NUM, filter_taps, firState0, BLOCK_SIZE);
    arm_fir_init_f32(&fir_instance_1, FILTER_TAP_NUM, filter_taps, firState1, BLOCK_SIZE);

    // Initialize fft
    arm_rfft_fast_init_f32(&f32_instance0, BUFF_SIZE);
    arm_rfft_fast_init_f32(&f32_instance1, BUFF_SIZE);
    arm_rfft_fast_init_f32(&f32_instance_rxy, BUFF_SIZE);
}


void loop() {
    if (abdma1.interrupted() && (abdma2.interrupted())) {
        ProcessAnalogData(&abdma1, &abdma2,  &delaySamples);
	computerSerial.println(delaySamples);
    }

    if(ledTimer >= 1000) { // heartbeat led
        digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
        ledTimer = 0;
    }
}
// -- written in vim
