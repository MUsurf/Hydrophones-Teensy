#pragma once

#include <arm_const_structs.h>
#include "firFilter.h"
#include "adcConfig.h"


/*
 *  Constants
 */

// These offsets are used to remove some error 
#define CALIBRATION_OFFSET_0    0
#define CALIBRATION_OFFSET_1    0

// Minimum value to print out. Does an okay job of detecting when a signal is present
#define THRESHOLD       100

// Window types. Google these if you have questions or want to add more.
enum windowTypes{HAMMING, HANNING, BLACKMANHARRIS, FLATTOP};

/*
 *  Functions
 */
void copy_from_dma_buff_to_dsp_buff(volatile uint16_t *dmaBuff, volatile uint16_t *end_dmaBuff, float32_t *dspBuff, float32_t offset);
void ProcessAnalogData(AnalogBufferDMA *pabdma0, AnalogBufferDMA *pabdma1, int8_t *delayVal);

// Windowing related functions
void setWindowFunction(uint8_t windowType);
void applyWindowToBuffer(float32_t *buffer);

// Complex math functions
void complex_array_time_reversal(float32_t *inputArr);

// Print functions
void printBuffers(bool print, uint16_t size, float32_t *buff0, float32_t *buff1 = nullptr);
uint32_t printCycles(uint32_t cycleCounterStart, const String inputStr);