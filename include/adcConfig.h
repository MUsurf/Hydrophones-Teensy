#pragma once

#include <ADC.h>
#include <ADC_Module.h>
#include <AnalogBufferDMA.h>
#include <DMAChannel.h>

// Pin numbers to read from
#define ADC_PIN_0       A0
#define ADC_PIN_1       A1

// Number of averages. Can be 0, 4, 8, 16 or 32
#define ADC_AVGS        0

// Resolution in bits. T4.1 may not work with all values
// For single-ended measurements: 8, 10, 12 or 16 bits. For differential measurements: 9, 11, 13 or 16 bits
#define ADC_RES         12

// How fast the ADC will try to sample
// VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED.
#define ADC_SA_SPD      ADC_SAMPLING_SPEED::VERY_HIGH_SPEED

// Changes ADCK to alter conversion speed
// VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED, VERY_HIGH_SPEED, ADACK_2_4, ADACK_4_0, ADACK_5_2 or ADACK_6_2
#define ADC_CNV_SPD     ADC_CONVERSION_SPEED::VERY_HIGH_SPEED

/*
 *  Sample rate must be > 2*(maximum frequency that we are trying to measure)
 *  Since the frequencies for competition are 25kHz to 40kHz, sample rate must be >80kSa/s
 *  100kSa/s is a nice even number that should be plenty fast for sampling.
 *  
 *  Equation for minimum sample rate to be able to determine a difference in the delays between hydrophones:
 *  speed of propogation of wave (probably ~1400m/s) / distance between the hydrophones (138mm) = min sample rate
 *  ** MAKE SURE UNITS MATCH WHEN TRYING TO SOLVE THE ABOVE EQUATION **
 *  This will probably be smaller than the minimum sample rate required to obtain the 40kHz frequency content
 */
#define SAMPLE_RATE     100000

/* 
 *  FFT will discreteize the continuous frequency spectrum into bins. 
 *  sample rate / buffer size = amount of frequency content in each bin or resolution.
 *  i.e. at buffer size = 500 and sample rate = 100kSa/s, the resolution will be 200Hz per bin.
 *  This means after fft, the first bin will hold the magnitude of frequency content from 0-200Hz, second bin from 201Hz to 400Hz, and so on. 
 * 
 *  For some of the DSP functions provided by arm_math.h, the buffer size must be 
 *  2^n where n is between 5 and 13 (inclusive)
 * 
 *  Minimum size should be 256 for the 500 Hz difference in pinger freq at competition, but I like more resolution, and this processor is pretty powerful.
 */
#define BUFF_SIZE       1024

// DMA Buffers and structures
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff1[BUFF_SIZE];
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff2[BUFF_SIZE];
extern AnalogBufferDMA abdma1;

DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff2_1[BUFF_SIZE];
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff2_2[BUFF_SIZE];
extern AnalogBufferDMA abdma2;


void initADC();