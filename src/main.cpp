/*
 *  Hydrophones ADC input and DSP bearing calculation
 *  for Mizzou SURF
 *  Mason Fleck
 * 
 *  Useful References:
 *      - http://pedvide.github.io/ADC/docs/Teensy_4_html/index.html
 *      - https://github.com/pedvide/ADC/blob/master/examples/synchronizedMeasurements/synchronizedMeasurements.ino
 *      - https://www.keil.com/pack/doc/CMSIS/DSP/html/index.html
 */

#include <Arduino.h>
#include <ADC.h>
#include <ADC_Module.h>
#include <ADC_util.h>
#include <AnalogBufferDMA.h>
#include <DMAChannel.h>
#include <arm_math.h>
#include <arm_const_structs.h>
#include "firFilter.h"
#include "constantInputs.h"

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

// These offsets are used to remove some error 
#define CALIBRATION_OFFSET_0    0
#define CALIBRATION_OFFSET_1    0

// Minimum value to print out. Does an okay job of detecting when a signal is present
#define THRESHOLD       100


// Window types. Google these if you have questions or want to add more.
float32_t window[BUFF_SIZE];
enum windowTypes{HAMMING, HANNING, BLACKMANHARRIS, FLATTOP};

// ADC
ADC *adc = new ADC();

// DMA Buffers and structures
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff1[BUFF_SIZE];
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff2[BUFF_SIZE];
AnalogBufferDMA abdma1(dma_adc_buff1, BUFF_SIZE, dma_adc_buff2, BUFF_SIZE);

DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff2_1[BUFF_SIZE];
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff2_2[BUFF_SIZE];
AnalogBufferDMA abdma2(dma_adc_buff2_1, BUFF_SIZE, dma_adc_buff2_2, BUFF_SIZE);

// FIR bandpass filter variables. Coeffs in firFilter.h
arm_fir_instance_f32 fir_instance_0, fir_instance_1;
float32_t firState0[FILTER_TAP_NUM + BLOCK_SIZE - 1], firState1[FILTER_TAP_NUM + BLOCK_SIZE - 1];

// FFT variables
arm_rfft_fast_instance_f32 f32_instance0, f32_instance1, f32_instance_rxy;

// Some various timers
elapsedMillis ledTimer;
elapsedMillis printTimer;

// Function prototypes
void ProcessAnalogData(AnalogBufferDMA *pabdma0, AnalogBufferDMA *pabdma1);
void setWindowFunction(uint8_t windowType);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(ADC_PIN_0, INPUT);
    pinMode(ADC_PIN_1, INPUT);

    Serial.begin(115200);

    adc->adc0->setAveraging(ADC_AVGS);
    adc->adc0->setResolution(ADC_RES);
    adc->adc0->setConversionSpeed(ADC_CNV_SPD);
    adc->adc0->setSamplingSpeed(ADC_SA_SPD);
    adc->adc0->recalibrate();

    adc->adc1->setAveraging(ADC_AVGS);
    adc->adc1->setResolution(ADC_RES);
    adc->adc1->setConversionSpeed(ADC_CNV_SPD);
    adc->adc1->setSamplingSpeed(ADC_SA_SPD);
    adc->adc1->recalibrate();

    abdma1.init(adc, ADC_0/*, DMAMUX_SOURCE_ADC_ETC*/);
    abdma2.init(adc, ADC_1/*, DMAMUX_SOURCE_ADC_ETC*/);

    // Start the dma operation..
    adc->adc0->startSingleRead(ADC_PIN_0); // call this to setup everything before the Timer starts, differential is also possible
    adc->adc1->startSingleRead(ADC_PIN_1);

    adc->adc0->startTimer(SAMPLE_RATE); // samples/sec
    adc->adc1->startTimer(SAMPLE_RATE); // samples/sec

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

    if ( /*printTimer >= 1000 &&*/ abdma1.interrupted() && (abdma2.interrupted())) {
        ProcessAnalogData(&abdma1, &abdma2);
        printTimer = 0;
    }
 

    if(ledTimer >= 1000) { // heartbeat led
        digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
        ledTimer = 0;
    }
}

void copy_from_dma_buff_to_dsp_buff(volatile uint16_t *dmaBuff, volatile uint16_t *end_dmaBuff, float32_t *dspBuff, float32_t offset) {
    volatile uint16_t *dmaBuff_ptr = dmaBuff;
    float32_t *dspBuff_ptr = dspBuff;

    offset += 2048.0;     // from unsigned 12 bit to signed values

    while(dmaBuff_ptr < end_dmaBuff) {
        *dspBuff_ptr = *dmaBuff_ptr - offset;   
        dspBuff_ptr++;
        dmaBuff_ptr++;
        
    }
}

void setWindowFunction(uint8_t windowType) {
    const float32_t n = PI/(BUFF_SIZE - 1);
    switch(windowType) {
        case HAMMING:
            for(uint16_t i = 0; i < BUFF_SIZE; i++) { window[i] = 0.54 - 0.46*cos(2*i*n); }
            break;

        case HANNING:
            for(uint16_t i = 0; i < BUFF_SIZE; i++) { window[i] = 0.5 - 0.5*cos(2*i*n); }
            break;

        case BLACKMANHARRIS:
            for(uint16_t i = 0; i < BUFF_SIZE; i++) { window[i] = 0.35875 - 0.48829*cos(2*i*n) + 0.14128*cos(4*i*n) - 0.01168*cos(6*i*n); }
            break;

        case FLATTOP:
            for(uint16_t i = 0; i < BUFF_SIZE; i++) { window[i] = 0.21557895 - 0.41663158*cos(2*i*n) + 0.277263158*cos(4*i*n) - 0.083578947*cos(6*i*n) + 0.006947368*cos(8*i*n); }
            break;
        
        default: // Rectangular
            for(uint16_t i = 0; i < BUFF_SIZE; i++) { window[i] = 1; }
            break;
    }
}

void applyWindowToBuffer(float32_t *buffer) {
    float32_t *windowPtr = window;
    float32_t *bufferPtr = buffer;

    for(uint16_t i = 0; i < BUFF_SIZE; i++) {
        *bufferPtr *= *windowPtr;
        bufferPtr++;
        windowPtr++;
    }
}

void complex_array_time_reversal(float32_t *inputArr) {
    float32_t tmpReal, tmpImag;
    uint16_t j = BUFF_SIZE/2 - 1;
    // swap first/last values
    for(uint16_t i = 0; i < BUFF_SIZE/4; i++) {
        tmpReal = inputArr[2*i];
        tmpImag = inputArr[2*i+1];

        inputArr[2*i] = inputArr[2*j];
        inputArr[2*i+1] = inputArr[2*j+1];

        inputArr[2*j] = tmpReal;
        inputArr[2*j+1] = tmpImag;

        j--;
    }
}

void printBuffers(bool print, uint16_t size, float32_t *buff0, float32_t *buff1 = nullptr) {
    if(print == true) {
        if(buff1 == nullptr) {
            for(int i = 0; i < size; i++) {
                Serial.print(buff0[i]);
                Serial.println();
            }
        }
        else {
            for(int i = 0; i < size; i++) {
                Serial.print(buff0[i]);
                Serial.print("\t");
                Serial.print(buff1[i]);
                Serial.println();
            }
        }
        
    }
}

uint32_t printCycles(uint32_t cycleCounterStart, const String inputStr) {
    uint32_t cycleCounterEnd = ARM_DWT_CYCCNT - cycleCounterStart;

    Serial.print(inputStr);
    Serial.println();
    Serial.print("Elapsed clock cycles: ");
    Serial.print(cycleCounterEnd);
    Serial.print("\t| Elapsed microseconds: ");
    Serial.print((double)cycleCounterEnd / (double)F_CPU * (double)1000000); // This assumes constant clock speed because its easier. Use F_CPU_ACTUAL otherwise
    Serial.println();

    return cycleCounterEnd;
}

void ProcessAnalogData(AnalogBufferDMA *pabdma0, AnalogBufferDMA *pabdma1) {
    uint32_t cycleCounterStart = ARM_DWT_CYCCNT; // 32 bit timer that increments every CPU clock cycle
    //uint32_t cycleCounterLast = cycleCounterStart;
    
    volatile uint16_t *pbuffer0 = pabdma0->bufferLastISRFilled();
    volatile uint16_t *end_pbuffer0 = pbuffer0 + pabdma0->bufferCountLastISRFilled();
    volatile uint16_t *pbuffer1 = pabdma1->bufferLastISRFilled();
    volatile uint16_t *end_pbuffer1 = pbuffer1 + pabdma0->bufferCountLastISRFilled();

    boolean print = false;

    if ((uint32_t)pbuffer0 >= 0x20200000u)  arm_dcache_delete((void*)pbuffer0, sizeof(dma_adc_buff1));
    if ((uint32_t)pbuffer1 >= 0x20200000u)  arm_dcache_delete((void*)pbuffer1, sizeof(dma_adc_buff1));

    float32_t workBuffer0[BUFF_SIZE], workBuffer1[BUFF_SIZE], firBuffer0[BUFF_SIZE], firBuffer1[BUFF_SIZE];
    
    // Copy DMA buffers to a work buffer
    copy_from_dma_buff_to_dsp_buff(pbuffer0, end_pbuffer0, firBuffer0, CALIBRATION_OFFSET_0);
    copy_from_dma_buff_to_dsp_buff(pbuffer1, end_pbuffer1, firBuffer1, CALIBRATION_OFFSET_1);

    // -------- Do not use pbuffers after this point. Use work buffers --------


    // Doing it this way does not seem to offer any noticeable speed up
    for(int i = 0; i < BUFF_SIZE / BLOCK_SIZE; i++) {
        arm_fir_f32(&fir_instance_0, /*h0*/  firBuffer0 + (i * BLOCK_SIZE), workBuffer0 + (i * BLOCK_SIZE), BLOCK_SIZE);
        arm_fir_f32(&fir_instance_1, /*h1*/  firBuffer1 + (i * BLOCK_SIZE), workBuffer1 + (i * BLOCK_SIZE), BLOCK_SIZE);
    }


    // Determine if a signal is present / meets the minimum threshold
    for(int i = 0; i < BUFF_SIZE; i++) {
        if(workBuffer0[i] > THRESHOLD || workBuffer1[i] > THRESHOLD) {
            print = true;
            break;
        }
    }
    if(print == false) { return; } // Don't waste time processing data if it doesnt meet the minimum threshold

    //printBuffers(print, BUFF_SIZE, firBuffer0, firBuffer1);
    //printBuffers(print, BUFF_SIZE, workBuffer0, workBuffer1);
    
    // Windows remove some of the errors/artifacts with doing FFTs of arrays of finite length
    applyWindowToBuffer(workBuffer0);
    applyWindowToBuffer(workBuffer1);
    
    //printBuffers(print, BUFF_SIZE, workBuffer0, workBuffer1);

    // Set up for FFT
    // Floats are easier for me to understand than Q15 or Q31 formats. Someone else can implement a Q15 version if they really want.
    // 32 bit float maths should be about the same speed as integer maths on Teensy 4.1. if double precision is required, its half speed of floats
    // We can use real fft here instead of complex fft and cut computation time in half.
    // The input data is all real. The output data is in complex form arr=[real0, imag0, real1, imag1,...]
    // Here is a good example http://gaidi.ca/weblog/configuring-cmsis-dsp-package-and-performing-a-real-fft
    float32_t fftOutput0[BUFF_SIZE], fftOutput1[BUFF_SIZE];

    // ooh math, shiny!!!!
    arm_rfft_fast_f32(&f32_instance0, workBuffer0, fftOutput0, 0);
    arm_rfft_fast_f32(&f32_instance1, workBuffer1, fftOutput1, 0);

    // compute the magnitude and put it in the mag buffers
    float32_t mag0[BUFF_SIZE/2], mag1[BUFF_SIZE/2];
    arm_cmplx_mag_f32(fftOutput0, mag0, BUFF_SIZE/2);
    arm_cmplx_mag_f32(fftOutput1, mag1, BUFF_SIZE/2);

    //printBuffers(print, BUFF_SIZE/2, mag0, mag1);

    // Get the values of magnitudes and the location in the buffer (corresponding frequency)
    float32_t maxVal0, maxVal1;
    uint32_t maxID0, maxID1;
    arm_max_f32(mag0, BUFF_SIZE/2, &maxVal0, &maxID0);
    arm_max_f32(mag1, BUFF_SIZE/2, &maxVal1, &maxID1);

    /*  Compute the cross correlation of the two signals.

     *  rxy(l) = x(n) convolved with y(-n) in time domain
     *  Through some fun properties of math,
     *  rxy(l) = X(w)Y(-w) in freq domain
     * 
     *  This is SIGNIFICANTLY faster than convolution for large array sizes even after 
     *  converting to frequency domain and back to time domain
     */
    float32_t rxy[BUFF_SIZE], rxy_time_domain[BUFF_SIZE];

    complex_array_time_reversal(fftOutput1);
    arm_cmplx_mult_cmplx_f32(fftOutput0, fftOutput1, rxy, BUFF_SIZE/2);

     // Inverse FFT
    arm_rfft_fast_f32(&f32_instance_rxy, rxy, rxy_time_domain, 1);

    //printBuffers(print, BUFF_SIZE, rxy);


    printCycles(cycleCounterStart, "Total Calculation Time");

    Serial.println();
}