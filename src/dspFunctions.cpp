#include "dspFunctions.h"

float32_t window[BUFF_SIZE];

void ProcessAnalogData(AnalogBufferDMA *pabdma0, AnalogBufferDMA *pabdma1, int8_t *delayVal) {
    uint32_t cycleCounterStart = ARM_DWT_CYCCNT; // 32 bit timer that increments every CPU clock cycle
    //uint32_t cycleCounterLast = cycleCounterStart;
    
    volatile uint16_t *pbuffer0 = pabdma0->bufferLastISRFilled();
    volatile uint16_t *end_pbuffer0 = pbuffer0 + pabdma0->bufferCountLastISRFilled();
    volatile uint16_t *pbuffer1 = pabdma1->bufferLastISRFilled();
    volatile uint16_t *end_pbuffer1 = pbuffer1 + pabdma0->bufferCountLastISRFilled();

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
    boolean meetsThreshold = false;

    for(int i = 0; i < BUFF_SIZE; i++) {
        if(workBuffer0[i] > THRESHOLD || workBuffer1[i] > THRESHOLD) {
            meetsThreshold = true;
            break;
        }
    }
    if(meetsThreshold == false) { return; } // Don't waste time processing data if it doesnt meet the minimum threshold

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

    // Calculate forward FFT. We only need the real part which is slightly faster
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

    // Get Y(-w)
    complex_array_time_reversal(fftOutput1);

    // rxy(l) = X(w)Y(-w)
    arm_cmplx_mult_cmplx_f32(fftOutput0, fftOutput1, rxy, BUFF_SIZE/2);

    // Inverse FFT
    arm_rfft_fast_f32(&f32_instance_rxy, rxy, rxy_time_domain, 1);

    //printBuffers(print, BUFF_SIZE, rxy);

    uint32_t delay_long_u = 0;
    float32_t valAtDelay = 0;

    // Find the maximum and the delay at that maximum
    arm_max_f32(rxy_time_domain, BUFF_SIZE, &valAtDelay, &delay_long_u);
    
    // Middle of the buffer is the delay == 0
    int32_t delay_long = delay_long_u - (BUFF_SIZE/2);

    if (delay_long > 127) {
        *delayVal = 127;
    }
    else if (delay_long < -128) {
        *delayVal = -128;
    }
    else {
        *delayVal = (int8_t)delay_long;
    }

    printCycles(cycleCounterStart, "Total Calculation Time");
    Serial.println();
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