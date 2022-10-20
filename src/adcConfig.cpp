#include "adcConfig.h"

ADC *adc = new ADC();

AnalogBufferDMA abdma1(dma_adc_buff1, BUFF_SIZE, dma_adc_buff2, BUFF_SIZE);
AnalogBufferDMA abdma2(dma_adc_buff2_1, BUFF_SIZE, dma_adc_buff2_2, BUFF_SIZE);

void initADC() {
    // Settings for ADC0
    adc->adc0->setAveraging(ADC_AVGS);
    adc->adc0->setResolution(ADC_RES);
    adc->adc0->setConversionSpeed(ADC_CNV_SPD);
    adc->adc0->setSamplingSpeed(ADC_SA_SPD);
    adc->adc0->recalibrate();

    // Settings for ADC1
    adc->adc1->setAveraging(ADC_AVGS);
    adc->adc1->setResolution(ADC_RES);
    adc->adc1->setConversionSpeed(ADC_CNV_SPD);
    adc->adc1->setSamplingSpeed(ADC_SA_SPD);
    adc->adc1->recalibrate();

    // Initialize DMA channels
    abdma1.init(adc, ADC_0/*, DMAMUX_SOURCE_ADC_ETC*/);
    abdma2.init(adc, ADC_1/*, DMAMUX_SOURCE_ADC_ETC*/);

    // Start the dma operation..
    adc->adc0->startSingleRead(ADC_PIN_0); // call this to setup everything before the Timer starts, differential is also possible
    adc->adc1->startSingleRead(ADC_PIN_1);

    // Set the ADCs to use a timer
    adc->adc0->startTimer(SAMPLE_RATE); // samples/sec
    adc->adc1->startTimer(SAMPLE_RATE); // samples/sec
}