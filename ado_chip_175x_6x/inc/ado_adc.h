/*
 * ado_adc.h
 *
 *  Created on: 18.10.2021
 *      Author: ASI
 */

#ifndef ADO_ADC_H_
#define ADO_ADC_H_

#include <chip.h>

#define ADC_VREF 2.048 // Volt

typedef struct {
    float(*convert)(uint16_t adcValue);
} adc_channel_t;

typedef struct {
    uint8_t activeChannels;
    const adc_channel_t *channelConverter;
} adc_channel_array_t;

void AdcInit(adc_channel_array_t *channels);
float AdcReadChannelResult(uint8_t chnIdx);


#endif /* ADO_ADC_H_ */
