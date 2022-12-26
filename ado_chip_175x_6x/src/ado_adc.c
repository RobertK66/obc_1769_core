/*
 * ado_adc.c
 *
 *  Created on: 17.10.2021
 *      Author: Andi
 */

//#include <chip.h>
//#include "stdint.h"
//#include "chip_lpc175x_6x.h"
//#include "adc_17xx_40xx.h"
//#include "stdio.h"
//#include "math.h"
#include "ado_adc.h"

//#define ADC_SUPPLY_CURRENT_CH 0
//#define ADC_SUPPLY_CURRENT_SP_CH 1
//#define ADC_TEMPERATURE_CH 2
//#define ADC_SUPPLY_VOLTAGE_CH 3
//
//#define ADC_VREF 2.048 // Volt


void read_transmit_sensors();

static adc_channel_array_t *chConfig = 0;

void AdcInit(adc_channel_array_t *channels) {
    chConfig = channels;
	ADC_CLOCK_SETUP_T init;
	init.adcRate = 1000;
	init.burstMode = 1;
	Chip_ADC_Init(LPC_ADC, &init);

	for (int i=0;i<channels->activeChannels;i++) {
	    Chip_ADC_EnableChannel(LPC_ADC, i, 1);
	}
	Chip_ADC_SetBurstCmd(LPC_ADC, 1);
	Chip_ADC_SetStartMode(LPC_ADC, 0, 0);
}

float AdcReadChannelResult(uint8_t chnIdx) {
    uint16_t adc_val;
    if (chnIdx >  chConfig->activeChannels) {
        chnIdx = 0;
    }
    Chip_ADC_ReadValue(LPC_ADC, chnIdx, &adc_val);
    return  chConfig->channelConverter[chnIdx].convert(adc_val);
}
