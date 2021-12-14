/*
===============================================================================
 Name        : l3_sensors.c
 Author      : Robert
 Created on	 : 14.12.2021
===============================================================================
*/

#include "l3_sensors.h"
#include <math.h>
#include <chip.h>
#include "ado_adc.h"


float convertCurrent_c0(uint16_t value) {
    return ((value * (ADC_VREF / 4095) - 0.01) / 100 / 0.075);
}

float convertCurrentSp_c1(uint16_t value) {
    return ((value * (ADC_VREF / 4095) - 0.01) / 100 / 0.075 - 0.0003);
}

float convertTemperature_c2(uint16_t value) {
    float temp = value * (4.9 / 3.9 * ADC_VREF / 4095);
    return (-1481.96 + sqrt(2196200 + (1.8639 - temp) * 257730));
}

float convertSupplyVoltage_c3(uint16_t value) {
    return (value * (2.0 * ADC_VREF / 4095));
}

// Index in static array equals channelNr on ADC
static const adc_channel_t AdcChannels [] = {
   { convertCurrent_c0 },
   { convertCurrentSp_c1 },
   { convertTemperature_c2 },
   { convertSupplyVoltage_c3 }
};
static const adc_channel_array_t Channels = {
    (sizeof(AdcChannels)/sizeof(adc_channel_t)), AdcChannels
};

void sen_init(void *data) {
	AdcInit((adc_channel_array_t *)&Channels);
}

void sen_main() {

}

sensor_values_t SenReadAllValues() {
	sensor_values_t values;
	values.SupplyCurrentBoard = AdcReadChannelResult(0);
	values.SupplyCurrentSidepanels = AdcReadChannelResult(1);
	values.Temperature = AdcReadChannelResult(2);
	values.SupplyVoltage =  AdcReadChannelResult(3);
	return values;
}
