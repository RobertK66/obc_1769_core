/*
===============================================================================
 Name        : l3_sensors.c
 Author      : Robert
 Created on	 : 14.12.2021
===============================================================================
*/

#include "l3_sensors.h"
#include <math.h>
#include <float.h>
#include <chip.h>
#include <ado_crc.h>
#include <ado_adc.h>

#include <ado_modules.h>

#include "ai2c/obc_i2c.h"

typedef sensor_values_t sen_eventarg_meassurement_t;


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

float convertTemperatureSHT30(uint16_t value) {
	return (((((float)value)*175.0)/65535.0) - 45.0);
}

float convertHumidity(uint16_t value) {
	return ((((float)value)*100.0)/65535.0);
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

static sensor_values_t senValues;
static bool readInProgress = false;
static I2C_Data readJob;
static uint8_t readTx[2];
static uint8_t readRx[8];

void sen_init(void *data) {
	AdcInit((adc_channel_array_t *)&Channels);
}

void sen_main() {
	if (readInProgress) {
		if (readJob.job_done == 1) {
			readInProgress = false;
			// get values from ADC pins
			senValues.SupplyCurrentBoard = AdcReadChannelResult(0);
			senValues.SupplyCurrentSidepanels = AdcReadChannelResult(1);
			senValues.Temperature = AdcReadChannelResult(2);
			senValues.SupplyVoltage =  AdcReadChannelResult(3);
			// Check return values from SHT30
			senValues.TempSHT30 = -FLT_MAX;
			senValues.HumidityPercent = -FLT_MAX;

			if (readJob.error == I2C_ERROR_NO_ERROR) {
				uint16_t value = ((uint16_t)readRx[0] << 8) + (uint16_t)readRx[1];
				uint8_t  crc = CRC8(readRx, 2);
				if (crc == readRx[2]) {
					senValues.TempSHT30 = convertTemperatureSHT30(value);
				}
				value = ((uint16_t)readRx[3] << 8) + (uint16_t)readRx[4];
				crc = CRC8(&readRx[3], 2);
				if (crc == readRx[5]) {
					senValues.HumidityPercent = convertHumidity(value);
				}
			}
			SysEvent(MODULE_ID_SENSORS, EVENT_INFO, EID_SEN_MEASSUREMENT, &senValues, sizeof(sensor_values_t));
		}
	}
}

bool ReadSHT30Async();

void SenReadAllValues() {
//	sensor_values_t values;
//	values.SupplyCurrentBoard = AdcReadChannelResult(0);
//	values.SupplyCurrentSidepanels = AdcReadChannelResult(1);
//	values.Temperature = AdcReadChannelResult(2);
//	values.SupplyVoltage =  AdcReadChannelResult(3);
	ReadSHT30Async();
}



bool ReadSHT30Async() {
	if (readInProgress) {
		return false;
	}
	readInProgress = true;

	readJob.device = LPC_I2C1;
	readJob.tx_size = 2;
	readJob.tx_data = readTx;
	readJob.rx_size = 6;
	readJob.rx_data = readRx;
	readJob.adress = 0x44;
	readTx[0] = 0x24;		// Clock Stretching disabled
	readTx[1] = 0x0B;		// Medium Repeatability

	i2c_add_job(&readJob);
	return true;
}

