/*
===============================================================================
 Name        : hw_check.c
 Author      : Robert
 Created on	 : 06.09.2021
===============================================================================
*/
#include "hw_check.h"

uint32_t loopCounter = 0;
uint8_t pinCnt = 0;
uint8_t pinIdx = 0;
hwc_initdata_t hwcInitData;

uint8_t hwc_nextOutput() {
	uint8_t idx = pinIdx;
	while( hwcInitData.gpios[idx].output == false ) {
		idx++;
		if (idx>=pinCnt) {
			idx = 0;
		}
		if (idx == pinIdx) {
			break;				// no output at all in the gpio array !!!???
		}
	}
	return idx;
}

void hwc_init (hwc_initdata_t *initData) {
	pinCnt = initData->entryCount;
	pinIdx = 0;
	pinIdx = hwc_nextOutput();
}

void hwc_main (void ) {
	loopCounter++;
	if (loopCounter >= 50000) {
		loopCounter = 0;
		GPIO_INIT_T pin = hwcInitData.gpios[pinIdx];
		Chip_GPIO_SetPinOutHigh(LPC_GPIO, pin.port, pin.pinNr);
		pinIdx++;
		pinIdx = hwc_nextOutput();
		pin = hwcInitData.gpios[pinIdx];
		Chip_GPIO_SetPinOutLow(LPC_GPIO, pin.port, pin.pinNr);
	}
}

