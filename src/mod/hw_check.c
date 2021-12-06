/*
===============================================================================
 Name        : hw_check.c
 Author      : Robert
 Created on	 : 06.09.2021
===============================================================================
*/
#include "hw_check.h"

#include "l2_debug_com.h"

uint32_t loopCounter = 0;
uint32_t toggleTimer = 1000000;
uint8_t pinCnt = 0;
hwc_gpioinit_t hwcInitData;
uint8_t outputCnt = 0;
const PINMUX_GRP_T2* hwcOutputs[50];
const PINMUX_GRP_T2* signalPin = 0;

void hwc_init (hwc_gpioinit_t *initData) {
	pinCnt = initData->entryCount;
	for (int i=0; i<pinCnt; i++) {
		if (initData->gpios[i].output == true) {
			hwcOutputs[outputCnt++] = &initData->gpios[i];
		}
	}
}

void hwc_main (void ) {
	loopCounter++;
	if (loopCounter >= toggleTimer) {
		loopCounter = 0;
		if (signalPin != 0) {
			Chip_GPIO_SetPinToggle(LPC_GPIO, signalPin->pingrp, signalPin->pinnum);
		}
	}
}

void HwcSetOutput(uint8_t outIdx, hwc_OutStatus stat) {
	if (outIdx > outputCnt) {
		outIdx = outputCnt - 1;
	}
	const PINMUX_GRP_T2 *pin = hwcOutputs[outIdx];
	signalPin = 0;
	switch (stat) {
		case HWC_Default:
			// Switch to default value
			Chip_GPIO_SetPinState(LPC_GPIO, pin->pingrp, pin->pinnum, pin->initval);
			break;
		case HWC_High:
			Chip_GPIO_SetPinOutHigh(LPC_GPIO, pin->pingrp, pin->pinnum);
			break;
		case HWC_Low:
			Chip_GPIO_SetPinOutLow(LPC_GPIO, pin->pingrp, pin->pinnum);
			break;
		case HWC_Signal_Slow:
			toggleTimer = 50000;
			signalPin = pin;
			break;
		case HWC_Signal_Fast:
			toggleTimer = 500;
			signalPin = pin;
			break;
	}
}
