/*
===============================================================================
 Name        : hw_check.c
 Author      : Robert
 Created on	 : 06.09.2021
===============================================================================
*/
#include "hw_check.h"

#include "l2_debug_com.h"

pinmux_array_t *hwcInitData;

const PINMUX_GRP_T2* signalPin = 0;
const PINMUX_GRP_T2* monitorPinIn = 0;
const PINMUX_GRP_T2* monitorPinOut = 0;
uint32_t loopCounter = 0;
uint32_t toggleTimer = 1000000;

inline void hwc_init (void *initData) {
	_hwc_init((pinmux_array_t *)initData);
}

void _hwc_init (pinmux_array_t *initData) {
	hwcInitData = initData;
}

void hwc_main (void ) {
	loopCounter++;
	if (loopCounter >= toggleTimer) {
		loopCounter = 0;
		if (signalPin != 0) {
			// toggle signal pin
			Chip_GPIO_SetPinToggle(LPC_GPIO, signalPin->pingrp, signalPin->pinnum);
		}
		if (monitorPinIn != 0) {
			// mirror the monitored pin to output (signalPin)
			bool val = Chip_GPIO_GetPinState(LPC_GPIO, monitorPinIn->pingrp, monitorPinIn->pinnum);
			Chip_GPIO_SetPinState(LPC_GPIO, monitorPinOut->pingrp, monitorPinOut->pinnum, val);
		}
	}

}

void HwcSetOutput(uint8_t idx, hwc_OutStatus stat) {
	if (idx > hwcInitData->entryCount) {
		idx = hwcInitData->entryCount;
	}
	const PINMUX_GRP_T2 *pin = &hwcInitData->pinmux[idx];
	if (IOCON_ISGPIO(pin) && pin->output) {
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
}

void HwcMirrorInput(uint8_t idxIn, uint8_t idxOut) {
	toggleTimer = 0;
	const PINMUX_GRP_T2 *pinOut = 0;
	const PINMUX_GRP_T2 *pinIn = 0;
	if ((idxIn <= hwcInitData->entryCount) && (idxOut <= hwcInitData->entryCount)){
		pinIn = &hwcInitData->pinmux[idxIn];
		if (IOCON_ISGPIO(pinIn) && !pinIn->output) {
			pinOut = &hwcInitData->pinmux[idxOut];
			if (IOCON_ISGPIO(pinOut) && pinOut->output) {
				toggleTimer = 50;
				monitorPinOut = pinOut;
				monitorPinIn = pinIn;
			}
		}
	}
	if (toggleTimer == 0) {
		// Something went wrong with init (out/in idx not fitting -> swith off Mirror feature
		toggleTimer = 50000;
		monitorPinOut = pinOut;
		monitorPinIn = pinIn;
	}

}

