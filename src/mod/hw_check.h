/*
===============================================================================
 Name        : hw_check.h
 Author      : Robert
 Created on	 : 06.09.2021
===============================================================================
*/

#ifndef MOD_HW_CHECK_H_
#define MOD_HW_CHECK_H_


#include "../ClimbObc.h"	// We need the Pin structure from here
#include <chip.h>

void hwc_init (pinmux_array_t *initData);
void hwc_main (void);

typedef enum {
	HWC_Default,
	HWC_High,
	HWC_Low,
	HWC_Signal_Slow,
	HWC_Signal_Fast
} hwc_OutStatus;

// Module API
void HwcSetOutput(uint8_t outIdxm, hwc_OutStatus stat);
void HwcMirrorInput(uint8_t idxIn, uint8_t idxOut);

#endif /* MOD_HW_CHECK_H_ */
