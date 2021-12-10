/*
===============================================================================
 Name        : hw_check.h
 Author      : Robert
 Created on	 : 06.09.2021
===============================================================================
*/

#ifndef MOD_HW_CHECK_H_
#define MOD_HW_CHECK_H_

#include "../temp-Base.h"
#include <chip.h>

typedef struct {
	uint8_t		 entryCount;
	const PINMUX_GRP_T2* pinmux;
} hwc_gpioinit_t;

void hwc_init (hwc_gpioinit_t *initData);
void hwc_main (void);

extern hwc_gpioinit_t hwcInitData;
static const MODULE_DEF_T hwcModuleDesc = {
		(void*)hwc_init,
		hwc_main
};

#define HwcInitModule(pinmux2) {								 	\
	hwcInitData.pinmux = pinmux2; 									\
	hwcInitData.entryCount = sizeof(pinmux2)/sizeof(PINMUX_GRP_T2);	\
	hwcModuleDesc.init(&hwcInitData); 								\
}

#define HwcMain() { 		\
	hwcModuleDesc.main(); 	\
}

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
