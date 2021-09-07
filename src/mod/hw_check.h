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

typedef struct {
	uint8_t		 entryCount;
	const GPIO_INIT_T* gpios;
} hwc_initdata_t;

void hwc_init (hwc_initdata_t *initData);
void hwc_main (void);

extern hwc_initdata_t hwcInitData;
static const MODULE_DEF_T hwcModuleDesc = {
		&hwcInitData,
		(void*)hwc_init,
		hwc_main
};

#define HwcInitModule(gpiodef) {								 	\
	hwcInitData.gpios = gpiodef; 									\
	hwcInitData.entryCount = sizeof(gpiodef)/sizeof(GPIO_INIT_T);	\
	hwcModuleDesc.init(&hwcInitData); 								\
}

#define HwcMain() { 		\
	hwcModuleDesc.main(); 	\
}

#endif /* MOD_HW_CHECK_H_ */
