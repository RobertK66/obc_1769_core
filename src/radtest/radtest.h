/*
 * radtest.h
 *
 *  Created on: 08.08.2022
 *      Author: Robert
 */

#ifndef RADTEST_RADTEST_H_
#define RADTEST_RADTEST_H_

#include <chip.h>

typedef struct {
	LPC_TIMER_T*	RadTestTimerPtr;
} rtst_initdata_t;

void rtst_init (void *initData);
void rtst_main (void);

#endif /* RADTEST_RADTEST_H_ */
