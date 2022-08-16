/*
 * mramtests.c
 *
 *  Created on: 16.08.2022
 *      Author: Robert
 */

#ifndef RADTEST_MRAMTESTS_C_
#define RADTEST_MRAMTESTS_C_

#include "mramtest.h"

uint8_t 		*expectedPagePatternsPtr; 						// points to fillpattern bytes[0..7] (in flash)
extern uint8_t 	*expectedPagePatternsPtr; 						// points fillpattern bytes[0..3]


void rtst_memtestinit(void) {
	expectedPagePatternsPtr = &expectedPagePatternsSeq[0];		// Reset the expected Pattern pointer to start value (possible: 0 ...3)

}


void rtst_mramtick(void) {

}

#endif /* RADTEST_MRAMTESTS_C_ */
