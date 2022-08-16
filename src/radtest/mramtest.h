/*
 * mramtest.h
 *
 *  Created on: 16.08.2022
 *      Author: Robert
 */

#ifndef RADTEST_MRAMTEST_H_
#define RADTEST_MRAMTEST_H_

#include <chip.h>

#define RADTST_PRGFLASH2_TARGET_PAGES 		8
#define RADTST_PRGFLASH2_TARGET_PAGESIZE 1024

#define RADTST_EXPECTED_PATTERN_CNT				4
extern uint8_t expectedPagePatternsSeq[2 * RADTST_EXPECTED_PATTERN_CNT];
extern uint8_t prgFlash2Target[RADTST_PRGFLASH2_TARGET_PAGES][RADTST_PRGFLASH2_TARGET_PAGESIZE];

void rtst_mramtick(void);

#endif /* RADTEST_MRAMTEST_H_ */
