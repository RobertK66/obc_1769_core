/*
 * mramtest.h
 *
 *  Created on: 16.08.2022
 *      Author: Robert
 */

#ifndef RADTEST_MRAMTEST_H_
#define RADTEST_MRAMTEST_H_

#include <chip.h>
#include <mod/ado_mram.h>

#define RADTST_PRGFLASH2_TARGET_PAGES 		8
#define RADTST_PRGFLASH2_TARGET_PAGESIZE 1024

#define RADTST_EXPECTED_PATTERN_CNT				4
extern uint8_t expectedPagePatternsSeq[2 * RADTST_EXPECTED_PATTERN_CNT];
extern uint8_t prgFlash2Target[RADTST_PRGFLASH2_TARGET_PAGES][RADTST_PRGFLASH2_TARGET_PAGESIZE];

typedef struct radtst_counter_s {				// only add uint32_t values (its printed as uint32_t[] !!!
//	uint32_t rtcgprTestCnt;
//	uint32_t signatureCheckCnt;
//	uint32_t ram2PageReadCnt;
//	uint32_t ram2PageWriteCnt;
	uint32_t mramPageReadCnt[MRAM_CHIP_CNT];
	uint32_t mramPageWriteCnt[MRAM_CHIP_CNT];
//	uint32_t i2cmemPageReadCnt;
//	uint32_t i2cmemPageWriteCnt;


//	uint32_t expSignatureChanged;				// This should stay on RADTST_FLASHSIG_PARTS (first time read after reset).
//	uint32_t expRam2BytesChanged;
//
//	uint32_t signatureCheckBlocked;
//	uint32_t signatureRebaseBlocked;
//	uint32_t rtcgprTestErrors;
//	uint32_t signatureErrorCnt;
//	uint32_t ram2PageReadError;
//	uint32_t ram2PageWriteError;
	uint32_t mramPageReadError[MRAM_CHIP_CNT];
	uint32_t mramPageWriteError[MRAM_CHIP_CNT];
	uint32_t mramPageReadBitError[MRAM_CHIP_CNT];
//	uint32_t i2cmemPageReadError;
//	uint32_t i2cmemPageWriteError;

} radtst_counter_t;


void rtst_memtesttick(void);
void rtst_memtestinit(void);

#endif /* RADTEST_MRAMTEST_H_ */
