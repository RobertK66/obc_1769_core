/*
 * mramtests.c
 *
 *  Created on: 16.08.2022
 *      Author: Robert
 */

#ifndef RADTEST_MRAMTESTS_C_
#define RADTEST_MRAMTESTS_C_

#define RADTST_SEQ_READCHECKS_SECONDS					15			// Initiate all read checks every n seconds
#define RADTST_SEQ_WRITECHECKS_SECONDS				   600			// Initiate all write checks every n seconds

#define RADTST_MRAM_TARGET_PAGESIZE		MRAM_MAX_WRITE_SIZE			// 1k pages ->
#define RADTST_MRAM_TARGET_PAGES		3 							//(128 * 1024) / MRAM_MAX_WRITE_SIZE		// 128k available



#include "memtest.h"

#include <mod/ado_mram.h>

typedef struct radtst_readcheckenabled_s {
	unsigned int 	prgFlash 	:1;				// not implemented yet
	unsigned int  	rtcGpr		:1;				// not implemented yet
	unsigned int  	ram2		:1;				// not implemented yet
	unsigned int  	mram		:1;				// trying to do
	unsigned int  	      :1;
	unsigned int  	      :1;
	unsigned int  	      :1;
	unsigned int  	      :1;
} radtst_readcheckenabled_t;

typedef struct radtst_workload_s {
	// Read processes running (async to mainloop call)
	unsigned int radtest_caclulate_flashsig 	:1;		//???
	unsigned int  	:1;
	unsigned int  	:1;
	unsigned int  	:1;
	unsigned int  	:1;
	unsigned int  	:1;
	unsigned int  	:1;
	unsigned int  	:1;

	// Write processes running (async to mainloop call)
	unsigned int  	:1;
	unsigned int  	:1;
	unsigned int  	:1;
	unsigned int  	:1;
	unsigned int  	:1;
	unsigned int  	:1;
	unsigned int  	:1;
	unsigned int  	:1;

	// Rebase processes running (async to mainloop call)
	unsigned int  radtest_rebase_flashsig		:1;			//???
	unsigned int  	:1;
	unsigned int  	:1;
	unsigned int  	:1;
	unsigned int  	:1;
	unsigned int  	:1;
	unsigned int  	:1;
	unsigned int  	:1;
} radtest_radtst_workload_t;


typedef enum radtst_sources_e {
	RADTST_SRC_PRGFLASH2,				// The upper half of program flash (0x00040000 - 0x0007FFFF)
	RADTST_SRC_RTCGPR,					// 20 bytes (5Words) general purpose registers in RTC (battery buffered)
	RADTST_SRC_RAM2,					// The 'upper' (unused) RAM Bank (0x2007C000 - 0x20084000(?))
	RADTST_SRC_MRAM,

	RADTST_SRC_UNKNOWN = 128
} radtst_sources_t;


radtst_readcheckenabled_t	readEnabled;						// read test do run and check for expected results
radtest_radtst_workload_t   busyBits;							// used to block mainloop operations until async (read writes) are done ....

uint8_t 					*expectedPagePatternsPtr; 			// points to fillpattern bytes[0..7] (in flash)
extern uint8_t 				*expectedPagePatternsPtr;   		// points fillpattern bytes[0..3] -> implemented in prgflash2_patterns.c

uint32_t 					radtstTicks = 0;					// Seconds counter

void rtst_memtestinit(void) {
	expectedPagePatternsPtr = &expectedPagePatternsSeq[0];		// Reset the expected Pattern pointer to start value (possible: 0 ...3)
	readEnabled.mram = false;									// We start with read checks disabled. Write checks prepare the expected memory for read checks
	readEnabled.prgFlash = false;
	readEnabled.ram2 = false;
	readEnabled.rtcGpr = false;
}

void rtst_writemram();
void RadWriteMramFinished(uint8_t chipIdx,mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);

void rtst_memtesttick(void) {
	radtstTicks++;
	if ((radtstTicks % RADTST_SEQ_WRITECHECKS_SECONDS) == 0) {
		// switch to next test pattern
		expectedPagePatternsPtr++;
		if (expectedPagePatternsPtr > &expectedPagePatternsSeq[3]) {
			expectedPagePatternsPtr = &expectedPagePatternsSeq[0];
		}
		readEnabled.mram = false;			// Disable all read checks until wirites are done
		readEnabled.prgFlash = false;
		readEnabled.ram2 = false;
		readEnabled.rtcGpr = false;

		//rtst_writemram();
	}
}

static uint8_t curPage[MRAM_CHIP_CNT];
uint8_t pageBuffer[MRAM_CHIP_CNT][RADTST_MRAM_TARGET_PAGESIZE + 4];

void rtst_writemram() {

	// Write the new Patterns to all mram chips
	for (int i = 0; i<MRAM_CHIP_CNT; i++) {
		curPage[i] = 1;		// Do not start at address 0 as there is already OBC 'Page0 data' stored there!
		uint8_t expByte = expectedPagePatternsPtr[curPage[i] % RADTST_EXPECTED_PATTERN_CNT];
		for (int x =0; x < RADTST_MRAM_TARGET_PAGESIZE; x++) {
			pageBuffer[i][x] = expByte;
		}
		MramWriteAsync(i, curPage[i] * RADTST_MRAM_TARGET_PAGESIZE, pageBuffer[i], RADTST_MRAM_TARGET_PAGESIZE, RadWriteMramFinished);
	}
	//printf("MRAM Write all pages started %ld\n", radtstCounter.mramPageWriteCnt);
	//radtstCounter.mramPageWriteCnt++;
}


void RadWriteMramFinished(uint8_t chipIdx,mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len) {
	if (result != MRAM_RES_SUCCESS) {
//		radtstCounter.mramPageWriteError++;
		//printf("MRAM write bus error %d.\n", result);
//      TODO: keine Ahnung, was hier besser/wahrscheinlich besser ist/wäre :-((((
//		readEnabled.mram = true;		// Maybe this will work later ...
//		return;							// but to continue with another page write makes no sense !?
	}
	curPage[chipIdx]++;

	if (curPage[chipIdx] < RADTST_MRAM_TARGET_PAGES) {
		//radtstCounter.mramPageWriteCnt++;
		uint8_t expByte = expectedPagePatternsPtr[curPage[chipIdx] % RADTST_EXPECTED_PATTERN_CNT];
		for (int x =0; x < RADTST_MRAM_TARGET_PAGESIZE; x++) {
			pageBuffer[chipIdx][x] = expByte;
		}
		MramWriteAsync(chipIdx, curPage[chipIdx] * RADTST_MRAM_TARGET_PAGESIZE, pageBuffer[chipIdx], RADTST_MRAM_TARGET_PAGESIZE, RadWriteMramFinished );
	} else {
		// All pages written restart read tests
		// readEnabled.mram = true if all Chip Idx have written all pages;
		//printf("MRAM Write all pages ended %ld.\n", radtstCounter.mramPageWriteCnt);
	}
}


#endif /* RADTEST_MRAMTESTS_C_ */
