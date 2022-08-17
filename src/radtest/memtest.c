/*
 * mramtests.c
 *
 *  Created on: 16.08.2022
 *      Author: Robert
 */

#ifndef RADTEST_MRAMTESTS_C_
#define RADTEST_MRAMTESTS_C_

#define RADTST_SEQ_READCHECKS_SECONDS				 15			// Initiate all read checks every n seconds
#define RADTST_SEQ_WRITECHECKS_SECONDS				 60			// Initiate all write checks every n seconds
#define RADTST_SEQ_COUNTERPRINT_SECONDS				120			// Initiate printout of current counters/errors

#define RADTST_MRAM_TARGET_PAGESIZE		MRAM_MAX_WRITE_SIZE			// 1k pages ->
#define RADTST_MRAM_TARGET_PAGES		3 							//(128 * 1024) / MRAM_MAX_WRITE_SIZE		// 128k available
#define RADTST_MRAM_TARGET_OFFSET		100                         // Start of target memory with offset 100 to keep page 0 untouched


#include "memtest.h"

#include <string.h>
#include <stdio.h>

#include "../mod/l2_debug_com.h"

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
//extern uint8_t 				*expectedPagePatternsPtr;   		// points fillpattern bytes[0..3] -> implemented in prgflash2_patterns.c

uint32_t 					radtstTicks = 0;					// Seconds counter

static radtst_counter_t radtstCounter;

static int      currentWriteChip = -1;
static int      chipsToFinishWrite = 0;
static int      currentReadChip = -1;
static int      chipsToFinishRead = 0;
static uint8_t 	curPage[MRAM_CHIP_CNT];
static uint8_t	pageBuffer[MRAM_CHIP_CNT][RADTST_MRAM_TARGET_PAGESIZE + 4];

static char     message[100];


void rtst_writemram();
void RadWriteMramFinished(uint8_t chipIdx,mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);
void RadReadMramFinished(uint8_t chipIdx,mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);

void rtst_memtestinit(void) {
	expectedPagePatternsPtr = &expectedPagePatternsSeq[0];		// Reset the expected Pattern pointer to start value (possible: 0 ...3)
	readEnabled.mram = false;									// We start with read checks disabled. Write checks prepare the expected memory for read checks
	readEnabled.prgFlash = false;
	readEnabled.ram2 = false;
	readEnabled.rtcGpr = false;

	const char *header = "MemCounter; chipIdx; pageWrites; writeErrors; pageReads; pageReadErrors; pageReadBiterrors;\n";
	uint8_t len = strlen(header);
	deb_print_pure_debug((uint8_t *)header, len);

	len = snprintf(message, 100, "Bit Error; chipIdx; page; byteOffset; \n");
	deb_print_pure_debug((uint8_t *)message, len);

	radtstTicks = RADTST_SEQ_WRITECHECKS_SECONDS - 5; // lets start with write checks -> to init all read checks from beginning on. (first tick does not work -> Chips not ready initialized !?
}

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

		for (int i = 0; i<MRAM_CHIP_CNT; i++) {
			curPage[i] = 0;
		}
		currentWriteChip = 0;
		chipsToFinishWrite = 6;
		uint8_t len = snprintf(message, 100, "Init MRAM Write.\n");
		deb_print_pure_debug((uint8_t *)message, len);
	}
	if (currentWriteChip >= 0) {
		// Write current test page chip at position offset + pagenr*pagesize
		uint8_t expByte = expectedPagePatternsPtr[curPage[currentWriteChip] % RADTST_EXPECTED_PATTERN_CNT];
		for (int x =0; x < RADTST_MRAM_TARGET_PAGESIZE; x++) {
			pageBuffer[currentWriteChip][x] = expByte;
		}
		MramWriteAsync(currentWriteChip,RADTST_MRAM_TARGET_OFFSET + curPage[currentWriteChip] * RADTST_MRAM_TARGET_PAGESIZE, pageBuffer[currentWriteChip], RADTST_MRAM_TARGET_PAGESIZE, RadWriteMramFinished);
		currentWriteChip++;	// Wait one tick for next chip.
		if (currentWriteChip >= MRAM_CHIP_CNT) {
			currentWriteChip = -1;
		}
	}
	if ((radtstTicks % RADTST_SEQ_READCHECKS_SECONDS) == 0) {
		if (readEnabled.mram) {
			for (int i = 0; i<MRAM_CHIP_CNT; i++) {
				curPage[i] = 0;
			}
			currentReadChip = 0;
			chipsToFinishRead = 6;
//			uint8_t len = snprintf(message, 100, "Init MRAM Write.\n");
//			deb_print_pure_debug((uint8_t *)message, len);
		}
	}
	if (currentReadChip >=0) {
		// Read the first page for this chip
		MramReadAsync(currentReadChip,RADTST_MRAM_TARGET_OFFSET + curPage[currentReadChip] * RADTST_MRAM_TARGET_PAGESIZE, pageBuffer[currentReadChip], RADTST_MRAM_TARGET_PAGESIZE, RadReadMramFinished);
		currentReadChip++;	// Wait one tick for next chip.
		if (currentReadChip >= MRAM_CHIP_CNT) {
			currentReadChip = -1;
		}
	}

	if ((radtstTicks % RADTST_SEQ_COUNTERPRINT_SECONDS) == 0) {
		for (int i = 0; i<MRAM_CHIP_CNT; i++) {
			uint8_t len = snprintf(message, 100, "MemCounter; %d; %d; %d; %d; %d; %d; \n", i, radtstCounter.mramPageWriteCnt[i], radtstCounter.mramPageWriteError[i],
			                                                                                  radtstCounter.mramPageReadCnt[i],  radtstCounter.mramPageReadError[i],
																						      radtstCounter.mramPageReadBitError[i]);
			deb_print_pure_debug((uint8_t *)message, len);
		}
	}
}





void RadWriteMramFinished(uint8_t chipIdx,mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len) {
	if (result == MRAM_RES_SUCCESS) {
		radtstCounter.mramPageWriteCnt[chipIdx]++;
	} else {
		radtstCounter.mramPageWriteError[chipIdx]++;
	}
	curPage[chipIdx]++;
	if (curPage[chipIdx] < RADTST_MRAM_TARGET_PAGES) {
		uint8_t expByte = expectedPagePatternsPtr[curPage[chipIdx] % RADTST_EXPECTED_PATTERN_CNT];
		for (int x =0; x < RADTST_MRAM_TARGET_PAGESIZE; x++) {
			pageBuffer[chipIdx][x] = expByte;
		}
		MramWriteAsync(chipIdx, RADTST_MRAM_TARGET_OFFSET + (curPage[chipIdx] * RADTST_MRAM_TARGET_PAGESIZE), pageBuffer[chipIdx], RADTST_MRAM_TARGET_PAGESIZE, RadWriteMramFinished );
	} else {
		// All pages written restart. Wait until all chips are finished and then ...
		chipsToFinishWrite--;
		if (chipsToFinishWrite == 0) {
			// ... restart the mram read test
			readEnabled.mram = true;
			uint8_t len = snprintf(message, 100, "MRAM write finished.\n");
			deb_print_pure_debug((uint8_t *)message, len);
		}
	}
}

void CheckPageContent(uint8_t chipIdx, uint8_t *data, uint32_t len, uint8_t expByte, uint32_t *errorCnt) {
	for (int x =0; x < len; x++) {
		if (data[x] != expByte) {
			(*errorCnt)++;		// TODO count really the bits here !?

			// The following can be very verbose. Shall we really do it!?
			uint8_t len = snprintf(message, 100, "Bit Error; %d; %d; %d; \n", chipIdx, curPage[chipIdx], x);
			deb_print_pure_debug((uint8_t *)message, len);
		}
	}
}


void RadReadMramFinished(uint8_t chipIdx,mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len) {
	if (result == MRAM_RES_SUCCESS) {
		radtstCounter.mramPageReadCnt[chipIdx]++;
	} else {
		radtstCounter.mramPageReadError[chipIdx]++;
	}

	uint8_t expByte = expectedPagePatternsPtr[curPage[chipIdx] % RADTST_EXPECTED_PATTERN_CNT];
	// Check read result for bit errors
	CheckPageContent(chipIdx, data, len, expByte, &radtstCounter.mramPageReadBitError[chipIdx]);



	// read next page for this chip
	curPage[chipIdx]++;
	if (curPage[chipIdx] < RADTST_MRAM_TARGET_PAGES) {
		MramReadAsync(chipIdx,RADTST_MRAM_TARGET_OFFSET + curPage[chipIdx] * RADTST_MRAM_TARGET_PAGESIZE, pageBuffer[chipIdx], RADTST_MRAM_TARGET_PAGESIZE, RadReadMramFinished);
	} else {
		// This chip has read all pages
		chipsToFinishRead--;
		if (chipsToFinishRead == 0) {
			uint8_t len = snprintf(message, 100, "MRAM read done.\n");
			deb_print_pure_debug((uint8_t *)message, len);
		}
	}
}

#endif /* RADTEST_MRAMTESTS_C_ */
