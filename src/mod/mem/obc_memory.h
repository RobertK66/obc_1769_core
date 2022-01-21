/*
===============================================================================
 Name        : obc_memory.h
 Author      : Robert
 Created on	 : 23.12.2021
===============================================================================
*/

#ifndef MOD_MEM_OBC_MEMORY_H_
#define MOD_MEM_OBC_MEMORY_H_

#include <chip.h>

#define MEM_CHANNEL_SDCARD_MASK		(0x80)
#define MEM_CHANNEL_ALLMRAM_MASK	(0x3F)
#define MEM_CHANNEL_MRAM0_MASK		(1 << 0)
#define MEM_CHANNEL_MRAM1_MASK		(1 << 1)
#define MEM_CHANNEL_MRAM2_MASK		(1 << 2)
#define MEM_CHANNEL_MRAM3_MASK		(1 << 3)
#define MEM_CHANNEL_MRAM4_MASK		(1 << 4)
#define MEM_CHANNEL_MRAM5_MASK		(1 << 5)
#define MEM_CHANNEL_MRAM_MASK(chipIdx)	(1 << chipIdx)

typedef struct {
	const PINMUX_GRP_T2*  	sdcPowerPin;
} mem_init_t;

typedef struct {
	uint8_t ChannelErrors;
	uint8_t ChannelBusy;
	uint8_t ChannelAvailble;
	uint8_t ChannelOnOff;
} mem_status_t;

typedef struct {
	uint32_t				lpcChipSerialNumber[4];		// SerialNr of LPC1769 Chip - used as unique ID for any OBC hardware.
	uint32_t        		basisBlockNumber;     		// The raw SDC-Blocknumber to start with OBC Data (Number of Block0 + 1)
	uint32_t        		blocksUsed;    	  	 		// already used Blocks -> makes impossible to add obcs if to much space is already used
    uint32_t        		blocksAvailable;     		// the size of the Partition Part usable for this OBC
} __attribute__((packed)) mem_sdcobcdataarea_t;;

void memInit(void *initdata);
void memMain(void);

void memCardOff(void);
void memCardOn(void);

void memChangeInstanceName(char* name);
void memGetInstanceName(char* name, uint8_t maxLen);

void memChangeCardName(char* name);
void memGetCardName(char* name, uint8_t maxLen);

mem_sdcobcdataarea_t *memGetObcArea();
//void memReadObcBlockAsync(uint32_t blockNr, );

mem_status_t memGetStatus(void);
uint32_t memGetSerialNumber(uint8_t idx);

// Defines for (internal defined) events. Can be used on debug and com APIs as generic 'Event'.
#define MODULE_ID_MEMORY			0x03
#define EVENT_MEM_OPERATIONAL		1
#define EVENT_MEM_BLOCK0_UPDATED	2

#endif /* MOD_MEM_OBC_MEMORY_H_ */
