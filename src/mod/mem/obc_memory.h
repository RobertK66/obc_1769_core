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
	uint8_t ChannelErrors;
	uint8_t ChannelBusy;
	uint8_t ChannelAvailble;
	uint8_t ChannelOnOff;
} memory_status_t;

void memInit(void *dummy);
void memMain(void);

void memChangeInstanceName(char* name);
void memGetInstanceName(char* name, uint8_t maxLen);
memory_status_t memGetStatus(void);

#define MODULE_ID_MEMORY			0x03
#define EVENT_MEM_OPERATIONAL		1
#define EVENT_MEM_BLOCK0_UPDATED	2



#endif /* MOD_MEM_OBC_MEMORY_H_ */
