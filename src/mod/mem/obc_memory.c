/*
===============================================================================
 Name        : obc_memory.c
 Author      : Robert
 Created on	 : 23.12.2021
===============================================================================
*/

#include "obc_memory.h"
#include <stdlib.h>
#include <string.h>

#include <ado_crc.h>
#include <ado_modules.h>
#include <mod/ado_mram.h>
#include <mod/ado_sdcard.h>

#include "../tim/obc_time.h"

#define C_MEM_BLOCK0_MARKER (0x1B1B1CEB)
#define C_MEM_PAGE0_MARKER (0x1A1B1CEB)

typedef enum {
	MEMST_NOT_INITIALIZED,
	MEMST_INIT_MRAM,
	MEMST_WAIT_SDCARD_POWERON,
	MEMST_INIT_SDCARD,
	MEMST_IDLE
} mem_state_t;

typedef enum {
	SW_DEVELOPER = 0,
	SW_PRODUCTION
} mem_softwaretype_t;

typedef enum {
	BL0_UPD_FIRTSINITIALIZED = 0,
	BL0_UPD_CARDNAME_CHANGED,
	BL0_UPD_SERIALNUMBER_CHANGED,		// most prop: SDCard was moved from other Hardwareinstance...
} mem_bl0_updateReason_t;

typedef struct {
	mem_softwaretype_t	type;
	uint8_t	major;
	uint8_t	minor;
	uint8_t	patch;
} __attribute__((packed)) mem_software_version;

typedef struct mem_struct_1 mem_location_entry_t;
typedef struct mem_struct_1 {
	uint8_t		chipMask;			// not used for sdc
	uint8_t		type;
	uint16_t	length;
	uint32_t	baseAdress;			// blockNr
	mem_location_entry_t *nextLocation;
}  __attribute__((packed)) mem_location_entry_t;

typedef struct {
	uint32_t				pageMarker;
	uint32_t				dummy;
	char					hwInstancename[16];
	mem_software_version 	softwareVersion;
	uint32_t				resetCount;
	mem_location_entry_t	*locationTable;
	uint16_t				crc16;
} __attribute__((packed)) mem_page0_t;

//typedef struct {
//	uint32_t				lpcChipSerialNumber[4];		// SerialNr of LPC1769 Chip - used as unique ID for any OBC hardware.
//	uint32_t        		basisBlockNumber;     		// The raw SDC-Blocknumber to start with OBC Data (Number of Block0 + 1)
//	uint32_t        		blocksUsed;    	  	 		// already used Blocks -> makes impossible to add obcs if to much space is already used
//    uint32_t        		blocksAvailable;     		// the size of the Partition Part usable for this OBC
//} __attribute__((packed)) mem_sdcobcdataarea_t;;

typedef struct {
	uint32_t				blockMarker;					// 0x00
	char					cardName[16];					// 0x04
	mem_sdcobcdataarea_t    obcDataAreaDescritpion[4];		// we allow data areas for up to 4 OBC hardware instances to be persisted on one SD Card.
	uint8_t					filler[378];
	uint16_t         		signature;             			// 0x1FE   		 '0xaa55'
} __attribute__((packed)) mem_block0_t;
// This dummy typedef checks if the structure size is really equal to our SDC BLOCK  size of 512
typedef char pCheckBlock0[(sizeof(mem_block0_t) == 512) ? 1 : -1];

typedef struct {
    uint8_t         status;             //0x80 - active partition
    uint8_t         headStart;          //starting head
    uint16_t        cylSectStart;       //starting cylinder and sector.
    uint8_t         type;               //partition type
    uint8_t         headEnd;            //ending head of the partition
    uint16_t        cylSectEnd;         //ending cylinder and sector
    uint32_t        firstSector;        //total sectors between MBR & the first sector of the partition
    uint32_t        sectorsTotal;       //size of this partition in sectors
} __attribute__((packed)) mem_partitioninfo_t;

typedef struct {
    unsigned char    nothing[440];          // ignore, placed here to fill the gap in the structure
    uint32_t         devSig;                // Device Signature (for Windows)
    uint16_t         filler;                // 0x0000
    mem_partitioninfo_t  partitionData[4];  // partition records (16x4)
    uint16_t         signature;             // 0xaa55
} __attribute__((packed))  mem_mbrinfo_t ;

//Structure to access volume boot sector data (its sector 0 on unpartitioned devices or the first sector of a partition)
typedef struct {
    uint8_t jumpBoot[3];
    uint8_t OEMName[8];
    // FAT-BIOS Param Block common part (DOS2.0)
    uint16_t bytesPerSector;             // default: 512
    uint8_t  sectorPerCluster;           // 1,2,4,8,16,...
    uint16_t reservedSectorCount;        // 32 for FAT32 ?
    uint8_t  numberofFATs;               // 2 ('almost always')
    uint16_t rootEntryCount;             // 0 for FAT32
    uint16_t totalSectors_F16;           // 0 for FAT32
    uint8_t  mediaType;
    uint16_t FATsize_F16;                // 0 for FAT32
    // DOS3.31 extensions
    uint16_t sectorsPerTrack;
    uint16_t numberofHeads;
    uint32_t hiddenSectors;
    uint32_t totalSectors_F32;
    // FAT32 Extended Bios Param Block
    uint32_t FATsize_F32;               // count of sectors occupied by one FAT
    uint16_t extFlags;
    uint16_t FSversion;                 // 0x0000 (defines version 0.0)
    uint32_t rootCluster;               // first cluster of root directory (=2)
    uint16_t FSinfo;                    // sector number of FSinfo structure (=1)
    uint16_t BackupBootSector;
    uint8_t  reserved[12];
    // FAT16 Extended Bios Param Block (offset moved by F32 Extensions)
    uint8_t  driveNumber;
    uint8_t  reserved1;
    uint8_t  bootSignature;
    uint32_t volumeID;
    uint8_t  volumeLabel[11];           // "NO NAME "
    uint8_t  fileSystemType[8];         // "FAT32"
    uint8_t  bootData[420];
    uint16_t signature;                 // 0xaa55
} __attribute__((packed)) mem_bootsector_t;

typedef struct {
	bool	sdcOperational;
	uint8_t	mramOperationalMask;
	char	instanceName[16];
	char	cardName[16];
}  __attribute__((packed)) mem_eventarg_operational_t;

typedef struct {
	mem_bl0_updateReason_t reason;
} __attribute__((packed)) mem_eventarg_bl0updated_t;


// local Prototypes
void memInitializeMram(void);
void memInitializeSdCard(void);
void memCreateFreshBlock0(mem_block0_t *pBlock0, uint32_t baseBlock, uint32_t partionSize);
void memCreateFreshPage0(mem_page0_t *pPage0);

void memPage0Updated(uint8_t chipIdx, mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);
void memBlock0Updated(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len);

// Module Variables
static mem_init_t 	*memInitPtr;
static uint32_t 	 	memChipSerialNumber[4] = {0x01010101, 0x02020202, 0x03030303, 0x04040404};
static mem_state_t   	memStatus;
static mem_status_t  channelStatus;

static uint16_t			memWaitLoops = 0;
static bool				block0Valid = false;
static mem_block0_t 	block0;
static uint32_t 		tempBlockNr;
static uint32_t 		partitionSize;
static bool 			block0NeedsUpdate = false;
static sdcard_block512 	tempBlock;
static mem_sdcobcdataarea_t *memObcDataArea = 0;


static mem_bl0_updateReason_t bl0Reason;


#define C_MAX_MRAMRECORD_SIZE (sizeof(mem_page0_t) + 8)		// replace with largest record size used in MRAM .....
static bool			page0Valid = false;
static mem_page0_t  page0;
static uint8_t		tempRxTxData[6][C_MAX_MRAMRECORD_SIZE];	// Data Area available per Chip-Idx.
static uint8_t		tempChipFinishedOk;
static uint8_t		tempChipFinishedError;
static uint8_t		page0NeedsUpdate = 0;
static int8_t		page0ValidCount = 0;


#define MEM_WRITE_PAGE0(mask) { \
	uint16_t crc = CRC16_XMODEM((uint8_t *)&page0, sizeof(mem_page0_t) - 2); \
	page0.crc16 = (crc <<8) | (crc>>8); \
	page0NeedsUpdate = mask; } \


mem_sdcobcdataarea_t *memGetObcArea(void) {
	return memObcDataArea;
}

uint32_t memGetSerialNumber(uint8_t idx) {
	return memChipSerialNumber[idx&0x03];
}

mem_status_t memGetStatus() {
	return channelStatus;
}

void memGetCardName(char* name, uint8_t maxLen) {
	memset(name, 0, maxLen);
	strncpy(name, "<Unknown>", maxLen);
	if (block0Valid) {
		strncpy(name, block0.cardName, maxLen);
	}
}

void memGetInstanceName(char* name, uint8_t maxLen) {
	memset(name, 0, maxLen);
	strncpy(name, "<Unknown>", maxLen);
	if (page0Valid) {
		strncpy(name, page0.hwInstancename, maxLen);
	}
}

void memInit(void *initdata) {
	memInitPtr = (mem_init_t *)initdata;

	// Read Serial Number of thid LPC1769 chip as unique hardware ID
	unsigned int iap_param_table[5];
	unsigned int iap_result_table[5];

	iap_param_table[0] = 58; 		//IAP command
	iap_entry(iap_param_table,iap_result_table);
	if(iap_result_table[0] == 0) { 	//return: CODE SUCCESS
		memChipSerialNumber[0] = iap_result_table[1];
		memChipSerialNumber[1] = iap_result_table[2];
		memChipSerialNumber[2] = iap_result_table[3];
		memChipSerialNumber[3] = iap_result_table[4];
	}

	// we have to wait for all other modules to work, so we move initialization to main.....
	// we start with switchend on SDCard
	SdcCardinitialize(0);
	memStatus = MEMST_NOT_INITIALIZED;
	channelStatus.ChannelAvailble = 0;
	channelStatus.ChannelBusy = 0;
	channelStatus.ChannelErrors = 0;
	channelStatus.ChannelOnOff = MEM_CHANNEL_SDCARD_MASK | MEM_CHANNEL_ALLMRAM_MASK;
	page0NeedsUpdate = 0;
	page0ValidCount = 0;
}

void memSendEventOperational() {
	mem_eventarg_operational_t args;
	args.sdcOperational = block0Valid;
	args.mramOperationalMask = page0ValidCount;
	if (block0Valid) {
		memcpy(args.cardName, block0.cardName, 16);
	}
	if (page0Valid) {
		memcpy(args.instanceName, page0.hwInstancename, 16);
	}
	SysEvent(MODULE_ID_MEMORY, EVENT_INFO, EVENT_MEM_OPERATIONAL, &args, sizeof(mem_eventarg_operational_t));
}

void memMain(void) {
	if(memWaitLoops>0) {
		memWaitLoops--;
	}
	if (memStatus == MEMST_NOT_INITIALIZED) {
		if ( MramIsChipItialized(5) == MRAM_RES_SUCCESS ) {
			memStatus = MEMST_INIT_MRAM;
			//memInitializeSdCard();
			memInitializeMram();
		}
	} else if(memStatus == MEMST_INIT_MRAM) {
		if ((page0Valid) && SdcIsCardinitialized(0)) {
			memStatus = MEMST_INIT_SDCARD;
			memInitializeSdCard();
		}

	} else if(memStatus == MEMST_WAIT_SDCARD_POWERON) {
		if (memWaitLoops == 0) { // or SD Card not available ....
			SdcCardinitialize(0);
			memStatus = MEMST_INIT_MRAM;
		}
	} else if(memStatus == MEMST_INIT_SDCARD) {
		if (block0Valid) { // or SD Card not available ....
			memStatus = MEMST_IDLE;
			memSendEventOperational();
		}
	}
	if (page0NeedsUpdate != 0x00) {
		uint8_t mask=0x01;
		for (int chipIdx=0;chipIdx<6;chipIdx++) {
			if (page0NeedsUpdate & mask) {
				MramWriteAsync(chipIdx, 0, (uint8_t*)&page0, sizeof(mem_page0_t), memPage0Updated);
			}
		}
	}
	if (block0NeedsUpdate) {
		block0NeedsUpdate = false;
		SdcWriteBlockAsync(0, tempBlockNr, &tempBlock, memBlock0Updated);
	}
	if (tempChipFinishedOk == 0x3F) {
		tempChipFinishedOk = 0x00;
		channelStatus.ChannelAvailble |= 0x03F;
		// all MRAM page0 reads where successful now
		if (page0ValidCount >= 1) {
			// we had at least one valid Page with  ok and no disagreement over content...
			page0Valid = true;
		} else if (page0ValidCount < 0) {
			// TODO: there was disagreement over which one of the blocks is  the correct one ....
			// at this moment without any more thinking let's take the one with the highest reset count....
			mem_page0_t *validPtr = 0;
			for (int chipIdx = 0; chipIdx < 6; chipIdx++) {
				mem_page0_t *p = (mem_page0_t *)tempRxTxData[chipIdx];
				if (p->pageMarker == C_MEM_PAGE0_MARKER) {
					if (validPtr == 0) {
						validPtr = p;
						page0NeedsUpdate =  (~(1<<chipIdx)) & 0x3F;
					} else {
						if (p->resetCount > validPtr->resetCount) {
							validPtr = p;
							page0NeedsUpdate =  (~(1<<chipIdx)) & 0x3F;
						}
					}
				}
			}
			if (validPtr != 0) {
				page0Valid = true;
				memcpy(&page0, validPtr, sizeof(mem_page0_t));
			}
		} else {
			// Not one valid blocks found. Create a new one from scratch and write it to all Chips
			page0Valid = true;
			memCreateFreshPage0(&page0);
		}
		// If we already got our epoch number (aka reset count) from RTC-GP registers then we update the mram Page0 with it.
		uint32_t epochRtc = tim_getEpochNumber();
		if (epochRtc > page0.resetCount) {
			page0.resetCount = epochRtc;
			MEM_WRITE_PAGE0(0x3F);
		} else {
			// If rtc had lower or no epoch number we use the one we got from mram now.
			tim_setEpochNumber(page0.resetCount);
		}
	}
}

void memPage0Updated(uint8_t chipIdx, mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len) {
	uint8_t chipMask = ~(0x01 << chipIdx);

	if (result == MRAM_RES_SUCCESS) {
		page0NeedsUpdate &= chipMask;
	} else {
		// TODO:?? retry counter / timeouts ....
	}
}

void memPage0Read(uint8_t chipIdx, mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len) {
	uint8_t chipMask = 1 << chipIdx;

	if (result == MRAM_RES_SUCCESS) {
		tempChipFinishedOk |= chipMask;
		mem_page0_t *page0Ptr = (mem_page0_t*)data;
		if (page0Ptr->pageMarker == C_MEM_PAGE0_MARKER) {
			uint16_t crc = CRC16_XMODEM((uint8_t*)data, sizeof(mem_page0_t));
			if (crc == 0x0000) {
				if (page0ValidCount == 0) {
					// first valid page goes into our varaible
					memcpy(&page0, page0Ptr, sizeof(mem_page0_t));
					page0ValidCount++;
				} else {
					// Compare next result with already received
					int isSame = memcmp(&page0, page0Ptr, sizeof(mem_page0_t));
					if (isSame == 0) {
						// OK another agreement on this page.
						page0ValidCount++;
					} else {
						// Mhhh what to do now. Who wins in the end !?
						page0ValidCount = -1;
					}
				}
			} else {
				// reason: wrong crc
				page0NeedsUpdate |= chipMask;
			}
		} else {
			// reason no block marker here
			page0NeedsUpdate |= chipMask;
		}
	} else {
		// SPI Errors !?
		// TODO: MRAM Error ..... How to work with degraded Hardware..... How to try to re-init .....
		tempChipFinishedError |= chipMask;
	}
}

void memInitializeMram(void) {
	// Page0 is located in all 6 MRAM Chips. Let's read them all....
	page0NeedsUpdate = 0x00;
	tempChipFinishedOk = 0x00;
	page0ValidCount = 0;
	MramReadAsync(0, 0, &(tempRxTxData[0][0]), sizeof(mem_page0_t), memPage0Read);
	MramReadAsync(1, 0, &(tempRxTxData[1][0]), sizeof(mem_page0_t), memPage0Read);
	MramReadAsync(2, 0, &(tempRxTxData[2][0]), sizeof(mem_page0_t), memPage0Read);
	MramReadAsync(3, 0, &(tempRxTxData[3][0]), sizeof(mem_page0_t), memPage0Read);
	MramReadAsync(4, 0, &(tempRxTxData[4][0]), sizeof(mem_page0_t), memPage0Read);
	MramReadAsync(5, 0, &(tempRxTxData[5][0]), sizeof(mem_page0_t), memPage0Read);
}


void memBootBlockRead(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len);
void memBlock0Read(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len);
void memInitializeSdCard(void) {
	SdcReadBlockAsync(0, 0, tempBlock.data, memBootBlockRead);
}
void memBootBlockRead(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len){
	if (result == SDC_RES_SUCCESS) {
		channelStatus.ChannelAvailble |= MEM_CHANNEL_SDCARD_MASK;
		// Check if the SDCard is formated as we wish it to be for OBC Usage
		// We expect either a Master Boot Record to be located in SD Cards Block0 or a 'prepared' Boot record.
		if ( data[0x1fe] == 0x55 && data[0x1ff] == 0xAA ) {		// MBR/Boot Marker
			 if (data[0] == 0xE9 || data[0] == 0xEB	) {			// There is no MBR we have a single Boot-Block at Sector 0
				 // Check if this is already our own Block0
				 if (((mem_block0_t *)data)->blockMarker == C_MEM_BLOCK0_MARKER) {
					 // This is our own Block0!
					 // TODO: additional checks .... e.g. Serial Nr of Chip -> sd card moved from one HW to another .....
					 memcpy(&block0, data, sizeof(block0));
					 memObcDataArea = &block0.obcDataAreaDescritpion[0];
					 block0Valid = true;
				 } else {
					// seems to be some other FATxx Boot block
					// TODO: Check if it is prepared to be used by us (e.g. by Partiton name or so....)
					//mem_bootsector_t *p = (mem_bootsector_t *)data;
					//if (p->volumeLabel[0] == 'O') {	// Has top start with O e.g. OBC ;-) Ort check for old versions of OBC block marker to
											 		// Update to new one .....
						block0NeedsUpdate = true;
						bl0Reason = BL0_UPD_FIRTSINITIALIZED;
						memCreateFreshBlock0((mem_block0_t *)tempBlock.data, tempBlockNr, partitionSize);
					//} else {
						// SDC not usable - wrong format
					//	channelStatus.ChannelErrors |= MEM_CHANNEL_SDCARD_MASK;
					//}
				 }
			 } else {
				// This seems to be a MasterBoot record
				mem_mbrinfo_t *p = (mem_mbrinfo_t *)data;
				// We ignore Partition 0 - this can be any normal Windows FAT or whatever we want to be available on OBC SDCards....
				// OBC Partition always uses Partition-1 of possible 0..3
				if (p->partitionData[1].firstSector > 0) {
					tempBlockNr = p->partitionData[1].firstSector;
					partitionSize = p->partitionData[1].sectorsTotal;
					SdcReadBlockAsync(0, tempBlockNr, tempBlock.data, memBootBlockRead);
				} else {
					// TODO: Event "SDCARD no partition '1' found.
					channelStatus.ChannelErrors |= MEM_CHANNEL_SDCARD_MASK;

				}
			 }
		} else {
			// TODO: Event "SDCARD no MBR/Boot record(s) found

		}
	} else {
		// TODO: Event "SDCARD error!"
		channelStatus.ChannelErrors |= MEM_CHANNEL_SDCARD_MASK;
		channelStatus.ChannelAvailble &= ~MEM_CHANNEL_SDCARD_MASK;
	}
}


void memBlock0Updated(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len) {
	if (result == SDC_RES_SUCCESS) {
		mem_eventarg_bl0updated_t args;
		args.reason = bl0Reason;
		SysEvent(MODULE_ID_MEMORY, EVENT_INFO, EVENT_MEM_BLOCK0_UPDATED, &args, sizeof(mem_eventarg_bl0updated_t));
		SdcReadBlockAsync(0, tempBlockNr, tempBlock.data, memBootBlockRead);
	} else {
		// TODO: Event "SDCARD error!"
	}
}

void memCreateFreshBlock0(mem_block0_t *pBlock0, uint32_t block0Nr, uint32_t partitionSize) {
	memset(pBlock0, 0, sizeof(mem_block0_t));
	pBlock0->blockMarker = C_MEM_BLOCK0_MARKER;
	// Initial Instance Name uses Chips serial Number to be unique.
	strcpy(pBlock0->cardName,"Card-");
	itoa(memChipSerialNumber[2], &(pBlock0->cardName[5]), 16);

	// We enter this OBC in the first Area of 4 available.
	pBlock0->obcDataAreaDescritpion[0].lpcChipSerialNumber[0] = memChipSerialNumber[0];
	pBlock0->obcDataAreaDescritpion[0].lpcChipSerialNumber[1] = memChipSerialNumber[1];
	pBlock0->obcDataAreaDescritpion[0].lpcChipSerialNumber[2] = memChipSerialNumber[2];
	pBlock0->obcDataAreaDescritpion[0].lpcChipSerialNumber[3] = memChipSerialNumber[3];
	pBlock0->obcDataAreaDescritpion[0].basisBlockNumber = block0Nr;
	pBlock0->obcDataAreaDescritpion[0].blocksAvailable = partitionSize;
	pBlock0->obcDataAreaDescritpion[0].blocksUsed = 1;

	pBlock0->signature = 0xaa55;					// Our Bloc0 is also marked as 'Boot Block'
}

void memCreateFreshPage0(mem_page0_t *pPage0) {
	memset(pPage0, 0, sizeof(mem_page0_t));
	pPage0->pageMarker = C_MEM_PAGE0_MARKER;
	// Initial Instance Name uses Chips serial Number to be unique.
	strcpy(pPage0->hwInstancename,"MRAM-");
	itoa(memChipSerialNumber[1], &(pPage0->hwInstancename[5]), 16);
	pPage0->resetCount = tim_getEpochNumber();
	MEM_WRITE_PAGE0(0x3f);
//	uint16_t crc = CRC16_XMODEM((uint8_t*)pPage0, sizeof(mem_page0_t) - 4);
//	pPage0->crc16 = (crc <<8) | (crc>>8);
}


void memChangeCardName(char* name) {
	if (block0Valid) {
		memcpy(tempBlock.data, &block0, sizeof(block0));
		mem_block0_t *p = (mem_block0_t *)tempBlock.data;
		memset(p->cardName, 0, 16);
		strncpy(p->cardName, name, 16);
		block0NeedsUpdate = true;
		bl0Reason = BL0_UPD_CARDNAME_CHANGED;
	}
}

void memChangeInstanceName(char* name) {
	if (page0Valid) {
		memset(page0.hwInstancename, 0, 16);
		strncpy(page0.hwInstancename, name, 16);
		MEM_WRITE_PAGE0(0x3f);
//		uint16_t crc = CRC16_XMODEM((uint8_t*)&page0, sizeof(mem_page0_t) - 2);
//		page0.crc16 = (crc <<8) | (crc>>8);
//		//uint16_t crcCheck = CRC16_XMODEM((uint8_t*)&page0, sizeof(mem_page0_t));
//		page0NeedsUpdate = 0x3F;
	}
}

void memCardOn(void) {
	Chip_GPIO_SetPinOutLow(LPC_GPIO, memInitPtr->sdcPowerPin->pingrp, memInitPtr->sdcPowerPin->pinnum);
	memStatus = MEMST_WAIT_SDCARD_POWERON;
	memWaitLoops = 1000;
	channelStatus.ChannelOnOff |= MEM_CHANNEL_SDCARD_MASK;
	channelStatus.ChannelAvailble &= ~MEM_CHANNEL_SDCARD_MASK;
	channelStatus.ChannelBusy &= ~MEM_CHANNEL_SDCARD_MASK;
	channelStatus.ChannelErrors &= ~MEM_CHANNEL_SDCARD_MASK;	//TODO: ??? keep this
	block0Valid = false;
	//SdcCardinitialize(0);
}

void memCardOff(void) {
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, memInitPtr->sdcPowerPin->pingrp, memInitPtr->sdcPowerPin->pinnum);
	channelStatus.ChannelOnOff &= ~MEM_CHANNEL_SDCARD_MASK;
}
