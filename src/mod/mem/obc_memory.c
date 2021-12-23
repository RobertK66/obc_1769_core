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

#include <ado_modules.h>
#include <mod/ado_mram.h>
#include <mod/ado_sdcard.h>


#define C_SDCOBC_BLOCK0_MARKER (0x1A1B1CEB)

typedef enum {
	MEMST_INIT,
	MEMST_INITIALIZING,
	MEMST_SDCARDONLY_OK,
	MEMST_MRAMONLY_DEGRADED,
	MEMST_MRAMONLY_OK,
	MEMST_FULLOK,
} mem_state_t;

typedef enum {
	SW_DEVELOPER = 0,
	SW_PRODUCTION
} mem_softwaretype_t;

typedef struct {
	mem_softwaretype_t	type;
	uint8_t	major;
	uint8_t	minor;
	uint8_t	patch;
} mem_software_version;

typedef struct mem_struct_1 mem_location_entry_t;
typedef struct mem_struct_1 {
	uint8_t		chipMask;			// not used for sdc
	uint8_t		type;
	uint16_t	length;
	uint32_t	baseAdress;			// blockNr
	mem_location_entry_t *nextLocation;
} mem_location_entry_t;

typedef struct {
	uint32_t				pageMarker;
	char					hwInstancename[16];
	mem_software_version 	softwareVersion;
	uint32_t				resetCount;
	mem_location_entry_t	*locationTable;
} mem_page0_t;

typedef struct {
	uint32_t				blockMarker;					// 0x00 	0
	uint32_t				lpcChipSerialNumber[4];			// 0x04		4
	char					hwInstancename[16];				// 0x14    20
	mem_software_version 	softwareVersion;				// 0x24    36
	mem_location_entry_t	*locationTableA;				// 0x28	   40
	mem_location_entry_t	*locationTableB;				// 0x2C	   44
	uint8_t					filler[462];					// 0x30	   48
	uint16_t         		signature;             			// 0x1FE   		 '0xaa55'
} mem_block0_t;
// This dummy typedef checks if the structure size is really equal to our SDC BLOCK  size of 512
typedef char pCheckBlock0[(sizeof(mem_block0_t) == 512) ? 1 : -1];

typedef struct partitionInfo_Structure {
    uint8_t         status;             //0x80 - active partition
    uint8_t         headStart;          //starting head
    uint16_t        cylSectStart;       //starting cylinder and sector.
    uint8_t         type;               //partition type
    uint8_t         headEnd;            //ending head of the partition
    uint16_t        cylSectEnd;         //ending cylinder and sector
    uint32_t        firstSector;        //total sectors between MBR & the first sector of the partition
    uint32_t        sectorsTotal;       //size of this partition in sectors
} __attribute__((packed)) partitionInfo_t;

typedef struct MBRinfo_Structure {
    unsigned char    nothing[440];          // ignore, placed here to fill the gap in the structure
    uint32_t         devSig;                // Device Signature (for Windows)
    uint16_t         filler;                // 0x0000
    partitionInfo_t  partitionData[4];      // partition records (16x4)
    uint16_t         signature;             // 0xaa55
} __attribute__((packed))  MBRinfo_t ;

//Structure to access volume boot sector data (its sector 0 on unpartitioned devices or the first sector of a partition)
typedef struct bootSector_S {
    uint8_t jumpBoot[3];
    uint8_t OEMName[8];
    // FAT-BIOS Param Block common part (DOS2.0)
    uint16_t bytesPerSector;             // default: 512
    uint8_t  sectorPerCluster;           // 1,2,4,8,16,...
    uint16_t reservedSectorCount;        // 32 for FAT32 ?
    uint8_t  numberofFATs;               // 2 ('almost always')
    uint16_t rootEntryCount;             // 0 for FAT32
    uint16_t totalSectors_F16;           // 0 for FAT32
    uint8_t mediaType;
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
} __attribute__((packed)) bootSector_t;

typedef struct {
	bool	sdcOperational;
	uint8_t	mramOperationalMask;
	char	instanceName[16];
} mem_eventarg_operational_t;

typedef enum {
	BL0_UPD_FIRTSINITIALIZED = 0,
	BL0_UPD_INSTANCENAME_CHANGED,
	BL0_UPD_SERIALNUMBER_CHANGED,		// most prop: SDCard was moved from other Hardwareinstance...
} mem_bl0_updateReason_t;

typedef struct {
	mem_bl0_updateReason_t reason;
} mem_eventarg_bl0updated_t;

void memInitializeMram(void);
void memInitializeSdCard(void);
void CreateFreshSdcBlock0(mem_block0_t *pBlock0);

void memBlock0Updated(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len);

static mem_state_t   memStatus;

static bool				block0Valid = false;
static mem_block0_t 	sdCardBlock0;
static uint32_t 		partitionSize;

static bool 			block0NeedsUpdate = false;
static sdcard_block512 	tempBlock;
static uint32_t 		tempBlockNr;
static mem_bl0_updateReason_t bl0Reason;

static bool			 page0Valid = false;
static mem_page0_t   mramPage0;


void memInit(void *dummy) {
	// we have to wait for all other modules to work, so we move initialization to main.....
	memStatus = MEMST_INIT;
}


void memMain(void) {

	if (memStatus == MEMST_INIT) {
		// Wait for SDCard to be initialized....
		if (SdcIsCardinitialized(0)) {
			memStatus = MEMST_INITIALIZING;
			memInitializeMram();
			memInitializeSdCard();
		}
	}
	if (memStatus == MEMST_INITIALIZING) {
		if (block0Valid) {
			memStatus = MEMST_SDCARDONLY_OK;
			mem_eventarg_operational_t args;
			memcpy(args.instanceName, sdCardBlock0.hwInstancename, 16);
			args.sdcOperational = true;
			args.mramOperationalMask = 0x00;
			SysEvent(MODULE_ID_MEMORY, EVENT_INFO, EVENT_MEM_OPERATIONAL, &args, sizeof(mem_eventarg_operational_t));
		}
		if (page0Valid) {
			memStatus = MEMST_MRAMONLY_OK;
		}
	}
	if (memStatus == MEMST_SDCARDONLY_OK) {
		// TODO: mram init page 0
	}
	if (memStatus == MEMST_MRAMONLY_OK) {
		if (block0Valid) {
			memStatus = MEMST_FULLOK;
			mem_eventarg_operational_t args;
			memcpy(args.instanceName, sdCardBlock0.hwInstancename, 16);
			args.sdcOperational = true;
			args.mramOperationalMask = 0x00;
			SysEvent(MODULE_ID_MEMORY, EVENT_INFO, EVENT_MEM_OPERATIONAL, &args, sizeof(mem_eventarg_operational_t));
		}
	}

	if (block0NeedsUpdate) {
		block0NeedsUpdate = false;
		SdcWriteBlockAsync(0, tempBlockNr, &tempBlock, memBlock0Updated);
	}

}

void memInitializeMram(void) {

}


void memBootBlockRead(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len);
void memBlock0Read(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len);
void memInitializeSdCard(void) {
	SdcReadBlockAsync(0, 0, tempBlock.data, memBootBlockRead);
}
void memBootBlockRead(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len){
	if (result == SDC_RES_SUCCESS) {
		// Check if the SDCard is formated as we wish it to be for OBC Usage
		// We expect either a Master Boot Record to be located in SD Cards Block0 or a 'prepared' Boot record.
		if ( data[0x1fe] == 0x55 && data[0x1ff] == 0xAA ) {		// MBR/Boot Marker
			 if (data[0] == 0xE9 || data[0] == 0xEB	) {			// There is no MBR we have a single Boot-Block at Sector 0
				 // Check if this is already our own Block0
				 if (((mem_block0_t *)data)->blockMarker == C_SDCOBC_BLOCK0_MARKER) {
					 // This is our own Boot0 block!!!
					 // TODO: aditional checks .... e.g. Serial Nr of Chip -> sd card moved from one HW to another .....
					 memcpy(&sdCardBlock0, data, sizeof(sdCardBlock0));
					 block0Valid = true;
				 } else {
					// seems to be some other FATxx Boot block
					// TODO: Check if it is prepared to be used by us (e.g. by Partiton name or so....)
					block0NeedsUpdate = true;
					bl0Reason = BL0_UPD_FIRTSINITIALIZED;
					CreateFreshSdcBlock0((mem_block0_t *)tempBlock.data);
				 }
			 } else {											// This seems to be a MasterBoot record
				MBRinfo_t *p = (MBRinfo_t *)data;
				// We ignore Partition 0 - this can be any normal Windows FAT or whatever we want to be available on OBC SDCards....
				// OBC Partition always uses Partition-1 of possible 0..3
				if (p->partitionData[1].firstSector > 0) {
					tempBlockNr = p->partitionData[1].firstSector;
					partitionSize = p->partitionData[1].sectorsTotal;

					SdcReadBlockAsync(0, tempBlockNr, tempBlock.data, memBootBlockRead);
				} else {
					// TODO: Event "SDCARD no partition '1' found.
				}
			 }
		} else {
			// TODO: Event "SDCARD no MBR/Boot resord(s) found
		}
	} else {
		// TODO: Event "SDCARD error!"
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

void CreateFreshSdcBlock0(mem_block0_t *pBlock0) {
	unsigned int iap_param_table[5];
	unsigned int iap_result_table[5];

	memset(pBlock0, 0, sizeof(mem_block0_t));
	pBlock0->blockMarker = C_SDCOBC_BLOCK0_MARKER;
	// Get Chip Serial Number from IAP
	iap_param_table[0] = 58; 		//IAP command
	iap_entry(iap_param_table,iap_result_table);
	if(iap_result_table[0] == 0) { 	//return: CODE SUCCESS
		pBlock0->lpcChipSerialNumber[0] = iap_result_table[1];
		pBlock0->lpcChipSerialNumber[1] = iap_result_table[2];
		pBlock0->lpcChipSerialNumber[2] = iap_result_table[3];
		pBlock0->lpcChipSerialNumber[3] = iap_result_table[4];
	} else {
		pBlock0->lpcChipSerialNumber[0] = 0x00000000;
		pBlock0->lpcChipSerialNumber[1] = 0x01010101;
		pBlock0->lpcChipSerialNumber[2] = 0x02020202;
		pBlock0->lpcChipSerialNumber[3] = 0x03030303;
	}
	// Initial Instance Name uses Chips serial Number to be unique.
	strcpy(pBlock0->hwInstancename,"OBC-");
	itoa(pBlock0->lpcChipSerialNumber[1], &(pBlock0->hwInstancename[4]), 16);
	pBlock0->locationTableA = 0;
	pBlock0->locationTableB = 0;
	pBlock0->softwareVersion.type = SW_DEVELOPER;
	pBlock0->softwareVersion.major = 0;
	pBlock0->softwareVersion.minor = 0;
	pBlock0->softwareVersion.patch = 0;
	pBlock0->signature = 0xaa55;					// Lets mark our Bloc0 as 'Boot Block'....
}

void memChangeInstanceName(char* name) {
	if (block0Valid) {
		memcpy(tempBlock.data, &sdCardBlock0, sizeof(sdCardBlock0));
		mem_block0_t *p = (mem_block0_t *)tempBlock.data;
		memset(p->hwInstancename, 0, 16);
		strcpy(p->hwInstancename, name);
		block0NeedsUpdate = true;
		bl0Reason = BL0_UPD_INSTANCENAME_CHANGED;
	}
}
