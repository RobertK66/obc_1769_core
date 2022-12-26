/*
 * mram.c
 *
 *  Created on: 27.12.2019
 */
 // Status register of MR25H40
 //   * B7       B6      B5      B4      B3      B2      B1      B0
 //   * SRWD     d.c     d.c     d.c     BP1     BP2     WEL     d.c.
 //   *
 //   * On init all bits should read 0x00.
 //   * The only bit we use here is WEL (Write Enable) and this is reset to 0
 //   * on power up.
 //   * None of the other (protection) bits are used at this moment by this software.
 //  Features for Block protections / Status Write are not coded yet.

#include "mod/ado_mram.h"

#include <chip.h>
#include <string.h>

#include <ado_modules.h>
#include <ado_sspdma.h>
#include <ado_spi.h>


#define ADO_SXX_AddJob(ctx, busNr, txData, rxData, txLen, rxLen, finished, activate) { \
    if (busNr == 99) { \
        ADO_SPI_AddJob(ctx, txData, rxData, txLen , rxLen, (void *)finished,(void *)activate); \
    } else { \
        ADO_SSP_AddJob(ctx, busNr, txData, rxData, txLen , rxLen, finished, activate); \
    } \
}

typedef enum mram_status_e {
	MRAM_STAT_NOT_INITIALIZED,
	MRAM_STAT_READ_SR,
	MRAM_STAT_READ_SR2,
	MRAM_STAT_IDLE,
	MRAM_STAT_RX_INPROGRESS,
	MRAM_STAT_WREN_SET,
	MRAM_STAT_TX_INPROGRESS,
	MRAM_STAT_WREN_CLR,
	MRAM_STAT_ERROR_SIG,		// Error signal-> Mainloop sends Error Event
	MRAM_STAT_ERROR				// TODO: what specific errors are there and what to do now ???? -> reinit SSP ???
} mram_status_t;

typedef struct mram_chip_s {
    ado_sspid_t     busNr;              // The SSP bus (or 99 for SPI) used for this Chip
    volatile bool   busyFlag;           // indicates that we wait for an bus job to finish
    mram_status_t   status;             // The current status of this Chip
    const PINMUX_GRP_T2* csPin;			// void (*chipSelectCb)(bool select);  // Callback to Chip Select GPIO
    void (*ioFinishedCb)(uint8_t chipIdx, mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);

    uint8_t         *ioDataPtr;         // Pointer to calllers original memory
    uint32_t        ioAdr;              // Callers original address
    uint32_t        ioLen;              // Callers original IO-LEN

    uint8_t         mramData[MRAM_MAX_WRITE_SIZE + 4];  // Data buffer needs 4 bytes additional to write data.
    uint8_t         tx[4];              // Command buffer
    uint8_t         rx[1];              // Rx (Status byte) buffer
} mram_chip_t;


// Event Data structs
typedef struct {
	uint8_t   chipIdx;
	uint8_t   status;
} mram_event_error_t;


static mram_chip_t MramChipStates[MRAM_CHIP_CNT];

//// prototypes
void MramActivate(uint32_t context);
void MramAddJobReadStatusRegister(mram_chip_t *mramPtr);
void MramJobFinished(uint32_t context, ado_sspstatus_t jobStatus, uint8_t *rxData, uint16_t rxSize);


// Be careful here! This Callback is sometimes called from IRQ !!!
// Do not do any complicated logic here!!!
inline void MramActivate(uint32_t context){
	Chip_GPIO_SetPinState(LPC_GPIO, ((mram_chip_t *)context)->csPin->pingrp,
			                        ((mram_chip_t *)context)->csPin->pinnum,
									! ((mram_chip_t *)context)->csPin->initval);  // here is a '!' ;-) !!!
}

// Be careful here! This Callback is called from IRQ !!!
// Do not do any complicated logic here!!!
void MramJobFinished(uint32_t context, ado_sspstatus_t jobStatus, uint8_t *rxData, uint16_t rxSize) {
    mram_chip_t * mramPtr = (mram_chip_t *)context;

    // Unselect CS by returning to its initval.
    Chip_GPIO_SetPinState(LPC_GPIO, mramPtr->csPin->pingrp, mramPtr->csPin->pinnum, mramPtr->csPin->initval);
    mramPtr->busyFlag = false;
    if (jobStatus != ADO_SSP_JOBDONE) {
    	mramPtr->status = MRAM_STAT_ERROR_SIG;
    }

//   TODO: Error handling on Job Level.....
//    } else {
//    	mram_event_joberror_t evtdata;
//    	evtdata.jobStatus = jobStatus;
//    	evtdata.statusPrev = mramPtr->status;
//
//    	// Set Status to error and trigger SysEvent
//        mramPtr->status = MRAM_STAT_ERROR_SIG;
//        SysEvent(MODULE_ID_MRAM, EVENT_ERROR, EID_MRAM_JOBERROR, &evtdata, sizeof(mram_event_joberror_t) );
//    }

}

inline void MramInitAll(void *chips) {
	_MramInitAll((mram_chipinit_array_t *)chips);
}
void _MramInitAll(mram_chipinit_array_t *chips) {
	uint8_t cnt = chips->entryCount;
	if (cnt > MRAM_CHIP_CNT) {
		cnt = MRAM_CHIP_CNT;
	}
	for (int i = 0; i < cnt; i++ ){
		mram_chip_t *mramPtr =  &MramChipStates[i];
		mramPtr->status = MRAM_STAT_NOT_INITIALIZED;
		mramPtr->csPin = chips->chipinits[i].csPin;
		mramPtr->busNr = chips->chipinits[i].busnr;
	}
}

inline void MramAddJobReadStatusRegister(mram_chip_t *mramPtr) {
	mramPtr->busyFlag = true;
	mramPtr->tx[0] = 0x05;
    mramPtr->rx[0] = 0xFF;
    ADO_SXX_AddJob((uint32_t)mramPtr, mramPtr->busNr, mramPtr->tx, mramPtr->rx, 1, 1, (void *)MramJobFinished , (void *)MramActivate);
}


void MramMain() {
    for (uint8_t chipIdx = 0; chipIdx < MRAM_CHIP_CNT; chipIdx++) {
        mram_chip_t *mramPtr =  &MramChipStates[chipIdx];

        if (mramPtr->busyFlag) {
            // TODO: timouts checken
        } else {
        	if (mramPtr->status == MRAM_STAT_NOT_INITIALIZED) {
        		// Send Status register Read Command
        		mramPtr->status = MRAM_STAT_READ_SR;
        		MramAddJobReadStatusRegister(mramPtr);
        	} else if (mramPtr->status == MRAM_STAT_READ_SR) {
        		// We have to make another read to get the SR (see Datasheet Rev 12.6,8/2020 Table 2 Footnot 1!!!)
        		mramPtr->status = MRAM_STAT_READ_SR2;
        		MramAddJobReadStatusRegister(mramPtr);
          	} else if (mramPtr->status == MRAM_STAT_READ_SR2) {
          		mramPtr->status = MRAM_STAT_IDLE;
           		// Check received status -> idle or error
        		if ((mramPtr->rx[0] & 0xFD) != 0x00) {
        			mramPtr->status = MRAM_STAT_ERROR;
        			// Send error event
        			mram_event_error_t evtdata;
        			evtdata.chipIdx = chipIdx;
        			evtdata.status = mramPtr->rx[0];
        			SysEvent(MODULE_ID_MRAM, EVENT_ERROR, EID_MRAM_JOBERROR, &evtdata, sizeof(mram_event_error_t) );
        		}
         	} else if (mramPtr->status == MRAM_STAT_RX_INPROGRESS) {
                // Rx job finished.
                mramPtr->status = MRAM_STAT_IDLE;
                mramPtr->ioFinishedCb(chipIdx, MRAM_RES_SUCCESS, mramPtr->ioAdr, mramPtr->ioDataPtr, mramPtr->ioLen);
            } else if (mramPtr->status == MRAM_STAT_WREN_SET) {
                // Write enable set job finished
                //Initiate write data job
                mramPtr->mramData[0] = 0x02;
                mramPtr->mramData[1] = (mramPtr->ioAdr & 0x00010000) >> 16;
                mramPtr->mramData[2] = (mramPtr->ioAdr & 0x0000ff00) >> 8;
                mramPtr->mramData[3] = (mramPtr->ioAdr & 0x000000ff);
                memcpy(&mramPtr->mramData[4],mramPtr->ioDataPtr, mramPtr->ioLen);           // TODO: we could make a Scatter DMA with Tx1-TX2 to solve this aditional copy .and the need of duplicate buffer !!!!....
                                                                                            // 
                mramPtr->busyFlag = true;
                ADO_SXX_AddJob((uint32_t)mramPtr, mramPtr->busNr, mramPtr->mramData, NULL,mramPtr->ioLen+4, 0, (void *)MramJobFinished , (void *)MramActivate);
                mramPtr->status = MRAM_STAT_TX_INPROGRESS;

            } else if (mramPtr->status == MRAM_STAT_TX_INPROGRESS) {
                // Write data job finished
                // Initiate Write Disabled job
                mramPtr->tx[0] = 0x04;
                mramPtr->busyFlag = true;
                ADO_SXX_AddJob((uint32_t)mramPtr, mramPtr->busNr, mramPtr->tx, NULL, 1, 0, (void *)MramJobFinished , (void *)MramActivate);
                mramPtr->status = MRAM_STAT_WREN_CLR;

            } else if (mramPtr->status == MRAM_STAT_WREN_CLR) {
                // Write disable job finished. This finally ends the Write-IO
                mramPtr->status = MRAM_STAT_IDLE;
                mramPtr->ioFinishedCb(chipIdx, MRAM_RES_SUCCESS, mramPtr->ioAdr, mramPtr->ioDataPtr, mramPtr->ioLen);
            } else if (mramPtr->status == MRAM_STAT_ERROR_SIG) {
            	mram_event_error_t evtdata;
            	evtdata.chipIdx = chipIdx;

				// Set Status to error and trigger SysEvent
				mramPtr->status = MRAM_STAT_ERROR;
				SysEvent(MODULE_ID_MRAM, EVENT_ERROR, EID_MRAM_JOBERROR, &evtdata, sizeof(mram_event_error_t) );
            }
        }
    }   // END FOR all chips.
}

mram_res_t MramIsChipItialized(uint8_t chipIdx) {
	mram_chip_t *mramPtr = 0;
	if (chipIdx < MRAM_CHIP_CNT) {
		mramPtr =  &MramChipStates[chipIdx];
	} else {
		return MRAM_RES_INVALID_CHIPIDX;
	}
	if (mramPtr->status == MRAM_STAT_IDLE) {
	  return MRAM_RES_SUCCESS;
	}
	  return MRAM_RES_NOTINITIALIZED;
}

void MramReadAsync(uint8_t chipIdx, uint32_t adr,  uint8_t *rx_data,  uint32_t len, void (*finishedHandler)(uint8_t chipIdx,mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len)) {
    mram_chip_t *mramPtr = 0;
    if (chipIdx < MRAM_CHIP_CNT) {
        mramPtr =  &MramChipStates[chipIdx];
    } else {
        finishedHandler(chipIdx, MRAM_RES_INVALID_CHIPIDX, adr, 0, len);
        return;
    }

    if (mramPtr->status != MRAM_STAT_IDLE) {
		finishedHandler(chipIdx, MRAM_RES_BUSY, adr, 0, len);
		return;
	}

	if (rx_data == NULL) {
		finishedHandler(chipIdx, MRAM_RES_DATA_PTR_INVALID, adr, 0, len);
		return;
	}

	if (len > MRAM_MAX_READ_SIZE) {
		finishedHandler(chipIdx, MRAM_RES_RX_LEN_OVERFLOW, adr, 0, len);
		return;
	}

	if (adr + len >  0x020000) {
		finishedHandler(chipIdx, MRAM_RES_INVALID_ADR,  adr, 0, len);
		return;
	}

	/*--- Read Data Bytes Command --- */
	mramPtr->tx[0] = 0x03;
	mramPtr->tx[1] = (adr & 0x00010000) >> 16;
	mramPtr->tx[2] = (adr & 0x0000ff00) >> 8;
	mramPtr->tx[3] = (adr & 0x000000ff);

	mramPtr->busyFlag = true;
	mramPtr->status = MRAM_STAT_RX_INPROGRESS;
	mramPtr->ioFinishedCb = finishedHandler;
	mramPtr->ioAdr = adr;
	mramPtr->ioDataPtr = rx_data;
	mramPtr->ioLen = len;

	ADO_SXX_AddJob((uint32_t)mramPtr, mramPtr->busNr, mramPtr->tx, rx_data, 4, len, (void *)MramJobFinished , (void *)MramActivate);
	return;
}

void MramWriteAsync(uint8_t chipIdx, uint32_t adr, uint8_t *data, uint32_t len,  void (*finishedHandler)(uint8_t chipIdx,mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len)) {
    mram_chip_t *mramPtr = 0;
    if (chipIdx < MRAM_CHIP_CNT) {
        mramPtr =  &MramChipStates[chipIdx];
    } else {
        finishedHandler(chipIdx, MRAM_RES_INVALID_CHIPIDX, adr, 0, len);
        return;
    }

    if (mramPtr->status != MRAM_STAT_IDLE) {
		finishedHandler(chipIdx, MRAM_RES_BUSY, adr, 0, len);
		return;
	}

	if (data == NULL) {
		finishedHandler(chipIdx, MRAM_RES_DATA_PTR_INVALID, adr, 0, len);
		return;
	}

	if (len > MRAM_MAX_READ_SIZE) {
		finishedHandler(chipIdx, MRAM_RES_RX_LEN_OVERFLOW, adr, 0, len);
		return;
	}

	if (adr + len >  0x020000) {
		finishedHandler(chipIdx, MRAM_RES_INVALID_ADR,  adr, 0, len);
		return;
	}

	/*--- SetWrtite enable Command --- */

	mramPtr->tx[0] = 0x06;

	mramPtr->busyFlag = true;
	mramPtr->status = MRAM_STAT_WREN_SET;
	mramPtr->ioFinishedCb = finishedHandler;
	mramPtr->ioAdr = adr;
	mramPtr->ioDataPtr = data;
	mramPtr->ioLen = len;

	ADO_SXX_AddJob((uint32_t)mramPtr, mramPtr->busNr, mramPtr->tx, NULL, 1, 0, (void *)MramJobFinished , (void *)MramActivate);
	return;
}

