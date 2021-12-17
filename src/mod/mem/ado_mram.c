/*
 * mram.c
 *
 *  Created on: 27.12.2019
 */

#include "ado_mram.h"

#include <chip.h>
#include <string.h>

#include "../ado_modules.h"
#include <ado_sspdma.h>
#include <ado_spi.h>

typedef enum mram_status_e {
	MRAM_STAT_NOT_INITIALIZED,
	MRAM_STAT_INITIALIZED,
	MRAM_STAT_IDLE,
	MRAM_STAT_RX_INPROGRESS,
	MRAM_STAT_WREN_SET,
	MRAM_STAT_TX_INPROGRESS,
	MRAM_STAT_WREN_CLR,
	MRAM_STAT_ERROR							// TODO: what specific errors are there and what to do now ???? -> reinit SSP ???
} mram_status_t;

typedef struct mram_chip_s {
    ado_sspid_t     busNr;              // The SSP bus used for this Chip
    volatile bool   busyFlag;           // indicates that we wait for an bus job to finish
    mram_status_t   status;             // The current status of this Chip
    void (*chipSelectCb)(bool select);  // Callback to Chip Select GPIO
    void (*ioFinishedCb)(mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);

    uint8_t         *ioDataPtr;         // Pointer to calllers original memory
    uint32_t        ioAdr;              // Callers original address
    uint32_t        ioLen;              // Callers original IO-LEN

    uint8_t         mramData[MRAM_MAX_WRITE_SIZE + 4];  // Data buffer needs 4 bytes additional to write data.
    uint8_t         tx[4];              // Command buffer
    uint8_t         rx[1];              // Rx (Status byte) buffer
} mram_chip_t;


// Event Data structs
typedef struct {
	ado_sspstatus_t jobStatus;
	mram_status_t   statusPrev;
} mram_event_joberror_t;



static mram_chip_t MramChipStates[MRAM_CHIP_CNT];

//// prototypes
//void ReadMramCmd(int argc, char *argv[]);
//void WriteMramCmd(int argc, char *argv[]);
//void ReadMramFinished (mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);
//void WriteMramFinished (mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len);

//
// Be careful here! This Callback is sometimes called from IRQ !!!
// Do not do any complicated logic here!!!
void MramActivate(uint32_t context){
    ((mram_chip_t *)context)->chipSelectCb(true);
}

// Be careful here! This Callback is called from IRQ !!!
// Do not do any complicated logic here!!!
void MramJobFinished(uint32_t context, ado_sspstatus_t jobStatus, uint8_t *rxData, uint16_t rxSize) {
    mram_chip_t * mramPtr = (mram_chip_t *)context;
    mramPtr->chipSelectCb(false);
    mramPtr->busyFlag = false;
    if (jobStatus == ADO_SSP_JOBDONE) {
        if (mramPtr->status == MRAM_STAT_NOT_INITIALIZED) {
            if ((mramPtr->rx[0] & 0xFD) != 0x00) {
                /* Error -  Status Value not as expected */
                mramPtr->status = MRAM_STAT_ERROR;
                return;
            }
            mramPtr->status = MRAM_STAT_INITIALIZED;
        }
    } else {
    	mram_event_joberror_t evtdata;
    	evtdata.jobStatus = jobStatus;
    	evtdata.statusPrev = mramPtr->status;

    	// Set Status to error and trigger SysEvent
        mramPtr->status = MRAM_STAT_ERROR;
        SysEvent(MODULE_ID_MRAM, EVENT_ERROR, EID_MRAM_JOBERROR, &evtdata, sizeof(mram_event_joberror_t) );
    }
}

void MramInit(uint8_t chipIdx, ado_sspid_t busNr, void(*csHandler)(bool select)) {
    if (chipIdx < MRAM_CHIP_CNT) {
        mram_chip_t *mramPtr =  &MramChipStates[chipIdx];
        mramPtr->status = MRAM_STAT_NOT_INITIALIZED;
        mramPtr->chipSelectCb = csHandler;
        mramPtr->busNr = busNr;

        //	/* Init mram  read Status register
        //	 * B7		B6		B5		B4		B3		B2		B1		B0
        //	 * SRWD		d.c		d.c		d.c		BP1		BP2		WEL		d.c.
        //	 *
        //	 * On init all bits should read 0x00.
        //	 * The only bit we use here is WEL (Write Enable) and this is reset to 0
        //	 * on power up.
        //	 * None of the other (protection) bits are used at this moment by this software.
        //	 */
        //	uint8_t *job_status = NULL;
        //	volatile uint32_t helper;

        /* Read Status register */
        mramPtr->tx[0] = 0x05;
        mramPtr->rx[0] = 0xFF;
        ADO_SSP_AddJob((uint32_t)mramPtr, mramPtr->busNr, mramPtr->tx, mramPtr->rx, 1, 1, MramJobFinished , MramActivate);
    }
}

inline void MramInitAll(void *chips) {
	_MramInitAll((mram_chipinit_array_t *)chips);
}
void _MramInitAll(mram_chipinit_array_t *chips) {
	if (chips->entryCount > MRAM_CHIP_CNT) {
		chips->entryCount = MRAM_CHIP_CNT;
	}
	for (int i = 0; i < chips->entryCount; i++ ){
		MramInit(i, chips->chipinits[i].busnr, chips->chipinits[i].csHandler);
	}
}



void MramInitSPI(uint8_t chipIdx, void(*csHandler)(bool select)) {
    if (chipIdx < MRAM_CHIP_CNT) {
        mram_chip_t *mramPtr =  &MramChipStates[chipIdx];
        mramPtr->status = MRAM_STAT_NOT_INITIALIZED;
        mramPtr->chipSelectCb = csHandler;
        mramPtr->busNr = 99;

        //  /* Init mram  read Status register
        //   * B7       B6      B5      B4      B3      B2      B1      B0
        //   * SRWD     d.c     d.c     d.c     BP1     BP2     WEL     d.c.
        //   *
        //   * On init all bits should read 0x00.
        //   * The only bit we use here is WEL (Write Enable) and this is reset to 0
        //   * on power up.
        //   * None of the other (protection) bits are used at this moment by this software.
        //   */
        //  uint8_t *job_status = NULL;
        //  volatile uint32_t helper;

        /* Read Status register */
        mramPtr->tx[0] = 0x05;
        mramPtr->rx[0] = 0xFF;
        ADO_SPI_AddJob((uint32_t)mramPtr, mramPtr->tx, mramPtr->rx, 1, 1, (void *)MramJobFinished , (void *)MramActivate);
    }
}


void MramMain() {
    for (uint8_t chipIdx = 0; chipIdx < MRAM_CHIP_CNT; chipIdx++) {
        mram_chip_t *mramPtr =  &MramChipStates[chipIdx];

        if (mramPtr->busyFlag) {
            // TODO: timouts checken
        } else {
            if (mramPtr->status == MRAM_STAT_RX_INPROGRESS) {
                // Rx job finished.
                mramPtr->status = MRAM_STAT_IDLE;
                mramPtr->ioFinishedCb(MRAM_RES_SUCCESS, mramPtr->ioAdr, mramPtr->ioDataPtr, mramPtr->ioLen);
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
                if (mramPtr->busNr == 99) {
                    ADO_SPI_AddJob((uint32_t)mramPtr, mramPtr->mramData, NULL, mramPtr->ioLen+4, 0,(void *) MramJobFinished , (void *)MramActivate);
                } else {
                    ADO_SSP_AddJob((uint32_t)mramPtr, mramPtr->busNr,mramPtr->mramData, NULL, mramPtr->ioLen+4, 0, MramJobFinished , MramActivate);
                }
                mramPtr->status = MRAM_STAT_TX_INPROGRESS;

            } else if (mramPtr->status == MRAM_STAT_TX_INPROGRESS) {
                // Write data job finished
                // Initiate Write Disabled job
                mramPtr->tx[0] = 0x04;

                mramPtr->busyFlag = true;
                if (mramPtr->busNr == 99) {
                    ADO_SPI_AddJob((uint32_t)mramPtr, mramPtr->tx, NULL, 1 , 0, (void *)MramJobFinished , (void *)MramActivate);
                } else {
                    ADO_SSP_AddJob((uint32_t)mramPtr, mramPtr->busNr, mramPtr->tx, NULL, 1 , 0, MramJobFinished , MramActivate);
                }
                mramPtr->status = MRAM_STAT_WREN_CLR;

            } else if (mramPtr->status == MRAM_STAT_WREN_CLR) {
                // Write disable job finished. This finally ends the Write-IO
                mramPtr->status = MRAM_STAT_IDLE;
                mramPtr->ioFinishedCb(MRAM_RES_SUCCESS, mramPtr->ioAdr, mramPtr->ioDataPtr, mramPtr->ioLen);
            } else if (mramPtr->status == MRAM_STAT_INITIALIZED) {
                // init ok
                mramPtr->status = MRAM_STAT_IDLE;
            }
        }
    }   // END FOR all chips.
}


void ReadMramAsync(uint8_t chipIdx, uint32_t adr,  uint8_t *rx_data,  uint32_t len, void (*finishedHandler)(mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len)) {
    mram_chip_t *mramPtr = 0;
    if (chipIdx < MRAM_CHIP_CNT) {
        mramPtr =  &MramChipStates[chipIdx];
    } else {
        finishedHandler(MRAM_RES_INVALID_CHIPIDX, adr, 0, len);
        return;
    }

    if (mramPtr->status != MRAM_STAT_IDLE) {
		finishedHandler(MRAM_RES_BUSY, adr, 0, len);
		return;
	}

	if (rx_data == NULL) {
		finishedHandler(MRAM_RES_DATA_PTR_INVALID, adr, 0, len);
		return;
	}

	if (len > MRAM_MAX_READ_SIZE) {
		finishedHandler(MRAM_RES_RX_LEN_OVERFLOW, adr, 0, len);
		return;
	}

	if (adr + len >  0x020000) {
		finishedHandler(MRAM_RES_INVALID_ADR,  adr, 0, len);
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

	if (mramPtr->busNr == 99) {
	    ADO_SPI_AddJob((uint32_t)mramPtr, mramPtr->tx, rx_data, 4 , len, (void *)MramJobFinished , (void *)MramActivate);
	} else {
	    ADO_SSP_AddJob((uint32_t)mramPtr, mramPtr->busNr, mramPtr->tx, rx_data, 4 , len, MramJobFinished , MramActivate);
	}

	return;
}

void WriteMramAsync(uint8_t chipIdx, uint32_t adr, uint8_t *data, uint32_t len,  void (*finishedHandler)(mram_res_t result, uint32_t adr, uint8_t *data, uint32_t len)) {
    mram_chip_t *mramPtr = 0;
    if (chipIdx < MRAM_CHIP_CNT) {
        mramPtr =  &MramChipStates[chipIdx];
    } else {
        finishedHandler(MRAM_RES_INVALID_CHIPIDX, adr, 0, len);
        return;
    }

    if (mramPtr->status != MRAM_STAT_IDLE) {
		finishedHandler(MRAM_RES_BUSY, adr, 0, len);
		return;
	}

	if (data == NULL) {
		finishedHandler(MRAM_RES_DATA_PTR_INVALID, adr, 0, len);
		return;
	}

	if (len > MRAM_MAX_READ_SIZE) {
		finishedHandler(MRAM_RES_RX_LEN_OVERFLOW, adr, 0, len);
		return;
	}

	if (adr + len >  0x020000) {
		finishedHandler(MRAM_RES_INVALID_ADR,  adr, 0, len);
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
	if (mramPtr->busNr == 99) {
	    ADO_SPI_AddJob((uint32_t)mramPtr, mramPtr->tx, NULL, 1 , 0, (void *)MramJobFinished , (void *)MramActivate);
	} else {
	    ADO_SSP_AddJob((uint32_t)mramPtr, mramPtr->busNr, mramPtr->tx, NULL, 1 , 0, MramJobFinished , MramActivate);
	}

	return;
}

