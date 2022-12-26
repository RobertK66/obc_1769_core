/*
 * ado_sdcard.c
 *
 *  Created on: 16.05.2020
 *      Author: Robert
 */
#include "mod/ado_sdcard.h"

#include <stdio.h>
#include <ado_crc.h>
#include <stdlib.h>

#include <ado_sspdma.h>
#include <ado_spi.h>

#include <ado_modules.h>

#define ADO_SDC_MAX_CARDS	3

#define ADO_SXX_AddJob(card, busNr, txData, rxData, txLen, rxLen, finished, activate) { \
    if (busNr == 99) { \
        ADO_SPI_AddJob(card, txData, rxData, txLen , rxLen, (void *)finished,(void *)activate); \
    } else { \
        ADO_SSP_AddJob(card, busNr, txData, rxData, txLen , rxLen, finished, activate); \
    } \
}

//SD commands, many of these are not used here
typedef enum ado_sdc_cmd_e
{
 ADO_SDC_CMD0_GO_IDLE_STATE   		  =  0,
 ADO_SDC_CMD1_SEND_OP_COND            =  1,
 ADO_SDC_CMD8_SEND_IF_COND			  =  8,
 ADO_SDC_CMD9_SEND_CSD                =  9,
 ADO_SDC_CMD12_STOP_TRANSMISSION      =  12,
 ADO_SDC_CMD13_SEND_STATUS            =  13,
 ADO_SDC_CMD16_SET_BLOCK_LEN          =  16,
 ADO_SDC_CMD17_READ_SINGLE_BLOCK      =  17,
 ADO_SDC_CMD18_READ_MULTIPLE_BLOCKS   =  18,
 ADO_SDC_CMD24_WRITE_SINGLE_BLOCK     =  24,
 ADO_SDC_CMD25_WRITE_MULTIPLE_BLOCKS  =  25,
 ADO_SDC_CMD32_ERASE_BLOCK_START_ADDR =  32,
 ADO_SDC_CMD33_ERASE_BLOCK_END_ADDR   =  33,
 ADO_SDC_CMD38_ERASE_SELECTED_BLOCKS  =  38,
 ADO_SDC_CMD41_SD_SEND_OP_COND		  =  41,   //ACMD41
 ADO_SDC_CMD55_APP_CMD				  =  55,
 ADO_SDC_CMD58_READ_OCR				  =  58,
 ADO_SDC_CMD59_CRC_ON_OFF 			  =  59
} ado_sdc_cmd_t;

typedef enum ado_sdc_status_e
{
	ADO_SDC_CARDSTATUS_UNDEFINED = 0,
	ADO_SDC_CARDSTATUS_INIT_RESET,
	ADO_SDC_CARDSTATUS_INIT_RESET2,
	ADO_SDC_CARDSTATUS_INIT_CMD8,
	ADO_SDC_CARDSTATUS_INIT_ACMD41_1,
	ADO_SDC_CARDSTATUS_INIT_ACMD41_2,
	ADO_SDC_CARDSTATUS_INIT_ACMD41_3,
	ADO_SDC_CARDSTATUS_INIT_READ_OCR,
	ADO_SDC_CARDSTATUS_INITIALIZED,
	ADO_SDC_CARDSTATUS_READ_SBCMD,
	ADO_SDC_CARDSTATUS_READ_SBWAITDATA,
	ADO_SDC_CARDSTATUS_READ_SBWAITDATA2,
	ADO_SDC_CARDSTATUS_READ_SBDATA,
	ADO_SDC_CARDSTATUS_WRITE_SBCMD,
	ADO_SDC_CARDSTATUS_WRITE_SBDATA,
	ADO_SDC_CARDSTATUS_WRITE_BUSYWAIT,
	ADO_SDC_CARDSTATUS_WRITE_CHECKSTATUS,
	ADO_SDC_CARDSTATUS_ERROR = 99
} ado_sdc_status_t;

typedef enum ado_sdc_cardtype_e
{
	ADO_SDC_CARD_UNKNOWN = 0,
	ADO_SDC_CARD_1XSD,				// V1.xx Card (not accepting CMD8 -> standard capacity
	ADO_SDC_CARD_20SD,				// V2.00 Card standard capacity
	ADO_SDC_CARD_20HCXC,			// V2.00 Card High Capacity HC or ExtendenCapacity XC
} ado_sdc_cardtype_t;

// SSP command level data
typedef struct ado_sdcars_s {
    ado_sdc_cardtype_t sdcType;
    uint8_t          sdcCmdData[10];
    uint8_t          sdcCmdResponse[24];
    uint8_t          *dataBuffer;           // Pointer to (Clients) Block ReadWrite Memory. Must Hold Block(512) + command byte + 2Byte Checksum
    uint32_t         curBlockNr;
    ado_sspid_t      sdcBusNr;              //
    ado_sdc_status_t sdcStatus;             //
    ado_sdc_status_t nextStatus;            // used by callback to switch state if SPI command finished its sending.
    bool             sdcCmdPending;         //
    uint16_t         sdcWaitLoops;          //
    const PINMUX_GRP_T2* csPin;			    //void             (*csHandler)(bool select);
    void             (*blockFinishedHandler)(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len);
} ado_sdcard_t;



//// module events
typedef struct {
	uint16_t			cardIdx:4;
	uint16_t			cardType:4;
	uint16_t			status:8;
	//uint16_t				status;				// in case of error this is prev status, else the current state is signaled.
} ado_sdcard_event_t;
//
//void CreateSdCardEventInitialized(ado_event_t *event, va_list* args) {
//    event->eventDataSize = sizeof(ado_sdcardevent_initialized_t) - sizeof(ado_event_t);
//    ado_sdcardevent_initialized_t *e = (ado_sdcardevent_initialized_t *)event;
//    e->sspBus = va_arg(*args, int);
//    e->cardType = va_arg(*args, int);
//}
//


// prototypes
void *SdcInit(ado_sbus_id_t sspId, const PINMUX_GRP_T2* csPin);
void *SdcInitSPI(const PINMUX_GRP_T2* csPin);
void DMAFinishedIRQ(uint32_t context, ado_sspstatus_t jobStatus, uint8_t *rxData, uint16_t rxSize);
void SdcSendCommand(ado_sdcard_t *sdCard, ado_sdc_cmd_t cmd, uint32_t nextState, uint32_t arg);
//void SdcReadBlock(ado_sspid_t bus, uint32_t blockNr);


// Currently we support one sdcard per SSP bus. but this could be expanded if CS-Callbacks are used for different cards on same bus....
//static ado_sdcard_t SdCard[3];
static uint8_t	 	cardCnt = 0;
static ado_sdcard_t *sdCards[ADO_SDC_MAX_CARDS];


void ActivateCS(uint32_t context) {
    ado_sdcard_t *sdCard = (ado_sdcard_t *)context;
    Chip_GPIO_SetPinState(LPC_GPIO, sdCard->csPin->pingrp, sdCard->csPin->pinnum, !sdCard->csPin->initval);

//    if (sdCard->csHandler != 0) {       // If no CS handler is provided we rely on 'native SSL' to be configured. Then the HW takes care of CS activation o nits own.
//        sdCard->csHandler(true);
//    }
}

inline void SdcInitAll(void* cards) { _SdcInitAll((sdcard_init_array_t*)cards); }

void _SdcInitAll(sdcard_init_array_t* cards) {
	if (cards->entryCount > ADO_SDC_MAX_CARDS) {
		cards->entryCount = ADO_SDC_MAX_CARDS;
		// TODO -> trigger init Warning!?
	}
	for (int i = 0; i < cards->entryCount; i++ ){
		if (cards->chipinits[i].busnr == ADO_SBUS_SPI) {
			SdcInitSPI(cards->chipinits[i].csPin);
		} else {
			SdcInit(cards->chipinits[i].busnr, cards->chipinits[i].csPin);
		}
	}
}

void *SdcInit(ado_sbus_id_t bus, const PINMUX_GRP_T2* csPin) {
	ado_sdcard_t *newCard = 0;
	newCard = malloc(sizeof(ado_sdcard_t));
	if (newCard != 0) {
		sdCards[cardCnt] = newCard;
		sdCards[cardCnt]->sdcBusNr = bus;
		sdCards[cardCnt]->csPin = csPin;
		sdCards[cardCnt]->sdcType = ADO_SDC_CARD_UNKNOWN;
		sdCards[cardCnt]->sdcStatus = ADO_SDC_CARDSTATUS_UNDEFINED;
		sdCards[cardCnt]->sdcCmdPending = false;
		sdCards[cardCnt]->sdcWaitLoops = 0;
		cardCnt++;
	}
    return (void *)newCard;
}

void *SdcInitSPI(const PINMUX_GRP_T2* csPin){ //void(*csHandler)(bool select)) {
	ado_sdcard_t *newCard = 0;
	newCard = malloc(sizeof(ado_sdcard_t));
	if (newCard != 0) {
		sdCards[cardCnt] = newCard;
		sdCards[cardCnt]->sdcBusNr = 99;
		sdCards[cardCnt]->csPin = csPin;
		sdCards[cardCnt]->sdcType = ADO_SDC_CARD_UNKNOWN;
		sdCards[cardCnt]->sdcStatus = ADO_SDC_CARDSTATUS_UNDEFINED;
		sdCards[cardCnt]->sdcCmdPending = false;
		sdCards[cardCnt]->sdcWaitLoops = 0;
		cardCnt++;
	}
    return (void *)newCard;
}

void SdcMain(void) {
	for (int i = 0; i<cardCnt; i++) {
		ado_sdcard_t *sdCard = sdCards[i];

		if (!sdCard->sdcCmdPending) {
			// prepare an event - in case needed
			// prefill with data used by all usages
			ado_sdcard_event_t eventdata;
			eventdata.cardIdx = i & 0x03;
			eventdata.cardType = sdCard->sdcType;


			// This is the sdc state engine reacting to a finished SPI job
			switch (sdCard->sdcStatus) {
			case ADO_SDC_CARDSTATUS_INIT_RESET:
				if (sdCard->sdcCmdResponse[1] == 0x01) {
					// Send CMD8
					SdcSendCommand(sdCard, ADO_SDC_CMD8_SEND_IF_COND, ADO_SDC_CARDSTATUS_INIT_CMD8, 0x000001AA);
				} else {
					// Try a 2nd time
					// All my SDHD cards switch to SPI mode with this repeat - regardless of power up sequence!
					SdcSendCommand(sdCard, ADO_SDC_CMD0_GO_IDLE_STATE, ADO_SDC_CARDSTATUS_INIT_RESET2, 0);
				}
				break;

			case ADO_SDC_CARDSTATUS_INIT_RESET2:
					if (sdCard->sdcCmdResponse[1] == 0x01) {
						// Send CMD8
						SdcSendCommand(sdCard, ADO_SDC_CMD8_SEND_IF_COND, ADO_SDC_CARDSTATUS_INIT_CMD8, 0x000001AA);
					} else {
						// Set Status to Error and send SysEvent with prev Status.
						eventdata.status = (uint8_t)sdCard->sdcStatus;
						sdCard->sdcStatus = ADO_SDC_CARDSTATUS_ERROR;
						SysEvent(MODULE_ID_SDCARD, EVENT_ERROR, EID_SDCARD_STATUS, &eventdata, sizeof(eventdata));
					}
					break;

			case ADO_SDC_CARDSTATUS_INIT_CMD8:
				if (sdCard->sdcCmdResponse[1] == 0x01) {
					sdCard->sdcType = ADO_SDC_CARD_20SD;
					// Send CMD55 (-> ACMD41)
					SdcSendCommand(sdCard, ADO_SDC_CMD55_APP_CMD, ADO_SDC_CARDSTATUS_INIT_ACMD41_1, 0);
				} else {
					sdCard->sdcType = ADO_SDC_CARD_1XSD;
					// Set Status to Error and send SysEvent with previous stat.
					eventdata.cardType = sdCard->sdcType;
					eventdata.status = (uint8_t)sdCard->sdcStatus;
					sdCard->sdcStatus = ADO_SDC_CARDSTATUS_ERROR;
					SysEvent(MODULE_ID_SDCARD, EVENT_ERROR, EID_SDCARD_UNIMPLEMENTED_TYPE, &eventdata, sizeof(eventdata));
					//printf("Sd Card version not implemented yet.\n");
					//sdCard->sdcStatus = ADO_SDC_CARDSTATUS_ERROR;
				}
				break;

			case ADO_SDC_CARDSTATUS_INIT_ACMD41_1:
				if (sdCard->sdcCmdResponse[1] == 0x01) {
					// Send (CMD55 - ) ACMD41
					if (sdCard->sdcType == ADO_SDC_CARD_1XSD) {
						SdcSendCommand(sdCard, ADO_SDC_CMD41_SD_SEND_OP_COND, ADO_SDC_CARDSTATUS_INIT_ACMD41_2, 0);
					} else {
						SdcSendCommand(sdCard, ADO_SDC_CMD41_SD_SEND_OP_COND, ADO_SDC_CARDSTATUS_INIT_ACMD41_2, 0x40000000);
					}
				} else {
					// Set Status to Error and send SysEvent
					eventdata.status = (uint8_t)sdCard->sdcStatus;
					sdCard->sdcStatus = ADO_SDC_CARDSTATUS_ERROR;
					SysEvent(MODULE_ID_SDCARD, EVENT_ERROR, EID_SDCARD_STATUS, &eventdata, sizeof(eventdata));
				}
				break;

			case ADO_SDC_CARDSTATUS_INIT_ACMD41_2:
				if (sdCard->sdcCmdResponse[1] == 0x01) {
					// Init sequence is not finished yet(idle mode).
					// Lets wait some main loops and then retry the ACMD41
					sdCard->sdcWaitLoops = 100;
					sdCard->sdcStatus = ADO_SDC_CARDSTATUS_INIT_ACMD41_3;
					//printf("."); // debug indicator only....
				} else if (sdCard->sdcCmdResponse[1] == 0x00) {
					//printf("\n");  // debug indicator only....
					// Send CMD58 to get R3 - OCR Response
					SdcSendCommand(sdCard, ADO_SDC_CMD58_READ_OCR, ADO_SDC_CARDSTATUS_INIT_READ_OCR, 0);
				} else {
					// Set Status to Error and send SysEvent with previous state
					eventdata.status = (uint8_t)sdCard->sdcStatus;
					sdCard->sdcStatus = ADO_SDC_CARDSTATUS_ERROR;
					SysEvent(MODULE_ID_SDCARD, EVENT_ERROR, EID_SDCARD_STATUS, &eventdata, sizeof(eventdata));
				}
				break;

			case ADO_SDC_CARDSTATUS_INIT_ACMD41_3:
				if (sdCard->sdcWaitLoops > 0) {
					sdCard->sdcWaitLoops--;
					if (sdCard->sdcWaitLoops == 0) {
						// Retry the ACMD41
						// Send CMD55 (-> ACMD41)
						SdcSendCommand(sdCard, ADO_SDC_CMD55_APP_CMD, ADO_SDC_CARDSTATUS_INIT_ACMD41_1, 0);
					}
				}
				break;

			case ADO_SDC_CARDSTATUS_INIT_READ_OCR:
				if (sdCard->sdcCmdResponse[1] == 0x00) {
					if ((sdCard->sdcType == ADO_SDC_CARD_20SD) &&
						((sdCard->sdcCmdResponse[2] & 0x40) != 0)) {
						sdCard->sdcType = ADO_SDC_CARD_20HCXC;
					}
					// Signal successful initialized status as event
					sdCard->sdcStatus = ADO_SDC_CARDSTATUS_INITIALIZED;
					eventdata.status = (uint8_t)sdCard->sdcStatus;
					SysEvent(MODULE_ID_SDCARD, EVENT_INFO, EID_SDCARD_STATUS, &eventdata, sizeof(eventdata));
					//printf("Card (type %d) initialized.\n", sdCard->sdcType);  // Debug indicator only!? or success event !? and/or Callback for Init!?

					// Signal our custom event to event logger
					// DefinedLogEvent(ado_sdcardevent_initialized_t, CreateSdCardEventInitialized, sdCard->sdcBusNr, sdCard->sdcType);

				} else {
					// Set and Signal Error
					eventdata.status = (uint8_t)sdCard->sdcStatus;
					sdCard->sdcStatus = ADO_SDC_CARDSTATUS_ERROR;
					SysEvent(MODULE_ID_SDCARD, EVENT_ERROR, EID_SDCARD_STATUS, &eventdata, sizeof(eventdata));
					// TODO: replace by error event
					//printf("Errors %02X reading OCR.\n", sdCard->sdcCmdResponse[1]);
					//sdCard->sdcStatus = ADO_SDC_CARDSTATUS_ERROR;
				}
				break;

			case ADO_SDC_CARDSTATUS_READ_SBCMD:
				// TODO: how many bytes can a response be delayed (Spec says 0..8 / 1..8 depending of card Type !?
				// all my cards always responded in byte 1 or 2.
				if ((sdCard->sdcCmdResponse[1] == 0x00) || (sdCard->sdcCmdResponse[2] == 0x00))  {
					// Lets wait for the start data token.
					sdCard->sdcCmdPending = true;
					sdCard->nextStatus = ADO_SDC_CARDSTATUS_READ_SBWAITDATA;
					ADO_SXX_AddJob((uint32_t)(sdCard), sdCard->sdcBusNr, sdCard->sdcCmdData, sdCard->sdcCmdResponse, 0 , 1, DMAFinishedIRQ, ActivateCS);
				} else {
					eventdata.status = (uint8_t)sdCard->sdcStatus;
					sdCard->sdcStatus = ADO_SDC_CARDSTATUS_ERROR;
					SysEvent(MODULE_ID_SDCARD, EVENT_ERROR, EID_SDCARD_STATUS, &eventdata, sizeof(eventdata));
					// TODO: Signal Error event
					//printf("Error %02X %02X with read block command\n",sdCard->sdcCmdResponse[1], sdCard->sdcCmdResponse[2] );
					//sdCard->sdcStatus = ADO_SDC_CARDSTATUS_ERROR;
				}
				break;

			case ADO_SDC_CARDSTATUS_READ_SBWAITDATA:
				if (sdCard->sdcCmdResponse[0] == 0xFE) {
					// Now we can read all data bytes including 2byte CRC + 1 byte over-read as always to get the shift register in the card emptied....
					sdCard->sdcCmdPending = true;
					sdCard->nextStatus = ADO_SDC_CARDSTATUS_READ_SBDATA;
					ADO_SXX_AddJob((uint32_t)(sdCard),  sdCard->sdcBusNr, sdCard->sdcCmdData,  sdCard->dataBuffer, 0 , 515, DMAFinishedIRQ, ActivateCS);
				} else {
					// Wait some mainloops before asking for data token again.
					sdCard->sdcStatus = ADO_SDC_CARDSTATUS_READ_SBWAITDATA2;
					sdCard->sdcWaitLoops = 10;
				}
				break;

			case ADO_SDC_CARDSTATUS_READ_SBWAITDATA2:
				if (sdCard->sdcWaitLoops > 0) {
					sdCard->sdcWaitLoops--;
					if (sdCard->sdcWaitLoops == 0) {
						// Read 1 byte to check for data token
						// TODO: this can loop for ever between this wait states here (e.g. if SD Card powerd off after init) -> check repeat count and goto error !!!
						sdCard->sdcCmdPending = true;
						sdCard->nextStatus = ADO_SDC_CARDSTATUS_READ_SBWAITDATA;
						ADO_SXX_AddJob((uint32_t)(sdCard), sdCard->sdcBusNr, sdCard->sdcCmdData, sdCard->sdcCmdResponse, 0 , 1,DMAFinishedIRQ,ActivateCS);
					}
				}
				break;

			case ADO_SDC_CARDSTATUS_READ_SBDATA:
			{
				sdCard->sdcStatus = ADO_SDC_CARDSTATUS_INITIALIZED;
				uint16_t crc = CRC16_XMODEM(sdCard->dataBuffer, 514);
				if (0 == crc) {
					sdCard->blockFinishedHandler(SDC_RES_SUCCESS, sdCard->curBlockNr, sdCard->dataBuffer, 512);
				} else {
					sdCard->blockFinishedHandler(SDC_RES_CRCERROR, sdCard->curBlockNr, sdCard->dataBuffer, 512);
				}
				break;
			}

			case ADO_SDC_CARDSTATUS_WRITE_SBCMD:
				if (sdCard->sdcCmdResponse[1] == 0x00) {
					// Now we can write all data bytes including 1 Start token and 2byte CRC. We expect 1 byte data response token....
					sdCard->sdcCmdPending = true;
					sdCard->nextStatus = ADO_SDC_CARDSTATUS_WRITE_SBDATA;
					//printf("\nw");      // Debug print only
					// TODO: sdcReadWriteData is provided by CLI Client -> move this out later....
					//ADO_SSP_AddJob((uint32_t)(sdCard),sdCard->sdcBusNr, sdCard->dataBuffer, sdCard->sdcCmdResponse, 515 , 3, DMAFinishedIRQ, ActivateCS);
					ADO_SXX_AddJob((uint32_t)(sdCard),sdCard->sdcBusNr, sdCard->dataBuffer, sdCard->sdcCmdResponse, 515 , 3, DMAFinishedIRQ, ActivateCS);
				} else {
					eventdata.status = (uint8_t)sdCard->sdcStatus;
					sdCard->sdcStatus = ADO_SDC_CARDSTATUS_ERROR;
					SysEvent(MODULE_ID_SDCARD, EVENT_ERROR, EID_SDCARD_STATUS, &eventdata, sizeof(eventdata));
					// TODO: signal error event
					//printf("Error %02X %02X with read block command\n",sdCard->sdcCmdResponse[1],sdCard->sdcCmdResponse[2] );
				}
				break;

			case ADO_SDC_CARDSTATUS_WRITE_SBDATA:
				if ((sdCard->sdcCmdResponse[0] & 0x1F) == 0x05) {
					// Data was accepted. now we wait until busy token is off again....
					// Read 1 byte to check for busy token
					sdCard->sdcCmdPending = true;
					sdCard->nextStatus = ADO_SDC_CARDSTATUS_WRITE_BUSYWAIT;
					ADO_SXX_AddJob((uint32_t)(sdCard),  sdCard->sdcBusNr,  sdCard->sdcCmdData,  sdCard->sdcCmdResponse, 0 , 1, DMAFinishedIRQ, ActivateCS);
				} else {
					eventdata.status = (uint8_t)sdCard->sdcStatus;
					sdCard->sdcStatus = ADO_SDC_CARDSTATUS_ERROR;
					SysEvent(MODULE_ID_SDCARD, EVENT_ERROR, EID_SDCARD_STATUS, &eventdata, sizeof(eventdata));

					// TODO: signal error event
					//printf("Error %02X %02X with write data block\n",  sdCard->sdcCmdResponse[0],  sdCard->sdcCmdResponse[1] );
					//sdCard->sdcStatus = ADO_SDC_CARDSTATUS_ERROR;
				}
				break;

			case ADO_SDC_CARDSTATUS_WRITE_BUSYWAIT:
				if ( sdCard->sdcCmdResponse[0] == 0x00) {
					// still busy !?
					// Read 1 byte to check for busy token
					// TODO: some mainloop wait here ....
					sdCard->sdcCmdPending = true;
					sdCard->nextStatus = ADO_SDC_CARDSTATUS_WRITE_BUSYWAIT;
					ADO_SXX_AddJob((uint32_t)(sdCard), sdCard->sdcBusNr, sdCard->sdcCmdData, sdCard->sdcCmdResponse, 0 , 1, DMAFinishedIRQ,ActivateCS);
				} else {
					// busy is off now. Lets Check Status
					// Send CMD13
					SdcSendCommand(sdCard, ADO_SDC_CMD13_SEND_STATUS, ADO_SDC_CARDSTATUS_WRITE_CHECKSTATUS, 0);
				}
				break;

			case ADO_SDC_CARDSTATUS_WRITE_CHECKSTATUS:
				if (sdCard->sdcCmdResponse[1] == 0x00) {
					sdCard->sdcStatus = ADO_SDC_CARDSTATUS_INITIALIZED;
					//printf("!\n");  // debug only output os signal success event
					sdCard->blockFinishedHandler(SDC_RES_SUCCESS, sdCard->curBlockNr, sdCard->dataBuffer, 512);
				} else {
					eventdata.status = (uint8_t)sdCard->sdcStatus;
					sdCard->sdcStatus = ADO_SDC_CARDSTATUS_ERROR;
					SysEvent(MODULE_ID_SDCARD, EVENT_ERROR, EID_SDCARD_STATUS, &eventdata, sizeof(eventdata));
					//TODO: signal error event
					//printf("Error %02X %02X answer to CMD13\n", sdCard->sdcCmdResponse[1], sdCard->sdcCmdResponse[2] );
					//sdCard->sdcStatus = ADO_SDC_CARDSTATUS_ERROR;
				}
				break;

			// Nothing needed here
			case ADO_SDC_CARDSTATUS_INITIALIZED:
				break;

			default:
				break;
			}


		}
	} // end for (cards)
}


void DMAFinishedIRQ(uint32_t context, ado_sspstatus_t jobStatus, uint8_t *rxData, uint16_t rxSize) {
    ado_sdcard_t *sdCard = (ado_sdcard_t *)context;

    // Deselect the cards CS line (by restoring it to initval)
    Chip_GPIO_SetPinState(LPC_GPIO, sdCard->csPin->pingrp, sdCard->csPin->pinnum, sdCard->csPin->initval);

//     if (sdCard->csHandler != 0) {
//        sdCard->csHandler(false);                       // Deselect the cards CS line
//    }
	if (jobStatus == ADO_SSP_JOBDONE) {
		sdCard->sdcStatus = sdCard->nextStatus;         // Signal Status switch to mainloop.
	} else {
	    // TODO: Check, if mainloop signals this error as event in this case !?
	    //       all job errors the same here ?
		sdCard->sdcStatus = ADO_SDC_CARDSTATUS_ERROR;
	}
	sdCard->sdcCmdPending = false;
}


void SdcSendCommand(ado_sdcard_t *sdCard, ado_sdc_cmd_t cmd, uint32_t nextState, uint32_t arg) {
	int responseSize = 3;

	sdCard->sdcCmdData[0] = 0x40 | cmd;
	sdCard->sdcCmdData[1] = (uint8_t)(arg>>24);
	sdCard->sdcCmdData[2] = (uint8_t)(arg>>16);
	sdCard->sdcCmdData[3] = (uint8_t)(arg>>8);
	sdCard->sdcCmdData[4] = (uint8_t)(arg);
	sdCard->sdcCmdData[5] = 0xFF;

	// This 2 cmds need correct crc
	if (cmd == ADO_SDC_CMD0_GO_IDLE_STATE) {
	    sdCard->sdcCmdData[5] = 0x95;
	} else if (cmd == ADO_SDC_CMD8_SEND_IF_COND) {
	    sdCard->sdcCmdData[5] = 0x87;
		responseSize = 7;
	} else if (cmd ==  ADO_SDC_CMD58_READ_OCR) {
		responseSize = 7;
	}  else if (cmd ==  ADO_SDC_CMD13_SEND_STATUS) {
		responseSize = 4;
	}

	sdCard->sdcCmdPending = true;
	sdCard->nextStatus = nextState;

	//ADO_SSP_AddJob((uint32_t)(sdCard), sdCard->sdcBusNr, sdCard->sdcCmdData, sdCard->sdcCmdResponse, 6 , responseSize, DMAFinishedIRQ, ActivateCS);
	ADO_SXX_AddJob((uint32_t)(sdCard), sdCard->sdcBusNr, sdCard->sdcCmdData, sdCard->sdcCmdResponse, 6 , responseSize, DMAFinishedIRQ, ActivateCS);

}


//void SdcPowerupCmd(int argc, char *argv[]) {
//	LPC_SSP_T *sspBase = 0;
//	if (sdcBusNr == ADO_SSP0) {
//		sspBase = LPC_SSP0;
//	} else if  (sdcBusNr == ADO_SSP0) {
//		sspBase = LPC_SSP1;
//	}
////  This does not work, because the direct CS is controlled by SSP and can not be easily hold at high without changing the SSP config.....
////  As all our tested cards do switch to SPI Mode when the INIT CMD0 is repeated once, we do not care for Power on yet.....
////	if (sspBase != 0) {
////		uint8_t dummy; (void)dummy;
////		// Make 80 Clock without CS
////		for (int i=0;i<10;i++) {
////			sspBase->DR = 0x55;
////			dummy = sspBase->SR;
////			dummy = sspBase->DR; // Clear Rx buffer.
////		}
////		{ uint32_t temp; (void)temp; while ((sspBase->SR & SSP_STAT_RNE) != 0) { temp = sspBase->DR; } }
////	}
//}

void SdcCardinitialize(uint8_t cardIdx){
	if (cardIdx <= cardCnt) {
		// Reset module state and initiate the card init sequence
		ado_sdcard_t *sdCard = sdCards[cardIdx];
		sdCard->sdcType = ADO_SDC_CARD_UNKNOWN;
		sdCard->sdcStatus = ADO_SDC_CARDSTATUS_UNDEFINED;
		SdcSendCommand(sdCard, ADO_SDC_CMD0_GO_IDLE_STATE, ADO_SDC_CARDSTATUS_INIT_RESET, 0);
	}
}

void SdcReadBlockAsync(uint8_t cardIdx, uint32_t blockNr, uint8_t *data,  void (*finishedHandler)(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len)) {
	if (cardIdx <= cardCnt) {
		ado_sdcard_t *sdCard = sdCards[cardIdx];
		uint32_t arg = 0;
		sdCard->curBlockNr = blockNr;
		if (sdCard->sdcType == ADO_SDC_CARD_20SD) {
			// SD Cards take byte addresses as argument
			arg = blockNr << 9;
		} else {
		  // 2.0 HC or XC take block addressing with fixed 512 byte blocks as argument
		  arg = blockNr;
		}

		sdCard->blockFinishedHandler = finishedHandler;
		sdCard->dataBuffer = data;              // This has to have room for 512+2 bytes

		if (sdCard->sdcStatus == ADO_SDC_CARDSTATUS_INITIALIZED) {
		   SdcSendCommand(sdCard, ADO_SDC_CMD17_READ_SINGLE_BLOCK, ADO_SDC_CARDSTATUS_READ_SBCMD, arg);
		} else {
		   finishedHandler(SDC_RES_ERROR, blockNr, 0, 0);
		}
	}
}

//void SdcWriteBlockAsync(uint8_t cardIdx, uint32_t blockNr, uint8_t *data, void (*finishedHandler)(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len)) {
void SdcWriteBlockAsync(uint8_t cardIdx, uint32_t blockNr, sdcard_block512 *data, void (*finishedHandler)(sdc_res_t result, uint32_t blockNr, uint8_t *data, uint32_t len)) {
	if (cardIdx <= cardCnt) {
		ado_sdcard_t *sdCard = sdCards[cardIdx];

		uint32_t arg = 0;
		sdCard->curBlockNr = blockNr;
		if (sdCard->sdcType == ADO_SDC_CARD_20SD) {
			// SD Cards take byte addresses as argument
			arg = blockNr << 9;
		} else {
			// 2.0 HC or XC take block addressing with fixed 512 byte blocks as argument
			arg = blockNr;
		}

		sdCard->blockFinishedHandler = finishedHandler;
		data->token = 0xFE;             // Data Block Start token.
		uint16_t crc = CRC16_XMODEM((uint8_t*)data, 512);
		data->crc[0] = (uint8_t)(crc >> 8);
		data->crc[1] = (uint8_t) crc;
		sdCard->dataBuffer = (uint8_t*)data;

		if (sdCard->sdcStatus == ADO_SDC_CARDSTATUS_INITIALIZED) {
			SdcSendCommand(sdCard, ADO_SDC_CMD24_WRITE_SINGLE_BLOCK, ADO_SDC_CARDSTATUS_WRITE_SBCMD, arg);
		} else {
			finishedHandler(SDC_RES_ERROR, blockNr, 0, 0);
		}
	}
}

bool SdcIsCardinitialized(uint8_t cardIdx) {
	bool retVal = false;
	if (cardIdx <= cardCnt) {
		retVal = (sdCards[cardIdx]->sdcStatus == ADO_SDC_CARDSTATUS_INITIALIZED);
	}
	return retVal;
}
