/*
 *  obc_ssp0.c
 *
 *  Created on: 07.10.2012 ...
 *      Author: Andi, Robert
 *
 *  Copied over from pegasus flight software on 2019-12-14
 */
#include "ado_sspdma.h"

#include <stdio.h>
#include <string.h>
#include <chip.h>

//#define ADO_SSPDMA_BITFLIPSAFE  // Use this define to reconfigure constant part of DMA-Ctrl structures with every DMA Request.
								// This costs ca. 3,6% performance with massive multi-job read/writes.

// Inline Block to clear SSP Rx Buffer.
//#define ADO_SSP_DUMP_RX(device) { uint32_t temp; (void)temp; while ((device->SR & SSP_STAT_RNE) != 0) { temp = device->DR; } }
#define ADO_SSP_DUMP_RX(device) { while ((device->SR & SSP_STAT_RNE) != 0) { rxDummy = device->DR; } }

#define ADO_INCREMENT_SSPJOBNR(anyUint) { anyUint++; if (anyUint == ADO_SSP_MAXJOBS) {anyUint = 0; } }


// Following constants hold the Configuration data for both SSPx interfaces. idx=0 -> SSP0 idx=1 ->SSP1
const LPC_SSP_T *ADO_SSP_RegBase[] = {
	LPC_SSP0,
	LPC_SSP1
};

const CHIP_SYSCTL_CLOCK_T ADO_SSP_SysCtlClock[] = {
	SYSCTL_CLOCK_SSP0,
	SYSCTL_CLOCK_SSP1
};

const uint8_t ADO_SSP_RxDmaChannel[] = {
	ADO_SSP0_RXDMACHANNEL,
	ADO_SSP1_RXDMACHANNEL
};

const uint8_t ADO_SSP_TxDmaChannel[] = {
	ADO_SSP0_TXDMACHANNEL,
	ADO_SSP1_TXDMACHANNEL
};

const uint32_t ADO_SSP_GPDMA_CONN_RX[] = {
	GPDMA_CONN_SSP0_Rx,
	GPDMA_CONN_SSP1_Rx
};

const uint32_t ADO_SSP_GPDMA_CONN_TX[] = {
	GPDMA_CONN_SSP0_Tx,
	GPDMA_CONN_SSP1_Tx
};

typedef struct ado_sspjob_s
{
	uint8_t *txData;
	uint8_t *rxData;
	uint16_t txSize;
	uint16_t rxSize;
	uint32_t context;											// Any data can be stored here by client. Its fed back to the callbacks when job processes.
	AdoSSP_ActivateHandler(ADO_SSP_JobActivated_IRQCallback);	// Use this for activating chip Select, if SSP SSL not used.
	AdoSSP_FinishedHandler(ADO_SSP_JobFinished_IRQCallback);	// Do not process received data with this callback. Only signal data available to your main routines.
} ado_sspjob_t;

typedef struct ado_sspjobs_s
{
	ado_sspjob_t job[ADO_SSP_MAXJOBS];
	uint8_t current_job;
	uint8_t jobs_pending;
	uint8_t txDmaChannel;
	uint8_t rxDmaChannel;
	DMA_TransferDescriptor_t dmaTd[4];
	uint16_t ssp_job_error_counter;
} ado_sspjobs_t;

// local/module variables
ado_sspjobs_t 	ado_sspjobs[2];		// One structure per SSP Master [0]:SSP0, [1]:SSP1
uint8_t 		rxDummy;            // used for both ssp as 'empty'/null destination
uint8_t 		txDummy = 0xFF;     // used for both ssp as 'empty'/null source. has to be 0xFF!

// Prototypes
void ADO_SSP_InitiateDMA(ado_sspid_t sspId, ado_sspjob_t *newJob);

// Init for one SSP bus.
void ADO_SSP_Init(ado_sspid_t sspId, uint32_t bitRate, CHIP_SSP_CLOCK_MODE_T clockMode) {
	LPC_SSP_T *pSSP = (LPC_SSP_T *)ADO_SSP_RegBase[sspId];
	
	Chip_Clock_EnablePeriphClock(ADO_SSP_SysCtlClock[sspId]);		
	Chip_Clock_SetPCLKDiv(ADO_SSP_SysCtlClock[sspId], SYSCTL_CLKDIV_1);		// No additional prescaler used.
	Chip_SSP_Set_Mode(pSSP, SSP_MODE_MASTER);								// SSP is Master
	Chip_SSP_SetFormat(pSSP, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, clockMode);	// SPI Mode with 8 bit  SSP_CLOCK_CPHA0_CPOL0 'mode0' this makes CS pulses per Byte (on native SSP SSL Port).
																			// 		                SSP_CLOCK_CPHA1_CPOL1 'mode3' keeps CS/SSL active for the whole job transfer. (this is 10% faster!)
	Chip_SSP_SetBitRate(pSSP, bitRate);				// This calculates settings with a near fit. Actual bitrate is calculated as ....
	
//	Chip_SSP_SetClockRate(pSSP, 5, 2);
//	Chip_SSP_SetClockRate(pSSP, 4, 2);				// SSPClk: 9.6Mhz   	( With 96Mhz SystemCoreClock -> SSP Clk = 48Mhz / (4+1) )
//	Chip_SSP_SetClockRate(pSSP, 3, 2);				// SSPClk: 12Mhz		( With 96Mhz SystemCoreClock -> SSP Clk = 48Mhz / (3+1) )
//	Chip_SSP_SetClockRate(pSSP, 2, 2);				// SSPClk: 16Mhz		( With 96Mhz SystemCoreClock -> SSP Clk = 48Mhz / (2+1) )
//	Chip_SSP_SetClockRate(pSSP, 1, 2);				// SSPClk: 24Mhz		( With 96Mhz SystemCoreClock -> SSP Clk = 48Mhz / (1+1) )
//	Chip_SSP_SetClockRate(pSSP, 0, 2);				// SSPClk: 48Mhz			- not working with my external wired socket !? ...

	Chip_SSP_DisableLoopBack(pSSP);
	Chip_SSP_Enable(pSSP);
	Chip_SSP_Int_Disable(pSSP, SSP_INT_BITMASK);
	Chip_SSP_DMA_Enable(pSSP);

	ADO_SSP_DUMP_RX(pSSP);

	/* Reset buffers to default values */
	ado_sspjobs[sspId].current_job = 0;
	ado_sspjobs[sspId].jobs_pending = 0;
	ado_sspjobs[sspId].ssp_job_error_counter = 0;
	ado_sspjobs[sspId].rxDmaChannel = ADO_SSP_RxDmaChannel[sspId];
	ado_sspjobs[sspId].txDmaChannel = ADO_SSP_TxDmaChannel[sspId];

	// Initialize the DMA Ctrl structures with empty src/dest addresses and len.
	// Receiver Channel - Setup as Scatter Transfer with 2 blocks.
	// First bytes are TX only. We write all Rx to a dummy Byte.
	ado_sspjobs[sspId].dmaTd[0].src = ADO_SSP_GPDMA_CONN_RX[sspId];     // The GPDMA_CONN_SSPn_Rx where n is 0/1 indicating used io source
	ado_sspjobs[sspId].dmaTd[0].dst = (uint32_t)&rxDummy;
	ado_sspjobs[sspId].dmaTd[0].lli = (uint32_t)(&ado_sspjobs[sspId].dmaTd[1]);
	ado_sspjobs[sspId].dmaTd[0].ctrl = 0x00009000;

	// Then the real receive starts to write into rxData
	ado_sspjobs[sspId].dmaTd[1].src = (uint32_t)&(pSSP->DR);    // Here the io-address of the used SSPn is needed directly
	ado_sspjobs[sspId].dmaTd[1].dst = 0;					    // This (destination address in RAM) will be filled by job entry later.
	ado_sspjobs[sspId].dmaTd[1].lli = 0;
	ado_sspjobs[sspId].dmaTd[1].ctrl = 0x88009000;

	// Transmit Channel - 2 Blocks
	// first n byte are the real TX
	ado_sspjobs[sspId].dmaTd[2].src = 0;					            // This (source address from RAM) will be filled by job entry later.
	ado_sspjobs[sspId].dmaTd[2].dst = ADO_SSP_GPDMA_CONN_TX[sspId];     // Here the GPDMA_CONN_SSPn_Tx for n=0/1 is needed to give the DMA io target.
	ado_sspjobs[sspId].dmaTd[2].lli = (uint32_t)(&ado_sspjobs[sspId].dmaTd[3]);
	ado_sspjobs[sspId].dmaTd[2].ctrl = 0x04009000;

	// Then the tx channel only sends dummy 0xFF bytes in order to receive all rx bytes
	ado_sspjobs[sspId].dmaTd[3].src = (uint32_t)&txDummy;
	ado_sspjobs[sspId].dmaTd[3].dst = (uint32_t)&(pSSP->DR);    // Here the io target needs to be given as real io-address.
	ado_sspjobs[sspId].dmaTd[3].lli = 0;
	ado_sspjobs[sspId].dmaTd[3].ctrl = 0x00009000;

	Chip_GPDMA_Init(LPC_GPDMA);
	NVIC_EnableIRQ (DMA_IRQn);
}


void ADO_SSP_AddJob(uint32_t context, ado_sspid_t sspId,
                    uint8_t *txData, uint8_t *rxData,
                    uint16_t txLen, uint16_t rxLen,
                    AdoSSP_FinishedHandler(finish),
                    AdoSSP_ActivateHandler(activate)){
	bool startIt = false;
	ado_sspjobs_t *jobs = &ado_sspjobs[sspId];

	if (jobs->jobs_pending >= ADO_SSP_MAXJOBS) {
		/* Maximum amount of jobs stored, job can't be added! */
		jobs->ssp_job_error_counter++;
		if (finish != 0) {
			finish(context, SSP_JOB_BUFFER_OVERFLOW, 0, 0);		// Use the IRQ callback to Signal this error. Maybe a direct return value would be better here !?
		}
		return;
	}

	// Be sure the following operations will not be mixed up with an DMA TC-IRQ ending and changing the current/pending job idx.
	// *****
	NVIC_DisableIRQ (DMA_IRQn);

	uint8_t newJobIdx = (jobs->current_job + jobs->jobs_pending) % ADO_SSP_MAXJOBS;
	ado_sspjob_t *newJob = &(jobs->job[newJobIdx]);
	newJob->txData = txData;
	newJob->rxData = rxData;
	newJob->txSize = txLen & 0x0FFF;    // DMA transfer size has 12 bits. Max: 4095‬ bytes
	newJob->rxSize = rxLen & 0x0FFF;    // DMA transfer size has 12 bits. Max: 4095‬ bytes
	newJob->context = context;
	newJob->ADO_SSP_JobFinished_IRQCallback = finish;
	newJob->ADO_SSP_JobActivated_IRQCallback = activate;

	if (jobs->jobs_pending == 0) {
		startIt = true;
	}
	jobs->jobs_pending++;

	NVIC_EnableIRQ (DMA_IRQn);
	// *****
	// Now we allow DMA IRQs to be executed from here on.

	if (startIt) {
		// If this was the first job entered into an empty queue, we start the communication now.
		// If a DMA was running before this Disable/Enable IRQ block,
		// we should never get here because jobs_pending must have been already > 0 then!
		ADO_SSP_DUMP_RX(ADO_SSP_RegBase[sspId]);
		ADO_SSP_InitiateDMA(sspId, newJob);
	}
}


/**
 *  The DRM IRQ will be called only by the last DMA in the Scattered List.
 *  We only use the RX Channels IRQs here,
 *  as the RX-DMA will always be triggered after the last byte was sent out by the corresponding TX DMA Channel.
 */
void DMA_IRQHandler(void) {
	// Chip_GPIO_SetPinOutLow(LPC_GPIO, 0, 4);      // Debug IO
	uint32_t tcs = LPC_GPDMA->INTTCSTAT;

//	void(*callback0)(uint32_t context, ado_sspstatus_t jobStatus, uint8_t *rxData, uint16_t rxSize) = 0;
//	uint32_t 		cb0Context;
//	uint8_t *		cb0Data;
//	uint16_t		cb0Size;
//
//	void(*callback1)(uint32_t context, ado_sspstatus_t jobStatus, uint8_t *rxData, uint16_t rxSize) = 0;
//	uint32_t 		cb1Context;
//	uint8_t *		cb1Data;
//	uint16_t		cb1Size;

	if ( tcs & (1UL<<ADO_SSP0_RXDMACHANNEL) ) {
		LPC_GPDMA->INTTCCLEAR = (1UL << ADO_SSP0_RXDMACHANNEL);
		// This was SSP0 finishing its RX Channel.
		ado_sspjob_t *job = &ado_sspjobs[0].job[ado_sspjobs[0].current_job];
		//if (job->ADO_SSP_JobFinished_IRQCallback != 0) {      // safety or performance? which to choose here?

		ado_sspjobs[0].jobs_pending--;
		job->ADO_SSP_JobFinished_IRQCallback(job->context, ADO_SSP_JOBDONE, job->rxData, job->rxSize);
//		callback0 = job->ADO_SSP_JobFinished_IRQCallback;
//		cb0Context = job->context;
//		cb0Data = job->rxData;
//		cb0Size = job->rxSize;

		//}


		if (ado_sspjobs[0].jobs_pending > 0) {
			// Continue with nextJob
		    ADO_INCREMENT_SSPJOBNR(ado_sspjobs[0].current_job);
			ADO_SSP_InitiateDMA(ADO_SSP0,&ado_sspjobs[0].job[ado_sspjobs[0].current_job]);
		}
	}
	if ( tcs & (1<<ADO_SSP1_RXDMACHANNEL) ) {
		LPC_GPDMA->INTTCCLEAR = (1UL << ADO_SSP1_RXDMACHANNEL);
		// This was SSP1 finishing its RX Channel.
		ado_sspjob_t *job = &ado_sspjobs[1].job[ado_sspjobs[1].current_job];
		//if (job->ADO_SSP_JobFinished_IRQCallback != 0) {      // safety or performance? which to choose here?

		job->ADO_SSP_JobFinished_IRQCallback(job->context, ADO_SSP_JOBDONE, job->rxData, job->rxSize);
//		callback1 = job->ADO_SSP_JobFinished_IRQCallback;
//		cb1Context = job->context;
//		cb1Data = job->rxData;
//		cb1Size = job->rxSize;

		//}
		ado_sspjobs[1].jobs_pending--;
		if (ado_sspjobs[1].jobs_pending > 0) {
			// Continue with nextJob
			ADO_INCREMENT_SSPJOBNR(ado_sspjobs[1].current_job);
			ADO_SSP_InitiateDMA(ADO_SSP1,&ado_sspjobs[1].job[ado_sspjobs[1].current_job]);
		}
	}

//	if (callback0 != 0) {
//		callback0(cb0Context, ADO_SSP_JOBDONE, cb0Data, cb0Size);
//	}
//	if (callback1 != 0) {
//		callback1(cb1Context, ADO_SSP_JOBDONE, cb1Data, cb1Size);
//	}


	// Chip_GPIO_SetPinOutHigh(LPC_GPIO, 0, 4);     // Debug IO
}

// __attribute__((always_inline))		// This gives another 0,5% Performance improvement but it needs 150..200 bytes more prog memory!
void ADO_SSP_InitiateDMA(ado_sspid_t sspId, ado_sspjob_t *newJob) {
	DMA_TransferDescriptor_t *pDmaTd = ado_sspjobs[sspId].dmaTd;

	if (newJob->ADO_SSP_JobActivated_IRQCallback != 0) {
		// This could be used to activate a CS line other than the SSL of the SSP HW Unit.
		newJob->ADO_SSP_JobActivated_IRQCallback(newJob->context);
	}

	// Adjust the rx/tx addresses and length in prepared dma control structures.
	(pDmaTd+0)->ctrl =  0x00009000 | (uint32_t)(newJob->txSize);
	(pDmaTd+1)->ctrl =  0x88009000 | (uint32_t)(newJob->rxSize);
	(pDmaTd+1)->dst = (uint32_t)newJob->rxData;
	(pDmaTd+2)->ctrl =  0x04009000 | (uint32_t)(newJob->txSize);
	(pDmaTd+2)->src = (uint32_t)newJob->txData;
	(pDmaTd+3)->ctrl =  0x00009000 | (uint32_t)(newJob->rxSize);

#ifdef ADO_SSPDMA_BITFLIPSAFE												//TODO: re-test this version after all refactorings finished.....
	txDummy = 0xFF;
	// Rewrite the constant part of the DMA-Ctrl structures every time used.
	LPC_SSP_T *pSSP = (LPC_SSP_T *)ADO_SSP_RegBase[sspId];
	(pDmaTd+0)->src = ...;
	(pDmaTd+0)->dst = (uint32_t)&rxDummy;
	(pDmaTd+0)->lli = (uint32_t)(pDmaTd+1);
	(pDmaTd+1)->src = (uint32_t)&(pSSP->DR);
	(pDmaTd+1)->lli = 0;
	(pDmaTd+2)->dst = ....;
	(pDmaTd+2)->lli = (uint32_t)(pDmaTd+3);
	(pDmaTd+3)->src = (uint32_t)&txDummy;
	(pDmaTd+3)->dst = (uint32_t)&(pSSP->DR);
	(pDmaTd+3)->lli = 0;
#endif


	// TODO: the variants with re-adjusting source and destination 'HW Address' vs 'Channel feature select' should be refactored
	//	( in its own _SGTransfer() routine !???) to avoid this 'mis-alignment' of block 1-2 if only one is used. Also performance should improve
	//  with maybe less function calls needed to start it up.....
	//
	//
	if (newJob->txSize > 0) {
		(pDmaTd+1)->src = (uint32_t)&(ADO_SSP_RegBase[sspId]->DR);
		(pDmaTd+3)->dst = (uint32_t)&(ADO_SSP_RegBase[sspId]->DR);
		if (newJob->rxSize == 0) {
		    // Cutoff Rx-DMA after TX part
		    (pDmaTd+0)->lli = 0;
		    (pDmaTd+2)->lli = 0;
		    // Enable IRQ after first TX part finished.
		    (pDmaTd+0)->ctrl =  0x80009000 | (uint32_t)(newJob->txSize);
		} else {
		    // Re-establish lli links to 2nd part
		    (pDmaTd+0)->lli = (uint32_t)(pDmaTd+1);
		    (pDmaTd+2)->lli = (uint32_t)(pDmaTd+3);
		}
		Chip_GPDMA_SGTransfer(LPC_GPDMA, ADO_SSP_RxDmaChannel[sspId],(pDmaTd+0), GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA);
		Chip_GPDMA_SGTransfer(LPC_GPDMA, ADO_SSP_TxDmaChannel[sspId],(pDmaTd+2), GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA);
	} else {
		// TX block is 0, so we start with second DMA Blocks - only rx part of job
		(pDmaTd+1)->src = ADO_SSP_GPDMA_CONN_RX[sspId];     // The GPDMA_CONN_SSPn_Rx where n is 0/1 indicating used io source;
		(pDmaTd+3)->dst = ADO_SSP_GPDMA_CONN_TX[sspId];     // Here the GPDMA_CONN_SSPn_Tx for n=0/1 is needed to give the DMA io target.;
		Chip_GPDMA_SGTransfer(LPC_GPDMA, ADO_SSP_RxDmaChannel[sspId],(pDmaTd+1), GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA);
		Chip_GPDMA_SGTransfer(LPC_GPDMA, ADO_SSP_TxDmaChannel[sspId],(pDmaTd+3), GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA);
	}
}
