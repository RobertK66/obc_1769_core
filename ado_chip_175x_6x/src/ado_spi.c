/*
 *  obc_spi.c
 *
 *  Created on: 30.8.2021
 *  Author: Robert
 *
  */
#include "ado_spi.h"

#include <stdio.h>
#include <string.h>
#include <chip.h>


// Inline Block to clear SSP Rx Buffer.
//#define ADO_SSP_DUMP_RX(device) { uint32_t temp; (void)temp; while ((device->SR & SSP_STAT_RNE) != 0) { temp = device->DR; } }
//#define ADO_SSP_DUMP_RX(device) { while ((device->SR & SSP_STAT_RNE) != 0) { rxDummy = device->DR; } }

#define ADO_SPI_DUMP_RX() { while ((LPC_SPI->SR & SPI_SR_SPIF) != 0) { rxDummy1 = LPC_SPI->DR; } }

#define ADO_INCREMENT_SPIJOBNR(anyUint) { anyUint++; if (anyUint == ADO_SPI_MAXJOBS) {anyUint = 0; } }

//
//// Following constants hold the Configuration data for both SSPx interfaces. idx=0 -> SSP0 idx=1 ->SSP1
//const LPC_SSP_T *ADO_SSP_RegBase[] = {
//	LPC_SSP0,
//	LPC_SSP1
//};
//
//const CHIP_SYSCTL_CLOCK_T ADO_SSP_SysCtlClock[] = {
//	SYSCTL_CLOCK_SSP0,
//	SYSCTL_CLOCK_SSP1
//};
//
//const uint8_t ADO_SSP_RxDmaChannel[] = {
//	ADO_SSP0_RXDMACHANNEL,
//	ADO_SSP1_RXDMACHANNEL
//};
//
//const uint8_t ADO_SSP_TxDmaChannel[] = {
//	ADO_SSP0_TXDMACHANNEL,
//	ADO_SSP1_TXDMACHANNEL
//};
//
//const uint32_t ADO_SSP_GPDMA_CONN_RX[] = {
//	GPDMA_CONN_SSP0_Rx,
//	GPDMA_CONN_SSP1_Rx
//};
//
//const uint32_t ADO_SSP_GPDMA_CONN_TX[] = {
//	GPDMA_CONN_SSP0_Tx,
//	GPDMA_CONN_SSP1_Tx
//};

typedef struct ado_spijob_s
{
	uint8_t *txData;
	uint8_t *rxData;
	uint16_t txSize;
	uint16_t rxSize;
	uint32_t context;											// Any data can be stored here by client. Its fed back to the callbacks when job processes.
	AdoSPI_ActivateHandler(ADO_SPI_JobActivated_IRQCallback);	// Use this for activating chip Select, if SSP SSL not used.
	AdoSPI_FinishedHandler(ADO_SPI_JobFinished_IRQCallback);	// Do not process received data with this callback. Only signal data available to your main routines.
} ado_spijob_t;

typedef struct ado_spijobs_s
{
	ado_spijob_t job[ADO_SPI_MAXJOBS];
	uint16_t frames_processsed;
	uint8_t current_job;
	uint8_t jobs_pending;
	uint16_t spi_job_error_counter;
} ado_spijobs_t;

// local/module variables
ado_spijobs_t 	ado_spijobs;		// One structure holding the job queue
uint8_t 		rxDummy1;            // used as 'empty'/null destination
//uint8_t 		txDummy = 0xFF;     //

// Prototypes
void ADO_SPI_StartJob(uint8_t jobIdx);


// Init the legacy SPI bus.
void ADO_SPI_Init(uint8_t clockDivider, SPI_CLOCK_MODE_T clockMode) {
    LPC_SPI_T *pSPI =  LPC_SPI;

    // Setup SPI Clock
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SPI);
    Chip_Clock_SetPCLKDiv(SYSCTL_CLOCK_SPI, SYSCTL_CLKDIV_1);     // No additional IO Clock prescaler used.
    if (clockDivider < 0x08) {
        clockDivider = 0x08;
    }
    pSPI->CCR = clockDivider;

    // Setup mode to Master with 8 bit ....
    pSPI->CR = (pSPI->CR & (~0xF1C)) | SPI_MODE_MASTER | SPI_CR_BIT_EN | SPI_BITS_8 | clockMode | SPI_DATA_MSB_FIRST;


    // needed for SPI !? wenn ja wie ....
    // ADO_SSP_DUMP_RX(pSSP);
    ADO_SPI_DUMP_RX();

    // Enable and clear IRSs
    Chip_SPI_Int_Enable(pSPI);
    Chip_SPI_Int_ClearStatus(pSPI, SPI_INT_SPIF);

    /* Reset buffers to default values */
    ado_spijobs.current_job = 0;
    ado_spijobs.jobs_pending = 0;

    ado_spijobs.spi_job_error_counter = 0;
    //ado_spijobs.spi_rov_error_counter = 0;
    //ado_spijobs.spi_initialized = true;

    // TODO: soillte irgendwie ins system init unabhängig von einzelmodulen .....
    NVIC_EnableIRQ(SPI_IRQn);

}

void SPI_IRQHandler(void)
{
    // There is only one IRQ bit for SPI (bit0) so we clear this by tetting to 1 without checking it.
    LPC_SPI->INT = SPI_INT_SPIF;
    uint32_t status = LPC_SPI->SR;

    ado_spijob_t *job = &ado_spijobs.job[ado_spijobs.current_job];

    if ((status & SPI_SR_ERROR) > 0 ) {
        ado_spijobs.spi_job_error_counter++;
        job->ADO_SPI_JobFinished_IRQCallback(job->context, SPI_JOB_STATE_ERROR, 0, 0);
        return;
    }

    // read rx frame
    uint8_t *rxPtr = &rxDummy1;
    if (ado_spijobs.frames_processsed >= job->txSize) {
        rxPtr = &job->rxData[ado_spijobs.frames_processsed - job->txSize];
    }
    // read the byte of the frame
    *rxPtr = LPC_SPI->DR;

    ado_spijobs.frames_processsed++;
    if (ado_spijobs.frames_processsed < (job->txSize + job->rxSize)) {
        // We have to continue ...
        uint8_t txFrame = 0xFF;
        if (ado_spijobs.frames_processsed < job->txSize) {
            // still in tx part -> read next txFrame
            txFrame = job->txData[ado_spijobs.frames_processsed];
        }
        LPC_SPI->DR = txFrame;
    } else {
        // transmission finished.
        job->ADO_SPI_JobFinished_IRQCallback(job->context, SPI_JOBDONE, job->rxData, job->rxSize);
        ado_spijobs.jobs_pending--;
        if (ado_spijobs.jobs_pending > 0) {
            // Continue with nextJob
            ADO_INCREMENT_SPIJOBNR(ado_spijobs.current_job);
            ADO_SPI_StartJob(ado_spijobs.current_job);
        }
    }
}

void ADO_SPI_StartJob(uint8_t jobIdx) {
    ado_spijob_t *jobToStart = &ado_spijobs.job[jobIdx];

    jobToStart->ADO_SPI_JobActivated_IRQCallback(jobToStart->context);
    ado_spijobs.frames_processsed = 0;
    // now write first byte on bus
    uint8_t txFrame = 0xFF;
    if (ado_spijobs.frames_processsed < jobToStart->txSize) {
        txFrame = jobToStart->txData[ado_spijobs.frames_processsed];
    }
    // Iniitate first frame transfer
    LPC_SPI->DR = txFrame;
}


void ADO_SPI_AddJob(uint32_t context,
                    uint8_t *txData, uint8_t *rxData,
                    uint16_t txLen, uint16_t rxLen,
                    AdoSPI_FinishedHandler(finish),
                    AdoSPI_ActivateHandler(activate)){
	bool startIt = false;

	if (ado_spijobs.jobs_pending >= ADO_SPI_MAXJOBS) {
		/* Maximum amount of jobs stored, job can't be added! */
	    ado_spijobs.spi_job_error_counter++;
		if (finish != 0) {
			finish(context, SPI_JOB_BUFFER_OVERFLOW, 0, 0);		// Use the IRQ callback to Signal this error. Maybe a direct return value would be better here !?
		}
		return;
	}

	// Be sure the following operations will not be mixed up with an IRQ ending and changing the current/pending job idx.
	// *****
	NVIC_DisableIRQ (SPI_IRQn);

	uint8_t newJobIdx = (ado_spijobs.current_job + ado_spijobs.jobs_pending) % ADO_SPI_MAXJOBS;
	ado_spijob_t *newJob = &(ado_spijobs.job[newJobIdx]);
	newJob->txData = txData;
	newJob->rxData = rxData;
	newJob->txSize = txLen & 0x0FFF;    //  Max: 4095‬ bytes
	newJob->rxSize = rxLen & 0x0FFF;    //  Max: 4095‬ bytes
	newJob->context = context;
	newJob->ADO_SPI_JobFinished_IRQCallback = finish;
	newJob->ADO_SPI_JobActivated_IRQCallback = activate;

	if (ado_spijobs.jobs_pending == 0) {
		startIt = true;
	}
	ado_spijobs.jobs_pending++;

	NVIC_EnableIRQ (SPI_IRQn);
	// *****
	// Now we allowIRQs to be executed from here on.

	if (startIt) {
	    // clear pending input
	    ADO_SPI_DUMP_RX();

	    ADO_SPI_StartJob(ado_spijobs.current_job);

//		// If this was the first job entered into an empty queue, we start the communication now.
//	    ado_spijob_t *jobToStart = ado_spijobs.job[ado_spijobs.current_job];
//
//	    jobToStart->ADO_SPI_JobActivated_IRQCallback(jobToStart->context);
//	    ado_spijobs.frames_processsed = 0;
//
//	    // now write first byte on bus
//	    uint8_t txFrame = 0xFF;
//	    if (ado_spijobs.frames_processsed < jobToStart->txLen) {
//	        uint8_t txFrame = jobToStart->txData[ado_spijobs.frames_processsed];
//	    }
//	    // Check rx len -> nothing to process at all !?
//	    // Iniitate first frame transfer
//	    LPC_SPI->DR = frame;

	    // Wait for irq to get rx frame and decide on next one....
	}
}

