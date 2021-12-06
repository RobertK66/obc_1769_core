/*
===============================================================================
 Name        : ClimbObc.c
 Author      : Robert
 Created on	 : 05.09.2021
===============================================================================
*/
#include "ClimbObc.h"

#include <chip.h>

#include <ado_sspdma.h>
#include <ado_spi.h>

#include "mod/tim/obc_time.h"
#include "mod/hw_check.h"
#include "mod/l2_debug_com.h"
#include "mod/mem/ado_sdcard.h"
#include "mod/mem/ado_mram.h"

#include "mod/l7_climb_app.h"

// local prototypes
//void BA_GpioInit(const GPIO_INIT_T* pinArray, uint32_t arrayLength);


// Chipselects TODO:check pin abstraction ... PORT!!!!
void CsMram01(bool select) {
    Chip_GPIO_SetPinState(LPC_GPIO, 0, P0_PIN_SSP0_MRAM_CS1, !select);
}
void CsMram02(bool select) {
#ifdef P2_PIN_SSP0_MRAM_CS2
    Chip_GPIO_SetPinState(LPC_GPIO, 2, P2_PIN_SSP0_MRAM_CS2, !select);
#else
    Chip_GPIO_SetPinState(LPC_GPIO, 1, P1_PIN_SSP0_MRAM_CS2, !select);
#endif
}
void CsMram03(bool select) {
#ifdef P2_PIN_SSP0_MRAM_CS3
    Chip_GPIO_SetPinState(LPC_GPIO, 2, P2_PIN_SSP0_MRAM_CS3, !select);
#else
    Chip_GPIO_SetPinState(LPC_GPIO, 1, P1_PIN_SSP0_MRAM_CS3, !select);
#endif
}
void CsMram11(bool select) {
#ifdef  P2_PIN_SSP1_MRAM_CS1
    Chip_GPIO_SetPinState(LPC_GPIO, 2, P2_PIN_SSP1_MRAM_CS1, !select);
#else
    Chip_GPIO_SetPinState(LPC_GPIO, 0, P0_PIN_SSP1_MRAM_CS1, !select);
#endif
}
void CsMram12(bool select) {
    Chip_GPIO_SetPinState(LPC_GPIO, 0, P0_PIN_SSP1_MRAM_CS2, !select);
}
void CsMram13(bool select) {
#ifdef 	P1_PIN_SSP1_MRAM_CS3
    Chip_GPIO_SetPinState(LPC_GPIO, 1 , P1_PIN_SSP1_MRAM_CS3, !select);
#else
    Chip_GPIO_SetPinState(LPC_GPIO, 0 , P0_PIN_SSP1_MRAM_CS3, !select);
#endif
}

void CsSdCard(bool select) {
#ifdef 	P0_PIN_SPI_CS_SD
    Chip_GPIO_SetPinState(LPC_GPIO, 0, P0_PIN_SPI_CS_SD, !select);
#else
    // TODO: ?? We have 2 SD Cards on OEM... Board with expansion....
#endif
}



// Main loop
int main(void) {
    // Read clock settings and update SystemCoreClock variable.
	// (Here in main() this sets the global available SystemCoreClock variable for the first time after all SystemInits finished)
	SystemCoreClockUpdate();

	DebInitModule(LPC_UART2);

	TimInitModule();
    HwcInitModule(pinmuxing2);

    // using 24000000 gives some init errors here.....
    ADO_SSP_Init(ADO_SSP0, 12000000, SSP_CLOCK_MODE3);
    ADO_SSP_Init(ADO_SSP1, 12000000, SSP_CLOCK_MODE3);
    ADO_SPI_Init(0x08, SPI_CLOCK_MODE3);                                   // Clock Divider 0x08 -> fastest, must be even: can be up to 0xFE for slower SPI Clocking

    void *sdCard;
    sdCard = SdcInitSPI(CsSdCard);
    //AdoSdcardCliInit(1, sdCard);

    MramInit(0, ADO_SSP0, CsMram01);
    MramInit(1, ADO_SSP0, CsMram02);
    MramInit(2, ADO_SSP0, CsMram03);
    MramInit(3, ADO_SSP1, CsMram11);
    MramInit(4, ADO_SSP1, CsMram12);
    MramInit(5, ADO_SSP1, CsMram13);

    AppInitModule();

    // Enter an infinite loop.
    while(1) {
    	TimMain();
        HwcMain();
        DebMain();
        MramMain();
        SdcMain(sdCard);
        AppMain();
    }
    return 0;
}

// This Init gets called after Reset - memory clear - copy of memory sections (constants or static initialized stuff) and all
// vector tables are set up for the used Chip.
// We (mis)use it here to make our own "Board Abstraction" and Hardware initialization.
void Chip_SystemInit(void) {
	// Note:
	// This is called before main loop and redlib-init is not finalized here (e.g. printf() not working yet)
	// also be aware that no other CHIP_ and Sytem_ functions are somehow initialized here, if needed (e.g. SystemCoreClock variable)!

	// To get things up and running we always start with internal IRC Clocking.
	Chip_SetupIrcClocking();
	// Next is to get the IOs connected to the right functions
	Chip_IOCON_SetPinMuxing2(LPC_IOCON, pinmuxing2, sizeof(pinmuxing2) / sizeof(PINMUX_GRP_T2));
	// For the GPIO - Func0 Pins now we set all direction and initial values for all outputs
	//BA_GpioInit(gpioinit_old, sizeof(gpioinit_old)/sizeof(GPIO_INIT_T));


	// Now we can switch to XTAL and wait until everything is stable. TODO: -> move this into its own mainloop module "System" !??
	Chip_SetupXtalClocking();		// Asumes 12Mhz Quarz -> PLL0 frq=384Mhz -> CPU frq=96MHz
	/* Setup FLASH access to 4 clocks (100MHz clock) */
	Chip_SYSCTL_SetFLASHAccess(FLASHTIM_100MHZ_CPU);
}

//// Set all GPIO direction bits, and initial values.
//void BA_GpioInit(const GPIO_INIT_T* pinArray, uint32_t arrayLength) {
//	/* Initializes GPIO */
//	Chip_GPIO_Init(LPC_GPIO);
//
//	/* Initialize IO Dirs and set default values for all outputs */
//	uint32_t ix;
//	for (ix = 0; ix < arrayLength; ix++ ) {
//		Chip_GPIO_WriteDirBit(LPC_GPIO, pinArray[ix].port, pinArray[ix].pinNr, pinArray[ix].output );
//		if ( pinArray[ix].output ) {
//			Chip_GPIO_SetPinState(LPC_GPIO,pinArray[ix].port, pinArray[ix].pinNr, pinArray[ix].initVal);
//		}
//	}
//}
//


