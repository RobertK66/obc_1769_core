/*
===============================================================================
 Name        : ClimbObc.c
 Author      : Robert
 Created on	 : 05.09.2021
===============================================================================
*/
#include "ClimbObc.h"

#include <chip.h>

//#include <stdio.h>
// To Use printf you have to:
//    - link against Redlib "nohost-nf" Librrary  (min: without hosting and no files)
//    - provide your implementation of __sys_write[, __sys_readc] functions
//
// When using CLI module of ADO lib, this comes with its own __sys_write, __sys_readc in order to redirect to the CLI target (UART or SWO-Trace)
//
//int __sys_write(int iFileHandle, char *pcBuffer, int iLength)
//{
//	unsigned int i;
//	for (i = 0; i < iLength; i++) {
//
//	}
//	return iLength;
//}

// local prototypes
void BA_GpioInit(const GPIO_INIT_T* pinArray, uint32_t arrayLength);

// Main loop
int main(void) {
    // Read clock settings and update SystemCoreClock variable.
	// (Here in main() this sets the global available SystemCoreClock variable for the first time after all SystemInits finished)
    SystemCoreClockUpdate();

    // Force the counter to be placed into memory
    volatile static int i = 0 ;
    // Enter an infinite loop, just incrementing a counter
    while(1) {
        i++ ;
        // "Dummy" NOP to allow source level single
        // stepping of tight while() loop
        __asm volatile ("nop");
    }
    return 0 ;
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
	Chip_IOCON_SetPinMuxing(LPC_IOCON, pinmuxing, sizeof(pinmuxing) / sizeof(PINMUX_GRP_T));
	// For the GPIO - Func0 Pins now we set all direction and initial values for all outputs
	BA_GpioInit(gpioinit, sizeof(gpioinit)/sizeof(GPIO_INIT_T));


	// Now we can switch to XTAL and wait until everything is stable. TODO: -> move this into its own mainloop module "System" !??
	Chip_SetupXtalClocking();		// Asumes 12Mhz Quarz -> PLL0 frq=384Mhz -> CPU frq=96MHz
	/* Setup FLASH access to 4 clocks (100MHz clock) */
	Chip_SYSCTL_SetFLASHAccess(FLASHTIM_100MHZ_CPU);
}

// Set all GPIO direction bits, and initial values.
void BA_GpioInit(const GPIO_INIT_T* pinArray, uint32_t arrayLength) {
	/* Initializes GPIO */
	Chip_GPIO_Init(LPC_GPIO);

	/* Initialize IO Dirs and set default values for all outputs */
	uint32_t ix;
	for (ix = 0; ix < arrayLength; ix++ ) {
		Chip_GPIO_WriteDirBit(LPC_GPIO, pinArray[ix].port, pinArray[ix].pinNr, pinArray[ix].output );
		if ( pinArray[ix].output ) {
			Chip_GPIO_SetPinState(LPC_GPIO,pinArray[ix].port, pinArray[ix].pinNr, pinArray[ix].initVal);
		}
	}
}
