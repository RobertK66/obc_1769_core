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

#include "mod/ado_modules.h"

#include "mod/tim/obc_time.h"
#include "mod/hw_check.h"
#include "mod/l2_debug_com.h"
#include "mod/mem/ado_sdcard.h"
#include "mod/mem/ado_mram.h"

#include "mod/l3_sensors.h"


#include "mod/l7_climb_app.h"






void CsMram01(bool select) {
    Chip_GPIO_SetPinState(LPC_GPIO, MRAM_CS01_PORT, MRAM_CS01_PIN, !select);
}
void CsMram02(bool select) {
	Chip_GPIO_SetPinState(LPC_GPIO, MRAM_CS02_PORT, MRAM_CS02_PIN, !select);
}
void CsMram03(bool select) {
	Chip_GPIO_SetPinState(LPC_GPIO, MRAM_CS03_PORT, MRAM_CS03_PIN, !select);
}
void CsMram11(bool select) {
	Chip_GPIO_SetPinState(LPC_GPIO, MRAM_CS11_PORT, MRAM_CS11_PIN, !select);
}
void CsMram12(bool select) {
	Chip_GPIO_SetPinState(LPC_GPIO, MRAM_CS12_PORT, MRAM_CS12_PIN, !select);
}
void CsMram13(bool select) {
	Chip_GPIO_SetPinState(LPC_GPIO, MRAM_CS13_PORT, MRAM_CS13_PIN, !select);
}

void CsSdCard(bool select) {
#ifdef 	P0_PIN_SPI_CS_SD
    Chip_GPIO_SetPinState(LPC_GPIO, 0, P0_PIN_SPI_CS_SD, !select);
#else
    // TODO: ?? We have 2 SD Cards on OEM... Board with expansion....
    // and there they are wired to SSP0 /SSP1
    // climb obc core code not tested (yet) with old hardware ....
#endif
}

static const mram_chipinit_t Mrams[] = {
		{ADO_SSP0, CsMram01},
		{ADO_SSP0, CsMram02},
		{ADO_SSP0, CsMram03},
		{ADO_SSP1, CsMram11},
		{ADO_SSP1, CsMram12},
		{ADO_SSP1, CsMram13}
};
static const mram_chipinit_array_t Chips = {
	(sizeof(Mrams)/sizeof(mram_chipinit_t)), Mrams
};

static const sdcard_init_t SdCards[] = {
		{ADO_SBUS_SPI, CsSdCard},
//		{ADO_SBUS_SSP0, CsSdCard0},
//		{ADO_SBUS_SSP1, CsSdCard1}
};
static const sdcard_init_array_t Cards = {
	(sizeof(SdCards)/sizeof(sdcard_init_t)), SdCards
};


static const MODULE_DEF_T Modules[] = {
		MOD_INIT( deb_init, deb_main, LPC_UART2),
		MOD_INIT( tim_init, tim_main, NULL ),
		MOD_INIT( hwc_init, hwc_main, &ObcPins ),
		MOD_INIT( MramInitAll, MramMain, &Chips),
		MOD_INIT( SdcInitAll, SdcMain, &Cards),
		MOD_INIT( sen_init, sen_main, NULL),
		MOD_INIT( app_init, app_main, NULL)

};
#define MODULE_CNT (sizeof(Modules)/sizeof(MODULE_DEF_T))

int main(void) {
    // Read clock settings and update SystemCoreClock variable.
	// (Here in main() this sets the global available SystemCoreClock variable for the first time after all SystemInits finished)
 	SystemCoreClockUpdate();

    // Layer 1 - Bus Inits
    ADO_SSP_Init(ADO_SSP0, 24000000, SSP_CLOCK_MODE3);
    ADO_SSP_Init(ADO_SSP1, 24000000, SSP_CLOCK_MODE3);
    ADO_SPI_Init(0x08, SPI_CLOCK_MODE3);                                   // Clock Divider 0x08 -> fastest, must be even: can be up to 0xFE for slower SPI Clocking

    // Init all other modules
    for (int i=0; i < MODULE_CNT; i++) {
    	Modules[i].init(Modules[i].initdata);
    }

    // Enter an infinite loop calling all registered modules main function.
    while(1) {
    	for (int i=0; i < MODULE_CNT; i++) {
    		Modules[i].main();
    	}
    }
    return 0;
}

// This init gets called after reset - memory clear - copy of memory sections (constants or static initialized stuff) and all
// vector tables are set up for the used Chip.
// We (mis)use it here to make our own "Board Abstraction" and Hardware initialization.
void Chip_SystemInit(void) {
	// Note:
	// This is called before main loop and redlib-init is not finalized here (e.g. printf() not working yet)
	// also be aware that no other CHIP_ and Sytem_ functions are somehow initialized here, if needed (e.g. SystemCoreClock variable)!

	// To get things up and running we always start with internal IRC Clocking.
	Chip_SetupIrcClocking();
	// Next is to get the IOs connected to the right functions and initialise the GPIO with its dir and initval
	Chip_IOCON_SetPinMuxing2(LPC_IOCON, pinmuxing2, sizeof(pinmuxing2) / sizeof(PINMUX_GRP_T2));

	// Now we can switch to XTAL and wait until everything is stable. TODO: -> move this into its own mainloop module "System" !??
	Chip_SetupXtalClocking();		// Asumes 12Mhz Quarz -> PLL0 frq=384Mhz -> CPU frq=96MHz
	/* Setup FLASH access to 4 clocks (100MHz clock) */
	Chip_SYSCTL_SetFLASHAccess(FLASHTIM_100MHZ_CPU);
}

