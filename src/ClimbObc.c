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
#include "mod/ai2c/obc_i2c.h"

#include <ado_modules.h>
#include <mod/ado_sdcard.h>
#include <mod/ado_mram.h>
#include "mod/ado_wirebus.h"

#include "mod/tim/obc_time.h"
#include "mod/tim/climb_gps.h"
#include "mod/hw_check.h"
#include "mod/l2_debug_com.h"

#include "mod/l3_sensors.h"
#include "mod/mem/obc_memory.h"
#include "mod/l7_climb_app.h"

#include "mod/thr/thr.h"
#include "mod/l4_thruster.h"



//typedef struct {
//	uint8_t resetBits;
//} init_report_t;
//

#if BA_BOARD == BA_OM13085_EM2T
// EM2T Test Hardware has 2 SD Cards connected to SSP0/SSP1
static const sdcard_init_t SdCards[] = {
	{ADO_SBUS_SSP0, PTR_FROM_IDX(PINIDX_SSP0_CS_SD)},
	{ADO_SBUS_SSP1, PTR_FROM_IDX(PINIDX_SSP1_CS_SD)}
};
#else
// OBC Hardware has one SD Card connected to SPI
static const sdcard_init_t SdCards[] = {
	{ADO_SBUS_SPI, PTR_FROM_IDX(PINIDX_SPI_CS_SD)}
};
#endif
static const sdcard_init_array_t Cards = {
	(sizeof(SdCards)/sizeof(sdcard_init_t)), SdCards
};

static const mram_chipinit_t Mrams[] = {
		{ADO_SSP0, PTR_FROM_IDX(PINIDX_SSP0_MRAM_CS1) },
		{ADO_SSP0, PTR_FROM_IDX(PINIDX_SSP0_MRAM_CS2) },
		{ADO_SSP0, PTR_FROM_IDX(PINIDX_SSP0_MRAM_CS3) },
		{ADO_SSP1, PTR_FROM_IDX(PINIDX_SSP1_MRAM_CS1) },
		{ADO_SSP1, PTR_FROM_IDX(PINIDX_SSP1_MRAM_CS2) },
		{ADO_SSP1, PTR_FROM_IDX(PINIDX_SSP1_MRAM_CS3) },
};
static const mram_chipinit_array_t Chips = {
	(sizeof(Mrams)/sizeof(mram_chipinit_t)), Mrams
};

static const mem_init_t MemoryInit = { PTR_FROM_IDX(PINIDX_SD_VCC_EN) };

static init_report_t InitReport;

static gps_initdata_t GpsInit = {
		LPC_UART0,
		PTR_FROM_IDX(PINIDX_GPIO4_CP),
		PTR_FROM_IDX(PINIDX_STACIE_C_IO1_P)
};

static thr_initdata_t ThrInit = {
		LPC_UART1, /// WILL USE UART1 since it is on Y+ side, which according to doccumentation given to me should be used for RS485 thruster LPC_UART1
		PTR_FROM_IDX(PINIDX_GPIO4_CP),
		PTR_FROM_IDX(PINIDX_STACIE_C_IO1_P)
};


// List of (wire) busses to be initialized.
static ado_wbus_config_t WBuses[] = {
		{ADO_WBUS_SPI,    0, LPC_SPI  },
		{ADO_WBUS_SSPDMA, 0, LPC_SSP0 },
		{ADO_WBUS_SSPDMA, 0, LPC_SSP1 },
		{ADO_WBUS_I2C, 	  0, LPC_I2C0 },
		{ADO_WBUS_I2C,    0, LPC_I2C1 },
		{ADO_WBUS_I2C,    0, LPC_I2C2 }
};
#define WBUS_CNT (sizeof(WBuses)/sizeof(ado_wbus_config_t))

static const MODULE_DEF_T Modules[] = {
		MOD_INIT( deb_init, deb_main, LPC_UART2),
		MOD_INIT( timInit, timMain, &InitReport ),
		MOD_INIT( hwc_init, hwc_main, &ObcPins ),
		MOD_INIT( MramInitAll, MramMain, &Chips),
		MOD_INIT( SdcInitAll, SdcMain, &Cards),
		MOD_INIT( sen_init, sen_main, NULL),
		MOD_INIT( memInit, memMain, &MemoryInit),
		MOD_INIT( gpsInit, gpsMain, &GpsInit),
		MOD_INIT( app_init, app_main, NULL),
		MOD_INIT( thrInit, thrMain, &ThrInit),
		MOD_INIT( app_init, app_main, NULL),
		MOD_INIT( l4_thruster_init, l4_thruster_main, NULL)

};
#define MODULE_CNT (sizeof(Modules)/sizeof(MODULE_DEF_T))

int main(void) {
    // Read clock settings and update SystemCoreClock variable.
	// (Here in main() this sets the global available SystemCoreClock variable for the first time after all SystemInits finished)
	SystemCoreClockUpdate();

	// Determine reset reason
	InitReport.resetBits = LPC_SYSCON->RSID & 0x003F;
	InitReport.rtcOscDelayMs = 0;

	// Clear all (set) bits in this register (if possible).
	LPC_SYSCON->RSID = InitReport.resetBits;

	if (Chip_GPIO_GetPinState(LPC_GPIO, PORT_FROM_IDX(PINIDX_EXT_WDT_TRIGGERED), PINNR_FROM_IDX(PINIDX_EXT_WDT_TRIGGERED))) {
		InitReport.hwWatchdog = true;
		// Reset the WD Flip Flop. (Clear pin must be initialized as output now)
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, PORT_FROM_IDX(PINIDX_CLR_WDT_FLPFLP), PINNR_FROM_IDX(PINIDX_CLR_WDT_FLPFLP));
		Chip_GPIO_SetPinOutLow(LPC_GPIO, PORT_FROM_IDX(PINIDX_CLR_WDT_FLPFLP), PINNR_FROM_IDX(PINIDX_CLR_WDT_FLPFLP));
	}
	if (Chip_GPIO_GetPinState(LPC_GPIO, PORT_FROM_IDX(PINIDX_BL_SEL1), PINNR_FROM_IDX(PINIDX_BL_SEL1))) {
		// Every 2nd Reset.
		InitReport.oddEven = true;
	}

	ADO_WBUS_Init(WBuses, WBUS_CNT);


    // Layer 1 - Bus Inits
    ADO_SSP_Init(ADO_SSP0, 24000000, SSP_CLOCK_MODE3);
    ADO_SSP_Init(ADO_SSP1, 24000000, SSP_CLOCK_MODE3);
    ADO_SPI_Init(0x08, SPI_CLOCK_MODE3);    // Clock Divider 0x08 -> fastest, must be even: can be up to 0xFE for slower SPI Clocking

    // Onboard I2C
    init_i2c(LPC_I2C1, 100);		// 100 kHz

    // Init all other modules
    for (int i=0; i < MODULE_CNT; i++) {
    	Modules[i].init(Modules[i].initdata);
    }

    // End WD reset pulse and make clr pin input again
    Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_CLR_WDT_FLPFLP), PINNR_FROM_IDX(PINIDX_CLR_WDT_FLPFLP));
    Chip_GPIO_SetPinDIRInput(LPC_GPIO, PORT_FROM_IDX(PINIDX_CLR_WDT_FLPFLP), PINNR_FROM_IDX(PINIDX_CLR_WDT_FLPFLP));


    SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_INIT, &InitReport, sizeof(InitReport) );

    // Enter an infinite loop calling all registered modules main function.
    while(1) {
    	for (int i=0; i < MODULE_CNT; i++) {
    		Modules[i].main();
    	}
    	Chip_GPIO_SetPinToggle(LPC_GPIO, PORT_FROM_IDX(PINIDX_WATCHDOG_FEED), PINNR_FROM_IDX(PINIDX_WATCHDOG_FEED));
    }
    return 0;
}

// This init gets called after reset - memory clear - copy of memory sections (constants or static initialized stuff) and all
// vector tables are set up for the used Chip.
void Chip_SystemInit(void) {
	// Note:
	// This is called before main loop and redlib-init is not finalized here (e.g. printf() not working yet)!
	// also be aware that no other CHIP_ and Sytem_ functions are somehow initialized here, if needed (e.g. SystemCoreClock variable)!

	// To get things up and running we always start with internal IRC Clocking.
	Chip_SetupIrcClocking();
	// Next is to get the IOs connected to the right functions and initialise the GPIO with its dir/opendrain and initval
	Chip_IOCON_SetPinMuxing2(LPC_IOCON, pinmuxing2, sizeof(pinmuxing2) / sizeof(PINMUX_GRP_T2));

	// Now we can switch to XTAL and wait until everything is stable. TODO: -> move this into its own mainloop module "System" !??
	Chip_SetupXtalClocking();		// Asumes 12Mhz Quarz -> PLL0 frq=384Mhz -> CPU frq=96MHz
	/* Setup FLASH access to 4 clocks (100MHz clock) */
	Chip_SYSCTL_SetFLASHAccess(FLASHTIM_100MHZ_CPU);
}

