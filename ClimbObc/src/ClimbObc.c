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

#include "mod/tim/obc_time.h"
#include "mod/tim/climb_gps.h"
#include "mod/hw_check.h"
#include "mod/l2_debug_com.h"

#include "mod/l3_sensors.h"
#include "mod/mem/obc_memory.h"
#include "mod/l7_climb_app.h"

//#include "radtest/radtest.h"
#include "mod/thr/thr.h"
#include "mod/l4_thruster.h"



#include "mod/psu/psu.h"

// prototypes
void init_mainlooptimer(LPC_TIMER_T* pTimer,  CHIP_SYSCTL_CLOCK_T timBitIdx);


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

static const mram_chipinit_t Mrams[] = {									// defines the Chip Select GPIOs used for the MRAM Chips
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

static const mem_init_t MemoryInit = {
		PTR_FROM_IDX(PINIDX_SD_VCC_EN)	// defines the GPIO pin which enables the Power Supply of the SD-Card
};

static const gps_initdata_t GpsInit = {
		LPC_UART0,
		PTR_FROM_IDX(PINIDX_GPIO4_CP),
		PTR_FROM_IDX(PINIDX_STACIE_C_IO1_P)
};

static const thr_initdata_t ThrInit = {
		LPC_UART1, ///Y+ sidepanel
};

static init_report_t InitReport;

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
		MOD_INIT( l4_thruster_init, l4_thruster_main, NULL),
		MOD_INIT( psu_init, psu_main, NULL)

};
#define MODULE_CNT (sizeof(Modules)/sizeof(MODULE_DEF_T))

// Main variables
static MODULE_STATUS_T ModStat[MODULE_CNT];
static MODULES_STATUS_T ModulesStatus;

static uint32_t inline calculatelongest_runtime(uint32_t started, uint32_t finished, uint32_t longest) {
	uint32_t runtime;
	if (finished >= started) {
		runtime = finished - started;
	} else {
		runtime = started - finished;
	}
	if (runtime > longest) {
		return runtime;
	} else {
		return longest;
	}
}


void mainloop(void) {
	  // Enter an infinite loop calling all registered modules main function.
	    while(1) {
	    	uint32_t finishedAtTicks;
	    	ModulesStatus.mainLoopStartedAtTicks = LPC_TIMER0->TC;
	    	for (int i=0; i < MODULE_CNT; i++) {
	    		ModulesStatus.curExecutingPtr = &ModStat[i];
	        	ModulesStatus.startedAtTicks = LPC_TIMER0->TC;
	    		Modules[i].main();
	    		finishedAtTicks = LPC_TIMER0->TC;
	    		ModulesStatus.curExecutingPtr->longestExecutionTicks = calculatelongest_runtime( ModulesStatus.startedAtTicks,
	    				                                                                         finishedAtTicks,
																							     ModulesStatus.curExecutingPtr->longestExecutionTicks);
	    	}

	    	finishedAtTicks = LPC_TIMER0->TC;
	    	ModulesStatus.longestMainLoopTicks = calculatelongest_runtime( ModulesStatus.mainLoopStartedAtTicks,
	    																   finishedAtTicks,
																		   ModulesStatus.longestMainLoopTicks);

	    	// Feed the watchdog
	    	Chip_GPIO_SetPinToggle(LPC_GPIO, PORT_FROM_IDX(PINIDX_WATCHDOG_FEED), PINNR_FROM_IDX(PINIDX_WATCHDOG_FEED));
	    }
}

int main(void) {
    // Read clock settings and update SystemCoreClock variable.
	// (Here in main() this sets the global available SystemCoreClock variable for the first time after all SystemInits finished)
	SystemCoreClockUpdate();

	// Determine reset reason
	InitReport.resetBits = LPC_SYSCON->RSID & 0x003F;
	InitReport.rtcOscDelayMs = 0;
	// Clear all (set) bits in this register (if possible).
	LPC_SYSCON->RSID = InitReport.resetBits;
	// Try to figure out if this was Hardware watchdog -> not possible in EM2!?
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

    // Layer 1 - Bus Inits
	// -------------------
	// SSP&SPI
    ADO_SSP_Init(ADO_SSP0, 24000000, SSP_CLOCK_MODE3);	// MRAM x3
    ADO_SSP_Init(ADO_SSP1, 24000000, SSP_CLOCK_MODE3);  // MRAM x3
    ADO_SPI_Init(0x08, SPI_CLOCK_MODE3);    // SDCard, Clock Divider 0x08 -> fastest, must be even: can be up to 0xFE for slower SPI Clocking

    // I2C buses
    // init_i2c(LPC_I2C0, 100);		// 100 kHz  C/D
    init_i2c(LPC_I2C1, 100);		// 100 kHz  on-board
    init_i2c(LPC_I2C2, 100);		// 100 kHz  A/B

    init_mainlooptimer(LPC_TIMER0, SYSCTL_CLOCK_TIMER0);		

    // Init all modules
    for (int i=0; i < MODULE_CNT; i++) {
    	ModStat[i].moduleIdx = i;
    	ModulesStatus.curExecutingPtr = &ModStat[i];
    	Modules[i].init(Modules[i].initdata);
    }

    // End WD reset pulse and make clr pin input again.
    Chip_GPIO_SetPinOutHigh(LPC_GPIO, PORT_FROM_IDX(PINIDX_CLR_WDT_FLPFLP), PINNR_FROM_IDX(PINIDX_CLR_WDT_FLPFLP));
    Chip_GPIO_SetPinDIRInput(LPC_GPIO, PORT_FROM_IDX(PINIDX_CLR_WDT_FLPFLP), PINNR_FROM_IDX(PINIDX_CLR_WDT_FLPFLP));

    SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_INIT, &InitReport, sizeof(InitReport) );

    // Enter an infinite loop calling all registered modules main function.
//    while(1) {
//    	uint32_t finishedAtTicks;
//    	ModulesStatus.mainLoopStartedAtTicks = LPC_TIMER0->TC;
//    	for (int i=0; i < MODULE_CNT; i++) {
//    		ModulesStatus.curExecutingPtr = &ModStat[i];
//        	ModulesStatus.startedAtTicks = LPC_TIMER0->TC;
//    		Modules[i].main();
//    		finishedAtTicks = LPC_TIMER0->TC;
//    		ModulesStatus.curExecutingPtr->longestExecutionTicks = calculatelongest_runtime( ModulesStatus.startedAtTicks,
//    				                                                                         finishedAtTicks,
//																						     ModulesStatus.curExecutingPtr->longestExecutionTicks);
//    	}
//
//    	finishedAtTicks = LPC_TIMER0->TC;
//    	ModulesStatus.longestMainLoopTicks = calculatelongest_runtime( ModulesStatus.mainLoopStartedAtTicks,
//    																   finishedAtTicks,
//																	   ModulesStatus.longestMainLoopTicks);
//
//    	// Feed the watchdog
//    	Chip_GPIO_SetPinToggle(LPC_GPIO, PORT_FROM_IDX(PINIDX_WATCHDOG_FEED), PINNR_FROM_IDX(PINIDX_WATCHDOG_FEED));
//    }
    mainloop();
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

void init_mainlooptimer(LPC_TIMER_T* pTimer, CHIP_SYSCTL_CLOCK_T timBitIdx) {
	Chip_TIMER_TIMER_SetCountClockSrc(pTimer, TIMER_CAPSRC_RISING_PCLK, 0); // Use internal clock for this timer (timer->CTCR)
	Chip_Clock_EnablePeriphClock(timBitIdx);								// (PCONP) power enable the timer block
	Chip_Clock_SetPCLKDiv(timBitIdx, SYSCTL_CLKDIV_8);  					// Clk/8 -> 96/4 = 12Mhz -> Tick = 83,3 ns
	Chip_TIMER_PrescaleSet(pTimer, 119); 									// (timer->PR) Prescaler = 119+1 -> Increment TC at every 120 Ticks -> 10us (when clk is 96 Mhz and Clck divide / 8)
	Chip_TIMER_Enable(pTimer);												// (timer->TCR) enable timer to start ticking
}

void main_showruntimes_cmd(int argc, char *argv[]) {
	  SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_RAWDATA, ModStat , sizeof(ModStat) );
	  SysEvent(MODULE_ID_CLIMBAPP, EVENT_INFO, EID_APP_RAWDATA, &ModulesStatus.longestMainLoopTicks , sizeof( ModulesStatus.longestMainLoopTicks) );
}


typedef struct __attribute__((packed)) ContextStateFrame {
  //uint32_t sp_old; 				// SP prior to fault !?
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t return_address;
  uint32_t xpsr;
  //uint32_t dummy;
} sContextStateFrame;

//#define HARDFAULT_HANDLING_ASM(_x)               \
//  __asm volatile(                                \
//      "tst lr, #4 \n"                            \
//      "ite eq \n"                                \
//      "mrseq r0, msp \n"                         \
//      "mrsne r0, psp \n"                         \
//      "b my_fault_handler_c \n"                  \
//                                                 )

__attribute__((optimize("O0")))
void my_fault_handler_c(sContextStateFrame *frame) {
	// Logic for dealing with the exception. Typically:
	//  - log the fault which occurred for postmortem analysis
	//  - If the fault is recoverable,
	//    - clear errors and return back to Thread Mode
	//  - else
	//    - reboot system
	int d;
	d = frame->lr;				// This is the prog-pointer to last stored return value code before fault occured -> translate with map file....
	//d = frame->return_address; 	// this is the PC which triggered the fault.


	// Configurable Fault Status Register
    // Consists of MMSR, BFSR and UFSR
	uint32_t _CFSR = SCB->CFSR; // (*((volatile unsigned long *)(0xE000ED28))) ;

	// Hard Fault Status Register
	uint32_t _HFSR = SCB->HFSR; //(*((volatile unsigned long *)(0xE000ED2C))) ;

	// Debug Fault Status Register
	uint32_t _DFSR = SCB->DFSR; //(*((volatile unsigned long *)(0xE000ED30))) ;

	// Auxiliary Fault Status Register
	uint32_t _AFSR = SCB->AFSR; //(*((volatile unsigned long *)(0xE000ED3C))) ;

	// Read the Fault Address Registers. These may not contain valid values.
	// Check BFARVALID/MMARVALID to see if they are valid values
	// MemManage Fault Address Register
	uint32_t _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
	// Bus Fault Address Register
	uint32_t _BFAR = (*((volatile unsigned long *)(0xE000ED38)));


	// Clear any logged faults from the CFSR
	SCB->CFSR |= SCB->CFSR;
	// the instruction we will return to when we exit from the exception
	frame->return_address = (uint32_t)mainloop;
	// the function we are returning to should never branch
	// so set lr to a pattern that would fault if it did
	frame->lr = 0xdeadbeef;
	// reset the psr state and only leave the
	// "thumb instruction interworking" bit set
	frame->xpsr = (1 << 24);  ///???


}


//void my_fault_handler_c2() {
//	// Logic for dealing with the exception. Typically:
//	//  - log the fault which occurred for postmortem analysis
//	//  - If the fault is recoverable,
//	//    - clear errors and return back to Thread Mode
//	//  - else
//	//    - reboot system
//
//	volatile unsigned long var = 0;
//	void * currentSP = (void *)((unsigned long)&var + 4);
//
//	sContextStateFrame *frame = (sContextStateFrame *)currentSP;
//
//	int d;
//	d = frame->lr;				// This is the prog-pointer to last stored return value code before fault occured -> translate with map file....
//	//d = frame->return_address; 	// this is the PC which triggered the fault.
//
//
//	// Configurable Fault Status Register
//    // Consists of MMSR, BFSR and UFSR
//	uint32_t _CFSR = SCB->CFSR; // (*((volatile unsigned long *)(0xE000ED28))) ;
//
//	// Hard Fault Status Register
//	uint32_t _HFSR = SCB->HFSR; //(*((volatile unsigned long *)(0xE000ED2C))) ;
//
//	// Debug Fault Status Register
//	uint32_t _DFSR = SCB->DFSR; //(*((volatile unsigned long *)(0xE000ED30))) ;
//
//	// Auxiliary Fault Status Register
//	uint32_t _AFSR = SCB->AFSR; //(*((volatile unsigned long *)(0xE000ED3C))) ;
//
//	// Read the Fault Address Registers. These may not contain valid values.
//	// Check BFARVALID/MMARVALID to see if they are valid values
//	// MemManage Fault Address Register
//	uint32_t _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
//	// Bus Fault Address Register
//	uint32_t _BFAR = (*((volatile unsigned long *)(0xE000ED38)));
//
//
//	// Clear any logged faults from the CFSR
//	SCB->CFSR |= SCB->CFSR;
//	// the instruction we will return to when we exit from the exception
//	//frame->return_address = (uint32_t)mainloop;
//	// the function we are returning to should never branch
//	// so set lr to a pattern that would fault if it did
//	frame->lr =(uint32_t)mainloop; // 0xdeadbeef;
//	// reset the psr state and only leave the
//	// "thumb instruction interworking" bit set
//	frame->xpsr = (1 << 24);  ///???
//
//
//}
