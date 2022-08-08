/*
 * ado_timers.c
 *
 *  Created on: 08.08.2022
 *      Author: Robert
 */
// Make the 4 TIMERS of the LPC available at runtime init. Remember IRQ Callbacks in static variables.
// TODO: move this to ado lib later!

#include "ado_timers.h"

static void (*irqHandler0)(void) = 0;
static void (*irqHandler1)(void) = 0;
static void (*irqHandler2)(void) = 0;
static void (*irqHandler3)(void) = 0;

void InitTimer(LPC_TIMER_T* pTimer, uint16_t tickMs, void (*iIrqHandler)(void)) {

	// Setup the timer counter to count 1ms ticks
	Chip_TIMER_TIMER_SetCountClockSrc(pTimer, TIMER_CAPSRC_RISING_PCLK, 0); // Use internal clock for this timer
	Chip_TIMER_Init(pTimer);                // Enables Peripheral Clock (PCONOP)
	Chip_TIMER_PrescaleSet(pTimer, 11999); 	// Prescaler = 11999+1 -> Increment TC at every 12000 Ticks -> 1ms (when clk is 96 Mhz and Clck divide / 8
	Chip_TIMER_Reset(pTimer);

	if (pTimer == LPC_TIMER0) {
		Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_TIMER0, SYSCTL_CLKDIV_8);  // Clk/8 -> 96/4 = 12Mhz -> Tick = 83,3 ns
		irqHandler0 = iIrqHandler;
		NVIC_EnableIRQ(TIMER0_IRQn);
	} else if (pTimer == LPC_TIMER1) {
		Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_TIMER1, SYSCTL_CLKDIV_8);  // Clk/8 -> 96/4 = 12Mhz -> Tick = 83,3 ns
		irqHandler1 = iIrqHandler;
		NVIC_EnableIRQ(TIMER1_IRQn);
	} else if (pTimer == LPC_TIMER2) {
		Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_TIMER2, SYSCTL_CLKDIV_8);  // Clk/8 -> 96/4 = 12Mhz -> Tick = 83,3 ns
		irqHandler2 = iIrqHandler;
		NVIC_EnableIRQ(TIMER2_IRQn);
	} else if (pTimer == LPC_TIMER3) {
		Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_TIMER3, SYSCTL_CLKDIV_8);  // Clk/8 -> 96/4 = 12Mhz -> Tick = 83,3 ns
		irqHandler3 = iIrqHandler;
		NVIC_EnableIRQ(TIMER3_IRQn);
	}
	Chip_TIMER_Enable(pTimer);

	// Set the tickMs value to trigger a IRQ at wished interval.
	Chip_TIMER_SetMatch(pTimer, 0 , tickMs);
	Chip_TIMER_MatchEnableInt(pTimer, 0);

}

void TIMER0_IRQHandler(void) {
	if (irqHandler0 != 0) {
		irqHandler0();
	}
}

void TIMER1_IRQHandler(void) {
	if (irqHandler1 != 0) {
		irqHandler1();
	}
}

void TIMER2_IRQHandler(void) {
	if (irqHandler2 != 0) {
		irqHandler2();
	}
}

void TIMER3_IRQHandler(void) {
	if (irqHandler3 != 0) {
		irqHandler3();
	}
}
