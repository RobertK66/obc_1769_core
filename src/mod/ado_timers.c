/*
 * ado_timers.c
 *
 *  Created on: 08.08.2022
 *      Author: Robert
 */
// Make the 4 TIMERS of the LPC available at runtime init. Remember IRQ Callbacks in static variables.
// TODO: move this to ado lib later!
// TODO: allow for all matchpoints to be used.



#include "ado_timers.h"

// Unfortunately this function Chip_Timer_GetClockIndex() is static in timer_17xx_40xx.c -> TODO make it public in ADO Library
static inline CHIP_SYSCTL_CLOCK_T MyChip_Timer_GetClockIndex(LPC_TIMER_T* ptr)   { CHIP_SYSCTL_CLOCK_T clkTMR;
	if (ptr == LPC_TIMER1) {
		clkTMR = SYSCTL_CLOCK_TIMER1;
	}
	else if (ptr == LPC_TIMER2) {
		clkTMR = SYSCTL_CLOCK_TIMER2;
	}
	else if (ptr == LPC_TIMER3) {
		clkTMR = SYSCTL_CLOCK_TIMER3;
	}
	else {
		clkTMR = SYSCTL_CLOCK_TIMER0;
	}
	return clkTMR;
}

static void (*irqHandler0)(void) = 0;
static void (*irqHandler1)(void) = 0;
static void (*irqHandler2)(void) = 0;
static void (*irqHandler3)(void) = 0;

void InitTimer(LPC_TIMER_T* pTimer, uint16_t tickMs, void (*iIrqHandler)(void)) {

	// Setup the timer counter to count 1ms ticks.
	CHIP_SYSCTL_CLOCK_T timerIdx = MyChip_Timer_GetClockIndex(pTimer);		// Convert pointer to SYYCTRL enum.
	Chip_TIMER_TIMER_SetCountClockSrc(pTimer, TIMER_CAPSRC_RISING_PCLK, 0); // Use internal clock for this timer (timer->CTCR)
	Chip_Clock_EnablePeriphClock(timerIdx);									// (PCONP) power enable the timer block
	Chip_Clock_SetPCLKDiv(timerIdx, SYSCTL_CLKDIV_8);  						// Clk/8 -> 96/4 = 12Mhz -> Tick = 83,3 ns
	Chip_TIMER_PrescaleSet(pTimer, 11999); 									// (timer->PR) Prescaler = 11999+1 -> Increment TC at every 12000 Ticks -> 1ms (when clk is 96 Mhz and Clck divide / 8

	// Use the Match channel 0
	Chip_TIMER_SetMatch(pTimer, 0 , tickMs);								// Match counter will trigger IRQ every every tickMs
	Chip_TIMER_ResetOnMatchEnable(pTimer, 0);								// Configure channel to reset the Timer Counter when match is reached.
	Chip_TIMER_MatchEnableInt(pTimer, 0);									// Enable the Match channel to trigger IRQ
	Chip_TIMER_Enable(pTimer);												// (timer->TCR) enable timer to start ticking

	// Remember the IRQ Handler for this timer and enable IRQ
	if (iIrqHandler != 0) {
		if (pTimer == LPC_TIMER0) {
			irqHandler0 = iIrqHandler;
			NVIC_EnableIRQ(TIMER0_IRQn);
		} else if (pTimer == LPC_TIMER1) {
			irqHandler1 = iIrqHandler;
			NVIC_EnableIRQ(TIMER1_IRQn);
		} else if (pTimer == LPC_TIMER2) {
			irqHandler2 = iIrqHandler;
			NVIC_EnableIRQ(TIMER2_IRQn);
		} else if (pTimer == LPC_TIMER3) {
			irqHandler3 = iIrqHandler;
			NVIC_EnableIRQ(TIMER3_IRQn);
		}
	}
}

// IRQ Handler call the registered Handlers if available. This functions 'automagically' overwrite the weak definition of cr_startup_lpc175x_6x.c ;-)
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
