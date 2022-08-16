/*
 * ado_timers.h
 *
 *  Created on: 08.08.2022
 *      Author: Robert
 */

#ifndef MOD_ADO_TIMERS_H_
#define MOD_ADO_TIMERS_H_

#include <chip.h>

void InitTimer(LPC_TIMER_T* pTimer, uint16_t tickMs, void (*iIrqHandler)(void));

#endif /* MOD_ADO_TIMERS_H_ */
