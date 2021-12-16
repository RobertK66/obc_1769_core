/*
===============================================================================
 Name        : obc_time.h
 Author      : Robert
 Created on	 : 12.09.2021
===============================================================================
*/

#ifndef MOD_TIM_OBC_TIME_H_
#define MOD_TIM_OBC_TIME_H_

#include <Chip.h>

void tim_init (void *dummy);
void tim_main (void);

#define MODULE_ID_TIME			0x01
#define EVENT_TIM_INITIALIZED	1

#endif /* MOD_TIM_OBC_TIME_H_ */
