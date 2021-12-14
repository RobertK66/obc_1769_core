/*
===============================================================================
 Name        : ado_modules.h
 Author      : Robert
 Created on	 : 13.12.2021
===============================================================================
*/

#ifndef MOD_ADO_MODULES_H_
#define MOD_ADO_MODULES_H_

#include <chip.h>

// common defines/structures used for module composition
typedef struct {
	//void  *initData;
	void  (*init)(void* initData);
	void  (*main)(void);
	void  *initdata;
} MODULE_DEF_T;

typedef struct {
	uint16_t 	modId:		6;	// Module Number
	uint16_t 	severity:	2;	//
	uint16_t 	eventId:	8;	/* Pin number */
} event_id_t;


// Event Handling
__attribute__ ((weak)) void _SysEvent(event_id_t eventId, uint8_t *data, uint16_t byteCnt);
#define SysEvent(evnt, ptr, len) { \
	_SysEvent(evnt, (uint8_t*)ptr, len); \
}


#endif /* MOD_ADO_MODULES_H_ */
