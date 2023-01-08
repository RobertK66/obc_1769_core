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

// common defines/structures used for module composition.
typedef struct {
	void  (*init)(void* initData);
	void  (*main)(void);
	void  *initdata;
} MODULE_DEF_T;

typedef struct {
	uint8_t	 moduleIdx;
	uint32_t longestExecutionTicks;
}  __attribute__((packed)) MODULE_STATUS_T;

typedef struct {
	MODULE_STATUS_T* curExecutingPtr;
	uint32_t startedAtTicks;
	uint32_t mainLoopStartedAtTicks;
	uint32_t longestMainLoopTicks;
} MODULES_STATUS_T;

typedef struct {
	uint16_t 	moduleId:	8;	//
	uint16_t 	eventId:	6;	//
	uint16_t 	severity:	2;	//
} event_id_t;

typedef struct {
	event_id_t	id;
	uint8_t 	*data;
	uint16_t	byteCnt;
} event_t;

typedef enum {
	EVENT_INFO = 0,
	EVENT_WARNING = 1,
	EVENT_ERROR = 2,
	EVENT_FATAL = 3
} event_severity_t;

// Event Handling
__attribute__ ((weak)) void _SysEvent(event_t event);

#define SysEvent(modNr, sev, evId, d, bCnt) { \
	event_t event;						\
	event.id.severity = (sev & 0x03); \
	event.id.moduleId = (modNr & 0xFF);	\
	event.id.eventId = (evId & 0x3F);	\
	event.data = (uint8_t*)d;		\
	event.byteCnt = (uint16_t)bCnt;	\
	_SysEvent(event); \
}

#define MOD_INIT( init, main, initdata ) { init, main, (void*)initdata }
#endif /* MOD_ADO_MODULES_H_ */
