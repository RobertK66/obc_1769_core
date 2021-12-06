/*
===============================================================================
 Name        : temp-Base.h
 Author      : Robert
 Created on	 : 06.09.2021
===============================================================================
*/
// This file should not be needed in the end. Temporary here we can have common structures and defines which are not ready to go
// to the ado lib. Either they should become useless here or the abstraction is good enough to go to some ado header files/modules....
#ifndef TEMP_BASE_H_
#define TEMP_BASE_H_

#include <chip.h>
#include <stdarg.h>
#include <ado_time.h>

// common defines/structures used for module composition
typedef struct {
	//void  *initData;
	void  (*init)(void* initData);
	void  (*main)(void);
} MODULE_DEF_T;


// Event Handling
__attribute__ ((weak)) void _SysEvent(uint8_t *data, uint16_t byteCnt);
#define SysEvent(ptr, len) { \
	_SysEvent((uint8_t*)ptr, len); \
}



// Event numbers are 16 bit
typedef uint16_t ado_event_nr;

// This is only the 'header of each concrete event. if data is available it follows directly as struct after this struct in memory.
typedef struct ado_event_s  ado_event_t;


typedef struct ado_event_s {
    void        (*initializer)(ado_event_t *this, va_list *args);      // Poor mans OO. The Callback which takes care of all additional 'Constructor' paramaters.
    ado_timestamp timestamp;    // This always represents ms after reset! If the event gets persisted additional info is needed (e.g. ResetCounter and/or UTCOffset)!
                                // If resetCounter is (persisted) accurate, then all events and their exact sequence are uniquely identifiable.
                                // If there exists a valid UTCOffset to a specific Reset Counter all events of this 'Reset-epoch' can be assigned to an exact UTC-timestamp
                                // The valid time between 2 resets without overrun occurring in eventlogger time stamps is approx. 49,7 days. So your system should have some
                                // means to reset at least all 49 days, or you have to manually increase the resetCounter and set a new UTCOffset.
    ado_event_nr  eventNr;
    uint16_t      eventDataSize;
}  __attribute__((packed))  ado_event_t;


#endif /* TEMP_BASE_H_ */
