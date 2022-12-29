/*
===============================================================================
 Name        : l7_climb_app.h
 Author      : Robert
 Created on	 : 07.09.2021
===============================================================================
*/

#ifndef MOD_L7_CLIMB_APP_H_
#define MOD_L7_CLIMB_APP_H_

#include <chip.h>

void app_init (void *dummy);
void app_main (void);
//void _SysEvent_Debug(event_debug_t event);

#define MODULE_ID_CLIMBAPP		0x00
//#define EID_APP_SENSORVALUES	1
#define EID_APP_FULLTIMEINFO    1
#define EID_APP_RAWDATA			2
#define EID_APP_STRING			3
#define EID_APP_SYSTEMINIT		4
#define EID_APP_SYSTEMSTATUS	5
#define EID_APP_INIT			6
#define EID_APP_RUNTIMES		7

#include "../build.h"

#ifndef BUILD_SWVERSION
#define BUILD_SWVERSION	"Dev-"__TIME__


#endif


#endif /* MOD_L7_CLIMB_APP_H_ */

