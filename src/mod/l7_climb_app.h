/*
===============================================================================
 Name        : l7_climb_app.h
 Author      : Robert
 Created on	 : 07.09.2021
===============================================================================
*/

#ifndef MOD_L7_CLIMB_APP_H_
#define MOD_L7_CLIMB_APP_H_

void app_init (void *dummy);
void app_main (void);

#include "../temp-Base.h"

// no init data needed for this module
static const MODULE_DEF_T appModuleDesc = {
		0,
		app_init,
		app_main
};

#define AppInitModule() {	\
	appModuleDesc.init((void*)0); 	\
}

#define AppMain() { 		\
	appModuleDesc.main(); 	\
}

#endif /* MOD_L7_CLIMB_APP_H_ */
