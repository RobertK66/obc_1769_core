/*
===============================================================================
 Name        : ClimbObc.h
 Author      : Robert
 Created on	 : 05.09.2021
===============================================================================
*/

#ifndef CLIMBOBC_H_
#define CLIMBOBC_H_

// Available board abstractions
#define BA_OM13085	1
#define BA_CLIMBOBC	2

#define BA_BOARD	BA_CLIMBOBC	// BA_CLIMBOBC //BA_OM13085

#if BA_BOARD == BA_OM13085
	#define BA_BOARD_REL	"revD1"
	#include "pinning_om13085.h"
#elif BA_BOARD == BA_CLIMBOBC
	#define BA_BOARD_REL	"EM2"
	#include "pinning_climbobc.h"
#endif

#endif /* CLIMBOBC_H_ */
