/*
===============================================================================
 Name        : ClimbObc.h
 Author      : Robert
 Created on	 : 05.09.2021
===============================================================================
*/

#ifndef CLIMBOBC_H_
#define CLIMBOBC_H_
#include "temp-Base.h"					// TODO: get rid of this, somewhen .....


// Available board abstractions
#define BA_OM13085		1
#define BA_OM13085_EM2T	2
#define BA_CLIMBOBC		3

// Switch the board Config here.
#define BA_BOARD	BA_OM13085_EM2T					// BA_CLIMBOBC //BA_OM13085 // BA_OM13085_EM2T

#if BA_BOARD == BA_OM13085
	#define BA_BOARD_REL	"revD1"
	#include "pinning_om13085.h"
#elif BA_BOARD == BA_OM13085_EM2T
	#define BA_BOARD_REL	"revD1+A_V1.0"
	#include "pinning_om13085.h"
#elif BA_BOARD == BA_CLIMBOBC
	#define BA_BOARD_REL	"EM2"
	#include "pinning_climbobc.h"
#endif

#endif /* CLIMBOBC_H_ */
