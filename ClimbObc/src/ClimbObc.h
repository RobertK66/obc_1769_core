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
// #define BA_OM13085		1  Obsolete, no use any more because no mrams and sdCard available.....
#define BA_OM13085_EM2T	2
#define BA_CLIMBOBC		3

// Switch the board Config here.

#ifndef BA_BOARD
#define BA_BOARD	BA_CLIMBOBC					// BA_CLIMBOBC //BA_OM13085 // BA_OM13085_EM2T
#endif

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

typedef struct {
	uint8_t		 entryCount;
	const PINMUX_GRP_T2* pinmux;
} pinmux_array_t;

static const pinmux_array_t ObcPins = {
	(sizeof(pinmuxing2)/sizeof(PINMUX_GRP_T2)), pinmuxing2
};

#define PINNR_FROM_IDX(idx) (ObcPins.pinmux[idx].pinnum)
#define PORT_FROM_IDX(idx) (ObcPins.pinmux[idx].pingrp)
#define PTR_FROM_IDX(idx) (&ObcPins.pinmux[idx])

#endif /* CLIMBOBC_H_ */
