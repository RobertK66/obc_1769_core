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
#define BA_BOARD	BA_CLIMBOBC					// BA_CLIMBOBC //BA_OM13085 // BA_OM13085_EM2T

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


// Assign MRAM CS Port and Pinnumbers for different Boards used
#if BA_BOARD == BA_OM13085_EM2T
	#define MRAM_CS01_PORT	0
	#define MRAM_CS01_PIN	P0_PIN_SSP0_MRAM_CS1
	#define MRAM_CS02_PORT	1
	#define MRAM_CS02_PIN	P1_PIN_SSP0_MRAM_CS2
	#define MRAM_CS03_PORT	1
	#define MRAM_CS03_PIN	P1_PIN_SSP0_MRAM_CS3
	#define MRAM_CS11_PORT	0
	#define MRAM_CS11_PIN	P0_PIN_SSP1_MRAM_CS1
	#define MRAM_CS12_PORT	0
	#define MRAM_CS12_PIN	P0_PIN_SSP1_MRAM_CS2
	#define MRAM_CS13_PORT	0
	#define MRAM_CS13_PIN	P0_PIN_SSP1_MRAM_CS3
#elif BA_BOARD == BA_CLIMBOBC
	#define MRAM_CS01_PORT	0
	#define MRAM_CS01_PIN	P0_PIN_SSP0_MRAM_CS1
	#define MRAM_CS02_PORT	2
	#define MRAM_CS02_PIN	P2_PIN_SSP0_MRAM_CS2
	#define MRAM_CS03_PORT	2
	#define MRAM_CS03_PIN	P2_PIN_SSP0_MRAM_CS3
	#define MRAM_CS11_PORT	2
	#define MRAM_CS11_PIN	P2_PIN_SSP1_MRAM_CS1
	#define MRAM_CS12_PORT	0
	#define MRAM_CS12_PIN	P0_PIN_SSP1_MRAM_CS2
	#define MRAM_CS13_PORT	1
	#define MRAM_CS13_PIN	P1_PIN_SSP1_MRAM_CS3
#endif





#endif /* CLIMBOBC_H_ */
