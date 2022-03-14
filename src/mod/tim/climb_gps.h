/*
 * climb_gps.h
 *
 *  Created on: 27.02.2022
 *      Author: Robert
 */
// Connecting a "Quectel L86 GNSS" GPS Module to UART and 2 GPIOs. ( https://www.soselectronic.de/products/various/l86-m33-bob-310645 )
// Goal is to 'simulate' the 'real GPS' used later on (type: ???) in order to synchronize obc local time module.
//
#ifndef MOD_TIM_CLIMB_GPS_H_
#define MOD_TIM_CLIMB_GPS_H_

#include <chip.h>

// init data needed. Choose UARt and 2 GPIO Pins to be used
typedef struct {
	LPC_USART_T 	*pUart;			// default is 9600baud
	const PINMUX_GRP_T2   *pEnablePin;	// Output LPC -> GPS enable internal Voltage regulator -> we disable because we use SP Power
	const PINMUX_GRP_T2   *pPpsPin;		// Input  LPC <- GPS gives a 100ms pulse every second (if gps locked and synchronized only???).
} gps_initdata_t;

// API module functions
void gpsInit (void *initData);
void gpsMain (void);

void gpsSendBytes(uint8_t *data, uint8_t len);



#define MODULE_ID_GPS 				0x04

#define EID_GPS_NMEA_MSG_RAW		1
#define EID_GPS_CRCERROR			2
#define EID_GPS_MSGERROR			3
#define EID_GPS_RXBYTEBUFFERFULL	4
#define EID_GPS_RXFIELDBUFFERFULL	5
#define EID_GPS_SATELLITES_IN_SIGHT	6
//#define EID_GPS_NMEA_MSG_GGA		7
//#define EID_GPS_NMEA_MSG_GSA		8
//#define EID_GPS_NMEA_MSG_VTG		9
//#define EID_GPS_NMEA_MSG_RMC		10


#endif /* MOD_TIM_CLIMB_GPS_H_ */
