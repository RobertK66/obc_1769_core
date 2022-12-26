/*
 * ado_wirebus.h
 *
 *  Created on: 05.05.2022
 *      Author: Robert
 */

#ifndef MOD_ADO_WIREBUS_H_
#define MOD_ADO_WIREBUS_H_

#include <chip.h>

typedef enum {
	ADO_WBUS_SPI = 1,
	//ADO_WBUS_SPIDMA = 2,	// not implemented yet
	//ADO_WBUS_SSP = 3,		// not (longer) implemented in ado lib.
	ADO_WBUS_SSPDMA = 4,
	ADO_WBUS_I2C = 5,
	//ADO_WB_I2CDMA = 6	// not implemented yet
} ado_wbus_type_t;


typedef struct ado_wbus_config {
	ado_wbus_type_t	busType	:3;
	uint16_t		dummy   :13;
	void *			busBase;
}  __attribute__((packed)) ado_wbus_config_t;




void ADO_WBUS_Init(ado_wbus_config_t* pWb, uint8_t busCount);


#endif /* MOD_ADO_WIREBUS_H_ */
