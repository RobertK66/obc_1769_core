/*
 * ado_crc.h
 *
 *  Created on: 08.06.2020
 *      Author: Robert
 */

#ifndef ADO_CRC_H_
#define ADO_CRC_H_

#include <chip.h>

// Optimized CRC16 for Poly=x16+x12+x5+1, refin=false refout=false
uint16_t CRC16_0x1021(const uint8_t* data_p, uint16_t length, uint16_t start);

// Following named CRC16 and aliases can be constructed with above routine.
// See: http://reveng.sourceforge.net/crc-catalogue/16.htm#crc.cat-bits.16
// This are versions and aliases with different start and xorout values.

// width=16 poly=0x1021 init=0xffff refin=false refout=false xorout=0x0000 check=0x29b1 residue=0x0000 name="CRC-16/IBM-3740"
#define CRC16_IBM_3740(ptr, len)	CRC16_0x1021(ptr,len,0xFFFF)
#define CRC16_CCITT_FALSE(ptr, len) CRC16_0x1021(ptr,len,0xFFFF)
#define CRC16_AUTOSAR(ptr, len)		CRC16_0x1021(ptr,len,0xFFFF)

// width=16 poly=0x1021 init=0x1d0f refin=false refout=false xorout=0x0000 check=0xe5cc residue=0x0000 name="CRC-16/SPI-FUJITSU"
#define CRC16_SPI_FUJITSU(ptr,len)	CRC16_0x1021(ptr,len,0x1D0F)
#define CRC16_AUG_CCITT(ptr,len)	CRC16_0x1021(ptr,len,0x1D0F)

// width=16 poly=0x1021 init=0x0000 refin=false refout=false xorout=0x0000 check=0x31c3 residue=0x0000 name="CRC-16/XMODEM"
#define CRC16_XMODEM(ptr, len)		CRC16_0x1021(ptr,len,0x0000)
#define CRC16_ZMODEM(ptr, len)		CRC16_0x1021(ptr,len,0x0000)
#define CRC16_ACORN(ptr, len)		CRC16_0x1021(ptr,len,0x0000)
#define CRC16_LTE(ptr, len)			CRC16_0x1021(ptr,len,0x0000)
#define CRC16_V_41_MSB(ptr, len)	CRC16_0x1021(ptr,len,0x0000)

// width=16 poly=0x1021 init=0xffff refin=false refout=false xorout=0xffff check=0xd64e residue=0x1d0f name="CRC-16/GENIBUS"
#define CRC16_GENIBUS(ptr, len) 	(0xFFFF ^ CRC16_0x1021(ptr,len,0xFFFF))
#define CRC16_DARC(ptr, len) 		(0xFFFF ^ CRC16_0x1021(ptr,len,0xFFFF))
#define CRC16_EPC(ptr, len) 		(0xFFFF ^ CRC16_0x1021(ptr,len,0xFFFF))
#define CRC16_EPC_C1G2(ptr, len) 	(0xFFFF ^ CRC16_0x1021(ptr,len,0xFFFF))
#define CRC16_I_CODE(ptr, len)	 	(0xFFFF ^ CRC16_0x1021(ptr,len,0xFFFF))

// width=16 poly=0x1021 init=0x0000 refin=false refout=false xorout=0xffff check=0xce3c residue=0x1d0f name="CRC-16/GSM"
#define CRC16_GSM(ptr, len)			(0xFFFF ^ CRC16_0x1021(ptr,len,0x0000))

// non optimized (i,j loop) crc8 for polynom: x8+x5+x4+1 / refin=false refout=false
uint8_t CRC8_0x31(const uint8_t* data_p, uint16_t length, uint8_t start);

// CRC8 polinom x8+x5+x4+1 / Start 0xff
#define CRC8(ptr, len) CRC8_0x31(ptr, len, 0xFF);

#endif /* ADO_CRC_H_ */
