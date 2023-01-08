/*
 * ado_crc.c
 *
 *  Created on: 08.06.2020
 *      Author: Robert copied from Pegasus code
 */

#include "ado_crc.h"

uint16_t CRC16_0x1021(const uint8_t* data_p, uint16_t length, uint16_t start){
    uint8_t x;
    uint16_t crc = start;

    while (length--){
        x = crc >> 8 ^ (*data_p++);
        x ^= x>>4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);
    }
    return crc;
}

//uint16_t Fletcher16(uint8_t* data, int len){
//    uint16_t sum1 = 0;
//    uint16_t sum2 = 0;
//    int index;
//    for(index=0;index<len;++index){
//        sum1 = (sum1 + data[index]) % 255;
//	sum2 = (sum2 + sum1) % 255;
//    }
//    return (sum2 << 8) | sum1;
//}


//uint8_t CRC8_0x31(const uint8_t* data_p, uint16_t len, uint8_t start) {
//	uint8_t crc = start;
//	size_t i, j;
//	for (i = 0; i < len; i++) {
//		crc ^= data_p[i];
//		for (j = 0; j < 8; j++) {
//			if ((crc & 0x80) != 0)
//				crc = (uint8_t)((crc << 1) ^ 0x31);
//			else
//				crc <<= 1;
//		}
//	}
//	return crc;
//}

uint8_t CRC8_poly(const uint8_t* data_p, uint16_t len, uint8_t poly, uint8_t start) {
	uint8_t crc = start;
	size_t i, j;
	for (i = 0; i < len; i++) {
		crc ^= data_p[i];
		for (j = 0; j < 8; j++) {
			if ((crc & 0x80) != 0)
				crc = (uint8_t)((crc << 1) ^ poly);
			else
				crc <<= 1;
		}
	}
	return crc;
}

