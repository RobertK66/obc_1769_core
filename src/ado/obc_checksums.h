/*
===============================================================================
 Name        : obc_checksums.h
 Author      : Robert
 Created on	 : 12.09.2021
===============================================================================
*/

#ifndef OBC_CHECKSUMS_H_
#define OBC_CHECKSUMS_H_

#include <Chip.h>

uint8_t CRC8(uint8_t* str, size_t length);
void c_CRC8(char data, uint8_t *checksum);

uint32_t crc32(uint8_t *data, uint32_t len);


#endif /* OBC_CHECKSUMS_H_ */
