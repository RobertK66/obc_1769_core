#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "obc_checksums.h"


//void c_CRC8(char data, uint8_t *checksum);

/* Update CRC8 Checksum */
void c_CRC8(char data, uint8_t *checksum)
{
    uint8_t i;
    *checksum ^= data;

    for (i = 0; i < 8; ++i)
    {
        *checksum = (*checksum << 1) ^ ((*checksum & 0x80) ? 0x07 : 0x00);
    }
}

/* Compute CRC8 (binary String) */
uint8_t CRC8(uint8_t* str, size_t length)
{
    uint8_t checksum = 0;

    for (; length--; c_CRC8(*str++, &checksum))
        ;

    return checksum;
}

uint8_t odd_parity_calc(uint8_t val)
{
    uint8_t parity = (val >> 7);
    int i;
    for (i = 6; i >= 0; i--)
    {
        parity = ((parity & 0b00000001) ^ ((val >> i) & 0b00000001));
    }

    /* ODD parity: True if sum of all ones in data byte is even */
    return 0x01 & (parity ^ 0b00000001);
}

uint8_t gps_checksum_calc(char *str)
{
    uint8_t checksum = 0;
    uint32_t i;

    char * ptr = memchr(str, '*', 80);

    if (ptr == NULL)
    {
        /* No valid GPS string */
        return 0;
    }

    uint8_t len = ptr - str;

    for (i = 1; i < len; i++) /* 80 Bytes is the maximum length of a GPS string including \r\n; skip $ */
    {
        if ((str[i] == '*'))
        {
            /* Checksum is calculated of bytes between $ and *. */
            break;
        }

        checksum = checksum ^ str[i];
    }

    return checksum;
}

uint32_t crc32(uint8_t *data, uint32_t len)
{
    unsigned int i;
    int j;
    unsigned int byte, crc, mask;

    i = 0;
    crc = 0xFFFFFFFF;

    while (i < len)
    {
        byte = data[i];            // Get next byte.
        crc = crc ^ byte;
        for (j = 7; j >= 0; j--)
        {    // Do eight times.
            mask = -(crc & 1);
            crc = (crc >> 1) ^ (0xEDB88320 & mask);
        }
        i = i + 1;
    }
    return ~crc;
}

uint16_t CRC16(const uint8_t* data_p, uint16_t length){
    uint8_t x;
    uint16_t crc = 0xFFFF;

    while (length--){
        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);
    }
    return crc;
}

uint16_t Fletcher16(uint8_t* data, int len){
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    int index;
    for(index=0;index<len;++index){
        sum1 = (sum1 + data[index]) % 255;
	sum2 = (sum2 + sum1) % 255;
    }
    return (sum2 << 8) | sum1;
}


