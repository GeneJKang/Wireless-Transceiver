#ifndef CRC16_H
#define CRC16_H
#include <stdint.h>
#include <stddef.h>

//CRC - CCITT - 16 - FALSE: Polynomial: x^16 + x^15 + x^5 + 1 (0x1021)
//init: 0xFFFF
//Check value: crc16("123456789") == 0x29B1
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

#endif
