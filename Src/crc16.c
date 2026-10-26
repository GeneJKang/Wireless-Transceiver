#include "crc16.h"

uint16_t crc16_ccitt(const uint8_t *data, size_t len){

	uint16_crc = 0xFFFF;	//Initialization crc register to full of 1s

	for(size_t i = 0; i < len; i++){

		uint16_t current_byte = (uint16_t)data[i] << 8;
		crc = crc ^ current_byte;

		for(int b = 0; b < 8; b++){

			if ((crc & 0x8000) != 0){	//Check highest bit
				crc = (crc << 1) ^ 0x1021;	//if top bit = 1, XOR crc
			} else{
				crc = crc << 1;	//if top bit = 0, shift left
			}

		}

	}
	return crc;
}
