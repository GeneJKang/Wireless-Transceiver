#ifndef RS_H
#define RS_H

#include <stdint.h>
#include <stdbool.h>

//Writes parity_len RS parity bytes into buf
//parity_len 0 does nothing
void rs_encode(uint8_t *buf, uint8_t msg_len, uint8_t parity_len);

//Repairs buf (len bytes, and parity) in place.
//Returns how many bytes it fixed or -1 if frame is too damaged to repair
//0 for no damaged bytes
int rs_decode(uint8_t *buf, uint8_t len, uint8_t parity_len);

#endif
