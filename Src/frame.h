// Selective Repeat
// Data frame
/*Byte Field Note
 * high nibble = type (DATA 1, ACK 2, POLL 3), low nibble = mode
 * 0	type | mode
 * 1	sequence	0 to 255 window
 * 2	remaining (high nibble) | next)mode (low nibble)	frames left in this burst (0 = last)
 * 3	cut | ptr		how many data bytes in frame (0 = short frame that ends in padding, 1 = 22, 2 = 24, 3 = 26) bit 5 = 0 bits 4 - 0 = ptt (offset of first message), 31 = none
 * 4-29	data		26 bytes data
 * 30-31	CRC		CRC over bytes 0 to 29
 * 0, 2, or 4 Reed-Solomon parity bytes over everything
 */

//ACK frame
/*Byte Field Note
 * 0	type	ACK
 * 1	base	next (in-order) frame sequence the receiver still needs
 * 2-3 	bitmap	bit i set = frame base + i received (16 bits)
 * 4-7	mode_ack this is used to hold the quality fields for mode_ack
 * 8-29	unused	zero
 * 30-31 CRC	CRC bytes over 0 to 29
 */

//POLL frame
/*Byte Field Notes
 * 0	POLL
 * 1-29	unused
 * 30-31 CRC	CRC bytes over 0 to 29
 */
//when ACK is lost a POLL frame is sent
//If received then receiver resends current ACK
//If nothing is answered then sender assumes link is dead

#ifndef FRAME_H
#define FRAME_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf24.h"	//NRF24_PAYLOAD


#define FRAME_MAX_DATA 26u //data area with no RS
#define FRAME_NUM_MODES 6u //mode 0-5 (table in frame.c)
#define FRAME_PTR_NONE 0xFFu //ptr value for no message in frame
#define FRAME_MODE_NONE 0xFFu //mode_ack value (no mode change accepted)

//Data rate and Reed-Solomon
typedef struct {
	uint8_t rf_dr;	//RF_SETUP data-rate bits (0x20 = 250 kbps, 0x00 = 1Mbps, 0x08 = 2 Mbps)
	uint8_t rs_len; //Reed-Solomon parity bytes (0, 2, or 4)
}frame_mode_t;

//Frame after parsing (or before building).
typedef struct {
	uint8_t type;	//LINK_DATA, LINK_ACK, LINK_POLL
	uint8_t mode;	//mode of current config
	uint8_t seq;	//DATA sequence
	uint8_t rem;	//DATA frames left in burst
	uint8_t next_mode; //DATA mode the sender proposes next
	uint8_t ptr;	//DATA where the first new message starts, or FRAME_PRT_NONE
	uint8_t cut; //DATA data byte in the area (22, 24, 26) or 0 for a short frame
	uint8_t data[FRAME_MAX_DATA];
	uint8_t base;	//ACK
	uint16_t bitmap;	//ACK
	uint8_t ok_frames; //ACK (DATA frames that arrived intact in the last burst)
	uint8_t bad_frames; //ACK (frames that arrived but failed CRC/RS)
	uint8_t fixed_bytes; //ACK (Bytes RS repaired in last burst)
	uint8_t mode_ack; //ACK (Mode the receiver agreed to switch to or FRAME_MODE_NONE)
	uint8_t fixed;	//Bytes Reed-Solomon repaired
} link_frame_t;

typedef enum {
	FRAME_OK = 0,
	FRAME_BAD_RS,	//too damaged for Reed-Solomon
	FRAME_BAD_CRC,
	FRAME_BAD_TYPE,
	FRAME_BAD_MODE,	//mode nibble doesn't match the agreed mode
	FRAME_BAD_FIELD	//passed CRC but a field is out of range
} frame_result_t;

//Returns the table row for a mode
//An out-of-range mode gets mode 0
const frame_mode_t *frame_mode(uint8_t mode);

void frame_build(uint8_t out[NRF24_PAYLOAD], const link_frame_t *f, uint8_t mode);

//Returns how many data bytes a DATA frame carries in this mode (26, 24, or 22)
uint8_t frame_data_len(uint8_t mode);

//Repairs, checks, and unpacks a received frame if broken.
//mode is the mode both sides agreed on.
//f is only valid when this returns FRAME_OK
frame_result_t frame_parse(const uint8_t in[NRF24_PAYLOAD], uint8_t mode, link_frame_t *f);

#endif










