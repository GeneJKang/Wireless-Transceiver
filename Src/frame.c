#include "link_SR.h" //LINK_DATA LINK_ACK LINK_POLL
#include "frame.h"
#include "crc16.h"
#include "rs.h"
#include <string.h>

enum {
	F_TYPEMODE = 0
};
enum {
	D_SEQ = 1,
	D_REMNEXT = 2,
	D_PTR = 3,
	D_DATA = 4
};
enum {
	A_BASE = 1,
	A_BM = 2,
	A_OK = 4,
	A_BAD = 5,
	A_FIXED = 6,
	A_MODEACK = 7
};

//Mode ladder (slowest/most rebust first)
static const frame_mode_t modes[FRAME_NUM_MODES] = {
		{NRF24_DR_250K, 4},	//0 (250 kbps, fixes 2 bytes)
		{NRF24_DR_250K, 0},	//1 (250 kbps, fixes no bytes)
		{NRF24_DR_1M, 4},	//2 (1 Mbps, fixes 2 bytes)
		{NRF24_DR_1M, 0}, 	//3 (1 Mbps, fixes no bytes)
		{NRF24_DR_2M, 2},	//4 (2 Mbps, fixes 1 byte)
		{NRF24_DR_2M, 0}	//5 (2 Mbps, fixes no bytes)
};

const frame_mode_t *frame_mode(uint8_t mode){
	return &modes[mode < FRAME_NUM_MODES ? mode : 0];
}

uint8_t frame_data_len(uint8_t mode){
	return (uint8_t)(FRAME_MAX_DATA - frame_mode(mode)->rs_len);
}

//depending on the mode CRC is on byte 30, 28, or 26
//Every frame uses the same position for a given mode
static uint8_t crc_pos(uint8_t mode){
	return (uint8_t)(D_DATA + frame_data_len(mode));
}

//build frames
//fill header and data then CRC over it then RS over that
void frame_build(uint8_t out[NRF24_PAYLOAD], const link_frame_t *f, uint8_t mode){
	uint8_t cp = crc_pos(mode);

	memset (out, 0, NRF24_PAYLOAD);	//init array out with 0
	out[F_TYPEMODE] = (uint8_t)((f->type << 4) | (mode & 0x0F));


	if (f->type == LINK_DATA){ //data command
		out[D_SEQ] = f->seq;
		out[D_REMNEXT] = (uint8_t)(((f->rem & 0x0F) << 4) | (f->next_mode & 0x0F));
		//cut the frame
		uint8_t cut_code = (f->cut == 22u || f->cut == 24u || f->cut == 26u) ? (uint8_t)((f->cut - 20u) / 2u) : 0u;
		uint8_t p = (f->ptr == FRAME_PTR_NONE) ? 0x1F : (uint8_t)(f->ptr & 0x1F);
		out[D_PTR] = (uint8_t)((cut_code << 6) | p);

		memcpy (&out[D_DATA], f->data, frame_data_len(mode));	//build data
	} else if (f->type == LINK_ACK){	//ACK comand
		out[A_BASE] = f->base;
		out[A_BM] = (uint8_t)(f->bitmap >> 8);
		out[A_BM + 1] = (uint8_t)(f->bitmap & 0xFF);
		out[A_OK] = f->ok_frames;
		out[A_BAD] = f->bad_frames;
		out[A_FIXED] = f->fixed_bytes;
		out[A_MODEACK] = f->mode_ack;
	}
	//POLL is nothing besides byte 0

	uint16_t crc = crc16_ccitt(out, cp);	//Build CRC
	out[cp] = (uint8_t)(crc >>8);
	out[cp + 1] = (uint8_t)(crc & 0xFF);

	rs_encode(out, (uint8_t)(cp + 2), frame_mode(mode)->rs_len);
}


//Parse frame
//parses in reverse order of frame_build
//Copies the original frame
frame_result_t frame_parse(const uint8_t in[NRF24_PAYLOAD], uint8_t mode, link_frame_t *f){

	if (mode >= FRAME_NUM_MODES) return FRAME_BAD_MODE;

	uint8_t buf[NRF24_PAYLOAD];
	uint8_t cp = crc_pos(mode);
	memcpy(buf, in, NRF24_PAYLOAD);	//copy frame

	int fixed = rs_decode(buf, NRF24_PAYLOAD, frame_mode(mode)->rs_len); //check if frame damaged and fix
	if (fixed < 0) return FRAME_BAD_RS;	//RS cant fix

	uint16_t got = (uint16_t)((buf[cp] << 8) | buf[cp + 1]);	//check if frame is still broke after repair
	if (crc16_ccitt(buf, cp) != got) return FRAME_BAD_CRC;	//frame still broken

	uint8_t type = (uint8_t)(buf[F_TYPEMODE] >> 4);
	if (type != LINK_DATA && type != LINK_ACK && type != LINK_POLL) return FRAME_BAD_TYPE;
	if ((buf[F_TYPEMODE] & 0x0F) != mode) return FRAME_BAD_MODE;

	memset(f, 0, sizeof *f);
	f->type = type;
	f->mode = mode;
	f->fixed = (uint8_t)fixed;

	if (type == LINK_DATA){ //Parse DATA
		uint8_t dlen = frame_data_len(mode);
		f->seq = buf[D_SEQ];
		f->rem = (uint8_t)(buf[D_REMNEXT] >> 4);
		f->next_mode = (uint8_t)(buf[D_REMNEXT] & 0x0F);
		uint8_t b = buf[D_PTR];

		if (b & 0x20u) return FRAME_BAD_FIELD; //unused bit is 0

		f->ptr = ((b & 0x1Fu) == 0x1Fu) ? FRAME_PTR_NONE : (uint8_t)(b & 0x1Fu);
		f->cut = (b >> 6) ? (uint8_t)(20u + 2u * (b >> 6)) : 0u;

		if (f->ptr != FRAME_PTR_NONE && f->ptr >= dlen) return FRAME_BAD_FIELD;
		if (f->cut > dlen) return FRAME_BAD_FIELD; //more bytes than the size
		if (f->next_mode >= FRAME_NUM_MODES) return FRAME_BAD_FIELD;

		memcpy(f->data, &buf[D_DATA], dlen);
	} else if (type == LINK_ACK){ //Parse ACK
		f->base = buf[A_BASE];
		f->ok_frames = buf[A_OK];
		f->bad_frames = buf[A_BAD];
		f->fixed_bytes = buf[A_FIXED];
		f->mode_ack = buf[A_MODEACK];
		f->bitmap = (uint16_t)((buf[A_BM] << 8) | buf[A_BM + 1]);

		if (f->mode_ack != FRAME_MODE_NONE && f->mode_ack >= FRAME_NUM_MODES) return FRAME_BAD_FIELD;
	}
	return FRAME_OK;	//nothing broken detected
}
