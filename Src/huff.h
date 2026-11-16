#ifndef HUFF_H
#define HUFF_H

#include <stdint.h>
#include <stdbool.h>
#include "link_SR.h"

//Huffman compression for messages and text commands
//Every message has a header byte
	//0x00 means no huffman
	//0x80 | table the rest is huffman bits made with that table (0-15)
	//0x40 | table
//table 0 is a default table and 1 to 15 is a dynamic learned table


#define HUFF_NUM_SYMBOLS 96 //all ASCII characters and escape
#define HUFF_MAX_BITS 15 //longest code a table may have
#define HUFF_TABLE_BYTES (HUFF_NUM_SYMBOLS / 2u) //code length
#define HUFF_HDR_RAW 0x00
#define HUFF_HDR_TABLE 0x40
#define HUFF_HDR_COMPRESSED 0x80

//Longest text huff_send() takes (header and text)
#define HUFF_MAX_TEXT (LINK_MAX_DATA - 1u)

//Learning settings (sender side)
#define HUFF_RETUNE_EVERY 256u	//messages between checks for a better table
#define HUFF_RETUNE_MIN_CHARS 512u //dont rank a table on less characters than this
#define HUFF_RETUNE_GAIN_PCT 97u //Switch only if new table needs <= HUFF_RETUNE_GAIN_PCT of the bits
#define HUFF_ANNOUNCE_EVERY 512u //resend the current table this often so a receiver that restarted gets it back

//Statistics
typedef struct {
	uint8_t tx_table; //table the sender is currently using
	uint8_t rx_table; //learned table that the receiver has (0 = none)
	uint32_t retunes; //times the sender switched tables
	uint32_t retune_checks; //times it looked
	uint32_t tables_sent; //table messages sent (new tables and repeats)
	uint32_t tables_received; //valid table messages received
	uint32_t bad_msgs; //messages received that couldnt be decoded
	uint32_t saves; //tables written to flash
	bool loaded; // a saved table was loaded at startup
}huff_stats_t;


//Builds table 0 and uses a valid saved table if needed
void huff_init(void);

//Turns learning on or off (on default)
void huff_set_learning(bool on);

//Compress the text with the sender's current table and return how many bytes it wrote
uint8_t huff_encode(const uint8_t *text, uint8_t len, uint8_t *out);

//Turn one compressed or raw message back into text using receiver's tables (-1 if invalid)
int huff_decode(const uint8_t *msg, uint8_t len, uint8_t *text, uint8_t text_max);

//Count the text's characters for learning and sends a table message first if the receiver needs one
link_result_t huff_send(const char *text, uint8_t len);

//hands back decoded text
//Like link_poll_recv()
bool huff_poll_recv(char *text, uint8_t *len);

//Writes the sender's current table to flash
bool huff_save(void);

huff_stats_t huff_get_stats(void);

#endif














