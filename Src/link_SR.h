// Selective Repeat
// Data frame
/*Byte Field Note
 * 0	type	DATA
 * 1	sequence	0 to 255 window
 * 2	remaining	frames left in this burst (0 = last)
 * 3	length		valid data bytes (0 to 26)
 * 4-29	data		26 bytes data
 * 30-31	CRC		CRC over bytes 0 to 29
 */

//ACK frame
/*Byte Field Note
 * 0	type	ACK
 * 1	base	next (in-order) frame sequence the receiver still needs
 * 2-3 	bitmap	bit i set = frame base + i received (16 bits)
 * 4-29	unused	zero
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

#ifndef LINK_SAW_H
#define LINK_SAW_H

#include <stdint.h>
#include <stdbool.h>

#define LINK_MAX_DATA 26u
#define LINK_DATA	0x01u
#define LINK_ACK	0x02u
#define LINK_POLL	0x03u

#ifndef SR_WINDOW
#define SR_WINDOW 16u	//window size
#endif


//Turntables
#define LINK_ACK_TIMEOUT 4u	//ms to wait for ACK after each send
#define LINK_TURNAROUND_US 150u	//wait for this long before any transmission (for TX->RX switch)
								//Usually its 130 us + SPI overhead
#define LINK_RX_SILENCE_MS 8u
#define LINK_MAX_POLLS 3u
#define LINK_MAX_ROUNDS 50u


//slots are indexed by seq % SR_WINDOW. Meaning for mapping to stay consistent
//when seq wraps 255 to 0, SR_Window must divide by 256 i.e. must be powers of 2
_Static_assert(SR_WINDOW >= 1 && SR_WINDOW <= 16 && (SR_WINDOW & (SR_WINDOW -1)) == 0,
		"SR_WINDOW must be a power of two between 1 and 16");



typedef enum {
	LINK_OK = 0,
	LINK_FAIL
} link_result_t;

typedef enum{
	SR_RX_NEW,	//stored in the window
	SR_RX_DUP,	//already have it or already delivered (lost ACK)
	SR_RX_OUTSIDE	//Not in current or previous window (drop)
} sr_rx_result_t;

typedef struct {
	uint32_t tx_ok;	//frame acked
	uint32_t tx_fail;	//frame abandoned after certain number of txs
	uint32_t ack_timeouts;
	uint32_t crc_fail;	//frame dropped for bad CRC from either side
	uint32_t rx_ok;	//new frames delivered to the output
	uint32_t rx_dup;	//duplicates re-acked and dropped
	uint32_t radio_err;	//nrf24_send() timeouts

	uint32_t frames_sent;	//DATA frames transmitted including resends
	uint32_t bursts;
	uint32_t polls_sent;
	uint32_t acks_sent;

} link_stats_t;

typedef struct {
	uint8_t base;		//oldest sequence not yet ACKed
	uint8_t next_seq;	//sequence the next queed frame gets
	bool acked[SR_WINDOW];	//ACK window
	uint8_t len[SR_WINDOW];
	uint8_t data[SR_WINDOW][LINK_MAX_DATA];	//kept until ACKED for resends
}sr_tx_t;

typedef struct {
	uint8_t base;	//next in-order seq still needed
	bool have[SR_WINDOW];	//frames still buffered and undelivered
	uint8_t len[SR_WINDOW];
	uint8_t data[SR_WINDOW][LINK_MAX_DATA];
}sr_rx_t;





void link_init(void);
link_result_t link_send(const uint8_t *data, uint8_t len);
bool link_poll_recv(uint8_t *data, uint8_t *len);
const link_stats_t *link_stats(void);



void sr_tx_init(sr_tx_t *tx);

//frames sent but not yet released by ACK (next_seq - base)
uint8_t sr_tx_in_flight(const sr_tx_t *tx);

//Copy a message into the window and assign it to a seq
//Return false (and copies nothing) if window is full or len > 26
bool sr_tx_queue(sr_tx_t *tx, const uint8_t *data, uint8_t len, uint8_t *seq_out);

//Fill out_seqs with every in-flight seq the receiver hasn't confirmed, by oldest first
uint8_t sr_tx_pending(const sr_tx_t *tx, uint8_t out_seqs[SR_WINDOW]);

//Look up the stored payload for an in-flight seq (to build its DATA frame)
const uint8_t *sr_tx_payload(const sr_tx_t *tx, uint8_t seq, uint8_t *len_out);

//Apply an ACK. Marks frames as confirmed and then slides the base past every
//consecutive confirmed frame, freeing up their slots
//Returns false if the ACK is impossible for this window (stale/bogus)
bool sr_tx_on_ack(sr_tx_t *tx, uint8_t ack_base, uint16_t bitmap);

void sr_rx_init(sr_rx_t *rx);
sr_rx_result_t sr_rx_accept(sr_rx_t *rx, uint8_t seq, const uint8_t *data, uint8_t len);

//Pop next in-order message if base has arrived. Call in a loop until it returns false
bool sr_rx_deliver(sr_rx_t *rx, uint8_t *data_out, uint8_t *len_out);

//ACK fields base and bit i = frame
//base + i is buffered
uint16_t sr_rx_bitmap(const sr_rx_t *rx);

link_result_t link_flush(void);


#endif










