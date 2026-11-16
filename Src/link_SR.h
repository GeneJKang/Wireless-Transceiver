#ifndef LINK_SAW_H
#define LINK_SAW_H

#include <stdint.h>
#include <stdbool.h>

//Longest message link_sen() accepts (messages are packed)
#define LINK_MAX_DATA 200u

//Bytes one window slow keeps for resends
#define LINK_SLOT_BYTES (1u + 26u)

//Messages waiting to be divided into frames
#define LINK_TX_BUF (SR_WINDOW * 26u)


#define LINK_MODE_DEFAULT 1u	//250 kbps, no RS
#define LINK_FALLBACK_MODE 0u //fallback mode in the event both boards cant confirm mode change
#define LINK_FALLBACK_US 1000000u //timeout time for switching modes and used to switch to mode 0
#define LINK_FALLBACK_MARGIN_US 10000u //sender falls back this much earlier than the receiver
#define LINK_CONFIRM_TRIES 5u	//POLLs the sender sends in the new mode to confirm the mode switch
#define LINK_CONFIRM_NEED 3u	//how many POLLs needs to be answered for the switch to start settling
#define LINK_DATA	0x01u
#define LINK_ACK	0x02u
#define LINK_POLL	0x03u

//TX FIFO queue for streaming
#ifndef LINK_STREAM
#define LINK_STREAM 1
#endif

//Mode Controller
#define CTL_ALPHA 0.04f	//How fast old measurements fade
#define CTL_HYSTERESIS 1.15F //compare to another mode and see if its better (multiplier)
#define CTL_RETRY_MIN 32u //Bursts before a mode that did badly is tried again
#define CTL_RETRY_MAX 1024u // ^ doubles each time up till this value if a retry fails
#define CTL_LADDER_EFF 0.8f //Faster data rate is chosen after the current rate reaches this rate (multiplier)
#define CTL_SETTLED 100u //Threshold to set a mode as "settled" and leaving it won't count as a failed retry
#define CTL_COUNT_BYTES 1 //0: Rank modes by frames
						  //1: rank modes by data bytes

#ifndef SR_WINDOW
#define SR_WINDOW 16u	//window size
#endif


//Turntables
#define LINK_ACK_TIMEOUT_US 3500u	//us to wait for ACK after each send
#define LINK_TURNAROUND_US 150u	//wait for this long before any transmission (for TX->RX switch)
								//Usually its 130 us + SPI overhead
#define LINK_RX_SILENCE_US 7500u //ACK if a burst goes quiet for this long
#define LINK_MAX_POLLS 3u
#define LINK_GIVE_UP_US 2000000u //Give up on a message after this much time passed after POLL from ACK


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

	uint32_t rs_fixed; //receiver (bytes RS repaired)

	uint32_t mode_switches; //switches confirmed in the new mode
	uint32_t switch_fails; //tentative switches undone
	uint32_t fallbacks; //drops to LINK_FALLBACK_MODE after long POLLing

	uint32_t stream_resync; //Receiver lost stream so resync
	uint32_t sibling_sends; //resends sent without RS because they no longer fit in current mode
} link_stats_t;

typedef struct {
	uint8_t base;		//oldest sequence not yet ACKed
	uint8_t next_seq;	//sequence the next queed frame gets
	bool acked[SR_WINDOW];	//ACK window
	uint8_t len[SR_WINDOW];
	uint8_t data[SR_WINDOW][LINK_SLOT_BYTES];	//kept until ACKED for resends
}sr_tx_t;

typedef struct {
	uint8_t base;	//next in-order seq still needed
	bool have[SR_WINDOW];	//frames still buffered and undelivered
	uint8_t len[SR_WINDOW];
	uint8_t data[SR_WINDOW][LINK_SLOT_BYTES];
}sr_rx_t;

//Link quality the receiver reported in its last ACK (sender side)
typedef struct{
	bool valid; //false until the first ACK arrives
	uint8_t ok_frames; //DATA frames that arrived intact in the last burst
	uint8_t bad_frames; //frames that arrived but were too damaged (rejected)
	uint8_t fixed_bytes; //bytes RS repaired in the last burst
	uint8_t mode_ack; //mode the receiver accepted or FRAME_MODE_NONE (0xFF)

}link_quality_t;


void link_propose_mode(uint8_t mode); //sender asks receiver to move to this mode
uint8_t link_current_mode(void);	//the mode the board is on right now
void link_set_auto(bool on); //mode automatic controller on/off

void link_init(void);
const link_quality_t *link_last_quality(void);
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










