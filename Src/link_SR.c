//Selective Repeat




#include "nrf24.h"
#include "crc16.h"
#include "port.h"
#include <string.h>
#include "link_SR.h"

#ifndef NODE_LOCAL	//for simulator
#define NODE_LOCAL
#endif

static NODE_LOCAL link_stats_t stats;

static NODE_LOCAL sr_tx_t tx_win;
static NODE_LOCAL sr_rx_t rx_win;

static NODE_LOCAL bool rx_burst_open;
static NODE_LOCAL uint32_t rx_last_ms;

typedef enum{
	D_TYPE = 0,
	D_SEQ = 1,
	D_REM = 2,
	D_LEN = 3,
	D_DATA = 4,
	D_CRC = 30
} DATA;

typedef enum{
	A_TYPE = 0,
	A_BASE = 1,
	A_BM = 2,
	A_CRC = 30
} ACK;

typedef enum{
	P_TYPE = 0,
	P_CRC = 30
} POLL;



//build frames
//build data frame
static void build_data(uint8_t f[NRF24_PAYLOAD], uint8_t type, uint8_t seq, uint8_t rem,
			uint8_t len, const uint8_t *data){

	memset (f, 0, NRF24_PAYLOAD);	//init array f with 0
	f[D_TYPE] = type;	//build frame
	f[D_SEQ] = seq;
	f[D_REM] = rem;
	f[D_LEN] = len;	//data length

	if (len) memcpy (&f[D_DATA], data, len);	//build data

	uint16_t crc = crc16_ccitt(f, D_CRC);	//Build CRC
	f[D_CRC] = (uint8_t)(crc >>8);
	f[D_CRC + 1] = (uint8_t)(crc & 0xFF);
}


//build ACK frame
static void build_ack(uint8_t f[NRF24_PAYLOAD], uint8_t type, uint8_t base, uint16_t bm){

	memset (f, 0, NRF24_PAYLOAD);	//init array f with 0
	f[A_TYPE] = type;	//build frame
	f[A_BASE] = base;
	f[A_BM] = (uint8_t)(bm >>8);	//build bitmap
	f[A_BM + 1] = (uint8_t)(bm & 0xFF);

	uint16_t crc = crc16_ccitt(f, A_CRC);	//Build CRC
	f[A_CRC] = (uint8_t)(crc >>8);
	f[A_CRC + 1] = (uint8_t)(crc & 0xFF);
}

//build POLL frame
static void build_poll(uint8_t f[NRF24_PAYLOAD], uint8_t type){

	memset (f, 0, NRF24_PAYLOAD);	//init array f with 0
	f[P_TYPE] = type;	//build frame

	uint16_t crc = crc16_ccitt(f, P_CRC);	//Build CRC
	f[P_CRC] = (uint8_t)(crc >>8);
	f[P_CRC + 1] = (uint8_t)(crc & 0xFF);
}


static bool frame_ok(const uint8_t f[NRF24_PAYLOAD]){

	//it uses position D_CRC/D_TYPE and it works because DATA, ACK, and POLL
	//All use the same CRC position in frame array. If that changes this
	//function BREAKS!
	uint16_t got = (uint16_t)((f[D_CRC] << 8) | f[D_CRC + 1]);
	if (crc16_ccitt(f, D_CRC) != got) return false;
	if (f[D_TYPE] != LINK_DATA && f[D_TYPE] != LINK_ACK && f[D_TYPE] != LINK_POLL) return false;
	if (f[D_TYPE] == LINK_DATA && f[D_LEN] > LINK_MAX_DATA) return false;	//CRC collision guard

	return true;	//nothing broken detected
}


const link_stats_t *link_stats(void){
	return &stats;
}

void sr_tx_init(sr_tx_t *tx){
    memset(tx, 0, sizeof *tx);
}

uint8_t sr_tx_in_flight(const sr_tx_t *tx){
	return (uint8_t)(tx->next_seq - tx->base);
}

//slot for a sequence
static uint8_t slot(uint8_t seq){
	return (uint8_t)(seq % SR_WINDOW);
}

bool sr_tx_queue(sr_tx_t *tx, const uint8_t *data, uint8_t len, uint8_t *seq_out){

	if (len > LINK_MAX_DATA) return false;
	if (sr_tx_in_flight(tx) >= SR_WINDOW) return false;	//window full

	uint8_t seq = tx->next_seq;
	uint8_t s = slot(seq);

	tx->acked[s] = false;	//new sequence so it is not acked
	tx->len[s] = len;
	if (len) memcpy(tx->data[s], data, len);	//guard

	tx->next_seq++;	//increment to next sequence
	if (seq_out) *seq_out = seq;
	return true;
}
//return number of pending tx that need acks and update out_seqs with
//frames that need acks
uint8_t sr_tx_pending(const sr_tx_t *tx, uint8_t out_seqs[SR_WINDOW]){
	uint8_t n = 0, inflight = sr_tx_in_flight(tx);

	for (uint8_t i = 0; i < inflight; i++){
		uint8_t seq = (uint8_t)(tx->base + i);
		if (!tx->acked[slot(seq)]) out_seqs[n++] = seq;	//update out_seqs array too
	}
	return n;
}
//return payload from frame
const uint8_t *sr_tx_payload(const sr_tx_t *tx, uint8_t seq, uint8_t *len_out){
	if ((uint8_t)(seq - tx->base) >= sr_tx_in_flight(tx)) return NULL;	//not in flight
	*len_out = tx->len[slot(seq)];
	return tx->data[slot(seq)];	//return data in window
}

//update bookkeeping
//marks frames that are ack confirmed and slides window forwards
bool sr_tx_on_ack(sr_tx_t *tx, uint8_t ack_base, uint16_t bitmap){

	uint8_t inflight = sr_tx_in_flight(tx);

	uint8_t advance = (uint8_t)(ack_base - tx->base);
	if (advance > inflight) return false;	//stale ACK or corruption that wasnt detected

	for (uint8_t i = 0; i < inflight; i++){
		uint8_t seq = (uint8_t)(tx->base + i);

		if (i < advance){
			//before ack_base (receiver has delivered it)
			tx->acked[slot(seq)] = true;
		} else {
			//At or after ack_base (check selective bitmap)
			uint8_t bit = (uint8_t)(seq - ack_base);	//0 to 15
			if (bitmap & (uint16_t)(1u << bit)) tx->acked[slot(seq)] = true;
		}
	}

	//Release consecutive confirmed frames form the front (slide)
	//Selective Repeat only moves base past an unbroken run
	//a gap stays in the window until gap is filled
	while (sr_tx_in_flight(tx) && tx->acked[slot(tx->base)]){
		tx->acked[slot(tx->base)] = false;	//clean slot for reuse, set to false for next loop around
		tx->base++;
	}
	return true;
}



//Transmit one frame and then go back to receiving

//Wait before sending
//Need around 130 us + SPI time to switch back to receive or may cause issues

//Listen after sending
//radio can only transmit or receive so after sending go back to receive
static void radio_tx(const uint8_t f[NRF24_PAYLOAD]){
	port_delay_us(LINK_TURNAROUND_US);
	if (nrf24_send(f, 5) != NRF_OK) stats.radio_err++;
	nrf24_listen();
}


//Watch the radio for up to timeout_ms for a valid ACK
//Bad frames are counted and skipped and framres that arent ACKed are skipped
static bool wait_for_ack (uint32_t timeout_ms, uint8_t *base, uint16_t *bitmap){
	uint8_t rxf[NRF24_PAYLOAD];
	uint32_t t0 = port_millis();

	while ((uint32_t)(port_millis() - t0) < timeout_ms){
		if (!nrf24_poll_rx(rxf)) continue;
		if (!frame_ok(rxf)) {
			stats.crc_fail++;
			continue;
		}
		if (rxf[A_TYPE] != LINK_ACK) continue;

		*base = rxf[A_BASE];
		*bitmap = (uint16_t)((rxf[A_BM] << 8) | rxf[A_BM + 1]);	//rejoints the two bytes
		return true;
	}
	return false;
}










void sr_rx_init(sr_rx_t *rx){
	memset(rx, 0, sizeof(*rx));
}

void link_init(void){
	memset(&stats, 0, sizeof(stats));
	rx_burst_open = false;
	sr_tx_init(&tx_win);
	sr_rx_init(&rx_win);
	nrf24_listen();
}



//Recevier


sr_rx_result_t sr_rx_accept(sr_rx_t *rx, uint8_t seq, const uint8_t *data, uint8_t len){

	if (len > LINK_MAX_DATA) return SR_RX_OUTSIDE;	//frame_ok should stop this

	uint8_t ahead = (uint8_t)(seq - rx->base);	//0 = exactly what we need
	uint8_t behind = (uint8_t)(rx->base - seq);	//1 = the one just delivered

	if (ahead < SR_WINDOW){
		uint8_t s = slot(seq);
		if (rx->have[s]) return SR_RX_DUP;	//already buffered and received
		rx->have[s] = true;
		rx->len[s] = len;
		if (len) memcpy(rx->data[s], data, len);	//this is data
		return SR_RX_NEW;
	}


	//Received and sent ACK but ACK was lost
	//Report DUP
	if (behind >= 1 && behind <= SR_WINDOW) return SR_RX_DUP;

	return SR_RX_OUTSIDE;
}

bool sr_rx_deliver(sr_rx_t *rx, uint8_t *data_out, uint8_t *len_out){

	uint8_t s = slot(rx->base);
	if (!rx->have[s]) return false;	//gap at the front (wait)

	//deliver data and increment base
	*len_out = rx->len[s];
	if (*len_out) memcpy(data_out, rx->data[s], *len_out);
	rx->have[s] = false;
	rx->base++;
	return true;
}

uint16_t sr_rx_bitmap(const sr_rx_t *rx){
	uint16_t bm = 0;
	for (uint8_t i = 0; i < SR_WINDOW; i++){
		if (rx->have[slot((uint8_t)(rx->base + i))]) bm |= (uint16_t)(1u << i);
	}
	return bm;
}

// Sender

//Send every onconfirmed frame in the window as a burst
//Get one ack for the whole burst (POLL if not)
//Return true if ACK was received
static bool run_burst(void){
	uint8_t list[SR_WINDOW];
	uint8_t n = sr_tx_pending(&tx_win, list);
	if (n == 0) return true;

	//Burst
	uint8_t txf[NRF24_PAYLOAD];
	for (uint8_t i = 0; i < n; i++){
		uint8_t len = 0;
		const uint8_t *payload = sr_tx_payload(&tx_win, list[i], &len);
		if (!payload) continue;	//payload does not exist

		// sending burst counts down remaining which tells us how much is left
		//0 means busrt is over
		build_data(txf, LINK_DATA, list[i], (uint8_t)(n - 1 - i), len, payload);

		//Only first frame needs turnaround delay
		//Receiver may still be switching back to RX after its ACK
		if (i == 0) port_delay_us(LINK_TURNAROUND_US);
		if (nrf24_send(txf,5) != NRF_OK) stats.radio_err++;
		if(i == n - 1u) nrf24_listen();
		stats.frames_sent++;
	}
	stats.bursts++;

	//Wait for ACK and POLL if not received
	//Loop runs once per wait
	uint8_t base;
	uint16_t bitmap;
	for(uint8_t poll = 0;/*nothing here */ ; poll++){
		if (wait_for_ack(LINK_ACK_TIMEOUT, &base, &bitmap)){
			//Count how many messages this ACK released for stats
			uint8_t before = sr_tx_in_flight(&tx_win);
			if (sr_tx_on_ack(&tx_win, base, bitmap)){
				stats.tx_ok += (uint8_t)(before - sr_tx_in_flight(&tx_win));
			}
			return true;
		}
		stats.ack_timeouts++;

		//Out of POLLs (give up)
		if (poll == LINK_MAX_POLLS) return false;

		//Receiver answers a POLL by resending ACK
		build_poll(txf, LINK_POLL);
		radio_tx(txf);
		stats.polls_sent++;
	}
}

//run bursts until cond() holds
//give up after LINK_MAX_ROUNDS failed rounds in a row
//Resets if ACK received (for active link)
static link_result_t run_until(bool (*cond)(void)){
	uint32_t dead_rounds = 0; //counter
	while (!cond()){	//loops when window_has_room is false
		if (run_burst()) {
			dead_rounds = 0;
		} else if (++dead_rounds >= LINK_MAX_ROUNDS){
			stats.tx_fail++;
			return LINK_FAIL;
		}
	}
	return LINK_OK;
}
//window full or no?
static bool window_has_room(void){
	return sr_tx_in_flight(&tx_win) < SR_WINDOW;
}
//window empty?
static bool window_empty(void){
	return sr_tx_in_flight(&tx_win) == 0;
}



link_result_t link_send(const uint8_t *data, uint8_t len){

	if (len > LINK_MAX_DATA) return LINK_FAIL;
	//Window full: push bursts until an ACK frees a slot
	//else just queue up
	if (run_until(window_has_room) != LINK_OK) return LINK_FAIL;

	sr_tx_queue(&tx_win, data, len, NULL);
	return LINK_OK;
}

link_result_t link_flush(void){
	return run_until(window_empty);
}



//Receiver

//send ACK and tell sender what we have on bitmap
//Sending frame closes current burst
static void send_ack(void){
	uint8_t f[NRF24_PAYLOAD];
	build_ack(f, LINK_ACK, rx_win.base, sr_rx_bitmap(&rx_win));
	radio_tx(f);
	stats.acks_sent++;
	rx_burst_open = false;
}


//Check radio for one incoming frame and react to it
//send an ACK if
//Last frame of a burst arrives (remaining == 0)
//A POLL arives
//A burst has gone quiet for LINK_RX_SLIENCE_MS

static void rx_service(void){
	uint8_t f[NRF24_PAYLOAD];

	if (nrf24_poll_rx(f)){
		if (!frame_ok(f)){	//corrupted
			stats.crc_fail++;
		} else if (f[D_TYPE] == LINK_DATA){	//received DATA (last frame)
			sr_rx_result_t r = sr_rx_accept(&rx_win, f[D_SEQ], &f[D_DATA], f[D_LEN]);
			if (r == SR_RX_DUP) stats.rx_dup++;

			//NEW or DUP mean burst is in progress
			//note the time and if its last remaining, ACK
			if (r != SR_RX_OUTSIDE){
				rx_burst_open = true;
				rx_last_ms = port_millis();
				if (f[D_REM] == 0) send_ack();	//last frame from the burst
			}
		} else if (f[D_TYPE] == LINK_POLL){	//received POLL
			send_ack();	//ACK was lost, resend it
		}
	}

	//Remaining = 0 frame got lost
	//Burst has gone quiet for LINK_RX_SILENCE_MS
	if (rx_burst_open && (uint32_t)(port_millis() - rx_last_ms) >= LINK_RX_SILENCE_MS){
		send_ack();
	}

}


//This should be called constantly on receiving board
//Checks the radio once and hands back the next in-order message if one is ready
//REturns false if theres nothing to deliver yet
bool link_poll_recv(uint8_t *data, uint8_t *len){
	rx_service();
	if (sr_rx_deliver(&rx_win, data, len)){
		stats.rx_ok++;
		return true;
	}
	return false;
}




















