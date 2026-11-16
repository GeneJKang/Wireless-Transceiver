//Selective Repeat




#include "nrf24.h"
#include "port.h"
#include <string.h>
#include <math.h>
#include "link_SR.h"
#include "frame.h"

//About how long one frame of burst takes for the controller to transmit
#if LINK_STREAM
#define LINK_FRAME_US_250K 1200u
#define LINK_FRAME_US_1M 310u
#define LINK_FRAME_US_2M 165u
#else
#define LINK_FRAME_US_250K 1400u
#define LINK_FRAME_US_1M 520u
#define LINK_FRAME_US_2M 380u
#endif




#ifndef NODE_LOCAL	//for simulator
#define NODE_LOCAL
#endif

static NODE_LOCAL uint8_t link_mode = LINK_MODE_DEFAULT;
static NODE_LOCAL link_stats_t stats;

static NODE_LOCAL sr_tx_t tx_win;
static NODE_LOCAL sr_rx_t rx_win;

//Packing (sender)
static NODE_LOCAL uint8_t tx_buf[LINK_TX_BUF];
static NODE_LOCAL uint16_t tx_buf_len;
static NODE_LOCAL uint8_t tx_rec_left;
static NODE_LOCAL uint16_t tx_need; //free bytes link_send is waiting for

//Packing (receiver)
static NODE_LOCAL uint8_t rx_chunk[LINK_SLOT_BYTES]; //[ptr][data size]
static NODE_LOCAL uint8_t rx_chunk_len; //data size length of the frame
static NODE_LOCAL uint8_t rx_chunk_off; //next byte to read in the data area
static NODE_LOCAL uint8_t rx_msg[LINK_MAX_DATA];
static NODE_LOCAL uint8_t rx_msg_len; //length from the header (0 = waiting for header)
static NODE_LOCAL uint8_t rx_msg_have; //bytes of received data so far
static NODE_LOCAL bool rx_synced; //false until a frame shows where the message starts

static NODE_LOCAL bool rx_burst_open;
static NODE_LOCAL uint32_t rx_last_us;

static NODE_LOCAL uint8_t q_ok , q_bad, q_fixed;
static NODE_LOCAL link_frame_t q_report;

//Qualtiy of the most recent ACK (sender)
static NODE_LOCAL link_quality_t last_q;
static NODE_LOCAL uint8_t proposed_mode; //mode sender is asking for
static NODE_LOCAL uint8_t alt_mode; //the other mode the receiver may be in or FRAME_MODE_NONE
static NODE_LOCAL uint8_t rx_proposal; //mode proposed
static NODE_LOCAL bool rx_verifying; //switched modes and checking if successful on otherside
static NODE_LOCAL uint8_t rx_prev_mode; //prev mode before switch
static NODE_LOCAL uint8_t rx_verify_heard;	//valid frames heard in the new mode
static NODE_LOCAL uint32_t rx_verify_start_us;
static NODE_LOCAL uint32_t last_good_us; //time of the last valid frame (both sides)



static NODE_LOCAL bool ctl_on;
static NODE_LOCAL bool ctl_fresh[FRAME_NUM_MODES];	//assumes it works perfectly
static NODE_LOCAL float ctl_frames[FRAME_NUM_MODES]; //recent frames delivered (older frames get pushed out)
static NODE_LOCAL float ctl_us[FRAME_NUM_MODES];	//recent time spent
static NODE_LOCAL uint16_t ctl_retry_in[FRAME_NUM_MODES]; //timeout (in bursts) until the mode can be tried again
static NODE_LOCAL uint16_t ctl_backoff[FRAME_NUM_MODES]; //next retry delay for this mode
static NODE_LOCAL float ctl_cal[3];	//speed of a perfect burst vs. timing table
static NODE_LOCAL float ctl_burst_n; //running average burst size (in frames)
static NODE_LOCAL uint16_t ctl_stay; //bursts sent in the current mode
static NODE_LOCAL uint16_t ctl_held;	//bursts since faster rates were last allowed to try again

const link_stats_t *link_stats(void){
	return &stats;
}

const link_quality_t *link_last_quality(void){
	return &last_q;
}

//Timeout for on data rate
typedef struct {
	uint32_t ack_timeout_us;	//how long to wait for an ACK (sender)
	uint32_t rx_silence_us;		//how long a burst can go quiet before ACKing (receiver)

	uint32_t frame_us; //how long one frame of a burst takes
	uint32_t ack_us;  //how long the ACK takes to come back after a burst
}link_timing_t;

//Returns the timeotus for a mode. They depend only on its data rate
static const link_timing_t *link_timing(uint8_t mode){
	static const link_timing_t t250 = { LINK_ACK_TIMEOUT_US, LINK_RX_SILENCE_US, LINK_FRAME_US_250K, 1600u};
	static const link_timing_t t1m = { 1550u, 2750u, LINK_FRAME_US_1M, 700u};
	static const link_timing_t t2m = { 1250u, 1950u, LINK_FRAME_US_2M, 550u};
	switch (frame_mode(mode) ->rf_dr){
	case NRF24_DR_2M: return &t2m;
	case NRF24_DR_1M: return &t1m;
	default: return &t250;
	}
}


static void apply_mode(uint8_t mode){ //change transition modes
	link_mode = mode;
	nrf24_set_rate(frame_mode(mode)->rf_dr);
	nrf24_listen();
}

static uint32_t verify_window_us(uint8_t new_mode){	//returns how long the receiver waits after switching
	return 2u * LINK_CONFIRM_TRIES * link_timing(new_mode)->ack_timeout_us;
}

void link_propose_mode(uint8_t mode){	//set the proposed mode if its valid
	if(mode < FRAME_NUM_MODES) proposed_mode = mode;
}

uint8_t link_current_mode(void){
	return link_mode;
}

void link_set_auto(bool on){
	ctl_on = on;
}

static void ctl_init(void){
	ctl_on = false;
	ctl_burst_n = 1.0f;
	ctl_stay = 0;
	ctl_held = 0;
	ctl_cal[0] = ctl_cal[1] = ctl_cal[2] = 1.0f;
	for (uint8_t m = 0; m < FRAME_NUM_MODES; m++){
		ctl_fresh[m] = true;
		ctl_frames[m] = ctl_us[m] = 0.0f;
		ctl_retry_in[m] = 0;
		ctl_backoff[m] = CTL_RETRY_MIN;
	}
}

//Index 0/1/2 for 250 kbps / 1 Mbps / 2 Mbps
static uint8_t ctl_rate_idx(uint8_t mode){
	uint8_t dr = frame_mode(mode)->rf_dr;
	return (dr == NRF24_DR_2M) ? 2u : (dr == NRF24_DR_1M) ? 1u : 0u;
}

//How long a burst of n frames should take in this mode when nothing goes wrong
static float ctl_clean_us(uint8_t mode, float n){
	const link_timing_t *t = link_timing(mode);
	return (float)LINK_TURNAROUND_US + n * (float)t->frame_us + (float)t->ack_us;
}

//value of one delivered frame
//for packing
static float ctl_frame_value(uint8_t mode){
	return CTL_COUNT_BYTES ? (float)frame_data_len(mode) : 1.0f;
}

//What the mode would deliver per microsecond on a perfect channel at the usual burst size
//This is best case scenario
static float ctl_clean_score(uint8_t mode){
	return ctl_cal[ctl_rate_idx(mode)] * ctl_burst_n * ctl_frame_value(mode) / ctl_clean_us(mode, ctl_burst_n);
}


//What the current mode is expected to deliver per microsecond
//measure rate, or the perfect channel rate if it has no measurements
static float ctl_score(uint8_t mode){
	if (ctl_fresh[mode] || (ctl_us[mode] <= 0.0f)) return ctl_clean_score(mode);
	return ctl_frame_value(mode) * ctl_frames[mode] / ctl_us[mode];
}

//Replaces a mode's measurements with an estimate
//Uses a ratio of eff
static void ctl_set_eff(uint8_t mode, float eff){
	ctl_fresh[mode] = false;
	ctl_us[mode] = ctl_clean_us(mode, ctl_burst_n);
	ctl_frames[mode] = eff * ctl_clean_score(mode) / ctl_frame_value(mode) * ctl_us[mode];
}

//Schedules the next retry (delay) after fail
//a fail doubles the delay everytime
static void ctl_schedule_retry(uint8_t mode, bool failed){
	if (!failed) ctl_backoff[mode] = CTL_RETRY_MIN;
	ctl_retry_in[mode] = ctl_backoff[mode];
	if (failed && ctl_backoff[mode] < CTL_RETRY_MAX) ctl_backoff[mode] = (uint16_t)(ctl_backoff[mode] * 2u);
}

//Mode transition failed so had to fall back out of a mode
//Mode is most likely bad
static void ctl_mode_fail(uint8_t mode){
	ctl_set_eff(mode, 0.0f);
	ctl_schedule_retry(mode, true);
}


//Switch test failed (fewer than 3 of 5 POLLs answered)
//This rates the mode on how many POLLs were answered 0 to 2
static void ctl_test_failed(uint8_t mode, uint8_t good, uint8_t tried){
	ctl_set_eff(mode, sqrtf((float)good / (float)tried));
	ctl_schedule_retry(mode, true);
}


//This is called after every burst (run_burst)
//Checks how the burst went and records the results
//Decides if it has to change modes depending on what is the best mode
static void ctl_update(uint8_t mode, uint8_t sent, uint8_t acked, bool first_try, uint32_t round_us){
	if (!ctl_on || sent == 0) return;

	//check what the best case scenario for the burst is
	float clean = ctl_clean_us(mode, (float)sent);
	float *cal = &ctl_cal[ctl_rate_idx(mode)];
	if (acked == sent && first_try) *cal += 0.1f * (clean / (float)round_us - *cal);

	ctl_burst_n += 0.15f *((float)sent - ctl_burst_n);

	//reduce the weight on older mode measurements and then add the newest burst
	float fade = CTL_ALPHA * (float)round_us / ctl_clean_us(mode, ctl_burst_n);
	if(fade > 0.3f) fade = 0.3f;
	if (acked == 0) fade = 0.5f;
	if (ctl_fresh[mode]){
		ctl_fresh[mode] = false;
		ctl_frames[mode] = ctl_us[mode] = 0.0f;
	}
	ctl_frames[mode] = (1.0f - fade) * ctl_frames[mode] + (float)acked;
	ctl_us[mode] = (1.0f - fade) * ctl_us[mode] + (float)round_us;

	//Count down the retry delays
	//if one goes to 0 then its a fresh mode again and can be used
	for (uint8_t m = 0; m < FRAME_NUM_MODES; m++){
		if (ctl_retry_in[m] && (--ctl_retry_in[m] == 0) && (m != link_mode)) ctl_fresh[m] = true;
	}

	//if a switch is happening let it happen
	if (alt_mode != FRAME_MODE_NONE) return;

	if (ctl_stay < 0xFFFF) ctl_stay++;

	//Best measured efficiency at each rate (-1 if nothing)
	float rate_eff[3] = {-1.0f, -1.0f, -1.0f};
	for (uint8_t m = 0; m < FRAME_NUM_MODES; m++){
		if (ctl_fresh[m]) continue;
		float eff = ctl_score(m) / ctl_clean_score(m); //ratio
		if (eff > rate_eff[ctl_rate_idx(m)]) rate_eff[ctl_rate_idx(m)] = eff;
	}

	//Do not try a unused mode if a weaker mode than it failed
	//Use the unused mode if a weaker mode failed IF the noise is interference
	bool hold_back = (ctl_held < CTL_RETRY_MAX);
	bool held_any = false;
	uint8_t best = link_mode;
	for (uint8_t m = 0; m < FRAME_NUM_MODES; m++){
		uint8_t r = ctl_rate_idx(m);
		if (ctl_fresh[m] && (r > ctl_rate_idx(link_mode)) && (rate_eff[r - 1] >= 0.0f) && (rate_eff[r - 1] < CTL_LADDER_EFF)){
			held_any = true;
			if (hold_back) continue;
		}
		if (ctl_score(m) > ctl_score(best)) best = m;
	}
	ctl_held = (held_any && hold_back) ? (uint16_t)(ctl_held + 1u) : 0u;
	if ((best == link_mode) || (ctl_score(best) <= CTL_HYSTERESIS * ctl_score(link_mode))){
		proposed_mode = link_mode; //nothing is better so dont change
	} else if (best != proposed_mode){
		//When leaving a mode the controller still holds the last measurement (stale) so retry that mode right after to be able to move back
		//Exponential backoff
		ctl_schedule_retry(link_mode, ctl_stay < CTL_SETTLED);
		proposed_mode = best;
	}
}



//Adds to a 8 bit counter that avoids wrapping
//A very bad burst should report 255
static void sat_add(uint8_t *c, uint8_t by){
	*c = (uint8_t)((*c + by > 255) ? 255 : *c + by);
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

	if (len > LINK_SLOT_BYTES) return false;
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


//Watch the radio for up to timeout_us for a valid ACK
//Bad frames are counted and skipped and framres that arent ACKed are skipped
static bool wait_for_ack (uint32_t timeout_us, uint8_t *base, uint16_t *bitmap){
	uint8_t rxf[NRF24_PAYLOAD];
	uint32_t t0 = port_micros();

	while ((uint32_t)(port_micros() - t0) < timeout_us){
		if (!nrf24_poll_rx(rxf)) continue;
		link_frame_t f;
		if (frame_parse(rxf, link_mode, &f) != FRAME_OK) {
			stats.crc_fail++;
			continue;
		}
		if (f.type != LINK_ACK) continue;

		*base = f.base;
		*bitmap = f.bitmap;
		last_q.valid = true;
		last_q.ok_frames = f.ok_frames;
		last_q.bad_frames = f.bad_frames;
		last_q.fixed_bytes = f.fixed_bytes;
		last_q.mode_ack = f.mode_ack;
		last_good_us = port_micros();
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
	link_mode = LINK_MODE_DEFAULT;
	q_ok = q_bad = q_fixed = 0;
	tx_buf_len = 0;
	tx_rec_left = 0;
	rx_chunk_len = rx_chunk_off = 0;
	rx_msg_len = rx_msg_have = 0;
	rx_synced = false;
	memset(&q_report, 0, sizeof q_report);
	q_report.mode_ack = FRAME_MODE_NONE;
	memset(&last_q, 0, sizeof last_q);
	proposed_mode = link_mode;
	alt_mode = FRAME_MODE_NONE;
	ctl_init();
	rx_proposal = FRAME_MODE_NONE;
	rx_verifying = false;
	last_good_us = port_micros();
	nrf24_set_rate(frame_mode(link_mode)->rf_dr);	//radio must match the starting mode
	sr_tx_init(&tx_win);
	sr_rx_init(&rx_win);
	nrf24_listen();
}



//Recevier


sr_rx_result_t sr_rx_accept(sr_rx_t *rx, uint8_t seq, const uint8_t *data, uint8_t len){

	if (len > LINK_SLOT_BYTES) return SR_RX_OUTSIDE;	//frame_ok should stop this

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

//Puts a stored window slot [ptr][chunk] into a DATA frame
static void pack_slot(link_frame_t *f, const uint8_t *slot_data, uint8_t len){
	memset(f->data, 0, sizeof f->data);
	f->ptr = slot_data[0];
	f->cut = (uint8_t)(len - 1);
	if (len > 1) memcpy (&f->data, &slot_data[1], len - 1);
}

//Copies a received DATA frame into [ptr][chunk]
static bool unpack_slot(const link_frame_t *f, uint8_t *out, uint8_t *len){
	uint8_t clen = f->cut ? f->cut : frame_data_len(f->mode);
	if (f->ptr != FRAME_PTR_NONE && f->ptr >= clen) return false;
	out[0] = f->ptr;
	memcpy(&out[1], f->data, clen);
	*len = (uint8_t)(1u + clen);
	return true;
}


//same rate as current mode but no RS
static uint8_t no_rs_mode (uint8_t mode){
	for (uint8_t m = 0; m < FRAME_NUM_MODES; m++){
		if (frame_mode(m)->rf_dr == frame_mode(mode)->rf_dr && frame_mode(m)->rs_len == 0) return m;
	}
	return mode;
}

//Cut the front of tx_buf into new frames while the window has free slots
//Frames are cut and then transmitted
static void cut_frames(void){
	while (tx_buf_len > 0 && sr_tx_in_flight(&tx_win) < SR_WINDOW){
		uint8_t slot_data[LINK_SLOT_BYTES];
		uint8_t c = frame_data_len(link_mode);
		if (tx_buf_len < c) c = (uint8_t)tx_buf_len;

		if (tx_rec_left >= c){	//still inside the same frame
			slot_data[0] = FRAME_PTR_NONE;
			tx_rec_left = (uint8_t)(tx_rec_left - c);
		} else{
			slot_data[0] = tx_rec_left;
			//search the record headers to find where the current data chunk's last record ends
			uint16_t off = tx_rec_left;
			while (off < c) off = (uint16_t)(off + 1u + tx_buf[off]);
			tx_rec_left = (uint8_t)(off - c);
		}
		memcpy(&slot_data[1], tx_buf, c);
		sr_tx_queue(&tx_win, slot_data, (uint8_t)(1u + c), NULL);

		tx_buf_len = (uint16_t)(tx_buf_len - c);
		memmove(tx_buf, &tx_buf[c], tx_buf_len);
	}
}

//applies an ACK to the send window and counts how many messages it released
static void apply_ack (uint8_t base, uint16_t bitmap){
	uint8_t before = sr_tx_in_flight(&tx_win);
	if (sr_tx_on_ack(&tx_win, base, bitmap)){
		stats.tx_ok += (uint8_t)(before - sr_tx_in_flight(&tx_win));
	}
}

//Send one POLL in mode and wait there for an ACK.
static bool poll_in(uint8_t mode, uint8_t *base, uint16_t *bitmap){
	uint8_t txf[NRF24_PAYLOAD];
	link_frame_t pf = {0};

	if (mode != link_mode) apply_mode(mode);
	pf.type = LINK_POLL;
	frame_build(txf, &pf, link_mode);
	radio_tx(txf);
	stats.polls_sent++;
	return wait_for_ack(link_timing(link_mode)->ack_timeout_us, base, bitmap);
}


//Sender side of mode switch
//When ACK received accepts the proposal swap and store previous mode into alt_mode
static void sender_switch(uint8_t new_mode){
	uint8_t base;
	uint16_t bitmap;
	uint8_t old = link_mode;
	uint8_t good = 0;
	uint8_t bad = 0;
	uint32_t start_us = port_micros();

	while ((good < LINK_CONFIRM_NEED) && (bad <= LINK_CONFIRM_TRIES - LINK_CONFIRM_NEED)){
		if (poll_in(new_mode, &base, &bitmap)){
			apply_ack(base, bitmap);
			good++;
		} else{
			bad++;
		}
	}

	if ( good >= LINK_CONFIRM_NEED){
		alt_mode = FRAME_MODE_NONE;
		proposed_mode = new_mode;
		ctl_stay = 0;
		stats.mode_switches++;
		return;
	}

	//receiver stays in new mode until the test window runs out
	uint32_t window = verify_window_us(new_mode) + 500u;
	uint32_t spent = (uint32_t)(port_micros() - start_us);
	if (spent < window) port_delay_us(window - spent);

	stats.switch_fails++;
	ctl_test_failed(new_mode, good, (uint8_t)(good + bad));

	//Find out where the receiver ended up with one POLL in each mode
	if (poll_in(old, &base, &bitmap)){
		apply_ack(base, bitmap);
		alt_mode = FRAME_MODE_NONE;
		proposed_mode = old;
		return;
	}

	if (poll_in(new_mode, &base, &bitmap)){
		apply_ack(base, bitmap);
		alt_mode = FRAME_MODE_NONE;
		proposed_mode = new_mode;
		ctl_stay = 0;
		stats.mode_switches++;
		return;
	}
	apply_mode(old);
	alt_mode = new_mode; //keep searching
	proposed_mode = old;



}

//Handles a valid ACK received on current mode
static void sender_on_ack(uint8_t base, uint16_t bitmap){
	apply_ack(base, bitmap);
	alt_mode = FRAME_MODE_NONE;
	if(proposed_mode != link_mode && last_q.mode_ack == proposed_mode){
		sender_switch(proposed_mode);
	}
}

//If the switch was unsuccessful find where the receiver is at
static uint8_t search_mode(void){
	if (alt_mode != FRAME_MODE_NONE) return alt_mode;
	if (proposed_mode != link_mode) return proposed_mode;
	return FRAME_MODE_NONE;
}

//If no valid ACK has arrivved for LINK_FALLBACK_US the receiver has dropped to fallback mode on its own timer
//So swap to fallback mode
static void sender_check_fallback(void){
	if (link_mode == LINK_FALLBACK_MODE) return;
	if ((uint32_t)(port_micros() - last_good_us) < LINK_FALLBACK_US - LINK_FALLBACK_MARGIN_US) return;
	ctl_mode_fail(link_mode); //the current mode stopped working
	apply_mode(LINK_FALLBACK_MODE);
	proposed_mode = LINK_FALLBACK_MODE;
	alt_mode = FRAME_MODE_NONE;
	stats.fallbacks++;
}





//Send every onconfirmed frame in the window as a burst
//Get one ack for the whole burst (POLL if not)
//Return true if ACK was received
static bool run_burst(void){
	uint8_t list[SR_WINDOW];
	sender_check_fallback();
	cut_frames(); //after fallback check, cut the frames in the new mode (if changed modes)
	uint8_t n = sr_tx_pending(&tx_win, list);
	if (n == 0) return true;

	//Get measurements to judge the current mode
	uint8_t burst_mode = link_mode;
	uint32_t burst_start_us = port_micros();
	bool measure = (alt_mode == FRAME_MODE_NONE); //measure if only both boards are in the same mode

	//Burst
	uint8_t txf[NRF24_PAYLOAD];

	//Receiver can be switching back to RX after the last ACK so wait once before starting burst
	port_delay_us(LINK_TURNAROUND_US);
#if LINK_STREAM
	nrf24_tx_init();
#endif
	for (uint8_t i = 0; i < n; i++){
		uint8_t len = 0;
		const uint8_t *payload = sr_tx_payload(&tx_win, list[i], &len);
		if (!payload) continue;	//payload does not exist

		// sending burst counts down remaining which tells us how much is left
		//0 means busrt is over
		link_frame_t df = {0};
		df.type = LINK_DATA;
		df.seq = list[i];
		df.rem = (uint8_t)(n - 1 - i); //0 is last frame of the burst
		df.next_mode = proposed_mode;	//set to proposed mode if there is a switch

		pack_slot(&df, payload, len);
		//Check f the frame was setup while the link was in a mode with more available data byte size
		uint8_t send_mode = link_mode;
		if ((uint8_t)(len - 1) > frame_data_len(link_mode)){
			send_mode = no_rs_mode(link_mode);
			stats.sibling_sends++;
		}
		frame_build(txf, &df, send_mode);

#if LINK_STREAM	//Queue the transmission
		if (nrf24_tx_push(txf,5) != NRF_OK) stats.radio_err++;
#else
		if (nrf24_send(txf,5) != NRF_OK) stats.radio_err++;
#endif
		stats.frames_sent++;
	}
#if LINK_STREAM	//wait for last frame to be transmitted
	if (nrf24_tx_finish(5) != NRF_OK) stats.radio_err++;
#endif
	nrf24_listen();
	stats.bursts++;

	//Wait for ACK and POLL if not received
	//Loop runs once per wait
	uint8_t base;
	uint16_t bitmap;
	for(uint8_t poll = 0;/*nothing here */ ; poll++){
		if (wait_for_ack(link_timing(link_mode)->ack_timeout_us, &base, &bitmap)){
			uint32_t round_us = (uint32_t)(port_micros() - burst_start_us);
			sender_on_ack(base, bitmap);

			uint8_t still = sr_tx_pending(&tx_win, list);
			if (measure) ctl_update(burst_mode, n, (uint8_t)(n - still), poll == 0, round_us);
			return true;
		}
		stats.ack_timeouts++;

		//Out of POLLs (give up)
		if (poll == LINK_MAX_POLLS){
			if (measure) ctl_update(burst_mode, n, 0, false, (uint32_t)(port_micros() - burst_start_us));
			return false;
		}

		//A switch is in progress and the receiver may be in another mode
		//POLL in the other mode and if theres no response come back here
		uint8_t other = search_mode();
		if (other != FRAME_MODE_NONE && (poll & 1u) == 0){
			uint8_t home = link_mode;
			if (poll_in(other, &base, &bitmap)){
				apply_ack(base, bitmap);
				if (other == alt_mode){
					//The mode was left after a failed test but the receiver never committed to the mode
					alt_mode = FRAME_MODE_NONE;
					proposed_mode = link_mode;
					stats.mode_switches++;
				} else{
					//The receiver accepted the mode proposal but the ACK was lost
					apply_mode(home);
					sender_switch(other);
				}
				return true;
			}
			apply_mode(home);
		}

		//Receiver answers a POLL by resending ACK

		link_frame_t pf = {0};
		pf.type = LINK_POLL;
		frame_build(txf, &pf, link_mode);

		radio_tx(txf);
		stats.polls_sent++;
	}
}

//run bursts until cond() holds
//give up after LINK_GIVE_UP_US
//Resets if ACK received (for active link)
static link_result_t run_until(bool (*cond)(void)){
	uint32_t last_ack_us = port_micros();
	while (!cond()){	//loops when buf_has_room is false
		if (run_burst()) {
			last_ack_us = port_micros();
		} else if ((uint32_t)(port_micros() - last_ack_us) >= LINK_GIVE_UP_US){
			stats.tx_fail++;
			return LINK_FAIL;
		}
	}
	return LINK_OK;
}
//buf full or no?
static bool buf_has_room(void){
	return (uint16_t)(LINK_TX_BUF - tx_buf_len) >= tx_need;
}
//buf empty?
static bool all_sent(void){
	return tx_buf_len == 0 && sr_tx_in_flight(&tx_win) == 0;
}



link_result_t link_send(const uint8_t *data, uint8_t len){

	if (len == 0 || len > LINK_MAX_DATA) return LINK_FAIL;
	//buffer full: push bursts until an ACK frees a slot
	//else just queue up
	tx_need = (uint16_t)(1u + len);
	if (run_until(buf_has_room) != LINK_OK) return LINK_FAIL;

	tx_buf[tx_buf_len++] = len; //record header (message length)
	memcpy(&tx_buf[tx_buf_len], data, len);
	tx_buf_len = (uint16_t)(tx_buf_len + len);
	return LINK_OK;
}

link_result_t link_flush(void){
	return run_until(all_sent);
}



//Receiver

//send ACK and tell sender what we have on bitmap
//Sending frame closes current burst
static void send_ack(void){
	uint8_t f[NRF24_PAYLOAD];

	//If the burst is still open use this burst's counter values
	//else just use the same report
	if (rx_burst_open){
		q_report.ok_frames = q_ok;
		q_report.bad_frames = q_bad;
		q_report.fixed_bytes = q_fixed;
		q_report.mode_ack = (rx_proposal != FRAME_MODE_NONE && !rx_verifying) ? rx_proposal : FRAME_MODE_NONE;	//Accept the mode proposal change in this burst
		q_ok = q_bad = q_fixed = 0;
	}
	bool switching = rx_burst_open && q_report.mode_ack != FRAME_MODE_NONE;

	link_frame_t af = q_report;
	af.type = LINK_ACK;
	af.base = rx_win.base;
	af.bitmap = sr_rx_bitmap(&rx_win);
	frame_build(f, &af, link_mode);

	radio_tx(f);
	stats.acks_sent++;
	rx_burst_open = false;

	//ACK accepted a switch so move modes now
	//Sender confirms by POLLing a new mode
	if (switching){
		rx_prev_mode = link_mode;
		apply_mode(q_report.mode_ack);
		rx_verifying = true;
		rx_verify_heard = 0;
		rx_verify_start_us = port_micros();
		rx_proposal = FRAME_MODE_NONE;
	}
}


//Check radio for one incoming frame and react to it
//send an ACK if
//Last frame of a burst arrives (remaining == 0)
//A POLL arives
//A burst has gone quiet for LINK_RX_SLIENCE_MS

static void rx_service(void){
	uint8_t f[NRF24_PAYLOAD];

	while (nrf24_poll_rx(f)){
		link_frame_t rf;
		uint8_t msg[LINK_SLOT_BYTES];
		uint8_t len;

		frame_result_t pr = frame_parse(f, link_mode, &rf);

		uint8_t sib = no_rs_mode(link_mode); //accept the same rate's no RS alternative in case a message doesnt fit a frame's data area
		if (pr != FRAME_OK && sib != link_mode && (f[0] & 0x0Fu) == sib){
			frame_result_t pr2 = frame_parse(f, sib, &rf);
			if (pr2 == FRAME_OK) pr = pr2;
		}

		if (pr == FRAME_OK){	//frame is good

			//Any valid frame in the new mode confirms the switch
			//Clear mode_ack so repeated ACKS dont ask for the switch again
			//Test if the POLLs have been heard in the new mode and confirm the switch
			if (rx_verifying && ((++rx_verify_heard >= LINK_CONFIRM_NEED) || (rf.type == LINK_DATA))){
				rx_verifying = false;
				q_report.mode_ack = FRAME_MODE_NONE; //Stop asking for the switch in repeating ACKs
				stats.mode_switches++;
			}
		} if (pr != FRAME_OK){	//corrupted (or sent in a different mode)
			stats.crc_fail++;
			sat_add(&q_bad, 1);
		} else if (rf.type == LINK_DATA){	//received DATA (last frame)
			if (!unpack_slot(&rf, msg, &len)){
				stats.crc_fail++;
				sat_add(&q_bad, 1);
			} else{
				sat_add(&q_ok, 1);
				sat_add(&q_fixed, rf.fixed);
				stats.rs_fixed += rf.fixed;
				rx_proposal = (rf.next_mode != link_mode) ? rf.next_mode : FRAME_MODE_NONE; //store the latest proposal
				sr_rx_result_t r = sr_rx_accept(&rx_win, rf.seq, msg, len);
				if (r == SR_RX_DUP) stats.rx_dup++;

				//Only a frame carrying something new restarts the fallback clock
				if (r == SR_RX_NEW) last_good_us = port_micros();
				//NEW or DUP mean burst is in progress
				//note the time and if its last remaining, ACK
				if (r != SR_RX_OUTSIDE){
					rx_burst_open = true;
					rx_last_us = port_micros();
					if (rf.rem == 0) send_ack();	//last frame from the burst
				}
			}
		} else if (rf.type == LINK_POLL){	//received POLL
			send_ack();	//ACK was lost, resend it
		}
	}

	//Remaining = 0 frame got lost
	//Burst has gone quiet for LINK_RX_SILENCE_MS
	if (rx_burst_open && (uint32_t)(port_micros() - rx_last_us) >= link_timing(link_mode)->rx_silence_us){
		send_ack();
	}

	//Switch was never confirmed and the sender didnt get our ACK or the confirmation POLLs were lost go back to the previous used mode
	if (rx_verifying && (uint32_t)(port_micros() - rx_verify_start_us) >= verify_window_us(link_mode)){
		apply_mode(rx_prev_mode);
		rx_verifying = false;
		q_report.mode_ack = FRAME_MODE_NONE;
		stats.switch_fails++;
	}

	//Nothing valid for LINK_FALLBACK_US and cant find the sender so go to fallback mode
	if (link_mode != LINK_FALLBACK_MODE && !rx_verifying && (uint32_t)(port_micros() - last_good_us) >= LINK_FALLBACK_US){
		apply_mode(LINK_FALLBACK_MODE);
		rx_burst_open = false;
		rx_proposal = FRAME_MODE_NONE;
		stats.fallbacks++;
	}
}



//This should be called constantly on receiving board
//Checks the radio once and hands back the next in-order message if one is ready
//REturns false if theres nothing to deliver yet
bool link_poll_recv(uint8_t *data, uint8_t *len){
	rx_service();


	for (;;){
		//current frame has been filled up so use next one
		if (rx_chunk_off >= rx_chunk_len){
			uint8_t slen;

			if (!sr_rx_deliver(&rx_win, rx_chunk, &slen)) return false;
			rx_chunk_len = (uint8_t)(slen - 1u);
			rx_chunk_off = 0;
			uint8_t ptr = rx_chunk[0];

			//check ptr against count of where next message starts
			//if they disagree drop the partial message and restart at ptr
			uint8_t remaining = (rx_msg_len == 0) ? 0u : (uint8_t)(rx_msg_len - rx_msg_have);
			bool agrees = (ptr == remaining) || (ptr == FRAME_PTR_NONE && rx_msg_len != 0);

			if (!rx_synced || !agrees){
				if (rx_synced) stats.stream_resync++;
				rx_msg_len = rx_msg_have = 0;
				if (ptr == FRAME_PTR_NONE){
					rx_synced = false;
					rx_chunk_off = rx_chunk_len; //nothing starts here, skip it
					continue;
				}
				rx_synced = true;
				rx_chunk_off = ptr;
			}
		}

		const uint8_t *area = &rx_chunk[1];
		if (rx_msg_len == 0){ //expecting a record header
			uint8_t h = area[rx_chunk_off];
			if (h == 0){  //rest of the frame is empty (padding)
				rx_chunk_off = rx_chunk_len;
				continue;
			}
			if (h > LINK_MAX_DATA){	//invalid length
				rx_synced = false;
				stats.stream_resync++;
				rx_chunk_off = rx_chunk_len;
				continue;
			}
			rx_msg_len = h;
			rx_msg_have = 0;
			rx_chunk_off++;
		}

		uint8_t take = (uint8_t)(rx_msg_len - rx_msg_have);
		uint8_t avail = (uint8_t)(rx_chunk_len - rx_chunk_off);

		if (take > avail) take = avail;
		memcpy(&rx_msg[rx_msg_have], &area[rx_chunk_off], take);
		rx_msg_have = (uint8_t)(rx_msg_have + take);
		rx_chunk_off = (uint8_t)(rx_chunk_off + take);

		if (rx_msg_have == rx_msg_len){  //whole message received
			memcpy(data, rx_msg, rx_msg_len);
			*len = rx_msg_len;
			rx_msg_len = rx_msg_have = 0;
			stats.rx_ok++;
			return true;
		}
	}
}

