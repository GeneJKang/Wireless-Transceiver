//Huffman compression with learning tables

#include "huff.h"
#include "crc16.h"
#include "port.h"
#include <string.h>
#include <stddef.h>

#ifndef NODE_LOCAL //for simulator
#define NODE_LOCAL
#endif

#define HUFF_ESC (HUFF_NUM_SYMBOLS - 1u) //index of the escape symbol
#define HUFF_NODES (2u * HUFF_NUM_SYMBOLS -1u) //leaves and internal nodes of a Huffman tree
#define SAVE_MAGIC 0x31465548u //HUF1
#define SAVE_VERSION 1

//Table 0
//ASCII 0x20 to 07E
static const uint8_t huff_table0[HUFF_NUM_SYMBOLS] = {
	 3, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,  5, 14, 14,	//sp ! " # $ % & ' ( ) * + , - . /
	 5,  4,  4,  4,  4,  5,  5,  5,  5,  5, 14, 14, 14, 14, 14, 14,	//0 1 2 3 4 5 6 7 8 9 : ; < = > ?
	14,  5,  7,  7,  5,  6,  5,  5,  7,  7, 14,  6,  6,  8,  6,  4,	//@ A B C D E F G H I J K L M N O
	 6, 14,  5,  6,  5,  7, 14,  6, 14, 14, 14, 14, 14, 14, 14, 14,	//P Q R S T U V W X Y Z [ bs ] ^ _
	14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,	//` a b c d e f g h i j k l m n o
	14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,	//p q r s t u v w x y z { | } ~ ESC
};

//build_code() output
typedef struct{
	//each symbols code and length (encoding)
	uint16_t code[HUFF_NUM_SYMBOLS];
	uint8_t len[HUFF_NUM_SYMBOLS];
	//symbols sorted by code
	uint8_t sorted[HUFF_NUM_SYMBOLS];
	uint16_t first_code[HUFF_MAX_BITS + 1u];
	uint8_t count[HUFF_MAX_BITS + 1u];
	uint8_t first_index[HUFF_MAX_BITS + 1u];
} huff_codes_t;


//Appends codes to out (bit writer)
typedef struct {
	uint8_t *out;
	uint16_t bits; //bits written so far
	uint16_t max_bits; //max bits can be written (stop)
}bit_writer_t;

typedef struct {
	uint32_t magic;
	uint8_t version;
	uint8_t nsym;
	uint8_t packed[HUFF_TABLE_BYTES]; //code lengths (2 per byte)
	uint16_t crc; //crc16_ccitt over everything above
} huff_saved_t;

static NODE_LOCAL huff_codes_t t0; //table 0

//sender
static NODE_LOCAL huff_codes_t tx_dyn; //dynamic table
static NODE_LOCAL uint8_t tx_dyn_len[HUFF_NUM_SYMBOLS]; //the length for the tx dynamic table
static NODE_LOCAL uint8_t tx_id; //table in use
static NODE_LOCAL uint16_t counts[HUFF_NUM_SYMBOLS]; //characters sent recently
static NODE_LOCAL uint16_t since_retune, since_announce; //messages since last check
static NODE_LOCAL uint32_t count_total;
static NODE_LOCAL bool announce_pending; //receiver hasnt been sent the current table yet
static NODE_LOCAL bool learning;
static NODE_LOCAL bool saved_current; //current table in flash

//Receiver
static NODE_LOCAL huff_codes_t rx_dyn; //last table the sender sent
static NODE_LOCAL uint8_t rx_id; //the rx table id

static NODE_LOCAL huff_stats_t st;

//Huffman tree workspace for make_lengths()
static NODE_LOCAL uint32_t tree_w[HUFF_NODES];
static NODE_LOCAL int16_t tree_parent[HUFF_NODES];
static NODE_LOCAL bool tree_used[HUFF_NODES];

//Which symbol a byte is (search the list)
static uint8_t symbol_of(uint8_t c){
	return (c >= 0x20u && c <= 0x7Eu) ? (uint8_t)(c - 0x20u) : (uint8_t)HUFF_ESC;
}


//Check that a list of lengths is a usable table
static bool lengths_ok(const uint8_t *len){
	uint32_t kraft = 0;
	uint8_t longest = 0;
	for (uint8_t s = 0; s < HUFF_NUM_SYMBOLS; s++){
		if (len[s] == 0 || len[s] > HUFF_MAX_BITS) return false;
		kraft += (uint32_t)1u << (HUFF_MAX_BITS - len[s]);
		if (len[s] > longest) longest = len[s];
	}
	return kraft == ((uint32_t)1u << HUFF_MAX_BITS) && longest >= 8u;
}


//Builds the table from a list of lengths
//Shortest first
static void build_codes(const uint8_t *lengths, huff_codes_t *t){
	memset(t, 0, sizeof *t);
	for (uint8_t s = 0; s < HUFF_NUM_SYMBOLS; s++) t->count[lengths[s]]++;

	uint16_t code = 0;
	uint8_t index = 0;
	for (uint8_t list = 1; list <= HUFF_MAX_BITS; list++){
		code = (uint16_t)(code << 1);
		t->first_code[list] = code;
		t->first_index[list] = index;
		for (uint8_t s = 0; s < HUFF_NUM_SYMBOLS; s++){

			if (lengths[s] != list) continue;

			t->code[s] = code++;
			t->len[s] = list;
			t->sorted[index++] = s;
		}
	}
}


//Calculates the best code lengths for the counted characters (Huffman tree)
//If the tree is deaper than 15 bits the counts are scaled down and is built again
static bool make_lengths(const uint16_t *cnt, uint8_t *len){
	for (uint8_t shift = 0; shift < 16u; shift++){
		uint16_t n = HUFF_NUM_SYMBOLS;
		for (uint16_t i = 0; i < HUFF_NODES; i++){
			tree_w[i] = (i < HUFF_NUM_SYMBOLS) ? (uint32_t)(cnt[i] >> shift) + 1u : 0u;
			tree_parent[i] = -1;
			tree_used[i] = false;
		}
		//join the two lightest unjoined nodes until one tree is left
		while (n < HUFF_NODES){
			int16_t a = -1, b = -1;
			for (uint16_t i = 0; i < n; i++){

				if (tree_used[i]) continue;

				if (a < 0 || tree_w[i] < tree_w[a])
				{
					b = a; a = (int16_t)i;
				} else if (b < 0 || tree_w[i] < tree_w[b]) {
					b = (int16_t)i;
				}
			}
			tree_w[n] = tree_w[a] + tree_w[b];
			tree_parent[a] = tree_parent[b] = (int16_t)n;
			tree_used[a] = tree_used[b] = true;
			n++;
		}

		//Symbols code length is the depth of the leaf
		uint8_t longest = 0;
		for (uint8_t s = 0; s < HUFF_NUM_SYMBOLS; s++){
			uint8_t d = 0;
			for (int16_t j = (int16_t)s; tree_parent[j] >= 0; j = tree_parent[j]){
				d++;
			}
			len[s] = d;

			if (d > longest) longest = d;

		}
		if (longest <= HUFF_MAX_BITS) return lengths_ok(len);
	}
	return false;
}


//Bits the counted text takes with the current length (escape bytes also carry the 8 original bits)
//Used to compare old table with new one
static uint32_t cost_bits(const uint8_t *len){
	uint32_t bits = 0;
	for (uint8_t s = 0; s < HUFF_NUM_SYMBOLS; s++)
		bits += (uint32_t)counts[s] * (uint32_t)(len[s] + (s == HUFF_ESC ? 8u : 0u));
	return bits;
}

//gets the packed lengths
static void pack_lengths(const uint8_t *len, uint8_t *out){
	for (uint8_t i = 0; i < HUFF_TABLE_BYTES; i++)
		out[i] = (uint8_t)((len[2u * i] << 4) | (len[2u * i + 1u] & 0x0Fu));
}

//gets the unpacked lengths
static void unpack_lengths(const uint8_t *in, uint8_t *len){
	for (uint8_t i = 0; i < HUFF_TABLE_BYTES; i++){
		len[2u * i] = (uint8_t)(in[i] >> 4);
		len[2u * i + 1u] = (uint8_t)(in[i] & 0x0Fu);
	}
}


//Starts the sender on a new dynamic table
static void use_table(const uint8_t *len){
	memcpy(tx_dyn_len, len, HUFF_NUM_SYMBOLS);
	build_codes(tx_dyn_len, &tx_dyn);
	tx_id = (uint8_t)(tx_id % 15u + 1u);
	announce_pending = true;
	saved_current = false;
}


//builds the best dynamic table with the current character frequencies
static void retune(void){
	st.retune_checks++;

	if (count_total < HUFF_RETUNE_MIN_CHARS) return; //too little character frequencies to retune

	uint8_t len[HUFF_NUM_SYMBOLS];
	if (make_lengths(counts, len)){
		uint32_t now = cost_bits(tx_id ? tx_dyn_len : huff_table0);
		uint32_t next = cost_bits(len);

		if ((uint64_t)next * 100u <= (uint64_t)now * HUFF_RETUNE_GAIN_PCT){
			use_table(len);
			st.retunes++;

		}
	}
	for (uint8_t s = 0; s < HUFF_NUM_SYMBOLS; s++) counts[s] >>= 1;
	count_total >>= 1;
}


//Adds a message's characters to the counts
static void count_text(const uint8_t *text, uint8_t len){
	for (uint8_t i = 0; i < len; i++){
		uint8_t s = symbol_of(text[i]);
		if (counts[s] == UINT16_MAX){	//prevents overflow by halving all counts
			for (uint8_t k = 0; k < HUFF_NUM_SYMBOLS; k++) counts[k] >>= 1;
			count_total >>= 1;
		}
		counts[s]++;
		count_total++;
	}
}


//Queues a table message (table values) for the sender's current table
static link_result_t send_table(void){
	uint8_t msg[1u + HUFF_TABLE_BYTES];
	msg[0] = (uint8_t)(HUFF_HDR_TABLE | tx_id);
	pack_lengths(tx_dyn_len, &msg[1]);
	link_result_t r = link_send(msg, sizeof msg);

	if (r == LINK_OK) st.tables_sent++;

	return r;
}


//Checks a received table frame and keeps it as the table for that id
//A bad message is dropped
static void take_table(const uint8_t *msg, uint8_t n){
	uint8_t id = (uint8_t)(msg[0] & 0x0Fu);
	uint8_t len[HUFF_NUM_SYMBOLS];

	if (n != 1u + HUFF_TABLE_BYTES || id == 0){
		st.bad_msgs++;
		return;
	}

	unpack_lengths(&msg[1], len);

	if (!lengths_ok(len)){
		st.bad_msgs++;
		return;
	}

	build_codes(len, &rx_dyn);
	rx_id = id;
	st.tables_received++;
}


//initializes huffman encoding
void huff_init(void){
	build_codes(huff_table0, &t0);
	memset(counts, 0, sizeof counts);
	memset(&st, 0, sizeof st);
	count_total = 0;
	since_retune = since_announce = 0;
	tx_id = 0;
	rx_id = 0;
	announce_pending = false;
	learning = true;
	saved_current = false;

	//Starts the table from flash if the flash page holds a valid table
	huff_saved_t rec;
	uint8_t len[HUFF_NUM_SYMBOLS];
	if (port_store_read(&rec, sizeof rec) && rec.magic == SAVE_MAGIC && rec.version == SAVE_VERSION
			&& rec.nsym == HUFF_NUM_SYMBOLS
			&& rec.crc == crc16_ccitt((const uint8_t *)&rec, offsetof(huff_saved_t, crc))){
		unpack_lengths(rec.packed, len);
		if (lengths_ok(len)){
			use_table(len);	//also sends the table to the receiver before the first message
			saved_current = true;
			st.loaded = true;
		}
	}
}

//this sets the mode for dynamic table
void huff_set_learning(bool on){
	learning = on;
}


//append the low n bits of v, return false once the output is full
static bool put_bits(bit_writer_t *w, uint16_t v, uint8_t n){
	if ((uint16_t)(w->bits + n) > w->max_bits) return false;

	for (uint8_t i = n; i-- > 0;){
		uint16_t byte = (uint16_t)(w->bits >> 3);
		uint8_t mask = (uint8_t)(0x80u >> (w->bits & 7u));

		if ((v >> i) & 1u){
			w->out[byte] |= mask;
		} else{
			w->out[byte] &= (uint8_t)~mask;
		}

		w->bits++;
	}
	return true;
}


//encode message using table
uint8_t huff_encode(const uint8_t *text, uint8_t len, uint8_t *out){

	if (len > HUFF_MAX_TEXT) return 0;

	const huff_codes_t *t = tx_id ? &tx_dyn : &t0;

	//Try to compress message (give up if worse than raw message)
	bit_writer_t w = { &out[1], 0, (uint16_t)(len > 0 ? (len - 1u) * 8u : 0u) };
	bool fits = true;
	for (uint8_t i = 0; i < len && fits; i++){
		uint8_t s = symbol_of(text[i]);
		fits = put_bits(&w, t->code[s], t->len[s]);

		if (fits && s == HUFF_ESC) fits = put_bits(&w, text[i], 8);

	}

	if (fits && len > 0){
		uint8_t pad = (uint8_t)((8u - (w.bits & 7u)) & 7u);
		put_bits(&w, 0xFFu, pad); //pad with 1s
		out[0] = (uint8_t)(HUFF_HDR_COMPRESSED | tx_id);
		return (uint8_t)(1u + w.bits / 8u);
	}
	//Better to use raw
	out[0] = HUFF_HDR_RAW;
	memcpy(&out[1], text, len);
	return (uint8_t)(1u + len);
}



//decode huffman message
int huff_decode(const uint8_t *msg, uint8_t len, uint8_t *text, uint8_t text_max){

	if (len == 0) return -1;

	uint8_t hdr = msg[0];
	if (hdr == HUFF_HDR_RAW){

		if ((uint8_t)(len - 1u) > text_max) return -1;

		memcpy(text, &msg[1], len - 1u);
		return len - 1;
	}

	if ((hdr & 0xF0u) != HUFF_HDR_COMPRESSED) return -1;

	uint8_t id = (uint8_t)(hdr & 0x0Fu);
	const huff_codes_t *t;

	if (id == 0){
		t = &t0;
	} else if (id == rx_id){
		t = &rx_dyn;
	} else {
		return -1; //made with a table never received
	}

	uint16_t total = (uint16_t)((len - 1u) * 8u);
	uint16_t pos = 0;
	uint8_t n = 0;
	while(pos < total){
		//Read one bit at a time until the bits are a complete code
		uint16_t code = 0;
		uint8_t list = 0;
		uint16_t start = pos;
		int sym = -1;
		while (pos < total && list < HUFF_MAX_BITS){
			code = (uint16_t)((code << 1) | ((msg[1u + (pos >> 3)] >> (7u - (pos & 7u))) & 1u));
			pos++;
			list++;
			if (t->count[list] && (uint16_t)(code - t->first_code[list]) < t->count[list]){
				sym = t->sorted[t->first_index[list] + (code - t->first_code[list])];
				break;
			}
		}
		if (sym < 0){
			//Ran out of bits partway through a code (thats padding if its all 1)
			if ((uint16_t)(pos - start) > 7u) return -1;

			if (code != (uint16_t)((1u << (pos - start)) - 1u)) return -1;

			break;
		}
		uint8_t c;
		if (sym == (int)HUFF_ESC){	//escape character (8 bits are the byte itself)

			if ((uint16_t)(pos + 8u) > total) return -1;

			c = 0;

			for (uint8_t i = 0; i < 8u; i++, pos++){
				c = (uint8_t)((c << 1) | ((msg[1u + (pos >> 3)] >> (7u - (pos & 7u))) & 1u));
			}
		} else{
			c = (uint8_t)(sym + 0x20);
		}

		if (n >= text_max) return -1;

		text[n++] = c;
	}
	return n;
}


//send huffman message
link_result_t huff_send(const char *text, uint8_t len){
	if (len > HUFF_MAX_TEXT) return LINK_FAIL;

	if (learning){
		count_text((const uint8_t *)text, len);
		if (++since_retune >= HUFF_RETUNE_EVERY){
			since_retune = 0;
			retune();
		}
	}

	//The receiver must have the table before the first message is sent
	if (tx_id != 0 && (announce_pending || since_announce >= HUFF_ANNOUNCE_EVERY)){
		if (send_table() != LINK_OK) return LINK_FAIL;
		announce_pending = false;
		since_announce = 0;
	}
	if (since_announce < UINT16_MAX) since_announce++;

	uint8_t msg[LINK_MAX_DATA];
	uint8_t n = huff_encode((const uint8_t *)text, len, msg);
	return link_send(msg, n);
}


//Hands back the decoded message
bool huff_poll_recv(char *text, uint8_t *len){
	uint8_t msg[LINK_MAX_DATA];
	uint8_t n;
	while (link_poll_recv(msg, &n)){
		if ((msg[0] & 0xF0u) == HUFF_HDR_TABLE){	//message is a new table
			take_table(msg, n);
			continue;
		}
		int r = huff_decode(msg, n, (uint8_t *)text, HUFF_MAX_TEXT);
		if (r >= 0){
			*len = (uint8_t)r;
			return true;
		}
		st.bad_msgs++;	//damaged or made with a table we dont have
	}
	return false;
}

//save huffman table to flash
bool huff_save(void){

	if (tx_id == 0 || saved_current) return true; //nothing new to save

	huff_saved_t rec;
	memset(&rec, 0, sizeof rec);
	rec.magic = SAVE_MAGIC;
	rec.version = SAVE_VERSION;
	rec.nsym = HUFF_NUM_SYMBOLS;
	pack_lengths(tx_dyn_len, rec.packed);
	rec.crc = crc16_ccitt((const uint8_t *)&rec, offsetof(huff_saved_t, crc));

	if (!port_store_write(&rec, sizeof rec)) return false;

	saved_current = true;
	st.saves++;
	return true;
}

huff_stats_t huff_get_stats(void){
	st.tx_table = tx_id;
	st.rx_table = rx_id;
	return st;
}


