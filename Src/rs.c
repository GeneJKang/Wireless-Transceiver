#include "rs.h"
#include <string.h>

#define RS_MAX_PARITY 4u //supports up to 4 parity bytes (2 fixed bytes)

static uint8_t gf_exp[512]; //gf_exp = 2^i (doubled so a + b never needs mod 255)
static uint8_t gf_log[256];	//gf_log[x] = i where 2^i = x (do not use gf_log[0])
static bool tables_ready;

//RS over GF(256)
//Primative Polnomial: x^8 + x^4 + x^3 + x^2 + 1 = 0x11D generator 2

//buf[0] is the highest power of x and buf[len - 1] is the lowest power



//Builds the exp/log tables. Multiplying in GF(256) is just adding logarithms
static void gf_init(void){
	uint16_t x = 1;
	for (uint16_t i = 0; i < 255; i++){
		gf_exp[i] = (uint8_t)x;
		gf_log[x] = (uint8_t)i;
		x <<= 1;	//multiply by the generator (2)
		if (x & 0x100) x ^= 0x11D; //reduce by the primative polynomial
	}
	for (uint16_t i = 255; i < 512; i++){
		gf_exp[i] = gf_exp[i - 255];
	}
	tables_ready = true;
}


//Multiplies two field elements.
//Adding in GF(256) is XOR
static uint8_t gf_mul(uint8_t a, uint8_t b){
	if (a == 0 || b == 0) return 0;
	return gf_exp[gf_log[a] + gf_log[b]];
}

//Divides a by b (b must not be 0)
static uint8_t gf_div(uint8_t a, uint8_t b){
	if (a == 0) return 0;
	return gf_exp[gf_log[a] + 255 - gf_log[b]];
}

//Returns 2^power which power taken mod 255
static uint8_t gf_pow2(int power){
	power %= 255;
	if (power < 0) power += 255;
	return gf_exp[power];
}

//Builds the generator polynomail g(x) = (x-1)(x - 2)( x - 2^2) ... (x - 2^(np-1))
//highest power (g[0]) goes first so g[0] i always 1
//Every valid codeword is a multiple of g(x)
static void gen_poly(uint8_t np, uint8_t g[RS_MAX_PARITY + 1]){
	memset(g, 0, RS_MAX_PARITY + 1);
	g[0] = 1;
	for (uint8_t i = 0; i < np; i++){
		uint8_t root = gf_pow2(i);
		//multiply the current g(x0 by (x + root)
		for (int j = i + 1; j > 0; j--){
			g[j] ^= gf_mul(g[j - 1], root);
		}
	}
}


//Computes the parity bytes for buf[0 ... msg_len-1] and writes them to
//buf[msg_len ... msg_len + parity_len - 1].
//Does parity long division

void rs_encode(uint8_t *buf, uint8_t msg_len, uint8_t parity_len){
	if (parity_len == 0) return;
	if (parity_len > RS_MAX_PARITY) return;
	if (!tables_ready) gf_init();

	uint8_t g[RS_MAX_PARITY + 1];
	uint8_t rem[RS_MAX_PARITY] = {0};
	gen_poly(parity_len, g);

	for (uint8_t i = 0; i < msg_len; i++){
		uint8_t fb = buf[i] ^ rem[0]; //byte leaving the register
		for (uint8_t j = 0; j + 1 < parity_len; j++){
			rem[j] = rem[j + 1] ^ gf_mul(fb, g[j + 1]);
		}
		rem[parity_len - 1] = gf_mul(fb, g[parity_len]);
	}
	memcpy(&buf[msg_len], rem, parity_len);
}


//Evaluates the received frame as a polynomial at x = 2^j for each j
//A good codeword is a multiple of g(x) so every syndrome is 0
//nonzero syndromes mean at least one byte is wrong
static bool syndromes(const uint8_t *buf, uint8_t len, uint8_t np, uint8_t s[RS_MAX_PARITY]){
	bool any = false;
	for (uint8_t j = 0; j < np; j++){
		uint8_t x = gf_pow2(j);
		uint8_t	v = 0;
		for (uint8_t i = 0; i < len; i++){
			v = gf_mul(v, x) ^ buf[i];	//Horner's rule
		}
		s[j] = v;
		if (v) any = true;	//at least one byte is wrong if true
	}
	return any;
}


//Repairs buf in place and returns how many bytes it fixed (0 if the frame was good)
// -1 if there are more errors than it can fix
//This uses syndromes to check if its good or not
// Then it uses Berlekamp-Massey to find the number of bad bytes
// Chien finds which bytes are bad, and Forney finds what the original value was
int rs_decode(uint8_t *buf, uint8_t len, uint8_t parity_len){
	if (parity_len == 0) return 0;
	if (parity_len > RS_MAX_PARITY || len <= parity_len) return -1;
	if (!tables_ready) gf_init();

	uint8_t np = parity_len;
	uint8_t s[RS_MAX_PARITY];
	if (!syndromes(buf, len, np, s)) return 0; //frame is good

	//Berlekamp-Massey (find how many bad bytes)
	uint8_t lam[RS_MAX_PARITY + 2] = {1}; //error locator L(x) (starts at 1)
	uint8_t prev[RS_MAX_PARITY + 2] = {1}; //copy of L(x) from the last length change
	uint8_t nerr = 0; //current degree of L(x) (number of bad bytes)
	uint8_t shift = 1; //how far prev(x) has shifted
	uint8_t prev_d = 1; //discrepancy when prev was saved

	for (uint8_t n = 0; n < np; n++){
		//Check how far L(x) is from predicted syndrome n
		uint8_t d = s[n];
		for (uint8_t i = 1; i <= nerr; i++){
			d ^= gf_mul(lam[i], s[n - i]);
		}

		if (d == 0){
			shift++;
			continue;
		}
		uint8_t coef = gf_div(d, prev_d);
		uint8_t saved[RS_MAX_PARITY + 2];
		memcpy(saved, lam, sizeof saved);

		//L(x) -= (d / prev_d) * x^shift * prev(x)
		for (uint8_t i = 0; i + shift < sizeof lam; i++){
			lam[i + shift] ^= gf_mul(coef, prev[i]);
		}

		if (2 * nerr <= n){ //locator needs to grow
			nerr = (uint8_t)(n + 1 - nerr);
			memcpy(prev, saved, sizeof prev);
			prev_d = d;
			shift = 1;
		} else {
			shift++;
		}
	}

	if (2 * nerr > np) return -1; //more errors than the parity can fix

	//Chien search
	//byte buf[i] is the coefficient of x^(len-1-i) so its locator
	//is x = 2^(len-1-i). The byte is bad if L(1/x) = 0;

	uint8_t pos[RS_MAX_PARITY];
	uint8_t found = 0;
	for (uint8_t i = 0; i < len; i++){
		int power = len - 1 - i;
		uint8_t xinv = gf_pow2(-power);
		uint8_t v = 0;
		uint8_t xp = 1;

		for (uint8_t k = 0; k <= nerr; k++){ //evaluate L(1/x)
			v ^= gf_mul(lam[k], xp);
			xp = gf_mul(xp, xinv);
		}
		if (v == 0){
			if (found == RS_MAX_PARITY) return -1;
			pos[found++] = i;
		}
	}
	if (found != nerr) return -1; //roots are outside the frame (too many errors)

	//Forney
	//Evaluate error and find original value
	// W(x) = S(x) * L(x) mod x^np
	uint8_t om[RS_MAX_PARITY] = {0};
	for (uint8_t i = 0; i < np; i++){
		for (uint8_t k = 0; k <= i && k <= nerr; k++){
			om[i] ^= gf_mul(s[i - k], lam[k]);
		}
	}

	for (uint8_t e = 0; e < found; e ++){
		int power = len - 1 - pos[e];
		uint8_t x = gf_pow2(power);
		uint8_t xinv = gf_pow2(-power);

		//W(1/x)
		uint8_t num = 0;
		uint8_t xp = 1;
		for(uint8_t i = 0; i < np; i++){
			num ^= gf_mul(om[i], xp);
			xp = gf_mul(xp, xinv);
		}

		//L'(1/x) is the formal derivatie that only keeps the odd-power terms
		uint8_t den = 0;
		xp = 1;
		for(uint8_t k = 1; k <= nerr; k++){
			if (k & 1) den ^= gf_mul(lam[k], xp);
			xp = gf_mul(xp, xinv);
		}
		if (den == 0) return -1;

		//error value = x * W(1/x) / L'(1/x) (first root of g is 2^0)
		buf[pos[e]] ^= gf_mul(x, gf_div(num, den));
	}

	//Check if its a correct repair which leaves every syndrome at 0
	if (syndromes(buf, len, np, s)) return -1;
	return found;
}
