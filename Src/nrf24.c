
//Includes
#include "nrf24.h"
#include "port.h"
#include <string.h>


//Low level helper functions

//The only function that touches the SPI line
static uint8_t nrf_cmd(uint8_t cmd, const uint8_t *out, uint8_t *in, uint8_t len){

	uint8_t tx[1 + NRF24_PAYLOAD];	//Local buffers of 33 bytes, 1 cmd + 32 data
	uint8_t rx[1 + NRF24_PAYLOAD];

	tx[0] = cmd;	//Gets the cmd and puts it into tx cmd byte
	if (out){		//Takes the out pointer (data we will transmit)
		memcpy(&tx[1], out, len);	//copies data into tx (transmit) buffer with the cmd byte
	} else{
		memset(&tx[1], CMD_NOP, len);	//send filler (0xFF) with cmd byte
	}

	port_csn(false);	//Select nRF board
	port_spi_xfer(tx, rx, (uint16_t) (1u + len));	//send frame to nRF24
	port_csn(true);		//Deselect nRF board

	if (in){		//Takes the in pointer (data we received)
		memcpy(in, &rx[1], len);	//copy received len bytes data into in ptr
	}

	return rx[0];	//return STATUS byte


}

//write into register
static void write_reg(uint8_t reg, uint8_t val){
	nrf_cmd((uint8_t) (CMD_W_REG | reg), &val, NULL, 1);
}

static void clear_flags(uint8_t mask){
	write_reg(STATUS_REG, mask);	//STATUS flags are cleared by writing 1 into them
}

//Read register and return v (register data)
uint8_t nrf24_read_reg(uint8_t reg){
	uint8_t v;
	nrf_cmd((uint8_t)(CMD_R_REG | reg), NULL, &v, 1);
	return v;
}

//Send NOP command and return status byte
uint8_t nrf24_status(void){
	return nrf_cmd(CMD_NOP, NULL, NULL, 0);
}

static void write_reg_n(uint8_t reg, const uint8_t *buf, uint8_t len){
	nrf_cmd((uint8_t) (CMD_W_REG | reg), buf, NULL, len);
}



bool nrf24_verify_confg(const nrf24_cfg_t *cfg){
	//Read everything after init. If this fails then something went wrong

	uint8_t a[5];
	bool ok = true;

	ok = ok & (nrf24_read_reg(EN_AA_REG) == 0x00);
	ok = ok & (nrf24_read_reg(SETUP_AW_REG) == 0x03);
	ok = ok & (nrf24_read_reg(RF_CH_REG) == cfg->channel);
	ok = ok & (nrf24_read_reg(RF_SETUP_REG) == cfg->rf_setup);
	ok = ok & (nrf24_read_reg(RX_PW_P0_REG) == NRF24_PAYLOAD);

	nrf_cmd(CMD_R_REG | TX_ADDR_REG, NULL, a, 5);
	ok = ok & (memcmp(a, cfg->addr, 5) == 0);

	return ok;
}


//Setup

//RF_SETUP Layout (in bits)
	//7: CONT_WAVE: Enable (1) continuous carrier wave (RF Testing)
	//6: Reserved: (default 0)
	//5: RF_DR_LOW: Air Data Rate (low  bit), Combined with Bit 3 for data rate selection
	//4: PLL_LOCK: Force PLL lock signal (only used for test)
	//3: RF_DR_HIGH: Air Data Rate (high bit) Combined with bit 5
	//2:1: RFPWR: RF Output Power Selection
	//0: LNA_HCURR: Setup LNA gain, Default 1

//Bits 5, 3
	//0:0, 0:1, 1:0, 1:1 : 1 Mbps, 2 MBps, 250 kpbs, Reserved
//Bits 2, 1
	//0:0, 0:1, 1:0, 1:1 : -18 dBm, -12 dBm, -6 dBm, 0 dBm (highest)


nrf_result_t nrf24_init(const nrf24_cfg_t *cfg){	//cfg is pointer to struct

	port_ce(false);
	port_csn(true);
	port_delay_us(150000);	//150 ms delay as per datasheet on nRF startup

	write_reg(CONFIG_REG, 0x00); //power down, CRC off, PRIM_TX
	write_reg(EN_AA_REG, 0x00);	//No auto ACK on any pipe
	write_reg(SETUP_RETR_REG, 0x00); //no auto retransmit
	write_reg(EN_RXADDR_REG, 0x01);	//Only pipe 0 RX enable
	write_reg(SETUP_AW_REG, 0x03);	//5 byte address width
	write_reg(RF_CH_REG, cfg->channel);	//RF channel / RF setup registers
	write_reg(RF_SETUP_REG, cfg->rf_setup);	//channel and rf_setup are fields inside cfg struct
	write_reg(RX_PW_P0_REG, NRF24_PAYLOAD);	//set number of bytes in RX payload in data pipe 0 to 32 bytes
	write_reg(DYNPD_REG, 0x00); //disable dynamic payload for all pipes
	write_reg(FEATURE_REG, 0x00); //Disable all features
	write_reg_n(RX_ADDR_P0_REG, cfg->addr, 5);	//write into addr for RX data up to 5 bytes
	write_reg_n(TX_ADDR_REG, cfg->addr, 5);	//^^ TX addr instead of RX

	nrf_cmd(CMD_FLUSH_TX, NULL, NULL, 0);	//Flush TX/RX after init
	nrf_cmd(CMD_FLUSH_RX, NULL, NULL, 0);
	clear_flags(STATUS_RX_DR_SHIFT | STATUS_TX_DS_SHIFT | STATUS_MAX_RT_SHIFT);

	write_reg(CONFIG_REG, CONFIG_PWR_SHIFT);	//Standby
	port_delay_us(2000);	//init to standby 1.5 ms

	if (nrf24_verify_config(cfg)){
		return NRF_OK;
	} else{
		return NRF_ERR;
	}

}




//TX
nrf_result_t nrf24_send(const uint8_t frame[NRF24_PAYLOAD], uint32_t timeout_ms){

	port_ce(false);	//Standby
	write_reg(CONFIG_REG, CONFIG_PWR_SHIFT);	//PRIM_RX = 0 (TX), 1 (RX)
	nrf_cmd(CMD_FLUSH_TX, NULL, NULL, 0);	//Flush any stale data in TX buffer
	clear_flags(STATUS_TX_DS_SHIFT | STATUS_MAX_RT_SHIFT);

	nrf_cmd(CMD_W_TX_PL, frame, NULL, NRF24_PAYLOAD);	//write frame into TX for payload

	port_ce(true);		//>= 10 us pulse
	port_delay_us(15);
	port_ce(false);		//back to standby after one frame

	uint32_t t0 = port_millis();
	for(;;){	//infinite loop
		(void)port_irq_take();
		uint8_t status = nrf24_status();

		if (status & STATUS_TX_DS_SHIFT){ 	//Check status of TX DS (Data Sent)
			clear_flags(STATUS_TX_DS_SHIFT);
			return NRF_OK;
		}
		if ((uint32_t)(port_millis() - t0) >= timeout_ms){
			//With auto ack off TX_DS flag should always trigger. If it doesnt
			//then radio is not on or something went wrong with wires
			nrf_cmd(CMD_FLUSH_TX, NULL, NULL, 0);
			return NRF_TIMEOUT;
		}
	}
}

//RX
void nrf24_listen(void){

	port_ce(false);
	write_reg(CONFIG_REG, CONFIG_PWR_SHIFT | CONFIG_PRIM_SHIFT);
	clear_flags(STATUS_RX_DR_SHIFT);
	port_ce(true);
	port_delay_us(130);	//RX Settling

}

bool nrf24_poll_rx(uint8_t frame[NRF24_PAYLOAD]){

	(void) port_irq_take();
	if (nrf24_read_reg(FIFO_STATUS_REG) & FIFO_RX_EMPTY){
		return false;
	}

	nrf_cmd(CMD_R_RX_PL, NULL, frame, NRF24_PAYLOAD);
	clear_flags(STATUS_RX_DR_SHIFT);
	return true;
}
