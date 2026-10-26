#ifndef NRF24_H
#define NRF24_H

#include <stdint.h>
#include <stdbool.h>

#define NRF24_PAYLOAD 32u	//32 byte payload

//Commands (datasheet)

#define CMD_R_REG 0x00	//000A AAAA
#define CMD_W_REG 0x20	//001A AAAA
#define CMD_R_RX_PL 0x61 //Read RX payload (Incoming)
#define CMD_W_TX_PL 0xA0 //Write TX Payload (Outgoing)
#define CMD_FLUSH_TX 0xE1 //Flush TX FIFO, used in TX mode

//Should not be used during transmission of ACK (ACK will be incomplete)
#define CMD_FLUSH_RX 0xE2 //Flush RX FIFO, used in RX mode

//TX payload reuse must not be activated or deactivated during package transmission
#define CMD_REUSE_TX_PL 0xE3 //Used for PTX device, Reuse last transmitted payload
#define CMD_R_RX_PL_WID 0x60

//Write payload to be transmitted together with ACK packet on PIPE PPP
//Maximum three ACK packet payloads can be pending
#define CMD_W_ACK_PL 0xA8 //1010 1PPP Used in RX MODE
#define CMD_TX_PL_NOACK 0xB0 //Disables autoack on this specfic packet
#define CMD_NOP 0xFF //No operation. Could be used to read STATUS register



//Registers (goes with CMD_R_REG and CMD_W_REG)
#define CONFIG_REG 0x00	//Cifugration Register
#define EN_AA_REG 0x01 //Enable Auto ACK
#define EN_RXADDR_REG 0x02 //Enable RX Addresses
#define SETUP_AW_REG 0x03 //Setup address Widths
#define SETUP_RETR_REG 0x04 //Setup of Auto Retransmission
#define RF_CH_REG 0x05 //RF Channel
#define RF_SETUP_REG 0x06 //RF Setup Register
#define STATUS_REG 0x07 //Status register
#define OBSERVE_TX_REG 0x08 //Transmit observe register
#define RPD_REG 0x09 //Received Power Detector
#define RX_ADDR_P0_REG 0x0A //Received address data pipe 0(5 bytes max)
#define RX_ADDR_P1_REG 0x0B //Received address data pipe 1 ^^
#define RX_ADDR_P2_REG 0x0C //Received address data pipe 2 ^^
#define RX_ADDR_P3_REG 0x0D //Received address data pipe 3 ^^
#define RX_ADDR_P4_REG 0x0E //Received address data pipe 4 ^^
#define RX_ADDR_P5_REG 0x0F //Received address data pipe 5 ^^
#define TX_ADDR_REG 0x10 //Transmit Address. Used for PTX device only
#define RX_PW_P0_REG 0x11 //Number of bytes in RX payload in data pipe 0 (1 to 32 bytes)
#define RX_PW_P1_REG 0x12 //Number of bytes in RX payload in data pipe 1  ^^
#define RX_PW_P2_REG 0x13 //Number of bytes in RX payload in data pipe 2  ^^
#define RX_PW_P3_REG 0x14 //Number of bytes in RX payload in data pipe 3  ^^
#define RX_PW_P4_REG 0x15 //Number of bytes in RX payload in data pipe 4  ^^
#define RX_PW_P5_REG 0x16 //Number of bytes in RX payload in data pipe 5  ^^
#define FIFO_STATUS_REG 0x17 //FIFO status register (capacity status for TX RX regs)
#define DYNPD_REG 0x1C //Enable dynamic payload length
#define FEATURE_REG 0x1D //Feature register


// CONFIG BIT SHIFT SETTINGS
#define CONFIG_PRIM_SHIFT (1u << 0)
#define CONFIG_PWR_SHIFT (1u << 1)
#define CONFIG_EN_CRC_SHIFT (1u << 3)
#define STATUS_MAX_RT_SHIFT (1u << 4)
#define STATUS_TX_DS_SHIFT (1u << 5)
#define STATUS_RX_DR_SHIFT (1u << 6)
#define FIFO_RX_EMPTY (1u << 0)


typedef enum { NRF_OK = 0, NRF_TIMEOUT, NRF_ERR } nrf_result_t;

typedef struct {
	uint8_t channel;	//0 to 125 channels, 2400 mhz + ch mhz
	uint8_t rf_setup;	//data rate + PA level
	uint8_t addr[5];	//shared pipe 0 / TX address
} nrf24_cfg_t;

nrf_result_t nrf24_init(const nrf24_cfg_t *cfg);

//Load one frame, pulse CE, and wait for TX_DS. Leaves radio on Stanby
nrf_result_t nrf24_send(const uint8_t frame[NRF24_PAYLOAD], uint32_t timeout_ms);

//Enter RX mode (CE high). Takes around 130 us before radio actuall hears
void nrf24_listen(void);

// Non blocking: if a frame is waiting, copy it out and return true
bool nrf24_poll_rx(uint8_t frame[NRF24_PAYLOAD]);

//debug helpers
uint8_t nrf24_read_reg(uint8_t reg);
uint8_t nrf24_status(void);
bool nrf24_verify_config(const nrf24_cfg_t *cfg);

#endif



