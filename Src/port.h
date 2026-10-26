#ifndef PORT_H
#define PORT_H

#include <stdint.h>
#include <stdbool.h>

//One time init of clocks, GPIO, SPI, Timers, IRQ line
void port_init(void);

//Full duplex SPI: clock out tx[i], store what came back in rx[i]
//CSN is not touched here because the driver frames transactions itself
void port_spi_xfer(const uint8_t *tx, uint8_t *rx, uint8_t len);

void port_csn(bool high); //nRF24 chip select, active low
void port_ce(bool high); //nRF24 chip enable

void port_delay_us(uint32_t us);
uint32_t port_millis(void); //free running ms counter

//Return true and clears flag if nRF24 IRQ line fell since last call
bool port_irq_take(void);

//Debug text output
void port_log(const char *s);

#endif
