#include "port.h"
#include "main.h"
#include <string.h>


#define NRF_CSN_PIN GPIO_PIN_12
#define NRF_CSN_PORT GPIOB
#define NRF_CE_PORT GPIOC
#define NRF_CE_PIN GPIO_PIN_7

//flash page storage
#define STORE_ADDR 0x080FF800u
#define STORE_BANK FLASH_BANK_2
#define STORE_PAGE 255u
#define STORE_SIZE 2048u

extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart2;




void port_init(void){
	//CE
	//start low
    __HAL_RCC_GPIOC_CLK_ENABLE();
    HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_RESET);
    GPIO_InitTypeDef g = {0};
    g.Pin = NRF_CE_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(NRF_CE_PORT, &g);



    //us = cycles / (SystemCoreClock / 1e6)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}



void port_spi_xfer(const uint8_t *tx, uint8_t *rx, uint8_t len){

    uint8_t scratch[33];
    if (!rx) rx = scratch; //caller doesn't want the reply
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx, rx, len, 10);

}

void port_csn(bool high){

	HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, high? GPIO_PIN_SET : GPIO_PIN_RESET);

}

void port_ce(bool high){
	HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void port_delay_us(uint32_t us){

	uint32_t start = DWT->CYCCNT;
	uint32_t cycles = us * (SystemCoreClock/1000000u);
	while((uint32_t)(DWT->CYCCNT - start) < cycles){}	//wrap safe

}

uint32_t port_millis(void){
	return HAL_GetTick();	//HAL's 1 ms SysTick counter
}


//Microseconds since boot using the DWT cycle counter that port_init() starts
//CYCCNT counts CPU cycles and wraps every 53s at 80 MHz
uint32_t port_micros(void){
	static uint32_t last_cycle, us_total, leftover;
	uint32_t now = DWT->CYCCNT;
	uint32_t cycle = (uint32_t)(now - last_cycle) + leftover;
	uint32_t per_us = SystemCoreClock / 1000000u; //80 cycles per us at 80 MHz

	last_cycle = now;
	us_total += cycle / per_us;
	leftover = cycle % per_us;
	return us_total;
}


bool port_irq_take(void){
	return false;	//TODO: Set up IRQ pin (IRQ pin not wired)
}

void port_log(const char *s){
	HAL_UART_Transmit(&huart2, (const uint8_t *)s, (uint16_t)strlen(s), 100);
}




//storage read/write

//read from page
bool port_store_read(void *buf, uint16_t len){
	if (len > STORE_SIZE) return false;
	memcpy(buf, (const void *)STORE_ADDR, len);
	return true;
}

//write to page
bool port_store_write(const void *buf, uint16_t len){
	if (len > STORE_SIZE) return false;
	bool ok = HAL_FLASH_Unlock() == HAL_OK;	//this also unlocks the flash
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS); //error flags block programming

	FLASH_EraseInitTypeDef erase = {0};
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.Banks = STORE_BANK;
	erase.Page = STORE_PAGE;
	erase.NbPages = 1;
	uint32_t bad_page;
	if (ok) ok = HAL_FLASHEx_Erase(&erase, &bad_page) == HAL_OK;

	//L4 writes 8 bytes at a time. Leftover is written as 0xFF
	for (uint16_t off = 0; ok && off < len; off = (uint16_t)(off + 8u)){
		uint64_t dw = UINT64_MAX;
		uint16_t left = (uint16_t)(len - off);
		memcpy(&dw, (const uint8_t *)buf + off, left < 8u ? left : 8u);
		ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, STORE_ADDR + off, dw) == HAL_OK;
	}
	HAL_FLASH_Lock(); //lock flash
	return ok && memcmp((const void *)STORE_ADDR, buf, len) == 0; //check if the memory has been written correct
}

