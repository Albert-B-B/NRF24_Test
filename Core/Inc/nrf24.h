#ifndef __NRF24_H__
#define __NRF24_H__

#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "spi.h"

#define nrf24_SPI hspi1 //Which spi handle to use

#define NRF24_CSN_GPIO_Port GPIOA //Which port is connected to CSN pin of nrf24l01
#define NRF24_CSN_Pin GPIO_PIN_4 //Which pin is connected to CSN pin of nrf24l01

#define NRF24_CE_GPIO_Port GPIOA //Which port is connected to CE pin of nrf24l01
#define NRF24_CE_Pin  GPIO_PIN_8 //Which pin is connected to CE pin of nrf24l01

#define NRF24_IRQ_GPIO_Port GPIOA //Which port is connected to IRQ pin of nrf24l01
#define NRF24_IRQ_Pin GPIO_PIN_9 //Which pin is connected to IRQ pin of nrf24l01

void nrf24_Init(void); //Initialize the nrf24l01 module
uint8_t nrf24_ReadRegister(uint8_t reg); //Read register from nrf24l01
void nrf24_ReadRegisterMulti(uint8_t reg, uint8_t* data, uint8_t length); //Read multi-byte register
void nrf24_WriteRegister(uint8_t reg, uint8_t data); //Write data to register of nrf24l01
void nrf24_Transmit(uint8_t* data, uint8_t length); //Transmits message stored in data with length of length
bool nrf24_Receive(uint8_t* data, uint8_t length); //Receives message and stores it in data with length of length. Returns true if message is received, false otherwise
void nrf24_SetChannel(uint8_t channel);
void nrf24_SetDataRate(uint8_t dataRate);
void nrf24_SetPowerLevel(uint8_t powerLevel); //0 for -18dBm, 1 for -12dBm, 2 for -6dBm, 3 for 0dBm (The output amplifier has a gain of 20dB)
void nrf24_SetRetransmission(uint8_t delay, uint8_t count); //Sets the retransmission settings. The delay is between 0 and 15 (in multiples of 250µs), and the count is between 0 and 15 (number of retransmissions)
void nrf24_SetRXTXMode(bool rx); //Sets the nrf24l01 to RX mode if rx is true, and to TX mode if rx is false
bool nrf24_dataReady(void); //Returns true if data is ready to be read, false otherwise
void nrf24_ClearIRQFlags(void); //Clears the IRQ flags by writing 1 to them in the STATUS register
void nrf24_FlushRX(void); //Flushes the RX FIFO
#endif /* __NRF24_H__ */