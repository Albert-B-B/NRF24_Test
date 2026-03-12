#include "nrf24.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include <stdint.h>

//Initialize the nrf24l01 module
void nrf24_Init(void){
  //Set CE and CSN pins as output
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = NRF24_CE_Pin | NRF24_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF24_CE_GPIO_Port, &GPIO_InitStruct);
  
  //Set CE and CSN pins to default state
  HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_RESET); //CE low
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_SET); //CSN high
  
  HAL_Delay(100); // Wait for NRF24L01 to power up (Tpwr_on = 100 ms max)

  // Clear any stale IRQ flags before touching CONFIG (avoids SPI status corruption on clones)
  nrf24_WriteRegister(0x07, 0x70);

  // Configure CONFIG register: Power up, TX mode, 2-byte CRC enabled
  // PWR_UP=1(bit1), EN_CRC=1(bit3), CRCO=1(bit2), PRIM_RX=0(bit0) -> 0x0E
  nrf24_WriteRegister(0x00, 0x0E);

  HAL_Delay(2); // Wait Tpd2stby (1.5 ms min) for transition from power-down to standby-I
  
  // Set TX address (5 bytes)
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_RESET);
  uint8_t cmd = 0x30; // W_REGISTER + TX_ADDR (0x10)
  HAL_SPI_Transmit(&nrf24_SPI, &cmd, 1, HAL_MAX_DELAY);
  uint8_t tx_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}; // Default address
  HAL_SPI_Transmit(&nrf24_SPI, tx_addr, 5, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_SET);
  
  // Set RX address for pipe 0 (needed for auto-ack)
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_RESET);
  cmd = 0x2A; // W_REGISTER + RX_ADDR_P0 (0x0A)
  HAL_SPI_Transmit(&nrf24_SPI, &cmd, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&nrf24_SPI, tx_addr, 5, HAL_MAX_DELAY); // Same as TX address
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_SET);
  
  // Flush TX FIFO
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_RESET);
  cmd = 0xE1; // FLUSH_TX
  HAL_SPI_Transmit(&nrf24_SPI, &cmd, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_SET);
  
  //Configure the nrf24l01 settings (e.g., channel, data rate, power level, etc.) as needed
  nrf24_SetChannel(76); //Set the RF channel to 76 (2.476GHz an arbitrary choice for testing) 
  nrf24_SetDataRate(0); //Set the data rate to 1Mbps
  nrf24_SetPowerLevel(3); //Set the power level to maximum (0dBm)
  
  HAL_Delay(2); // Wait for settings to take effect
} 

//Reads register from nrf24l01
uint8_t nrf24_ReadRegister(uint8_t reg){
  uint8_t data;
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_RESET);
  uint8_t command = reg & 0x1F;
  HAL_SPI_Transmit(&nrf24_SPI, &command, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&nrf24_SPI, &data, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_SET);
  return data;
}

//Reads a multi-byte register (e.g. 5-byte addresses) from nrf24l01
void nrf24_ReadRegisterMulti(uint8_t reg, uint8_t* data, uint8_t length){
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_RESET);
  uint8_t command = reg & 0x1F;
  HAL_SPI_Transmit(&nrf24_SPI, &command, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&nrf24_SPI, data, length, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_SET);
}

//Writes data to register of nrf24l01
void nrf24_WriteRegister(uint8_t reg, uint8_t data){
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_RESET); //Pull CSN low to start communication
  uint8_t command = (reg & 0x1F) | 0x20; //The first 5 bits of the command are the register address, the last 3 bits are 1 for write operation
  HAL_SPI_Transmit(&nrf24_SPI, &command, 1, HAL_MAX_DELAY); //Send the command
  HAL_SPI_Transmit(&nrf24_SPI, &data, 1, HAL_MAX_DELAY); //Send the data
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_SET); //Pull CSN high to end communication
}

void nrf24_FlushTX() {
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_RESET); //Pull CSN low to start communication
  uint8_t command = 0xE1; //The command for flushing TX FIFO is 0xE1
  HAL_SPI_Transmit(&nrf24_SPI, &command, 1, HAL_MAX_DELAY); //Send the command
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_SET); //Pull CSN high to end communication
}

void nrf24_ClearIRQFlags() {
  // Clear RX_DR, TX_DS, and MAX_RT flags by writing 1 to them in the STATUS register (0x07)
  nrf24_WriteRegister(0x07, 0x70); // 0x70 = 0b01110000 (sets bits 4, 5, and 6 to clear the flags)
}

//Transmits message stored in data with length of length
void nrf24_Transmit(uint8_t* data, uint8_t length) {
  // Flush TX FIFO before loading new payload
  nrf24_FlushTX();

  // Load payload into TX FIFO (CSN must bracket the SPI transaction)
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_RESET);
  uint8_t command = 0xA0; // W_TX_PAYLOAD
  HAL_SPI_Transmit(&nrf24_SPI, &command, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&nrf24_SPI, data, length, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_SET);

  // Pulse CE HIGH for > 10µs to trigger transmission, then return to standby
  HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_SET);
  HAL_Delay(1); // 1 ms >> 10 µs minimum requirement
  HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_RESET);
}

//Receives message and stores it in data with length of length. Returns true if message is received, false otherwise
bool nrf24_Receive(uint8_t* data, uint8_t length) {
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_RESET); //Pull CSN low to start communication
  uint8_t command = 0x61; //The command for reading payload is 0x61
  HAL_SPI_Transmit(&nrf24_SPI, &command, 1, HAL_MAX_DELAY); //Send the command
  HAL_SPI_Receive(&nrf24_SPI, data, length, HAL_MAX_DELAY); //Read the data
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_SET); //Pull CSN high to end communication
  return true; //Return true if message is received
}

//Sets the RF frequency channel. The channel number is between 0 and 127, which corresponds to frequencies between 2.4GHz and 2.527GHz (1MHz steps)
void nrf24_SetChannel(uint8_t channel) {
  nrf24_WriteRegister(0x05, channel & 0x7F); //The channel is set by writing to the RF_CH register (0x05). The channel number is between 0 and 127, which corresponds to frequencies between 2.4GHz and 2.527GHz
}

void nrf24_SetRetransmission(uint8_t delay, uint8_t count) {
  uint8_t value = ((delay & 0x0F) << 4) | (count & 0x0F); //The retransmission settings are configured by writing to the SETUP_RETR register (0x04). The delay is between 0 and 15 (in multiples of 250µs), and the count is between 0 and 15 (number of retransmissions)
  nrf24_WriteRegister(0x04, value); //Write the value to the SETUP_RETR register
}

//Clears received data from the RX FIFO by flushing it
void nrf24_FlushRX() {
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_RESET); //Pull CSN low to start communication
  uint8_t command = 0xE2; //The command for flushing RX FIFO is 0xE2
  HAL_SPI_Transmit(&nrf24_SPI, &command, 1, HAL_MAX_DELAY); //Send the command
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_SET); //Pull CSN high to end communication
}

//Sets the nrf24l01 to RX mode if rx is true, and to TX mode if rx is false
void nrf24_SetRXTXMode(bool rx)  {
  uint8_t value = nrf24_ReadRegister(0x00); //Read the current value of the CONFIG register (0x00)
  if (rx) {
    value |= 0x01; //Set the PRIM_RX bit (bit 0) for RX mode
    nrf24_WriteRegister(0x00, value); //Write CONFIG first so PRIM_RX=1 before CE goes high
    HAL_Delay(2); //Wait Tpd2stby (1.5 ms min) for the module to reach standby-I
    HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_SET); //CE high -> enter RX mode
  } else {
    HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_RESET); //CE low -> leave RX/TX mode
    value &= 0xFE; //Clear the PRIM_RX bit (bit 0) for TX mode
    nrf24_WriteRegister(0x00, value); //Write the modified value back to the CONFIG register
  }
}

//0 for 1Mbps, 1 for 2Mbps
void nrf24_SetDataRate(uint8_t dataRate) {
  uint8_t value = nrf24_ReadRegister(0x06); //Read the current value of the RF_SETUP register (0x06)
  value &= 0xD7; //Clear the RF_DR bits (bits 3 and 5) in the value
  if (dataRate == 1) {
    value |= 0x08; //Set the RF_DR bit (bit 3) for 2Mbps data rate
    //Clear the interrupts to prevent them from being triggered by the change in data rate
    nrf24_ClearIRQFlags();
  }
  nrf24_WriteRegister(0x06, value); //Write the modified value back to the RF_SETUP register
}

 //0 for -18dBm, 1 for -12dBm, 2 for -6dBm, 3 for 0dBm (The output amplifier has a gain of 20dB)
void nrf24_SetPowerLevel(uint8_t powerLevel) {
  uint8_t value = nrf24_ReadRegister(0x06); //Read the current value of the RF_SETUP register (0x06)
  value &= 0xF9; //Clear the RF_PWR bits (bits 1 and 2) in the value
  value |= (powerLevel & 0x03) << 1; //Set the RF_PWR bits according to the power level (0 for -18dBm, 1 for -12dBm, 2 for -6dBm, 3 for 0dBm)
  nrf24_WriteRegister(0x06, value); //Write the modified value back to the RF_SETUP register
}
//Returns true if data is ready to be read, false otherwise
bool nrf24_dataReady(void) {
  return nrf24_ReadRegister(7) & 0x40; //The data ready status is indicated by the RX_DR bit (bit 6) in the STATUS register (0x07). If this bit is set, it means that there is data ready to be read from the RX FIFO
} 
