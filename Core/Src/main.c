/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Drone Receiver - NRF24L01 to UART Bridge
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    uint8_t command;        // Command type
    uint32_t data;          // Command data (32-bit)
} uart_command_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_PAYLOAD_SIZE    32      // Maximum NRF24L01 payload size
#define UART_BUFFER_SIZE    128     // UART output buffer size
#define RF_CHANNEL          76      // RF channel (2.476 GHz)
#define PAYLOAD_SIZE        32      // Expected payload size

// UART Command Protocol
#define UART_CMD_SIZE       5       // Command size: 1 byte cmd + 4 bytes data
#define UART_RX_TIMEOUT     10      // UART receive timeout in ms

// NRF24L01 TX Buffer
#define TX_BUFFER_SIZE      3       // Maximum 3 messages in TX buffer
#define TX_MESSAGE_SIZE     32      // Each message is 32 bytes

typedef struct {
    uint8_t messages[TX_BUFFER_SIZE][TX_MESSAGE_SIZE];  // TX message buffer
    uint8_t count;                                      // Number of messages queued
} tx_buffer_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t message_count = 0;
static uint32_t command_count = 0;
static uint32_t tx_count = 0;
static uint8_t uart_rx_buffer[UART_CMD_SIZE];
static tx_buffer_t tx_buffer = {0};  // TX buffer for outgoing messages
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void NRF24_Configure(void);
static void ProcessReceivedMessage(uint8_t *data, uint8_t length);
static bool UART_ReadCommand(uart_command_t *cmd);
static void ProcessUARTCommand(uart_command_t *cmd);
static void UART_SendRawBytes(const uint8_t *data, uint8_t length);
static bool TX_QueueMessage(uint8_t *data, uint8_t length);
static void TX_SendAllMessages(void);
static void NRF24_SwitchToTX(void);
static void NRF24_SwitchToRX(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Configure NRF24L01 for receiving
 */
static void NRF24_Configure(void)
{
    // Initialize NRF24L01
    nrf24_Init();
    
    // Set RF channel
    nrf24_SetChannel(RF_CHANNEL);
    
    // Set data rate to 1Mbps (0=250kbps, 1=1Mbps, 2=2Mbps)
    nrf24_SetDataRate(1);
    
    // Set power level to maximum (0=-18dBm, 1=-12dBm, 2=-6dBm, 3=0dBm)
    nrf24_SetPowerLevel(3);
    
    // Disable auto-acknowledgment for all pipes
    nrf24_WriteRegister(0x01, 0x00);
    
    // Enable only pipe 0
    nrf24_WriteRegister(0x02, 0x01);
    
    // Set payload width for pipe 0
    nrf24_WriteRegister(0x11, PAYLOAD_SIZE);
    
    // Clear any pending interrupts and flush RX FIFO
    nrf24_ClearIRQFlags();
    nrf24_FlushRX();
    
    // Set to RX mode and start listening
    nrf24_SetRXTXMode(true);
}

/**
 * @brief Process and forward received message over UART using printf
 */
static void ProcessReceivedMessage(uint8_t *data, uint8_t length)
{
    message_count++;
    
    // Send message header and data in hex format
    printf("[MSG_%lu] ", (unsigned long)message_count);
    
    // Print data as hex bytes
    for (uint8_t i = 0; i < length; i++) {
        printf("%02X", data[i]);
        if (i < length - 1) printf(" ");
    }
    
    // Print data as ASCII (if printable)
    printf(" | ");
    for (uint8_t i = 0; i < length; i++) {
        if (data[i] >= 32 && data[i] <= 126) {
            printf("%c", data[i]);
        } else {
            printf(".");
        }
    }
    printf("\r\n");
}

/**
 * @brief Read a complete UART command (non-blocking)
 * @param cmd Pointer to store the parsed command
 * @return true if complete command received, false otherwise
 */
static bool UART_ReadCommand(uart_command_t *cmd)
{
    static uint8_t rx_index = 0;
    static uint32_t last_rx_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Reset buffer if timeout occurred (incomplete command)
    if (rx_index > 0 && (current_time - last_rx_time) > UART_RX_TIMEOUT) {
        rx_index = 0;
        printf("UART: Command timeout, buffer reset\r\n");
    }
    
    // Check if data is available
    while ((USART2->ISR & USART_ISR_RXNE_RXFNE) && rx_index < UART_CMD_SIZE) {
        uart_rx_buffer[rx_index] = (uint8_t)(USART2->RDR);
        rx_index++;
        last_rx_time = current_time;
        
        // Check if we have a complete command
        if (rx_index >= UART_CMD_SIZE) {
            // Parse the command: first byte is command, next 4 bytes are data (little-endian)
            cmd->command = uart_rx_buffer[0];
            cmd->data = ((uint32_t)uart_rx_buffer[4] << 24) |
                       ((uint32_t)uart_rx_buffer[3] << 16) |
                       ((uint32_t)uart_rx_buffer[2] << 8)  |
                       ((uint32_t)uart_rx_buffer[1]);
            
            rx_index = 0;  // Reset for next command
            return true;
        }
    }
    
    return false;  // Command not complete yet
}

  /**
   * @brief Send raw bytes on UART2 without formatting
   * @param data Pointer to bytes to send
   * @param length Number of bytes to send
   */
  static void UART_SendRawBytes(const uint8_t *data, uint8_t length)
  {
    for (uint8_t i = 0; i < length; i++) {
      while (!(USART2->ISR & USART_ISR_TXE_TXFNF)) {
        // Wait until TX register can accept new data
      }
      USART2->TDR = data[i];
    }
  }

/**
 * @brief Process received UART command
 * @param cmd Pointer to the received command
 */
static void ProcessUARTCommand(uart_command_t *cmd)
{
    command_count++;
  uint8_t raw_cmd[UART_CMD_SIZE] = {
    cmd->command,
    (uint8_t)(cmd->data & 0xFF),
    (uint8_t)((cmd->data >> 8) & 0xFF),
    (uint8_t)((cmd->data >> 16) & 0xFF),
    (uint8_t)((cmd->data >> 24) & 0xFF)
  };
    
    printf("[CMD_%lu] Type: 0x%02X, Data: 0x%08lX (%lu)\r\n",
           (unsigned long)command_count,
           cmd->command,
           (unsigned long)cmd->data,
           (unsigned long)cmd->data);
    
    // Handle specific command types
    switch (cmd->command) {
        case 0x10:
            printf(" -> Get telemetry\r\n", (unsigned long)cmd->data);
            break;
        case 0x50:
            printf("  -> Launch\r\n", (unsigned long)cmd->data);
            break;
        case 0x55:
            printf("  -> Servo 1 set to %lu\r\n", (unsigned long)cmd->data);
            break;
        case 0x56:
            printf("  -> Servo 2 set to %lu\r\n", (unsigned long)cmd->data);
            break;
        case 0x57:
            printf("  -> Servo 3 set to %lu\r\n", (unsigned long)cmd->data);
            break;
        case 0x58:
            printf("  -> Servo 4 set to %lu\r\n", (unsigned long)cmd->data);
            break;
        case 0x59:
            printf("  -> Thrust set to %lu\r\n", (unsigned long)cmd->data);
            break;
        default:
            printf("  -> Unknown command type\r\n");
            printf("  -> Raw data: ");
            for (uint8_t i = 0; i < UART_CMD_SIZE; i++) {
                printf("%02X ", raw_cmd[i]);
            }
            printf("\r\n");
            break;
    }

    // Build 32-byte message: [CMD][DATA_0..3][reserved zeros]
    uint8_t tx_message[TX_MESSAGE_SIZE] = {0};
    tx_message[0] = raw_cmd[0];
    tx_message[1] = raw_cmd[1];
    tx_message[2] = raw_cmd[2];
    tx_message[3] = raw_cmd[3];
    tx_message[4] = raw_cmd[4];

    // Queue and immediately transmit to drone
    if (TX_QueueMessage(tx_message, TX_MESSAGE_SIZE)) {
        TX_SendAllMessages();
    } else {
        printf("  -> TX buffer full! Command dropped\r\n");
    }
}

/**
 * @brief Queue a message for transmission
 * @param data   Pointer to data to send
 * @param length Length of data (capped to TX_MESSAGE_SIZE)
 * @return true if queued successfully, false if buffer is full
 */
static bool TX_QueueMessage(uint8_t *data, uint8_t length)
{
    if (tx_buffer.count >= TX_BUFFER_SIZE) {
        return false;
    }
    if (length > TX_MESSAGE_SIZE) {
        length = TX_MESSAGE_SIZE;
    }
    memcpy(tx_buffer.messages[tx_buffer.count], data, length);
    if (length < TX_MESSAGE_SIZE) {
        memset(&tx_buffer.messages[tx_buffer.count][length], 0, TX_MESSAGE_SIZE - length);
    }
    tx_buffer.count++;
    return true;
}

/**
 * @brief Switch NRF24L01 to TX mode
 */
static void NRF24_SwitchToTX(void)
{
    nrf24_SetRXTXMode(false);
    HAL_Delay(1);
}

/**
 * @brief Switch NRF24L01 to RX mode
 */
static void NRF24_SwitchToRX(void)
{
    nrf24_SetRXTXMode(true);
    HAL_Delay(1);
}

/**
 * @brief Send all queued messages via NRF24L01 then return to RX mode
 */
static void TX_SendAllMessages(void)
{
    if (tx_buffer.count == 0) {
        return;
    }

    NRF24_SwitchToTX();

    for (uint8_t i = 0; i < tx_buffer.count; i++) {
        tx_count++;
        printf("[TX_%lu] Sending %d/%d... ", (unsigned long)tx_count, i + 1, tx_buffer.count);

        nrf24_Transmit(tx_buffer.messages[i], TX_MESSAGE_SIZE);
        HAL_Delay(5);

        uint8_t status = nrf24_ReadRegister(0x07);
        if (status & 0x20) {
            printf("OK\r\n");
        } else if (status & 0x10) {
            printf("FAILED (max retries)\r\n");
        } else {
            printf("UNKNOWN\r\n");
        }

        nrf24_ClearIRQFlags();

        if (i < tx_buffer.count - 1) {
            HAL_Delay(10);
        }
    }

    tx_buffer.count = 0;
    NRF24_SwitchToRX();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  
  /* USER CODE BEGIN 2 */

  // Initialize USART2 for printf output (115200, 8N1)
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
  USART2->BRR = (170000000 + (115200 / 2)) / 115200;  // APB1 = 170MHz
  USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

  // Send startup message
  printf("\r\n=== Drone Receiver Started ===\r\n");
  printf("System Clock: %lu MHz\r\n", HAL_RCC_GetSysClockFreq() / 1000000);
  
  // Configure NRF24L01 for receiving
  NRF24_Configure();
  
  // Verify NRF24L01 is responding by reading back the RF channel register (0x05)
  uint8_t channel_reg = nrf24_ReadRegister(0x05);
  if (channel_reg != RF_CHANNEL) {
    printf("ERROR: NRF24L01 not responding! Channel reg=0x%02X, expected 0x%02X. Check wiring.\r\n",
           channel_reg, RF_CHANNEL);
    while(1) {
      HAL_Delay(1000);
    }
  }
  
  printf("NRF24L01 initialized successfully\r\n");
  printf("RF Channel: %d (2.%03d GHz)\r\n", RF_CHANNEL, 400 + RF_CHANNEL);
  printf("Payload Size: %d bytes\r\n", PAYLOAD_SIZE);
  printf("UART Command Format: 1 byte cmd + 4 bytes data\r\n");
  printf("Listening for NRF24L01 messages and UART commands...\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // Priority 1: Check if data is available from NRF24L01
    // Keep reading until FIFO is empty (up to 3 messages can be buffered)
    while (nrf24_dataReady()) {
      uint8_t rx_buffer[MAX_PAYLOAD_SIZE];
      
      // Receive the message
      bool success = nrf24_Receive(rx_buffer, PAYLOAD_SIZE);
      
      if (success) {
        // Process and forward the message over UART
        ProcessReceivedMessage(rx_buffer, PAYLOAD_SIZE);
      }
      
      // Clear interrupt flags after each message
      nrf24_ClearIRQFlags();
    }
    
    // Priority 2: Check for UART commands
    uart_command_t uart_cmd;
    if (UART_ReadCommand(&uart_cmd)) {
      ProcessUARTCommand(&uart_cmd);
    }
    
    // Optional: Add a small delay to prevent excessive polling
    // HAL_Delay(1);
    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
