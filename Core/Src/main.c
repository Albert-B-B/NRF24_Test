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

typedef struct {
    uint8_t messages[TX_BUFFER_SIZE][TX_MESSAGE_SIZE];  // TX message buffer
    uint8_t count;                                      // Number of messages queued
} tx_buffer_t;

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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static char uart_buf[UART_BUFFER_SIZE];
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
static void UART_EnableRX(void);
static bool UART_ReadCommand(uart_command_t *cmd);
static void ProcessUARTCommand(uart_command_t *cmd);
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
 * @brief Enable UART2 RX functionality
 * @note Assumes UART2 is already configured by the HAL/CubeMX
 */
static void UART_EnableRX(void)
{
    // Enable USART2 peripheral clock if not already enabled
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    
    // Ensure RX is enabled (should already be from CubeMX config)
    USART2->CR1 |= USART_CR1_RE;
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
 * @brief Process received UART command
 * @param cmd Pointer to the received command
 */
static void ProcessUARTCommand(uart_command_t *cmd)
{
    command_count++;
    
    printf("[CMD_%lu] Type: 0x%02X, Data: 0x%08lX (%lu)\r\n",
           (unsigned long)command_count,
           cmd->command,
           (unsigned long)cmd->data,
           (unsigned long)cmd->data);
    
    // Create a 32-byte message to send to the drone
    uint8_t tx_message[TX_MESSAGE_SIZE] = {0};
    
    // Format: [CMD][DATA_3][DATA_2][DATA_1][DATA_0][RESERVED...]
    tx_message[0] = cmd->command;
    tx_message[1] = (uint8_t)(cmd->data & 0xFF);         // LSB
    tx_message[2] = (uint8_t)((cmd->data >> 8) & 0xFF);
    tx_message[3] = (uint8_t)((cmd->data >> 16) & 0xFF);
    tx_message[4] = (uint8_t)((cmd->data >> 24) & 0xFF); // MSB
    
    // Queue the message for transmission
    if (TX_QueueMessage(tx_message, TX_MESSAGE_SIZE)) {
        printf("  -> Command queued for transmission\r\n");
    } else {
        printf("  -> TX buffer full! Command dropped\r\n");
    }
    
    // TODO: Implement specific command handling
    switch (cmd->command) {
        case 0x01:
            printf("  -> Command 0x01: Reserved for future use\r\n");
            break;
        case 0x02:
            printf("  -> Command 0x02: Reserved for future use\r\n");
            break;
        // Add more command types here as needed
        default:
            printf("  -> Unknown command type\r\n");
            break;
    }
}

/**
 * @brief Queue a message for transmission
 * @param data Pointer to the data to send
 * @param length Length of the data (should be TX_MESSAGE_SIZE)
 * @return true if queued successfully, false if buffer is full
 */
static bool TX_QueueMessage(uint8_t *data, uint8_t length)
{
    if (tx_buffer.count >= TX_BUFFER_SIZE) {
        return false;  // Buffer is full
    }
    
    // Copy message to buffer
    if (length > TX_MESSAGE_SIZE) {
        length = TX_MESSAGE_SIZE;  // Truncate if too long
    }
    
    memcpy(tx_buffer.messages[tx_buffer.count], data, length);
    
    // Pad with zeros if message is shorter than TX_MESSAGE_SIZE
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
    nrf24_SetRXTXMode(false);  // Set to TX mode (PRIM_RX=0, CE low)
    // Small delay to ensure mode switch
    HAL_Delay(1);
}

/**
 * @brief Switch NRF24L01 to RX mode
 */
static void NRF24_SwitchToRX(void)
{
    nrf24_SetRXTXMode(true);   // Set to RX mode (PRIM_RX=1, CE high)
    // Small delay to ensure mode switch
    HAL_Delay(1);
}

/**
 * @brief Send all queued messages and return to RX mode
 */
static void TX_SendAllMessages(void)
{
    if (tx_buffer.count == 0) {
        return;  // Nothing to send
    }
    
    printf("Sending %d queued message(s)...\r\n", tx_buffer.count);
    
    // Switch to TX mode
    NRF24_SwitchToTX();
    
    // Send all queued messages
    for (uint8_t i = 0; i < tx_buffer.count; i++) {
        tx_count++;
        
        printf("[TX_%lu] Sending message %d/%d... ", 
               (unsigned long)tx_count, i+1, tx_buffer.count);
        
        // Transmit the message
        nrf24_Transmit(tx_buffer.messages[i], TX_MESSAGE_SIZE);
        
        // Wait a bit for transmission to complete
        HAL_Delay(5);
        
        // Check transmission status
        uint8_t status = nrf24_ReadRegister(0x07);  // STATUS register
        if (status & 0x20) {  // TX_DS bit
            printf("OK\r\n");
        } else if (status & 0x10) {  // MAX_RT bit
            printf("FAILED (Max retries)\r\n");
        } else {
            printf("UNKNOWN\r\n");
        }
        
        // Clear interrupt flags
        nrf24_ClearIRQFlags();
        
        // Small delay between messages
        if (i < tx_buffer.count - 1) {
            HAL_Delay(10);
        }
    }
    
    // Clear the TX buffer
    tx_buffer.count = 0;
    
    // Switch back to RX mode
    NRF24_SwitchToRX();
    
    printf("TX complete, returned to RX mode\r\n");
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

  // Send startup message
  printf("\r\n=== Drone Receiver Started ===\r\n");
  printf("System Clock: %lu MHz\r\n", HAL_RCC_GetSysClockFreq() / 1000000);
  
  // Enable UART RX for command reception
  UART_EnableRX();
  
  // Configure NRF24L01 for receiving
  NRF24_Configure();
  
  // Verify NRF24L01 is responding
  uint8_t config_reg = nrf24_ReadRegister(0x00);
  if (config_reg == 0xFF) {
    printf("ERROR: NRF24L01 not responding! Check wiring.\r\n");
    while(1) {
      HAL_Delay(1000);
    }
  }
  
  printf("NRF24L01 initialized successfully\r\n");
  printf("RF Channel: %d (2.%03d GHz)\r\n", RF_CHANNEL, 400 + RF_CHANNEL);
  printf("Payload Size: %d bytes\r\n", PAYLOAD_SIZE);
  printf("UART Command Format: 1 byte cmd + 4 bytes data\r\n");
  printf("TX Buffer: %d messages x %d bytes each\r\n", TX_BUFFER_SIZE, TX_MESSAGE_SIZE);
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
    bool uart_command_received = false;
    
    // Process all available UART commands
    while (UART_ReadCommand(&uart_cmd)) {
      ProcessUARTCommand(&uart_cmd);
      uart_command_received = true;
    }
    
    // Priority 3: Send queued TX messages if any UART commands were processed
    if (uart_command_received && tx_buffer.count > 0) {
      TX_SendAllMessages();
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
