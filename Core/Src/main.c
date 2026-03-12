/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Set to 0 for TX test mode, 1 for RX test mode */
#define NRF24_TEST_MODE_RX  1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static char     uart_buf[80];
static uint32_t tx_count = 0;
static uint32_t rx_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* --- Minimal UART2 driver (direct register access, no HAL UART required) --- */
/* PA2 = USART2_TX (AF7), PA3 = USART2_RX (AF7) — already configured by MX_GPIO_Init */
static void UART2_Init(void)
{
    /* Enable USART2 peripheral clock (PCLK1 = 170 MHz) */
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;
    /* BRR = PCLK1 / BaudRate = 170 000 000 / 115 200 ≈ 1477 */
    USART2->BRR = 1477U;
    /* Enable TX and USART */
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

static void UART2_Print(const char *str)
{
    while (*str) {
        while (!(USART2->ISR & USART_ISR_TXE_TXFNF));
        USART2->TDR = (uint8_t)(*str++);
    }
}

/* Redirect printf → USART2 via newlib weak hook */
int __io_putchar(int ch)
{
    while (!(USART2->ISR & USART_ISR_TXE_TXFNF));
    USART2->TDR = (uint8_t)ch;
    return ch;
}

/* Dump key nRF24L01+ registers to UART to verify SPI and config */
static void nrf24_PrintRegisters(void)
{
    uint8_t addr[5];
    uint8_t r;
    UART2_Print("--- nRF24L01+ register dump ---\r\n");

    r = nrf24_ReadRegister(0x00);
    snprintf(uart_buf, sizeof(uart_buf), "  CONFIG     0x00 = 0x%02X", r);
    UART2_Print(uart_buf);
    if (r == 0xFF) UART2_Print(" !! 0xFF: SPI not responding - check wiring/VCC !!\r\n");
    else           UART2_Print(" (0x0F=RX ok, 0x0E=TX ok)\r\n");

    r = nrf24_ReadRegister(0x01);
    snprintf(uart_buf, sizeof(uart_buf), "  EN_AA      0x01 = 0x%02X  (expect 0x00)\r\n", r);
    UART2_Print(uart_buf);

    r = nrf24_ReadRegister(0x02);
    snprintf(uart_buf, sizeof(uart_buf), "  EN_RXADDR  0x02 = 0x%02X  (expect 0x01)\r\n", r);
    UART2_Print(uart_buf);

    r = nrf24_ReadRegister(0x05);
    snprintf(uart_buf, sizeof(uart_buf), "  RF_CH      0x05 = %u   (expect 76)\r\n", r);
    UART2_Print(uart_buf);

    r = nrf24_ReadRegister(0x06);
    snprintf(uart_buf, sizeof(uart_buf), "  RF_SETUP   0x06 = 0x%02X  (expect 0x06)\r\n", r);
    UART2_Print(uart_buf);

    r = nrf24_ReadRegister(0x07);
    snprintf(uart_buf, sizeof(uart_buf), "  STATUS     0x07 = 0x%02X\r\n", r);
    UART2_Print(uart_buf);

    nrf24_ReadRegisterMulti(0x0A, addr, 5);
    snprintf(uart_buf, sizeof(uart_buf), "  RX_ADDR_P0 0x0A = %02X:%02X:%02X:%02X:%02X  (expect E7 x5)\r\n",
             addr[0], addr[1], addr[2], addr[3], addr[4]);
    UART2_Print(uart_buf);

    nrf24_ReadRegisterMulti(0x10, addr, 5);
    snprintf(uart_buf, sizeof(uart_buf), "  TX_ADDR    0x10 = %02X:%02X:%02X:%02X:%02X  (expect E7 x5)\r\n",
             addr[0], addr[1], addr[2], addr[3], addr[4]);
    UART2_Print(uart_buf);

    r = nrf24_ReadRegister(0x11);
    snprintf(uart_buf, sizeof(uart_buf), "  RX_PW_P0   0x11 = %u    (expect 4)\r\n", r);
    UART2_Print(uart_buf);

    r = nrf24_ReadRegister(0x17);
    snprintf(uart_buf, sizeof(uart_buf), "  FIFO_STAT  0x17 = 0x%02X\r\n", r);
    UART2_Print(uart_buf);

    UART2_Print("-------------------------------\r\n");
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
  /* USER CODE BEGIN 2 */

  /* --- Peripheral init --- */
  UART2_Init();

  nrf24_Init();
  nrf24_WriteRegister(0x01, 0x00); /* EN_AA       : disable auto-ack on all pipes */
  nrf24_WriteRegister(0x04, 0x00); /* SETUP_RETR  : no retransmissions            */

#if NRF24_TEST_MODE_RX
  /* ---- RX mode ---- */
  UART2_Print("\r\nNRF24L01+ RX Test\r\n");
  /* Enable pipe 0, set its payload width to 4 bytes */
  nrf24_WriteRegister(0x02, 0x01); /* EN_RXADDR   : enable pipe 0 */
  nrf24_WriteRegister(0x11, 0x04); /* RX_PW_P0    : 4-byte payload */
  nrf24_FlushRX();                 /* Clear any stale data in RX FIFO */
  nrf24_ClearIRQFlags();           /* Clear any stale interrupt flags */
  nrf24_SetRXTXMode(true);         /* PRIM_RX=1, CE high -> listening */
#else
  /* ---- TX mode ---- */
  UART2_Print("\r\nNRF24L01+ TX Test\r\n");
  nrf24_SetRXTXMode(false);        /* PRIM_RX=0, CE low -> standby */
#endif

  /* Print startup configuration */
  uint8_t cfg      = nrf24_ReadRegister(0x00); /* CONFIG    */
  uint8_t rf_setup = nrf24_ReadRegister(0x06); /* RF_SETUP  */
  uint8_t rf_ch    = nrf24_ReadRegister(0x05); /* RF_CH     */
  snprintf(uart_buf, sizeof(uart_buf),
           "CONFIG=0x%02X RF_SETUP=0x%02X CH=%u (2.%03uGHz)\r\n",
           cfg, rf_setup, (unsigned)rf_ch, 400u + (unsigned)rf_ch);
  UART2_Print(uart_buf);
#if NRF24_TEST_MODE_RX
  UART2_Print("Listening for 4-byte packets...\r\n");
#else
  UART2_Print("Sending a 4-byte packet every 500 ms...\r\n");
#endif

  nrf24_PrintRegisters();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

#if NRF24_TEST_MODE_RX
    /* ---- RX loop ---- */

    /* Heartbeat every 3 s: proves the loop is running and SPI still works.
       STATUS=0x0E/0x0F = module alive.  STATUS=0xFF = SPI fault. */
    static uint32_t last_hb = 0;
    if (HAL_GetTick() - last_hb >= 3000U) {
        last_hb = HAL_GetTick();
        uint8_t hb_s = nrf24_ReadRegister(0x07);
        uint8_t hb_f = nrf24_ReadRegister(0x17);
        snprintf(uart_buf, sizeof(uart_buf),
                 "[HB] STATUS=0x%02X FIFO=0x%02X rx_count=%lu\r\n",
                 hb_s, hb_f, (unsigned long)rx_count);
        UART2_Print(uart_buf);
    }

    if (nrf24_dataReady()) {
        uint8_t rx_buf[4] = {0};
        nrf24_Receive(rx_buf, sizeof(rx_buf));
        nrf24_ClearIRQFlags();

        snprintf(uart_buf, sizeof(uart_buf),
                 "RX#%lu bytes=[%02X %02X %02X %02X]\r\n",
                 (unsigned long)rx_count,
                 rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
        UART2_Print(uart_buf);
        rx_count++;
    }
#else
    /* ---- TX loop ---- */
    /* Build 4-byte payload: sync byte + 24-bit counter
       Visible on oscilloscope as repeating SPI bursts every 500 ms.
       Visible on spectrum analyser as GFSK packets at 2.476 GHz (CH 76). */
    uint8_t payload[4] = {
        0xAA,                                   /* fixed sync byte          */
        (uint8_t)((tx_count >> 16) & 0xFFU),    /* counter high byte        */
        (uint8_t)((tx_count >>  8) & 0xFFU),    /* counter mid byte         */
        (uint8_t)( tx_count        & 0xFFU)     /* counter low byte         */
    };

    nrf24_Transmit(payload, sizeof(payload));

    /* Allow time for the packet to leave the air (~320 µs at 1 Mbps) */
    HAL_Delay(5);

    /* Read STATUS (0x07) and FIFO_STATUS (0x17) for diagnostics */
    uint8_t status      = nrf24_ReadRegister(0x07);
    uint8_t fifo_status = nrf24_ReadRegister(0x17);

    snprintf(uart_buf, sizeof(uart_buf),
             "TX#%lu STATUS=0x%02X FIFO=0x%02X",
             (unsigned long)tx_count, (unsigned)status, (unsigned)fifo_status);
    UART2_Print(uart_buf);

    if (status & 0x20U) {          /* bit 5 = TX_DS: packet sent successfully */
        UART2_Print(" [TX_DS OK]\r\n");
    } else if (status & 0x10U) {   /* bit 4 = MAX_RT: retransmit limit hit    */
        UART2_Print(" [MAX_RT - check wiring]\r\n");
    } else {
        UART2_Print(" [no IRQ flag set]\r\n");
    }

    nrf24_ClearIRQFlags();
    tx_count++;

    HAL_Delay(495); /* ~500 ms period total (5 ms wait + 495 ms idle) */
#endif
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
