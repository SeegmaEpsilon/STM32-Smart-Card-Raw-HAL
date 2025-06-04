/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RNG_HandleTypeDef hrng;

UART_HandleTypeDef huart1;
SMARTCARD_HandleTypeDef hsc3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RNG_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_SMARTCARD_Init(void);
/* USER CODE BEGIN PFP */
void print_debug(const char *format, ...);
void print_debug_bytes(const uint8_t *buf, uint16_t len);
static uint8_t hex_char_to_val(char c);
static uint16_t parse_hex_line(uint8_t *in, uint16_t in_len, uint8_t *out);
HAL_StatusTypeDef SC_send_APDU(uint8_t *apdu, uint16_t apdu_len, uint8_t *resp, uint16_t *resp_len);
void SC_cold_reset(void);
HAL_StatusTypeDef SC_get_ATR(void);
void SC_parse_ATR(const uint8_t *buf, uint16_t len);
HAL_StatusTypeDef SC_SELECT_PSE(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "stdarg.h"
#include "string.h"

/* send str with value over UART */

#define RX_BUF_LEN 256
#define ATR_MAX_LEN 32
#define SC_RESP_MAX_LEN 256

static uint8_t uart_rx_byte = 0;
static uint8_t uart_rx_buf[RX_BUF_LEN];
static uint16_t uart_rx_len = 0;
static uint8_t uart_cmd_received = 0;

static uint8_t atr_buf[ATR_MAX_LEN];
static uint16_t atr_len = 0;

static uint8_t hex_char_to_val(char c)
{
  if(c >= '0' && c <= '9') return c - '0';
  if(c >= 'A' && c <= 'F') return c - 'A' + 10;
  if(c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0xFF;
}

static uint16_t parse_hex_line(uint8_t *in, uint16_t in_len, uint8_t *out)
{
  uint16_t out_len = 0;
  for(uint16_t i = 0; i + 1 < in_len && out_len < 64; i++)
  {
    if(in[i] == ' ') continue;
    uint8_t hi = hex_char_to_val(in[i]);
    uint8_t lo = hex_char_to_val(in[i + 1]);
    if(hi == 0xFF || lo == 0xFF) break;
    out[out_len++] = (hi << 4) | lo;
    i++;
  }
  return out_len;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    if(uart_rx_byte == '\r' || uart_rx_byte == '\n')
    {
      if(uart_rx_len > 0)
      {
        uart_cmd_received = 1;
      }
    }
    else if(uart_rx_len < RX_BUF_LEN - 1)
    {
      uart_rx_buf[uart_rx_len++] = uart_rx_byte;
    }
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
  }
}

void print_debug(const char *format, ...)
{
  uint8_t temp[256] = {0};

  va_list args;
  va_start(args, format);
  vsprintf((char*)temp, format, args);
  va_end(args);

  while(HAL_UART_Transmit(&huart1, temp, strlen((char*)temp), 100) == HAL_BUSY);
}

void print_debug_bytes(const uint8_t *buf, uint16_t len)
{
  char hex[4];
  for(uint16_t i = 0; i < len; ++i)
  {
    snprintf(hex, sizeof(hex), "%02X ", buf[i]);
    HAL_UART_Transmit(&huart1, (uint8_t*)hex, 3, HAL_MAX_DELAY);
  }
}

HAL_StatusTypeDef SC_send_APDU(uint8_t *apdu, uint16_t apdu_len, uint8_t *resp, uint16_t *resp_len)
{
  HAL_StatusTypeDef ret;
  *resp_len = 0;

  print_debug("C-APDU => ");
  print_debug_bytes(apdu, apdu_len);
  print_debug("\r\n");

  // Передаём APDU-запрос
  ret = HAL_SMARTCARD_Transmit(&hsc3, apdu, apdu_len, 1000);
  if(ret != HAL_OK) return ret;

  while(*resp_len < SC_RESP_MAX_LEN)
  {
    uint8_t b;
    ret = HAL_SMARTCARD_Receive(&hsc3, &b, 1, 200);
    if(ret == HAL_OK)
    {
      resp[(*resp_len)++] = b;
    }
    else
    {
      break; // таймаут = конец ATR
    }
  }

  if((*resp_len > 0))
  {
    print_debug("R-APDU <= ");
    print_debug_bytes(resp, *resp_len);
    print_debug("\r\n");
    return HAL_OK;
  }
  else
  {
    print_debug("R-APDU <= ERROR\r\n");
    return HAL_TIMEOUT;
  }
}

void SC_cold_reset(void)
{
  /* Assume VCC already powered */
  HAL_GPIO_WritePin(SC_RST_GPIO_Port, SC_RST_Pin, GPIO_PIN_RESET); /* RST = Low (active) */

  __HAL_RCC_USART3_CLK_ENABLE(); /* enable card clock first */
  HAL_Delay(2); /* > 400 cycles = 0.11 ms, we use 2 ms for safety */

  HAL_GPIO_WritePin(SC_RST_GPIO_Port, SC_RST_Pin, GPIO_PIN_SET); /* Release reset */
}

HAL_StatusTypeDef SC_get_ATR(void)
{
  atr_len = 0;
  uint8_t byte;
  HAL_StatusTypeDef ret;

  while(atr_len < ATR_MAX_LEN)
  {
    ret = HAL_SMARTCARD_Receive(&hsc3, &byte, 1, 100);
    if(ret == HAL_OK)
    {
      atr_buf[atr_len++] = byte;
    }
    else
    {
      break; // таймаут = конец ATR
    }
  }
  return (atr_len > 0) ? HAL_OK : HAL_TIMEOUT;
}

void SC_parse_ATR(const uint8_t *buf, uint16_t len)
{
  if(len < 2)
  {
    print_debug("ATR too short\r\n");
    return;
  }

  uint8_t i = 0;
  uint8_t TS = buf[i++];
  print_debug("TS = 0x%02X (%s convention)\r\n", TS, TS == 0x3B ? "Direct" : TS == 0x3F ? "Inverse" : "Unknown");

  uint8_t T0 = buf[i++];
  uint8_t Y = (T0 & 0xF0) >> 4;
  uint8_t K = T0 & 0x0F;
  print_debug("T0 = 0x%02X (Y1=0x%X, K=%d historical bytes)\r\n", T0, Y, K);

  // Печатаем интерфейсные байты TA1–TDn
  for(uint8_t level = 1; Y && i < len; ++level)
  {
    if(Y & 0x01 && i < len) print_debug("TA%d = 0x%02X\r\n", level, buf[i++]);
    if(Y & 0x02 && i < len) print_debug("TB%d = 0x%02X\r\n", level, buf[i++]);
    if(Y & 0x04 && i < len) print_debug("TC%d = 0x%02X\r\n", level, buf[i++]);

    if(Y & 0x08 && i < len)
    {
      uint8_t TD = buf[i++];
      print_debug("TD%d = 0x%02X (Y%d=0x%X, T=%d)\r\n", level, TD, level + 1, (TD >> 4), TD & 0x0F);
      Y = (TD & 0xF0) >> 4;
    }
    else break;
  }

  // Печатаем historical bytes
  print_debug("Historical bytes: ");
  for(uint8_t j = 0; j < K && i < len; ++j)
    print_debug("%02X ", buf[i++]);
  print_debug("\r\n");

  // Попробуем вывести ASCII-эквивалент
  print_debug("ASCII: ");
  for(uint8_t j = 0; j < K && i - K + j < len; ++j)
  {
    char c = buf[i - K + j];
    print_debug("%c", (c >= 32 && c <= 126) ? c : '.');
  }
  print_debug("\r\n");

  // TCK (если есть)
  if(i < len)
  {
    uint8_t TCK = buf[i++];
    uint8_t sum = 0;
    for(uint8_t j = 1; j < len - 1; ++j)
      sum ^= buf[j];
    sum ^= TCK;
    print_debug("TCK = 0x%02X (%s)\r\n", TCK, sum == 0 ? "OK" : "Invalid checksum");
  }
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
  MX_RNG_Init();
  MX_USART1_UART_Init();
  MX_USART3_SMARTCARD_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);

  print_debug("\r\n=== Smart-Card CLI ===\r\n");
  print_debug("Press the K1 button to activate the smart-card...\r\n");

  uint32_t button_k1_last_ms = 0;
  uint32_t ticks_current_ms = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
    ticks_current_ms = HAL_GetTick();
    if(uart_cmd_received)
    {
      uint8_t apdu[64];
      uint8_t resp[SC_RESP_MAX_LEN];
      uint16_t apdu_len = parse_hex_line(uart_rx_buf, uart_rx_len, apdu);
      uint16_t resp_len = 0;

      SC_send_APDU(apdu, apdu_len, resp, &resp_len);

      uart_rx_len = 0;
      uart_cmd_received = 0;
    }

    if(HAL_GPIO_ReadPin(BUTTON_K1_GPIO_Port, BUTTON_K1_Pin) == GPIO_PIN_RESET
        && (ticks_current_ms - button_k1_last_ms > 1000))
    {
      SC_cold_reset();
      print_debug("Cold reset done, waiting for ATR...\r\n");

      if(SC_get_ATR() == HAL_OK)
      {
        print_debug("ATR length: %d bytes\r\nATR: ", atr_len);
        print_debug_bytes(atr_buf, atr_len);

        SC_parse_ATR(atr_buf, atr_len);

        uint8_t resp[SC_RESP_MAX_LEN];
        uint16_t resp_len;

        memset(resp, 0, sizeof(resp));
        memset(&resp_len, 0, sizeof(resp_len));

        uint8_t apdu_get_data_wrong[] = {0x80, 0xCA, 0x9F, 0x7F, 0x00};
        SC_send_APDU(apdu_get_data_wrong, sizeof(apdu_get_data_wrong), resp, &resp_len);

        memset(resp, 0, sizeof(resp));
        memset(&resp_len, 0, sizeof(resp_len));

        uint8_t apdu_get_data_correct[] = {0x80, 0xCA, 0x9F, 0x7F, 0x2D};
        SC_send_APDU(apdu_get_data_correct, sizeof(apdu_get_data_correct), resp, &resp_len);

        print_debug("\r\nDone.\r\n");
      }
      else
      {
        printf("ATR receive ERROR\r\n");
      }
      button_k1_last_ms = ticks_current_ms;
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief RNG Initialization Function
 * @param None
 * @retval None
 */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if(HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if(HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_SMARTCARD_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  hsc3.Instance = USART3;
  hsc3.Init.BaudRate = 9600;
  hsc3.Init.WordLength = SMARTCARD_WORDLENGTH_9B;
  hsc3.Init.StopBits = SMARTCARD_STOPBITS_1_5;
  hsc3.Init.Parity = SMARTCARD_PARITY_EVEN;
  hsc3.Init.Mode = SMARTCARD_MODE_TX_RX;
  hsc3.Init.CLKPolarity = SMARTCARD_POLARITY_LOW;
  hsc3.Init.CLKPhase = SMARTCARD_PHASE_1EDGE;
  hsc3.Init.CLKLastBit = SMARTCARD_LASTBIT_ENABLE;
  hsc3.Init.Prescaler = 6;
  hsc3.Init.GuardTime = 16;
  hsc3.Init.NACKState = SMARTCARD_NACK_ENABLE;
  if(HAL_SMARTCARD_Init(&hsc3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SC_RST_GPIO_Port, SC_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUTTON_K1_Pin BUTTON_K0_Pin */
  GPIO_InitStruct.Pin = BUTTON_K1_Pin | BUTTON_K0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SC_RST_Pin */
  GPIO_InitStruct.Pin = SC_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SC_RST_GPIO_Port, &GPIO_InitStruct);

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
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
