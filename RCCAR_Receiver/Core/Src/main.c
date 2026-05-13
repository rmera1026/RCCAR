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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NRF_CMD_R_REGISTER       0x00
#define NRF_CMD_W_REGISTER       0x20
#define NRF_CMD_R_RX_PAYLOAD     0x61
#define NRF_CMD_FLUSH_RX         0xE2

#define NRF_REG_CONFIG           0x00
#define NRF_REG_EN_AA            0x01
#define NRF_REG_EN_RXADDR        0x02
#define NRF_REG_SETUP_AW         0x03
#define NRF_REG_RF_CH            0x05
#define NRF_REG_RF_SETUP         0x06
#define NRF_REG_STATUS           0x07
#define NRF_REG_RX_ADDR_P0       0x0A
#define NRF_REG_RX_PW_P0         0x11

#define NRF_STATUS_RX_DR         0x40

#define CMD_STOP                 0
#define CMD_FORWARD              1
#define CMD_REVERSE              2
#define CMD_LEFT                 3
#define CMD_RIGHT                4

#define RX_FAILSAFE_TIMEOUT_MS   250

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Motor_Set(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, in1 ? GPIO_PIN_SET : GPIO_PIN_RESET); // IN1
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, in2 ? GPIO_PIN_SET : GPIO_PIN_RESET); // IN2
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, in3 ? GPIO_PIN_SET : GPIO_PIN_RESET); // IN3
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, in4 ? GPIO_PIN_SET : GPIO_PIN_RESET); // IN4
}

static uint8_t NRF24_SPI_Transfer(uint8_t value)
{
  uint8_t rx = 0;
  HAL_SPI_TransmitReceive(&hspi1, &value, &rx, 1, HAL_MAX_DELAY);
  return rx;
}

static void NRF24_CSN(uint8_t state)
{
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void NRF24_CE(uint8_t state)
{
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t NRF24_ReadReg(uint8_t reg)
{
  uint8_t value;
  NRF24_CSN(0);
  NRF24_SPI_Transfer(NRF_CMD_R_REGISTER | (reg & 0x1F));
  value = NRF24_SPI_Transfer(0xFF);
  NRF24_CSN(1);
  return value;
}

static void NRF24_WriteReg(uint8_t reg, uint8_t value)
{
  NRF24_CSN(0);
  NRF24_SPI_Transfer(NRF_CMD_W_REGISTER | (reg & 0x1F));
  NRF24_SPI_Transfer(value);
  NRF24_CSN(1);
}

static void NRF24_WriteBuf(uint8_t reg, const uint8_t *data, uint8_t len)
{
  uint8_t i;
  NRF24_CSN(0);
  NRF24_SPI_Transfer(NRF_CMD_W_REGISTER | (reg & 0x1F));
  for (i = 0; i < len; i++) {
    NRF24_SPI_Transfer(data[i]);
  }
  NRF24_CSN(1);
}

static void NRF24_ReadPayload(uint8_t *data, uint8_t len)
{
  uint8_t i;
  NRF24_CSN(0);
  NRF24_SPI_Transfer(NRF_CMD_R_RX_PAYLOAD);
  for (i = 0; i < len; i++) {
    data[i] = NRF24_SPI_Transfer(0xFF);
  }
  NRF24_CSN(1);
}

static void NRF24_InitRx(void)
{
  const uint8_t addr[5] = {'c', 'a', 'r', '0', '1'};

  NRF24_CE(0);
  HAL_Delay(5);

  NRF24_WriteReg(NRF_REG_EN_AA, 0x01);
  NRF24_WriteReg(NRF_REG_EN_RXADDR, 0x01);
  NRF24_WriteReg(NRF_REG_SETUP_AW, 0x03);     // 5-byte address
  NRF24_WriteReg(NRF_REG_RF_CH, 76);          // 2.476 GHz
  NRF24_WriteReg(NRF_REG_RF_SETUP, 0x06);     // 1 Mbps, 0 dBm
  NRF24_WriteBuf(NRF_REG_RX_ADDR_P0, addr, 5);
  NRF24_WriteReg(NRF_REG_RX_PW_P0, 1);        // 1-byte payload
  NRF24_WriteReg(NRF_REG_STATUS, 0x70);       // clear pending IRQ flags
  NRF24_WriteReg(NRF_REG_CONFIG, 0x0F);       // PWR_UP=1, PRIM_RX=1, CRC on

  NRF24_CSN(0);
  NRF24_SPI_Transfer(NRF_CMD_FLUSH_RX);
  NRF24_CSN(1);

  HAL_Delay(2);
  NRF24_CE(1);
}

static uint8_t NRF24_TryReadByte(uint8_t *byte)
{
  uint8_t status = NRF24_ReadReg(NRF_REG_STATUS);
  if ((status & NRF_STATUS_RX_DR) == 0) {
    return 0;
  }

  NRF24_ReadPayload(byte, 1);
  NRF24_WriteReg(NRF_REG_STATUS, NRF_STATUS_RX_DR);
  return 1;
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
  uint8_t rxByte = 0;
  uint32_t lastRxTick = HAL_GetTick();
  NRF24_InitRx();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (NRF24_TryReadByte(&rxByte)) {
      lastRxTick = HAL_GetTick();
      switch (rxByte) {
        case CMD_FORWARD:
          Motor_Set(0, 1, 0, 1); // left motor forward, right motor forward
          break;
        case CMD_REVERSE:
          Motor_Set(1, 0, 1, 0); // left motor reverse, right motor reverse
          break;
        case CMD_LEFT:
          Motor_Set(0, 1, 1, 0); // left motor forward, right motor reverse
          break;
        case CMD_RIGHT:
          Motor_Set(1, 0, 0, 1); // left motor reverse, right motor forward
          break;
        case CMD_STOP:
        default:
          Motor_Set(0, 0, 0, 0); // stop both motors
          break;
      }
    } else if ((HAL_GetTick() - lastRxTick) > RX_FAILSAFE_TIMEOUT_MS) {
      Motor_Set(0, 0, 0, 0);
    }

    HAL_Delay(10);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_IN1_Pin|MOTOR_IN2_Pin|MOTOR_IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CE_Pin NRF_CSN_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin|NRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_IN1_Pin MOTOR_IN2_Pin MOTOR_IN3_Pin */
  GPIO_InitStruct.Pin = MOTOR_IN1_Pin|MOTOR_IN2_Pin|MOTOR_IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_IN4_Pin */
  GPIO_InitStruct.Pin = MOTOR_IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_IN4_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
