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
#define NRF_CMD_W_TX_PAYLOAD     0xA0
#define NRF_CMD_FLUSH_TX         0xE1

#define NRF_REG_CONFIG           0x00
#define NRF_REG_EN_AA            0x01
#define NRF_REG_EN_RXADDR        0x02
#define NRF_REG_SETUP_AW         0x03
#define NRF_REG_SETUP_RETR       0x04
#define NRF_REG_RF_CH            0x05
#define NRF_REG_RF_SETUP         0x06
#define NRF_REG_STATUS           0x07
#define NRF_REG_RX_ADDR_P0       0x0A
#define NRF_REG_TX_ADDR          0x10

#define NRF_STATUS_TX_DS         0x20
#define NRF_STATUS_MAX_RT        0x10

#define CMD_STOP                 0
#define CMD_FORWARD              1
#define CMD_REVERSE              2
#define CMD_LEFT                 3
#define CMD_RIGHT                4

#define JOY_DEFAULT_CENTER       2048
#define JOY_DEADZONE_Y           400
#define JOY_DEADZONE_X           180
#define JOY_CROSS_CANCEL_X       260
#define JOY_CROSS_CANCEL_Y       260
#define JOY_AXIS_PRIORITY        0
#define JOY_HYSTERESIS_X         0
#define JOY_HYSTERESIS_Y         0
#define JOY_PRIORITY_HYSTERESIS  0
#define JOY_CALIBRATION_SAMPLES  40
#define MODE_TOGGLE_DEBOUNCE_MS  200
#define JOY_BTN_GPIO_Port        GPIOA
#define JOY_BTN_Pin              GPIO_PIN_4
#define JOY_SWAP_AXES            1
#define JOY_INVERT_X             1
#define JOY_INVERT_Y             1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
typedef enum {
  DRIVE_MODE_FB = 1,
  DRIVE_MODE_LR = 0
} DriveMode_t;

static uint16_t joyCenterX = JOY_DEFAULT_CENTER;
static uint16_t joyCenterY = JOY_DEFAULT_CENTER;
static DriveMode_t driveMode = DRIVE_MODE_FB;
static GPIO_PinState buttonPrevState = GPIO_PIN_SET;
static uint32_t lastModeToggleMs = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

static void NRF24_InitTx(void)
{
  const uint8_t addr[5] = {'c', 'a', 'r', '0', '1'};

  NRF24_CE(0);
  HAL_Delay(5);

  NRF24_WriteReg(NRF_REG_EN_AA, 0x01);
  NRF24_WriteReg(NRF_REG_EN_RXADDR, 0x01);
  NRF24_WriteReg(NRF_REG_SETUP_AW, 0x03);    // 5-byte address
  NRF24_WriteReg(NRF_REG_SETUP_RETR, 0x2F);  // auto-retry delay/count
  NRF24_WriteReg(NRF_REG_RF_CH, 76);         // 2.476 GHz
  NRF24_WriteReg(NRF_REG_RF_SETUP, 0x06);    // 1 Mbps, 0 dBm
  NRF24_WriteBuf(NRF_REG_TX_ADDR, addr, 5);
  NRF24_WriteBuf(NRF_REG_RX_ADDR_P0, addr, 5);
  NRF24_WriteReg(NRF_REG_STATUS, 0x70);
  NRF24_WriteReg(NRF_REG_CONFIG, 0x0E);      // PWR_UP=1, PRIM_RX=0, CRC on

  NRF24_CSN(0);
  NRF24_SPI_Transfer(NRF_CMD_FLUSH_TX);
  NRF24_CSN(1);

  HAL_Delay(2);
}

static void NRF24_SendByte(uint8_t value)
{
  uint8_t status;

  NRF24_CSN(0);
  NRF24_SPI_Transfer(NRF_CMD_W_TX_PAYLOAD);
  NRF24_SPI_Transfer(value);
  NRF24_CSN(1);

  NRF24_CE(1);
  HAL_Delay(1);
  NRF24_CE(0);

  status = NRF24_ReadReg(NRF_REG_STATUS);
  NRF24_WriteReg(NRF_REG_STATUS, NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);

  if (status & NRF_STATUS_MAX_RT) {
    NRF24_CSN(0);
    NRF24_SPI_Transfer(NRF_CMD_FLUSH_TX);
    NRF24_CSN(1);
  }
}

static void Joystick_Read(uint16_t *x, uint16_t *y)
{
  if (HAL_ADC_Start(&hadc1) != HAL_OK) {
    *x = 2048;
    *y = 2048;
    return;
  }

  if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
    *x = (uint16_t)HAL_ADC_GetValue(&hadc1);
  } else {
    *x = 2048;
  }

  if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
    *y = (uint16_t)HAL_ADC_GetValue(&hadc1);
  } else {
    *y = 2048;
  }

  HAL_ADC_Stop(&hadc1);
}

static void Joystick_CalibrateCenter(void)
{
  uint32_t sumX = 0;
  uint32_t sumY = 0;
  uint16_t x;
  uint16_t y;
  uint8_t i;

  for (i = 0; i < JOY_CALIBRATION_SAMPLES; i++) {
    Joystick_Read(&x, &y);
    sumX += x;
    sumY += y;
    HAL_Delay(5);
  }

  joyCenterX = (uint16_t)(sumX / JOY_CALIBRATION_SAMPLES);
  joyCenterY = (uint16_t)(sumY / JOY_CALIBRATION_SAMPLES);
}

static int16_t Abs16(int16_t value)
{
  return (value < 0) ? (int16_t)(-value) : value;
}

static uint8_t UpdateDriveModeFromButton(void)
{
  GPIO_PinState buttonNow = HAL_GPIO_ReadPin(JOY_BTN_GPIO_Port, JOY_BTN_Pin);
  uint32_t nowMs = HAL_GetTick();

  if ((buttonPrevState == GPIO_PIN_SET) &&
      (buttonNow == GPIO_PIN_RESET) &&
      ((nowMs - lastModeToggleMs) > MODE_TOGGLE_DEBOUNCE_MS)) {
    if (driveMode == DRIVE_MODE_FB) {
      driveMode = DRIVE_MODE_LR;
    } else {
      driveMode = DRIVE_MODE_FB;
    }
    lastModeToggleMs = nowMs;
    buttonPrevState = buttonNow;
    return 1;
  }

  buttonPrevState = buttonNow;
  return 0;
}

static uint8_t Command_FromJoystick(uint16_t x, uint16_t y)
{
  int16_t dx = (int16_t)x - (int16_t)joyCenterX;
  int16_t dy = (int16_t)y - (int16_t)joyCenterY;
  int16_t axisX = dx;
  int16_t axisY = dy;
  int16_t joystickX;
  int16_t joystickY;

#if JOY_SWAP_AXES
  axisX = dy;
  axisY = dx;
#endif

  joystickX = axisX;
  joystickY = axisY;

#if JOY_INVERT_X
  joystickX = -joystickX;
#endif

#if JOY_INVERT_Y
  joystickY = -joystickY;
#endif

  if (driveMode == DRIVE_MODE_FB) {
    if (joystickY > JOY_DEADZONE_Y) {
      return CMD_FORWARD;
    }
    if (joystickY < -JOY_DEADZONE_Y) {
      return CMD_REVERSE;
    }
  } else {
    if (joystickX > JOY_DEADZONE_X) {
      return CMD_RIGHT;
    }
    if (joystickX < -JOY_DEADZONE_X) {
      return CMD_LEFT;
    }
  }
  
  return CMD_STOP;
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
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  uint16_t joyX = 2048;
  uint16_t joyY = 2048;
  uint8_t cmd = CMD_STOP;

  NRF24_InitTx();
  HAL_Delay(300);
  Joystick_CalibrateCenter();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (UpdateDriveModeFromButton() != 0U) {
      cmd = CMD_STOP;
    } else {
      Joystick_Read(&joyX, &joyY);
      cmd = Command_FromJoystick(joyX, joyY);
    }

    NRF24_SendByte(cmd);

    HAL_Delay(50);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
