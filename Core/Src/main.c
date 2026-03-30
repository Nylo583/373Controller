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
#include <stdio.h>
#include <math.h>
#include "MadgwickAHRS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum State {
	STATE_NONE,
	STATE_INIT,
	STATE_CONNECT,
	STATE_CALIBRATION,
	STATE_MENU,
	STATE_OPERATION
};

typedef struct {
	int16_t x, y, z;
} XYZ;

typedef struct {
	float w,x,y,z;
} Quaternion;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

enum State state = STATE_INIT;
enum State prev_state = STATE_NONE;

uint8_t spi1_rx_buffer[100] = {0};
uint8_t spi1_tx_buffer[100] = {0};
Quaternion q;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */


#define LCD_ADDR  (0x27 << 1)
#define LCD_BL    0x08  // backlight
#define LCD_EN    0x04
#define LCD_RW    0x02
#define LCD_RS    0x01

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// - begin bmi

#define SPI1_GPIOX GPIOA
#define SPI1_CS GPIO_PIN_4

void BMI323_Read(uint8_t register_address, uint8_t size) {
	  spi1_tx_buffer[0] = 0x80 | register_address;
	  spi1_tx_buffer[1] = 0;

	  for(int i=0;i<2*size;i++)
		  spi1_tx_buffer[2+i]=0;


	  HAL_GPIO_WritePin(SPI1_GPIOX, SPI1_CS, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1, spi1_tx_buffer, spi1_rx_buffer, 2+2*size, 1000);
	  HAL_GPIO_WritePin(SPI1_GPIOX, SPI1_CS, GPIO_PIN_SET);
}

void BMI323_Write(uint8_t register_address, uint8_t size, uint8_t *data) {
	spi1_tx_buffer[0] = register_address;

	for(int i=0;i<size;i++)
	    {
			spi1_tx_buffer[2*i+1]=data[2*i];
			spi1_tx_buffer[2*i+2]=data[2*i+1];
	    }

	HAL_GPIO_WritePin(SPI1_GPIOX, SPI1_CS, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, spi1_tx_buffer, 1+2*size, 1000);
	HAL_GPIO_WritePin(SPI1_GPIOX, SPI1_CS, GPIO_PIN_SET);
	HAL_Delay(1);
}

int16_t BMI323_Assemble(uint8_t start) {
	  return spi1_rx_buffer[2*start+2] | (spi1_rx_buffer[2*start+3] << 8);
}


void BMI323_Init() {
	HAL_GPIO_WritePin(SPI1_GPIOX, SPI1_CS, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(SPI1_GPIOX, SPI1_CS, GPIO_PIN_SET);
	HAL_Delay(1); // toggling cs on init switches BMI to SPI mode


	uint8_t reset[2]={0xB6,0x00};

	BMI323_Write(0x7E,1,reset);

	HAL_Delay(20);

	// accel
	uint8_t acc[2]={0x08,0x70};
	BMI323_Write(0x20,1,acc);

	/* gyro  */
	uint8_t gyr[2]={0x18,0x70};
	BMI323_Write(0x21,1,gyr);

	HAL_Delay(500);
}

int BMI323_ConnectionTest() {
	BMI323_Read(0x00, 3);
	uint16_t chip_id = BMI323_Assemble(0) & 0xFF;
	uint16_t s_stat = BMI323_Assemble(2);

	if ((chip_id == 67) && ((s_stat & 0x01) == 0)) {
		return 1;
	}

	return 0;
}

void BMI323_ReadAccel(XYZ *data) {
	BMI323_Read(0x03, 3);
	data->x = (int16_t)BMI323_Assemble(0);
	data->y = (int16_t)BMI323_Assemble(1);
	data->z = (int16_t)BMI323_Assemble(2);
}

void BMI323_ReadGyro(XYZ* data) {
	BMI323_Read(0x06, 3);
	data->x = (int16_t)BMI323_Assemble(0);
	data->y = (int16_t)BMI323_Assemble(1);
	data->z = (int16_t)BMI323_Assemble(2);
}

void BMI323_Quaternion(Quaternion* q) {

	/*
	 * This function implements the Madgwick IMU sensor fusion algorithm,
	 * which is open source and can be obtained for free from:
	 * https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
	 */

	XYZ accel, gyro;
	BMI323_ReadAccel(&accel);
	BMI323_ReadGyro(&gyro);

	// convert a and g to normalized floats
	float ax = accel.x * (2.0f / 32768.0f);
	float ay = accel.y * (2.0f / 32768.0f);
	float az = accel.z * (2.0f / 32768.0f);

	float gx = gyro.x * (250.0f / 32768.0f) * (M_PI / 180.0f);
	float gy = gyro.y * (250.0f / 32768.0f) * (M_PI / 180.0f);
	float gz = gyro.z * (250.0f / 32768.0f) * (M_PI / 180.0f);

	MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);

	q->w = q0;
	q->x = q1;
	q->y = q2;
	q->z = q3;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
    	BMI323_Quaternion(&q);
    }
}

// - end bmi

// - begin RE


// previous 2-bit state becomes the top 2 bits of the new state!
static volatile int RE_count = 0;
static int RE_prev = 0;

static const int RE_gray[16] = {
		0, -1, 1, 0,
		1, 0, 0, -1,
		-1, 0, 0, 1,
		0, 1, -1, 0
};

void RE_Update() {
	int a = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET) ? 1 : 0;
	int b = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET) ? 1 : 0;
	int RE_current = (a << 1) | b;
	int res = RE_gray[(RE_prev << 2) | RE_current];

	RE_count += res;
	RE_prev = RE_current;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_11 || GPIO_Pin == GPIO_PIN_2) {
		RE_Update();
	}
}

// - end RE

// - begin display

static void LCD_write_pcf(uint8_t data)
{
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &data, 1, HAL_MAX_DELAY);
}

static void LCD_pulse_en(uint8_t data)
{
    LCD_write_pcf(data | LCD_EN);
    HAL_Delay(1);
    LCD_write_pcf(data & ~LCD_EN);
    HAL_Delay(1);
}

static void LCD_write_nibble(uint8_t data, uint8_t rs)
{
    uint8_t nibble = (data & 0xF0) | LCD_BL | rs;
    LCD_pulse_en(nibble);
}

/* sends a full byte as two nibbles */
static void LCD_send(uint8_t data, uint8_t rs)
{
    LCD_write_nibble(data,        rs);  // high nibble
    LCD_write_nibble(data << 4,   rs);  // low nibble
}

#define LCD_cmd(b)   LCD_send((b), 0x00)
#define LCD_data(b)  LCD_send((b), LCD_RS)

/* ── init ─────────────────────────────────────────────────── */

void LCD_init(void)
{
    HAL_Delay(50);                      // wait for LCD power-on

    // reset sequence:send 0x3 nibble three times to ensure 8-bit mode
    LCD_write_nibble(0x30, 0);  HAL_Delay(5);
    LCD_write_nibble(0x30, 0);  HAL_Delay(1);
    LCD_write_nibble(0x30, 0);  HAL_Delay(1);

    // switch to 4-bit mode
    LCD_write_nibble(0x20, 0);  HAL_Delay(1);

    // from here, full bytes as two nibbles
    LCD_cmd(0x28);   // function set:  4-bit | 2 lines | 5x8
    LCD_cmd(0x0C);   // display on, cursor off, blink off
    LCD_cmd(0x06);   // entry mode: increment, no shift
    LCD_cmd(0x01);   // clear display
    HAL_Delay(2);    // clear needs >1.5ms
}

void LCD_set_cursor(uint8_t col, uint8_t row)
{
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};  // was missing rows 2 and 3
    LCD_cmd(0x80 | (col + row_offsets[row]));
}

void LCD_print(const char *str)
{
    while (*str)
        LCD_data((uint8_t)*str++);
}

void LCD_Clear() {
	LCD_cmd(0x01);   // clear display
	HAL_Delay(2);    // clear needs >1.5ms
}

// - end display

// - begin SM



// - end SM

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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  BMI323_Init();
  int test = BMI323_ConnectionTest();
  while (test == 0) {
	  HAL_Delay(5);
	  test = BMI323_ConnectionTest();
  }

  uint32_t t1;
  t1 = HAL_GetTick();

  HAL_TIM_Base_Start_IT(&htim2);

  LCD_init();
  LCD_set_cursor(0, 0);
  LCD_print("Hello!");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  if (HAL_GetTick() - t1 >= 1000) {

	          t1 = HAL_GetTick();
	          printf("q: %.4f %.4f %.4f %.4f\r\n", q0, q1, q2, q3);

	          char buffer[21];

              LCD_set_cursor(0, 0);
              sprintf(buffer, "w: %.4f", q0);
              LCD_print(buffer);

              LCD_set_cursor(0, 1);
              sprintf(buffer, "x: %.4f", q1);
              LCD_print(buffer);

              LCD_set_cursor(0, 2);
              sprintf(buffer, "y: %.4f", q2);
              LCD_print(buffer);

              LCD_set_cursor(0, 3);
              sprintf(buffer, "z: %.4f", q3);
              LCD_print(buffer);

              LCD_set_cursor(12, 2);
              sprintf(buffer, "count:");
              LCD_print(buffer);

              LCD_set_cursor(12, 3);
              sprintf(buffer, "%d", RE_count);
              LCD_print(buffer);

	  }


	  prev_state = state;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B07CB4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD2_Pin|LD3_Pin|LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD3_Pin LD1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD3_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : B2_Pin B3_Pin */
  GPIO_InitStruct.Pin = B2_Pin|B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */




#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

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
