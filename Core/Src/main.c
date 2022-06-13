/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

#include <stdio.h>
#include <string.h>
#include "stm32l562e_discovery_lcd.h"
#include "lsm6dso.h"
#include "NanoEdgeAI.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef union {
  int16_t i16bit[AXIS_NUMBER];
  uint8_t u8bit[AXIS_NUMBER * 2];
} axis3bit16_t;

typedef enum{
	LEARNING,
	DETECTION
} NEAI_STATE;

typedef enum{
	DETECTION_CPU,
	NEAI_STATS
} LCD_DISPLAY_TYPE;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LCD_INSTANCE			0
#define ACC_DATA_RATE			833		// 833, 1667 or 3333 Hz
#define DATALOG					0		// 0: program in NanoEdge AI mode | 1: program in datalogging mode
#define LEARNING_ITERATIONS		50		// When DATALOG 1: number of signals logged
#define NEAI_THRESHOLD			95		// Between 1 and 99, typically around 90

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

stmdev_ctx_t accel_dev;

float acc_buffer_mg[AXIS_NUMBER * DATA_INPUT_USER + 1] = {0.0};
uint8_t display_string[50];
NEAI_STATE neai_state = LEARNING;
uint8_t learn_iteration = 0;
uint8_t similarity = 0, previous_similarity = 0;

float CPU_percent = 0.0;
uint32_t cycle_count = 55000000;

LCD_DISPLAY_TYPE lcd_display = DETECTION_CPU, previous_lcd_display = DETECTION_CPU;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

float map(float x, float in_min, float in_max, float out_min, float out_max);
void LCD_Screen_Init(void);
void LCD_Set_Text_Display(uint32_t backgroundColor, uint32_t textColor, sFONT * fontSize);
int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void LSM6DSO_Init(void);
void print_acc_data(void);
void draw_similarity_graph_axes(void);
void display_similarity_graph(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

#if DATALOG
  __HAL_TIM_SET_AUTORELOAD(&htim6, 14999);		// Set loop period to 1,5s
#endif

  enum neai_state error_code = neai_anomalydetection_init();	// Anomaly Detection init
  if (error_code != NEAI_OK) {									// Checking NEAI error code
	  printf("NEAI ERROR %d\nPROGRAM STOPPED\r\n", error_code);
	  while(1){}
  }

  LSM6DSO_Init();
  LCD_Screen_Init();
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 55;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00A03AC8;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 10999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_NORSRAM_TimingTypeDef Timing = {0};
  FMC_NORSRAM_TimingTypeDef ExtTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FMC_NORSRAM_DEVICE;
  hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_ENABLE;
  hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  hsram1.Init.NBLSetupTime = 0;
  hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  hsram1.Init.MaxChipSelectPulse = DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.DataHoldTime = 0;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */
  ExtTiming.AddressSetupTime = 15;
  ExtTiming.AddressHoldTime = 15;
  ExtTiming.DataSetupTime = 255;
  ExtTiming.DataHoldTime = 0;
  ExtTiming.BusTurnAroundDuration = 15;
  ExtTiming.CLKDivision = 16;
  ExtTiming.DataLatency = 17;
  ExtTiming.AccessMode = FMC_ACCESS_MODE_A;

  if (HAL_SRAM_Init(&hsram1, &Timing, &ExtTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED_GREEN_Pin|BLE_RSTN_Pin|AUDIO_RESETN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UCPD_DBn_GPIO_Port, UCPD_DBn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_TE_GPIO_Port, LCD_TE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_PWR_ON_GPIO_Port, LCD_PWR_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LCD_RST_Pin|STMOD_SEL_12_Pin|STMOD_SEL_34_Pin|CTP_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_GREEN_Pin BLE_RSTN_Pin AUDIO_RESETN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|BLE_RSTN_Pin|AUDIO_RESETN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GYRO_ACC_INT_Pin SDIO_DETECT_Pin CTP_INT_Pin */
  GPIO_InitStruct.Pin = GYRO_ACC_INT_Pin|SDIO_DETECT_Pin|CTP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : UCPD_DBn_Pin */
  GPIO_InitStruct.Pin = UCPD_DBn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UCPD_DBn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_TE_Pin */
  GPIO_InitStruct.Pin = LCD_TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_PWR_ON_Pin */
  GPIO_InitStruct.Pin = LCD_PWR_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_PWR_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_INT_Pin */
  GPIO_InitStruct.Pin = BLE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PG4 PG2 PG3 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RST_Pin STMOD_SEL_12_Pin STMOD_SEL_34_Pin CTP_RST_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|STMOD_SEL_12_Pin|STMOD_SEL_34_Pin|CTP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : UCPD_FLT_Pin */
  GPIO_InitStruct.Pin = UCPD_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UCPD_FLT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI13_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI13_IRQn);

}

/* USER CODE BEGIN 4 */


/*
 * __io_putchar(int ch) and int _write(int file,char *ptr, int len):
 * Functions allowing to use printf to a serial terminal
 */
int __io_putchar(int ch)
{
	uint8_t c[1];
	c[0] = ch & 0x00FF;
	HAL_UART_Transmit(&huart1, &*c, 1, 1000);
	return ch;
}

int _write(int file,char *ptr, int len)
{
	int DataIdx;
	for(DataIdx= 0; DataIdx< len; DataIdx++) {
		__io_putchar(*ptr++);
	}
	return len;
}


/*
 * Function to map a value from an input range to an output range
 */
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/*
 * LCD screen Init function
 */
void LCD_Screen_Init(void)
{
	if(BSP_LCD_Init(LCD_INSTANCE, LCD_ORIENTATION_PORTRAIT) != BSP_ERROR_NONE) {
			while(1){}
	}
	if(BSP_LCD_SetFuncDriver() != BSP_ERROR_NONE) {
			while(1){}
	}
	if(BSP_LCD_DisplayOn(LCD_INSTANCE) != BSP_ERROR_NONE) {
			while(1){}
	}
}

/*
 * LCD screen display settings
 */
void LCD_Set_Text_Display(uint32_t backgroundColor, uint32_t textColor, sFONT * fontSize)
{
	BSP_LCD_SetFont(fontSize);
	BSP_LCD_SetBackColor(backgroundColor);
	BSP_LCD_SetTextColor(textColor);
}


/*
 * I2C read and write functions used by the LSM6DSO driver
 */
int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	int32_t status = 0;
	status = HAL_I2C_Mem_Write(handle, LSM6DSO_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return status;
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	int32_t status = 0;
	status = HAL_I2C_Mem_Read(handle, LSM6DSO_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return status;
}


/*
 * LSM6DSO accelerometer init function
 */
void LSM6DSO_Init(void)
{
	uint8_t whoamI = 0, rst = 0;
	accel_dev.handle = &hi2c1;
	accel_dev.write_reg = platform_write;
	accel_dev.read_reg = platform_read;

	lsm6dso_reset_set(&accel_dev, PROPERTY_ENABLE);		// Restore default configuration
	HAL_Delay(100);										// Wait sensor boot time (min = 10 ms)
	lsm6dso_device_id_get(&accel_dev, &whoamI);			// Check device ID
	if (whoamI != LSM6DSO_ID) {
		printf("ERROR LSM6DSO WHOAMI: whoami = %d\r\n", whoamI);
		LSM6DSO_Init();
	}
	do {
		lsm6dso_reset_get(&accel_dev, &rst);
	} while (rst);

	lsm6dso_i3c_disable_set(&accel_dev, LSM6DSO_I3C_DISABLE);			// Disable I3C interface
	lsm6dso_block_data_update_set(&accel_dev, PROPERTY_ENABLE);			// Enable Block Data Update
	lsm6dso_xl_full_scale_set(&accel_dev, LSM6DSO_2g);					// Set full scale (sensitivity) to 2g
	if(ACC_DATA_RATE == 3333)
		lsm6dso_xl_data_rate_set(&accel_dev, LSM6DSO_XL_ODR_3333Hz);	// Set Output Data Rate to 3333 Hz
	else if(ACC_DATA_RATE == 1667)
		lsm6dso_xl_data_rate_set(&accel_dev, LSM6DSO_XL_ODR_1667Hz);	// Set Output Data Rate to 1667 Hz
	else
		lsm6dso_xl_data_rate_set(&accel_dev, LSM6DSO_XL_ODR_833Hz);		// Set Output Data Rate to 833 Hz
}

/*
 * Print accelerometer data buffer
 */
void print_acc_data(void)
{
	static char acc_buffer_mg_uint8[8 * DATA_INPUT_USER * AXIS_NUMBER + 1];
	for(uint32_t buf_index = 0 ; buf_index < DATA_INPUT_USER * AXIS_NUMBER ; buf_index++) {
		snprintf(acc_buffer_mg_uint8 + 8*buf_index, 8, "%.5f", acc_buffer_mg[buf_index]);
		*(acc_buffer_mg_uint8 + 8*buf_index + 7) = ' ';
	}
	*(acc_buffer_mg_uint8 + 8*(DATA_INPUT_USER * AXIS_NUMBER)) = '\n';
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)acc_buffer_mg_uint8, sizeof(acc_buffer_mg_uint8));
}

/*
 * Draw X and Y axes to display similarity graph
 */
void draw_similarity_graph_axes(void)
{
	// Draw X-axis (line + arrow)
	BSP_LCD_DrawLine(LCD_INSTANCE, 8, 233, 230, 233, LCD_COLOR_WHITE);
	BSP_LCD_DrawLine(LCD_INSTANCE, 230, 233, 227, 236, LCD_COLOR_WHITE);
	BSP_LCD_DrawLine(LCD_INSTANCE, 230, 233, 227, 230, LCD_COLOR_WHITE);

	// Draw Y-axis (line + arrow)
	BSP_LCD_DrawLine(LCD_INSTANCE, 11, 236, 11, 175, LCD_COLOR_WHITE);
	BSP_LCD_DrawLine(LCD_INSTANCE, 11, 175, 8, 178, LCD_COLOR_WHITE);
	BSP_LCD_DrawLine(LCD_INSTANCE, 11, 175, 14, 178, LCD_COLOR_WHITE);
}

/*
 * Display the similarity graph
 */
void display_similarity_graph(void)
{
	static uint8_t pix[43][49] = {{0}};
	static uint8_t previous_x_pix = 50, previous_y_pix = 0;
	static float last_y_pix = 0.0;

	BSP_LCD_FillRect(LCD_INSTANCE, 13, 182, 212, 49, BSP_LCD_GetBackColor());
	pix[42][(uint32_t)last_y_pix] = 0;
	BSP_LCD_DrawLine(LCD_INSTANCE, 220, 231 - (uint32_t)last_y_pix, 224, 231 - (uint32_t)last_y_pix, BSP_LCD_GetBackColor());
	last_y_pix = map((float)similarity, 0.0, 100.0, 0.0, 48.0);
	pix[42][(uint32_t)last_y_pix] = 1;
	BSP_LCD_DrawLine(LCD_INSTANCE, 220, 231 - (uint32_t)last_y_pix, 224, 231 - (uint32_t)last_y_pix, LCD_COLOR_WHITE);
	for(uint8_t x_pix = 0 ; x_pix < 42 ; x_pix++) {
		for(uint8_t y_pix = 0 ; y_pix < 49 ; y_pix++) {
			pix[x_pix][y_pix] = pix[x_pix + 1][y_pix];
			if(pix[x_pix][y_pix] == 1) {
				BSP_LCD_DrawLine(LCD_INSTANCE, 13 + 5*x_pix, 231 - y_pix, 13 + 5*x_pix+4, 231 - y_pix, LCD_COLOR_WHITE);
				BSP_LCD_DrawLine(LCD_INSTANCE, 13 + 5*x_pix+5, 231 - y_pix, 13 + 5*x_pix+9, 231 - y_pix, BSP_LCD_GetBackColor());
				if(x_pix > previous_x_pix && previous_x_pix < 41)
					BSP_LCD_DrawLine(LCD_INSTANCE, 13 + 5*previous_x_pix+4, 231 - previous_y_pix, 13 + 5*x_pix, 231 - y_pix, LCD_COLOR_WHITE);
				previous_x_pix = x_pix;
				previous_y_pix = y_pix;
			}
		}
	}
}

/*
 * Get first half of accelerometer data buffer
 */
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1) {
		axis3bit16_t acc_buffer_raw;
		for(uint16_t data_index = 0 ; data_index < DATA_INPUT_USER / 2 ; data_index++) {
			static uint8_t acc_flag_data_ready = 0;
			do {
				lsm6dso_xl_flag_data_ready_get(&accel_dev, &acc_flag_data_ready);
			} while(!acc_flag_data_ready);
			memset(acc_buffer_raw.i16bit, 0x00, AXIS_NUMBER * sizeof(int16_t));
			lsm6dso_acceleration_raw_get(&accel_dev, acc_buffer_raw.i16bit);
			acc_buffer_mg[AXIS_NUMBER * data_index] = lsm6dso_from_fs2_to_mg(acc_buffer_raw.i16bit[0]);
			acc_buffer_mg[AXIS_NUMBER * data_index + 1] = lsm6dso_from_fs2_to_mg(acc_buffer_raw.i16bit[1]);
			acc_buffer_mg[AXIS_NUMBER * data_index + 2] = lsm6dso_from_fs2_to_mg(acc_buffer_raw.i16bit[2]);
		}
	}
}

/*
 * Get second half of accelerometer data buffer
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1) {
		axis3bit16_t acc_buffer_raw;
		for(uint16_t data_index = DATA_INPUT_USER / 2 ; data_index < DATA_INPUT_USER ; data_index++) {
			static uint8_t acc_flag_data_ready = 0;
			do {
				lsm6dso_xl_flag_data_ready_get(&accel_dev, &acc_flag_data_ready);
			} while(!acc_flag_data_ready);
			memset(acc_buffer_raw.i16bit, 0x00, AXIS_NUMBER * sizeof(int16_t));
			lsm6dso_acceleration_raw_get(&accel_dev, acc_buffer_raw.i16bit);
			acc_buffer_mg[AXIS_NUMBER * data_index] = lsm6dso_from_fs2_to_mg(acc_buffer_raw.i16bit[0]);
			acc_buffer_mg[AXIS_NUMBER * data_index + 1] = lsm6dso_from_fs2_to_mg(acc_buffer_raw.i16bit[1]);
			acc_buffer_mg[AXIS_NUMBER * data_index + 2] = lsm6dso_from_fs2_to_mg(acc_buffer_raw.i16bit[2]);
		}
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(lcd_display == DETECTION_CPU) {
		if(neai_state == LEARNING) {
#if !DATALOG
			if(learn_iteration == 0 || previous_lcd_display != lcd_display) {
				BSP_LCD_Clear(LCD_INSTANCE, LCD_COLOR_ST_BLUE_DARK);
				LCD_Set_Text_Display(LCD_COLOR_ST_BLUE_DARK, LCD_COLOR_ST_YELLOW, &Font24);
				sprintf((char *)display_string, "Learning");
				BSP_LCD_DisplayStringAtLine(LCD_INSTANCE, 1, display_string, CENTER_MODE);
				sprintf((char *)display_string, "in progress");
				BSP_LCD_DisplayStringAtLine(LCD_INSTANCE, 2, display_string, CENTER_MODE);
				LCD_Set_Text_Display(LCD_COLOR_ST_BLUE_DARK, LCD_COLOR_ST_YELLOW, &Font20);
				BSP_LCD_DrawRect(LCD_INSTANCE, 20, 150, 200, 30, LCD_COLOR_ST_YELLOW);
			}
#endif
			if(learn_iteration < LEARNING_ITERATIONS) {
		    	HAL_UART_TxHalfCpltCallback(&huart1);
		    	HAL_UART_TxCpltCallback(&huart1);
#if DATALOG
				print_acc_data();
				BSP_LCD_Clear(LCD_INSTANCE, LCD_COLOR_ST_BLUE_DARK);
				LCD_Set_Text_Display(LCD_COLOR_ST_BLUE_DARK, LCD_COLOR_ST_YELLOW, &Font24);
				sprintf((char *)display_string, "DATALOG");
				BSP_LCD_ClearStringLine(LCD_INSTANCE, 4);
				BSP_LCD_DisplayStringAtLine(LCD_INSTANCE, 4, display_string, CENTER_MODE);
				sprintf((char *)display_string, "%d/%d", learn_iteration+1, LEARNING_ITERATIONS);
				BSP_LCD_ClearStringLine(LCD_INSTANCE, 6);
				BSP_LCD_DisplayStringAtLine(LCD_INSTANCE, 6, display_string, CENTER_MODE);
#else
				sprintf((char *)display_string, "%d/%d", learn_iteration+1, LEARNING_ITERATIONS);
				BSP_LCD_ClearStringLine(LCD_INSTANCE, 4);
				BSP_LCD_DisplayStringAtLine(LCD_INSTANCE, 4, display_string, CENTER_MODE);
				neai_anomalydetection_learn(acc_buffer_mg);
				printf("Learn %d/%d\r\n", learn_iteration+1, LEARNING_ITERATIONS);
#endif
				learn_iteration++;
			} else {
				neai_state = DETECTION;
				__HAL_TIM_SET_AUTORELOAD(&htim6, 9999);
			}
		} else if(neai_state == DETECTION) {
#if !DATALOG
			HAL_UART_TxHalfCpltCallback(&huart1);
			HAL_UART_TxCpltCallback(&huart1);
			previous_similarity = similarity;
			neai_anomalydetection_detect(acc_buffer_mg, &similarity);
			printf("Similarity = %d%%\r\n", similarity);
			if(similarity < NEAI_THRESHOLD && (previous_similarity >= NEAI_THRESHOLD || previous_lcd_display != lcd_display)) {
				BSP_LCD_Clear(LCD_INSTANCE, LCD_COLOR_DARKRED);
				LCD_Set_Text_Display(LCD_COLOR_DARKRED, LCD_COLOR_ST_YELLOW, &Font24);
				sprintf((char *)display_string, "Detection");
				BSP_LCD_DisplayStringAtLine(LCD_INSTANCE, 1, display_string, CENTER_MODE);
				sprintf((char *)display_string, "in progress");
				BSP_LCD_DisplayStringAtLine(LCD_INSTANCE, 2, display_string, CENTER_MODE);
				sprintf((char *)display_string, "ANOMALY");
				BSP_LCD_DisplayStringAt(LCD_INSTANCE, 0, 110, display_string, CENTER_MODE);
				LCD_Set_Text_Display(LCD_COLOR_DARKRED, LCD_COLOR_ST_YELLOW, &Font20);
				draw_similarity_graph_axes();
			} else if (similarity >= NEAI_THRESHOLD && (previous_similarity < NEAI_THRESHOLD || previous_lcd_display != lcd_display)) {
				BSP_LCD_Clear(LCD_INSTANCE, LCD_COLOR_DARKGREEN);
				LCD_Set_Text_Display(LCD_COLOR_DARKGREEN, LCD_COLOR_ST_YELLOW, &Font24);
				sprintf((char *)display_string, "Detection");
				BSP_LCD_DisplayStringAtLine(LCD_INSTANCE, 1, display_string, CENTER_MODE);
				sprintf((char *)display_string, "in progress");
				BSP_LCD_DisplayStringAtLine(LCD_INSTANCE, 2, display_string, CENTER_MODE);
				sprintf((char *)display_string, "NOMINAL");
				BSP_LCD_DisplayStringAt(LCD_INSTANCE, 0, 110, display_string, CENTER_MODE);
				LCD_Set_Text_Display(LCD_COLOR_DARKGREEN, LCD_COLOR_ST_YELLOW, &Font20);
				draw_similarity_graph_axes();
			}
			sprintf((char *)display_string, "Similarity = %d%%", similarity);
			BSP_LCD_ClearStringLine(LCD_INSTANCE, 7);
			BSP_LCD_DisplayStringAtLine(LCD_INSTANCE, 7, display_string, CENTER_MODE);
#endif
		}
#if !DATALOG
		if(neai_state == LEARNING)
			BSP_LCD_FillRect(LCD_INSTANCE, 20, 150, (uint32_t)map((float)learn_iteration, 0, LEARNING_ITERATIONS, 0, 200), 30, LCD_COLOR_ST_YELLOW);
		else if(neai_state == DETECTION)
			display_similarity_graph();
#endif
	} else {
		BSP_LCD_FillRect(LCD_INSTANCE, 0, 0, 240, 30, LCD_COLOR_ST_BLUE_LIGHT);
		LCD_Set_Text_Display(LCD_COLOR_ST_BLUE_LIGHT, LCD_COLOR_BLACK, &Font20);
		sprintf((char *)display_string, "NanoEdge AI lib");
		BSP_LCD_DisplayStringAt(LCD_INSTANCE, 0, 7, display_string, CENTER_MODE);

		BSP_LCD_FillRect(LCD_INSTANCE, 0, 30, 120, 70, LCD_COLOR_ST_BLUE_DARK);
		LCD_Set_Text_Display(LCD_COLOR_ST_BLUE_DARK, LCD_COLOR_ST_YELLOW, &Font16);
		sprintf((char *)display_string, "Accuracy");
		BSP_LCD_DisplayStringAt(LCD_INSTANCE, 15, 40, display_string, LEFT_MODE);
		LCD_Set_Text_Display(LCD_COLOR_ST_BLUE_DARK, LCD_COLOR_ST_YELLOW, &Font20);
		sprintf((char *)display_string, "100.0%%");
		BSP_LCD_DisplayStringAt(LCD_INSTANCE, 18, 65, display_string, LEFT_MODE);

		BSP_LCD_FillRect(LCD_INSTANCE, 120, 30, 120, 70, LCD_COLOR_ST_BLUE);
		LCD_Set_Text_Display(LCD_COLOR_ST_BLUE, LCD_COLOR_ST_YELLOW, &Font16);
		sprintf((char *)display_string, "Confidence");
		BSP_LCD_DisplayStringAt(LCD_INSTANCE, 125, 40, display_string, LEFT_MODE);
		LCD_Set_Text_Display(LCD_COLOR_ST_BLUE, LCD_COLOR_ST_YELLOW, &Font20);
		sprintf((char *)display_string, "100.0%%");
		BSP_LCD_DisplayStringAt(LCD_INSTANCE, 140, 65, display_string, LEFT_MODE);

		BSP_LCD_FillRect(LCD_INSTANCE, 0, 100, 120, 70, LCD_COLOR_RED);
		LCD_Set_Text_Display(LCD_COLOR_RED, LCD_COLOR_ST_YELLOW, &Font16);
		sprintf((char *)display_string, "RAM");
		BSP_LCD_DisplayStringAt(LCD_INSTANCE, 42, 110, display_string, LEFT_MODE);
		LCD_Set_Text_Display(LCD_COLOR_RED, LCD_COLOR_ST_YELLOW, &Font20);
		sprintf((char *)display_string, "2.8kB");
		BSP_LCD_DisplayStringAt(LCD_INSTANCE, 25, 130, display_string, LEFT_MODE);
		LCD_Set_Text_Display(LCD_COLOR_RED, LCD_COLOR_ST_YELLOW, &Font12);
		sprintf((char *)display_string, "+ Buffer 3.1kB");
		BSP_LCD_DisplayStringAt(LCD_INSTANCE, 12, 150, display_string, LEFT_MODE);

		BSP_LCD_FillRect(LCD_INSTANCE, 120, 100, 120, 70, LCD_COLOR_ORANGE);
		LCD_Set_Text_Display(LCD_COLOR_ORANGE, LCD_COLOR_ST_YELLOW, &Font16);
		sprintf((char *)display_string, "Flash");
		BSP_LCD_DisplayStringAt(LCD_INSTANCE, 154, 110, display_string, LEFT_MODE);
		LCD_Set_Text_Display(LCD_COLOR_ORANGE, LCD_COLOR_ST_YELLOW, &Font20);
		sprintf((char *)display_string, "3.6kB");
		BSP_LCD_DisplayStringAt(LCD_INSTANCE, 145, 135, display_string, LEFT_MODE);


		BSP_LCD_FillRect(LCD_INSTANCE, 0, 170, 240, 70, LCD_COLOR_DARKGRAY);
		LCD_Set_Text_Display(LCD_COLOR_DARKGRAY, LCD_COLOR_ST_YELLOW, &Font16);
		sprintf((char *)display_string, "CPU clock cycles");
		BSP_LCD_DisplayStringAt(LCD_INSTANCE, 0, 178, display_string, CENTER_MODE);
		sprintf((char *)display_string, "Learn: 619865");
		BSP_LCD_DisplayStringAt(LCD_INSTANCE, 5, 202, display_string, LEFT_MODE);
		sprintf((char *)display_string, "1.13%%");
		BSP_LCD_DisplayStringAt(LCD_INSTANCE, 175, 202, display_string, LEFT_MODE);
		BSP_LCD_DrawRect(LCD_INSTANCE, 173, 200, 59, 16, LCD_COLOR_ST_YELLOW);
		sprintf((char *)display_string, "Detect:616990");
		BSP_LCD_DisplayStringAt(LCD_INSTANCE, 5, 222, display_string, LEFT_MODE);
		sprintf((char *)display_string, "0.56%%");
		BSP_LCD_DisplayStringAt(LCD_INSTANCE, 175, 222, display_string, LEFT_MODE);
		BSP_LCD_DrawRect(LCD_INSTANCE, 173, 220, 59, 16, LCD_COLOR_ST_YELLOW);

		HAL_TIM_Base_Stop_IT(&htim6);
	}
	previous_lcd_display = lcd_display;
}


/*
 * Function called when a rising edge is detected on push button pin (= button pressed)
 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13) {
		if(lcd_display == DETECTION_CPU) {
			previous_lcd_display = DETECTION_CPU;
			lcd_display = NEAI_STATS;
		} else {
			previous_lcd_display = NEAI_STATS;
			lcd_display = DETECTION_CPU;
			HAL_TIM_Base_Start_IT(&htim6);
		}
	}
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
