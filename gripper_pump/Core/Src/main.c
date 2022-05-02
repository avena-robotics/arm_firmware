/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "fsm.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct Counters_Handle {
	uint32_t main_loop;
	uint32_t timer6;
	uint32_t timer7;
	uint32_t adc2_counter;
	uint32_t can_rx_counter;
	uint32_t can_tx_counter;
	uint32_t spi_rx_counter;
	uint32_t spi_tx_counter;
	uint32_t spi_txrx_counter;
} __attribute__ ((packed)) Counters_Handle_t;

typedef struct Gripper_Pump_Configuration_Handle {
	uint8_t can_node_id;
	bool safety_enabled;
//	bool canbus_enabled;
//	bool canbus_watchdog_enabled;
} __attribute__ ((packed)) Gripper_Pump_Configuration_Handle_t;

typedef struct Gripper_Pump_Command_Handle {
	bool enable_pump;			//
} __attribute__ ((packed)) Gripper_Pump_Command_Handle_t;
typedef struct _Status_Handle {
	uint16_t pump_current;
	uint16_t pump_voltage;
	bool b_pump_is_working; // current over "0" limit
	bool b_pressure_level_1; // input from sensor - level 1
	bool b_pressure_level_2; // input from sensor - level 2

	uint8_t errors;
	uint8_t warnings;
} __attribute__ ((packed)) Gripper_Pump_Status_Handle_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc2;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FDCAN_RxHeaderTypeDef can_rx_header; // CAN Bus Transmit Header
FDCAN_TxHeaderTypeDef can_tx_header; // CAN Bus Transmit Header
uint8_t can_rx_data[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  //CAN Bus Receive Buffer
uint8_t can_tx_data[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  //CAN Bus Send Buffer


Gripper_Pump_Configuration_Handle_t g_gripper_pump_configuration = {
	.safety_enabled = false,
	.can_node_id = 0x06,
};

volatile Counters_Handle_t g_counters = {
	.main_loop = 0,
	.timer6 = 0,
	.timer7 = 0,
	.spi_rx_counter = 0,
	.spi_tx_counter = 0,
	.spi_txrx_counter = 0,
};

volatile FSMStatus_t 	g_fsm_status = {
	.state = FSM_START,
	.state_is_running = false,
	.transition_is_running = false,
};

Gripper_Pump_Command_Handle_t g_gripper_pump_command = {
	.enable_pump = false,
};
volatile Gripper_Pump_Status_Handle_t 	g_gripper_pump_status = {

};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Gripper_Pump_Init(void);
static void setPWM(TIM_HandleTypeDef, uint32_t, uint16_t, uint16_t); 
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
  MX_ADC2_Init();
  MX_FDCAN1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	Gripper_Pump_Init();
//	HAL_TIM_Base_Start_IT(&htim2);
//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		TIM2->CCR1 = 30;
//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);		
		g_counters.main_loop++;
		 //TIM1->CCR1 = 30;
//		TIM2->CCR1 = 30;
//		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
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

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV2;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 12;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 2;
  hfdcan1.Init.DataTimeSeg1 = 12;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 2;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10802D9B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 16999;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 16999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PUMP_ON_Pin|SEC_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_ON_GPIO_Port, LED_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : WEJSCIE_2_Pin WEJSCIE_1_Pin */
  GPIO_InitStruct.Pin = WEJSCIE_2_Pin|WEJSCIE_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PUMP_ON_Pin SEC_OUT_Pin */
  GPIO_InitStruct.Pin = PUMP_ON_Pin|SEC_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_ON_Pin */
  GPIO_InitStruct.Pin = LED_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SEC_IN_Pin */
  GPIO_InitStruct.Pin = SEC_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SEC_IN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse)
{
 HAL_TIM_PWM_Stop(&timer, channel); // stop generation of pwm
 TIM_OC_InitTypeDef sConfigOC;
 timer.Init.Period = period; // set the period duration
 HAL_TIM_PWM_Init(&timer); // reinititialise with new period value
 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 sConfigOC.Pulse = pulse; // set the pulse duration
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
 HAL_TIM_PWM_Start(&timer, channel); // start pwm generation
} 

void Gripper_Pump_Init() {
	// TIMERS
	HAL_TIM_Base_Start_IT(&htim6); 	// Enable 10 kHz timer for fast calculation
	HAL_TIM_Base_Start_IT(&htim7);  	// Enable  1 kHz timer for FSM
}

// CAN FD
void FDCAN_Set_Filters() {
	// CAN FILTERS
	FDCAN_FilterTypeDef   can_filter_config_0; // BROADCAST
	can_filter_config_0.IdType = FDCAN_STANDARD_ID;
	can_filter_config_0.FilterIndex = 0;
	can_filter_config_0.FilterType = FDCAN_FILTER_MASK;
	can_filter_config_0.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	can_filter_config_0.FilterID1 = 0x0F0;
	can_filter_config_0.FilterID2 = 0x70F ;
	HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter_config_0); //Initialize CAN Filter

	FDCAN_FilterTypeDef   can_filter_config_1; // UNICAST
	can_filter_config_1.IdType = FDCAN_STANDARD_ID;
	can_filter_config_1.FilterIndex = 1;
	can_filter_config_1.FilterType = FDCAN_FILTER_MASK;
	can_filter_config_1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	can_filter_config_1.FilterID1 = 0x100 + g_gripper_pump_configuration.can_node_id;
	can_filter_config_1.FilterID2 = 0x70F;
	HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter_config_1); //Initialize CAN Filter

}

// FSM
void FSM_START_Callback() 
{
	HAL_TIM_Base_Stop_IT(&htim6); // Disable 10 kHz timer

//	FLASH_Configuration_Load(); // Read configuration from FLASH

	// FDCAN
	HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 10, 0);
	HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);

	FDCAN_Set_Filters();

	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 3, 3, FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_Start(&hfdcan1); //Initialize CAN Bus
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);// Initialize CAN Bus Rx Interrupt
	HAL_FDCAN_EnableISOMode(&hfdcan1);
	
	// Start ADC readings
	// HAL_ADC_Start_IT(&hadc2);

	HAL_TIM_Base_Start_IT(&htim6); // Enable 10 kHz timer
}

void FSM_READY_TO_OPERATE_Callback() 
{
	g_gripper_pump_command.enable_pump = false;
	HAL_GPIO_WritePin(PUMP_ON_GPIO_Port, PUMP_ON_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_ON_GPIO_Port, LED_ON_Pin, GPIO_PIN_RESET);
	g_gripper_pump_status.b_pressure_level_1 = (HAL_GPIO_ReadPin(WEJSCIE_1_GPIO_Port, WEJSCIE_1_Pin) == GPIO_PIN_SET) ? (0) : (1);
	g_gripper_pump_status.b_pressure_level_2 = (HAL_GPIO_ReadPin(WEJSCIE_2_GPIO_Port, WEJSCIE_2_Pin) == GPIO_PIN_SET) ? (0) : (1);
}

void FSM_OPERATION_ENABLE_Callback() 
{
	HAL_GPIO_WritePin(LED_ON_GPIO_Port, LED_ON_Pin, GPIO_PIN_SET);
	if (g_gripper_pump_command.enable_pump)
	{
		HAL_GPIO_WritePin(PUMP_ON_GPIO_Port, PUMP_ON_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(PUMP_ON_GPIO_Port, PUMP_ON_Pin, GPIO_PIN_RESET);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  // Read & Update The ADC Result
  g_gripper_pump_status.pump_current = HAL_ADC_GetValue(&hadc2);
	g_counters.adc2_counter++;
	HAL_ADC_Start_IT(&hadc2);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{
	// SECURITY ALERT
    if(GPIO_Pin == SEC_IN_Pin && g_gripper_pump_configuration.safety_enabled == true && g_fsm_status.state == FSM_OPERATION_ENABLE) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
			FSM_Activate_Transition(FSM_TRANSITION_OPERATION_ENABLE_TO_READY_TO_OPERATE);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
	if(htim->Instance == TIM2)
	{
		g_counters.spi_tx_counter++;
	}

	if(htim->Instance == TIM6) 	// 10kHz (5,0) - fast recalculation
	{
		g_counters.timer6++;
	}

	if(htim->Instance == TIM7) 	// 1kHz - FSM_Tasks
	{
		//setPWM(htim2, TIM_CHANNEL_1, 255, 100);
		// read
		g_gripper_pump_status.b_pressure_level_1 = (HAL_GPIO_ReadPin(WEJSCIE_1_GPIO_Port, WEJSCIE_1_Pin) == GPIO_PIN_SET) ? (0) : (1);
		g_gripper_pump_status.b_pressure_level_2 = (HAL_GPIO_ReadPin(WEJSCIE_2_GPIO_Port, WEJSCIE_2_Pin) == GPIO_PIN_SET) ? (0) : (1);

		// Start ADC Conversion
    HAL_ADC_Start(&hadc2);
    // Poll ADC1 Perihperal & TimeOut = 1mSec
    HAL_ADC_PollForConversion(&hadc2, 1);
    // Read The ADC Conversion Result & Map It To PWM DutyCycle
    g_gripper_pump_status.pump_current = HAL_ADC_GetValue(&hadc2);
		
		if (g_gripper_pump_status.pump_current > 500)
		{
			g_gripper_pump_status.b_pump_is_working = true;
		}
		else
		{
			g_gripper_pump_status.b_pump_is_working = false;
		}
		
		// FSM
		FSM_Tick();
		
		g_counters.timer7++;
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan1, uint32_t RxFifo0ITs) 
{

	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
	{
		// RETRIEVE CAN MESSAGE -------------------------------------------------------------------------
		HAL_FDCAN_GetRxMessage(hfdcan1, FDCAN_RX_FIFO0, &can_rx_header, can_rx_data);
		g_counters.can_rx_counter++;

		// RESET SAFE TIMER -----------------------------------------------------------------------------
//		HAL_TIM_Base_Stop(&htim6);
//		TIM6->CNT = 0;
//		HAL_TIM_Base_Start_IT(&htim6);

		bool l_send_response = true;
		uint8_t l_cmd 	= (can_rx_header.Identifier & 0x0F0) >> 4; // Ustalenie polecenie
		bool l_unicast 	= (can_rx_header.Identifier & 0x100) >> 8; // Ustalenie typu transmisji: BROADCAST|UNICAST

		// CAN TRANSMITION CONFIGURATION
		can_tx_header.Identifier = can_rx_header.Identifier | g_gripper_pump_configuration.can_node_id | 0x01 << 9;
		can_tx_header.IdType = FDCAN_STANDARD_ID;
		can_tx_header.TxFrameType = FDCAN_DATA_FRAME;
		can_tx_header.DataLength = FDCAN_DLC_BYTES_12;
		can_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		can_tx_header.BitRateSwitch = FDCAN_BRS_ON;
		can_tx_header.FDFormat = FDCAN_FD_CAN;
		can_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
		can_tx_header.MessageMarker = 0;

		// UPDATE JOINT INFO ----------------------------------------------------------------------------
		uint8_t numer_w_szeregu = 0;
		uint8_t dlugosc_danych_polecenia = 0;

		if (!l_unicast)
		{
			numer_w_szeregu = g_gripper_pump_configuration.can_node_id; // ???
		}

		switch (l_cmd) {

			case 0x0: // Wykonaj akcje
			{
				dlugosc_danych_polecenia = 2;
				uint8_t offset = dlugosc_danych_polecenia * numer_w_szeregu;

				g_gripper_pump_command.enable_pump = can_rx_data[offset];

				// -------------------------------------------------------------------------------------------------
				can_tx_data[0] 	= g_gripper_pump_status.b_pump_is_working;
				can_tx_data[1] 	= g_gripper_pump_status.b_pressure_level_1;
				can_tx_data[2] 	= g_gripper_pump_status.b_pressure_level_2;
				can_tx_data[9] 	= FSM_Get_State(); // FSM
				can_tx_data[12] = g_gripper_pump_status.errors;
				can_tx_data[13] = g_gripper_pump_status.warnings;

				can_tx_header.DataLength = FDCAN_DLC_BYTES_20;

				break;
			}
			case 0x1: // Zmien stan FSM
			{
				dlugosc_danych_polecenia = 1;
				// uint8_t - FSM
				uint8_t offset = dlugosc_danych_polecenia * numer_w_szeregu;

				FSM_Set_State(can_rx_data[offset]);

				can_tx_data[0] = FSM_Get_State();

				can_tx_header.DataLength = FDCAN_DLC_BYTES_1;
				break;
			}

			case 0xA: // RESET
			{
				l_send_response = false;
				NVIC_SystemReset();
				break;
			}

//			case 0xB: // SET CAN ID
//			{
//				if (FSM_Get_State() == FSM_INIT)
//				{
//					dlugosc_danych_polecenia = 1;
//					// uint8_t - can id
//					uint8_t offset = dlugosc_danych_polecenia * numer_w_szeregu;
//					g_gripper_pump_configuration.can_node_id = can_rx_data[offset]; // moze byc od 0 do F - TODO: zapis do FLASH i restart

//					// FLASH_Configuration_Save(); // flash configuration
//					FDCAN_Set_Filters(); // reload can filters

//					can_tx_data[0] = 1;
//				}
//				else
//				{
//					can_tx_data[0] = 0;
//				}


//				can_tx_header.DataLength = FDCAN_DLC_BYTES_1;
//				break;
//			}

			case 0xF: // Konfiguracja
			{
				dlugosc_danych_polecenia = 2;
				// uint8_t tryb pracy
				uint8_t offset = dlugosc_danych_polecenia * numer_w_szeregu;
				
				if (FSM_Get_State() == FSM_INIT || FSM_Get_State() == FSM_READY_TO_OPERATE)
				{
					g_gripper_pump_configuration.safety_enabled = (can_rx_data[offset + 1] & 0x04) >> 2;
					can_tx_data[0] = 1;
				}
				else
				{
					can_tx_data[0] = 0;
				}

				can_tx_header.DataLength = FDCAN_DLC_BYTES_1;
				break;
			}

			default:
			{
				l_send_response = false;

			}

		}

		//	SEND FRAME VIA FD CAN -------------------------------------------------------------------------------
		if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan1, &can_tx_header, can_tx_data) == HAL_OK && l_send_response)
		{
			g_counters.can_tx_counter++;
		}

		// clear global rx and tx buffer
		for (int i = 0; i < 24; i++)
		{
			can_rx_data[i]	= 0;
			can_tx_data[i]	= 0;
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
