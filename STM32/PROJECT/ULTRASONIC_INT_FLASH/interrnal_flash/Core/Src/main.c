/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define FLASH_USER_START_ADDR   0x080FF800// Start address for storing logs in internal flash
#include<string.h>
#include<stdio.h>
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
RTC_HandleTypeDef hrtc;         // RTC handle
TIM_HandleTypeDef htim2;        // Timer handle for input capture
UART_HandleTypeDef huart2;      // UART handle for debugging
/* USER CODE BEGIN PV */
uint32_t ic_val1 = 0;           // First captured timer value
uint32_t ic_val2 = 0;           // Second captured timer value
uint32_t diff = 0;               // Difference between capture values
uint8_t is_first_capture = 0;   // Flag for first capture
float distance_cm = 0;           // Calculated distance in cm
#define THRESHOLD_CM 20.0f       // Threshold distance to detect object
uint8_t object_present = 0;      // Current object state
uint8_t object_previous = 0;     // Previous object state
char msg[64];                    // General message buffer
RTC_TimeTypeDef sTime;           // RTC time
RTC_DateTypeDef sDate;           // RTC date
char timestamp[32];              // String timestamp
uint32_t current_address = FLASH_USER_START_ADDR;  // Current flash write address

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Trigger HC-SR04 ultrasonic sensor
void HCSR04_Trigger(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  // TRIG high
  for (volatile int i = 0; i < 160; i++);             // ~10 µs delay
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // TRIG low
}
// Start measurement
void Start_HCSR04(void)
{
  is_first_capture = 0;
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);// Start input capture interrupt
  HCSR04_Trigger();
}

// Timer callback for capturing echo pulse
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    if (is_first_capture == 0)  // Rising edge detected
    {
      ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
      is_first_capture = 1;
    }
    else if (is_first_capture == 1) // Falling edge detected
    {
      ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

      HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_1); // Stop timer interrupt

      // Calculate pulse duration
      if (ic_val2 > ic_val1)
        diff = ic_val2 - ic_val1;
      else
        diff = (0xFFFF - ic_val1 + ic_val2);

      distance_cm = (float)diff * 0.0343 / 2.0; // Convert time to distance
    }
  }
}

// Write log string to internal flash
void Write_Timestamp_To_Flash(char *log)
{
    HAL_FLASH_Unlock(); // Unlock flash for writing

    uint64_t *data_ptr = (uint64_t *)log;
    size_t len = strlen(log);
    size_t double_word_count = (len + 7) / 8; // Number of 8-byte chunks

    for (size_t i = 0; i < double_word_count; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, current_address, data_ptr[i]) != HAL_OK)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)"Flash write failed\r\n", 20, HAL_MAX_DELAY);
            break;
        }
        HAL_UART_Transmit(&huart2, (uint8_t *)"Flash write success\r\n", 20, HAL_MAX_DELAY);
        current_address += 8;
        if (current_address + len >= 0x08100000) // Prevent writing beyond flash
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)"Flash full\r\n", 12, HAL_MAX_DELAY);
            HAL_FLASH_Lock();
            return;
        }
    }

    HAL_FLASH_Lock(); // Lock flash after writing
}

// Read and print logs from internal flash
void Read_Logs_From_Flash(void)
{
    char log_buffer[64];
    uint32_t read_addr = FLASH_USER_START_ADDR;

    while (read_addr < current_address)
    {
    	memset(log_buffer, 0, sizeof(log_buffer));
        memcpy(log_buffer, (char *)read_addr, 64);
        HAL_UART_Transmit(&huart2, (uint8_t *)"Read: ", 6, 2000);
        HAL_UART_Transmit(&huart2, (uint8_t *)log_buffer, strlen(log_buffer), HAL_MAX_DELAY);
        read_addr += 64;
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
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
    // Erase flash page for storing logs
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PageError = 0;

  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Page        = ((FLASH_USER_START_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE);
  EraseInitStruct.NbPages     = 1;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
	  HAL_UART_Transmit(&huart2,(uint8_t *)"failed\r\n",8 , 2000);
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    // Get current time and date
	      HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  	  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);  // Must be called right after time
	  	  snprintf(timestamp, sizeof(timestamp), "%02d:%02d:%02d",sTime.Hours, sTime.Minutes, sTime.Seconds);
           // Measure distance
	  	  Start_HCSR04();

	  	   char buffer[50];
	  	   char tx_data1[70];
	  	   snprintf(buffer, sizeof(buffer), "Distance: %.2f cm\r\n", distance_cm);
	  	   HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 2000);
	  	   HAL_Delay(2000);
         // Check if object is present
	  	   if (distance_cm < THRESHOLD_CM)
	     	 	     object_present = 1;
	  	   else
	  	      	 object_present = 0;
        // Detect IN event
	  	   if (object_present && !object_previous)
	  	   {
	  	      	HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
	  	      	snprintf(tx_data1, sizeof(tx_data1), "IN [%s]: %.2f cm\r\n", timestamp, distance_cm);
	  	      	Write_Timestamp_To_Flash(tx_data1);
	  	        HAL_Delay(100);
	  	        Read_Logs_From_Flash();
	  	      }
            // Detect OUT event
	  	     else if (!object_present && object_previous)
	  	     {
	  	      	HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
	  	      	snprintf(tx_data1, sizeof(tx_data1), "OUT [%s]: %.2f cm\r\n", timestamp, distance_cm);
	  	      	Write_Timestamp_To_Flash(tx_data1);
	  	        HAL_Delay(100);
	  	        Read_Logs_From_Flash();
	  	      }

	  	     object_previous = object_present;
	  	     HAL_Delay(100);  // Sampling interval
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
