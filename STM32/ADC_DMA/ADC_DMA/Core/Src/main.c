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
#include<stdio.h>
#include "string.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t adc_val = 0; // Variable to store ADC value

// Callback function called when ADC conversion is complete (DMA mode)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1) // Check if callback is for ADC1
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Toggle LED2 on each ADC update
  }
}
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  HAL_Init(); // Initialize HAL library

  SystemClock_Config(); // Configure system clock

  MX_GPIO_Init();       // Initialize GPIO
  MX_DMA_Init();        // Initialize DMA
  MX_USART2_UART_Init();// Initialize UART2
  MX_ADC1_Init();       // Initialize ADC1

  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, &adc_val, 1); // Start ADC1 in DMA mode to store result in adc_val
  /* USER CODE END 2 */

  char msg[50]; // Buffer to store UART message

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Format ADC value into string
    snprintf(msg, sizeof(msg), "ADC: %lu\r\n", adc_val);

    // Send ADC value string over UART2
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    HAL_Delay(500); // Delay 500 ms before next transmission
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
/* ADC1 Initialization Function */
static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;                        // Use ADC1
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1; // Set ADC clock
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;       // 12-bit resolution
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;       // Right-aligned data
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;       // Single channel scan
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;    // End of conversion after each conversion
  hadc1.Init.LowPowerAutoWait = DISABLE;           // No auto wait
  hadc1.Init.ContinuousConvMode = DISABLE;         // Single conversion mode (DMA handles continuous)
  hadc1.Init.NbrOfConversion = 1;                  // Only one channel
  hadc1.Init.DiscontinuousConvMode = DISABLE;      // No discontinuous mode
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START; // Software trigger
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;      // Enable continuous DMA requests
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;    // Preserve old data on overrun
  hadc1.Init.OversamplingMode = DISABLE;          // No oversampling

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler(); // Handle ADC init error
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;          // Independent ADC mode
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler(); // Handle multimode config error
  }

  sConfig.Channel = ADC_CHANNEL_5;               // ADC channel 5
  sConfig.Rank = ADC_REGULAR_RANK_1;             // First in regular sequence
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5; // Sampling time
  sConfig.SingleDiff = ADC_SINGLE_ENDED;         // Single-ended input
  sConfig.OffsetNumber = ADC_OFFSET_NONE;        // No offset
  sConfig.Offset = 0;                            

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler(); // Handle channel config error
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
/* USART2 Initialization Function */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;              // UART2 instance
  huart2.Init.BaudRate = 115200;         // Baud rate 115200
  huart2.Init.WordLength = UART_WORDLENGTH_8B; // 8-bit word
  huart2.Init.StopBits = UART_STOPBITS_1;      // 1 stop bit
  huart2.Init.Parity = UART_PARITY_NONE;       // No parity
  huart2.Init.Mode = UART_MODE_TX_RX;          // TX and RX enabled
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE; // No hardware flow control
  huart2.Init.OverSampling = UART_OVERSAMPLING_16; // Oversampling 16
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler(); // Handle UART init error
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
