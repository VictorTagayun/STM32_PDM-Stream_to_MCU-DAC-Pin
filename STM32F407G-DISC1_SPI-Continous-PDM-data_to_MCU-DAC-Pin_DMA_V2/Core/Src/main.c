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
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pdm2pcm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include "pdm_tones.h"

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
 CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t SPI_PDM_rx_data_2k[2000], SPI_PDM_rx_data_4k[4000], SPI_PDM_tx_data_4k[4000];

int16_t PCM_outBuffer[250];
uint16_t PCM_DAC_outBuffer[250];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

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
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  MX_PDM2PCM_Init();
  MX_DAC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

	printf("Starting >> STM32F407G-DISC1_SPI-Continous-PDM-data_to_MCU-DAC-Pin_DMA \n");


	// first bit to MSB - 8bit tone "AA"
	for (uint16_t i = 0; i < sizeof(tone500HzA4_pdmdata) * 4; i++) {
		pdmBuffer8_8000bits_1000bytes_1[i >> 3] = 0xAA;
		//		printf("i = %d,  i >>3 = %d, i mod sizeof(tone500HzA4_pdmdata) = %d \n",i, i >>3, i % sizeof(tone500HzA4_pdmdata));
	}

	// first bit to MSB - 8bit tone500Hz_pdmdata
	for (uint16_t i = 0; i < sizeof(tone500Hz_pdmdata) * 4; i++) {
		pdmBuffer8_8000bits_1000bytes_2[i >> 3] = (pdmBuffer8_8000bits_1000bytes_2[i >> 3] << 1) + tone500Hz_pdmdata[i % sizeof(tone500Hz_pdmdata)];
		//		printf("i = %d,  i >>3 = %d, i mod sizeof(tone500Hz_pdmdata) = %d \n",i, i >>3, i % sizeof(tone500Hz_pdmdata));
	}

	// first bit to MSB - 8bit tone500HzA4_pdmdata
	for (uint16_t i = 0; i < sizeof(tone500HzA4_pdmdata) * 4; i++) {
		pdmBuffer8_8000bits_1000bytes_3[i >> 3] = (pdmBuffer8_8000bits_1000bytes_3[i >> 3] << 1) + tone500HzA4_pdmdata[i % sizeof(tone500HzA4_pdmdata)];
		//		printf("i = %d,  i >>3 = %d, i mod sizeof(tone500HzA4_pdmdata) = %d \n",i, i >>3, i % sizeof(tone500HzA4_pdmdata));
	}

	for (uint16_t i = 0; i < 1000; i++) {
		pdmBuffer8_8000bits_4000bytes[i] = pdmBuffer8_8000bits_1000bytes_1[i]; // 0xAA data
	}

	for (uint16_t i = 0; i < 1000; i++) {
		pdmBuffer8_8000bits_4000bytes[i + 1000] = pdmBuffer8_8000bits_1000bytes_2[i]; // tone500Hz_pdmdata
	}

	for (uint16_t i = 0; i < 1000; i++) {
		pdmBuffer8_8000bits_4000bytes[i + 2000] = pdmBuffer8_8000bits_1000bytes_1[i]; // 0xAA data
	}

	for (uint16_t i = 0; i < 1000; i++) {
		pdmBuffer8_8000bits_4000bytes[i + 3000] = pdmBuffer8_8000bits_1000bytes_3[i]; // tone500HzA4_pdmdata
	}

	for (uint16_t i = 0; i < 4000; i++) {
		SPI_PDM_rx_data_4k[i] = pdmBuffer8_8000bits_4000bytes[i]; // tone500HzA4_pdmdata
	}

//	GPIOD->BSRR = (1 << 15); // Set
//	PDM_Filter(&pdmBuffer8_8000bits_4000bytes[0], &PCM_outBuffer[0], &PDM1_filter_handler); // 2000bytes x 8bit/bytes = 16,000 bit / 64 decimate = 250 int of data
//	GPIOD->BSRR = (1 << (15 + 16)); // Reset
//
//	GPIOD->BSRR = (1 << 15); // Set
//	PDM_Filter(&pdmBuffer8_8000bits_4000bytes[2000], &PCM_outBuffer[250], &PDM1_filter_handler); // 2000bytes x 8bit/bytes = 16,000 bit / 64 decimate = 250 int of data
//	GPIOD->BSRR = (1 << (15 + 16)); // Reset

	for (int cntr = 0; cntr < 250; cntr++) { // 4000bytes x 8bit/bytes = 32,000 bit / 64 decimate = 500 int of data
		PCM_DAC_outBuffer[cntr] = (PCM_outBuffer[cntr] >> 4) + 2048; // convert to 12bit DC offset of DAC data
	}

	HAL_SPI_Transmit_DMA(&hspi1, SPI_PDM_rx_data_4k, 4000);
	HAL_SPI_Receive_DMA(&hspi2, SPI_PDM_rx_data_2k, 2000);

	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) PCM_DAC_outBuffer, 250, DAC_ALIGN_12B_R); // 4000bytes x 8bit/bytes = 32,000 bit / 64 decimate = 500 int of data

	HAL_TIM_Base_Start(&htim2); //Start the timer

	HAL_Delay(100);

	printf("Ending   >> STM32F407G-DISC1_SPI-Continous-PDM-data_to_MCU-DAC-Pin_DMA \n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  RCC_OscInitStruct.PLL.PLLM = 7;
  RCC_OscInitStruct.PLL.PLLN = 344;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_CRC_DR_RESET(&hcrc);
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 64-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the LPUART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);

	return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_TriggerCallback could be implemented in the user file
   */

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

//  if ((timerPeriodElapsedCallbk_cntr % 2) == 0)
//  {
//	  GPIOD->BSRR = (1 << 14); // Set
//	  PDM_Filter(&pdmBuffer8_8000bits_4000bytes[0], &PCM_outBuffer[0], &PDM1_filter_handler); // 2000bytes x 8bit/bytes = 16,000 bit / 64 decimate = 250 int of data
//
//	  for (int cntr = 0; cntr < 250; cntr++) { // 4000bytes x 8bit/bytes = 32,000 bit / 64 decimate = 500 int of data
//			PCM_DAC_outBuffer[cntr] = (PCM_outBuffer[cntr] >> 4) + 2048; // convert to 12bit DC offset of DAC data
//	  }
//	  GPIOD->BSRR = (1 << (14 + 16)); // Reset
//  } else
//  {
//	  GPIOD->BSRR = (1 << 15); // Set
//	  PDM_Filter(&pdmBuffer8_8000bits_4000bytes[2000], &PCM_outBuffer[250], &PDM1_filter_handler); // 2000bytes x 8bit/bytes = 16,000 bit / 64 decimate = 250 int of data
//
//	  for (int cntr = 250; cntr < 500; cntr++) { // 4000bytes x 8bit/bytes = 32,000 bit / 64 decimate = 500 int of data
//			PCM_DAC_outBuffer[cntr] = (PCM_outBuffer[cntr] >> 4) + 2048; // convert to 12bit DC offset of DAC data
//	  }
//	  GPIOD->BSRR = (1 << (15 + 16)); // Reset
//  }
//  timerPeriodElapsedCallbk_cntr++;

}

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_RxHalfCpltCallback() should be implemented in the user file
   */

  	  GPIOD->BSRR = (1 << 14); // Set
  	  // 2000bytes x 8bit/bytes = 16,000 bit / 64 decimate = 250 int of data
  	  PDM_Filter(&SPI_PDM_rx_data_2k[0], &PCM_outBuffer[0], &PDM1_filter_handler);

  	  for (int cntr = 0; cntr < 125; cntr++) { // 4000bytes x 8bit/bytes = 32,000 bit / 64 decimate = 500 int of data
  		  // First 1/2 of received data goes to the second 1/2 of PCM output
  		  PCM_DAC_outBuffer[cntr] = (PCM_outBuffer[cntr] >> 4) + 2048; // convert to 12bit DC offset of DAC data
  	  }
  	  GPIOD->BSRR = (1 << (14 + 16)); // Reset

}

/**
  * @brief  Rx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_RxCpltCallback should be implemented in the user file
   */

  	  GPIOD->BSRR = (1 << 15); // Set
  	  // 2000bytes x 8bit/bytes = 16,000 bit / 64 decimate = 250 int of data
  	  PDM_Filter(&SPI_PDM_rx_data_2k[1000], &PCM_outBuffer[125], &PDM1_filter_handler); // 2000bytes x 8bit/bytes = 16,000 bit / 64 decimate = 250 int of data

  	  for (int cntr = 125; cntr < 250; cntr++) { // 4000bytes x 8bit/bytes = 32,000 bit / 64 decimate = 500 int of data
  		  // Second 1/2 of received data goes to the next First 1/2 of PCM output
  		  PCM_DAC_outBuffer[cntr] = (PCM_outBuffer[cntr] >> 4) + 2048; // convert to 12bit DC offset of DAC data
  	  }
  	  GPIOD->BSRR = (1 << (15 + 16)); // Reset

}

/**
  * @brief  Rx Half Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	printf("Error_Handler \n");
	__disable_irq();
	while (1) {
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
