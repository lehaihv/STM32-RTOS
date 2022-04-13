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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "ssd1306.h"
#include "fonts.h"
#include "test.h"
#include "bitmap.h"
#include "horse_anim.h"
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

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "Task1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task2c */
osThreadId_t Task2cHandle;
const osThreadAttr_t Task2c_attributes = {
  .name = "Task2c",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
void StartTask1(void *argument);
void StartTask2(void *argument);

/* USER CODE BEGIN PFP */
void Task_action(int  ADC_INPUT);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ADC_Select_CH1 (void)
  {
  	  ADC_ChannelConfTypeDef sConfig = {0};
  	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  	  */
  	  sConfig.Channel = ADC_CHANNEL_1;
  	  sConfig.Rank = 1;
  	  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;;
  	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  	  {
  	    Error_Handler();
  	  }
  }

  void ADC_Select_CH2 (void)
  {
  	  ADC_ChannelConfTypeDef sConfig = {0};
  	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  	  */
  	  sConfig.Channel = ADC_CHANNEL_2;
  	  sConfig.Rank = 1;
  	  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;;
  	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  	  {
  	    Error_Handler();
  	  }
  }

  void ADC_Select_CHTemp (void)
  {
  	  ADC_ChannelConfTypeDef sConfig = {0};
  	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  	  */
  	  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  	  sConfig.Rank = 1;
  	  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;;
  	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  	  {
  	    Error_Handler();
  	  }
  }

uint16_t ADC_VAL[3];//1, value_adc2;// = 4096; // adc_value
float Temp = 0;
#define Avg_Slope .0025
#define V25 0.76
char msg[5];
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
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_Init (); // initialize the diaply
  SSD1306_GotoXY (0,0); // goto 10, 10
  SSD1306_Puts ("CH1:  ", &Font_11x18, 1); // print Voltmeter_Back to Intel Nuc
  SSD1306_UpdateScreen(); // update screen
  //HAL_Delay(3000);
  SSD1306_GotoXY (0,20);
  SSD1306_Puts ("CH2:  ", &Font_11x18, 1);
  SSD1306_GotoXY (0,40);
  SSD1306_Puts ("Temp:  ", &Font_11x18, 1);
  SSD1306_UpdateScreen(); // update screen
  // Get ADC value
  //HAL_ADC_Start(&hadc1);
  //HAL_ADC_Start(&hadc2);
  //HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  //value_adc1 = HAL_ADC_GetValue(&hadc1);
  //value_adc2 = HAL_ADC_GetValue(&hadc2);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task1 */
  Task1Handle = osThreadNew(StartTask1, NULL, &Task1_attributes);

  /* creation of Task2c */
  Task2cHandle = osThreadNew(StartTask2, NULL, &Task2c_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x00702991;
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

}

/* USER CODE BEGIN 4 */
void Task_action(int ADC_INPUT)
{
	/*if (ADC_INPUT == 1) {
		ADC_Select_CH1();
	    HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		ADC_VAL[0] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		sprintf(msg,"%hu\r\n",ADC_VAL[0]);
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		SSD1306_GotoXY (60,0); // goto 10, 10
				SSD1306_Putc ((char)(ADC_VAL[0]/1000+48), &Font_11x18, 1);
				SSD1306_Putc ((char)((ADC_VAL[0]%1000)/100+48), &Font_11x18, 1);
				SSD1306_Putc ((char)((ADC_VAL[0]%100)/10+48), &Font_11x18, 1);
				SSD1306_Putc ((char)(ADC_VAL[0]%10+48), &Font_11x18, 1);
				SSD1306_UpdateScreen(); // update screen
				HAL_Delay(200);
	}
	if (ADC_INPUT == 2) {
		ADC_Select_CH2();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		ADC_VAL[1] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		ADC_Select_CHTemp();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		ADC_VAL[2] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		Temp = ((3.3*ADC_VAL[2]/4095 - V25)/Avg_Slope)+25;

	    sprintf(msg,"%hu\r\n",ADC_VAL[1]);
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

		sprintf(msg,"%hu\r\n",(uint16_t)Temp);
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

		SSD1306_GotoXY (60,20); // goto 10, 10
				SSD1306_Putc ((char)(ADC_VAL[1]/1000+48), &Font_11x18, 1);
				SSD1306_Putc ((char)((ADC_VAL[1]%1000)/100+48), &Font_11x18, 1);
				SSD1306_Putc ((char)((ADC_VAL[1]%100)/10+48), &Font_11x18, 1);
				SSD1306_Putc ((char)(ADC_VAL[1]%10+48), &Font_11x18, 1);

				SSD1306_GotoXY (60,42); // goto 10, 10
				SSD1306_Putc ((char)((int)(Temp)/100+48), &Font_11x18, 1);
				SSD1306_Putc ((char)(((int)(Temp)%100)/10+48), &Font_11x18, 1);
				SSD1306_Puts (".", &Font_11x18, 1);
				SSD1306_Putc ((char)((int)(Temp)%10+48), &Font_11x18, 1);
				SSD1306_UpdateScreen(); // update screen
				HAL_Delay(200);
	}*/
	//////////////////
	switch (ADC_INPUT)
	{
	case 1:
		ADC_Select_CH1();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		ADC_VAL[0] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		sprintf(msg,"%hu\r\n",ADC_VAL[0]);
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		SSD1306_GotoXY (60,0); // goto 10, 10
		SSD1306_Putc ((char)(ADC_VAL[0]/1000+48), &Font_11x18, 1);
		SSD1306_Putc ((char)((ADC_VAL[0]%1000)/100+48), &Font_11x18, 1);
		SSD1306_Putc ((char)((ADC_VAL[0]%100)/10+48), &Font_11x18, 1);
		SSD1306_Putc ((char)(ADC_VAL[0]%10+48), &Font_11x18, 1);
		SSD1306_UpdateScreen(); // update screen
		break;
	case 2:
		ADC_Select_CH2();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		ADC_VAL[1] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		ADC_Select_CHTemp();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		ADC_VAL[2] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		Temp = ((3.3*ADC_VAL[2]/4095 - V25)/Avg_Slope)+25;

		sprintf(msg,"%hu\r\n",ADC_VAL[1]);
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

		sprintf(msg,"%hu\r\n",(uint16_t)Temp);
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

		SSD1306_GotoXY (60,20); // goto 10, 10
		SSD1306_Putc ((char)(ADC_VAL[1]/1000+48), &Font_11x18, 1);
		SSD1306_Putc ((char)((ADC_VAL[1]%1000)/100+48), &Font_11x18, 1);
		SSD1306_Putc ((char)((ADC_VAL[1]%100)/10+48), &Font_11x18, 1);
		SSD1306_Putc ((char)(ADC_VAL[1]%10+48), &Font_11x18, 1);

		SSD1306_GotoXY (60,40); // goto 10, 10
		SSD1306_Putc ((char)((int)(Temp)/100+48), &Font_11x18, 1);
		SSD1306_Putc ((char)(((int)(Temp)%100)/10+48), &Font_11x18, 1);
		SSD1306_Puts (".", &Font_11x18, 1);
		SSD1306_Putc ((char)((int)(Temp)%10+48), &Font_11x18, 1);
		SSD1306_UpdateScreen(); // update screen
		break;
	default:
		break;
	}
	//SSD1306_UpdateScreen(); // update screen
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask1 */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask1 */
void StartTask1(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	Task_action(1);
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask2 */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask2 */
void StartTask2(void *argument)
{
  /* USER CODE BEGIN StartTask2 */
  /* Infinite loop */
  for(;;)
  {
	Task_action(2);
    osDelay(2000);
  }
  /* USER CODE END StartTask2 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
