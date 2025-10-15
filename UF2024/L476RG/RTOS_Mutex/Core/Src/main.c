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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"

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
UART_HandleTypeDef huart2;

osThreadId HPTHandle;
osThreadId MPTHandle;
osThreadId LPTHandle;
osTimerId PDTimerHandle;
osTimerId OTTimerHandle;
osMutexId sem_mutexHandle;
osSemaphoreId bin_semHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void HPT_Task(void const * argument);
void MPT_Task(void const * argument);
void LPT_Task(void const * argument);
void PDCallback(void const * argument);
void OTCallback(void const * argument);

/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of sem_mutex */
  osMutexDef(sem_mutex);
  sem_mutexHandle = osMutexCreate(osMutex(sem_mutex));

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of bin_sem */
  osSemaphoreDef(bin_sem);
  bin_semHandle = osSemaphoreCreate(osSemaphore(bin_sem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of PDTimer */
  osTimerDef(PDTimer, PDCallback);
  PDTimerHandle = osTimerCreate(osTimer(PDTimer), osTimerPeriodic, NULL);

  /* definition and creation of OTTimer */
  osTimerDef(OTTimer, OTCallback);
  OTTimerHandle = osTimerCreate(osTimer(OTTimer), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of HPT */
  osThreadDef(HPT, HPT_Task, osPriorityAboveNormal, 0, 128);
  HPTHandle = osThreadCreate(osThread(HPT), NULL);

  /* definition and creation of MPT */
  osThreadDef(MPT, MPT_Task, osPriorityNormal, 0, 128);
  MPTHandle = osThreadCreate(osThread(MPT), NULL);

  /* definition and creation of LPT */
  osThreadDef(LPT, LPT_Task, osPriorityBelowNormal, 0, 128);
  LPTHandle = osThreadCreate(osThread(LPT), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

/* USER CODE BEGIN Header_HPT_Task */
/**
 * @brief  Function implementing the HPT thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_HPT_Task */
void HPT_Task(void const * argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	osTimerStart(PDTimerHandle, 500);
	for(;;)
	{
		char *str = "HPT task start\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);

		osSemaphoreWait(sem_mutexHandle, osWaitForever);
		HAL_UART_Transmit(&huart2, (uint8_t *)"HPT wait key\n\n", 14, 1000);
		while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
//		while (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
		osSemaphoreRelease(sem_mutexHandle);
		HAL_UART_Transmit(&huart2, (uint8_t *)"HPT release sem\n\n", 17, 1000);


		HAL_UART_Transmit(&huart2, (uint8_t *)"Exit HPT task\n\n", 15, 1000);
		vTaskDelay(500);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_MPT_Task */
/**
 * @brief Function implementing the MPT thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_MPT_Task */
void MPT_Task(void const * argument)
{
  /* USER CODE BEGIN MPT_Task */
	/* Infinite loop */
	for(;;)
	{
		char *str = "MPT task start\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);

		HAL_UART_Transmit(&huart2, (uint8_t *)"Exit MPT task\n\n", 15, 1000);
		vTaskDelay(2000);
	}
  /* USER CODE END MPT_Task */
}

/* USER CODE BEGIN Header_LPT_Task */
/**
 * @brief Function implementing the LPT thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LPT_Task */
void LPT_Task(void const * argument)
{
  /* USER CODE BEGIN LPT_Task */
	/* Infinite loop */
	for(;;)
	{
		char *str = "LPT task start\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);

		osSemaphoreWait(sem_mutexHandle, osWaitForever);
		HAL_UART_Transmit(&huart2, (uint8_t *)"LPT wait key\n\n", 14, 1000);
		while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
		osSemaphoreRelease(sem_mutexHandle);
		HAL_UART_Transmit(&huart2, (uint8_t *)"LPT release sem\n\n", 17, 1000);

		HAL_UART_Transmit(&huart2, (uint8_t *)"Exit LPT task\n\n", 15, 1000);
		vTaskDelay(3000);
	}
  /* USER CODE END LPT_Task */
}

/* PDCallback function */
void PDCallback(void const * argument)
{
  /* USER CODE BEGIN PDCallback */
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

  /* USER CODE END PDCallback */
}

/* OTCallback function */
void OTCallback(void const * argument)
{
  /* USER CODE BEGIN OTCallback */

  /* USER CODE END OTCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
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
