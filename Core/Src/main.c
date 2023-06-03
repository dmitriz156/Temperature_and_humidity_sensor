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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>
#include "stdio.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "DHT22.h"
#include "max7219_lib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	float temper;
	float humid;
} QUEUE_t;

typedef struct {
	float Temper;
	float Humid;
} Queue_Temp_Hum_t;

typedef struct {
	float max_deviation_t_pos;
	float max_deviation_t_neg;
	float max_deviation_h_pos;
	float max_deviation_h_neg;
} Queue_Dev_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* Definitions for myDHT22Task */
osThreadId_t myDHT22TaskHandle;
const osThreadAttr_t myDHT22Task_attributes = {
  .name = "myDHT22Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myCalculationTa */
osThreadId_t myCalculationTaHandle;
const osThreadAttr_t myCalculationTa_attributes = {
  .name = "myCalculationTa",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myDisplayTask */
osThreadId_t myDisplayTaskHandle;
const osThreadAttr_t myDisplayTask_attributes = {
  .name = "myDisplayTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myDataQueue */
osMessageQueueId_t myDataQueueHandle;
const osMessageQueueAttr_t myDataQueue_attributes = {
  .name = "myDataQueue"
};
/* Definitions for myTempDisplayQueue */
osMessageQueueId_t myTempDisplayQueueHandle;
const osMessageQueueAttr_t myTempDisplayQueue_attributes = {
  .name = "myTempDisplayQueue"
};
/* Definitions for myCalcDisplayQueue */
osMessageQueueId_t myCalcDisplayQueueHandle;
const osMessageQueueAttr_t myCalcDisplayQueue_attributes = {
  .name = "myCalcDisplayQueue"
};
/* USER CODE BEGIN PV */

//float temper, humid;
//float tMid, hMid;
//float tMin = 100, hMin = 100;
//float tMax, hMax;
//float max_deviation_t_pos;
//float max_deviation_t_neg;
//float max_deviation_h_pos;
//float max_deviation_h_neg;
//float value_t_per_t [60] = {0,};
//float value_h_per_t [60] = {0,};
uint8_t seconds_counter = 0; //якщо треба 1 год.
uint8_t counter = 0;
uint16_t time_counter = 0;
uint8_t time_flag = 0;
char LCD_Char[4]={'0','0','0','0'};
uint8_t btn_state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
void StartDHT22Task(void *argument);
void StartCalculationTask(void *argument);
void StartDisplayTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void num_to_str(unsigned int value){
//LCD_Char[3]=((value/1000)%10);
//LCD_Char[2]=((value/100)%10);
//LCD_Char[1]=((value/10)%10);
//LCD_Char[0]=(value%10);
//}

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  max7219_Init();
  HAL_Delay(100);///////////////////////////////////////////////////////////////////////
  HAL_TIM_Base_Start_IT(&htim2);

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

  /* Create the queue(s) */
  /* creation of myDataQueue */
  myDataQueueHandle = osMessageQueueNew (1, sizeof(QUEUE_t), &myDataQueue_attributes);

  /* creation of myTempDisplayQueue */
  myTempDisplayQueueHandle = osMessageQueueNew (1, sizeof(Queue_Temp_Hum_t), &myTempDisplayQueue_attributes);

  /* creation of myCalcDisplayQueue */
  myCalcDisplayQueueHandle = osMessageQueueNew (1, sizeof(Queue_Dev_t), &myCalcDisplayQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of myDHT22Task */
  myDHT22TaskHandle = osThreadNew(StartDHT22Task, NULL, &myDHT22Task_attributes);

  /* creation of myCalculationTa */
  myCalculationTaHandle = osThreadNew(StartCalculationTask, NULL, &myCalculationTa_attributes);

  /* creation of myDisplayTask */
  myDisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &myDisplayTask_attributes);

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

//	  DHT_data d = DHT_getData(DHT22);
//	  temper = d.temp;
//	  humid = d.hum;
////	  printf ("T = %f\r\n", d.temp);
////	  printf ("H = %f\r\n", d.hum);
//	  HAL_Delay(1000);
//	  //////////////////
//	  if(btn_state == 0)
//	  {
//		  max7219_Send_float(temper);
//	  }
//	  else
//	  {
//		  max7219_Send_float(humid);
//	  }

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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 900-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 9000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CS_Pin PA5 */
  GPIO_InitStruct.Pin = SPI_CS_Pin|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13)
	{
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); // сразу же отключаем прерывания на этом пине
		// либо выполняем какое-то действие прямо тут, либо поднимаем флажок
		HAL_TIM_Base_Start_IT(&htim1); // запускаем таймер //200 ms
	}
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//
//	if(htim->Instance == TIM1)
//	{
//		HAL_TIM_Base_Stop_IT(&htim1); // останавливаем таймер
//		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);  // очищаем бит EXTI_PR (бит прерывания)
//		NVIC_ClearPendingIRQ(EXTI15_10_IRQn); // очищаем бит NVIC_ICPRx (бит очереди)
//		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);   // включаем внешнее прерывание
//		//btn_cur = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
//
//		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1)
//		{
//			if(btn_state == 0)
//			{
//				btn_state = 1;
//			}
//			else btn_state = 0;
//		}
//		/*if((btn_prev == 0) && (btn_cur != 0))
//		{
//		}*/
//	}
//}

//void max7219_Transmit(uint8_t adress, uint8_t data){
//	uint8_t tx_buffer[1] = {0};
//	HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin, 0);         //SPI_CS set
//	tx_buffer[0] = adress;
//	HAL_SPI_Transmit(&hspi1, tx_buffer, 1, 0xFFFF);
//	tx_buffer[0] = data;
//	HAL_SPI_Transmit(&hspi1, tx_buffer, 1, 0xFFFF);
//	HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin, 1);         //SPI_CS reset
//}
//
//void max7219_Init (void){
//	max7219_Transmit(0x09, 0x00); //Без режиму декодування
//	max7219_Transmit(0x0A, 0x0F); //Максимальна яскравість
//	max7219_Transmit(0x0B, 0x0B); //Задіюєм всі розряди дисплею
//	max7219_Transmit(0x0C, 0x01); //Виводимо дисплей зі сну
//	max7219_Transmit(0x0F, 0x01); //Тест дисплею
//	HAL_Delay(2000);
//	max7219_Transmit(0x0F, 0x01); //Тест дисплею
//}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDHT22Task */
/**
* @brief Function implementing the myDHT22Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDHT22Task */
void StartDHT22Task(void *argument)
{
  /* USER CODE BEGIN 5 */
	QUEUE_t measure;
	Queue_Temp_Hum_t Measure;

	float temper, humid;
  /* Infinite loop */
  for(;;)
  {
	  DHT_data d = DHT_getData(DHT22);
	  temper = roundf(d.temp * 10)/10;
	  humid = roundf(d.hum * 10)/10;

	  measure.temper = temper;//roundf(d.temp * 10)/10;   /* round(d.temp * 10) / 10;roundf(val * 100) / 100;    Result: 37.77 floorf(val * 100) / 100; ceilf(val * 100) / 100;*/
	  measure.humid = humid;//roundf(d.hum * 10)/10;
	  Measure.Temper = temper;
	  Measure.Humid = humid;
	  osDelay(1000);
	  //HAL_Delay(1000);
	  //xQueueSend(myDataQueueHandle, &measure, 10);
	  osMessageQueuePut(myDataQueueHandle, &measure, 0, osWaitForever);
	  osMessageQueuePut(myTempDisplayQueueHandle, &Measure, 0, osWaitForever);
	  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCalculationTask */
/**
* @brief Function implementing the myCalculationTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCalculationTask */
void StartCalculationTask(void *argument)
{
  /* USER CODE BEGIN StartCalculationTask */
	QUEUE_t measure;
	Queue_Dev_t deviation;

	static uint8_t position;

	static float value_t_per_t [60] = {1000,};
	static float value_h_per_t [60] = {1000,};

	static float tMid, hMid;
	static float tMin = 100, hMin = 100;
	static float tMax, hMax;
	float sum_temper;
	float sum_humid;

	for (int i = 0; i < 60; ++i)
	{
		value_t_per_t [i] = 111;
		value_h_per_t [i] = 111;
	}

  /* Infinite loop */
	for(;;)
	{
		//if(xQueueReceive(myDataQueueHandle, &measure, 10) == pdTRUE)
		if(osMessageQueueGet(myDataQueueHandle, &measure, 0, osWaitForever) == osOK)
		{
			if(time_flag == 1)
			{
				if(seconds_counter < 60)
				{
					seconds_counter ++;
					time_flag = 0;
				}
				else{
					if(counter < 60)
					{
						value_t_per_t [counter] = measure.temper;
						value_h_per_t [counter] = measure.humid;

						//якщо не пройшла година то розраховуються знач. пройденого часу в хв.
						for(int i = 0; i < 60; ++i)
						{
							if(value_t_per_t [i] == 111)
							{
								position = (i-1);
								break;
							}
							else position = 60;
						}

						for(int i = 0; i < position; ++i)
						{
							if(tMax <= value_t_per_t [i])
							{
								tMax = value_t_per_t [i];
							}
							if(hMax <= value_h_per_t [i])
							{
								hMax = value_h_per_t [i];
							}

							if(tMin >= value_t_per_t [i])
							{
								if (value_t_per_t [i] != 0)
									tMin = value_t_per_t [i];
							}
							if(hMin >= value_h_per_t [i])
							{
								if (value_h_per_t [i] != 0)
									hMin = value_h_per_t [i];
							}
						}

						counter ++;
						for(int i = 0; i < position; ++i)
						{
							sum_temper = sum_temper + value_t_per_t [i];
							sum_humid = sum_humid + value_h_per_t [i];
						}
						tMid = sum_temper / position;
						hMid = sum_humid / position;
						sum_temper = 0;
						sum_humid = 0;
						deviation.max_deviation_t_pos = tMax - tMid;
						deviation.max_deviation_h_pos = hMax - hMid;
						deviation.max_deviation_t_neg = tMin - tMid;
						deviation.max_deviation_h_neg = hMin - hMid;

						tMax = 0;
						hMax = 0;
						tMin = 100;
						hMin = 100;
					}
					else
					{
						counter = 0;
					}
					time_flag = 0;
					//Скидання секундного прапорця
					seconds_counter = 0;
				}

			}
			//time_flag = 0;
		}
		osMessageQueuePut(myCalcDisplayQueueHandle, &deviation, 0, osWaitForever);
		osDelay(1);
	}
  /* USER CODE END StartCalculationTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the myDisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
	Queue_Temp_Hum_t Measure;
	Queue_Dev_t deviation;
  /* Infinite loop */
  for(;;)
  {
	  osMessageQueueGet(myTempDisplayQueueHandle, &Measure, 0, osWaitForever);
	  osMessageQueueGet(myCalcDisplayQueueHandle, &deviation, 0, osWaitForever);
	  switch(btn_state)
	  {
	  case 0:
		  max7219_Send_float(Measure.Temper);
		  break;
	  case 1:
		  max7219_Send_float(Measure.Humid);
		  break;
	  case 2:
		  max7219_Send_float(deviation.max_deviation_t_pos);
	  		  break;
	  case 3:
		  max7219_Send_float(deviation.max_deviation_t_neg);
	  		  break;
	  case 4:
		  max7219_Send_float(deviation.max_deviation_h_pos);
	  		  break;
	  case 5:
		  max7219_Send_float(deviation.max_deviation_h_neg);
	  		  break;
	  }
	  osDelay(1);
  }
  /* USER CODE END StartDisplayTask */
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


  if(htim->Instance == TIM1)
  {
	  HAL_TIM_Base_Stop_IT(&htim1); // останавливаем таймер
	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);  // очищаем бит EXTI_PR (бит прерывания)
	  NVIC_ClearPendingIRQ(EXTI15_10_IRQn); // очищаем бит NVIC_ICPRx (бит очереди)
	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);   // включаем внешнее прерывание
	  //btn_cur = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1)
	  {
		  if(btn_state < 5)
		  {
			  btn_state ++;
		  }
		  else btn_state = 0;
	  }
  }

  if(htim->Instance == TIM2)
  {
//	  if(time_counter < 60)
//	  {
//		  time_counter ++;
//	  }
//	  else
//	  {
//		  time_flag = 1;
//		  time_counter = 0;
//	  }
	  time_flag = 1;
  }

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
