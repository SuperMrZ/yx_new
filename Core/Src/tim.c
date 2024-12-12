/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
#define pi acos(-1)
#define MAX_INDEX 64

float t[MAX_INDEX] = {1000, 667, 500, 400, 333, 286, 250, 222, 200, 182, 167, 154, 143, 133, 125, 118, 111, 105, 100, 95, 91, 87, 83, 80, 77, 74, 71, 69, 67, 65, 63, 61, 59, 57, 56, 54, 53, 51, 50, 49, 48, 47, 45, 42, 38, 36, 33, 31, 29, 28, 26, 25, 20, 17, 14, 13, 11, 10, 9, 8, 5, 4, 3, 2};
float f[MAX_INDEX] = {1.000000, 1.499250, 2.000000, 2.500000, 3.003003, 3.496503, 4.000000, 4.504505, 5.000000, 5.494505, 5.988024, 6.493506, 6.993007, 7.518797, 8.000000, 8.474576, 9.009009, 9.523810, 10.000000, 10.526316, 10.989011, 11.494253, 12.048193, 12.500000, 12.987013, 13.513514, 14.084507, 14.492754, 14.925373, 15.384615, 15.873016, 16.393443, 16.949153, 17.543860, 17.857143, 18.518519, 18.867925, 19.607843, 20.000000, 20.408163, 20.833333, 21.276596, 22.222222, 23.809524, 26.315789, 27.777778, 30.303030, 32.258065, 34.482759, 35.714286, 38.461538, 40.000000, 50.000000, 58.823529, 71.428571, 76.923077, 90.909091, 100.000000, 111.111111, 125.000000, 200.000000, 250.000000, 333.333333, 500.000000};





float tt[MAX_INDEX];
int sin_signal_switch;
int i = 0;
// float time = 0;
float sin_signal;
/* USER CODE END 0 */

TIM_HandleTypeDef htim3;

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16799;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /* TIM3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
	// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  // {

  //   while(time * 1000 > t[i] && i < (MAX_INDEX - 1))
	//     {
	// 	    i++;
  //       time=0;
	//     }
	//   if(i == MAX_INDEX)
	//   {
	// 	  sin_signal = 0;
	// 	  time = 0;
	// 	  sin_signal_switch = 0;
	//   }
	//   sin_signal = sin(2 * pi * f[i] * time);//正弦信号
	
	//   time += 0.002;

  // }



/* USER CODE END 1 */
