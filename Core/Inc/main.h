/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_MISO_Pin GPIO_PIN_4
#define SPI1_MISO_GPIO_Port GPIOB
#define SPI1_CLK_Pin GPIO_PIN_3
#define SPI1_CLK_GPIO_Port GPIOB
#define HALL_SENSOR_PIN2_Pin GPIO_PIN_7
#define HALL_SENSOR_PIN2_GPIO_Port GPIOI
#define HALL_SENSOR_PIN1_Pin GPIO_PIN_6
#define HALL_SENSOR_PIN1_GPIO_Port GPIOI
#define LASER_Pin GPIO_PIN_8
#define LASER_GPIO_Port GPIOC
#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG
#define DRDY_IST8310_Pin GPIO_PIN_3
#define DRDY_IST8310_GPIO_Port GPIOG
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef struct{//25ֽSBUS洢ṹ
	uint8_t Start;
	uint16_t Ch1;
	uint16_t Ch2;
	uint16_t Ch3;
	uint16_t Ch4;
	uint16_t Ch5;
	uint16_t Ch6;
	uint16_t Ch7;
	uint16_t Ch8;
	uint16_t Ch9;
	uint16_t Ch10;
	uint16_t Ch11;
	uint16_t Ch12;
	uint16_t Ch13;
	uint16_t Ch14;
	uint16_t Ch15;
	uint16_t Ch16;
	uint8_t Flag;
	uint8_t End;
	
	uint16_t SF;
	uint16_t SF_last;
	uint16_t SH;
	uint16_t SH_last;
	uint16_t SE;
	uint16_t SE_last;
	uint16_t SG;
	uint16_t SG_last;

}SBUS_Buffer;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
