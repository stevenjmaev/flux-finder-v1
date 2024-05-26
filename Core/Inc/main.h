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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "matrix.h"
#include "frame_converter.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN0_Pin GPIO_PIN_13
#define BTN0_GPIO_Port GPIOC
#define BTN1_Pin GPIO_PIN_14
#define BTN1_GPIO_Port GPIOC
#define BTN2_Pin GPIO_PIN_15
#define BTN2_GPIO_Port GPIOC
#define HALL_Pin GPIO_PIN_0
#define HALL_GPIO_Port GPIOA
#define THERM1_Pin GPIO_PIN_4
#define THERM1_GPIO_Port GPIOA
#define THERM2_Pin GPIO_PIN_5
#define THERM2_GPIO_Port GPIOA
#define REF_1p25_Pin GPIO_PIN_6
#define REF_1p25_GPIO_Port GPIOA
#define I_SENSE_Pin GPIO_PIN_7
#define I_SENSE_GPIO_Port GPIOA
#define ROW0_Pin GPIO_PIN_0
#define ROW0_GPIO_Port GPIOB
#define ROW2_Pin GPIO_PIN_2
#define ROW2_GPIO_Port GPIOB
#define ROW3_Pin GPIO_PIN_10
#define ROW3_GPIO_Port GPIOB
#define COL0_Pin GPIO_PIN_11
#define COL0_GPIO_Port GPIOB
#define SPI1_CS0_Pin GPIO_PIN_12
#define SPI1_CS0_GPIO_Port GPIOB
#define uSD_DETECT_Pin GPIO_PIN_9
#define uSD_DETECT_GPIO_Port GPIOA
#define COL1_Pin GPIO_PIN_10
#define COL1_GPIO_Port GPIOA
#define COL2_Pin GPIO_PIN_11
#define COL2_GPIO_Port GPIOA
#define COL3_Pin GPIO_PIN_12
#define COL3_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_15
#define LED0_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB
#define ROW1_Pin GPIO_PIN_5
#define ROW1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
