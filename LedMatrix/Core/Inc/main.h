/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define MT_R1_Pin GPIO_PIN_0
#define MT_R1_GPIO_Port GPIOA
#define MT_R2_Pin GPIO_PIN_1
#define MT_R2_GPIO_Port GPIOA
#define RS485_TX_Pin GPIO_PIN_4
#define RS485_TX_GPIO_Port GPIOA
#define INPUT1_Pin GPIO_PIN_5
#define INPUT1_GPIO_Port GPIOA
#define INPUT2_Pin GPIO_PIN_6
#define INPUT2_GPIO_Port GPIOA
#define INPUT3_Pin GPIO_PIN_7
#define INPUT3_GPIO_Port GPIOA
#define INPUT4_Pin GPIO_PIN_0
#define INPUT4_GPIO_Port GPIOB
#define INPUT5_Pin GPIO_PIN_1
#define INPUT5_GPIO_Port GPIOB
#define RELAY1_Pin GPIO_PIN_10
#define RELAY1_GPIO_Port GPIOB
#define RELAY2_Pin GPIO_PIN_11
#define RELAY2_GPIO_Port GPIOB
#define MT_R3_Pin GPIO_PIN_12
#define MT_R3_GPIO_Port GPIOB
#define MT_R4_Pin GPIO_PIN_13
#define MT_R4_GPIO_Port GPIOB
#define MT_R5_Pin GPIO_PIN_14
#define MT_R5_GPIO_Port GPIOB
#define MT_R6_Pin GPIO_PIN_15
#define MT_R6_GPIO_Port GPIOB
#define MT_LAT_Pin GPIO_PIN_3
#define MT_LAT_GPIO_Port GPIOB
#define MT_CLK_Pin GPIO_PIN_4
#define MT_CLK_GPIO_Port GPIOB
#define MT_B_Pin GPIO_PIN_5
#define MT_B_GPIO_Port GPIOB
#define MT_OE_Pin GPIO_PIN_6
#define MT_OE_GPIO_Port GPIOB
#define MT_A_Pin GPIO_PIN_7
#define MT_A_GPIO_Port GPIOB
#define MT_R8_Pin GPIO_PIN_8
#define MT_R8_GPIO_Port GPIOB
#define MT_R7_Pin GPIO_PIN_9
#define MT_R7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
