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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Led_Er_Pin GPIO_PIN_0
#define Led_Er_GPIO_Port GPIOA
#define BT1_Pin GPIO_PIN_2
#define BT1_GPIO_Port GPIOA
#define BT2_Pin GPIO_PIN_3
#define BT2_GPIO_Port GPIOA
#define Lora_SCK_Pin GPIO_PIN_5
#define Lora_SCK_GPIO_Port GPIOA
#define Lora_MISO_Pin GPIO_PIN_6
#define Lora_MISO_GPIO_Port GPIOA
#define Lora_MOSI_Pin GPIO_PIN_7
#define Lora_MOSI_GPIO_Port GPIOA
#define Lora_NSS_Pin GPIO_PIN_0
#define Lora_NSS_GPIO_Port GPIOB
#define Lora_RST_Pin GPIO_PIN_1
#define Lora_RST_GPIO_Port GPIOB
#define Lora_IRQ_Pin GPIO_PIN_2
#define Lora_IRQ_GPIO_Port GPIOB
#define Lora_DIO1_Pin GPIO_PIN_10
#define Lora_DIO1_GPIO_Port GPIOB
#define Bt_En_Pin GPIO_PIN_15
#define Bt_En_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
