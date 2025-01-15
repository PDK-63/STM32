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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Button_Joystick_Left_Pin GPIO_PIN_0
#define Button_Joystick_Left_GPIO_Port GPIOA
#define Joystick_Y_L_Pin GPIO_PIN_1
#define Joystick_Y_L_GPIO_Port GPIOA
#define Joystick_Y_R_Pin GPIO_PIN_2
#define Joystick_Y_R_GPIO_Port GPIOA
#define BT1_UP_Pin GPIO_PIN_3
#define BT1_UP_GPIO_Port GPIOA
#define BT4_L_Pin GPIO_PIN_4
#define BT4_L_GPIO_Port GPIOA
#define BT3_DOW_Pin GPIO_PIN_5
#define BT3_DOW_GPIO_Port GPIOA
#define BT2_R_Pin GPIO_PIN_6
#define BT2_R_GPIO_Port GPIOA
#define Joystick_X_R_Pin GPIO_PIN_0
#define Joystick_X_R_GPIO_Port GPIOB
#define Joystick_Y_RB1_Pin GPIO_PIN_1
#define Joystick_Y_RB1_GPIO_Port GPIOB
#define LED_Test_Pin GPIO_PIN_2
#define LED_Test_GPIO_Port GPIOB
#define RX_Pin GPIO_PIN_10
#define RX_GPIO_Port GPIOB
#define TX_Pin GPIO_PIN_11
#define TX_GPIO_Port GPIOB
#define BT11_BACK_Pin GPIO_PIN_12
#define BT11_BACK_GPIO_Port GPIOB
#define BT10_OK_Pin GPIO_PIN_13
#define BT10_OK_GPIO_Port GPIOB
#define BT9_SELECT_Pin GPIO_PIN_14
#define BT9_SELECT_GPIO_Port GPIOB
#define BT8_L_Pin GPIO_PIN_15
#define BT8_L_GPIO_Port GPIOB
#define BT7_DOWN_RIGHT_Pin GPIO_PIN_8
#define BT7_DOWN_RIGHT_GPIO_Port GPIOA
#define BT6_R_Pin GPIO_PIN_9
#define BT6_R_GPIO_Port GPIOA
#define BT5_UP_RIGHT_Pin GPIO_PIN_10
#define BT5_UP_RIGHT_GPIO_Port GPIOA
#define Button_Joystick_Right_Pin GPIO_PIN_11
#define Button_Joystick_Right_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
