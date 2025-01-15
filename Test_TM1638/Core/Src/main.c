/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
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

/* USER CODE BEGIN PV */
const uint8_t FontNumber[] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F, // 9
    0x77, // A
    0x7C, // b
    0x39, // C
    0x5E, // d
    0x79, // E
    0x71  // F
};

const char FontChar[] =
{
	0x00, // (32) <space>
	0xC6, // (33) !
	0x22, // (34) "
	0x7E, // (35) #
	0x6D, // (36) $
	0x00, // (37) %
	0x00, // (38) &
	0x02, // (39) '
	0x30, // (40) (
	0x06, // (41) )
	0x63, // (42) *
	0x00, // (43) +
	0x04, // (44) ,
	0x40, // (45) -
	0x80, // (46) .
	0x52, // (47) /
	0x3F, // (48) 0
	0x06, // (49) 1
	0x6B, // (50) 2
	0x4F, // (51) 3
	0x66, // (52) 4
	0x6D, // (53) 5
	0x7D, // (54) 6
	0x27, // (55) 7
	0x7F, // (56) 8
	0x6F, // (57) 9
	0x00, // (58) :
	0x00, // (59) ;
	0x00, // (60) <
	0x48, // (61) =
	0x00, // (62) >
	0x53, // (63) ?
	0x5F, // (64) @
	0x77, // (65) A
	0x7F, // (66) B
	0x39, // (67) C
	0x3F, // (68) D
	0x79, // (69) E
	0x71, // (70) F
	0x3D, // (71) G
	0x76, // (72) H
	0x06, // (73) I
	0x1F, // (74) J
	0x69, // (75) K
	0x38, // (76) L
	0x55, // (77) M
	0x37, // (78) N
	0x3F, // (79) O
	0x73, // (80) P
	0x67, // (81) Q
	0x31, // (82) R
	0x6D, // (83) S
	0x78, // (84) T
	0x3E, // (85) U
	0x2A, // (86) V
	0x3D, // (87) W
	0x76, // (88) X
	0x6E, // (89) Y
	0x6B, // (90) Z
	0x39, // (91) [
	0x64, // (92) 
	0x0F, // (93) ]
	0x00, // (94) ^
	0x08, // (95) _
	0x20, // (96) `
	0x5F, // (97) a
	0x7C, // (98) b
	0x58, // (99) c
	0x5E, // (100) d
	0x7B, // (101) e
	0x31, // (102) f
	0x6F, // (103) g
	0x74, // (104) h
	0x04, // (105) i
	0x0E, // (106) j
	0x75, // (107) k
	0x30, // (108) l
	0x55, // (109) m
	0x54, // (110) n
	0x5C, // (111) o
	0x73, // (112) p
	0x67, // (113) q
	0x50, // (114) r
	0x6D, // (115) s
	0x78, // (116) t
	0x1C, // (117) u
	0x2A, // (118) v
	0x1D, // (119) w
	0x76, // (120) x
	0x6E, // (121) y
	0x47, // (122) z
	0x46, // (123) {
	0x06, // (124) |
	0x70, // (125) }
	0x01, // (126) ~
};

uint8_t button_State;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

void TM1638_Init(uint8_t Data) {
    for (int i = 0; i < 8; i++) {
        // Set DI pin
			if (Data & 0x01) {
					HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, GPIO_PIN_SET);
			} else {
					HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, GPIO_PIN_RESET);
			}
			
			// Toggle CLK pin
			HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
			Data >>= 1;
			HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);
    }
}

void WriteCmd(uint8_t cmd) {
    HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_RESET);
    TM1638_Init(cmd);
    HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_SET);
}

// Xoa man hinh va tat tat ca cac Led
void LedReset(void) {
    WriteCmd(0x40);
    HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_RESET);
    TM1638_Init(0xC0);
    
    for (int i = 0; i < 32; i++) {
        TM1638_Init(0x00);
    }
    
    HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_SET);
}

void WriteDigit(uint8_t number, uint8_t pos) {
    WriteCmd(0x44);
    HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_RESET);
    TM1638_Init(0xC0 | pos);
    TM1638_Init(FontNumber[number]);
    HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_SET);
}

// Ham hien thi so 
void DisplayLongNumber(int32_t number) {
    uint8_t digits[8] = {0};
    
    for (int i = 0; i < 8; i++) {
        digits[i] = number % 10;
        number /= 10;
    }
    
    WriteCmd(0x40);
    HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_RESET);
    TM1638_Init(0xC0);
    
    for (int i = 7; i >= 0; i--) {
        TM1638_Init(FontNumber[digits[i]]);
        TM1638_Init(0x00); 
    }
    
    HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_SET);
}

// Ham hien thi gia tri  String
void DisplayString(const char *msg) {
    WriteCmd(0x40);
    HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_RESET);
    TM1638_Init(0xC0);
    
    for (int i = 0; i < 8; i++) {
        if (msg[i] == '\0') break; // End of string
        TM1638_Init(FontChar[msg[i] - 32]);
        TM1638_Init(0x00); // Space between characters
    }
    
    HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_SET);
}

/* Ham dieu khien Led*/
void SetLed(uint8_t value, uint8_t pos)
{
	HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, GPIO_PIN_RESET);
	WriteCmd(0x44);
	HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_RESET);
	uint8_t dataSend = 0xC1 + (pos << 1);
	TM1638_Init(dataSend);
	TM1638_Init(value);
	
	HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_SET);
}

uint8_t TM1638_ReadByte(void)
{
	uint8_t data = 0;
	for(int i = 0; i < 8; i++)
	{
			data >>= 1;
			HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
			if(HAL_GPIO_ReadPin(DATA_GPIO_Port, DATA_Pin) == GPIO_PIN_SET)
			{
				data |= 0x80;
			}
			HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);
	}
	return data;
}

uint8_t Read_button()
{
	uint8_t button = 0;
	HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_RESET);
	WriteCmd(0x42);
	// Set chan Data la Input
	for(int i = 0; i < 4; i++)
	{
		button |= (TM1638_ReadByte() << (i *8));
	}
	// Set chan Data la chan Output
	HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_SET);
	return button;
}
/* Ham doc nut nhan*/
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
  /* USER CODE BEGIN 2 */
    WriteCmd(0x8F);
	//Led_Reset();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		DisplayLongNumber(632002);
		HAL_Delay(500);
		DisplayString("--CH1-E-");
		HAL_Delay(500);
		
		/*	Chuong tirnh Led chop tat tu 1-8*/
		for(int i = 0; i < 8; i++)
		{
			for(int j = 0; j < 8; j++)
			{
				if(i == j)
				{
					SetLed(1, j);
					HAL_Delay(10);
				}
				else{
					SetLed(0, j);
					HAL_Delay(10);
				}
			}
		}

	//button_State = Read_button();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Led_Er_Pin|STB_Pin|CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Led_Er_Pin STB_Pin CLK_Pin */
  GPIO_InitStruct.Pin = Led_Er_Pin|STB_Pin|CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DATA_Pin */
  GPIO_InitStruct.Pin = DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DATA_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
