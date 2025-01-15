/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "led.h"
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
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//bien phuc vu cai dat
uint8_t flag_hetBH = 0;
uint32_t TS2 = 0,TS1 = 0,thoi_gian_hoatdong = 0,My_Baudrate = 0,My_ID = 0;
int trangthainutnhan = 0;
void luu_thong_tin(void);

#define MENU HAL_GPIO_ReadPin(INPUT3_GPIO_Port, INPUT3_Pin)
#define UP HAL_GPIO_ReadPin(INPUT4_GPIO_Port, INPUT4_Pin)
#define DOWN HAL_GPIO_ReadPin(INPUT5_GPIO_Port, INPUT5_Pin)
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
 */
void Tru30s(void);
void COUNTDOWN(void);
void DK_Pin(void);
void Pause();

void HienThiPin(uint8_t level)
{
//	int x = 36;
//	int y = 18;
//	for(int a = x; a < x + 22; x++)
//	{
//		for(int b = y; b < y + 6; y++)
//		{
//			setpixel(a, b, 1);
//		}
//	}
	//hien thi khung pin ngang(tren)
	for(int i = 36;i < 37+22;i++)
	{
		setpixel(i, 18, 1);
	}
	//hien thi khung pin ngang(duoi)
	for(int i = 36;i < 37+22;i++)
	{
		setpixel(i, 21 + 2, 1);
	}
	// hien thi khung Pin doc(trai)
	for(int i=18;i<18+6;i++)
	{
		setpixel(36, i, 1);
	}
	// hien thi khung Pin doc(phai)
	for(int i=18;i<18+6;i++)
	{
		setpixel(36+22, i, 1);
	}
	// hien thi dau cuc Pin
	for(int i=19;i<18+5;i++)
	{
			setpixel(36+23, i, 1);
	}

	// hien thi Level
	switch (level)
	{
		case 0:
			level = 1;
			break;
		case 1:
			level = 2;
			break;
		case 2:
			level = 3;
			break;
		case 3:
			level = 4;
			break;
		case 4:
			level = 5;
			break;
	}

	int vtri_batdau = 39 - 1;
	for(int t = 0; t < level; t++)
	{
		for(int i = vtri_batdau; i < vtri_batdau + 3; i++)
		{
			for(int j = 18; j < 24; j++)
			{
				setpixel(i, j, 1);
			}
		}
		vtri_batdau += 4;
	}
//	switch(level)
//	{
//		// Level 1
//		case 0:
//		{
//			for(int i = 39-1; i < 42-1; i++)
//			{
//				for(int j = 18; j < 24; j++)
//				{
//					setpixel(i, j, 1);
//				}
//			}
//			break;
//		}
//		// Level 2
//		case 1:
//		{
//			for(int i = 39 - 1; i < 42 - 1; i++)
//			{
//				for(int j = 18; j < 24; j++)
//				{
//					setpixel(i, j, 1);
//				}
//			}
//			for(int i = 43 - 1; i < 46 - 1; i++)
//			{
//				for(int j = 18; j < 24; j++)
//				{
//					setpixel(i, j, 1);
//				}
//			}
//			break;
//		}
//		// Level 3
//		case 2:
//		{
//			for(int i = 39 - 1; i < 42 - 1; i++){
//				for(int j = 18; j < 24; j++){
//					setpixel(i, j, 1);
//				}
//			}
//			for(int i = 43 - 1; i < 46 - 1; i++){
//				for(int j = 18; j < 24; j++){
//					setpixel(i, j, 1);
//				}
//			}
//
//			for(int i = 47 - 1; i < 50 - 1; i++){
//				for(int j = 18; j < 24; j++){
//					setpixel(i, j, 1);
//				}
//			}
//			break;
//		}
//		// Level 4
//		case 3:
//		{
//			for(int i = 39 - 1; i < 42 - 1; i++){
//				for(int j = 18; j < 24; j++){
//					setpixel(i, j, 1);
//				}
//			}
//			for(int i = 43 - 1; i < 46 - 1; i++){
//				for(int j = 18; j < 24; j++){
//					setpixel(i, j, 1);
//				}
//			}
//
//			for(int i = 47 - 1; i < 50 - 1; i++){
//				for(int j = 18; j < 24; j++){
//					setpixel(i, j, 1);
//				}
//			}
//
//			for(int i = 51 - 1; i < 54 - 1; i++){
//				for(int j = 18; j < 24; j++){
//					setpixel(i, j, 1);
//				}
//			}
//			break;
//		}
//
//		// Level 5
//		case 4:
//		{
//			for(int i = 39 - 1; i < 42 - 1; i++){
//				for(int j = 18; j < 24; j++){
//					setpixel(i, j, 1);
//				}
//			}
//
//			for(int i = 43 - 1; i < 46 - 1; i++){
//				for(int j = 18; j < 24; j++){
//					setpixel(i, j, 1);
//				}
//			}
//
//			for(int i = 47 - 1; i < 50 - 1; i++){
//				for(int j = 18; j < 24; j++){
//					setpixel(i, j, 1);
//				}
//			}
//			for(int i = 51 - 1; i < 54 - 1; i++){
//				for(int j = 18; j < 24; j++){
//					setpixel(i, j, 1);
//				}
//			}
//			for(int i = 55 - 1; i < 58 - 1; i++){
//				for(int j = 18; j < 24; j++){
//					setpixel(i, j, 1);
//				}
//			}
//			break;
//		}
//	}
}

uint16_t trangthaiPin = 0;
uint16_t demPin = 0;
void DK_Pin(){
	if(HAL_GPIO_ReadPin(INPUT5_GPIO_Port,INPUT5_Pin) == 0){
		if(trangthaiPin == 1){
			demPin++;
		}
		if(demPin == 1)
		{
			HienThiPin(0);
		}
		else if(demPin == 2)
		{
			HienThiPin(1);
		}
		else if(demPin == 3)
		{
			HienThiPin(2);
		}
		else if(demPin == 4)
		{
			HienThiPin(3);
		}
		else if(demPin == 5)
		{
			HienThiPin(4);
		}
		else if(demPin >= 6){
			demPin = 1;
		}
		trangthaiPin = 0;
	}
	else{
		trangthaiPin = 1;
	}
}

int cnt = 0;
int dem = 420;
uint8_t m[32];
uint8_t m1[32];
uint32_t dem1 = 0,stick = 0;

uint32_t count_1s =0;

uint8_t a,b,c,d,e;

int tt2 = 0;
int cnt2 = 0;

int tt_dung = 0;
int cnt_dung = 0;
// khai bao ham dung
int dung = 0;

int main(void)
{
  /* USER CODE BEGIN 1 */
//	uint8_t m[32];
//	uint8_t m1[32];
//	uint32_t dem1 = 0,stick = 0;
//
//	uint32_t count_1s =0;
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //doc thong tin ra
  led_init();
//  write_string(0, "12.34 %", 0, 0);
//  write_string(0, "100.0 %", 0, 16);
//  write_string(0, "56.78 %", 0, 32);
//  write_string(0, "15.12 %", 0, 48);
//  write_string(0, "92.68 %", 0, 64);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int dem = 420; // 7 mins
  int tt=0;
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
//	  a = HAL_GPIO_ReadPin(INPUT1_GPIO_Port,INPUT1_Pin);
//	  // Start
//	  b = HAL_GPIO_ReadPin(INPUT2_GPIO_Port,INPUT2_Pin);
//	  c = HAL_GPIO_ReadPin(INPUT3_GPIO_Port,INPUT3_Pin);
//	  // ERROR
//	  d = HAL_GPIO_ReadPin(INPUT4_GPIO_Port,INPUT4_Pin);
//	  // Level
//	  e = HAL_GPIO_ReadPin(INPUT5_GPIO_Port,INPUT5_Pin);
//	  if(a | b | c | d | e != 0){
//		  printf("Da nhan");
//	  }

	  DK_Pin();
	  a = HAL_GPIO_ReadPin(INPUT1_GPIO_Port,INPUT1_Pin);
	  b = HAL_GPIO_ReadPin(INPUT2_GPIO_Port,INPUT2_Pin);			// countdown
	  c = HAL_GPIO_ReadPin(INPUT4_GPIO_Port,INPUT4_Pin);			// tru 30s
	  d = HAL_GPIO_ReadPin(INPUT5_GPIO_Port,INPUT5_Pin);			// Level
	  if(b == 0){
		  tt = 1;
		  if(HAL_GetTick() - count_1s > 1000)
		  {
			  if(dem >= 0 && !dung){
				  sprintf(m,"                   %d:%.2d",dem/60,dem%60);
				  write_string(0, m, 0, 3);
				  dem--;

				  // Ham Pause dong ho
				  if(a == 0){
					  dung = 1;
				  }
				  else{
					  dung = 0;
					  count_1s = HAL_GetTick();
				  }
//
			  }
			  else{
				  sprintf(m,"                  0:00");
				  write_string(0, m, 0, 3);
				  HAL_GPIO_WritePin(GPIOA, MT_R1_Pin, GPIO_PIN_SET);
			  }
		  }
		  if(c == 0){
		  	HAL_Delay(10);
		  	if(c == 0){
				sprintf(m,"               %d:%.2d",dem/60,dem%60);
				write_string(0, m, 0, 3);
				dem -= 30;
		  	}
		  }
	  }

	  else if(b	 == 1){
		  clear_lcd();
		  dem = 420;
		  tt = 1;
	  }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3999;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MT_R1_Pin|MT_R2_Pin|RS485_TX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RELAY1_Pin|RELAY2_Pin|MT_R3_Pin|MT_R4_Pin
                          |MT_R5_Pin|MT_R6_Pin|MT_LAT_Pin|MT_CLK_Pin
                          |MT_B_Pin|MT_OE_Pin|MT_A_Pin|MT_R8_Pin
                          |MT_R7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MT_R1_Pin MT_R2_Pin */
  GPIO_InitStruct.Pin = MT_R1_Pin|MT_R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_TX_Pin */
  GPIO_InitStruct.Pin = RS485_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT1_Pin INPUT2_Pin INPUT3_Pin */
  GPIO_InitStruct.Pin = INPUT1_Pin|INPUT2_Pin|INPUT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT4_Pin INPUT5_Pin */
  GPIO_InitStruct.Pin = INPUT4_Pin|INPUT5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY1_Pin RELAY2_Pin */
  GPIO_InitStruct.Pin = RELAY1_Pin|RELAY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MT_R3_Pin MT_R4_Pin MT_R5_Pin MT_R6_Pin
                           MT_LAT_Pin MT_CLK_Pin MT_B_Pin MT_OE_Pin
                           MT_A_Pin MT_R8_Pin MT_R7_Pin */
  GPIO_InitStruct.Pin = MT_R3_Pin|MT_R4_Pin|MT_R5_Pin|MT_R6_Pin
                          |MT_LAT_Pin|MT_CLK_Pin|MT_B_Pin|MT_OE_Pin
                          |MT_A_Pin|MT_R8_Pin|MT_R7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
