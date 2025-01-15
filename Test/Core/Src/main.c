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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
int tinh_vi_tri();

void doc_nen();
void doc_line();
void tinh_nguong();
void hien_thi_led();
void PID();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t dulieu_nen[100];		
uint16_t dulieu_line[100];	
uint16_t nguong[100];
uint16_t gt_digital[100];
uint16_t gt_analog[100];

uint8_t tocdo;
int tong_ts;
int tong_gt;
int VT_HIEN_TAI;
int VT_SAU_DO;
int dem = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
	
// Ham hoc nen 
void doc_nen(){
	uint8_t i;
	for(i = 0; i < 14; i++){
		HAL_GPIO_WritePin(Line1_GPIO_Port, Line1_Pin, i&0x01);
		HAL_GPIO_WritePin(Line2_GPIO_Port, Line2_Pin, i&0x02);
		HAL_GPIO_WritePin(Line3_GPIO_Port, Line3_Pin, i&0x04);
		HAL_GPIO_WritePin(Line4_GPIO_Port, Line4_Pin, i&0x08);
		
		// Khoi tao gia tri ADC1
		HAL_ADC_Start(&hadc1);
		dulieu_nen[i] = HAL_ADC_GetValue(&hadc1);
	}
}

// Ham doc line
void doc_line(){
	uint8_t i;
	for(i = 0; i < 14; i++){
		HAL_GPIO_WritePin(Line1_GPIO_Port, Line1_Pin, i&0x01);
		HAL_GPIO_WritePin(Line2_GPIO_Port, Line2_Pin, i&0x02);
		HAL_GPIO_WritePin(Line3_GPIO_Port, Line3_Pin, i&0x04);
		HAL_GPIO_WritePin(Line4_GPIO_Port, Line4_Pin, i&0x08);
		
		// Khoi tao gia tri ADC1
		HAL_ADC_Start(&hadc1);
		dulieu_line[i] = HAL_ADC_GetValue(&hadc1);
	}
}
// Ham tinh nguong
void tinh_nguong()
{
	for(uint8_t i = 0; i < 14; i++) 
	{
		nguong[i] = (dulieu_line[i] + dulieu_nen[i] )/2;
	} 	
}

// Ham tinh vi tri cua line
int tinh_vi_tri() 
{
	
	for(uint8_t i = 0; i < 14; i++) 
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,i&0x01);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,i&0x02);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,i&0x04);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0 ,i&0x08);
		HAL_ADC_Start(&hadc1); 
		HAL_ADC_PollForConversion(&hadc1,1000);
		gt_analog[i] = HAL_ADC_GetValue(&hadc1);  
		if(gt_analog[i] <= nguong[i])
    {
      gt_digital[i]=0;
    }
    else
    {
      gt_digital[i]=1;
    }		
	}	
	tong_ts = ( 0*gt_digital[0] + 
						  100*gt_digital[1] + 
							200*gt_digital[2] + 
							300*gt_digital[3] + 
							400*gt_digital[4] + 
							500*gt_digital[5] + 
							600*gt_digital[6] + 
							700*gt_digital[7] + 
							800*gt_digital[8] + 
							900*gt_digital[9] + 
							1000*gt_digital[10] + 
							1100*gt_digital[11]	+
							1200*gt_digital[12]	+
							1300*gt_digital[13]);
	tong_gt = ( gt_digital[0] + 
							gt_digital[1] + 
							gt_digital[2] + 
							gt_digital[3] + 
							gt_digital[4] + 
							gt_digital[5] + 
							gt_digital[6] + 
							gt_digital[7] + 
							gt_digital[8] + 
							gt_digital[9] + 
							gt_digital[10] + 
							gt_digital[11] + 
							gt_digital[12]	+
							gt_digital[13]);
	VT_HIEN_TAI = (tong_ts / tong_gt);
	if(VT_SAU_DO < 200 && VT_HIEN_TAI == 0)
  {
    VT_HIEN_TAI = 0;
  }
  if(VT_SAU_DO > 1100 && VT_HIEN_TAI == 0)
  {
    VT_HIEN_TAI = 1300;
  }
	if(VT_SAU_DO > 500 && VT_SAU_DO <800 && VT_HIEN_TAI == 0)
	{
		dem++;
	}
  VT_SAU_DO = VT_HIEN_TAI;
	return VT_HIEN_TAI;
}

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Line2_Pin|Line3_Pin|Line4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Line1_GPIO_Port, Line1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Line2_Pin Line3_Pin Line4_Pin */
  GPIO_InitStruct.Pin = Line2_Pin|Line3_Pin|Line4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Line1_Pin */
  GPIO_InitStruct.Pin = Line1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Line1_GPIO_Port, &GPIO_InitStruct);

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
