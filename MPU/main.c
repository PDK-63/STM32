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
#include "math.h"
#include "stdlib.h"
#include "stdint.h"
#include "sd_hal_mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define dt	0.004
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
SD_MPU6050 mpu1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* PID Parameter*/
const float Kp = 30;
const float Ki = 200;
const float Kd = 0;
const float SetPoint = 0;
//----------------------------------
/* MPU6050 parameter*/
float g_x;
float g_y;
float g_z;
float a_x;
float a_y;
float a_z;
float gForcex = 0;
float gForcey = 0;
float gForcez = 0;
float ax_off = 0.0306123048;
float ay_off = -0.00247998047;
float az_off = 1.00272071;
float gx_off = 376.86;
float gy_off = 93.00;
float gz_off = -92.64;
float rotx = 0;
float roty = 0;
float rotz = 0;
float pitch;
float roll;
float angle_roll = 0;
float angle_pitch = 0;
//----------------------------------
/* Motor parameter*/
int8_t DirR, DirL;
volatile int16_t CounterR, CounterL;
int16_t TimerHIGHR, TimerLOWR, TimerHIGHL, TimerLOWL;
volatile int32_t step_R, step_L;
float error, errorLast, ValuePID_R, ValuePID_L, ValuePID, I = 0, ModeLeft, ModeRight;
//----------------------------------
/* Motor control */
void Speed_R(int16_t speed)
{
	if(speed <0)
	{
		DirR = -1;
		HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, 0);
	}		
	else if(speed >0)	
	{
		DirR = 1;
		HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, 1);
	}	
	else DirR = 0;
	TimerHIGHR  = abs(speed)/2;
	TimerLOWR   = abs(speed);
}

void Speed_L(int16_t speed)		
{
	if(speed <0)
	{
		DirL = -1;
		HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, 0);
	}		
	else if(speed >0)	
	{
		DirL = 1;
		HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, 1);
	}	
	else DirL = 0;
	TimerHIGHL  = abs(speed)/2;
	TimerLOWL   = abs(speed);
}
//----------------------------------
/* Timer interrupt */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		if(DirR !=0)	
		{
			CounterR++;
			if(CounterR <= TimerHIGHR)  HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, 1);
			else 											  HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, 0);
			if(CounterR > TimerLOWR) 
			{
				CounterR =0;
				if(DirR > 0)			step_R++;
				else if(DirR < 0)	step_R--;
			}
		}
		if(DirL !=0)	
		{
			CounterL++;
			if(CounterL <= TimerHIGHL)	HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, 1);
			else 											  HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, 0);
			if(CounterL > TimerLOWL) 
			{
				CounterL =0;
				if(DirL > 0)			step_L++;
				else if(DirL < 0)	step_L--;
			}
		}
	}
}

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
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		SD_MPU6050_Result result ;
		result = SD_MPU6050_Init(&hi2c1,&mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_250s );
		SD_MPU6050_ReadAll(&hi2c1,&mpu1);
	  int16_t g_x = mpu1.Gyroscope_X;
	  int16_t g_y = mpu1.Gyroscope_Y;
	  int16_t g_z = mpu1.Gyroscope_Z;
		
	  int16_t a_x = mpu1.Accelerometer_X;
	  int16_t a_y = mpu1.Accelerometer_Y;
	  int16_t a_z = mpu1.Accelerometer_Z;
 
		rotx = (float)(g_x - gx_off)/131;
		roty = (float)(g_y - gy_off)/131;
		rotz = (float)(g_z - gz_off)/131;

		gForcex = (float)(a_x - ax_off)/16384 ;
		gForcey = (float)(a_y - ay_off)/16384;
		gForcez = (float)(a_z - az_off)/16384 ;
		 
	  roll  = atan2((double)gForcey,(double)gForcez)*57.29577951;
	  pitch = -atan2((double)gForcex,(double)sqrt((double)gForcey*(double)gForcey+(double)gForcez*(double)gForcez))*57.29577951;
	  angle_roll  = 0.988*(angle_roll  + rotx*dt) +0.012*roll;
	  angle_pitch = 0.988*(angle_pitch + roty*dt) +0.012*pitch;

		error = angle_pitch - SetPoint;
		
		if(((ValuePID > 10) && (ValuePID < 20)) || ((ValuePID < -10) && (ValuePID > -20)))		error += ValuePID * 0.014;
		else if(((ValuePID > 20) && (ValuePID < 25)) || ((ValuePID < -20) && (ValuePID > -25)))		error += ValuePID * 0.015;
		else if(ValuePID > 25 || ValuePID < -25)	error += ValuePID * 0.016;
		
		I += (Ki*error) * dt ;
		if(I < -400)		 I = -400;
		else if(I > 400) I =  400;
		
		ValuePID = Kp * error + I + Kd * (error - errorLast) / dt;
		if (ValuePID > 400)				ValuePID = 400;                                      
		else if (ValuePID < -400)	ValuePID = -400;
		
		if(ValuePID < 6 && ValuePID > -6) ValuePID = 0;
		errorLast = error;
		
		if (angle_roll > 30 || angle_roll < -30) 
		{              
			ValuePID = 0;                                                          
			I = 0;                                                                                                                                                                   
		}
		
		ValuePID_L = ValuePID;
		ValuePID_R = ValuePID;
		
		if(ValuePID_L > 0) 				ValuePID_L =  405 - (1 / (ValuePID_L + 9)) * 5500;
		else if(ValuePID_L < 0) 	ValuePID_L = -405 - (1 / (ValuePID_L - 9)) * 5500;
		if(ValuePID_R > 0) 				ValuePID_R =  405 - (1 / (ValuePID_R + 9)) * 5500;
		else if(ValuePID_R < 0) 	ValuePID_R = -405 - (1 / (ValuePID_R - 9)) * 5500;
	
		if(ValuePID_L > 0) 			ModeLeft  =  400 - ValuePID_L;
		else if(ValuePID_L < 0) ModeLeft  = -400 - ValuePID_L;
		else 										ModeLeft  =  0;
		if(ValuePID_R > 0) 			ModeRight =  400 - ValuePID_R;
		else if(ValuePID_R < 0) ModeRight = -400 - ValuePID_R;
		else 										ModeRight =  0;
		
		Speed_R(-ModeRight);
		Speed_L(ModeLeft);
		
		HAL_Delay(4); 
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 14;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STEP1_Pin|DIR1_Pin|DIR2_Pin|STEP2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STEP1_Pin DIR1_Pin DIR2_Pin STEP2_Pin */
  GPIO_InitStruct.Pin = STEP1_Pin|DIR1_Pin|DIR2_Pin|STEP2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
