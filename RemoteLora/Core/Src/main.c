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
#include "stdio.h"
#include "LoRa.h"
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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MAX_DATA 128
uint8_t read_data[MAX_DATA];
uint8_t send_data[MAX_DATA];
#define METER_TYPE "S1"
#define METER_VERSION 01
uint32_t SerialNumber = 12345678;
uint8_t LuuLuong1 = 11,LuuLuong2 = 22,LuuLuong3 = 33,DungLuongPin = 78,NhietDoMoiTruong = 99,Loi=88;
int32_t adc0_value_count = 0,adc1_value_count = 0,adc2_value_count = 0,water_count=0;

LoRa myLoRa;
uint8_t read_data[MAX_DATA];
uint8_t send_data[MAX_DATA];
int			RSSI;
uint16_t data_length = 0, data_ok = 0;
uint16_t blink_led=0,count_send_ping=0,flag_sendping=0;
uint8_t IO1_state=0,IO2_state=0;
uint16_t adc[3];
void reset_lora_ble(void)
{
	//khoi tao lora
	myLoRa = newLoRa();

	myLoRa.hSPIx                 = &hspi1;
	myLoRa.CS_port               = LORA_NSS_GPIO_Port;
	myLoRa.CS_pin                = LORA_NSS_Pin;
	myLoRa.reset_port            = LORA_RESET_GPIO_Port;
	myLoRa.reset_pin             = LORA_RESET_Pin;
	myLoRa.DIO0_port						 = LORA_IRQ_GPIO_Port;
	myLoRa.DIO0_pin							 = LORA_IRQ_Pin;

	myLoRa.frequency             = 433;							  // default = 433 MHz
	myLoRa.spredingFactor        = SF_7;							// default = SF_7
	myLoRa.bandWidth			       = BW_125KHz;				  // default = BW_125KHz
	myLoRa.crcRate				       = CR_4_5;						// default = CR_4_5
	myLoRa.power					       = POWER_20db;				// default = 20db
	myLoRa.overCurrentProtection = 120; 							// default = 100 mA
	myLoRa.preamble				       = 10;		  					// default = 8;

	LoRa_reset(&myLoRa);
	if(LoRa_init(&myLoRa) == LORA_OK)
	{
		HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
	}
	else
	{
		while(1)
		{
			  HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
			  HAL_Delay(10);
		}
	}
//	LoRa_startReceiving(&myLoRa);
//	HAL_Delay(1000);
}
uint8_t addCRC(uint8_t *data, uint16_t length)
{

	uint16_t len = length;
	uint16_t i=0;
	uint8_t checksum = 0;
	for(i=0;i<len;i++)
	{
		checksum+=(~(data[i])+1);
	}
	return checksum;

}
uint8_t check_crc(uint8_t *data, uint16_t length)
{
	uint16_t crc_temp,crcok = 0;
	crcok += (data[length-3] - '0')*100;
	crcok += (data[length-2] - '0')*10;
	crcok += (data[length-1] - '0');

	crc_temp = addCRC(data,length-3);

	if(crcok == crc_temp)
	{
		return 1;
	}
	else
	{
		return 0;
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
	uint32_t stick_1s = 0,flag_sw1 = 0,flag_sw2 = 0,flag_sw3 = 0,flag_sw4 = 0;
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */


  if(HAL_GPIO_ReadPin(MCU_PWR_READ1_GPIO_Port, MCU_PWR_READ1_Pin) == GPIO_PIN_SET)
  {
	  HAL_GPIO_WritePin(MCU_PWR_EN_GPIO_Port, MCU_PWR_EN_Pin, GPIO_PIN_SET);
	  flag_sw1 = 1;
  }
  else if(HAL_GPIO_ReadPin(MCU_PWR_READ2_GPIO_Port, MCU_PWR_READ2_Pin) == GPIO_PIN_SET)
  {
	  HAL_GPIO_WritePin(MCU_PWR_EN_GPIO_Port, MCU_PWR_EN_Pin, GPIO_PIN_SET);
	  flag_sw2 = 1;
  }
  else if(HAL_GPIO_ReadPin(MCU_PWR_READ3_GPIO_Port, MCU_PWR_READ3_Pin) == GPIO_PIN_SET)
  {
	  HAL_GPIO_WritePin(MCU_PWR_EN_GPIO_Port, MCU_PWR_EN_Pin, GPIO_PIN_SET);
	  flag_sw3 = 1;
  }
  else if(HAL_GPIO_ReadPin(MCU_PWR_READ4_GPIO_Port, MCU_PWR_READ4_Pin) == GPIO_PIN_SET)
  {
	  HAL_GPIO_WritePin(MCU_PWR_EN_GPIO_Port, MCU_PWR_EN_Pin, GPIO_PIN_SET);
	  flag_sw4 = 1;
  }
  else
  {
	  HAL_GPIO_WritePin(MCU_PWR_EN_GPIO_Port, MCU_PWR_EN_Pin, GPIO_PIN_RESET);
  }
  reset_lora_ble();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  if(HAL_GPIO_ReadPin(MCU_PWR_READ1_GPIO_Port, MCU_PWR_READ1_Pin) == GPIO_PIN_SET)
//	  {
//		  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
//	  }
//	  else
//	  {
//		  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
//	  }
	  if(HAL_GPIO_ReadPin(myLoRa.DIO0_port, myLoRa.DIO0_pin) == GPIO_PIN_SET)
	  {
		  HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);//nhay led status
		  data_length = LoRa_receive(&myLoRa, read_data, MAX_DATA);
		  RSSI = LoRa_getRSSI(&myLoRa);

//		  send_data[0] = 'V';
//		  send_data[1]= 'A';
//		  send_data[2]= 'N';
//		  send_data[3]= ' ';
//		  LoRa_transmit(&myLoRa, send_data, 4, 500);
//		  stick_1s = HAL_GetTick();
	  }
//	  if(HAL_GetTick() - stick_1s >=100)
//	  	  {
//	  		  send_data[0] = 'V';
//	  		  send_data[1]= 'I';
//	  		  send_data[2]= 'E';
//	  		  send_data[3]= 'T';
//	  		  LoRa_transmit(&myLoRa, send_data, 4, 500);
//	  		  stick_1s = HAL_GetTick();
//	  	  }
	  if(flag_sw1 == 1)
	  {
		  send_data[0] = 'V';
		  send_data[1]= 'I';
		  send_data[2]= 'E';
		  send_data[3]= '1';
		  LoRa_transmit(&myLoRa, send_data, 4, 500);
		  HAL_Delay(100);

		  HAL_GPIO_WritePin(MCU_PWR_EN_GPIO_Port, MCU_PWR_EN_Pin, GPIO_PIN_RESET);
	  }
	  else if(flag_sw2 == 1)
	  {
		  send_data[0] = 'V';
		  send_data[1]= 'I';
		  send_data[2]= 'E';
		  send_data[3]= '2';
		  LoRa_transmit(&myLoRa, send_data, 4, 500);
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(MCU_PWR_EN_GPIO_Port, MCU_PWR_EN_Pin, GPIO_PIN_RESET);
	  }
	  else if(flag_sw3 == 1)
	  {
		  send_data[0] = 'V';
		  send_data[1]= 'I';
		  send_data[2]= 'E';
		  send_data[3]= '3';
		  LoRa_transmit(&myLoRa, send_data, 4, 500);
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(MCU_PWR_EN_GPIO_Port, MCU_PWR_EN_Pin, GPIO_PIN_RESET);
	  }
	  else if(flag_sw4 == 1)
	  {
		  send_data[0] = 'V';
		  send_data[1]= 'I';
		  send_data[2]= 'E';
		  send_data[3]= '4';
		  LoRa_transmit(&myLoRa, send_data, 4, 500);
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(MCU_PWR_EN_GPIO_Port, MCU_PWR_EN_Pin, GPIO_PIN_RESET);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MCU_PWR_EN_Pin|LORA_RESET_Pin|LORA_NSS_Pin|LED_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MCU_PWR_READ1_Pin MCU_PWR_READ2_Pin */
  GPIO_InitStruct.Pin = MCU_PWR_READ1_Pin|MCU_PWR_READ2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_PWR_READ3_Pin MCU_PWR_READ4_Pin */
  GPIO_InitStruct.Pin = MCU_PWR_READ3_Pin|MCU_PWR_READ4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_PWR_EN_Pin LED_STATUS_Pin */
  GPIO_InitStruct.Pin = MCU_PWR_EN_Pin|LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_RESET_Pin LORA_NSS_Pin */
  GPIO_InitStruct.Pin = LORA_RESET_Pin|LORA_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_IRQ_Pin */
  GPIO_InitStruct.Pin = LORA_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LORA_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_DI1_Pin */
  GPIO_InitStruct.Pin = LORA_DI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LORA_DI1_GPIO_Port, &GPIO_InitStruct);

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
