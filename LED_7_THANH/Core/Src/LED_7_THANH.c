#include "LED_7_THANH.h"

uint16_t dem = 1234;
uint8_t so1 = 0, so2 = 0, so3 = 0, so4 = 0;
uint8_t mang_led[10] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90};
uint8_t count_quet_led = 0;

#define DATA_LOW HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, GPIO_PIN_RESET);
#define DATA_HIGH HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, GPIO_PIN_SET);

#define CLK_LOW HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
#define CLK_HIGH HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);

#define LAT_HIGH HAL_GPIO_WritePin(LAT_GPIO_Port, LAT_Pin, GPIO_PIN_SET);
#define LAT_LOW HAL_GPIO_WritePin(LAT_GPIO_Port, LAT_Pin, GPIO_PIN_RESET);

void truyen_8_bit(uint8_t data)
{
	uint8_t temp=0,i;
	for(i = 0; i < 8; i++)
	{
		CLK_LOW;
		temp = data & 0x80;
		if(temp == 0x80)
		{
			DATA_HIGH;
		}
		else
		{
			DATA_LOW;
		}
		CLK_HIGH;
		data = data * 2;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim1.Instance)
	{
		if(count_quet_led > 3)
		{
			count_quet_led = 0;
		}
		else
		{
			count_quet_led++;
		}
		if(count_quet_led == 0)
		{
			LAT_LOW;
			truyen_8_bit(so1);
			truyen_8_bit(0x1);
			LAT_HIGH;
		}
		else if(count_quet_led == 1)
		{
			LAT_LOW;
			truyen_8_bit(so2);
			truyen_8_bit(0x2);
			LAT_HIGH;
				}
		else if(count_quet_led == 2)
		{
			 LAT_LOW;
			 truyen_8_bit(so3);
			 truyen_8_bit(0x4);
			LAT_HIGH;
		}
		else if(count_quet_led == 3)
		{
			LAT_LOW;
			truyen_8_bit(so4);
			truyen_8_bit(0x8);
			LAT_HIGH;
		}
	}
}

void led_init(void)
{
	HAL_TIM_Base_Start_IT(&htim1);
	so1 = mang_led[dem/1000];
	so2 = mang_led[(dem%1000)/100];
	so3 = mang_led[(dem%100)/10];
	so4 = mang_led[(dem%10)/1];
}
