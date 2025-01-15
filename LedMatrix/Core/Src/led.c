/*
 * led.c
 *
 *  Created on: Oct 17, 2023
 *      Author: os
 */

#include "led.h"
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim1.Instance)
	{
		quetledMT();
	}
}

/*
 * Hiện tại bẳng led đấu từ data 8 -> 1
 */
void MT_Send1(uint8_t data1,uint8_t data2,uint8_t data3,uint8_t data4,uint8_t data5,uint8_t data6,uint8_t data7,uint8_t data8)
{
	uint8_t i,tg1,tg2,tg3,tg4,tg5,tg6,tg7,tg8;

	for(i=0;i<8;i++)
	{
		tg1 = data1;
		tg1 = tg1&0x80;
		tg2 = data2;
		tg2 = tg2&0x80;
		tg3 = data3;
		tg3 = tg3&0x80;
		tg4 = data4;
		tg4 = tg4&0x80;
		tg5 = data5;
		tg5 = tg5&0x80;
		tg6 = data6;
		tg6 = tg6&0x80;
		tg7 = data7;
		tg7 = tg7&0x80;
		tg8 = data8;
		tg8 = tg8&0x80;

		if(tg1 == 0x80)
		{
			HAL_GPIO_WritePin(MT_R8_GPIO_Port, MT_R8_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(MT_R8_GPIO_Port, MT_R8_Pin, GPIO_PIN_SET);
		}
		if(tg2 == 0x80)
		{
			HAL_GPIO_WritePin(MT_R7_GPIO_Port, MT_R7_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(MT_R7_GPIO_Port, MT_R7_Pin, GPIO_PIN_SET);
		}
		if(tg3 == 0x80)
		{
			HAL_GPIO_WritePin(MT_R6_GPIO_Port, MT_R6_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(MT_R6_GPIO_Port, MT_R6_Pin, GPIO_PIN_SET);
		}
		if(tg4 == 0x80)
		{
			HAL_GPIO_WritePin(MT_R5_GPIO_Port, MT_R5_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(MT_R5_GPIO_Port, MT_R5_Pin, GPIO_PIN_SET);
		}
		if(tg5 == 0x80)
		{
			HAL_GPIO_WritePin(MT_R4_GPIO_Port, MT_R4_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(MT_R4_GPIO_Port, MT_R4_Pin, GPIO_PIN_SET);
		}
		if(tg6 == 0x80)
		{
			HAL_GPIO_WritePin(MT_R3_GPIO_Port, MT_R3_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(MT_R3_GPIO_Port, MT_R3_Pin, GPIO_PIN_SET);
		}
		if(tg7 == 0x80)
		{
			HAL_GPIO_WritePin(MT_R2_GPIO_Port, MT_R2_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(MT_R2_GPIO_Port, MT_R2_Pin, GPIO_PIN_SET);
		}
		if(tg8 == 0x80)
		{
			HAL_GPIO_WritePin(MT_R1_GPIO_Port, MT_R1_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(MT_R1_GPIO_Port, MT_R1_Pin, GPIO_PIN_SET);
		}
		HAL_GPIO_WritePin(MT_CLK_GPIO_Port, MT_CLK_Pin, GPIO_PIN_RESET);
		data1 <<=1;
		data2 <<=1;
		data3 <<=1;
		data4 <<=1;
		data5 <<=1;
		data6 <<=1;
		data7 <<=1;
		data8 <<=1;
		HAL_GPIO_WritePin(MT_CLK_GPIO_Port, MT_CLK_Pin, GPIO_PIN_SET);
	}
}
void MT_Send(uint8_t data)
{
	uint8_t i,tg;

	for(i=0;i<8;i++)
	{
		tg = data;
		tg = tg&0x80;
		if(tg == 0x80)
		{
			HAL_GPIO_WritePin(MT_R8_GPIO_Port, MT_R8_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MT_R7_GPIO_Port, MT_R7_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MT_R6_GPIO_Port, MT_R6_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MT_R5_GPIO_Port, MT_R5_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MT_R4_GPIO_Port, MT_R4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MT_R3_GPIO_Port, MT_R3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MT_R2_GPIO_Port, MT_R2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MT_R1_GPIO_Port, MT_R1_Pin, GPIO_PIN_RESET);


		}
		else
		{
			HAL_GPIO_WritePin(MT_R8_GPIO_Port, MT_R8_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MT_R7_GPIO_Port, MT_R7_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MT_R6_GPIO_Port, MT_R6_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MT_R5_GPIO_Port, MT_R5_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MT_R4_GPIO_Port, MT_R4_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MT_R3_GPIO_Port, MT_R3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MT_R2_GPIO_Port, MT_R2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MT_R1_GPIO_Port, MT_R1_Pin, GPIO_PIN_SET);
		}
		HAL_GPIO_WritePin(MT_CLK_GPIO_Port, MT_CLK_Pin, GPIO_PIN_RESET);
		data <<=1;
		HAL_GPIO_WritePin(MT_CLK_GPIO_Port, MT_CLK_Pin, GPIO_PIN_SET);
	}
}

uint8_t buffer_quet1[SO_BUFFER_RAM];
uint8_t buffer_quet2[SO_BUFFER_RAM];
uint8_t buffer_quet3[SO_BUFFER_RAM];
uint8_t buffer_quet4[SO_BUFFER_RAM];
uint8_t buffer_quet5[SO_BUFFER_RAM];
uint8_t buffer_quet6[SO_BUFFER_RAM];
uint8_t buffer_quet7[SO_BUFFER_RAM];
uint8_t buffer_quet8[SO_BUFFER_RAM];

extern GUI_CONST_STORAGE GUI_CHARINFO GUI_FontArialNarrow16_CharInfo[657];
extern GUI_CONST_STORAGE GUI_CHARINFO GUI_FontHPSimplified15_CharInfo[124];

u8 write_char(u8 font,u8 data, u8 x, u8 y)
{
    u8 i,j,len_heigh;
    unsigned char * pointer;
    pointer = (unsigned char *)GUI_FontArialNarrow16_CharInfo[data - 0x20].pData;
    len_heigh = GUI_FontArialNarrow16_CharInfo[data - 0x20].BytesPerLine;
    for(j=0;j<16;j++)
    for(i=0;i<len_heigh;i++)
    {
    	buffer_quet1[j*SO_BYTE_TREN_HANG + i + x] = pointer[j*len_heigh + i];
    }
    return len_heigh;
}

u8 write_char1(u8 font,u8 data, u16 x, u16 y)
{
    u16 i,j,k,len_heigh,max_bit,data_font,count_bit = 0,count_font = 0,tg;
    unsigned char * pointer;
//    pointer = (unsigned char *)GUI_FontArialNarrow16_CharInfo[data - 0x20].pData;
//    len_heigh = GUI_FontArialNarrow16_CharInfo[data - 0x20].BytesPerLine;
//    max_bit = GUI_FontArialNarrow16_CharInfo[data - 0x20].XSize;

    pointer = (unsigned char *)GUI_FontHPSimplified15_CharInfo[data - 0x20].pData;
    len_heigh = GUI_FontHPSimplified15_CharInfo[data - 0x20].BytesPerLine;
    max_bit = GUI_FontHPSimplified15_CharInfo[data - 0x20].XSize;

    for(j=0;j<15;j++)//15 là số hàng của font
    {
    	count_bit = 0;//xoa count bit
		for(i=0;i<len_heigh;i++)//lặp số cột của font
		{
			data_font = pointer[count_font++];//lấy data từ font
			for(k=0;k<8;k++)
			{
				if(x+count_bit >= 64) break;

				//kiểm tra bit của font để set pixel
				if( (data_font &0x80) == 0x80)
					setpixel(x+count_bit, y+j, 1);
				else
					setpixel(x+count_bit, y+j, 0);
				data_font *= 2;

				count_bit++;//nếu số bit đếm = max bit của font thì break
				if(count_bit>=max_bit) break;
			}
	//    	buffer_quet1[j*SO_BYTE_TREN_HANG + i + x] = pointer[j*len_heigh + i];

		}
    }

    //check bảo hành
	if(flag_hetBH == 1)
	{
		for(i=0;i<8;i++)
		{
			for(j=0;j<8;j++)
			{
				setpixel(5 + i,5 + j, 0);

				setpixel(14 + i,16 + j, 0);

				setpixel(24 + i,32 + j, 0);

				setpixel(34 + i,48 + j, 0);

				setpixel(44 + i,64 + j, 0);

			}
		}
	}
    return max_bit;
}

void clear_lcd(void)
{
	for(uint16_t i=0;i<SO_BUFFER_RAM;i++)
	{
		buffer_quet1[i] = 0;
		buffer_quet2[i] = 0;
		buffer_quet3[i] = 0;
		buffer_quet4[i] = 0;
		buffer_quet5[i] = 0;
		buffer_quet6[i] = 0;
		buffer_quet7[i] = 0;
		buffer_quet8[i] = 0;
	}
}

u8 write_string(u8 font,u8 *data, u8 x, u8 y)
{
    u8 x_pointer = 0;
    while(*data)
    {
        x_pointer += write_char1(0,*data,x_pointer,y) -1;
        data++;

    }
    return x_pointer;
}

void led_init(void)
{
	for(uint16_t j=0;j<SO_BUFFER_RAM;j++)
	{
		buffer_quet1[j] = 0;
	}
	HAL_TIM_Base_Start_IT(&htim1);

#ifdef LED_TEST
	for(uint16_t y=0;y<80;y++)
	for(uint16_t x=0;x<64;x++)
	{
	  setpixel(x, y, 1);
//	  HAL_Delay(1);
	}
	HAL_Delay(2000);
	for(uint16_t y=0;y<80;y++)
	for(uint16_t x=0;x<64;x++)
	{
	  setpixel(x, y, 0);
//	  HAL_Delay(1);
	}
#endif
}

void quetledMT(void)
{
	static uint8_t quetlled = 0;

	//day data ra
	for(uint8_t i=0;i<((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8);i++)
	{
//	MT_Send(buffer_quet1[48*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)]);
//	MT_Send(buffer_quet1[32*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)]);
//	MT_Send(buffer_quet1[16*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)]);
//	MT_Send(buffer_quet1[0  + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)]);
	MT_Send1(buffer_quet1[48*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet2[48*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet3[48*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet4[48*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet5[48*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet6[48*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet7[48*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet8[48*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)]);
	MT_Send1(buffer_quet1[32*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet2[32*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet3[32*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet4[32*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet5[32*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet6[32*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet7[32*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet8[32*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)]);
	MT_Send1(buffer_quet1[16*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet2[16*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet3[16*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet4[16*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet5[16*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet6[16*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet7[16*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet8[16*SO_BANG_SU_DUNG + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)]);
	MT_Send1(buffer_quet1[0  + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet2[0  + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet3[0  + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet4[0  + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet5[0  + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet6[0  + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet7[0  + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)],
			buffer_quet8[0  + i + quetlled*((SO_DOT_TREN_HANG*SO_BANG_SU_DUNG)/8)]);
	}

	HAL_GPIO_WritePin(MT_OE_GPIO_Port, MT_OE_Pin, GPIO_PIN_RESET);
	//chot du lieu
	HAL_GPIO_WritePin(MT_LAT_GPIO_Port, MT_LAT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MT_LAT_GPIO_Port, MT_LAT_Pin, GPIO_PIN_RESET);

	//tat data di


	//count quet led
	if(quetlled == 0)
	{
	HAL_GPIO_WritePin(MT_A_GPIO_Port, MT_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_B_GPIO_Port, MT_B_Pin, GPIO_PIN_RESET);
	}
	else if(quetlled == 1)
	{
	HAL_GPIO_WritePin(MT_A_GPIO_Port, MT_A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MT_B_GPIO_Port, MT_B_Pin, GPIO_PIN_RESET);
	}
	else if(quetlled == 2)
	{
	HAL_GPIO_WritePin(MT_A_GPIO_Port, MT_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_B_GPIO_Port, MT_B_Pin, GPIO_PIN_SET);
	}
	else if(quetlled == 3)
	{
	HAL_GPIO_WritePin(MT_A_GPIO_Port, MT_A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MT_B_GPIO_Port, MT_B_Pin, GPIO_PIN_SET);
	}

	quetlled ++;
	if(quetlled > 3) quetlled = 0;
	HAL_GPIO_WritePin(MT_OE_GPIO_Port, MT_OE_Pin, GPIO_PIN_SET);
//	HAL_Delay(2);
}

void setpixel(uint16_t x,uint16_t y,uint8_t data)// x sẽ từ 0 -> 64 , y sẽ từ 0 -> 128
{
	uint16_t convertx,converty,offsetx,offsety;

	convertx = x/8;//chia lấy nguyên sẽ ra vị trí
	offsetx = x-convertx*8;//lấy phần dư sẽ ra vị trí cần set điểm

	converty = y/SO_DOT_TREN_COT;//chia dư cho 16 để lấy ra xem là mảng nào
	offsety = y - converty*SO_DOT_TREN_COT;

	if(data == 1)//nếu data = 1 thì dùng phép |
	{
		if(converty == 0)// nếu là 0 thì là của buffer số 1
		{
			buffer_quet1[offsety*SO_BYTE_TREN_HANG + convertx] |= (1<<(7 - offsetx));
		}
		else if(converty == 1)// nếu là 1 thì là của buffer số 2
		{
			buffer_quet2[offsety*SO_BYTE_TREN_HANG + convertx] |= (1<<(7 - offsetx));
		}
		else if(converty == 2)// nếu là 2 thì là của buffer số 3
		{
			buffer_quet3[offsety*SO_BYTE_TREN_HANG + convertx] |= (1<<(7 - offsetx));
		}
		else if(converty == 3)// nếu là 3 thì là của buffer số 4
		{
			buffer_quet4[offsety*SO_BYTE_TREN_HANG + convertx] |= (1<<(7 - offsetx));
		}
		else if(converty == 4)// nếu là 4 thì là của buffer số 5
		{
			buffer_quet5[offsety*SO_BYTE_TREN_HANG + convertx] |= (1<<(7 - offsetx));
		}
		else if(converty == 5)// nếu là 5 thì là của buffer số 6
		{
			buffer_quet6[offsety*SO_BYTE_TREN_HANG + convertx] |= (1<<(7 - offsetx));
		}
		else if(converty == 6)// nếu là 6 thì là của buffer số 7
		{
			buffer_quet7[offsety*SO_BYTE_TREN_HANG + convertx] |= (1<<(7 - offsetx));
		}
		else if(converty == 7)// nếu là 7 thì là của buffer số 8
		{
			buffer_quet8[offsety*SO_BYTE_TREN_HANG + convertx] |= (1<<(7 - offsetx));
		}

	}
	else// nếu khác 1 thì clear data đi để xóa điểm led
	{
		if(converty == 0)// nếu là 0 thì là của buffer số 1
		{
			buffer_quet1[offsety*SO_BYTE_TREN_HANG + convertx] &= (~(1<<(7 - offsetx)));
		}
		else if(converty == 1)// nếu là 1 thì là của buffer số 2
		{
			buffer_quet2[offsety*SO_BYTE_TREN_HANG + convertx] &= (~(1<<(7 - offsetx)));
		}
		else if(converty == 2)// nếu là 2 thì là của buffer số 3
		{
			buffer_quet3[offsety*SO_BYTE_TREN_HANG + convertx] &= (~(1<<(7 - offsetx)));
		}
		else if(converty == 3)// nếu là 3 thì là của buffer số 4
		{
			buffer_quet4[offsety*SO_BYTE_TREN_HANG + convertx] &= (~(1<<(7 - offsetx)));
		}
		else if(converty == 4)// nếu là 4 thì là của buffer số 5
		{
			buffer_quet5[offsety*SO_BYTE_TREN_HANG + convertx] &= (~(1<<(7 - offsetx)));
		}
		else if(converty == 5)// nếu là 5 thì là của buffer số 6
		{
			buffer_quet6[offsety*SO_BYTE_TREN_HANG + convertx] &= (~(1<<(7 - offsetx)));
		}
		else if(converty == 6)// nếu là 6 thì là của buffer số 7
		{
			buffer_quet7[offsety*SO_BYTE_TREN_HANG + convertx] &= (~(1<<(7 - offsetx)));
		}
		else if(converty == 7)// nếu là 7 thì là của buffer số 8
		{
			buffer_quet8[offsety*SO_BYTE_TREN_HANG + convertx] &= (~(1<<(7 - offsetx)));
		}
	}
}
