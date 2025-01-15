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
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "ssd1306_fonts.h"
#include <string.h>
#include <stdio.h>
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define OK 			HAL_GPIO_ReadPin(BT10_OK_GPIO_Port, BT10_OK_Pin)
//#define UP 			HAL_GPIO_ReadPin(BT9_DOWN_GPIO_Port, BT9_DOWN_Pin)
//#define DOWN 		HAL_GPIO_ReadPin(BT11_UP_GPIO_Port, BT11_UP_Pin)
//#define BACK		HAL_GPIO_ReadPin(BT1_UP_GPIO_Port, BT1_UP_Pin)

//#define UP			HAL_GPIO_ReadPin(BT1_UP_GPIO_Port, BT1_UP_Pin)						// Dieu khien len
//#define DOWN		HAL_GPIO_ReadPin(BT3_DOW_GPIO_Port, BT3_DOW_Pin)					// Dieu khien xuong
//#define LEFT		HAL_GPIO_ReadPin(BT4_L_GPIO_Port, BT4_L_Pin)						// Dieu khien sang trai
//#define RIGHT		HAL_GPIO_ReadPin(BT2_R_GPIO_Port, BT2_R_Pin)					// Dieu khien sang phai
//
//#define BACK		HAL_GPIO_ReadPin(BT9_SELECT_GPIO_Port, BT9_SELECT_Pin)
//#define SELECT		HAL_GPIO_ReadPin(BT11_BACK_GPIO_Port, BT11_BACK_Pin)

//#define UP_RIGHT 		HAL_GPIO_ReadPin(BT5_UP_RIGHT_GPIO_Port, BT5_UP_RIGHT_Pin)
//#define DOWN_RIGHT 		HAL_GPIO_ReadPin(BT7_DOWN_RIGHT_GPIO_Port, BT7_DOWN_RIGHT_Pin)
//#define LEFT_R			HAL_GPIO_ReadPin(BT8_L_GPIO_Port, BT8_L_Pin)
//#define RIGHT_R			HAL_GPIO_ReadPin(BT6_R_GPIO_Port, BT6_R_Pin)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

const char *Option[3] =
{
	"Game 1",		// Game Pong
	"Game 2",		// Flappy bird
	"Game 3"		//
};

typedef struct {
    GPIO_TypeDef* Port;
    uint16_t Pin;
    uint8_t State;
} Button;

Button buttons[] = {
    {BT1_UP_GPIO_Port, BT1_UP_Pin, 1},   // UP
    {BT3_DOW_GPIO_Port, BT3_DOW_Pin, 1}, // DOWN
    {BT4_L_GPIO_Port, BT4_L_Pin, 1},     // LEFT
    {BT2_R_GPIO_Port, BT2_R_Pin, 1},     // RIGHT
    {BT9_SELECT_GPIO_Port, BT9_SELECT_Pin, 1}, // SELECT
    {BT11_BACK_GPIO_Port, BT11_BACK_Pin, 1},    // BACK

	{BT5_UP_RIGHT_GPIO_Port, BT5_UP_RIGHT_Pin},
	{BT7_DOWN_RIGHT_GPIO_Port, BT7_DOWN_RIGHT_Pin},
	{BT8_L_GPIO_Port, BT8_L_Pin},
	{BT6_R_GPIO_Port, BT6_R_Pin}
};

enum {UP, DOWN, LEFT, RIGHT, SELECT, BACK, UP_R, DOWN_R, LEFT_R, RIGHT_R};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
int trangthai = 1;
int cnt = 0;

uint8_t tmp_up_r = 0;
uint8_t tmp_down_r = 0;
uint8_t tmp_left_r = 0;
uint8_t tmp_right_r = 0;

int select = 0;
int enter = -1;
int enter1 = -1;

// Khai bao cac bien cho nut nhan Menu
uint8_t tt_up = 1;					// bien trang thai cho nut nhan Len
uint8_t tt_down = 1;				// Bien trang thai cho nut nhan Xuong
uint8_t tt_left = 1;
uint8_t tt_right = 1;
uint8_t tt_slect = 1;					// Bien trang thai cho nut nhan OK
uint8_t tt_back = 1;				// Bien trang thai cho nut Back
uint8_t tt_up_r = 1;
uint8_t tt_down_r = 1;
uint8_t tt_left_r = 1;
uint8_t tt_right_r = 1;

uint8_t cnt_menu = 0;				// Bien dem trong su dung trong Menu
uint8_t cnt_Oled = 0;

uint8_t pre_cnt_menu = 0;
uint8_t pre_cnt_Oled = 0;
uint8_t pre_enter = -1;
uint8_t pre_enter1 = -1;
int i;

// Bien Joystick
char buffer_adc_x_right[2];
char buffer_adc_y_right[2];
uint16_t Read_ADC[2];


/* Bien Game Pong*/
const unsigned long PADDLE_RATE = 20;		//	Toc do cua thanh trươt
const unsigned long BALL_RATE = 5;			// Toc do cua bong
const uint8_t PADDLE_HEIGHT = 10;			// Chiều cao của thanh truot

uint8_t ball_x = 64, ball_y = 32;			// Toa do ban dau cua bong
uint8_t ball_dir_x = 1, ball_dir_y = 1;		// Huong di chuyen cua bong theo truc x va truc y
unsigned long ball_update;					// Luu thoi gian tiep theo cua bong

unsigned long paddle_update;				// Luu thoi gian tiep theo cua thanh truot
const uint8_t CPU_X = 12;					// Toa do X cua thanh truot
uint8_t CPU_Y = 16;							// Toa do Y cua thanh trươt

const uint8_t PLAYER_X = 115;				// To do nguoi choi X
uint8_t PLAYER_Y = 16;						// Toa do cua nguoi choi y

#define BALL_SIZE 3							// Kich thuoc cua bong

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Bien theo doi qua trinh bong di chuyen qua diem o giua
uint8_t cnt_points1 = 0;					// Bien dem khi bong di qua vi tri giua
uint8_t cnt_points2 = 0;
uint8_t tmp_point = 0;							// Bien theo doi khi bong di qua vi tri giua

uint8_t Ar_point[10];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);

void Read_BT(void);
void Display_OLED(void);
void Read_Buttons(void);

void drawCourt(void);
void Game_Pong(void);
void Wait_Update(void);
void Reset_Game_Pong(void);
void Mid_line(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Ham Kiem tra nut nhan Doc Gia Tri Nut Nhan khi chay thu
void Read_BT(void)
{
	// Doc Nut Nhan BT1			==> UP
    if (UP == 0)
	{
		if (tt_up == 1)
		{
			cnt++;
			tt_up = 0;
		}
	}
	else
	{
		tt_up = 1;
	}

    // Doc Nut Nhan BT2			==> RIGHT
    if (RIGHT == 0)
	{
		if (tt_right == 1)
		{
			cnt++;
			tt_right = 0;
		}
	}
	else
	{
		tt_right = 1;
	}

    // Doc Nut Nhan BT3			==> DOWN
	if (DOWN == 0)
	{
		if (tt_down == 1)
		{
			cnt++;
			tt_down = 0;
		}
	}
	else
	{
		tt_down = 1;
	}

	// Doc Nut Nhan BT4			==> LEFT
	if (LEFT == 0)
	{
		if (tt_left == 1)
		{
			cnt--;
			tt_left = 0;
		}
	}
	else
	{
		tt_left = 1;
	}

	// Doc Nut Nhan BT9			==> Select
	if (SELECT == 0)
	{
		if (tt_slect == 1)
		{
			cnt++;
			tt_slect = 0;
		}
	}
	else
	{
		tt_slect = 1;
	}

	// Doc Nut Nhan BT11			==> Back
	if (BACK == 0)
	{
		if (tt_back == 1)
		{
			cnt++;
			tt_back = 0;
		}
	}
	else
	{
		tt_back = 1;
	}

	// Doc nut nhan BT5
	if(UP_R == 0)
	{
		if(tt_up_r == 1)
		{
			tmp_up_r++;
			tt_up_r = 0;
		}
	}
	else
	{
		tt_up_r = 1;
	}

	// Doc nut nhan BT6
	if(DOWN_R == 0)
	{
		if(tt_right_r == 1)
		{
			tmp_right_r++;
			tt_right_r = 0;
		}
	}
	else
	{
		tt_right_r = 1;
	}
	// Doc nut ngan BT7
	if(LEFT_R == 0)
	{
		if(tt_down_r == 1)
		{
			tmp_down_r++;
			tt_down_r = 0;
		}
	}
	else
	{
		tt_down_r = 1;
	}
	// Doc nut nhan BT8
	if(DOWN_R == 0)
	{
		if(tt_right_r == 1)
		{
			tmp_right_r++;
			tt_down_r = 0;
		}
	}
	else
	{
		tt_down_r = 1;
	}
}

void Read_Buttons(void) {
    for (int i = 0; i < sizeof(buttons) / sizeof(buttons[0]); i++) {
        if (HAL_GPIO_ReadPin(buttons[i].Port, buttons[i].Pin) == GPIO_PIN_RESET) {
            if (buttons[i].State == 1) {
                buttons[i].State = 0;
                switch (i) {
                    case UP:
                    	cnt_menu--;
                    	//cnt_Oled = -1;
                    	break;
                    case DOWN:
                    	cnt_menu++;
                    	//cnt_Oled = -1;
                    	break;
                    case LEFT:
                    	cnt_Oled--;
                    	break;
                    case RIGHT:
                    	cnt_Oled++;
                    	break;
                    case SELECT:
                    	enter = cnt_Oled;
                    	enter1 = cnt_menu;

                    	break;
                    case BACK:
                    	enter = -1;
                    	enter1 = -1;
                    	cnt_menu = 0;
                    	Display_OLED();
                    	break;
                }
            }
        } else {
            buttons[i].State = 1;
        }
    }
}

// Ham Hien thi tren man hinh OLED
void Display_OLED(void)
{
	ssd1306_Fill(Black);
	ssd1306_DrawBitmap(0,0,background3,128,64,White);
	ssd1306_DrawBitmap(0, 0, epd_bitmap__a_frm0_40, 128, 64, White);

	ssd1306_SetCursor(6, 53);
	ssd1306_WriteString("Menu", Font_7x10, White);

	ssd1306_SetCursor(90, 53);
	ssd1306_WriteString(" Back", Font_7x10, White);

	ssd1306_UpdateScreen();

}

// Hien thi trong Menu
void Display_Menu(void)
{
	ssd1306_Clear();
	ssd1306_Fill(Black);
	for(int i = 0; i < 3; i++)
	{
		ssd1306_SetCursor(7, 2 + i*20);
		ssd1306_WriteString(Option[i], Font_11x18, White);

		if(cnt_menu == (i + 1))
		{
			ssd1306_DrawRectangle(0, 2 + i * 20, 127, 18 + i * 20, White);
		}

		if(cnt_menu >= 3)
		{
			cnt_menu = 3;
		}
	}
	ssd1306_UpdateScreen();
}

// Ham dieu khien trang thai trong Menu
void Display_Select(void)
{
	Read_Buttons();
	if((cnt_menu != pre_cnt_menu) || (enter != pre_enter) || (cnt_Oled != pre_cnt_Oled) || (enter1 != pre_enter1))
	{
		// Thuc hien ngoai man hinh chinh
		if(cnt_Oled == 1)
		{
			ssd1306_SetCursor(87, 52);
			ssd1306_WriteString(" ", Font_7x10, White);
			ssd1306_SetCursor(0, 52);
			ssd1306_WriteString(">", Font_7x10, White);
		}

		else if(cnt_Oled == 2)
		{
			ssd1306_SetCursor(0, 52);
			ssd1306_WriteString(" ", Font_7x10, White);
			ssd1306_SetCursor(87, 52);
			ssd1306_WriteString(">", Font_7x10, White);
		}
		else if(cnt_Oled > 2)
		{
			cnt_Oled = 2;
		}

		if((cnt_Oled == 1) && (enter == 1))
		{
			Display_Menu();
		}
		if((cnt_menu == 1) && (enter1 == 1))
		{
			SSD1306_Clear();
			ssd1306_Fill(Black);
			drawCourt();
			Game_Pong();
			enter1 = 1;
		}
		if((cnt_menu == 2) && (enter1 == 2))
		{

			SSD1306_Clear();
			ssd1306_Fill(Black);
			ssd1306_DrawBitmap(0, 0, Image4, 128, 64, White);
			enter1 = 2;
		}

		if((cnt_menu == 3) && (enter1 == 3))
		{
			SSD1306_Clear();
			ssd1306_Fill(Black);
			ssd1306_DrawBitmap(0, 0, garfield_128x64, 128, 64, White);
			enter1 = 3;
		}
	}

	ssd1306_UpdateScreen();
	// Cap nhat lai gia tri cua bien "cnt_menu" va "enter" de khong bi nhay khi cho vao vong lap "White"
	pre_cnt_menu = cnt_menu;
	pre_cnt_Oled = cnt_Oled;
	pre_enter = enter;
	pre_enter1 = enter1;
}

/* GAME */

/* Ham ve khung vien*/
void drawCourt(void) {
    ssd1306_DrawRectangle(0, 0, 127, 63, White);
    Mid_line();
    ssd1306_UpdateScreen();
}

void Game_Pong(void)
{
	uint8_t  update = 0;
	unsigned long time = HAL_GetTick();
	 static uint8_t up_state = 0;
	 static uint8_t down_state = 0;

	 up_state |= (HAL_GPIO_ReadPin(BT1_UP_GPIO_Port, BT1_UP_Pin) == 0);					// Đọc trạng thái nút nhấn lên, nếu được nhấn trả về 1, ngược lại trả về 0
	 down_state |= (HAL_GPIO_ReadPin(BT3_DOW_GPIO_Port, BT3_DOW_Pin) == 0);				// Đọc trạng thái nút nhấn xuong, nếu được nhấn trả về 1, ngược lại trả về 0

	 if(time > ball_update)
	 {
		 uint8_t new_x = ball_x + ball_dir_x;					// Tinh vi tri x cua bong
		 uint8_t new_y = ball_y + ball_dir_y;					// Tinh vi tri y cua bong

		 /*Neu bong tram vao canh trai hoac phai thi dao nguoc lai huong di chuyen cua X (Tuc la cham vao thanh danh bong)*/
		 if(new_x == 2 || new_y == 125)
		 {
			 tmp_point = 0;
			 ball_dir_x = -ball_dir_x;
			 new_x += ball_dir_x + ball_dir_x;
		 }

		 /*Neu bong tram vao canh tren hoac duoi thi dao nguoc lai huong di chuyen cua Y */
		 else if(new_y == 0 || new_y == 63)
		 {
			 tmp_point = 0;
			 ball_dir_y = -ball_dir_y;
			 new_y += ball_dir_y + ball_dir_y;
		 }

		 else if(new_x == CPU_X && new_y >= CPU_Y && new_y <= CPU_Y + PADDLE_HEIGHT)
		 {
			 tmp_point = 0;
			 ball_dir_x = -ball_dir_x;
			 new_x += ball_dir_x + ball_dir_x;
		 }

		 else if(new_x == PLAYER_X && new_y >= PLAYER_Y && new_y <= PLAYER_Y + PADDLE_HEIGHT)
		 {
			 tmp_point = 0;
			 ball_dir_x = -ball_dir_x;
			 new_x += ball_dir_x + ball_dir_x;
		 }

		 /*		TH khi nguoi choi thang*/
		 else if((new_x == 64) && (new_y != 64) && (tmp_point == 0))
		 {
			 cnt_points1++;
			 tmp_point = 1;
			 ssd1306_SetCursor(30, 30);
			 sprintf(Ar_point,"%d", cnt_points1);
			 ssd1306_WriteChar(Ar_point, Font_6x8, White);
			 ssd1306_UpdateScreen();

			 // Neu so len di qua vach giua lon hon hoac bang 5 thi thang
			 if(cnt_points1 >= 5)
			 {
				 ssd1306_Fill(Black);
				 ssd1306_SetCursor(40, 32);
				 ssd1306_WriteString("You Win", Font_7x10, White);
				 cnt_points1 = 0;

				 ssd1306_UpdateScreen();

				 Wait_Update();
				 Reset_Game_Pong();
			 }
		 }

		 else if(new_x <= 2 || new_x >= 125)
		 {
			 tmp_point = 0;
			 cnt_points1 = 0;
			 ssd1306_Fill(Black);
			 ssd1306_SetCursor(40, 32);
			 ssd1306_WriteString("Game Over", Font_7x10, White);
			 ssd1306_UpdateScreen();

			 Wait_Update();
			 Reset_Game_Pong();
		 }
		 else
		 {
			 ssd1306_DrawPixel(ball_x, ball_y, Black);		// Xoa bong cu khi danh ra ngoai bang cach to mau den
			 ssd1306_DrawPixel(new_x, new_y, White);		// Cap nhat lai vi tri cua bong khi danh ra ngoai

			 /*Sua kich thuoc bong lon hon bi treo chip*/
	 //		 ssd1306_FillCircle(ball_x, ball_y, BALL_SIZE, Black);
	 //		 ssd1306_FillCircle(new_x, new_y, BALL_SIZE, White);
			 ball_x = new_x;
			 ball_y = new_y;
			 ball_update += BALL_RATE;
			 update = 1;
		 }
	}
	 if(time > paddle_update) {
		 paddle_update += PADDLE_RATE;

		 ssd1306_Line(CPU_X, CPU_Y, CPU_X, CPU_Y + PADDLE_HEIGHT, Black);
		 const uint8_t half_paddle = PADDLE_HEIGHT >> 1;
		 if(CPU_Y + half_paddle > ball_y) {
			 CPU_Y -= 1;
		 }
		 if(CPU_Y + half_paddle < ball_y) {
			 CPU_Y += 1;
		 }
		 if(CPU_Y < 1) CPU_Y = 1;
		 if(CPU_Y + PADDLE_HEIGHT > 63) CPU_Y = 63 - PADDLE_HEIGHT;
		 ssd1306_Line(CPU_X, CPU_Y, CPU_X, CPU_Y + PADDLE_HEIGHT, White);

		 ssd1306_Line(PLAYER_X, PLAYER_Y, PLAYER_X, PLAYER_Y + PADDLE_HEIGHT, Black);
		 if(up_state) {
			 PLAYER_Y -= 1;
		 }
		 if(down_state) {
			 PLAYER_Y += 1;
		 }
		 up_state = down_state = 0;
		 if(PLAYER_Y < 1) PLAYER_Y = 1;
		 if(PLAYER_Y + PADDLE_HEIGHT > 63) PLAYER_Y = 63 - PADDLE_HEIGHT;
		 ssd1306_Line(PLAYER_X, PLAYER_Y, PLAYER_X, PLAYER_Y + PADDLE_HEIGHT, White);

		 update = 1;
	 }

	 if(update) {
		 ssd1306_UpdateScreen();
	 }
}

/* Khi nhan nut len hoac xuong se bat choi lai tu dau
 * 	Co tac dung khi nguoi choi thua cuoc
 *
 * */
void Wait_Update(void)
{
	while(1)
	{
		if((HAL_GPIO_ReadPin(BT1_UP_GPIO_Port, BT1_UP_Pin) == 0) || (HAL_GPIO_ReadPin(BT3_DOW_GPIO_Port, BT3_DOW_Pin) == 0))
		{
			while((HAL_GPIO_ReadPin(BT1_UP_GPIO_Port, BT1_UP_Pin) == 0) || (HAL_GPIO_ReadPin(BT3_DOW_GPIO_Port, BT3_DOW_Pin) == 0));
			break;
		}
	}
}

void Reset_Game_Pong(void)
{
	// Đặt lại vị trí của bóng ở giữa màn hình
	ball_x = SCREEN_WIDTH / 2 - BALL_SIZE / 2;
	ball_y = SCREEN_HEIGHT / 2 - BALL_SIZE / 2;

	// Đặt lại hướng bóng
	ball_dir_x = (rand() % 2 == 0) ? 1 : -1;
	ball_dir_y = (rand() % 2 == 0) ? 1 : -1;

	// Đặt lại vị trí của thanh truot danh bong
	CPU_Y = SCREEN_HEIGHT / 2 - PADDLE_HEIGHT / 2;
	PLAYER_Y = SCREEN_HEIGHT / 2 - PADDLE_HEIGHT / 2;

	// Xóa toàn bộ màn hình và vẽ lại sân chơi
	ssd1306_Fill(Black);
	drawCourt();
	ssd1306_UpdateScreen();
}

// Ke duong thang o giua
void Mid_line(void)
{
	ssd1306_Line(64, 64, 64, 0, White);
	ssd1306_UpdateScreen();
}

/*	Game 2: Flappy Bird		*/

/*
 * 			****		Value:	64
 * 			*  *
 * 			*  *
 * 			*  *
 * 			****
 *
 *			**
 *			***
 *			**					0
 * Bat dau game, vat the xuat hien o giua man hinh ben trai
 *	+, Co 2 nut nhan chinh: tang va giam (hoac co the dung 1 nut nhan tang) moi lan nhan tang 1 pixel (pixel duoc lay gia tri tu  0 den 64)
 *		-, Khi khong nhan nut vat the se roi tu diem dang dung ve 0
 *		-, Moi khi di qua ong tinh la 1 diem
 *		-, Khi cham vao Ong hoac roi ve vi tri 0 Thua cuoc
 * */

void Reset_Game_Flappy_Bird(void);			// Ham Reset Game Flappy brid
void Game_Flappy_Bird(void)
{
	uint8_t update_Game = 0;
	uint8_t Time = HAL_GetTick();
	static uint16_t Up_Value = 0;
	static uint16_t Down_Value = 0;

	Up_Value |= (HAL_GPIO_ReadPin(BT1_UP_GPIO_Port, BT1_UP_Pin) ==  0);				// nut nhan cho vat the tang
	Down_Value |= (HAL_GPIO_ReadPin(BT3_DOW_GPIO_Port, BT3_DOW_Pin) == 0);			// Nut nhan cho vat the giam

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	unsigned long Start = HAL_GetTick();
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  drawCourt();
  /*Sau 2s Bong moi xuat hien*/
  while(HAL_GetTick() - Start < 2000);
  ball_update = HAL_GetTick();
  paddle_update = ball_update;


  //  ssd1306_Fill(Black);
//  ssd1306_DrawBitmap(0, 0, epd_bitmap__a_frm0_40, 128, 64, White);
//
//  ssd1306_UpdateScreen();
  //Display_OLED();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //Read_BT();
	  //Display_Select();
	  // Doc gia tri ADC
//	  snprintf(buffer_adc_x_right, sizeof(buffer_adc_x_right), "%d",Read_ADC[1]);
//	  snprintf(buffer_adc_y_right, sizeof(buffer_adc_y_right), "%d",Read_ADC[2]);
	  //Read_Buttons();
	 Game_Pong();

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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Test_GPIO_Port, LED_Test_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Button_Joystick_Left_Pin BT1_UP_Pin BT4_L_Pin BT3_DOW_Pin
                           BT2_R_Pin BT7_DOWN_RIGHT_Pin BT6_R_Pin BT5_UP_RIGHT_Pin
                           Button_Joystick_Right_Pin */
  GPIO_InitStruct.Pin = Button_Joystick_Left_Pin|BT1_UP_Pin|BT4_L_Pin|BT3_DOW_Pin
                          |BT2_R_Pin|BT7_DOWN_RIGHT_Pin|BT6_R_Pin|BT5_UP_RIGHT_Pin
                          |Button_Joystick_Right_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Test_Pin */
  GPIO_InitStruct.Pin = LED_Test_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Test_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BT11_BACK_Pin BT10_OK_Pin BT9_SELECT_Pin BT8_L_Pin */
  GPIO_InitStruct.Pin = BT11_BACK_Pin|BT10_OK_Pin|BT9_SELECT_Pin|BT8_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
