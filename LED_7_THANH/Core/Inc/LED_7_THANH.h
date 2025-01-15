/*
 * LED_7_THANH.h
 *
 *  Created on: Sep 5, 2023
 *      Author: My_Friend
 */

#ifndef INC_LED_7_THANH_H_
#define INC_LED_7_THANH_H_
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern uint16_t dem;
extern uint8_t so1, so2, so3, so4;
extern uint8_t mang_led[10];
extern uint8_t count_quet_led;

void led_init(void);

#endif /* INC_LED_7_THANH_H_ */
