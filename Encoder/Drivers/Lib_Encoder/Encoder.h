/*
 * Encoder.h
 *
 *  Created on: Jul 9, 2024
 *      Author: Khanh
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "stdint.h"
#include "main.h"

//typedef struct {
//	int16_t velocity;
//	int16_t possition;
//	uint32_t last_counter_value;
//} encoder_instance;

#define NUMBER_OF_TICKS_PER_REV       500
#define ONE_REV_LENGTH_CM             22
#define TWOPI                         6.28318530718  // radians per rotation

typedef struct{
	float velocity; /* velocity of the motor radians/sec*/
	float position; /* position of the motor (radians) */
	int64_t last_counter_value; /* counter value for the last iteration*/
	float timer_period; // period
	TIM_HandleTypeDef *htim_encoder; /* timer instance*/
	uint8_t first_time; /*flag that shows whether it has been run for the first time*/
}encoder_inst;


void get_encoder_speed(encoder_inst *encoder);
void reset_encoder(encoder_inst *encoder);

//void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim, float update_interval, uint32_t PPR);
//void reset_encoder(encoder_instance *encoder_value);
#endif /* ENCODER_H_ */
