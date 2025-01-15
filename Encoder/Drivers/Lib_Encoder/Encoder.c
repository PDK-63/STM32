/*
 * Encoder.c
 *
 *  Created on: Jul 9, 2024
 *      Author: Khanh
 */

#include "Encoder.h"
#define NUMBER_OF_TICKS_PER_REV       500
#define ONE_REV_LENGTH_CM             22
#define TWOPI                         6.28318530718  // radians per rotation

void get_encoder_speed(encoder_inst *encoder)
{
	int64_t temp_counter = __HAL_TIM_GET_COUNTER(encoder -> htim_encoder);
	float scaling = TWOPI / (NUMBER_OF_TICKS_PER_REV * (encoder -> timer_period));
	// if running this code for the first time after reset, set velocity to zero
	if(encoder -> first_time)
	{
		encoder ->velocity = 0;
		encoder -> first_time = 0;
	}
	else
	{	// if the counter value is equal to the old value, the velocity is zero

		if(temp_counter == encoder ->last_counter_value)
		{
			encoder ->velocity = 0;
		}
		// if it is higher:
		else if(temp_counter > encoder ->last_counter_value)
		{
			// overflow occurred:
			if (temp_counter - encoder ->last_counter_value >  __HAL_TIM_GET_AUTORELOAD(encoder -> htim_encoder) / 2)
			{
				encoder ->velocity = scaling * ( -(encoder ->last_counter_value) -
						(__HAL_TIM_GET_AUTORELOAD(encoder -> htim_encoder)
								- temp_counter));
			}
			// no overflow:
			else
			{
				encoder ->velocity = scaling * (temp_counter -
						encoder ->last_counter_value);
			}

		}
		else
		{
			// no overflow
			if ((encoder ->last_counter_value - temp_counter) <  __HAL_TIM_GET_AUTORELOAD(encoder -> htim_encoder) / 2)
			{
				encoder ->velocity = scaling * (temp_counter
						- encoder ->last_counter_value);
			}
			// overflow occurred
			else
			{
				encoder ->velocity = scaling * (temp_counter +
						(__HAL_TIM_GET_AUTORELOAD(encoder -> htim_encoder) -
								encoder ->last_counter_value));
			}
		}
	}

	encoder ->position += encoder ->velocity * encoder -> timer_period;
	encoder ->last_counter_value = temp_counter;
}

void reset_encoder(encoder_inst *encoder)
{
	encoder -> position = 0;
	encoder -> first_time = 1;
	encoder -> last_counter_value = 0;
	encoder -> velocity = 0;
}
//void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim, float update_interval, uint32_t PPR)
//{
//    uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim);
//    static uint8_t first_time = 0;
//
//    if (!first_time)
//    {
//        encoder_value->velocity = 0;
//        first_time = 1;
//    }
//    else
//    {
//        if (temp_counter == encoder_value->last_counter_value)
//        {
//            encoder_value->velocity = 0;
//        }
//        else if (temp_counter > encoder_value->last_counter_value)
//        {
//            if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
//            {
//                encoder_value->velocity = -((int32_t)encoder_value->last_counter_value + (__HAL_TIM_GET_AUTORELOAD(htim) - temp_counter));
//            }
//            else
//            {
//                encoder_value->velocity = (int32_t)temp_counter - (int32_t)encoder_value->last_counter_value;
//            }
//        }
//        else
//        {
//            if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
//            {
//                encoder_value->velocity = (int32_t)temp_counter - (int32_t)encoder_value->last_counter_value;
//            }
//            else
//            {
//                encoder_value->velocity = (int32_t)temp_counter + (__HAL_TIM_GET_AUTORELOAD(htim) - (int32_t)encoder_value->last_counter_value);
//            }
//        }
//    }
//
//    encoder_value->position += encoder_value->velocity;
//    encoder_value->last_counter_value = temp_counter;
//
//    // Tính RPS (Rounds Per Second)
//    encoder_value->rps = (float)encoder_value->velocity / (PPR * update_interval);
//
//    // Nếu muốn tính RPM (Rounds Per Minute)
//    encoder_value->rpm = (float)encoder_value->velocity / PPR / update_interval * 60.0;
//}
//
//void reset_encoder(encoder_instance *encoder_value)
//{
//    encoder_value->velocity = 0;
//    encoder_value->position = 0;
//    encoder_value->last_counter_value = 0;
//    encoder_value->rps = 0;
//    encoder_value->rpm = 0;
//}
