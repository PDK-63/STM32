/*
 * iBus.h
 *
 *  Created on: Jul 18, 2024
 *      Author: Khanh
 */

#ifndef IBUS_H_
#define IBUS_H_

#include <stdbool.h>
#include <stdint.h>

/* User configuration */
#define IBUS_USER_CHANNELS		6		// Use 6 channels


/* Defines */
#define IBUS_LENGTH				0x20	// 32 bytes
#define IBUS_COMMAND40			0x40	// Command to set servo or motor speed is always 0x40
#define IBUS_MAX_CHANNLES		14


/* Main Functions */
void ibus_init();
bool ibus_read(uint16_t* ibus_data);


/* Sub Functions */
bool ibus_is_valid();
bool ibus_checksum();
void ibus_update(uint16_t* ibus_data);
void ibus_soft_failsafe(uint16_t* ibus_data, uint8_t fail_safe_max);
void ibus_reset_failsafe();

#endif /* IBUS_H_ */
