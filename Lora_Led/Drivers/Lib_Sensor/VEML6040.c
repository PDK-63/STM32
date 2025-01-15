/*
 * VEML6040.c
 *
 *  Created on: Oct 8, 2024
 *      Author: Khanh
 */

#include "main.h"
#include "stm32f1xx_hal.h"
#include <math.h>
#include "VEML6040.h"

// Biến để lưu địa chỉ I2C và cấu hình cuối cùng của cảm biến
extern I2C_HandleTypeDef hi2c1;  // Handle I2C (sử dụng I2C1 trong ví dụ này)
static uint8_t lastConfiguration = 0;

uint8_t veml6040_begin(void) {
    HAL_StatusTypeDef result;

    // Kiểm tra xem thiết bị có sẵn sàng hay không
    result = HAL_I2C_IsDeviceReady(&hi2c1, VEML6040_I2C_ADDRESS << 1, 2, HAL_MAX_DELAY);

    // Kiểm tra kết quả trả về
    if (result == HAL_OK) {
        return 1;  // Cảm biến sẵn sàng (true)
    } else {
        return 0;  // Cảm biến không sẵn sàng hoặc không kết nối (false)
    }
}

void veml6040_setConfiguration(uint8_t configuration) {
    uint8_t data[3];
    data[0] = COMMAND_CODE_CONF;    // Lệnh gửi đến cảm biến
    data[1] = configuration;        // Cấu hình
    data[2] = 0;                    // Byte 0 ở cuối (không sử dụng)

    HAL_I2C_Master_Transmit(&hi2c1, VEML6040_I2C_ADDRESS << 1, data, 3, HAL_MAX_DELAY);
    lastConfiguration = configuration;
}

static uint16_t veml6040_read(uint8_t commandCode) {
    uint8_t buffer[2];
    uint16_t data = 0;

    // Gửi lệnh đọc
    HAL_I2C_Master_Transmit(&hi2c1, VEML6040_I2C_ADDRESS << 1, &commandCode, 1, HAL_MAX_DELAY);

    // Nhận dữ liệu
    HAL_I2C_Master_Receive(&hi2c1, VEML6040_I2C_ADDRESS << 1, buffer, 2, HAL_MAX_DELAY);

    data = buffer[0];
    data |= buffer[1] << 8;

    return data;
}

uint16_t veml6040_getRed(void) {
    return veml6040_read(COMMAND_CODE_RED);
}

uint16_t veml6040_getGreen(void) {
    return veml6040_read(COMMAND_CODE_GREEN);
}

uint16_t veml6040_getBlue(void) {
    return veml6040_read(COMMAND_CODE_BLUE);
}

uint16_t veml6040_getWhite(void) {
    return veml6040_read(COMMAND_CODE_WHITE);
}

float veml6040_getAmbientLight(void) {
    uint16_t sensorValue;
    float ambientLightInLux;

    sensorValue = veml6040_getGreen();

    switch (lastConfiguration & 0x70) {
        case VEML6040_IT_40MS:
            ambientLightInLux = sensorValue * VEML6040_GSENS_40MS;
            break;
        case VEML6040_IT_80MS:
            ambientLightInLux = sensorValue * VEML6040_GSENS_80MS;
            break;
        case VEML6040_IT_160MS:
            ambientLightInLux = sensorValue * VEML6040_GSENS_160MS;
            break;
        case VEML6040_IT_320MS:
            ambientLightInLux = sensorValue * VEML6040_GSENS_320MS;
            break;
        case VEML6040_IT_640MS:
            ambientLightInLux = sensorValue * VEML6040_GSENS_640MS;
            break;
        case VEML6040_IT_1280MS:
            ambientLightInLux = sensorValue * VEML6040_GSENS_1280MS;
            break;
        default:
            ambientLightInLux = -1;
            break;
    }
    return ambientLightInLux;
}

uint16_t veml6040_getCCT(float offset) {
    uint16_t red, blue, green;
    float cct, ccti;

    red = veml6040_getRed();
    green = veml6040_getGreen();
    blue = veml6040_getBlue();

    ccti = ((float)red - (float)blue) / (float)green;
    ccti = ccti + offset;
    cct = 4278.6 * pow(ccti, -1.2455);

    return (uint16_t)cct;
}
