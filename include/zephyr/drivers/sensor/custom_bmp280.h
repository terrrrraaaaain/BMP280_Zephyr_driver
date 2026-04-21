/*
 * Copyright (c) 2026 Franciszek Trzeciak
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/sensor.h>


#ifndef CUSTOM_BMP280_H
#define CUSTOM_BMP280_H

enum bmp280_attribute
{
    BMP280_ATTR_T_STANDBY = SENSOR_ATTR_PRIV_START,
    BMP280_ATTR_FILTER,
    BMP280_ATTR_3_WIRE_SPI,
    BMP280_ATTR_RESET,
    BMP280_ATTR_MODE,

};

// Standby time (in normal mode) constatnts

#define BMP280_T_STANDBY_0_5MS (struct sensor_value){.val1 = 0, .val2 = 500}
#define BMP280_T_STANDBY_62_5MS (struct sensor_value){.val1 = 0, .val2 = 62500}
#define BMP280_T_STANDBY_125MS (struct sensor_value){.val1 = 0, .val2 = 125000}
#define BMP280_T_STANDBY_250MS (struct sensor_value){.val1 = 0, .val2 = 250000}
#define BMP280_T_STANDBY_500MS (struct sensor_value){.val1 = 0, .val2 = 500000}
#define BMP280_T_STANDBY_1000MS (struct sensor_value){.val1 = 1, .val2 = 0}
#define BMP280_T_STANDBY_2000MS (struct sensor_value){.val1 = 2, .val2 = 0}
#define BMP280_T_STANDBY_4000MS (struct sensor_value){.val1 = 4, .val2 = 0}

// IIR Filter modes constatnts

#define BMP280_FILTER_OFF (struct sensor_value){.val1 = 0, .val2 = 0}
#define BMP280_FILTER_X2 (struct sensor_value){.val1 = 2, .val2 = 0}
#define BMP280_FILTER_X4 (struct sensor_value){.val1 = 4, .val2 = 0}
#define BMP280_FILTER_X8 (struct sensor_value){.val1 = 8, .val2 = 0}
#define BMP280_FILTER_X16 (struct sensor_value){.val1 = 16, .val2 = 0}

// Oversampling constants

#define BMP280_OVERSAMPLING_OFF (struct sensor_value){.val1 = 0, .val2 = 0}
#define BMP280_OVERSAMPLING_X1 (struct sensor_value){.val1 = 1, .val2 = 0}
#define BMP280_OVERSAMPLING_X2 (struct sensor_value){.val1 = 2, .val2 = 0}
#define BMP280_OVERSAMPLING_X4 (struct sensor_value){.val1 = 4, .val2 = 0}
#define BMP280_OVERSAMPLING_X8 (struct sensor_value){.val1 = 8, .val2 = 0}
#define BMP280_OVERSAMPLING_X16 (struct sensor_value){.val1 = 16, .val2 = 0}

// 3 wire spi on/off

#define BMP280_3_WIRE_SPI_OFF (struct sensor_value) {.val1 = 0, .val2 = 0}
#define BMP280_3_WIRE_SPI_ON (struct sensor_value) {.val1 = 1, .val2 = 0}

// Mode constatnts

#define BMP280_MODE_SLEEP (struct sensor_value){.val1 = 0, .val2 = 0}
#define BMP280_MODE_FORCED (struct sensor_value){.val1 = 1, .val2 = 0}
#define BMP280_MODE_NORMAL (struct sensor_value){.val1 = 2, .val2 = 0}


#define BMP280_RESET (struct sensor_value){.val1 = 1, .val2 = 0}

#endif