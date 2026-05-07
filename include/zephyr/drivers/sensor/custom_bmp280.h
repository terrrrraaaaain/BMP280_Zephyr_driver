/*
 * Copyright (c) 2026 Franciszek Trzeciak
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file custom_bmp280.h
 * @brief BMP280 Driver config Constants for runtime use
 * @author Franciszek Trzeciak
 * @date 2026-04-20
 */
#include <zephyr/drivers/sensor.h>

#ifndef CUSTOM_BMP280_H
#define CUSTOM_BMP280_H
/**
 * @brief BMP280 specific attributes
 *
 */
enum bmp280_attribute
{
    BMP280_ATTR_T_STANDBY = SENSOR_ATTR_PRIV_START,
    BMP280_ATTR_FILTER,
    BMP280_ATTR_3_WIRE_SPI,
    BMP280_ATTR_RESET,
    BMP280_ATTR_MODE,
    BMP280_ATTR_PRESS_SEA_LEVEL
};

/**
 * @name BMP280 Standby time between measurement in normal mode
 * @{
 */
#define BMP280_T_STANDBY_0_5MS (struct sensor_value){.val1 = 0, .val2 = 500}
#define BMP280_T_STANDBY_62_5MS (struct sensor_value){.val1 = 0, .val2 = 62500}
#define BMP280_T_STANDBY_125MS (struct sensor_value){.val1 = 0, .val2 = 125000}
#define BMP280_T_STANDBY_250MS (struct sensor_value){.val1 = 0, .val2 = 250000}
#define BMP280_T_STANDBY_500MS (struct sensor_value){.val1 = 0, .val2 = 500000}
#define BMP280_T_STANDBY_1000MS (struct sensor_value){.val1 = 1, .val2 = 0}
#define BMP280_T_STANDBY_2000MS (struct sensor_value){.val1 = 2, .val2 = 0}
#define BMP280_T_STANDBY_4000MS (struct sensor_value){.val1 = 4, .val2 = 0}
/**
 * @}
 */

/**
 * @name BMP280 IIR Filter coefficient
 * @{
 */
#define BMP280_FILTER_OFF (struct sensor_value){.val1 = 0, .val2 = 0}
#define BMP280_FILTER_X2 (struct sensor_value){.val1 = 2, .val2 = 0}
#define BMP280_FILTER_X4 (struct sensor_value){.val1 = 4, .val2 = 0}
#define BMP280_FILTER_X8 (struct sensor_value){.val1 = 8, .val2 = 0}
#define BMP280_FILTER_X16 (struct sensor_value){.val1 = 16, .val2 = 0}
/**
 * @}
 */

/**
 * @name BMP280 Oversampling
 * @{
 */
#define BMP280_OVERSAMPLING_OFF (struct sensor_value){.val1 = 0, .val2 = 0}
#define BMP280_OVERSAMPLING_X1 (struct sensor_value){.val1 = 1, .val2 = 0}
#define BMP280_OVERSAMPLING_X2 (struct sensor_value){.val1 = 2, .val2 = 0}
#define BMP280_OVERSAMPLING_X4 (struct sensor_value){.val1 = 4, .val2 = 0}
#define BMP280_OVERSAMPLING_X8 (struct sensor_value){.val1 = 8, .val2 = 0}
#define BMP280_OVERSAMPLING_X16 (struct sensor_value){.val1 = 16, .val2 = 0}
/**
 * @}
 */

/**
 * @name BMP280 Enable/Disable SPI 3 wire
 * @{
 */
#define BMP280_3_WIRE_SPI_OFF (struct sensor_value){.val1 = 0, .val2 = 0}
#define BMP280_3_WIRE_SPI_ON (struct sensor_value){.val1 = 1, .val2 = 0}
/**
 * @}
 */

/**
 * @name BMP280 Working modes
 * @{
 */
#define BMP280_MODE_SLEEP (struct sensor_value){.val1 = 0, .val2 = 0}
#define BMP280_MODE_FORCED (struct sensor_value){.val1 = 1, .val2 = 0}
#define BMP280_MODE_NORMAL (struct sensor_value){.val1 = 2, .val2 = 0}
/**
 * @}
 */
/**
 * @name BMP280 Reset val
 * @{
 */
#define BMP280_RESET_TO_DEFAULT (struct sensor_value){.val1 = 1, .val2 = 0}
#define BMP280_RESET_TO_SLEEP (struct sensor_value){.val1 = 2, .val2 = 0}
/**
 * @}
 */

/**
 * @name BMP280 standard sea level pressure = 101.325 kPa
 * @{
 */
#define BMP280_DEFAULT_SEA_LEVEL_PRESSURE (struct sensor_value){.val1 = 101, .val2 = 325000}
/**
 * @}
 */

#endif