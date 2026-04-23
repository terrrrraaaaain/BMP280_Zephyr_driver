/*
 * Copyright (c) 2026 Franciszek Trzeciak
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file custom_bmp280.h
 * @brief BMP280 Driver config Constants for DT 
 * @author Franciszek Trzeciak
 * @date 2026-04-22
 */

#ifndef CUSTOM_BMP280_H
#define CUSTOM_BMP280_H

/**
 * @name BMP280 Standby time between measurement in normal mode
 * @{
 */
#define BMP280_T_STANDBY_0_5MS 0
#define BMP280_T_STANDBY_62_5MS 1
#define BMP280_T_STANDBY_125MS 2
#define BMP280_T_STANDBY_250MS 3
#define BMP280_T_STANDBY_500MS 4
#define BMP280_T_STANDBY_1000MS 5
#define BMP280_T_STANDBY_2000MS 6
#define BMP280_T_STANDBY_4000MS 7
/**
 * @}
 */


/**
 * @name BMP280 IIR Filter coeficient
 * @{
 */
#define BMP280_FILTER_OFF 0
#define BMP280_FILTER_X2 1
#define BMP280_FILTER_X4 2
#define BMP280_FILTER_X8 3
#define BMP280_FILTER_X16 4
/**
 * @}
 */

// Oversampling constants
/**
 * @name BMP280 Oversampling
 * @{
 */
#define BMP280_OVERSAMPLING_OFF 0
#define BMP280_OVERSAMPLING_X1 1
#define BMP280_OVERSAMPLING_X2 2
#define BMP280_OVERSAMPLING_X4 3
#define BMP280_OVERSAMPLING_X8 4
#define BMP280_OVERSAMPLING_X16 5
/**
 * @}
 */

/**
 * @name BMP280 Enable/Disable SPI 3 wire
 * @{
 */
#define BMP280_3_WIRE_SPI_OFF 0
#define BMP280_3_WIRE_SPI_ON 1
/**
 * @}
 */

/**
 * @name BMP280 Working modes
 * @{
 */
#define BMP280_MODE_SLEEP 0
#define BMP280_MODE_FORCED 1
#define BMP280_MODE_NORMAL 2
/**
 * @}
 */

#endif