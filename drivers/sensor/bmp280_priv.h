/*
 * Copyright (c) 2026 Franciszek Trzeciak
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 *   @file
 *   @brief Memory addresss map, default values and configs
 *   for BMP280 sensor according to datasheet
 *   @author Franciszek Trzeciak
 *   @date 2026-04-16
 */

#include "zephyr/drivers/sensor/custom_bmp280.h"

#ifndef BMP280_PRIV_H
#define BNMP280_PRIV_H
/**
 * @name Register map
 * @brief BMP280 Register map
 * @{
 */

/** @brief Chip id number memory address. Should be 0x56 */
#define BMP280_ADDR_ID 0xD0

/**
 * @name Pressure memory addressses.
 * @brief Pressure memory addressses.
 * @{
 */
#define BMP280_ADDR_PRESS_MSB 0xF7
#define BMP280_ADDR_PRESS_LSB 0xF8
#define BMP280_ADDR_PRESS_XLSB 0xF9
/**
 * @}
 */

/**
 * @name Temperature memory addressses.
 * @brief Temperature memory addressses.
 * @{
 */
#define BMP280_ADDR_TEMP_MSB 0xFA
#define BMP280_ADDR_TEMP_LSB 0xFB
#define BMP280_ADDR_TEMP_XLSB 0xFC
/**
 * @}
 */

/** @brief Software BMP280 reset memory addressses. Write 0xB6 to reset sensor*/
#define BMP280_ADDR_RESET 0xE0

/** @brief Configs, status and Measurement Control memory addressses*/
#define BMP280_ADDR_CONFIG 0XF5
#define BMP280_ADDR_CTRL_MEAS 0XF4
#define BMP280_ADDR_STATUS 0XF3

/** @brief Calibrations constatnts memory address*/
#define BMP280_ADDR_CALIB00 0x88
/**
 * @}
 */

/** @brief Expected Sensor ID value*/
#define BMP280_SENSOR_ID 0x58
/** @brief  Value for soft reseting BMP280*/
#define BMP280_RESET_VAL 0xB6
/** @brief  Value read from BMP280_ADDR_PRESS_MSB or BMP280_ADDR_TEMP_MSB when measuring is off*/
#define BMP280_NO_VALUE_1 0x80

/** @brief Celssius to Kelvins converter*/
#define BMP280_CELSSIUS_TO_KELVIN_Q24_8 69926

/** @brief ln(2) in Q32.32*/
#define BMP280_LN2_Q20_20 726817
/** @brief */
#define BMP280_1_Q20_20 (1LL << 20)

/** @brief Barometric constant R/(g*M)≈8.31/(9.81*0.0289)≈29.271 m/K  */
#define BMP280_ALT_CONST_Q16_16 1918342

/**
 * @name MEASUREMENT CONTROL (CTRL_MEAS)
 * @brief measuremnt control byte predefined values, masks and setting/getting macros
 * @{
 */

// BMP280 working modes
#define BMP280_MODE_SLEEP_C 0b00
#define BMP280_MODE_FORCED_C 0b01
#define BMP280_MODE_NORMAL_C 0b11

// Temperature oversampling
#define BMP280_OVERSAMPLING_TEMPERATURE_POS 5
#define BMP280_OVERSAMPLING_TEMPERATURE_X_0 (0b000 << BMP280_OVERSAMPLING_TEMPERATURE_POS)
#define BMP280_OVERSAMPLING_TEMPERATURE_X_1 (0b001 << BMP280_OVERSAMPLING_TEMPERATURE_POS)
#define BMP280_OVERSAMPLING_TEMPERATURE_X_2 (0b010 << BMP280_OVERSAMPLING_TEMPERATURE_POS)
#define BMP280_OVERSAMPLING_TEMPERATURE_X_4 (0b011 << BMP280_OVERSAMPLING_TEMPERATURE_POS)
#define BMP280_OVERSAMPLING_TEMPERATURE_X_8 (0b100 << BMP280_OVERSAMPLING_TEMPERATURE_POS)
#define BMP280_OVERSAMPLING_TEMPERATURE_X_16 (0b101 << BMP280_OVERSAMPLING_TEMPERATURE_POS)

// Pressure oversampling
#define BMP280_OVERSAMPLING_PRESSURE_POS 2
#define BMP280_OVERSAMPLING_PRESSURE_X_0 (0b000 << BMP280_OVERSAMPLING_PRESSURE_POS)
#define BMP280_OVERSAMPLING_PRESSURE_X_1 (0b001 << BMP280_OVERSAMPLING_PRESSURE_POS)
#define BMP280_OVERSAMPLING_PRESSURE_X_2 (0b010 << BMP280_OVERSAMPLING_PRESSURE_POS)
#define BMP280_OVERSAMPLING_PRESSURE_X_4 (0b011 << BMP280_OVERSAMPLING_PRESSURE_POS)
#define BMP280_OVERSAMPLING_PRESSURE_X_8 (0b100 << BMP280_OVERSAMPLING_PRESSURE_POS)
#define BMP280_OVERSAMPLING_PRESSURE_X_16 (0b101 << BMP280_OVERSAMPLING_PRESSURE_POS)

// CTRL_MEAS Masks  - oversampling for temperature and pressure and mode
#define BMP280_MASK_CTRL_MEAS_MODE (0b11)
#define BMP280_MASK_CTRL_MEAS_OSRS_T (0b111 << BMP280_OVERSAMPLING_TEMPERATURE_POS)
#define BMP280_MASK_CTRL_MEAS_OSRS_P (0b111 << BMP280_OVERSAMPLING_PRESSURE_POS)

// CTRL_MEAS setters and getters
#define BMP280_CTRL_MEAS_SET_MODE(ctrl_meas, mode) ((uint8_t)(((ctrl_meas) & ~BMP280_MASK_CTRL_MEAS_MODE) | (mode)))
#define BMP280_CTRL_MEAS_SET_OSRS_T(ctrl_meas, osrs_t) ((uint8_t)(((ctrl_meas) & ~BMP280_MASK_CTRL_MEAS_OSRS_T) | (osrs_t)))
#define BMP280_CTRL_MEAS_SET_OSRS_P(ctrl_meas, osrs_p) ((uint8_t)(((ctrl_meas) & ~BMP280_MASK_CTRL_MEAS_OSRS_P) | (osrs_p)))

#define BMP280_CTRL_MEAS_GET_MODE(ctrl_meas) ((uint8_t)((ctrl_meas) & BMP280_MASK_CTRL_MEAS_MODE))
#define BMP280_CTRL_MEAS_GET_OSRS_T(ctrl_meas) ((uint8_t)((ctrl_meas) & BMP280_MASK_CTRL_MEAS_OSRS_T))
#define BMP280_CTRL_MEAS_GET_OSRS_P(ctrl_meas) ((uint8_t)((ctrl_meas) & BMP280_MASK_CTRL_MEAS_OSRS_P))
/**
 * @}
 *
 */

/**
 * @name CONFIG
 * @brief Config byte predefined values, masks and setting/getting macros
 * @{
 */
// Standby time beetwen measurements in normal mode
#define BMP280_CONFIG_T_STANDBY_POS 5
#define BMP280_CONFIG_T_STANDBY_0_5_MS (0b000 << BMP280_CONFIG_T_STANDBY_POS)
#define BMP280_CONFIG_T_STANDBY_62_5_MS (0b001 << BMP280_CONFIG_T_STANDBY_POS)
#define BMP280_CONFIG_T_STANDBY_125_MS (0b010 << BMP280_CONFIG_T_STANDBY_POS)
#define BMP280_CONFIG_T_STANDBY_250_MS (0b011 << BMP280_CONFIG_T_STANDBY_POS)
#define BMP280_CONFIG_T_STANDBY_500_MS (0b100 << BMP280_CONFIG_T_STANDBY_POS)
#define BMP280_CONFIG_T_STANDBY_1000_MS (0b101 << BMP280_CONFIG_T_STANDBY_POS)
#define BMP280_CONFIG_T_STANDBY_2000_MS (0b110 << BMP280_CONFIG_T_STANDBY_POS)
#define BMP280_CONFIG_T_STANDBY_4000_MS (0b111 << BMP280_CONFIG_T_STANDBY_POS)

// IIR Filter coefficients
#define BMP280_CONFIG_FILTER_POS 2
#define BMP280_CONFIG_FILTER_OFF (0b000 << BMP280_CONFIG_FILTER_POS)
#define BMP280_CONFIG_FILTER_X_2 (0b001 << BMP280_CONFIG_FILTER_POS)
#define BMP280_CONFIG_FILTER_X_4 (0b010 << BMP280_CONFIG_FILTER_POS)
#define BMP280_CONFIG_FILTER_X_8 (0b011 << BMP280_CONFIG_FILTER_POS)
#define BMP280_CONFIG_FILTER_X_16 (0b100 << BMP280_CONFIG_FILTER_POS)

// SPI 3 wire control
#define BMP280_CONFIG_SPI_3_WIRE_POS 0
#define BMP280_CONFIG_SPI_3_WIRE_ON (0b1 << BMP280_CONFIG_SPI_3_WIRE_POS)
#define BMP280_CONFIG_SPI_3_WIRE_OFF (0b0 << BMP280_CONFIG_SPI_3_WIRE_POS)

// CONFIGS Masks
#define BMP280_MASK_CONFIG_T_STANDBY (0b111 << BMP280_CONFIG_T_STANDBY_POS)
#define BMP280_MASK_CONFIG_FILTER (0b111 << BMP280_CONFIG_FILTER_POS)
#define BMP280_MASK_CONFIG_SPI_3_WIRE (0b1)
#define BMP280_MASK_CONFIG_RESERVED (0b1 << 1)

// CONFIGS getters and setters
#define BMP280_CONFIG_SET_T_STANDBY(source, mode) (uint8_t)(((source) & ~BMP280_MASK_CONFIG_T_STANDBY) | (mode))
#define BMP280_CONFIG_SET_FILTER(source, mode) (uint8_t)(((source) & ~BMP280_MASK_CONFIG_FILTER) | (mode))
#define BMP280_CONFIG_SET_SPI_3_WIRE(source, mode) (uint8_t)(((source) & ~BMP280_MASK_CONFIG_SPI_3_WIRE) | (mode))

#define BMP280_CONFIG_GET_T_STANDBY(source) ((uint8_t)((source) & BMP280_MASK_CONFIG_T_STANDBY))
#define BMP280_CONFIG_GET_FILTER(source) ((uint8_t)((source) & BMP280_MASK_CONFIG_FILTER))
#define BMP280_CONFIG_GET_SPI_3_WIRE(source) ((uint8_t)((source) & BMP280_MASK_CONFIG_SPI_3_WIRE))
/**
 * @}
 *
 */

/**
 * @name STATUS
 * @brief Status bits masks
 * @{
 */
#define BMP280_STATUS_MEASURING_POS 3
#define BMP280_STATUS_IM_UPADTE_POS 0

#define BMP280_MASK_STATUS_MEASURING (0b1 << BMP280_STATUS_MEASURING_POS)
#define BMP280_MASK_STATUS_IM_UPADTE (0b1 << BMP280_STATUS_IM_UPADTE_POS)
/**
 * @}
 *
 */

/**
 * @name SPI helper macros
 * @{
 */
#define BMP280_ON_I2C_DEF(inst) {.i2c = I2C_DT_SPEC_INST_GET(inst)}
#define BMP280_ON_SPI_DEF(inst) {.spi = SPI_DT_SPEC_INST_GET(inst, 0, 0)}
/**
 * @}
 *
 */
/**
 * @name Users constants to registers mapping
 * @brief Allows conversion between users constatnts to values writen to BMP280 registers
 * @{
 */
/**
 * @brief Structure for mapping attribute's values to registers
 * */
struct bmp280_param_reg_elem
{
    struct sensor_value val; /**< User constatnt */
    uint8_t reg;             /**< Raw data for registers*/
};

#define BMP280_MODE_CNT 3
const static struct bmp280_param_reg_elem mode_map[] = {
    {BMP280_MODE_SLEEP, BMP280_MODE_SLEEP_C},
    {BMP280_MODE_FORCED, BMP280_MODE_FORCED_C},
    {BMP280_MODE_NORMAL, BMP280_MODE_NORMAL_C},

};

#define BMP280_OVERSAMPLING_CNT 6
const static struct bmp280_param_reg_elem ovrsm_t_map[] = {
    {BMP280_OVERSAMPLING_OFF, BMP280_OVERSAMPLING_TEMPERATURE_X_0},
    {BMP280_OVERSAMPLING_X1, BMP280_OVERSAMPLING_TEMPERATURE_X_1},
    {BMP280_OVERSAMPLING_X2, BMP280_OVERSAMPLING_TEMPERATURE_X_2},
    {BMP280_OVERSAMPLING_X4, BMP280_OVERSAMPLING_TEMPERATURE_X_4},
    {BMP280_OVERSAMPLING_X8, BMP280_OVERSAMPLING_TEMPERATURE_X_8},
    {BMP280_OVERSAMPLING_X16, BMP280_OVERSAMPLING_TEMPERATURE_X_16},
};

const static struct bmp280_param_reg_elem ovrsm_p_map[] = {
    {BMP280_OVERSAMPLING_OFF, BMP280_OVERSAMPLING_PRESSURE_X_0},
    {BMP280_OVERSAMPLING_X1, BMP280_OVERSAMPLING_PRESSURE_X_1},
    {BMP280_OVERSAMPLING_X2, BMP280_OVERSAMPLING_PRESSURE_X_2},
    {BMP280_OVERSAMPLING_X4, BMP280_OVERSAMPLING_PRESSURE_X_4},
    {BMP280_OVERSAMPLING_X8, BMP280_OVERSAMPLING_PRESSURE_X_8},
    {BMP280_OVERSAMPLING_X16, BMP280_OVERSAMPLING_PRESSURE_X_16},
};

#define BMP280_T_STANDBY_CNT 8
const static struct bmp280_param_reg_elem standby_t_map[] = {
    {BMP280_T_STANDBY_0_5MS, BMP280_CONFIG_T_STANDBY_0_5_MS},
    {BMP280_T_STANDBY_62_5MS, BMP280_CONFIG_T_STANDBY_62_5_MS},
    {BMP280_T_STANDBY_125MS, BMP280_CONFIG_T_STANDBY_125_MS},
    {BMP280_T_STANDBY_250MS, BMP280_CONFIG_T_STANDBY_250_MS},
    {BMP280_T_STANDBY_500MS, BMP280_CONFIG_T_STANDBY_500_MS},
    {BMP280_T_STANDBY_1000MS, BMP280_CONFIG_T_STANDBY_1000_MS},
    {BMP280_T_STANDBY_2000MS, BMP280_CONFIG_T_STANDBY_2000_MS},
    {BMP280_T_STANDBY_4000MS, BMP280_CONFIG_T_STANDBY_4000_MS},

};

#define BMP280_FILTER_CNT 5
const static struct bmp280_param_reg_elem filter_map[] = {
    {BMP280_FILTER_OFF, BMP280_CONFIG_FILTER_OFF},
    {BMP280_FILTER_X2, BMP280_CONFIG_FILTER_X_2},
    {BMP280_FILTER_X4, BMP280_CONFIG_FILTER_X_4},
    {BMP280_FILTER_X8, BMP280_CONFIG_FILTER_X_8},
    {BMP280_FILTER_X16, BMP280_CONFIG_FILTER_X_16},
};
/**
 * @}
 *
 */

#define BMP280_DEFAULT_MODE_IDX 0
#define BMP280_DEFAULT_T_STBY_IDX 0
#define BMP280_DEFAULT_FILTER_IDX 0
#define BMP280_DEFAULT_OVRSMPL_IDX 1
#define BMP280_DEFAULT_SPI_3_WIRE_IDX 0

#endif