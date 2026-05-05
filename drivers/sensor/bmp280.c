/*
 * Copyright (c) 2026 Franciszek Trzeciak
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * This software includes compensation algorithms adapted from the
 * Bosch Sensortec BMP280 datasheet.
 * Original algorithmic logic: Copyright (c) Bosch Sensortec GmbH.
 */

/**
 *   @file bmp280.c
 *   @brief Implemenatation of the BMP280 sensor driver for Zepyhr RTOS
 * 		This driver inculdes I2C communication, runtime sensor configuration
 * 		and compensation methods described in datasheet
 *   @date 2026-04-16
 */

#define DT_DRV_COMPAT custom_bmp280

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include "bmp280_priv.h"

/*
 *	Definitions
 */

/**
 * @brief Temperature calibration coefficients
 *
 */
struct bmp280_calibTemperatureData
{
	uint16_t dT1;
	int16_t dT2;
	int16_t dT3;
};

/**
 * @brief Pressure calibration coefficients
 *
 */
struct bmp280_calibPressureData
{
	uint16_t dP1;
	int16_t dP2;
	int16_t dP3;
	int16_t dP4;
	int16_t dP5;
	int16_t dP6;
	int16_t dP7;
	int16_t dP8;
	int16_t dP9;
};

/**
 * @brief Calibration data for temperature and pressure, temperature cooefficient for pressure compensation, and flag whether temperature cooeficient has been updated on new data
 *
 */
struct bmp280_calibData
{
	struct bmp280_calibTemperatureData tCalib;
	struct bmp280_calibPressureData pCalib;
	int32_t t; /** for pressure compensation from temperature compensation*/
	int32_t p; /** for altidut computation from pressure compensation. 24.8 format*/
	bool t_cooef_cmpt;
	bool p_cooef_cmpt;
};

/**
 * @brief Set of Bus specific IO methods
 *
 */
struct bmp280_ioMethods
{
	bool (*isBusReady)(const struct device *dev);
	int (*readByte)(const struct device *dev, uint8_t addr, uint8_t *buffer);
	int (*readBurst)(const struct device *dev, uint8_t addr, uint8_t *buffer, size_t size);
	int (*writeByte)(const struct device *dev, uint8_t addr, uint8_t buffer);
	int (*writeBurst)(const struct device *dev, uint8_t addr, const uint8_t *buffer, size_t size);
	bool (*busType)(void); // 1 for I2C, 0 for SPI
};

/**
 * @brief IO abstraction layer and bus structure
 *
 */
struct bmp280_ioAPI
{
	struct bmp280_ioMethods methods;
	/**
	 * @brief bus union for IO abstraction layer
	 */
	union bmp280_bus
	{
		struct i2c_dt_spec i2c;
		struct spi_dt_spec spi;
	} bus;
};

/**
 * @brief Device Tree initial data
 *
 */
struct bmp280_DT_params
{
	uint8_t mode;
	uint8_t t_stby;
	uint8_t iir_filter;
	uint8_t ovrsmplT;
	uint8_t ovrsmplP;
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	bool spi3wire;
#endif
};

/**
 * @brief Main driver configuration structure
 *
 */
struct bmp280_config
{
	struct bmp280_ioAPI ioAPI;
	struct bmp280_DT_params DTparams;
};

/**
 * @brief Main driver configuration structure
 *
 */
struct bmp280_data
{
	uint8_t press_raw[3];
	uint8_t temp_raw[3];
	uint32_t pressSL;
	uint8_t ctrl_meas;
	struct bmp280_calibData calib;
};

/*
 *	Helper function declaraction
 */

/** @brief  Check if BMP280 everything is copied (Im bit) */
static bool bmp280_isImReady(const struct device *dev);

/** @brief Check if BMP280 is currently measuring */
static bool bmp280_isMeasuring(const struct device *dev);

/** @brief  Check if chipID is valid */
static bool bmp280_chipID_OK(const struct device *dev);

/** @brief Read raw pressure bytes from sensor */
static int bmp280_getPressureRaw(const struct device *dev);

/** @brief  Read raw temperature bytes from sensor*/
static int bmp280_getTemperatureRaw(const struct device *dev);

/** @brief  Get Control Measuremnt byte and store to bmp280_data.ctrl_meas */
static int bmp280_getCtrlMeas(const struct device *dev);

/** @brief  Set Control Measuremnt byte and using bmp280_data.ctrl_meas */
static int bmp280_setCtrlMeas(const struct device *dev);

/** @brief  Get Configuration byte */
static int bmp280_getConfig(const struct device *dev, uint8_t *configs);

/** @brief  Set Configuration byte */
static int bmp280_setConfig(const struct device *dev, uint8_t t_stby, uint8_t mask);

/** @brief Compensation and conversion function adapted from Bosh BMP280 datasheet for temperature*/
static int calibTemp(const struct device *dev, struct sensor_value *temperature);

/** @brief Compensation and conversion function adapted from Bosh BMP280 datasheet for pressure*/
static int calibPress24_8(const struct device *dev, uint32_t *pressure);

/** @brief Converts pressure from Q24.8 format to decimal */
static int calibPress(const struct device *dev, struct sensor_value *pressure);

/** @brief Computes altitude AML acrdoing to sea level pressure set via atribute*/
static int computeAltitude(const struct device *dev, struct sensor_value *alt);

/** @brief Waiting time estimation for new data in force mode */
static uint16_t bmp280_timeToRead_ms(const struct device *dev);

/** @brief Software reset of sensor with DT parameters initialization*/
static int bmp280_softResetToDefault(const struct device *dev);
/** @brief Software reset of sensor */
static int bmp280_softResetToSleep(const struct device *dev);

/** @brief Compares to sensor value structures */
static bool sensor_value_equal(const struct sensor_value *v1, const struct sensor_value *v2);
/** @brief Compares sensor value structure to val1 and val2 provided as integers */
static bool int_sensor_value_equal(const struct sensor_value *v1, const int val1, const int val2);

/** @brief attribute(struct sensor_value) to bmp280 regs converter (uses mappings from custom_bmp280_priv.h)
 * @see drivers/sensor/bmp280_priv.h for mappings
 * */
static inline int bmp280_attrValToReg(const struct bmp280_param_reg_elem *map, size_t s, const struct sensor_value *attr, uint8_t *reg);

/** @brief  bmp280 regs to attribute(struct sensor_value) converter (uses mappings from custom_bmp280_priv.h)
 * @see drivers/sensor/bmp280_priv.h for mappings
 * */
static inline int bmp280_regToAttrVal(const struct bmp280_param_reg_elem *map, size_t s, struct sensor_value *attr, const uint8_t reg);

/*
 *	Implementations of main API functions
 */

/**
 * @brief Initialize BMP280 driver
 * @note Default settings are fetched from the Devicetree node.
 * Refer to dts/bindings/sensor/bosch,bmp280-custom.yaml for valid property values.
 * @param dev Pointer to device structure
 * @return 0 on success, negative errno code from I2C/SPI bus driver on communication failure
 * @retval -ENXIO if device chipID mismatch (expected 0x58) or bus is not working
 */
static int bmp280_init_bare(const struct device *dev)
{
	const struct bmp280_config *conf = dev->config;
	struct bmp280_data *data = dev->data;

	if (!conf->ioAPI.methods.isBusReady(dev))
	{
		return -ENXIO;
	}
	if (!bmp280_chipID_OK(dev))
	{
		return -ENXIO;
	}

	int c = 0;
	while (!bmp280_isImReady(dev))
	{
		if (c > 10)
		{
			return -EIO;
		}
		k_msleep(1);
		c++;
	}

	uint8_t calBuf[26];
	c = conf->ioAPI.methods.readBurst(dev, BMP280_ADDR_CALIB00, calBuf, 26);
	data->calib.tCalib.dT1 = (calBuf[1] << 8) | calBuf[0];
	data->calib.tCalib.dT2 = (calBuf[3] << 8) | calBuf[2];
	data->calib.tCalib.dT3 = (calBuf[5] << 8) | calBuf[4];
	data->calib.pCalib.dP1 = (calBuf[7] << 8) | calBuf[6];
	data->calib.pCalib.dP2 = (calBuf[9] << 8) | calBuf[8];
	data->calib.pCalib.dP3 = (calBuf[11] << 8) | calBuf[10];
	data->calib.pCalib.dP4 = (calBuf[13] << 8) | calBuf[14];
	data->calib.pCalib.dP5 = (calBuf[15] << 8) | calBuf[16];
	data->calib.pCalib.dP6 = (calBuf[17] << 8) | calBuf[18];
	data->calib.pCalib.dP7 = (calBuf[19] << 8) | calBuf[20];
	data->calib.pCalib.dP8 = (calBuf[21] << 8) | calBuf[22];
	data->calib.pCalib.dP9 = (calBuf[23] << 8) | calBuf[24];

	data->calib.t_cooef_cmpt = 0; // reset temp calibration info readiness flag for pressure calibration
	data->calib.p_cooef_cmpt = 0; // reset pressure calibration info readiness flag for altitude calibration

	bmp280_getCtrlMeas(dev);
	return 0;
}

/**
 * @brief Initialize BMP280 driver with defualt DT configuration
 * @note Default settings are fetched from the Devicetree node.
 * Refer to dts/bindings/sensor/bosch,bmp280-custom.yaml for valid property values.
 * @param dev Pointer to device structure
 * @return 0 on success, negative errno code from I2C/SPI bus driver on communication failure
 * @retval -ENXIO if device chipID mismatch (expected 0x58) or bus is not working
 * @retval -EINVAL if invalid configuration in device tree
 */
static int bmp280_init_full(const struct device *dev)
{
	const struct bmp280_config *conf = dev->config;
	struct bmp280_data *data = dev->data;
	int c = 0;

	c = bmp280_init_bare(dev);
	if (c)
		return c;

	// DT configuration
	if (conf->DTparams.mode >= 0 && conf->DTparams.mode <= BMP280_MODE_CNT)
		data->ctrl_meas = BMP280_CTRL_MEAS_SET_MODE(data->ctrl_meas, mode_map[conf->DTparams.mode].reg);
	else
		return -EINVAL;

	if (conf->DTparams.ovrsmplT >= 0 && conf->DTparams.mode <= BMP280_OVERSAMPLING_CNT)
		data->ctrl_meas = BMP280_CTRL_MEAS_SET_OSRS_T(data->ctrl_meas, ovrsm_t_map[conf->DTparams.ovrsmplT].reg);
	else
		return -EINVAL;

	if (conf->DTparams.ovrsmplP >= 0 && conf->DTparams.mode <= BMP280_OVERSAMPLING_CNT)
		data->ctrl_meas = BMP280_CTRL_MEAS_SET_OSRS_P(data->ctrl_meas, ovrsm_p_map[conf->DTparams.ovrsmplP].reg);
	else
		return -EINVAL;

	uint8_t config = 0;
	if (conf->DTparams.t_stby >= 0 && conf->DTparams.t_stby <= BMP280_T_STANDBY_CNT)
		config = BMP280_CONFIG_SET_T_STANDBY(config, standby_t_map[conf->DTparams.t_stby].reg);
	else
		return -EINVAL;

	if (conf->DTparams.iir_filter >= 0 && conf->DTparams.iir_filter <= BMP280_FILTER_CNT)
		config = BMP280_CONFIG_SET_FILTER(config, filter_map[conf->DTparams.iir_filter].reg);
	else
		return -EINVAL;
	if (conf->DTparams.iir_filter >= 0 && conf->DTparams.iir_filter <= BMP280_FILTER_CNT)
		config = BMP280_CONFIG_SET_FILTER(config, filter_map[conf->DTparams.iir_filter].reg);
	else
		return -EINVAL;

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	if (!conf->ioAPI.methods.busType()) // check if we use SPI
		if (conf->DTparams.spi3wire = 1)
			config = BMP280_CONFIG_SET_SPI_3_WIRE(config, BMP280_CONFIG_SPI_3_WIRE_ON);
		else if (conf->DTparams.spi3wire = 0)
			config = BMP280_CONFIG_SET_SPI_3_WIRE(config, BMP280_CONFIG_SPI_3_WIRE_OFF);
		else
			return -EINVAL;
#endif

	c = bmp280_setConfig(dev, config, BMP280_MASK_CONFIG_FILTER | BMP280_MASK_CONFIG_T_STANDBY | BMP280_MASK_CONFIG_SPI_3_WIRE);
	if (c)
		return c;

	c = bmp280_setCtrlMeas(dev);
	if (c)
		return c;

	bmp280_getCtrlMeas(dev);
	bmp280_getConfig(dev, &config);
	return 0;
}

/**
 * @brief Fetch ADC data from sensor
 *
 * @param dev Pointer to device structure
 * @param channel Sensor channel to fetch data from (supported SENSOR_CHAN_AMBIENT_TEMP, SENSOR_CHAN_PRESS,SENSOR_CHAN_ALL)
 * @return 0 on success, negative errno code from I2C/SPI bus driver on communication failure
 * @retval -ENXIO if device chipID mismatch (expected 0x58)
 * @retval -ETIMEDOUT timed out waiting for data ready
 * @retval -ENODATA if read 0x80 from registers
 * @retval -ENOTSUP if channel is not supported
 */
static int bmp280_sample_fetch(const struct device *dev, enum sensor_channel channel)
{
	struct bmp280_data *data = dev->data;
	uint8_t cycles = 0;

	if (!bmp280_chipID_OK(dev))
	{
		return -ENXIO;
	}
	bmp280_getCtrlMeas(dev);
	if (BMP280_CTRL_MEAS_GET_MODE(data->ctrl_meas) == BMP280_MODE_SLEEP_C)
	{

		data->ctrl_meas = BMP280_CTRL_MEAS_SET_MODE(data->ctrl_meas, BMP280_MODE_FORCED_C); // assuming that the aim of caling fetch in sleep mode is to triger a single measure
		bmp280_setCtrlMeas(dev);															// start measuring in forced mode (if in sleep wake up and force measurement)

		k_msleep(bmp280_timeToRead_ms(dev)); // wait for result based on configuration
		while (bmp280_isMeasuring(dev) || !bmp280_isImReady(dev))
		{
			if (cycles >= 500)
				return -ETIMEDOUT;
			k_usleep(100);
			cycles = cycles + 1;
		}
	}
	else
	{
		if (BMP280_CTRL_MEAS_GET_MODE(data->ctrl_meas) == BMP280_MODE_FORCED_C) // already started measurment
		{
			while (bmp280_isMeasuring(dev) || !bmp280_isImReady(dev))
			{
				if (cycles >= 400)
					return -ETIMEDOUT;
				k_usleep(200);
				cycles = cycles + 1;
			}
		}
	}
	int c = bmp280_getCtrlMeas(dev);
	data->calib.t_cooef_cmpt = 0;
	data->calib.p_cooef_cmpt = 0;

	switch (channel)
	{
	case SENSOR_CHAN_AMBIENT_TEMP:
		return bmp280_getTemperatureRaw(dev);
		break;
	case SENSOR_CHAN_PRESS:
		c = bmp280_getTemperatureRaw(dev);
		if (c == 0)
			return bmp280_getPressureRaw(dev);
		return c;
		break;
	case SENSOR_CHAN_ALL:
		int c = bmp280_getTemperatureRaw(dev);
		if (c == 0)
			return bmp280_getPressureRaw(dev);
		return c;
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

/**
 * @brief Convert raw ADC data to human readable
 *
 * @param dev Pointer to device structure
 * @param channel Sensor channel to convert data from (supported SENSOR_CHAN_AMBIENT_TEMP, SENSOR_CHAN_PRESS)
 * @param reading Pointer to sencsor_value structure where store data to
 * @return 0 on success
 * @retval -ENOTSUP if channel is not supported
 * @retval -EINVAL if division by zero occurse during pressure conversion
 */
static int bmp280_channel_get(const struct device *dev, enum sensor_channel channel, struct sensor_value *reading)
{
	switch (channel)
	{
	case SENSOR_CHAN_AMBIENT_TEMP:
		calibTemp(dev, reading);
		return 0;
	case SENSOR_CHAN_PRESS:
		if (!((struct bmp280_data *)(dev->data))->calib.t_cooef_cmpt)
		{
			struct sensor_value dummy;
			calibTemp(dev, &dummy);
		}
		return calibPress(dev, reading);
	default:
		return -ENOTSUP;
	}
	return 0;
}

/**
 * @brief Set sensor attribute or configuration
 *
 * @param dev Pointer to device structure
 * @param channel Channel for which attributes are set (supported SENSOR_CHAN_AMBIENT_TEMP, SENSOR_CHAN_PRESS,SENSOR_CHAN_ALL)
 * @param attr Attribute to be set (supported SENSOR_ATTR_OVERSAMPLING (only in sleep mode), BMP280_ATTR_T_STANDBY,
 * 				BMP280_ATTR_FILTER,BMP280_ATTR_3_WIRE_SPI (only for BMP280 on SPI bus),BMP280_ATTR_RESET,BMP280_ATTR_MODE (only in sleep mode))
 * @param val	Pointer to sensor_value structure where the value set channel's attribute to
 * @see include/zephyr/drivers/sensor/custom_bmp280.h for predefined values and custom attributes
 * @return 0 on success, negative errno code from I2C/SPI bus driver on communication failure
 * @retval -ENOTSUP if channel, attribute or attribute value is not supported at all or in current mode
 */
static int bmp280_attr_set(const struct device *dev, enum sensor_channel channel, enum sensor_attribute attr, const struct sensor_value *val)
{
	const struct bmp280_config *conf = dev->config;
	struct bmp280_data *data = dev->data;
	uint8_t reg;
	int ret = 0;
	if (BMP280_CTRL_MEAS_GET_MODE(data->ctrl_meas) == BMP280_MODE_NORMAL_C)
	{
		if ((int)attr == SENSOR_ATTR_OVERSAMPLING || (int)attr == BMP280_ATTR_MODE)
		{
			return -ENOTSUP;
		}
	}
	switch ((int)attr)
	{
	case SENSOR_ATTR_OVERSAMPLING:
		switch (channel)
		{
		case SENSOR_CHAN_AMBIENT_TEMP:
			ret = bmp280_attrValToReg(ovrsm_t_map, BMP280_OVERSAMPLING_CNT, val, &reg);
			if (ret)
				return ret;
			data->ctrl_meas = BMP280_CTRL_MEAS_SET_OSRS_T(data->ctrl_meas, reg);
			break;
		case SENSOR_CHAN_PRESS:
			ret = bmp280_attrValToReg(ovrsm_p_map, BMP280_OVERSAMPLING_CNT, val, &reg);
			if (ret)
				return ret;
			data->ctrl_meas = BMP280_CTRL_MEAS_SET_OSRS_P(data->ctrl_meas, reg);
			break;
		case SENSOR_CHAN_ALL:
			ret = bmp280_attrValToReg(ovrsm_p_map, BMP280_OVERSAMPLING_CNT, val, &reg);
			if (ret)
				return ret;
			data->ctrl_meas = BMP280_CTRL_MEAS_SET_OSRS_P(data->ctrl_meas, reg);

			ret = bmp280_attrValToReg(ovrsm_t_map, BMP280_OVERSAMPLING_CNT, val, &reg);
			if (ret)
				return ret;
			data->ctrl_meas = BMP280_CTRL_MEAS_SET_OSRS_T(data->ctrl_meas, reg);
			break;
		default:
			return -ENOTSUP;
		}
		return bmp280_setCtrlMeas(dev);

	case BMP280_ATTR_T_STANDBY:
		if (channel == SENSOR_CHAN_ALL || channel == SENSOR_CHAN_AMBIENT_TEMP || channel == SENSOR_CHAN_PRESS)
		{
			ret = bmp280_attrValToReg(standby_t_map, BMP280_T_STANDBY_CNT, val, &reg);
			if (ret)
				return ret;

			return bmp280_setConfig(dev, reg, BMP280_MASK_CONFIG_T_STANDBY);
		}
		else
			return -ENOTSUP;

	case BMP280_ATTR_FILTER:
		if (channel == SENSOR_CHAN_ALL || channel == SENSOR_CHAN_AMBIENT_TEMP || channel == SENSOR_CHAN_PRESS)
		{
			ret = bmp280_attrValToReg(filter_map, BMP280_FILTER_CNT, val, &reg);
			if (ret)
				return ret;

			return bmp280_setConfig(dev, reg, BMP280_MASK_CONFIG_FILTER);
		}
		else
			return -ENOTSUP;
	case BMP280_ATTR_3_WIRE_SPI:
		if ((channel == SENSOR_CHAN_ALL || channel == SENSOR_CHAN_AMBIENT_TEMP || channel == SENSOR_CHAN_PRESS) && !conf->ioAPI.methods.busType())
		{

			if (int_sensor_value_equal(val, 1, 0))
			{
				reg = BMP280_CONFIG_SPI_3_WIRE_ON;
			}
			else
			{
				reg = BMP280_CONFIG_SPI_3_WIRE_OFF;
			}
			return bmp280_setConfig(dev, reg, BMP280_MASK_CONFIG_SPI_3_WIRE);
		}
		else
			return -ENOTSUP;
	case BMP280_ATTR_RESET:
		if (channel == SENSOR_CHAN_ALL || channel == SENSOR_CHAN_AMBIENT_TEMP || channel == SENSOR_CHAN_PRESS)
		{

			if (int_sensor_value_equal(val, 1, 0))
			{
				return bmp280_softResetToDefault(dev);
			}
			else if (int_sensor_value_equal(val, 2, 0))
			{
				return bmp280_softResetToSleep(dev);
			}
			else
				return -EINVAL;
		}
		else
			return -ENOTSUP;

	case BMP280_ATTR_MODE:
		if (channel == SENSOR_CHAN_ALL || channel == SENSOR_CHAN_AMBIENT_TEMP || channel == SENSOR_CHAN_PRESS)
		{
			ret = bmp280_attrValToReg(mode_map, BMP280_MODE_CNT, val, &reg);
			if (ret)
				return ret;
			data->ctrl_meas = BMP280_CTRL_MEAS_SET_MODE(data->ctrl_meas, reg);
			return bmp280_setCtrlMeas(dev);
		}
		else
			return -ENOTSUP;
	default:
		return -ENOTSUP;
	}
	return 0;
}
/**
 * @brief Get sensor attribute or current configuration
 *
 * @param dev Pointer to device structure
 * @param channel Channel for which attribute is read (supported SENSOR_CHAN_AMBIENT_TEMP, SENSOR_CHAN_PRESS,SENSOR_CHAN_ALL)
 * @param attr Attribute to be retrieved (supported SENSOR_ATTR_OVERSAMPLING, BMP280_ATTR_T_STANDBY,
 * 				BMP280_ATTR_FILTER,BMP280_ATTR_3_WIRE_SPI (only for BMP280 on SPI bus),BMP280_ATTR_RESET,BMP280_ATTR_MODE)
 * @param val	Pointer to sensor_value structure where the current value will be stored
 * @see include/zephyr/drivers/sensor/custom_bmp280.h for predefined values and custom attributes
 * @return 0 on success, negative errno code from I2C/SPI bus driver on communication failure
 * @retval -ENOTSUP if channel, attribute or attribute value is not supported
 */
static int bmp280_attr_get(const struct device *dev, enum sensor_channel channel, enum sensor_attribute attr, struct sensor_value *val)
{
	const struct bmp280_config *config = dev->config;
	struct bmp280_data *data = dev->data;
	int c = 0;
	switch ((int)attr)
	{
	case SENSOR_ATTR_OVERSAMPLING:
		c = bmp280_getCtrlMeas(dev);
		if (c)
			return c;
		switch (channel)
		{
		case SENSOR_CHAN_AMBIENT_TEMP:
			return bmp280_regToAttrVal(ovrsm_t_map, BMP280_OVERSAMPLING_CNT, val, BMP280_CTRL_MEAS_GET_OSRS_T(data->ctrl_meas));
		case SENSOR_CHAN_PRESS:
			return bmp280_regToAttrVal(ovrsm_p_map, BMP280_OVERSAMPLING_CNT, val, BMP280_CTRL_MEAS_GET_OSRS_P(data->ctrl_meas));
		default:
			return -ENOTSUP;
		}

	case BMP280_ATTR_MODE:
		if (channel == SENSOR_CHAN_ALL || channel == SENSOR_CHAN_AMBIENT_TEMP || channel == SENSOR_CHAN_PRESS)
		{
			c = bmp280_getCtrlMeas(dev);
			if (c)
				return c;

			return bmp280_regToAttrVal(mode_map, BMP280_MODE_CNT, val, BMP280_CTRL_MEAS_GET_MODE(data->ctrl_meas));
		}
		else
			return -ENOTSUP;
	case BMP280_ATTR_FILTER:
		if (channel == SENSOR_CHAN_ALL || channel == SENSOR_CHAN_AMBIENT_TEMP || channel == SENSOR_CHAN_PRESS)
		{
			uint8_t configs;
			c = bmp280_getConfig(dev, &configs);
			if (c)
				return c;

			return bmp280_regToAttrVal(filter_map, BMP280_FILTER_CNT, val, BMP280_CONFIG_GET_FILTER(configs));
		}
		else
			return -ENOTSUP;
	case BMP280_ATTR_T_STANDBY:
		if (channel == SENSOR_CHAN_ALL || channel == SENSOR_CHAN_AMBIENT_TEMP || channel == SENSOR_CHAN_PRESS)
		{
			uint8_t configs;
			c = bmp280_getConfig(dev, &configs);
			if (c)
				return c;

			return bmp280_regToAttrVal(standby_t_map, BMP280_T_STANDBY_CNT, val, BMP280_CONFIG_GET_T_STANDBY(configs));
		}
		else
			return -ENOTSUP;
	case BMP280_ATTR_3_WIRE_SPI:
		if ((channel == SENSOR_CHAN_ALL || channel == SENSOR_CHAN_AMBIENT_TEMP || channel == SENSOR_CHAN_PRESS) && !config->ioAPI.methods.busType())
		{
			uint8_t configs;
			c = bmp280_getConfig(dev, &configs);
			if (c)
				return c;
			if (BMP280_CONFIG_GET_SPI_3_WIRE(configs) == BMP280_CONFIG_SPI_3_WIRE_ON)
			{
				*val = BMP280_3_WIRE_SPI_ON;
			}
			else
			{
				*val = BMP280_3_WIRE_SPI_OFF;
			}
			return 0;
		}
		else
			return -ENOTSUP;
	default:
		break;
	}
	return -ENOTSUP;
}

/*
 *	Implementations of helper functions
 */

static int bmp280_setCtrlMeas(const struct device *dev)
{
	const struct bmp280_config *conf = dev->config;
	const struct bmp280_data *data = dev->data;

	return conf->ioAPI.methods.writeByte(dev, BMP280_ADDR_CTRL_MEAS, data->ctrl_meas);
}

static int bmp280_getCtrlMeas(const struct device *dev)
{
	const struct bmp280_config *conf = dev->config;
	struct bmp280_data *data = dev->data;
	return conf->ioAPI.methods.readByte(dev, BMP280_ADDR_CTRL_MEAS, &data->ctrl_meas);
}

static int bmp280_setConfig(const struct device *dev, uint8_t configs, uint8_t mask)
{
	const struct bmp280_config *conf = dev->config;
	uint8_t configs_c;
	int c = bmp280_getConfig(dev, &configs_c);
	if (c)
		return c;
	configs = (configs_c & (BMP280_MASK_CONFIG_RESERVED | ~mask)) | (configs & mask);
	return conf->ioAPI.methods.writeByte(dev, BMP280_ADDR_CONFIG, configs);
}

static int bmp280_getConfig(const struct device *dev, uint8_t *configs)
{
	const struct bmp280_config *conf = dev->config;
	return conf->ioAPI.methods.readByte(dev, BMP280_ADDR_CONFIG, configs);
}

static int bmp280_getPressureRaw(const struct device *dev)
{

	const struct bmp280_config *conf = dev->config;
	struct bmp280_data *data = dev->data;

	int c = conf->ioAPI.methods.readBurst(dev, BMP280_ADDR_PRESS_MSB, data->press_raw, 3);

	if (c == 0)
	{
		if (data->press_raw[0] == BMP280_NO_VALUE_1 && BMP280_CTRL_MEAS_GET_OSRS_P(data->ctrl_meas) != BMP280_OVERSAMPLING_PRESSURE_X_0)
			return -ENODATA;
		else
			return 0;
	}
	return c;
}

static int bmp280_getTemperatureRaw(const struct device *dev)
{
	const struct bmp280_config *conf = dev->config;
	struct bmp280_data *data = dev->data;

	int c = conf->ioAPI.methods.readBurst(dev, BMP280_ADDR_TEMP_MSB, data->temp_raw, 3);

	if (c == 0)
	{
		if (data->temp_raw[0] == BMP280_NO_VALUE_1 && BMP280_CTRL_MEAS_GET_OSRS_T(data->ctrl_meas) != BMP280_OVERSAMPLING_TEMPERATURE_X_0)
			return -ENODATA;
		else
			return 0;
	}
	return c;
}

// Adapted from Bosh BMP280 datasheet
static int calibTemp(const struct device *dev, struct sensor_value *temperature)
{
	struct bmp280_data *data = dev->data;

	if (!data->calib.t_cooef_cmpt)
	{

		int32_t tRaw = data->temp_raw[0] << 12 | data->temp_raw[1] << 4 | data->temp_raw[2] >> 4;

		int32_t v1 = ((((tRaw >> 3) - ((int32_t)data->calib.tCalib.dT1 << 1))) *
					  (int32_t)data->calib.tCalib.dT1) >>
					 11;
		int32_t v2 = (((((tRaw >> 4) - ((int32_t)data->calib.tCalib.dT1)) *
						((tRaw >> 4) - ((int32_t)data->calib.tCalib.dT1))) >>
					   12) *
					  ((int32_t)data->calib.tCalib.dT3)) >>
					 14;
		data->calib.t = v1 + v2;
		data->calib.t_cooef_cmpt = 1;
	}
	uint32_t tempT = (data->calib.t * 5 + 128) >> 8;
	temperature->val1 = tempT / 100;
	temperature->val2 = (tempT) % 100 * 1000;

	return 0;
}

// Adapted from Bosh BMP280 datasheet
static int calibPress24_8(const struct device *dev, uint32_t *pressure)
{
	struct bmp280_data *data = dev->data;
	if (!data->calib.p_cooef_cmpt)
	{
		int64_t v1 = ((int64_t)data->calib.t) - 128000;
		int64_t v2 = v1 * v1 * (int64_t)data->calib.pCalib.dP6;
		v2 = v2 + ((v1 * (int64_t)data->calib.pCalib.dP5) << 17);
		v2 = v2 + (((int64_t)data->calib.pCalib.dP4) << 35);
		v1 = ((v1 * v1 * (int64_t)data->calib.pCalib.dP3) >> 8) +
			 ((v1 * (int64_t)data->calib.pCalib.dP2) << 12);
		v1 = (((((int64_t)1) << 47) + v1)) * ((int64_t)data->calib.pCalib.dP1) >> 33;
		if (v1 == 0)
			return -EINVAL; // division by 0
		int64_t tempP = data->press_raw[0] << 12 | data->press_raw[1] << 4 | data->press_raw[2] >> 4;
		tempP = 1048576 - tempP;
		tempP = (((tempP << 31) - v2) * 3125) / v1;
		v1 = (((int64_t)data->calib.pCalib.dP9) * (tempP >> 13) * (tempP >> 13)) >> 25;
		v2 = (((int64_t)data->calib.pCalib.dP8) * tempP) >> 19;
		data->calib.p = ((tempP + v1 + v2) >> 8) + (((int64_t)data->calib.pCalib.dP7) << 4);
		data->calib.p_cooef_cmpt = 1;
		*pressure = data->calib.p;
	}
	else
	{
		*pressure = data->calib.p;
	}
	return 0;
}

static int calibPress(const struct device *dev, struct sensor_value *pressure)
{
	uint32_t tempP = 0;
	calibPress24_8(dev, &tempP);
	tempP *= 1000;		// mPa
	tempP = tempP >> 8; // conversion from 24.8 code to integer mPa
	pressure->val1 = (int32_t)tempP / 1000000;
	pressure->val2 = (int32_t)tempP % 1000000;
	return 0;
}

static int32_t ln(uint32_t x)
{
	int32_t result = 0;
	int32_t n = (find_msb_set(x) - 1) - 8;
	result = n * BMP280_LN2_Q24_8;
	if (n >= 0)
		x = x >> n;
	else
		x = x << (-n);
	x -= 256;

	result += (int32_t)((((int64_t)BMP280_LN2_COOEF_1_Q24_8 * ((x * x) >> 8)) + ((int64_t)BMP280_LN2_COOEF_2_Q24_8 * x)) >> 8);
	return result;
}

static int computeAltitude(const struct device *dev, struct sensor_value *alt)
{
	struct bmp280_data *data = dev->data;
	uint32_t press;
	uint32_t temp;
	int ret = calibPress24_8(dev, &press);
	if (ret)
		return ret;
	if (!data->calib.t_cooef_cmpt)
	{
		struct sensor_value dummy;
		calibTemp(dev, &dummy);
	}
	temp = ((data->calib.t / 20) + BMP280_CELSSIUS_TO_KELVIN_Q24_8);
	int32_t h = (((int64_t)temp * (ln(data->pressSL) - ln(press)) + 128) >> 8) * BMP280_ALT_CONST_Q24_8 >> 8;
	alt->val1 = h / 256;
	alt->val2 = (int32_t)((int64_t)((h % 256) * 1000000) / 256);
	return 0;
}

static bool bmp280_isImReady(const struct device *dev)
{
	const struct bmp280_config *config = dev->config;
	uint8_t status = 0;
	config->ioAPI.methods.readByte(dev, BMP280_ADDR_STATUS, &status);
	return !(status & BMP280_MASK_STATUS_IM_UPADTE);
}

static bool bmp280_isMeasuring(const struct device *dev)
{
	const struct bmp280_config *config = dev->config;
	uint8_t status = 0;
	config->ioAPI.methods.readByte(dev, BMP280_ADDR_STATUS, &status);
	return (status & BMP280_MASK_STATUS_MEASURING) >> 3;
}
static bool bmp280_chipID_OK(const struct device *dev)
{
	uint8_t id;
	const struct bmp280_config *conf = dev->config;
	int c = conf->ioAPI.methods.readByte(dev, BMP280_ADDR_ID, &id);
	if (id != BMP280_SENSOR_ID || c)
	{
		return 0;
	}
	return 1;
}
static int bmp280_softResetToDefault(const struct device *dev)
{
	const struct bmp280_config *conf = dev->config;

	if (!conf->ioAPI.methods.isBusReady(dev))
	{
		return -ENODEV;
	}
	int c = conf->ioAPI.methods.writeByte(dev, BMP280_ADDR_RESET, BMP280_RESET_VAL);
	if (c)
		return c;
	return bmp280_init_full(dev);
}

static int bmp280_softResetToSleep(const struct device *dev)
{
	const struct bmp280_config *conf = dev->config;

	if (!conf->ioAPI.methods.isBusReady(dev))
	{
		return -ENODEV;
	}
	int c = conf->ioAPI.methods.writeByte(dev, BMP280_ADDR_RESET, BMP280_RESET_VAL);
	if (c)
		return c;
	return bmp280_init_bare(dev);
}

static uint16_t bmp280_timeToRead_ms(const struct device *dev)
{
	struct bmp280_data *data = dev->data;
	uint16_t t = 0;
	switch (BMP280_CTRL_MEAS_GET_OSRS_T(data->ctrl_meas))
	{
	case BMP280_OVERSAMPLING_TEMPERATURE_X_0:
		t += 0;
		break;
	case BMP280_OVERSAMPLING_TEMPERATURE_X_1:
		t += 2;
		break;
	case BMP280_OVERSAMPLING_TEMPERATURE_X_2:
		t += 4;
		break;
	case BMP280_OVERSAMPLING_TEMPERATURE_X_4:
		t += 8;
		break;
	case BMP280_OVERSAMPLING_TEMPERATURE_X_8:
		t += 16;
		break;
	case BMP280_OVERSAMPLING_TEMPERATURE_X_16:
		t += 32;
		break;
	default:
		break;
	}
	switch (BMP280_CTRL_MEAS_GET_OSRS_P(data->ctrl_meas))
	{
	case BMP280_OVERSAMPLING_PRESSURE_X_0:
		t += 0;
		break;
	case BMP280_OVERSAMPLING_PRESSURE_X_1:
		t += 2;
		break;
	case BMP280_OVERSAMPLING_PRESSURE_X_2:
		t += 4;
		break;
	case BMP280_OVERSAMPLING_PRESSURE_X_4:
		t += 8;
		break;
	case BMP280_OVERSAMPLING_PRESSURE_X_8:
		t += 16;
		break;
	case BMP280_OVERSAMPLING_PRESSURE_X_16:
		t += 32;
		break;
	default:
		break;
	}
	t += 2; // measurment starting time
	return t;
}

static inline bool sensor_value_equal(const struct sensor_value *v1, const struct sensor_value *v2)
{
	return (v1->val1 == v2->val1) && (v1->val2 == v2->val2);
}
static inline bool int_sensor_value_equal(const struct sensor_value *v1, const int val1, const int val2)
{
	return (v1->val1 == val1) && (v1->val2 == val2);
}

static inline int bmp280_attrValToReg(const struct bmp280_param_reg_elem *map, size_t s, const struct sensor_value *attr, uint8_t *reg)
{
	for (size_t i = 0; i < s; i++)
	{
		if (sensor_value_equal(&map[i].val, attr))
		{
			*reg = map[i].reg;
			return 0;
		}
	}

	return -EINVAL;
}

static inline int bmp280_regToAttrVal(const struct bmp280_param_reg_elem *map, size_t s, struct sensor_value *attr, const uint8_t reg)
{
	for (size_t i = 0; i < s; i++)
	{
		if (map[i].reg == reg)
		{
			*attr = map[i].val;
			return 0;
		}
	}

	return -EINVAL;
}
/*
 *	 bmp280 IO API I2C
 */
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
static int bmp280_i2c_readBurst(const struct device *dev, uint8_t addr, uint8_t *buffer, size_t size)
{
	const struct bmp280_config *conf = dev->config;

	return i2c_burst_read_dt(&(conf->ioAPI.bus.i2c), addr, buffer, size);
}

static int bmp280_i2c_writeBurst(const struct device *dev, uint8_t addr, const uint8_t *buffer, size_t size)
{
	const struct bmp280_config *conf = dev->config;
	return i2c_burst_write_dt(&(conf->ioAPI.bus.i2c), addr, buffer, size);
}

static int bmp280_i2c_readByte(const struct device *dev, uint8_t addr, uint8_t *buffer)
{
	const struct bmp280_config *conf = dev->config;

	return i2c_reg_read_byte_dt(&(conf->ioAPI.bus.i2c), addr, buffer);
}

static int bmp280_i2c_writeByte(const struct device *dev, uint8_t addr, uint8_t buffer)
{
	const struct bmp280_config *conf = dev->config;
	return i2c_reg_write_byte_dt(&(conf->ioAPI.bus.i2c), addr, buffer);
}

static inline bool bmp280_i2c_isBusReady(const struct device *dev)
{
	return device_is_ready(((struct bmp280_config *)dev->config)->ioAPI.bus.i2c.bus);
}
static inline bool bmp280_i2c_busType(void) // 1 for I2C, 0 for SPI
{
	return 1;
}

const struct bmp280_ioMethods bmp280_i2c_ioMethods_set = {
	.readBurst = bmp280_i2c_readBurst,
	.writeBurst = bmp280_i2c_writeBurst,
	.readByte = bmp280_i2c_readByte,
	.writeByte = bmp280_i2c_writeByte,
	.isBusReady = bmp280_i2c_isBusReady,
	.busType = bmp280_i2c_busType,
};
#endif

/*
 *	 bmp280 IO API SPI
 */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

static int bmp280_spi_readBurst(const struct device *dev, uint8_t addr, uint8_t *buffer, size_t size)
{
	const struct bmp280_config *conf = dev->config;

	return -ENOTSUP
}

static int bmp280_spi_writeBurst(const struct device *dev, uint8_t addr, const uint8_t *buffer, size_t size)
{
	const struct bmp280_config *conf = dev->config;
	return -ENOTSUP
}

static int bmp280_spi_readByte(const struct device *dev, uint8_t addr, uint8_t *buffer)
{
	const struct bmp280_config *conf = dev->config;
	return -ENOTSUP
}

static int bmp280_spi_writeByte(const struct device *dev, uint8_t addr, uint8_t buffer)
{
	const struct bmp280_config *conf = dev->config;
	return -ENOTSUP
}

static inline bool bmp280_spic_isBusReady(const struct device *dev)
{
	return -ENOTSUP
}
static inline bool bmp280_spi_busType(void) // 1 for I2C, 0 for SPI
{
	return 0;
}

const struct bmp280_ioMethods bmp280_i2c_ioMethods_set = {
	.readBurst = bmp280_spi_readBurst,
	.writeBurst = bmp280_spi_writeBurst,
	.readByte = bmp280_spi_readByte,
	.writeByte = bmp280_spi_writeByte,
	.isBusReady = bmp280_spi_isBusReady,
	.busType = bmp280_spi_busType,
};
#endif

/**
 *	@name Zephyr sensor API integration
 * 	@{
 */

/**
 * @brief sensor API methods
 *
 */
const struct sensor_driver_api bmp280_api = {
	.attr_set = bmp280_attr_set,
	.attr_get = bmp280_attr_get,
	.sample_fetch = bmp280_sample_fetch,
	.channel_get = bmp280_channel_get};

/* clang-format off */
#define BMP280_DEF(inst)                                                                             \
	static struct bmp280_data bmp280_data_##inst = {                                                 \
		/* initialize RAM values as needed, e.g.: */                                                 \
	};                                                                                               \
	static const struct bmp280_config bmp280_config_##inst = {                                       \
		.ioAPI = {                                                                                   \
			.bus = COND_CODE_1(DT_INST_ON_BUS(inst, i2c),                                            \
							   (BMP280_ON_I2C_DEF(inst)),                                            \
							   (BMP280_ON_SPI_DEF(inst))),                                           \
                                                                                                     \
			.methods = COND_CODE_1(DT_INST_ON_BUS(inst, i2c),                                        \
								   (bmp280_i2c_ioMethods_set),                                       \
								   (bmp280_spi_ioMethods_set)),                                      \
		},                                                                                           \
		.DTparams = {                                                                                \
			.mode = DT_INST_PROP_OR(inst, mode, BMP280_DEFAULT_MODE_IDX),                            \
			.t_stby = DT_INST_PROP_OR(inst, t_standby, BMP280_DEFAULT_T_STBY_IDX),                   \
			.ovrsmplT = DT_INST_PROP_OR(inst, temperature_oversampling, BMP280_DEFAULT_OVRSMPL_IDX), \
			.ovrsmplP = DT_INST_PROP_OR(inst, pressure_oversampling, BMP280_DEFAULT_OVRSMPL_IDX),    \
			.iir_filter = DT_INST_PROP_OR(inst, iir_filter, BMP280_DEFAULT_FILTER_IDX),              \
			IF_ENABLED(DT_INST_ON_BUS(inst,spi),(													 \
				.spi3wire = DT_INST_PROP_OR(inst, spi_3_wire_enable, BMP280_DEFAULT_SPI_3_WIRE_IDX),)\
			)     																					 \
		},                                                                                           \
	};                                                                                               \
	SENSOR_DEVICE_DT_INST_DEFINE(inst,                                                               \
								 bmp280_init_full,                                                   \
								 NULL,                                                               \
								 &bmp280_data_##inst,                                                \
								 &bmp280_config_##inst,                                              \
								 POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,                           \
								 &bmp280_api);

DT_INST_FOREACH_STATUS_OKAY(BMP280_DEF)
/* clang-format on */

/**
 * @}
 */