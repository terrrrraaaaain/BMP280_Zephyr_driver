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
 *   @file
 *   @brief Memory address map, default values and configs
 *   for BMP280 sensor according to datasheet
 */

#define DT_DRV_COMPAT custom_bmp280

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include "bmp280_priv.h"
/*
chanels
	SENSOR_CHAN_AMBIENT_TEMP
	SENSOR_CHAN_PRESS
*/

/*
atributes
	SENSOR_ATTR_SAMPLING_FREQUENCY,
*/

/*
 *	Definitions
 */

struct bmp280_calibTemperatureData
{
	uint16_t dT1;
	int16_t dT2;
	int16_t dT3;
};

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

struct bmp280_calibData
{
	struct bmp280_calibTemperatureData tCalib;
	struct bmp280_calibPressureData pCalib;
	int32_t t; // for pressure compensation from temperatur compensation
	bool t_cooef_cmpt;
};

struct bmp280_ioMethods
{
	bool (*isBusReady)(const struct device *dev);
	int (*readByte)(const struct device *dev, uint8_t addr, uint8_t *buffer);
	int (*readBurst)(const struct device *dev, uint8_t addr, uint8_t *buffer, size_t size);
	int (*writeByte)(const struct device *dev, uint8_t addr, uint8_t buffer);
	int (*writeBurst)(const struct device *dev, uint8_t addr, const uint8_t *buffer, size_t size);
};

struct bmp280_ioAPI
{
	struct bmp280_ioMethods methods;
	union bmp280_bus
	{
		struct i2c_dt_spec i2c;
		struct spi_dt_spec spi;
	} bus;
};

struct bmp280_config
{
	struct i2c_dt_spec i2c_bus;
	struct bmp280_ioAPI ioAPI;
};

struct bmp280_data
{
	uint8_t press_raw[3];
	uint8_t temp_raw[3];
	uint8_t ctrl_meas;
	struct bmp280_calibData calib;
};

/*
 *	Helper function definitions
 */

// status getters
static bool bmp280_isImReady(const struct device *dev);
static bool bmp280_isMeasuring(const struct device *dev);
static bool bmp280_chipID_OK(const struct device *dev);
// get raw data from sensor
static int bmp280_getPressureRaw(const struct device *dev);
static int bmp280_getTemperatureRaw(const struct device *dev);

// getters and setters of ctrl_meas
static int bmp280_getCtrlMeas(const struct device *dev);
static int bmp280_setCtrlMeas(const struct device *dev);

// Compensation and conversion function adapted from Bosh BMP280 datasheet
static int calibTemp(const struct device *dev, struct sensor_value *temperature);
static int calibPress(const struct device *dev, struct sensor_value *pressure);

// Waiting time estimation in force mode
static uint16_t bmp280_timeToRead_ms(const struct device *dev);

// Software reset
static int bmp280_softReset(const struct device *dev);

/*
 *	Implementations of main API functions
 */

static int bmp280_init(const struct device *dev)
{
	printk("Initializing BMP280_Ft\n");
	const struct bmp280_config *conf = dev->config;
	struct bmp280_data *data = dev->data;

	if (!conf->ioAPI.methods.isBusReady(dev))
	{
		printk("bus err\n");
		return -ENODEV;
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
	printk("waited for boot %dm sec\n", c);

	uint8_t calBuf[26];
	c = conf->ioAPI.methods.readBurst(dev, BMP280_ADDR_CALIB00, calBuf, 26);
	printk("read Calib %d\n", c);

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

	printk("%d\n", data->calib.tCalib.dT1);
	printk("%d\n", data->calib.tCalib.dT2);
	printk("%d\n", data->calib.tCalib.dT3);
	printk("%d\n", data->calib.pCalib.dP1);
	printk("%d\n", data->calib.pCalib.dP2);
	printk("%d\n", data->calib.pCalib.dP3);
	printk("%d\n", data->calib.pCalib.dP4);
	printk("%d\n", data->calib.pCalib.dP5);
	printk("%d\n", data->calib.pCalib.dP6);
	printk("%d\n", data->calib.pCalib.dP7);
	printk("%d\n", data->calib.pCalib.dP8);
	printk("%d\n", data->calib.pCalib.dP9);

	data->calib.t_cooef_cmpt = 0; // reset temp calibration info readiness flag for pressure calibration

	data->ctrl_meas = BMP280_CTRL_MEAS_SET_MODE(data->ctrl_meas, BMP280_MODE_FORCED);
	data->ctrl_meas = BMP280_CTRL_MEAS_SET_OSRS_T(data->ctrl_meas, BMP280_OVERSAMPLING_TEMPERATURE_X_16);
	data->ctrl_meas = BMP280_CTRL_MEAS_SET_OSRS_P(data->ctrl_meas, BMP280_OVERSAMPLING_PRESSURE_X_16);

	c = bmp280_setCtrlMeas(dev);
	c = bmp280_getCtrlMeas(dev);

	printk("CTRL(%d) %d\n", c, (data->ctrl_meas));
	return 0;
}

static int bmp280_sample_fetch(const struct device *dev, enum sensor_channel chanel)
{
	struct bmp280_data *data = dev->data;
	if (!bmp280_chipID_OK(dev))
	{
		return -ENXIO;
	}
	if (BMP280_CTRL_MEAS_GET_MODE(data->ctrl_meas) == BMP280_MODE_FORCED || BMP280_CTRL_MEAS_GET_MODE(data->ctrl_meas) == BMP280_MODE_SLEEP)
	{

		data->ctrl_meas = BMP280_CTRL_MEAS_SET_MODE(data->ctrl_meas, BMP280_MODE_FORCED); // assuming that the aim of caling fetch in sleep mode is to triger a single measure
		bmp280_setCtrlMeas(dev);														  // start measuring in forced mode (if in sleep wake up and force measurement)

		k_msleep(bmp280_timeToRead_ms(dev)); // wait for result based on configuration
		printk("wait %dms\n", bmp280_timeToRead_ms(dev));
		uint8_t cycles = 0;
		while (bmp280_isMeasuring(dev) || !bmp280_isImReady(dev))
		{
			if (cycles >= 500)
				return -ETIMEDOUT;
			k_usleep(100);
		}
	}
	int c = bmp280_getCtrlMeas(dev);
	printk("CTRL(%d) %d\n", c, (((struct bmp280_data *)dev->data)->ctrl_meas));
	data->calib.t_cooef_cmpt = 0;
	switch (chanel)
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

static int bmp280_chanel_get(const struct device *dev, enum sensor_channel chanel, struct sensor_value *reading)
{
	switch (chanel)
	{
	case SENSOR_CHAN_AMBIENT_TEMP:
		printk("temp raw = %d\n", ((struct bmp280_data *)(dev->data))->temp_raw[0]);
		calibTemp(dev, reading);
		break;
	case SENSOR_CHAN_PRESS:
		if (!((struct bmp280_data *)(dev->data))->calib.t_cooef_cmpt)
		{
			struct sensor_value dummy;
			calibTemp(dev, &dummy);
		}
		calibPress(dev, reading);

		break;
	default:
		return -ENOTSUP;
	}
	return 0;
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

static int bmp280_getPressureRaw(const struct device *dev)
{

	const struct bmp280_config *conf = dev->config;
	struct bmp280_data *data = dev->data;

	int c = conf->ioAPI.methods.readBurst(dev, BMP280_ADDR_PRESS_MSB, data->press_raw, 3);
	printk("P_c = %d\n", c);

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
	printk("T_c = %d\n", c);
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
		printk("temp calib full\n");
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
	printk("temp calib min\n");

	uint32_t tempT = (data->calib.t * 5 + 128) >> 8;
	temperature->val1 = tempT / 100;
	temperature->val2 = (tempT) % 100 * 1000;

	return 0;
}

// Adapted from Bosh BMP280 datasheet
static int calibPress(const struct device *dev, struct sensor_value *pressure)
{
	printk("press calib\n");

	struct bmp280_data *data = dev->data;
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
	tempP = ((tempP + v1 + v2) >> 8) + (((int64_t)data->calib.pCalib.dP7) << 4);
	tempP *= 1000;		// mPa
	tempP = tempP >> 8; // conversion from 24.8 code to integer mPa
	pressure->val1 = (int32_t)tempP / 1000000;
	pressure->val2 = (int32_t)tempP % 1000000;
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
static int bmp280_softReset(const struct device *dev)
{
	const struct bmp280_config *conf = dev->config;

	if (!conf->ioAPI.methods.isBusReady(dev))
	{
		printk("I2C err\n");
		return -ENODEV;
	}
	int c = conf->ioAPI.methods.writeByte(dev, BMP280_ADDR_RESET, BMP280_RESET);
	if (c)
		return c;
	return bmp280_init(dev);
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

const struct bmp280_ioMethods bmp280_i2c_ioMethods_set = {
	.readBurst = bmp280_i2c_readBurst,
	.writeBurst = bmp280_i2c_writeBurst,
	.readByte = bmp280_i2c_readByte,
	.writeByte = bmp280_i2c_writeByte,
	.isBusReady = bmp280_i2c_isBusReady,
};
#endif

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

const struct bmp280_ioMethods bmp280_i2c_ioMethods_set = {
	.readBurst = bmp280_spi_readBurst,
	.writeBurst = bmp280_spi_writeBurst,
	.readByte = bmp280_spi_readByte,
	.writeByte = bmp280_spi_writeByte,
	.isBusReady = bmp280_spi_isBusReady,
};
#endif

/*
 *	Zephyr sensor API integration
 */

const struct sensor_driver_api bmp280_api = {
	.attr_set = NULL,
	.attr_get = NULL,
	.trigger_set = NULL,
	.sample_fetch = bmp280_sample_fetch,
	.channel_get = bmp280_chanel_get,
	.get_decoder = NULL,
	.submit = NULL,
};
#define BMP280_ON_I2C_DEF(inst) {.i2c = I2C_DT_SPEC_INST_GET(inst)}
#define BMP280_ON_SPI_DEF(inst) {.spi = SPI_DT_SPEC_INST_GET(inst, 0, 0)}

#define BMP280_DEF(inst)                                                   \
	static struct bmp280_data bmp280_data_##inst = {                       \
		/* initialize RAM values as needed, e.g.: */                       \
	};                                                                     \
	static const struct bmp280_config bmp280_config_##inst = {             \
		.ioAPI = {                                                         \
			.bus = COND_CODE_1(DT_INST_ON_BUS(inst, i2c),                  \
							   (BMP280_ON_I2C_DEF(inst)),                  \
							   (BMP280_ON_SPI_DEF(inst))),                 \
                                                                           \
			.methods = COND_CODE_1(DT_INST_ON_BUS(inst, i2c),              \
								   (bmp280_i2c_ioMethods_set),             \
								   (bmp280_spi_ioMethods_set)),            \
		},                                                                 \
	};                                                                     \
	SENSOR_DEVICE_DT_INST_DEFINE(inst,                                     \
								 bmp280_init,                              \
								 NULL,                                     \
								 &bmp280_data_##inst,                      \
								 &bmp280_config_##inst,                    \
								 POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
								 &bmp280_api);

DT_INST_FOREACH_STATUS_OKAY(BMP280_DEF)