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

struct bmp280_config
{
	struct i2c_dt_spec i2c_bus;
};

struct bmp280_data
{
	uint8_t press_raw[3];
	uint8_t temp_raw[3];
	uint8_t ctrl_meas;
	struct bmp280_calibData calib;
};

bool bmp280_imReady(const struct device *dev);

/*
 *	Definitions
 */

static int bmp280_init(const struct device *dev)
{
	printk("Initializing BMP280_Ft\n");
	const struct bmp280_config *conf = dev->config;
	struct bmp280_data *data = dev->data;

	if (!device_is_ready(conf->i2c_bus.bus))
	{
		printk("I2C err\n");
		return -ENODEV;
	}

	uint8_t id;
	int c = i2c_reg_read_byte_dt(&(conf->i2c_bus), BMP280_ADDR_ID, &id);
	printk("chip ID = 0x%x\n", id);

	if (id != BMP280_SENSOR_ID)
	{
		return -ENXIO;
	}
	c = 0;
	while (!bmp280_imReady(dev))
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
	c = i2c_burst_read_dt(&(conf->i2c_bus), BMP280_ADDR_CALIB00, calBuf, 26);
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

	data->ctrl_meas = BMP280_OVERSAMPLING_TEMPERATURE_X_16 | BMP280_OVERSAMPLING_PRESSURE_X_16 | BMP280_MODE_NORMAL;
	c = bmp280_setCtrlMeas(dev);
	c = bmp280_getCtrlMeas(dev);
	printk("CTRL(%d) %d\n", c, (data->ctrl_meas));
	return 0;
}

static int bmp280_getPressureRaw(const struct device *dev)
{
	const struct bmp280_config *conf = dev->config;
	struct bmp280_data *data = dev->data;

	int c = i2c_burst_read_dt(&(conf->i2c_bus), BMP280_ADDR_PRESS_MSB, data->press_raw, 3);
	printk("P_c = %d", c);

	if (c == 0)
	{
		if (data->press_raw[0] == BMP280_NO_VALUE_1)
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

	int c = i2c_burst_read_dt(&(conf->i2c_bus), BMP280_ADDR_TEMP_MSB, data->temp_raw, 3);
	printk("T_c = %d", c);
	if (c == 0)
	{
		if (data->temp_raw[0] == BMP280_NO_VALUE_1)
			return -ENODATA;
		else
			return 0;
	}
	return c;
}

static int bmp280_sample_fetch(const struct device *dev, enum sensor_channel chanel)
{
	int c = bmp280_getCtrlMeas(dev);
	printk("CTRL(%d) %d\n", c, (((struct bmp280_data *)dev->data)->ctrl_meas));
	((struct bmp280_data *)dev->data)->calib.t_cooef_cmpt = 0;
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

// Adapted from Bosh BMP280 datasheet
static uint8_t calibTemp(const struct device *dev, struct sensor_value *temperature)
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
static uint8_t calibPress(const struct device *dev, struct sensor_value *pressure)
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

int bmp280_setCtrlMeas(const struct device *dev)
{
	const struct bmp280_config *conf = dev->config;
	const struct bmp280_data *data = dev->data;
	return i2c_reg_write_byte_dt(&(conf->i2c_bus), BMP280_ADDR_CTRL_MEAS, data->ctrl_meas);
}

int bmp280_getCtrlMeas(const struct device *dev)
{
	const struct bmp280_config *conf = dev->config;
	struct bmp280_data *data = dev->data;
	return i2c_reg_read_byte_dt(&(conf->i2c_bus), BMP280_ADDR_CTRL_MEAS, &data->ctrl_meas);
}


bool bmp280_imReady(const struct device *dev)
{
	const struct bmp280_config *config = dev->config;
	uint8_t status = 0;
	i2c_reg_read_byte_dt(&(config->i2c_bus), BMP280_ADDR_STATUS, &status);
	return !(status & 0b00000001);
}



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

#define BMP280_DEF(inst)                                                   \
	static struct bmp280_data bmp280_data_##inst = {                       \
		/* initialize RAM values as needed, e.g.: */                       \
	};                                                                     \
	static const struct bmp280_config bmp280_config_##inst = {             \
		.i2c_bus = I2C_DT_SPEC_INST_GET(inst),                             \
	};                                                                     \
	SENSOR_DEVICE_DT_INST_DEFINE(inst,                                     \
								 bmp280_init,                              \
								 NULL,                                     \
								 &bmp280_data_##inst,                      \
								 &bmp280_config_##inst,                    \
								 POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
								 &bmp280_api);

DT_INST_FOREACH_STATUS_OKAY(BMP280_DEF)