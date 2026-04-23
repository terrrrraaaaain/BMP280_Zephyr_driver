/*
 * Copyright (c) 2026 Franciszek Trzeciak
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BMP280 Zephyr driver demo
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/custom_bmp280.h> // see for more settings

#define SETTINGS // show config, then change and show again
// #define MEASURE // measure loop

// #define ALL // all above

#ifdef ALL
#define SETTINGS
#define MEASURE
#endif

void readAndPrint(const struct device *dev);
void dumpConfiguration(const struct device *dev);
void setConfiguration(const struct device *dev);

int main(void)
{
    const struct device *bmp = DEVICE_DT_GET_ANY(custom_bmp280);
    if (bmp == NULL)
        printf("ERROR: compatible device found\n");
    if (!device_is_ready(bmp))
        printf("ERROR: Device %s not ready\n", bmp->name);
    printk("\n");
#ifdef SETTINGS
    printk("\n--DT configuration (see app.overlay)--\n");
    dumpConfiguration(bmp);

    printk("\n------Setting new configuration-------\n");
    setConfiguration(bmp);

    printk("\n----------New configuration-----------\n");
    dumpConfiguration(bmp);
#endif
#ifndef MEASURE
    printk("\n----------------Done------------------\n");

    while (1)
        ;
#endif
#ifdef MEASURE

    while (1)
    {
        // read Temperature and Pressure
        readAndPrint(bmp);
        k_msleep(1000);
    }
#endif
}

void readAndPrint(const struct device *dev)
{
    struct sensor_value temp;
    struct sensor_value press;

    int64_t start_time, stop_time;

    start_time = k_uptime_get();
    // retreive data from sensor
    int8_t c = sensor_sample_fetch(dev);
    stop_time = k_uptime_get();
    // convert raw data
    if (c == 0)
    {
        int8_t c3 = sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
        int8_t c2 = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);

        // show results
        printk("Fetch time: %lldms\n", stop_time - start_time);
        printk("RETURN CODES\nfetch (%d), get (%d_%d)\n", c, c2, c3);
        printk("Temp\t%d.%06d  *C\n", temp.val1, temp.val2);
        printk("Press\t%d.%06d kPa\n", press.val1, press.val2);
        printk("---------------------------\n");
    }
    else
    {
        printk("Fetch error code %d\n", c);
        printk("---------------------------\n");
    }
}

struct bmp280_name_map
{
    struct sensor_value val;
    const char *name;
};

static const struct bmp280_name_map standby_names[] = {
    {BMP280_T_STANDBY_0_5MS, "0.5ms"},
    {BMP280_T_STANDBY_62_5MS, "62.5ms"},
    {BMP280_T_STANDBY_125MS, "125ms"},
    {BMP280_T_STANDBY_250MS, "250ms"},
    {BMP280_T_STANDBY_500MS, "500ms"},
    {BMP280_T_STANDBY_1000MS, "1000ms"},
    {BMP280_T_STANDBY_2000MS, "2000ms"},
    {BMP280_T_STANDBY_4000MS, "4000ms"}};

static const struct bmp280_name_map filter_names[] = {
    {BMP280_FILTER_OFF, "OFF"},
    {BMP280_FILTER_X2, "X2"},
    {BMP280_FILTER_X4, "X4"},
    {BMP280_FILTER_X8, "X8"},
    {BMP280_FILTER_X16, "X16"}};

static const struct bmp280_name_map oversampling_names[] = {
    {BMP280_OVERSAMPLING_OFF, "OFF"},
    {BMP280_OVERSAMPLING_X1, "X1"},
    {BMP280_OVERSAMPLING_X2, "X2"},
    {BMP280_OVERSAMPLING_X4, "X4"},
    {BMP280_OVERSAMPLING_X8, "X8"},
    {BMP280_OVERSAMPLING_X16, "X16"}};

static const struct bmp280_name_map mode_names[] = {
    {BMP280_MODE_SLEEP, "SLEEP"},
    {BMP280_MODE_FORCED, "FORCED"},
    {BMP280_MODE_NORMAL, "NORMAL"}};

const char *bmp280_val_to_str(const struct bmp280_name_map *map, size_t size, struct sensor_value target)
{
    for (size_t i = 0; i < size; i++)
    {
        if (map[i].val.val1 == target.val1 && map[i].val.val2 == target.val2)
        {
            return map[i].name;
        }
    }
    return "UNKNOWN";
}

#define CFG_TO_STR(map_array, target_val) \
    bmp280_val_to_str(map_array, ARRAY_SIZE(map_array), target_val)

void dumpConfiguration(const struct device *dev)
{
    struct sensor_value config;

    int c2 = sensor_attr_get(dev, SENSOR_CHAN_ALL, BMP280_ATTR_MODE, &config);
    printk("MODE(%d)\t\t%s\n", c2, CFG_TO_STR(mode_names, config));

    c2 = sensor_attr_get(dev, SENSOR_CHAN_ALL, BMP280_ATTR_T_STANDBY, &config);
    printk("STANDBY TIME(%d)\t%s\n", c2, CFG_TO_STR(standby_names, config));

    c2 = sensor_attr_get(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_OVERSAMPLING, &config);
    printk("OVRSMPL T(%d)\t%s\n", c2, CFG_TO_STR(oversampling_names, config));

    c2 = sensor_attr_get(dev, SENSOR_CHAN_PRESS, SENSOR_ATTR_OVERSAMPLING, &config);
    printk("OVRSMPL P(%d)\t%s\n", c2, CFG_TO_STR(oversampling_names, config));

    c2 = sensor_attr_get(dev, SENSOR_CHAN_PRESS, BMP280_ATTR_FILTER, &config);
    printk("FILTER(%d)\t%s\n", c2, CFG_TO_STR(filter_names, config));
}

void setConfiguration(const struct device *dev)
{

    int c2 = sensor_attr_set(dev, SENSOR_CHAN_ALL, BMP280_ATTR_RESET, &BMP280_RESET_TO_SLEEP); // force sleep mode to set new mode and Oversaampling.

    c2 = sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_OVERSAMPLING, &BMP280_OVERSAMPLING_X4);
    printk("OVRSMPL T SET CODE(%d)\n", c2);

    c2 = sensor_attr_set(dev, SENSOR_CHAN_PRESS, SENSOR_ATTR_OVERSAMPLING, &BMP280_OVERSAMPLING_X4);
    printk("OVRSMPL P SET CODE(%d)\n", c2);

    c2 = sensor_attr_set(dev, SENSOR_CHAN_PRESS, BMP280_ATTR_FILTER, &BMP280_FILTER_X2);
    printk("FILTER SET CODE(%d)\n", c2);

    c2 = sensor_attr_set(dev, SENSOR_CHAN_ALL, BMP280_ATTR_T_STANDBY, &BMP280_T_STANDBY_2000MS);
    printk("STANDBY TIME SET CODE(%d)\n", c2);

    c2 = sensor_attr_set(dev, SENSOR_CHAN_ALL, BMP280_ATTR_MODE, &BMP280_MODE_NORMAL);
    printk("MODE SET CODE(%d)\t\n", c2);
}